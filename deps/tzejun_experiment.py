"""Evaluate and view tzejun's actual trained DIP-E026 policy.

This runner intentionally does not import Stable-Baselines3 or PyTorch. It
executes the actor weights exported from the repository's exact checkpoint,
and validates those weights against the repository's exported test vectors
before starting MuJoCo. The original ``best_model.zip`` is retained beside
the export for provenance and hash checking.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import platform
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from motor_model import action_to_pwm, action_to_torque


ROOT = Path(__file__).resolve().parent
XML_PATH = ROOT / "real_robot.xml"
WEIGHTS_PATH = ROOT / "firmware" / "policy_weights.h"
TESTVECTORS_PATH = ROOT / "firmware" / "policy_testvectors.h"
CHECKPOINT_PATH = ROOT / "models" / "best_real" / "best_model.zip"
OUT_DIR = ROOT / "experiment_tzejun"
MPL_CONFIG_DIR = ROOT / ".matplotlib"
MPL_CONFIG_DIR.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))

SOURCE_COMMIT = "45403944d29ebe7142404ab84a2077b4cb54e49a"
CHECKPOINT_SHA256 = "A850B7CD7B0FBB9BDEC828439D4BEC6EFFA589099ACB2405ACDC6FA4F879511D"
WEIGHTS_SHA256 = "019C8390344F2EA8FF7F7AAB51276D777CE118CA38C71F55C813DE94C16131B6"

FRAME_SKIP = 5
CONTROL_DT = 0.005
MAX_STEPS = 2000
FALL_ANGLE = 0.70
SETTLE_STEPS = 10
MAX_V_FORWARD = 0.3
MAX_V_TURN = 0.5
ARMATURE_NOMINAL = 2.25e-3
ACTION_RATE_WEIGHT = 0.2
POSITION_TAU_S = 2.0
POSITION_WEIGHT = 2.5
POSITION_OBS_SCALE = 10.0
YAW_TAU_S = 30.0
YAW_WEIGHT = 3.0
YAW_CLIP_RAD = 0.2
YAW_OBS_SCALE = 5.0
TURN_LPF = 0.9
TURN_WEIGHT = 2.0
TURN_INSTANT_WEIGHT = 0.2
SHAPING_GAMMA = 0.99
HISTORY_TAPS = (2, 5, 11, 23, 47)
HISTORY_LEN = 48

EVAL_COMMANDS = (
    (0.0, 0.0),
    (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
    (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0),
    (0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5),
    (0.5, 0.5), (-0.5, 0.5), (0.5, -0.5), (-0.5, -0.5),
    (0.75, 0.25), (-0.75, -0.25), (0.25, 0.75),
)
EVAL_PAYLOADS = (
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.25, 0.25, 0.25, 0.25,
    0.5, 0.5, 0.5, 0.5,
    0.75, 0.75, 0.75,
    1.0, 1.0, 1.0, 1.0,
)

CSV_FIELDS = (
    "suite", "episode", "seed", "t_s", "target_v_m_s", "target_yaw_rad_s",
    "payload_kg", "x_m", "pitch_deg", "roll_deg", "yaw_deg",
    "pitch_rate_rad_s", "yaw_rate_rad_s", "v_forward_m_s",
    "action_left", "action_right", "pwm_left", "pwm_right",
    "torque_left_nm", "torque_right_nm", "reward", "terminated",
)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def parse_c_array(path: Path, name: str) -> np.ndarray:
    text = path.read_text(encoding="utf-8")
    match = re.search(
        rf"static\s+const\s+float\s+{re.escape(name)}\s*\[\s*\d+\s*\]\s*=\s*\{{(.*?)\}}\s*;",
        text,
        flags=re.S,
    )
    if not match:
        raise ValueError(f"array {name!r} not found in {path}")
    tokens = re.findall(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", match.group(1))
    return np.asarray([float(x) for x in tokens], dtype=np.float32)


class ExportedPolicy:
    """Exact 34 -> 64 -> 64 -> 2 actor exported from best_model.zip."""

    def __init__(self) -> None:
        self.w0 = parse_c_array(WEIGHTS_PATH, "policy_w0").reshape(64, 34)
        self.b0 = parse_c_array(WEIGHTS_PATH, "policy_b0")
        self.w1 = parse_c_array(WEIGHTS_PATH, "policy_w1").reshape(64, 64)
        self.b1 = parse_c_array(WEIGHTS_PATH, "policy_b1")
        self.w2 = parse_c_array(WEIGHTS_PATH, "policy_w2").reshape(2, 64)
        self.b2 = parse_c_array(WEIGHTS_PATH, "policy_b2")

    def predict(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32)
        if x.shape != (34,):
            raise ValueError(f"expected observation shape (34,), got {x.shape}")
        h1 = np.tanh(self.w0 @ x + self.b0)
        h2 = np.tanh(self.w1 @ h1 + self.b1)
        return np.clip(self.w2 @ h2 + self.b2, -1.0, 1.0).astype(np.float32)

    def validate(self) -> dict:
        obs = parse_c_array(TESTVECTORS_PATH, "policy_test_obs").reshape(-1, 34)
        expected = parse_c_array(TESTVECTORS_PATH, "policy_test_expected").reshape(-1, 2)
        actual = np.stack([self.predict(row) for row in obs])
        maximum = float(np.max(np.abs(actual - expected)))
        if maximum > 2e-5:
            raise RuntimeError(f"exported actor failed test vectors: max error {maximum:.9g}")
        return {"vectors": int(len(obs)), "max_abs_action_error": maximum,
                "passed": True, "tolerance": 2e-5}


class CheckpointPolicy:
    """Direct Stable-Baselines3/PyTorch inference from best_model.zip."""

    def __init__(self) -> None:
        import stable_baselines3
        import torch
        from stable_baselines3 import PPO
        self.torch_version = torch.__version__
        self.sb3_version = stable_baselines3.__version__
        self.model = PPO.load(CHECKPOINT_PATH, device="cpu")
        if self.model.observation_space.shape != (34,):
            raise RuntimeError(f"checkpoint observation shape is {self.model.observation_space.shape}, expected (34,)")
        if self.model.action_space.shape != (2,):
            raise RuntimeError(f"checkpoint action shape is {self.model.action_space.shape}, expected (2,)")
        if not np.allclose(self.model.action_space.low, -1.0) or not np.allclose(self.model.action_space.high, 1.0):
            raise RuntimeError(f"checkpoint action bounds are {self.model.action_space}, expected [-1, 1]")

    def predict(self, obs: np.ndarray) -> np.ndarray:
        action, _ = self.model.predict(np.asarray(obs, dtype=np.float32), deterministic=True)
        return np.asarray(action, dtype=np.float32)

    def validate(self) -> dict:
        obs = parse_c_array(TESTVECTORS_PATH, "policy_test_obs").reshape(-1, 34)
        expected = parse_c_array(TESTVECTORS_PATH, "policy_test_expected").reshape(-1, 2)
        checkpoint_actions = np.stack([self.predict(row) for row in obs])
        exported = ExportedPolicy()
        exported_actions = np.stack([exported.predict(row) for row in obs])
        checkpoint_error = float(np.max(np.abs(checkpoint_actions - expected)))
        crosscheck_error = float(np.max(np.abs(checkpoint_actions - exported_actions)))
        if checkpoint_error > 2e-5 or crosscheck_error > 2e-5:
            raise RuntimeError(
                "checkpoint/export validation failed: "
                f"reference error={checkpoint_error:.9g}, crosscheck={crosscheck_error:.9g}"
            )
        return {
            "vectors": int(len(obs)),
            "max_abs_checkpoint_vs_reference_error": checkpoint_error,
            "max_abs_checkpoint_vs_export_error": crosscheck_error,
            "passed": True,
            "tolerance": 2e-5,
        }


def quat_from_roll_pitch(roll: float, pitch: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return np.asarray([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=np.float64)


def decode_state(data: mujoco.MjData) -> tuple[float, float, float, np.ndarray, float]:
    qw, qx, qy, qz = (float(x) for x in data.qpos[3:7])
    pitch = math.asin(float(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)))
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    rot = np.asarray([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    local_vel = rot.T @ np.asarray(data.qvel[:3], dtype=np.float64)
    return roll, pitch, yaw, local_vel, float(data.qvel[5])


class ObservationState:
    def __init__(self) -> None:
        self.hist: list[np.ndarray] | None = None
        self.pos_err: float | None = None
        self.yaw_err: float | None = None

    def reset(self) -> None:
        self.hist = None
        self.pos_err = None
        self.yaw_err = None

    def build(self, data: mujoco.MjData, target_v: float, target_yaw: float) -> np.ndarray:
        raw = np.concatenate([data.qpos, data.qvel]).astype(np.float64)
        _roll, pitch, _yaw, local_vel, actual_yaw_rate = decode_state(data)
        base = raw[2:].copy()
        base[7:10] = local_vel
        base = np.concatenate([base, [target_v, target_yaw]])
        if self.pos_err is None:
            self.pos_err = 0.0
        else:
            self.pos_err = (math.exp(-CONTROL_DT / POSITION_TAU_S) * self.pos_err
                            + (local_vel[0] - target_v) * CONTROL_DT)
        if self.yaw_err is None:
            self.yaw_err = 0.0
        else:
            self.yaw_err = float(np.clip(
                math.exp(-CONTROL_DT / YAW_TAU_S) * self.yaw_err
                + (actual_yaw_rate - target_yaw) * CONTROL_DT,
                -YAW_CLIP_RAD, YAW_CLIP_RAD))
        sample = np.asarray([pitch, raw[13], local_vel[0]], dtype=np.float32)
        if self.hist is None:
            self.hist = [sample.copy() for _ in range(HISTORY_LEN)]
        self.hist.insert(0, sample)
        del self.hist[HISTORY_LEN:]
        obs = np.concatenate(
            [base] + [self.hist[t] for t in HISTORY_TAPS]
            + [[self.pos_err * POSITION_OBS_SCALE, self.yaw_err * YAW_OBS_SCALE]]
        ).astype(np.float32)
        if obs.shape != (34,):
            raise RuntimeError(f"observation builder produced {obs.shape}, expected (34,)")
        return obs


@dataclass
class EpisodeConfig:
    suite: str
    episode: int
    seed: int
    initial_roll: float
    initial_pitch: float
    random_velocity: bool
    target_v: float
    target_yaw: float
    payload_kg: float


class TzejunPlant:
    def __init__(self, policy) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.data = mujoco.MjData(self.model)
        if not math.isclose(float(self.model.opt.timestep), 0.001, abs_tol=1e-12):
            raise RuntimeError(f"expected 0.001 s physics timestep, got {self.model.opt.timestep}")
        self.policy = policy
        self.obs_state = ObservationState()
        self.payload_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "payload_body")
        self.turn_filt = 0.0
        self.prev_potential = 0.0
        self.prev_action: np.ndarray | None = None
        self.target_v = 0.0
        self.target_yaw = 0.0
        self.step_count = 0

    def _potential(self, pitch: float, roll: float, vf: float, vl: float, turn: float) -> float:
        wf = min(1.0, abs(self.target_v) / MAX_V_FORWARD) if MAX_V_FORWARD else 0.0
        wt = min(1.0, abs(self.target_yaw) / MAX_V_TURN) if MAX_V_TURN else 0.0
        fw = (1.0 - wf) * 0.5 + wf
        tw = (1.0 - wt) * 0.1 + wt
        posture = 1.0 - max(wf, wt)
        return float(-posture * (pitch * pitch + roll * roll)
                     - fw * (vf - self.target_v) ** 2
                     - tw * (turn - self.target_yaw) ** 2
                     - 0.5 * vl * vl)

    def reset(self, cfg: EpisodeConfig) -> np.ndarray:
        mujoco.mj_resetData(self.model, self.data)
        self.model.dof_armature[6:8] = ARMATURE_NOMINAL
        self.model.body_mass[self.payload_id] = cfg.payload_kg
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[3:7] = quat_from_roll_pitch(cfg.initial_roll, cfg.initial_pitch)
        self.data.qpos[7:9] = 0.0
        self.data.qvel[:] = 0.0
        if cfg.random_velocity:
            rng = np.random.default_rng(cfg.seed)
            # RealRobotEnv draws roll/pitch first and qvel second from the same
            # generator. Consume the first draw to preserve that exact order.
            rng.uniform(-0.05, 0.05, size=2)
            self.data.qvel[:6] = rng.uniform(-0.10, 0.10, size=6)
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.target_v, self.target_yaw = cfg.target_v, cfg.target_yaw
        self.step_count = 0
        self.prev_action = None
        self.obs_state.reset()
        roll, pitch, _yaw, lv, yaw_rate = decode_state(self.data)
        self.turn_filt = yaw_rate
        self.prev_potential = self._potential(pitch, roll, float(lv[0]), float(lv[1]), yaw_rate)
        return self.obs_state.build(self.data, self.target_v, self.target_yaw)

    def step(self, obs: np.ndarray) -> tuple[np.ndarray, dict]:
        self.step_count += 1
        action = self.policy.predict(obs)
        torque = action_to_torque(action.astype(np.float64), np.asarray(self.data.qvel[6:8]))
        self.data.ctrl[:] = torque
        for _ in range(FRAME_SKIP):
            mujoco.mj_step(self.model, self.data)
        roll, pitch, yaw, lv, yaw_rate = decode_state(self.data)
        vf = float(lv[0])
        effort = float(np.sum(np.square(torque / 0.4)))
        reward = 2.0 - 3.0 * abs(pitch) - 0.1 * effort
        reward -= 2.0 * abs(vf - self.target_v)
        self.turn_filt = TURN_LPF * self.turn_filt + (1.0 - TURN_LPF) * yaw_rate
        reward -= TURN_WEIGHT * abs(self.turn_filt - self.target_yaw)
        reward -= TURN_INSTANT_WEIGHT * abs(yaw_rate - self.target_yaw)
        potential = self._potential(pitch, roll, vf, float(lv[1]), self.turn_filt)
        reward += SHAPING_GAMMA * potential - self.prev_potential
        self.prev_potential = potential
        terminated = self.step_count > SETTLE_STEPS and (
            abs(pitch) > FALL_ANGLE or abs(roll) > FALL_ANGLE)
        if terminated:
            reward = -10.0
        next_obs = self.obs_state.build(self.data, self.target_v, self.target_yaw)
        if self.prev_action is not None:
            reward -= ACTION_RATE_WEIGHT * float(np.sum(np.square(action - self.prev_action)))
        self.prev_action = action.copy()
        reward -= POSITION_WEIGHT * abs(float(self.obs_state.pos_err))
        reward -= YAW_WEIGHT * abs(float(self.obs_state.yaw_err))
        pwm = action_to_pwm(action.astype(np.float64))
        record = {
            "t_s": self.step_count * CONTROL_DT,
            "x_m": float(self.data.qpos[0]),
            "pitch_deg": math.degrees(pitch), "roll_deg": math.degrees(roll), "yaw_deg": math.degrees(yaw),
            "pitch_rate_rad_s": float(self.data.qvel[4]), "yaw_rate_rad_s": yaw_rate,
            "v_forward_m_s": vf,
            "action_left": float(action[0]), "action_right": float(action[1]),
            "pwm_left": float(pwm[0]), "pwm_right": float(pwm[1]),
            "torque_left_nm": float(torque[0]), "torque_right_nm": float(torque[1]),
            "reward": float(reward), "terminated": bool(terminated),
        }
        return next_obs, record


def settling_time(pitch_deg: np.ndarray, dt: float = CONTROL_DT) -> float | None:
    window = max(1, int(round(0.5 / dt)))
    inside = np.abs(pitch_deg) < 2.0
    if len(inside) < window:
        return None
    hits = np.convolve(inside.astype(np.int32), np.ones(window, dtype=np.int32), mode="valid")
    indices = np.flatnonzero(hits == window)
    return float(indices[0] * dt) if len(indices) else None


def episode_metrics(rows: list[dict], cfg: EpisodeConfig) -> dict:
    p = np.asarray([r["pitch_deg"] for r in rows], dtype=np.float64)
    torque = np.asarray([[r["torque_left_nm"], r["torque_right_nm"]] for r in rows])
    speed = np.asarray([r["v_forward_m_s"] for r in rows])
    yaw_rate = np.asarray([r["yaw_rate_rad_s"] for r in rows])
    tail = slice(max(0, len(rows) - int(2.0 / CONTROL_DT)), None)
    settle = settling_time(p)
    initial_deg = abs(math.degrees(cfg.initial_pitch))
    rms = float(np.sqrt(np.mean(np.square(p[tail])))) if len(p) else None
    overshoot = float(max(0.0, np.max(np.abs(p)) - initial_deg)) if len(p) else None
    survived = bool(rows and not rows[-1]["terminated"] and len(rows) == MAX_STEPS)
    return {
        "suite": cfg.suite, "episode": cfg.episode, "seed": cfg.seed,
        "initial_pitch_deg": math.degrees(cfg.initial_pitch),
        "target_v_m_s": cfg.target_v, "target_yaw_rad_s": cfg.target_yaw,
        "payload_kg": cfg.payload_kg, "steps": len(rows), "duration_s": len(rows) * CONTROL_DT,
        "survived": survived, "return": float(sum(r["reward"] for r in rows)),
        "settling_time_s": settle,
        "max_abs_pitch_deg": float(np.max(np.abs(p))) if len(p) else None,
        "recovery_overshoot_deg": overshoot, "steady_rms_pitch_deg": rms,
        "mean_abs_torque_nm": float(np.mean(np.abs(torque))) if len(torque) else None,
        "rms_torque_nm": float(np.sqrt(np.mean(np.square(torque)))) if len(torque) else None,
        "torque_variance": float(np.var(torque)) if len(torque) else None,
        "final_x_m": float(rows[-1]["x_m"]) if rows else None,
        "mean_v_tail_m_s": float(np.mean(speed[len(speed) // 2:])) if len(speed) else None,
        "mean_yaw_rate_tail_rad_s": float(np.mean(yaw_rate[len(yaw_rate) // 2:])) if len(yaw_rate) else None,
        "success": bool(settle is not None and settle < 1.5 and overshoot < 3.0
                        and rms is not None and rms < 1.5 and survived),
    }


def run_episode(plant: TzejunPlant, cfg: EpisodeConfig) -> tuple[list[dict], dict]:
    obs = plant.reset(cfg)
    rows: list[dict] = []
    for _ in range(MAX_STEPS):
        obs, row = plant.step(obs)
        row.update({"suite": cfg.suite, "episode": cfg.episode, "seed": cfg.seed,
                    "target_v_m_s": cfg.target_v, "target_yaw_rad_s": cfg.target_yaw,
                    "payload_kg": cfg.payload_kg})
        rows.append(row)
        if row["terminated"]:
            break
    return rows, episode_metrics(rows, cfg)


def aggregate(metrics: list[dict]) -> dict:
    keys = ("settling_time_s", "recovery_overshoot_deg", "steady_rms_pitch_deg",
            "mean_abs_torque_nm", "rms_torque_nm", "torque_variance", "final_x_m",
            "mean_v_tail_m_s", "mean_yaw_rate_tail_rad_s", "return", "duration_s")
    out: dict[str, object] = {"episodes": len(metrics)}
    for key in keys:
        vals = np.asarray([m[key] for m in metrics if m[key] is not None], dtype=np.float64)
        out[key] = {
            "mean": float(np.mean(vals)) if len(vals) else None,
            "std": float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0 if len(vals) else None,
            "ci95": float(1.96 * np.std(vals, ddof=1) / math.sqrt(len(vals))) if len(vals) > 1 else 0.0 if len(vals) else None,
            "n": int(len(vals)),
        }
    out["survival_fraction"] = float(np.mean([m["survived"] for m in metrics]))
    out["success_rate"] = float(np.mean([m["success"] for m in metrics]))
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows({k: row[k] for k in CSV_FIELDS} for row in rows)


def make_plots(fixed_rows: list[dict], official_metrics: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    representative = [r for r in fixed_rows if r["episode"] == 0]
    t = np.asarray([r["t_s"] for r in representative])
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    axes[0, 0].plot(t, [r["pitch_deg"] for r in representative], label="pitch")
    axes[0, 0].plot(t, [r["roll_deg"] for r in representative], label="roll")
    axes[0, 0].axhline(2, color="grey", ls="--", lw=0.8); axes[0, 0].axhline(-2, color="grey", ls="--", lw=0.8)
    axes[0, 0].set(ylabel="angle (deg)", title="Recovery from +10 deg"); axes[0, 0].legend()
    axes[0, 1].plot(t, [r["v_forward_m_s"] for r in representative], label="actual")
    axes[0, 1].plot(t, [r["target_v_m_s"] for r in representative], label="command")
    axes[0, 1].set(ylabel="speed (m/s)", title="Station keeping"); axes[0, 1].legend()
    axes[1, 0].plot(t, [r["action_left"] for r in representative], label="left")
    axes[1, 0].plot(t, [r["action_right"] for r in representative], label="right", alpha=0.8)
    axes[1, 0].set(xlabel="time (s)", ylabel="pre-deadband action", title="Exact policy output"); axes[1, 0].legend()
    axes[1, 1].plot(t, [r["torque_left_nm"] for r in representative], label="left")
    axes[1, 1].plot(t, [r["torque_right_nm"] for r in representative], label="right", alpha=0.8)
    axes[1, 1].set(xlabel="time (s)", ylabel="torque (N m)", title="Motor-model torque"); axes[1, 1].legend()
    fig.suptitle("tzejun best_real policy: measured-plant MuJoCo replay")
    fig.savefig(OUT_DIR / "fixed_pitch_recovery.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    cmd_v = np.asarray([m["target_v_m_s"] for m in official_metrics]); act_v = np.asarray([m["mean_v_tail_m_s"] for m in official_metrics])
    cmd_y = np.asarray([m["target_yaw_rad_s"] for m in official_metrics]); act_y = np.asarray([m["mean_yaw_rate_tail_rad_s"] for m in official_metrics])
    axes[0].scatter(cmd_v, act_v, c=[m["payload_kg"] for m in official_metrics], cmap="viridis")
    axes[0].plot([-0.35, 0.35], [-0.35, 0.35], "k--", lw=1)
    axes[0].set(xlabel="commanded speed (m/s)", ylabel="actual tail mean (m/s)", title="20-command forward tracking")
    axes[1].scatter(cmd_y, act_y, c=[m["payload_kg"] for m in official_metrics], cmap="viridis")
    axes[1].plot([-0.55, 0.55], [-0.55, 0.55], "k--", lw=1)
    axes[1].set(xlabel="commanded yaw rate (rad/s)", ylabel="actual tail mean (rad/s)", title="20-command yaw tracking")
    fig.savefig(OUT_DIR / "official_command_tracking.png", dpi=160); plt.close(fig)


def run_experiment(seeds: int) -> dict:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    started_utc = datetime.now(timezone.utc).isoformat()
    start_clock = time.perf_counter()
    policy = CheckpointPolicy(); validation = policy.validate(); plant = TzejunPlant(policy)
    all_rows: list[dict] = []; fixed_rows: list[dict] = []
    fixed_metrics: list[dict] = []; official_metrics: list[dict] = []
    print(f"checkpoint_sha256={sha256(CHECKPOINT_PATH)}")
    print(f"torch={policy.torch_version} stable_baselines3={policy.sb3_version}")
    print(f"checkpoint_test_max_abs_error={validation['max_abs_checkpoint_vs_reference_error']:.9g}")
    print(f"checkpoint_export_max_abs_error={validation['max_abs_checkpoint_vs_export_error']:.9g}")
    print(f"physics_dt={plant.model.opt.timestep:.6f}s control_hz={1.0 / CONTROL_DT:.1f}")
    for seed in range(seeds):
        cfg = EpisodeConfig("fixed_pitch", seed, seed, 0.0,
                            math.radians(10.0 if seed % 2 == 0 else -10.0),
                            False, 0.0, 0.0, 0.0)
        rows, metrics = run_episode(plant, cfg)
        fixed_rows.extend(rows); all_rows.extend(rows); fixed_metrics.append(metrics)
        print(f"fixed seed={seed:02d} init={metrics['initial_pitch_deg']:+.1f}deg "
              f"steps={metrics['steps']:4d} settle={metrics['settling_time_s']} "
              f"rms={metrics['steady_rms_pitch_deg']:.4f}deg "
              f"x_final={metrics['final_x_m']:+.4f}m success={metrics['success']}")
    for ep, ((vf, vy), payload) in enumerate(zip(EVAL_COMMANDS, EVAL_PAYLOADS)):
        seed = 1000 + ep; rng = np.random.default_rng(seed)
        roll, pitch = rng.uniform(-0.05, 0.05, size=2)
        cfg = EpisodeConfig("official_schedule", ep, seed, float(roll), float(pitch), True,
                            float(vf * MAX_V_FORWARD), float(vy * MAX_V_TURN), float(payload))
        rows, metrics = run_episode(plant, cfg)
        all_rows.extend(rows); official_metrics.append(metrics)
        print(f"official ep={ep:02d} cmd=({cfg.target_v:+.3f},{cfg.target_yaw:+.3f}) "
              f"payload={payload:.2f} steps={metrics['steps']:4d} return={metrics['return']:.2f} "
              f"v={metrics['mean_v_tail_m_s']:+.3f} yaw={metrics['mean_yaw_rate_tail_rad_s']:+.3f}")
    write_csv(OUT_DIR / "rollouts.csv", all_rows)
    (OUT_DIR / "fixed_pitch_episodes.json").write_text(json.dumps(fixed_metrics, indent=2), encoding="utf-8")
    (OUT_DIR / "official_schedule_episodes.json").write_text(json.dumps(official_metrics, indent=2), encoding="utf-8")
    make_plots(fixed_rows, official_metrics)
    summary = {
        "executed": True,
        "run": {"started_utc": started_utc,
                "finished_utc": datetime.now(timezone.utc).isoformat(),
                "wall_time_s": time.perf_counter() - start_clock,
                "python_executable": sys.executable},
        "controller": "tzejun run-7 best_real PPO actor",
        "controller_type": "standalone PPO PWM policy (not PID residual; no LLM)",
        "source_commit": SOURCE_COMMIT,
        "checkpoint": {"path": "models/best_real/best_model.zip", "sha256": sha256(CHECKPOINT_PATH),
                       "expected_sha256": CHECKPOINT_SHA256, "bytes": CHECKPOINT_PATH.stat().st_size,
                       "inference_backend": "Stable-Baselines3 PPO.load + PyTorch CPU",
                       "actor_architecture": "34 -> 64 -> 64 -> 2",
                       "critic_architecture": "34 -> 64 -> 64 -> 1",
                       "validation": validation},
        "exported_actor": {"path": "firmware/policy_weights.h", "sha256": sha256(WEIGHTS_PATH),
                           "expected_sha256": WEIGHTS_SHA256,
                           "architecture": "34 -> 64 -> 64 -> 2; tanh hidden activations; linear clipped output",
                           "validation": validation},
        "simulation": {"xml": "real_robot.xml", "mujoco_version": mujoco.__version__,
                       "numpy_version": np.__version__, "python_version": platform.python_version(),
                       "torch_version": policy.torch_version,
                       "stable_baselines3_version": policy.sb3_version,
                       "physics_timestep_s": float(plant.model.opt.timestep),
                       "control_rate_hz": 1.0 / CONTROL_DT, "episode_steps": MAX_STEPS,
                       "episode_duration_s": MAX_STEPS * CONTROL_DT},
        "fixed_pitch_protocol": {
            "seeds": list(range(seeds)), "initial_pitch_rad": "+/-0.17453292519943295 alternating",
            "initial_pitch_rate_rad_s": 0.0, "wheel_position_rad": 0.0,
            "target_speed_m_s": 0.0, "target_yaw_rad_s": 0.0,
            "settling_definition": "first entry into |pitch| < 2 deg sustained for 0.5 s",
            "success_definition": "settling <1.5 s, recovery overshoot <3 deg, tail RMS pitch <1.5 deg, survived 10 s",
            "aggregate": aggregate(fixed_metrics)},
        "official_tzejun_schedule": {"episodes": len(official_metrics),
                                     "seeds": list(range(1000, 1000 + len(official_metrics))),
                                     "payloads_kg": list(EVAL_PAYLOADS),
                                     "aggregate": aggregate(official_metrics)},
        "limitations": [
            "The run evaluates the repository checkpoint; it does not retrain it.",
            "The tzejun policy is a standalone PPO PWM controller, so it is not evidence for an LLM reward or PPO-on-PID residual claim.",
            "The checkpoint was loaded directly with Stable-Baselines3/PyTorch; exported weights were used only as an independent action cross-check.",
            "Simulation results are not hardware validation.",
        ],
    }
    if summary["checkpoint"]["sha256"] != CHECKPOINT_SHA256:
        raise RuntimeError("checkpoint hash differs from inspected tzejun artifact")
    if summary["exported_actor"]["sha256"] != WEIGHTS_SHA256:
        raise RuntimeError("exported weight hash differs from inspected tzejun artifact")
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"summary={OUT_DIR / 'summary.json'}")
    print(json.dumps(summary["fixed_pitch_protocol"]["aggregate"], indent=2))
    return summary


def view_policy() -> None:
    import glfw
    import mujoco.viewer
    policy = CheckpointPolicy(); policy.validate(); plant = TzejunPlant(policy)
    cfg = EpisodeConfig("viewer", 0, 0, 0.0, 0.0, False, 0.0, 0.0, 0.0)
    obs = plant.reset(cfg)
    state = {"run": True, "reset": False, "v": 0.0, "yaw": 0.0}
    def keys(keycode: int) -> None:
        if keycode == glfw.KEY_UP: state["v"] = 0.0 if state["v"] > 0 else MAX_V_FORWARD
        elif keycode == glfw.KEY_DOWN: state["v"] = 0.0 if state["v"] < 0 else -MAX_V_FORWARD
        elif keycode == glfw.KEY_LEFT: state["yaw"] = 0.0 if state["yaw"] > 0 else MAX_V_TURN
        elif keycode == glfw.KEY_RIGHT: state["yaw"] = 0.0 if state["yaw"] < 0 else -MAX_V_TURN
        elif keycode == glfw.KEY_SPACE: state["v"] = state["yaw"] = 0.0
        elif keycode == glfw.KEY_R: state["reset"] = True
        elif keycode == glfw.KEY_ESCAPE: state["run"] = False
    print("Viewer: arrow keys toggle drive/turn; Space stops; R resets; Esc quits.")
    with mujoco.viewer.launch_passive(plant.model, plant.data, key_callback=keys) as viewer:
        while viewer.is_running() and state["run"]:
            tick = time.perf_counter()
            if state["reset"]: obs = plant.reset(cfg); state["reset"] = False
            plant.target_v = float(state["v"]); plant.target_yaw = float(state["yaw"])
            obs, record = plant.step(obs)
            if record["terminated"]: print(f"fall/reset at t={record['t_s']:.3f}s"); obs = plant.reset(cfg)
            viewer.sync()
            delay = CONTROL_DT - (time.perf_counter() - tick)
            if delay > 0: time.sleep(delay)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=15, help="fixed-pitch evaluation episodes")
    parser.add_argument("--check", action="store_true", help="load the checkpoint and run one MuJoCo control step")
    parser.add_argument("--view", action="store_true", help="open interactive MuJoCo viewer")
    args = parser.parse_args()
    if args.seeds < 1: parser.error("--seeds must be at least 1")
    if args.check:
        policy = CheckpointPolicy()
        validation = policy.validate()
        plant = TzejunPlant(policy)
        cfg = EpisodeConfig("check", 0, 0, 0.0, 0.0, False, 0.0, 0.0, 0.0)
        obs = plant.reset(cfg)
        _, record = plant.step(obs)
        print(f"checkpoint_sha256={sha256(CHECKPOINT_PATH)}")
        print(f"checkpoint_reference_error={validation['max_abs_checkpoint_vs_reference_error']}")
        print(f"physics_dt={plant.model.opt.timestep} control_dt={CONTROL_DT}")
        print(f"one_step_pitch_deg={record['pitch_deg']:.6f}")
        print("CHECK PASSED")
    elif args.view:
        view_policy()
    else:
        run_experiment(args.seeds)


if __name__ == "__main__":
    main()
