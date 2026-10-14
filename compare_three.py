"""Three-way comparison: stock Yahboom PID vs frozen tzejun vs best Ollama tzejun.

All three run in the SAME existing MuJoCo plant with the SAME motor model, from
matched initial states. The PID is a faithful port of the firmware cascade
(pid_control.c, Normal-mode gains Kp=9600 Kd=48, Vel 6200/31, Turn 1700/20):
  balance  = Balance_Kp/100 * pitch_deg + Balance_Kd/100 * gyro_raw
  velocity = -Vel_Kp/100 * Enc_bias - Vel_Ki/100 * Enc_integral   (0.84/0.16 LPF)
  turn     = 0 while standing (firmware myTurn_Kd = 0, Turn_Target = 0)
then PWM_Ignore (+1300 deadband) and PWM_Limit (+/-2600), torque via pwm_to_torque.

The NN controllers use the reference observation pipeline they were trained on and
action_to_torque. Single-threaded; simulation only, not hardware validation.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
DEPS = HERE / "deps"
sys.path.insert(0, str(DEPS))

from motor_model import action_to_torque, pwm_to_torque, PWM_DEADBAND  # noqa
from tzejun_experiment import (decode_state, quat_from_roll_pitch, parse_c_array,  # noqa
                               CONTROL_DT, FRAME_SKIP, FALL_ANGLE, SETTLE_STEPS,
                               ObservationState, XML_PATH, ARMATURE_NOMINAL)

GYRO_LSB_PER_RAD_S = 939.65078
COUNTS_PER_REV = 1320.0
COUNTS_PER_TICK_PER_RAD_S = CONTROL_DT * COUNTS_PER_REV / (2 * math.pi)  # ~1.0504
NN_PWM_LIMIT = 2600.0

# Normal-mode stock gains (pid_control.c / app_mode.c Set_PID Normal branch).
BAL_KP, BAL_KD = 9600.0, 48.0
VEL_KP, VEL_KI = 6200.0, 31.0


def pwm_ignore(pwm):
    return pwm + np.sign(pwm) * PWM_DEADBAND if pwm != 0 else 0.0


class StockPID:
    name = "stock_PID"

    def __init__(self):
        self.reset()

    def reset(self):
        self.enc_bias = 0.0
        self.enc_integral = 0.0

    def torque(self, data):
        _roll, pitch, _yaw, _lv, _yr = decode_state(data)
        pitch_deg = math.degrees(pitch)
        gyro_raw = float(data.qvel[4]) * GYRO_LSB_PER_RAD_S
        balance = BAL_KP / 100.0 * pitch_deg + BAL_KD / 100.0 * gyro_raw
        enc_l = float(data.qvel[6]) * COUNTS_PER_TICK_PER_RAD_S
        enc_r = float(data.qvel[7]) * COUNTS_PER_TICK_PER_RAD_S
        enc_least = 0.0 - (enc_l + enc_r)
        self.enc_bias = self.enc_bias * 0.84 + enc_least * 0.16
        self.enc_integral += self.enc_bias  # Movement = 0 (no command)
        self.enc_integral = float(np.clip(self.enc_integral, -8000, 8000))
        velocity = -self.enc_bias * VEL_KP / 100.0 - self.enc_integral * VEL_KI / 100.0
        turn = 0.0  # standing: Turn_Target=0, myTurn_Kd=0
        ml = pwm_ignore(balance + velocity + turn)
        mr = pwm_ignore(balance + velocity - turn)
        ml = float(np.clip(ml, -NN_PWM_LIMIT, NN_PWM_LIMIT))
        mr = float(np.clip(mr, -NN_PWM_LIMIT, NN_PWM_LIMIT))
        self.last_pwm = (ml, mr)
        return np.array([pwm_to_torque(ml, data.qvel[6]), pwm_to_torque(mr, data.qvel[7])])


class NNController:
    def __init__(self, actor, name):
        self.actor = actor
        self.name = name
        self.reset()

    def reset(self):
        self.obs_state = ObservationState()
        self.obs_state.reset()

    def torque(self, data):
        obs = self.obs_state.build(data, 0.0, 0.0)
        action = self.actor.predict(obs)
        self.last_pwm = None
        return action_to_torque(np.asarray(action, dtype=np.float64), np.asarray(data.qvel[6:8]))


class SB3Actor:
    def __init__(self, path):
        from stable_baselines3 import PPO
        self.model = PPO.load(str(path), device="cpu")

    def predict(self, obs):
        a, _ = self.model.predict(np.asarray(obs, dtype=np.float32), deterministic=True)
        return np.asarray(a, dtype=np.float32)


SCENARIOS = {"nominal": dict(mu=0.9, payload=0.0, force=0.0),
             "low_friction": dict(mu=0.5, payload=0.0, force=0.0),
             "payload_025": dict(mu=0.9, payload=0.25, force=0.0),
             "forward_push": dict(mu=0.9, payload=0.0, force=1.0)}


def run_trial(controller, scn, seed, duration):
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    model.dof_armature[6:8] = ARMATURE_NOMINAL
    pid = model.body("payload_body").id
    model.geom_friction[:, 0] = scn["mu"]
    model.body_mass[pid] = scn["payload"]
    hx, hy, hz = 0.03, 0.03, 0.02
    model.body_inertia[pid] = scn["payload"] * np.array([hy*hy+hz*hz, hx*hx+hz*hz, hx*hx+hy*hy]) / 3.0
    model.geom_rgba[model.geom("payload_geom").id, 3] = 1.0 if scn["payload"] else 0.0
    mujoco.mj_setConst(model, data)
    rng = np.random.default_rng(20261012 + seed)
    pitch0 = math.radians(10.0 if seed % 2 == 0 else -10.0)
    roll0 = float(rng.uniform(-math.radians(0.5), math.radians(0.5)))
    data.qpos[:] = model.qpos0
    data.qpos[3:7] = quat_from_roll_pitch(roll0, pitch0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    controller.reset()
    chassis = model.body("chassis").id
    n = int(round(duration / CONTROL_DT))
    rows = []
    for step in range(n):
        t = step * CONTROL_DT
        data.xfrc_applied[:] = 0.0
        if scn["force"] and 6.0 <= t < 6.2:
            data.xfrc_applied[chassis, 0] = scn["force"]
        torque = controller.torque(data)
        data.ctrl[:] = torque
        for _ in range(FRAME_SKIP):
            mujoco.mj_step(model, data)
        roll, pitch, yaw, lv, yr = decode_state(data)
        rows.append((t, math.degrees(pitch), math.degrees(roll), float(lv[0]), yr,
                     float(data.qpos[0]), float(np.linalg.norm(data.qpos[:2])),
                     float(data.actuator_force[0]), float(data.actuator_force[1])))
        if step > SETTLE_STEPS and (abs(pitch) > FALL_ANGLE or abs(roll) > FALL_ANGLE):
            break
    a = np.asarray(rows)
    return metrics(a, duration)


def metrics(a, duration):
    t = a[:, 0]
    survived = bool(len(a) and t[-1] >= duration - 1e-8)
    tail = (t >= duration - 2.0 - 1e-9) if survived else np.zeros(len(t), dtype=bool)
    rms = lambda x, m: float(np.sqrt(np.mean(x[m] ** 2))) if m.any() else None
    pitch, speed, yaw, radius = a[:, 1], a[:, 3], a[:, 4], a[:, 6]
    torque = a[:, 7:9]
    # action/pwm reversal proxy: torque sign flips
    rev = float(np.mean(np.abs(np.diff(np.sign(torque[:, 0]))) > 0)) if len(a) > 1 else 0.0
    inside = (np.abs(pitch) < 2.0).astype(int)
    win = max(1, int(round(0.5 / CONTROL_DT)))
    settle = None
    if len(inside) >= win:
        h = np.convolve(inside, np.ones(win, int), "valid")
        k = np.flatnonzero(h == win)
        settle = float(t[k[0]]) if len(k) else None
    pr, sr, yr2 = rms(pitch, tail), rms(speed, tail), rms(yaw, tail)
    ms = float(np.mean(speed[tail])) if tail.any() else None
    disp = float(np.max(radius[tail])) if tail.any() else None
    success = bool(survived and settle is not None and settle < 1.5 and pr is not None and pr < 1.0
                   and sr is not None and sr < 0.02 and ms is not None and abs(ms) < 0.005
                   and yr2 is not None and yr2 < 0.05 and disp is not None and disp < 0.25)
    return {"survived": survived, "settling_s": settle, "pitch_rms_deg": pr,
            "speed_rms_m_s": sr, "mean_speed_m_s": ms, "yaw_rate_rms_rad_s": yr2,
            "max_displacement_m": disp, "mean_abs_torque_nm": float(np.mean(np.abs(torque))),
            "torque_reversal_rate": rev, "max_abs_pitch_deg": float(np.max(np.abs(pitch))),
            "stationkeeping_success": success}


def agg(ms):
    out = {"trials": len(ms), "survival": float(np.mean([m["survived"] for m in ms])),
           "stationkeeping_success": float(np.mean([m["stationkeeping_success"] for m in ms]))}
    for k in ("pitch_rms_deg", "speed_rms_m_s", "mean_speed_m_s", "yaw_rate_rms_rad_s",
              "max_displacement_m", "mean_abs_torque_nm", "torque_reversal_rate", "settling_s"):
        v = np.asarray([m[k] for m in ms if m[k] is not None], float)
        out[k] = round(float(np.mean(v)), 4) if len(v) else None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--ollama", default=str(HERE / "runs" / "expB_main" / "ollama_seed00" / "selected_policy.zip"))
    ap.add_argument("--out", type=Path, default=HERE / "runs" / "compare_three")
    args = ap.parse_args()
    import torch
    torch.set_num_threads(1)
    frozen = SB3Actor(DEPS / "models" / "best_real" / "best_model.zip")
    ollama = SB3Actor(args.ollama)
    controllers = [StockPID(), NNController(frozen, "frozen_tzejun"), NNController(ollama, "ollama_tzejun")]
    args.out.mkdir(parents=True, exist_ok=True)
    result = {}
    print(f"{'scenario':14} {'controller':16} surv  station  pitchRMS  yawRMS  speedRMS  meanSpd  disp   |tq|   rev")
    for scn_name, scn in SCENARIOS.items():
        result[scn_name] = {}
        for c in controllers:
            ms = [run_trial(c, scn, s, args.duration) for s in range(args.seeds)]
            a = agg(ms)
            result[scn_name][c.name] = a
            print(f"{scn_name:14} {c.name:16} {a['survival']:.2f}  {a['stationkeeping_success']:.2f}    "
                  f"{a['pitch_rms_deg']}   {a['yaw_rate_rms_rad_s']}   {a['speed_rms_m_s']}   "
                  f"{a['mean_speed_m_s']}  {a['max_displacement_m']}  {a['mean_abs_torque_nm']}  {a['torque_reversal_rate']}",
                  flush=True)
    (args.out / "comparison.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    # Figure: nominal bars for key metrics.
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    names = [c.name for c in controllers]
    mets = ["pitch_rms_deg", "yaw_rate_rms_rad_s", "speed_rms_m_s", "mean_abs_torque_nm", "stationkeeping_success", "survival"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    colors = ["#999999", "#0072B2", "#D55E00"]
    for ax, m in zip(axes.flat, mets):
        vals = [result["nominal"][n].get(m) or 0 for n in names]
        ax.bar(range(len(names)), vals, color=colors)
        ax.set_xticks(range(len(names)), names, rotation=15, fontsize=8)
        ax.set_title(m); ax.grid(alpha=0.25, axis="y")
    fig.suptitle("Nominal: stock PID vs frozen tzejun vs Ollama tzejun (zero-latency sim; not hardware)")
    fig.savefig(args.out / "comparison.png", dpi=150); plt.close(fig)
    print("\nwrote", args.out / "comparison.json", "and comparison.png")


if __name__ == "__main__":
    main()
