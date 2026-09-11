"""Hybrid firmware-PID plus bounded neural residual control in MuJoCo.

The official Normal/Weight_M controller remains the primary controller.  A
transparent supervisor selects Weight_M at |pitch| >= 10 degrees and returns
to Normal at |pitch| <= 7 degrees.  A small PPO actor adds bounded left/right
PWM residuals after the firmware PID path and before the motor plant.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Protocol

import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
import numpy as np

import firmware_pid_sim as fw


ROOT = Path(__file__).resolve().parent
POLICY_DIR = ROOT / "neural_policy"
POLICY_PATH = POLICY_DIR / "hybrid_residual_ppo.zip"
METADATA_PATH = POLICY_DIR / "policy_metadata.json"
EVALUATION_PATH = POLICY_DIR / "evaluation.json"

SWITCH_TO_WEIGHT_DEG = 10.0
SWITCH_BACK_TO_NORMAL_DEG = 7.0
RESIDUAL_PWM_LIMIT = 350
EPISODE_SECONDS = 4.0
POLICY_OBSERVATION_NAMES = (
    "pitch_deg",
    "pitch_rate_deg_s",
    "forward_speed_m_s",
    "yaw_rate_rad_s",
    "encoder_left_counts_5ms",
    "encoder_right_counts_5ms",
    "encoder_bias",
    "encoder_integral",
    "target_speed_m_s",
    "target_yaw_rate_rad_s",
    "base_pwm_left",
    "base_pwm_right",
    "weight_mode",
)
POLICY_OBSERVATION_SCALES = np.asarray(
    (30.0, 400.0, 1.0, 5.0, 60.0, 60.0, 60.0, 8000.0, 0.5, 1.0, 2600.0, 2600.0, 1.0),
    dtype=np.float32,
)


class ResidualPolicy(Protocol):
    name: str

    def predict(self, observation: np.ndarray) -> np.ndarray:
        """Return normalized left/right residual actions in [-1, 1]."""


class ZeroResidualPolicy:
    name = "zero residual (hybrid PID baseline)"

    def predict(self, observation: np.ndarray) -> np.ndarray:
        del observation
        return np.zeros(2, dtype=np.float32)


class PPOResidualPolicy:
    def __init__(self, path: Path = POLICY_PATH) -> None:
        from stable_baselines3 import PPO

        if not path.exists():
            raise FileNotFoundError(
                f"trained policy not found: {path}\n"
                "Run: python neural_residual_sim.py --train --timesteps 50000"
            )
        self.path = path
        self.model = PPO.load(path, device="cpu")
        self.name = f"PPO deterministic actor ({path.name})"

    def predict(self, observation: np.ndarray) -> np.ndarray:
        action, _state = self.model.predict(observation, deterministic=True)
        return np.asarray(action, dtype=np.float32).reshape(2)


@dataclass
class HybridModeSupervisor:
    enter_weight_deg: float = SWITCH_TO_WEIGHT_DEG
    exit_weight_deg: float = SWITCH_BACK_TO_NORMAL_DEG
    mode: fw.ControlMode = fw.ControlMode.NORMAL

    def reset(self) -> None:
        self.mode = fw.ControlMode.NORMAL

    def select(self, pitch_rad: float) -> fw.ControlMode:
        magnitude_deg = abs(math.degrees(pitch_rad))
        if self.mode is fw.ControlMode.NORMAL and magnitude_deg >= self.enter_weight_deg:
            self.mode = fw.ControlMode.WEIGHT
        elif self.mode is fw.ControlMode.WEIGHT and magnitude_deg <= self.exit_weight_deg:
            self.mode = fw.ControlMode.NORMAL
        return self.mode


def policy_observation(
    simulation: fw.FirmwarePidSimulation,
    state: fw.RobotObservation,
    base_pwm: tuple[int, int],
) -> np.ndarray:
    telemetry = simulation.controller.last
    raw = np.asarray(
        (
            math.degrees(state.pitch_rad),
            math.degrees(state.pitch_rate_rad_s),
            state.forward_speed_m_s,
            state.yaw_rate_rad_s,
            telemetry.encoder_left,
            telemetry.encoder_right,
            simulation.controller.encoder_bias,
            simulation.controller.encoder_integral,
            simulation.controller.target_speed_m_s,
            simulation.controller.target_yaw_rate_rad_s,
            base_pwm[0],
            base_pwm[1],
            1.0 if simulation.controller.mode is fw.ControlMode.WEIGHT else 0.0,
        ),
        dtype=np.float32,
    )
    return np.clip(raw / POLICY_OBSERVATION_SCALES, -5.0, 5.0).astype(np.float32)


class NeuralResidualSimulation(fw.FirmwarePidSimulation):
    """Firmware simulation with automatic mode switching and PWM residuals."""

    def __init__(
        self,
        policy: ResidualPolicy,
        payload_kg: float = 0.0,
        initial_pitch_deg: float = 1.5,
    ) -> None:
        self.policy = policy
        self.supervisor = HybridModeSupervisor()
        self.last_policy_observation = np.zeros(len(POLICY_OBSERVATION_NAMES), dtype=np.float32)
        self.last_action = np.zeros(2, dtype=np.float32)
        self.last_base_pwm = (0, 0)
        self.last_residual_pwm = (0, 0)
        self.last_combined_pwm = (0, 0)
        super().__init__(
            mode=fw.ControlMode.NORMAL,
            payload_kg=payload_kg,
            initial_pitch_deg=initial_pitch_deg,
        )
        # The firmware-only baseline intentionally limits its startup angle to
        # 15 degrees.  Assisted-control validation also covers large switching
        # transients, while remaining below the 40-degree firmware cutoff.
        self.initial_pitch_deg = float(np.clip(initial_pitch_deg, -35.0, 35.0))
        self.reset()

    def reset(self) -> None:
        if hasattr(self, "controller"):
            self.controller.set_mode(fw.ControlMode.NORMAL)
        super().reset()
        self.supervisor.reset()
        self.last_policy_observation[:] = 0.0
        self.last_action[:] = 0.0
        self.last_base_pwm = (0, 0)
        self.last_residual_pwm = (0, 0)
        self.last_combined_pwm = (0, 0)

    def set_mode(self, mode: fw.ControlMode) -> None:
        """Keep mode ownership in the hybrid supervisor while assisted control is active."""
        del mode
        self.controller.set_mode(self.supervisor.mode)

    def prepare_control(self) -> np.ndarray:
        state = self.observe()
        selected_mode = self.supervisor.select(state.pitch_rad)
        self.controller.set_mode(selected_mode)
        self.last_base_pwm = self.controller.update(state)
        self.last_policy_observation = policy_observation(self, state, self.last_base_pwm)
        return self.last_policy_observation.copy()

    def apply_residual_action(self, action: np.ndarray) -> tuple[int, int]:
        normalized = np.clip(np.asarray(action, dtype=np.float32).reshape(2), -1.0, 1.0)
        residual = np.rint(normalized * RESIDUAL_PWM_LIMIT).astype(int)
        if self.controller.last.stopped:
            residual[:] = 0
            combined = np.zeros(2, dtype=int)
        else:
            combined = np.clip(
                np.asarray(self.last_base_pwm, dtype=int) + residual,
                -fw.PWM_COMMAND_LIMIT,
                fw.PWM_COMMAND_LIMIT,
            ).astype(int)
        self.last_action = normalized
        self.last_residual_pwm = (int(residual[0]), int(residual[1]))
        self.last_combined_pwm = (int(combined[0]), int(combined[1]))
        self.motor.set_pwm(*self.last_combined_pwm)
        return self.last_combined_pwm

    def _physics_step(self) -> fw.RobotObservation:
        self._apply_external_force()
        self.last_torque = self.motor.step(self.data, self.controller.battery_voltage)
        mujoco.mj_step(self.model, self.data)
        self.physics_steps += 1
        return self.observe()

    def advance_control_interval(self) -> fw.RobotObservation:
        state = self.observe()
        for _ in range(fw.CONTROL_STEPS):
            state = self._physics_step()
        return state

    def step(self) -> fw.RobotObservation:
        if self.physics_steps % fw.CONTROL_STEPS == 0:
            observation = self.prepare_control()
            self.apply_residual_action(self.policy.predict(observation))
        return self._physics_step()


class ResidualBalanceEnv(gym.Env[np.ndarray, np.ndarray]):
    """200 Hz Gymnasium environment used to train the bounded residual actor."""

    metadata = {"render_modes": []}

    def __init__(self, episode_seconds: float = EPISODE_SECONDS) -> None:
        super().__init__()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -5.0,
            5.0,
            shape=(len(POLICY_OBSERVATION_NAMES),),
            dtype=np.float32,
        )
        self.horizon = round(episode_seconds / fw.CONTROL_PERIOD_S)
        self.simulation = NeuralResidualSimulation(ZeroResidualPolicy(), initial_pitch_deg=0.0)
        self.steps = 0
        self.push_step = 0
        self.push_direction = "forward"
        self.push_force_n = 0.0
        self.previous_action = np.zeros(2, dtype=np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        del options
        if self.np_random.random() < 0.30:
            initial_pitch = float(self.np_random.uniform(-26.0, 26.0))
        else:
            initial_pitch = float(self.np_random.uniform(-8.0, 8.0))
        self.simulation.initial_pitch_deg = initial_pitch
        self.simulation.set_payload_mass(float(self.np_random.uniform(0.0, 0.20)))
        self.simulation.set_ground_friction(float(self.np_random.uniform(0.90, 1.30)))
        self.simulation.reset()
        self.simulation.set_battery_voltage(float(self.np_random.uniform(10.8, 12.6)))
        self.simulation.set_targets(
            float(self.np_random.uniform(-0.15, 0.15)),
            float(self.np_random.uniform(-0.45, 0.45)),
        )
        self.simulation.data.qvel[4] = float(self.np_random.uniform(-0.70, 0.70))
        mujoco.mj_forward(self.simulation.model, self.simulation.data)
        self.steps = 0
        self.push_step = int(self.np_random.integers(80, max(81, self.horizon - 160)))
        self.push_direction = str(self.np_random.choice(tuple(fw.PUSH_DIRECTIONS)))
        self.push_force_n = float(self.np_random.uniform(1.0, 6.0))
        self.previous_action[:] = 0.0
        observation = self.simulation.prepare_control()
        return observation, self._info(self.simulation.observe())

    def _info(self, state: fw.RobotObservation) -> dict:
        return {
            "pitch_deg": math.degrees(state.pitch_rad),
            "pitch_rate_deg_s": math.degrees(state.pitch_rate_rad_s),
            "speed_m_s": state.forward_speed_m_s,
            "mode": self.simulation.controller.mode.value,
            "base_pwm": self.simulation.last_base_pwm,
            "residual_pwm": self.simulation.last_residual_pwm,
            "combined_pwm": self.simulation.last_combined_pwm,
            "stopped": self.simulation.controller.last.stopped,
        }

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        if self.steps == self.push_step:
            self.simulation.push_direction(
                self.push_direction,
                self.push_force_n,
                fw.PUSH_DURATION_S,
            )
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self.simulation.apply_residual_action(action)
        state = self.simulation.advance_control_interval()
        self.steps += 1

        pitch = state.pitch_rad
        pitch_rate = state.pitch_rate_rad_s
        speed_error = (
            state.forward_speed_m_s - self.simulation.controller.target_speed_m_s
        )
        yaw_error = state.yaw_rate_rad_s - self.simulation.controller.target_yaw_rate_rad_s
        action_delta = action - self.previous_action
        self.previous_action = action.copy()
        reward = (
            1.0
            - 8.0 * pitch * pitch
            - 0.05 * pitch_rate * pitch_rate
            - 1.5 * speed_error * speed_error
            - 0.15 * yaw_error * yaw_error
            - 0.010 * float(action @ action)
            - 0.015 * float(action_delta @ action_delta)
        )
        if abs(math.degrees(pitch)) < 3.0:
            reward += 0.20

        terminated = bool(
            self.simulation.controller.last.stopped
            or abs(math.degrees(pitch)) >= 40.0
            or self.simulation.data.qpos[2] < 0.015
        )
        truncated = self.steps >= self.horizon
        if terminated:
            reward -= 25.0

        if terminated or truncated:
            observation = policy_observation(
                self.simulation, state, self.simulation.last_base_pwm
            )
        else:
            observation = self.simulation.prepare_control()
        return observation, float(reward), terminated, truncated, self._info(state)


def train_policy(total_timesteps: int, seed: int) -> Path:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    import torch

    POLICY_DIR.mkdir(parents=True, exist_ok=True)
    environment = ResidualBalanceEnv()
    model = PPO(
        "MlpPolicy",
        environment,
        learning_rate=3.0e-4,
        n_steps=2048,
        batch_size=256,
        n_epochs=10,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.20,
        ent_coef=0.001,
        policy_kwargs={
            "activation_fn": torch.nn.ReLU,
            "net_arch": {"pi": [32, 32], "vf": [64, 64]},
        },
        verbose=1,
        seed=seed,
        device="cpu",
    )
    checkpoint = CheckpointCallback(
        save_freq=10_000,
        save_path=str(POLICY_DIR / "checkpoints"),
        name_prefix="hybrid_residual",
    )
    model.learn(total_timesteps=max(2048, total_timesteps), callback=checkpoint)
    model.save(POLICY_PATH)
    metadata = {
        "algorithm": "PPO",
        "model": POLICY_PATH.name,
        "seed": seed,
        "training_timesteps": int(model.num_timesteps),
        "deterministic_inference": True,
        "residual_pwm_limit_each_motor": RESIDUAL_PWM_LIMIT,
        "switch_to_weight_deg": SWITCH_TO_WEIGHT_DEG,
        "switch_back_to_normal_deg": SWITCH_BACK_TO_NORMAL_DEG,
        "observation_names": POLICY_OBSERVATION_NAMES,
        "observation_scales": POLICY_OBSERVATION_SCALES.tolist(),
        "action_order": ["left_pwm_residual", "right_pwm_residual"],
        "actor_hidden_layers": [32, 32],
        "activation": "ReLU",
    }
    METADATA_PATH.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    environment.close()
    print(f"saved policy: {POLICY_PATH}")
    print(f"saved metadata: {METADATA_PATH}")
    return POLICY_PATH


def _rollout_metrics(policy: ResidualPolicy, seed: int) -> dict:
    environment = ResidualBalanceEnv()
    observation, _info = environment.reset(seed=seed)
    pitch_samples: list[float] = []
    residual_samples: list[float] = []
    terminated = truncated = False
    final_info: dict = {}
    while not (terminated or truncated):
        action = policy.predict(observation)
        observation, _reward, terminated, truncated, final_info = environment.step(action)
        pitch_samples.append(abs(float(final_info["pitch_deg"])))
        residual_samples.extend(abs(float(value)) for value in final_info["residual_pwm"])
    environment.close()
    pitch_array = np.asarray(pitch_samples, dtype=float)
    return {
        "fallen": bool(terminated),
        "mean_abs_pitch_deg": float(np.mean(pitch_array)),
        "rms_pitch_deg": float(np.sqrt(np.mean(pitch_array * pitch_array))),
        "max_abs_pitch_deg": float(np.max(pitch_array)),
        "mean_abs_residual_pwm": float(np.mean(residual_samples)),
    }


def evaluate_policy(policy: ResidualPolicy, episodes: int, seed: int) -> dict:
    baseline_rows = [_rollout_metrics(ZeroResidualPolicy(), seed + i) for i in range(episodes)]
    assisted_rows = [_rollout_metrics(policy, seed + i) for i in range(episodes)]

    def summarize(rows: list[dict]) -> dict:
        return {
            "episodes": len(rows),
            "falls": sum(int(row["fallen"]) for row in rows),
            "mean_abs_pitch_deg": float(np.mean([row["mean_abs_pitch_deg"] for row in rows])),
            "rms_pitch_deg": float(np.mean([row["rms_pitch_deg"] for row in rows])),
            "mean_max_abs_pitch_deg": float(np.mean([row["max_abs_pitch_deg"] for row in rows])),
            "mean_abs_residual_pwm": float(
                np.mean([row["mean_abs_residual_pwm"] for row in rows])
            ),
        }

    result = {
        "seed_start": seed,
        "hybrid_pid_without_neural_residual": summarize(baseline_rows),
        "hybrid_pid_with_neural_residual": summarize(assisted_rows),
    }
    POLICY_DIR.mkdir(parents=True, exist_ok=True)
    EVALUATION_PATH.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"saved evaluation: {EVALUATION_PATH}")
    return result


class NeuralControlPanel(fw.ControlPanel):
    def __init__(self, simulation: NeuralResidualSimulation) -> None:
        self.policy_label = simulation.policy.name
        super().__init__(simulation)
        self.root.title("Neural residual + STM32 firmware PID - MuJoCo")

    def _refresh_model_info(self) -> None:
        super()._refresh_model_info()
        self.model_info.set(
            self.model_info.get()
            + f"\nassist threshold  : {SWITCH_TO_WEIGHT_DEG:.0f}° / {SWITCH_BACK_TO_NORMAL_DEG:.0f}°"
            + f"\nresidual limit    : ±{RESIDUAL_PWM_LIMIT} PWM"
        )

    def update(self, observation: fw.RobotObservation) -> bool:
        simulation: NeuralResidualSimulation = self.simulation
        self.mode.set(simulation.controller.mode.value)
        if not super().update(observation):
            return False
        self.status.set(
            self.status.get()
            + f"\nAUTO MODE={simulation.controller.mode.value:8s}  "
            + f"base={simulation.last_base_pwm}  residual={simulation.last_residual_pwm}  "
            + f"motor={simulation.last_combined_pwm}"
        )
        try:
            self.root.update_idletasks()
            self.root.update()
            return True
        except self.tk.TclError:
            return False


def run_viewer(simulation: NeuralResidualSimulation) -> None:
    panel = NeuralControlPanel(simulation)
    with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
        with viewer.lock():
            viewer.cam.distance = 0.65
            viewer.cam.azimuth = 120
            viewer.cam.elevation = -18
            viewer.cam.lookat[:] = np.asarray(
                (simulation.data.qpos[0], simulation.data.qpos[1], fw.CAMERA_LOOKAT_HEIGHT_M)
            )
        while viewer.is_running():
            started = time.perf_counter()
            state = simulation.step()
            if panel.camera_follow_enabled:
                with viewer.lock():
                    viewer.cam.lookat[:] = fw.camera_follow_lookat(
                        viewer.cam.lookat, simulation.data.qpos[:3]
                    )
            viewer.sync()
            if not panel.update(state):
                break
            remaining = simulation.model.opt.timestep - (time.perf_counter() - started)
            if remaining > 0.0:
                time.sleep(remaining)


def run_headless(simulation: NeuralResidualSimulation, duration_s: float) -> dict:
    maximum_pitch = 0.0
    weight_ticks = 0
    state = simulation.observe()
    for _ in range(round(max(0.0, duration_s) / simulation.model.opt.timestep)):
        state = simulation.step()
        maximum_pitch = max(maximum_pitch, abs(math.degrees(state.pitch_rad)))
        weight_ticks += int(simulation.controller.mode is fw.ControlMode.WEIGHT)
    result = {
        "policy": simulation.policy.name,
        "time_s": state.time_s,
        "mode": simulation.controller.mode.value,
        "pitch_deg": math.degrees(state.pitch_rad),
        "max_abs_pitch_deg": maximum_pitch,
        "speed_m_s": state.forward_speed_m_s,
        "base_pwm": simulation.last_base_pwm,
        "residual_pwm": simulation.last_residual_pwm,
        "combined_pwm": simulation.last_combined_pwm,
        "weight_mode_physics_ticks": weight_ticks,
        "stopped": simulation.controller.last.stopped,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", action="store_true", help="train and save the PPO policy")
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--payload-kg", type=float, default=0.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=1.5)
    parser.add_argument("--speed", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--baseline", action="store_true", help="run hybrid PID with zero residual")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.train:
        train_policy(args.timesteps, args.seed)
    policy: ResidualPolicy
    policy = ZeroResidualPolicy() if args.baseline else PPOResidualPolicy(POLICY_PATH)
    if args.evaluate:
        evaluate_policy(policy, max(1, args.episodes), args.seed + 10_000)
        return 0
    simulation = NeuralResidualSimulation(
        policy,
        payload_kg=args.payload_kg,
        initial_pitch_deg=args.initial_pitch_deg,
    )
    simulation.set_targets(args.speed, args.yaw_rate)
    if args.headless or args.train:
        run_headless(simulation, args.duration)
    else:
        run_viewer(simulation)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
