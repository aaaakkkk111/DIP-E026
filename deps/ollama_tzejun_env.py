"""Gymnasium adapter of the EXISTING tzejun measured-plant MuJoCo environment.

Observation/history, physics, motor transfer, action bounds and fall grace are
reused from tzejun_experiment.py. Only the scalar training reward is replaced.
No PID/reward NN is added; real_robot.xml and motor_model.py are not modified.
Run .venv/Scripts/python.exe -E ollama_tzejun_env.py --self-test for actual
Gym/SB3, seeded replay and original-plant parity checks. Simulation validation
does not prove hardware safety.
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Mapping

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from motor_model import action_to_pwm, action_to_torque
from tzejun_experiment import (
    ARMATURE_NOMINAL, CONTROL_DT, FALL_ANGLE, FRAME_SKIP, MAX_STEPS,
    MAX_V_FORWARD, MAX_V_TURN, SETTLE_STEPS, XML_PATH, ObservationState,
    decode_state, quat_from_roll_pitch,
)

REWARD_KEYS = ("angle", "rate", "position", "speed", "yaw", "effort", "slew", "survival")
DEFAULT_REWARD_WEIGHTS = {
    "angle": 0.22, "rate": 0.08, "position": 0.12, "speed": 0.18,
    "yaw": 0.20, "effort": 0.05, "slew": 0.05, "survival": 0.10,
}
REWARD_SCALES = {
    "angle_rad": math.radians(10.0), "rate_rad_s": 2.0,
    "position_m": 0.25, "speed_m_s": 0.15, "yaw_rad_s": 0.25,
    "effort_nm": 0.40, "slew_action": 0.20,
}
FALL_PENALTY = 25.0  # Immutable: the LLM cannot change/remove this penalty.


def validate_reward_weights(weights: Mapping[str, float] | None) -> dict[str, float]:
    """Reject invalid proposals without silently renormalizing them."""
    result = dict(DEFAULT_REWARD_WEIGHTS if weights is None else weights)
    if set(result) != set(REWARD_KEYS):
        raise ValueError(f"reward weights must have exactly these keys: {REWARD_KEYS}")
    for key, value in result.items():
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
            raise ValueError(f"reward weight {key} must be numeric, not {type(value).__name__}")
        if not math.isfinite(float(value)) or not 0.0 <= float(value) <= 1.0:
            raise ValueError(f"reward weight {key} must be finite and within [0, 1]")
        result[key] = float(value)
    if not math.isclose(sum(result.values()), 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("reward weights must sum to 1.0 within 1e-6")
    if result["angle"] <= 0.0:
        raise ValueError("angle penalty must be positive")
    return result


class OllamaTzejunEnv(gym.Env):
    """34-observation/2-PWM-action interface, 200 Hz, same checkpoint plant.

    randomize varies in-memory chassis mass/inertia +/-10%, friction .5..1.2,
    payload 0..0.25 kg and reflected motor armature .0018...0027 kg m^2. These
    ranges are experiment assumptions, not measured physical parameters.
    Commands default to zero. command_probability retains optional tracking.

    reset options: pitch, roll (rad), pitch_rate (rad/s), wheel_position (rad,
    scalar or pair), target_v (m/s), target_yaw (rad/s), payload (kg), friction,
    mass_scale, armature. Default angles are uniform +/-10 and +/-1 degrees;
    qvel otherwise starts at zero. Evaluation must pass explicit fixed angles.

    reward_terms replaces the original absolute/potential-shaping reward with
    normalized squared costs and a fixed fall penalty. Returns measured on
    those two different rewards must not be compared on the same numeric scale.
    """
    metadata = {"render_modes": [], "render_fps": 30}

    def __init__(self, reward_weights: Mapping[str, float] | None = None,
                 max_steps: int = MAX_STEPS, randomize: bool = False,
                 command_probability: float = 0.0) -> None:
        super().__init__()
        if int(max_steps) != max_steps or max_steps < 1:
            raise ValueError("max_steps must be a positive integer")
        if not 0 <= command_probability <= 1:
            raise ValueError("command_probability must be within [0, 1]")
        self.max_steps = int(max_steps)
        self.randomize = bool(randomize)
        self.command_probability = float(command_probability)
        self.reward_weights = validate_reward_weights(reward_weights)
        self.model = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.data = mujoco.MjData(self.model)
        if not math.isclose(float(self.model.opt.timestep), .001, abs_tol=1e-12):
            raise RuntimeError("real_robot.xml must retain its 1 ms physics timestep")
        if (self.model.nq, self.model.nv, self.model.nu) != (9, 8, 2):
            raise RuntimeError("require free base plus two wheel joints")
        if FRAME_SKIP != 5 or not math.isclose(CONTROL_DT, .005, abs_tol=1e-12):
            raise RuntimeError("tzejun checkpoint requires 5 physics steps / 200 Hz control")
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(34,), dtype=np.float32)
        self.action_space = spaces.Box(-1., 1., shape=(2,), dtype=np.float32)
        self.obs_state = ObservationState()
        self.chassis_id = self.model.body("chassis").id
        self.payload_id = self.model.body("payload_body").id
        self._body_mass = self.model.body_mass.copy()
        self._body_inertia = self.model.body_inertia.copy()
        self.target_v = self.target_yaw = 0.
        self.payload_kg = 0.
        self.friction_mu = .9
        self.external_force = np.zeros(3, dtype=np.float64)
        self.reference_xy = np.zeros(2, dtype=np.float64)
        self.step_count = 0
        self.prev_action: np.ndarray | None = None
        self.last_info: dict = {}
        self._episode_done = False

    def set_reward_weights(self, weights: Mapping[str, float]) -> None:
        self.reward_weights = validate_reward_weights(weights)

    def set_command(self, speed: float = 0., yaw: float = 0.) -> None:
        if not (math.isfinite(speed) and math.isfinite(yaw)):
            raise ValueError("commands must be finite")
        if abs(speed) > MAX_V_FORWARD or abs(yaw) > MAX_V_TURN:
            raise ValueError("commands exceed existing tzejun range")
        self.target_v, self.target_yaw = float(speed), float(yaw)

    def set_external_force(self, force) -> None:
        arr = np.asarray(force, dtype=np.float64)
        if arr.shape != (3,) or not np.all(np.isfinite(arr)):
            raise ValueError("external force needs three finite world-frame components")
        self.external_force[:] = arr

    def _set_parameters(self, payload: float, friction: float, mass_scale: float, armature: float) -> None:
        if not all(math.isfinite(v) for v in (payload, friction, mass_scale, armature)):
            raise ValueError("plant parameters must be finite")
        if not (0 <= payload <= 4 and .05 <= friction <= 2 and .5 <= mass_scale <= 2 and 0 < armature <= .01):
            raise ValueError("plant parameters outside documented simulation ranges")
        self.payload_kg, self.friction_mu = float(payload), float(friction)
        self.model.body_mass[:] = self._body_mass
        self.model.body_inertia[:] = self._body_inertia
        self.model.body_mass[self.chassis_id] *= mass_scale
        self.model.body_inertia[self.chassis_id] *= mass_scale
        self.model.body_mass[self.payload_id] = payload
        # Same existing payload box as RobustPlant (.06 x .06 x .04 m).
        # Uniform-box inertia is an ADDED payload assumption, not CAD evidence.
        hx, hy, hz = .03, .03, .02
        self.model.body_inertia[self.payload_id] = payload * np.asarray(
            [hy*hy+hz*hz, hx*hx+hz*hz, hx*hx+hy*hy]) / 3.
        self.model.geom_friction[:, 0] = friction  # Change both contacting surfaces.
        self.model.dof_armature[6:8] = armature
        self.model.geom_rgba[self.model.geom("payload_geom").id, 3] = 1. if payload else 0.
        mujoco.mj_setConst(self.model, self.data)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        opts = dict(options or {})
        initial = {
            "pitch": float(self.np_random.uniform(-math.radians(10), math.radians(10))),
            "roll": float(self.np_random.uniform(-math.radians(1), math.radians(1))),
            "payload": 0., "friction": .9, "mass_scale": 1.,
            "armature": ARMATURE_NOMINAL, "pitch_rate": 0., "wheel_position": 0.,
            "target_v": 0., "target_yaw": 0.,
        }
        if self.randomize:
            initial.update(payload=float(self.np_random.uniform(0, .25)),
                           friction=float(self.np_random.uniform(.5, 1.2)),
                           mass_scale=float(self.np_random.uniform(.9, 1.1)),
                           armature=float(self.np_random.uniform(.0018, .0027)))
        if self.command_probability > 0 and self.np_random.random() < self.command_probability:
            initial.update(target_v=float(self.np_random.uniform(-.15, .15)),
                           target_yaw=float(self.np_random.uniform(-.25, .25)))
        unknown = set(opts) - set(initial)
        if unknown:
            raise ValueError(f"unknown reset options: {sorted(unknown)}")
        initial.update(opts)
        mujoco.mj_resetData(self.model, self.data)
        self._set_parameters(*(float(initial[k]) for k in ("payload", "friction", "mass_scale", "armature")))
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[3:7] = quat_from_roll_pitch(float(initial["roll"]), float(initial["pitch"]))
        self.data.qpos[7:9] = initial["wheel_position"]
        self.data.qvel[:] = 0.
        self.data.qvel[4] = float(initial["pitch_rate"])
        self.data.act[:] = 0.
        self.data.ctrl[:] = 0.
        self.data.xfrc_applied[:] = 0.
        self.data.qfrc_applied[:] = 0.
        mujoco.mj_forward(self.model, self.data)
        self.set_command(float(initial["target_v"]), float(initial["target_yaw"]))
        self.external_force[:] = 0.
        self.reference_xy[:] = self.data.qpos[:2]
        self.step_count = 0
        self.prev_action = None
        self.obs_state.reset()
        self._episode_done = False
        obs = self.obs_state.build(self.data, self.target_v, self.target_yaw)
        self.last_info = self._state_info(np.zeros(2), np.zeros(2), np.zeros(2), False)
        self.last_info["reset_parameters"] = {
            k: float(v) if np.isscalar(v) else np.asarray(v).tolist() for k, v in initial.items()}
        return obs, dict(self.last_info)

    def _state_info(self, action, requested_torque, actual_torque, terminated: bool) -> dict:
        roll, pitch, yaw, local_vel, yaw_rate = decode_state(self.data)
        pwm = action_to_pwm(action)
        return {
            "t_s": float(self.data.time), "step": int(self.step_count),
            "pitch": float(pitch), "roll": float(roll), "yaw": float(yaw),
            "pitch_deg": math.degrees(pitch), "roll_deg": math.degrees(roll), "yaw_deg": math.degrees(yaw),
            "pitch_rate_rad_s": float(self.data.qvel[4]), "yaw_rate_rad_s": float(yaw_rate),
            "v_forward_m_s": float(local_vel[0]), "x_m": float(self.data.qpos[0]),
            "y_m": float(self.data.qpos[1]), "radius_m": float(np.linalg.norm(self.data.qpos[:2])),
            "position_error_m": float(np.linalg.norm(self.data.qpos[:2] - self.reference_xy)),
            "target_v_m_s": self.target_v, "target_yaw_rad_s": self.target_yaw,
            "action_left": float(action[0]), "action_right": float(action[1]),
            "pwm_left": float(pwm[0]), "pwm_right": float(pwm[1]),
            "raw_torque_left_nm": float(requested_torque[0]), "raw_torque_right_nm": float(requested_torque[1]),
            "torque_left_nm": float(actual_torque[0]), "torque_right_nm": float(actual_torque[1]),
            "payload_kg": self.payload_kg, "friction_mu": self.friction_mu,
            "force_x_n": float(self.external_force[0]), "force_y_n": float(self.external_force[1]),
            "terminated": bool(terminated),
        }

    def reward_terms(self, info: Mapping, action: np.ndarray, previous_action: np.ndarray | None) -> dict[str, float]:
        """Normalized costs; cap fallen-state magnitudes to keep finite arithmetic."""
        def squared_cost(value, scale):
            arr = np.clip(np.asarray(value, dtype=np.float64) / scale, -10., 10.)
            return -float(np.mean(np.square(arr)))
        return {
            "angle": squared_cost(info["pitch"], REWARD_SCALES["angle_rad"]),
            "rate": squared_cost(info["pitch_rate_rad_s"], REWARD_SCALES["rate_rad_s"]),
            "position": squared_cost(info["position_error_m"], REWARD_SCALES["position_m"]),
            "speed": squared_cost(info["v_forward_m_s"] - self.target_v, REWARD_SCALES["speed_m_s"]),
            "yaw": squared_cost(info["yaw_rate_rad_s"] - self.target_yaw, REWARD_SCALES["yaw_rad_s"]),
            "effort": squared_cost([info["torque_left_nm"], info["torque_right_nm"]], REWARD_SCALES["effort_nm"]),
            "slew": 0. if previous_action is None else squared_cost(action - previous_action, REWARD_SCALES["slew_action"]),
            "survival": 0. if info["terminated"] else 1.,
        }

    def step(self, action):
        if self._episode_done:
            raise RuntimeError("reset() required after terminated/truncated")
        command = np.asarray(action, dtype=np.float32)
        if command.shape != (2,) or not np.all(np.isfinite(command)):
            raise ValueError("action must contain two finite numbers")
        command = np.clip(command, -1., 1.)
        # Identical to TzejunPlant: motor torque evaluated at control start,
        # held over five fixed 1 ms physics steps, independent of rendering.
        requested = action_to_torque(command.astype(np.float64), np.asarray(self.data.qvel[6:8]))
        self.data.ctrl[:] = requested
        self.data.xfrc_applied[:] = 0.
        self.data.xfrc_applied[self.chassis_id, :3] = self.external_force
        _, _, yaw, _, _ = decode_state(self.data)
        self.reference_xy += CONTROL_DT * self.target_v * np.asarray([math.cos(yaw), math.sin(yaw)])
        for _ in range(FRAME_SKIP):
            mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        roll, pitch, _, _, _ = decode_state(self.data)
        terminated = bool(self.step_count > SETTLE_STEPS and (abs(pitch) > FALL_ANGLE or abs(roll) > FALL_ANGLE))
        truncated = bool(self.step_count >= self.max_steps and not terminated)
        next_obs = self.obs_state.build(self.data, self.target_v, self.target_yaw)
        info = self._state_info(command, requested, self.data.actuator_force[:2], terminated)
        terms = self.reward_terms(info, command, self.prev_action)
        contributions = {key: self.reward_weights[key] * terms[key] for key in REWARD_KEYS}
        reward = float(sum(contributions.values()) - (FALL_PENALTY if terminated else 0.))
        info.update(reward=reward, reward_terms=terms, reward_contributions=contributions,
                    fall_penalty=-FALL_PENALTY if terminated else 0., truncated=truncated)
        if not np.all(np.isfinite(next_obs)) or not math.isfinite(reward):
            raise FloatingPointError("MuJoCo produced non-finite state or reward")
        self.prev_action = command.copy()
        self.last_info = info
        self._episode_done = terminated or truncated
        return next_obs, reward, terminated, truncated, dict(info)


def self_test() -> dict:
    """Execute checks rather than inferring compatibility from file names."""
    from stable_baselines3.common.env_checker import check_env
    from tzejun_experiment import EpisodeConfig, ExportedPolicy, TzejunPlant
    env = OllamaTzejunEnv(max_steps=500)
    check_env(env, warn=True)
    options = {"pitch": math.radians(10), "roll": 0., "pitch_rate": 0.,
               "wheel_position": 0., "target_v": 0., "target_yaw": 0.}
    actor = ExportedPolicy()
    parity = TzejunPlant(actor)
    ref_obs = parity.reset(EpisodeConfig("adapter-check", 0, 7, 0., math.radians(10), False, 0., 0., 0.))
    obs, _ = env.reset(seed=7, options=options)
    max_obs_error = float(np.max(np.abs(obs - ref_obs)))
    max_qpos_error = 0.
    for _ in range(200):
        obs, reward, terminated, truncated, _ = env.step(actor.predict(obs))
        ref_obs, record = parity.step(ref_obs)
        max_obs_error = max(max_obs_error, float(np.max(np.abs(obs - ref_obs))))
        max_qpos_error = max(max_qpos_error, float(np.max(np.abs(env.data.qpos - parity.data.qpos))))
        assert isinstance(reward, float)
        assert terminated == record["terminated"]
        if terminated or truncated:
            break
    assert max_obs_error < 2e-6, max_obs_error
    assert max_qpos_error < 1e-8, max_qpos_error
    def replay():
        result = []
        obs, _ = env.reset(seed=42, options=options)
        for _ in range(100):
            obs, reward, done, cutoff, _ = env.step(actor.predict(obs))
            result.append(np.concatenate([env.data.qpos.copy(), env.data.qvel.copy(), [reward]]))
            if done or cutoff:
                break
        return np.stack(result)
    first, second = replay(), replay()
    assert np.array_equal(first, second), float(np.max(np.abs(first - second)))
    env.reset(seed=11, options={**options, "payload": .5, "friction": .5})
    env.set_external_force([.5, 0, 0])
    _, reward, _, _, info = env.step([0, 0])
    assert info["payload_kg"] == .5 and info["friction_mu"] == .5 and math.isfinite(reward)
    result = {"passed": True, "gym_sb3_check_env": "passed", "physics_dt_s": float(env.model.opt.timestep),
              "control_dt_s": CONTROL_DT, "observation_shape": list(env.observation_space.shape),
              "action_shape": list(env.action_space.shape), "plant_max_qpos_error": max_qpos_error,
              "observation_max_abs_error": max_obs_error, "seeded_replay_bit_identical": True,
              "payload_friction_force_smoke": "passed", "reward_is_neural_network": False}
    print(json.dumps(result, indent=2))
    env.close()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if not args.self_test:
        parser.error("this adapter is imported by training; use --self-test")
    self_test()
