"""Data-driven optimal controller for the MuJoCo balance robot.

The policy identifies a local linear model directly from short MuJoCo rollouts
and solves the discrete LQR problem.  No hand-tuned PD/PID gains are used at
runtime: the feedback matrix is learned from the simulated dynamics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class LearningReport:
    samples: int
    spectral_radius: float
    controllability_rank: int


@dataclass(frozen=True)
class OnlineLearningProgress:
    """Small, UI-friendly snapshot of a live system-identification session."""

    episodes: int
    samples: int
    policy_updates: int
    phase: str
    spectral_radius: float | None


class LearnedLQRPolicy:
    """System-identification plus LQR policy for common/differential wheel torque."""

    state_size = 4  # pitch, pitch rate, forward speed, yaw rate

    def __init__(
        self,
        model: mujoco.MjModel,
        samples: int = 800,
        seed: int = 7,
        control_interval: int = 10,
    ) -> None:
        self.model = model
        self.control_interval = control_interval
        self._gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self._velocity_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_velocity"
        )
        self._left_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")
        ]
        self._right_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")
        ]
        self.gain, self.equilibrium, self.report = self._learn(samples=samples, seed=seed)

    @staticmethod
    def _pitch(quaternion: np.ndarray) -> float:
        w, x, y, z = map(float, quaternion)
        return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))

    def state(self, data: mujoco.MjData) -> np.ndarray:
        gyro = data.sensordata[
            self.model.sensor_adr[self._gyro_id] : self.model.sensor_adr[self._gyro_id] + 3
        ]
        velocity = data.sensordata[
            self.model.sensor_adr[self._velocity_id] : self.model.sensor_adr[self._velocity_id] + 3
        ]
        return np.asarray(
            (self._pitch(data.qpos[3:7]), gyro[1], velocity[0], gyro[2]), dtype=float
        )

    def _set_sample_state(self, data: mujoco.MjData, rng: np.random.Generator) -> None:
        mujoco.mj_resetData(self.model, data)
        pitch = rng.uniform(math.radians(-6.0), math.radians(6.0))
        half = pitch / 2.0
        data.qpos[3:7] = (math.cos(half), 0.0, math.sin(half), 0.0)
        data.qvel[4] = rng.uniform(-0.7, 0.7)
        forward_speed = rng.uniform(-0.35, 0.35)
        data.qvel[0] = forward_speed
        wheel_rate = forward_speed / 0.16
        data.qvel[self._left_dof] = wheel_rate + rng.uniform(-0.2, 0.2)
        data.qvel[self._right_dof] = wheel_rate + rng.uniform(-0.2, 0.2)
        data.qvel[5] = rng.uniform(-0.4, 0.4)
        mujoco.mj_forward(self.model, data)

    @staticmethod
    def _solve_dare(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
        p = q.copy()
        for _ in range(2_000):
            middle = r + b.T @ p @ b
            next_p = a.T @ p @ a - a.T @ p @ b @ np.linalg.solve(middle, b.T @ p @ a) + q
            if np.max(np.abs(next_p - p)) < 1e-10:
                p = next_p
                break
            p = next_p
        return np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)

    @classmethod
    def from_transitions(
        cls,
        states: np.ndarray,
        actions: np.ndarray,
        next_states: np.ndarray,
        control_interval: int,
    ) -> "LearnedLQRPolicy":
        """Fit an LQR policy from measured ``(x, u, x_next)`` transitions.

        This constructor is deliberately independent of MuJoCo rollout code so
        the same identification path can be driven by live simulation data (or
        later by recorded hardware data).
        """
        x = np.asarray(states, dtype=float)
        u = np.asarray(actions, dtype=float)
        y = np.asarray(next_states, dtype=float)
        if x.ndim != 2 or x.shape[1] != cls.state_size or u.shape != (len(x), 2):
            raise ValueError("invalid transition matrix shape")
        if y.shape != x.shape or len(x) < 3 * (cls.state_size + 2):
            raise ValueError("not enough measured transitions for identification")
        if not (np.isfinite(x).all() and np.isfinite(u).all() and np.isfinite(y).all()):
            raise ValueError("transitions contain non-finite values")

        regressors = np.column_stack((x, u, np.ones(len(x))))
        coefficients, *_ = np.linalg.lstsq(regressors, y, rcond=None)
        a = coefficients[: cls.state_size].T
        b = coefficients[cls.state_size : cls.state_size + 2].T
        offset = coefficients[-1]
        stationary = np.column_stack((np.eye(cls.state_size) - a, -b))
        equilibrium, *_ = np.linalg.lstsq(stationary, offset, rcond=None)
        equilibrium_state = equilibrium[: cls.state_size]

        q = np.diag((75.0, 4.0, 6.0, 1.5))
        r = np.diag((0.22, 0.35))
        gain = cls._solve_dare(a, b, q, r)
        controllability = np.column_stack((b, a @ b, a @ a @ b, a @ a @ a @ b))
        report = LearningReport(
            samples=len(x),
            spectral_radius=float(np.max(np.abs(np.linalg.eigvals(a - b @ gain)))),
            controllability_rank=int(np.linalg.matrix_rank(controllability)),
        )
        if not (
            np.isfinite(gain).all()
            and np.isfinite(equilibrium_state).all()
            and math.isfinite(report.spectral_radius)
        ):
            raise ValueError("identified model did not yield a finite policy")

        policy = cls.__new__(cls)
        policy.control_interval = control_interval
        policy.gain = gain
        policy.equilibrium = equilibrium_state
        policy.report = report
        return policy

    def _learn(self, samples: int, seed: int) -> tuple[np.ndarray, np.ndarray, LearningReport]:
        rng = np.random.default_rng(seed)
        states: list[np.ndarray] = []
        actions: list[np.ndarray] = []
        next_states: list[np.ndarray] = []
        data = mujoco.MjData(self.model)
        for _ in range(samples):
            self._set_sample_state(data, rng)
            state = self.state(data)
            common = rng.uniform(-1.8, 1.8)
            differential = rng.uniform(-0.5, 0.5)
            data.ctrl[0] = common - differential
            data.ctrl[1] = common + differential
            for _ in range(self.control_interval):
                mujoco.mj_step(self.model, data)
            states.append(state)
            actions.append(np.asarray((common, differential)))
            next_states.append(self.state(data))

        learned = self.from_transitions(
            np.asarray(states),
            np.asarray(actions),
            np.asarray(next_states),
            self.control_interval,
        )
        return learned.gain, learned.equilibrium, learned.report

    def act(
        self,
        state: np.ndarray,
        target_speed: float,
        target_yaw_rate: float,
        torque_limit: float,
    ) -> tuple[float, float]:
        target = self.equilibrium.copy()
        target[2] = target_speed
        target[3] = target_yaw_rate
        common, differential = -self.gain @ (state - target)
        left = float(np.clip(common - differential, -torque_limit, torque_limit))
        right = float(np.clip(common + differential, -torque_limit, torque_limit))
        return left, right


class OnlineLQRSession:
    """Episode-based online identification that can be observed in the viewer.

    Early episodes deliberately use bounded torque exploration and may fall.
    Each 20 ms transition comes from the *running* MuJoCo environment.  After
    enough data has accumulated, a candidate LQR policy is fitted from those
    measurements; a stable candidate is then used in a visible verification
    episode.  Failed verification episodes contribute more data and retry.
    """

    def __init__(
        self,
        timestep: float,
        control_interval: int = 10,
        seed: int = 19,
        min_samples: int = 90,
    ) -> None:
        self.timestep = timestep
        self.control_interval = control_interval
        self.min_samples = min_samples
        self._rng = np.random.default_rng(seed)
        self._states: list[np.ndarray] = []
        self._actions: list[np.ndarray] = []
        self._next_states: list[np.ndarray] = []
        self.policy: LearnedLQRPolicy | None = None
        self.episodes = 0
        self.policy_updates = 0
        self.phase = "准备进行受限探索"
        self._episode_seconds = 0.0
        self._successful_policy_seconds = 0.0
        self.complete = False

    @property
    def progress(self) -> OnlineLearningProgress:
        return OnlineLearningProgress(
            episodes=self.episodes,
            samples=len(self._states),
            policy_updates=self.policy_updates,
            phase=self.phase,
            spectral_radius=None if self.policy is None else self.policy.report.spectral_radius,
        )

    def begin_episode(self) -> float:
        """Begin a new rollout and return a small randomized starting pitch."""
        self.episodes += 1
        self._episode_seconds = 0.0
        self._successful_policy_seconds = 0.0
        if self.policy is None:
            self.phase = f"探索回合 {self.episodes}：收集真实状态转移"
        else:
            self.phase = f"验证回合 {self.episodes}：使用刚学习的策略"
        # The first 9.5° perturbation makes the initially untrained behaviour
        # immediately visible; subsequent resets cover both tilt directions.
        if self.episodes == 1:
            return math.radians(9.5)
        return float(self._rng.uniform(math.radians(-7.0), math.radians(7.0)))

    def choose_action(self, state: np.ndarray, torque_limit: float) -> tuple[float, float]:
        """Choose bounded exploration or learned common/differential torque."""
        if self.policy is None:
            common = float(self._rng.uniform(-3.0, 3.0))
            differential = float(self._rng.uniform(-0.80, 0.80))
            left = common - differential
            right = common + differential
        else:
            left, right = self.policy.act(state, 0.0, 0.0, torque_limit)
        return (
            float(np.clip(left, -torque_limit, torque_limit)),
            float(np.clip(right, -torque_limit, torque_limit)),
        )

    def record_transition(
        self,
        state: np.ndarray,
        wheel_torques: tuple[float, float],
        next_state: np.ndarray,
    ) -> None:
        common = 0.5 * (wheel_torques[0] + wheel_torques[1])
        differential = 0.5 * (wheel_torques[1] - wheel_torques[0])
        self._states.append(np.asarray(state, dtype=float).copy())
        self._actions.append(np.asarray((common, differential), dtype=float))
        self._next_states.append(np.asarray(next_state, dtype=float).copy())
        self._episode_seconds += self.timestep * self.control_interval

    def should_finish_episode(self, state: np.ndarray) -> bool:
        pitch_magnitude = abs(float(state[0]))
        if self.policy is None:
            return pitch_magnitude > math.radians(16.0) or self._episode_seconds >= 0.80
        if pitch_magnitude > math.radians(18.0):
            self.phase = "验证未通过，补充该失败轨迹后重新辨识"
            return True
        self._successful_policy_seconds += self.timestep * self.control_interval
        if self._successful_policy_seconds >= 4.0:
            self.complete = True
            self.phase = "学习完成：已通过 4 秒稳定验证"
        return False

    def update_policy(self) -> bool:
        """Fit only after a full-rank, stable candidate has enough live data."""
        if len(self._states) < self.min_samples:
            self.phase = f"已收集 {len(self._states)}/{self.min_samples} 条转移，继续探索"
            return False
        try:
            candidate = LearnedLQRPolicy.from_transitions(
                np.asarray(self._states),
                np.asarray(self._actions),
                np.asarray(self._next_states),
                self.control_interval,
            )
        except (ValueError, np.linalg.LinAlgError):
            self.phase = "辨识模型不可用，继续收集更多轨迹"
            return False
        report = candidate.report
        if report.controllability_rank < LearnedLQRPolicy.state_size or report.spectral_radius >= 0.995:
            self.phase = "候选策略未通过稳定性检查，继续探索"
            return False
        self.policy = candidate
        self.policy_updates += 1
        self.phase = (
            f"第 {self.policy_updates} 次在线更新完成，开始稳定性验证"
        )
        return True
