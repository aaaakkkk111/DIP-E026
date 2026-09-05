#!/usr/bin/env python3
"""Safe interactive MuJoCo demo of a two-wheel balancing robot.

Architecture:
  sensors -> state estimator -> learned LQR policy -> motors -> robot
      ^                 |                     ^
      |             1--5 s log ------- short-window monitor -> safety limits

At startup, the policy gathers short randomized MuJoCo rollouts, identifies a
local linear dynamics model, then solves a
discrete LQR problem.  The runtime loop uses that learned feedback matrix;
there is no hand-tuned PD/PID gain law.  The monitor is deterministic and
offline: it can only tighten command limits and never receives actuator access.

The experiment panel also offers a visible online-learning mode.  It begins
without using the startup policy, learns solely from successive live episodes,
and visibly progresses from bounded exploration to LQR verification.
"""

from __future__ import annotations

import argparse
import math
import re
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Deque
import tkinter as tk
from tkinter import font as tkfont

import glfw
import mujoco
import mujoco.viewer
import numpy as np

from learned_lqr import LearnedLQRPolicy, OnlineLQRSession


MODEL_PATH = Path(__file__).with_name("balance_bot.xml")
SUPPORTED_LANGUAGES = ("zh", "en")
LANGUAGE_OPTIONS = {"中文": "zh", "English": "en"}


def bilingual(language: str, chinese: str, english: str) -> str:
    """Return one localized string while keeping language selection explicit."""
    return english if language == "en" else chinese


@dataclass
class ResetAwareTicker:
    """Periodic simulation-time ticker that survives MuJoCo data resets."""

    interval: float
    next_time: float = 0.0
    previous_time: float = -math.inf
    elapsed_time: float = 0.0

    def is_due(self, sim_time: float) -> bool:
        if math.isfinite(self.previous_time):
            if sim_time < self.previous_time:
                # The first time value after mj_resetData is also the elapsed
                # portion of the new episode.
                self.elapsed_time += max(sim_time, 0.0)
            else:
                self.elapsed_time += max(sim_time - self.previous_time, 0.0)
        else:
            self.elapsed_time = max(sim_time, 0.0)

        if sim_time < self.previous_time:
            # Online learning starts every episode from t=0.  The viewer's old
            # deadline must not be carried into the newly reset episode.
            self.next_time = sim_time
        self.previous_time = sim_time
        if sim_time + 1e-12 < self.next_time:
            return False
        self.next_time = sim_time + self.interval
        return True


def localized_mode(mode: str, language: str) -> str:
    if language == "en":
        return mode
    return {
        "NORMAL": "正常",
        "STABILITY": "稳定",
        "RECOVERY": "恢复",
    }.get(mode, mode)


def localized_learning_phase(phase: str, language: str) -> str:
    """Translate the finite set of online-learning phases for the English UI."""
    if language != "en":
        return phase
    exact = {
        "准备进行受限探索": "Preparing bounded exploration",
        "验证未通过，补充该失败轨迹后重新辨识": "Validation failed; adding the failed trajectory and identifying again",
        "学习完成：已通过 4 秒稳定验证": "Learning complete: passed the 4-second stability check",
        "辨识模型不可用，继续收集更多轨迹": "Identified model is unusable; collecting more trajectories",
        "候选策略未通过稳定性检查，继续探索": "Candidate policy failed the stability check; continuing exploration",
        "探索发生跌倒，使用该轨迹更新模型": "Exploration fall detected; using this trajectory to update the model",
        "当前回合结束，利用真实轨迹进行在线辨识": "Episode complete; identifying from the measured trajectory",
    }
    if phase in exact:
        return exact[phase]
    match = re.fullmatch(r"探索回合 (\d+)：收集真实状态转移", phase)
    if match:
        return f"Exploration episode {match.group(1)}: collecting measured transitions"
    match = re.fullmatch(r"验证回合 (\d+)：使用刚学习的策略", phase)
    if match:
        return f"Validation episode {match.group(1)}: using the newly learned policy"
    match = re.fullmatch(r"已收集 (\d+)/(\d+) 条转移，继续探索", phase)
    if match:
        return f"Collected {match.group(1)}/{match.group(2)} transitions; continuing exploration"
    match = re.fullmatch(r"第 (\d+) 次在线更新完成，开始稳定性验证", phase)
    if match:
        return f"Online update {match.group(1)} complete; starting stability validation"
    return phase


def localized_analyzer_summary(summary: str, language: str) -> str:
    """Localize the deterministic short-window status used by the viewer."""
    if language == "en":
        return summary
    if summary == "collecting a 1--5 s telemetry window":
        return "正在收集 1--5 秒遥测窗口"
    match = re.fullmatch(r"collecting window \(([^)]+)\)", summary)
    if match:
        return f"正在收集窗口（{match.group(1)}）"
    match = re.fullmatch(
        r"([\d.]+)s window \| pitch rms ([\d.]+) deg, peak ([\d.]+) deg, "
        r"rate ([\d.]+) deg/s( -> guarded command limits)?",
        summary,
    )
    if match:
        suffix = " -> 已启用保守指令限制" if match.group(5) else ""
        return (
            f"{match.group(1)}s 窗口 | 俯仰 RMS {match.group(2)}°，"
            f"峰值 {match.group(3)}°，角速度 {match.group(4)}°/s{suffix}"
        )
    return summary


@dataclass
class SafetyLimits:
    """Hard limits that neither text commands nor the analyzer can bypass."""

    max_speed: float = 1.20
    max_yaw_rate: float = 1.50
    max_torque: float = 4.0
    warning_pitch: float = math.radians(10.0)


@dataclass
class Command:
    forward_speed: float = 0.0
    yaw_rate: float = 0.0
    reference_x: float = 0.0


@dataclass
class RobotState:
    sim_time: float
    x: float
    y: float
    forward_speed: float
    pitch: float
    pitch_rate: float
    yaw: float
    yaw_rate: float
    left_wheel_angle: float
    right_wheel_angle: float
    left_torque: float
    right_torque: float


@dataclass
class TelemetryFrame:
    """Compact log record sent to the short-window analyzer."""

    sim_time: float
    pitch: float
    pitch_rate: float
    forward_speed: float
    yaw_rate: float
    left_torque: float
    right_torque: float
    target_speed: float
    target_yaw_rate: float


@dataclass
class TuningUpdate:
    """The only online update allowed around the learned policy."""

    speed_limit: float | None = None
    yaw_rate_limit: float | None = None
    reason: str = "monitoring only"


@dataclass
class ScenarioConfig:
    """Interactive physical and sensor conditions for robustness experiments."""

    friction: float = 1.5
    slope_deg: float = 0.0
    imu_noise_deg_s: float = 0.0
    imu_delay_ms: float = 0.0
    motor_delay_ms: float = 0.0
    camera_follow: bool = False


@dataclass
class ExperimentMetrics:
    max_pitch_deg: float = 0.0
    max_speed: float = 0.0
    last_recovery_s: float | None = None
    recovery_count: int = 0

    def clear(self) -> None:
        self.max_pitch_deg = 0.0
        self.max_speed = 0.0
        self.last_recovery_s = None
        self.recovery_count = 0


def quaternion_to_euler(quaternion: np.ndarray) -> tuple[float, float, float]:
    """Return world-frame roll, pitch, yaw (XYZ convention) from MuJoCo WXYZ."""
    w, x, y, z = map(float, quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class StateEstimator:
    """Lightweight estimator: low-pass noisy rate and velocity channels only.

    Pitch is deliberately not delayed: on a balancing robot it is the primary
    safety signal. Real IMUs can replace this class with an EKF while keeping
    the same RobotState interface.
    """

    def __init__(self, alpha: float = 0.82) -> None:
        self.alpha = alpha
        self._previous: RobotState | None = None

    def reset(self) -> None:
        self._previous = None

    def update(self, raw: RobotState) -> RobotState:
        if self._previous is None:
            self._previous = raw
            return raw
        old = self._previous
        a = self.alpha
        estimate = replace(
            raw,
            forward_speed=a * raw.forward_speed + (1.0 - a) * old.forward_speed,
            pitch_rate=a * raw.pitch_rate + (1.0 - a) * old.pitch_rate,
            yaw_rate=a * raw.yaw_rate + (1.0 - a) * old.yaw_rate,
        )
        self._previous = estimate
        return estimate


class SimulationScenario:
    """Injects controlled disturbances without changing the controller API."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.config = ScenarioConfig()
        self.metrics = ExperimentMetrics()
        self._rng = np.random.default_rng(7)
        self._sensor_history: Deque[RobotState] = deque()
        self._motor_history: Deque[tuple[float, float]] = deque()
        self._terrain_dirty = False
        self._push_force_x = 0.0
        self._push_until = -math.inf
        self._recovery_start = -math.inf
        self._recovery_pending = False
        self.chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.tire_ids = (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_tire"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_tire"),
        )

    def reset(self, clear_metrics: bool = True) -> None:
        self._sensor_history.clear()
        self._motor_history.clear()
        self._push_force_x = 0.0
        self._push_until = -math.inf
        self._recovery_pending = False
        if clear_metrics:
            self.metrics.clear()

    def set_friction(self, value: float) -> None:
        friction = float(np.clip(value, 0.20, 2.50))
        self.config.friction = friction
        for geom_id in (self.ground_id, *self.tire_ids):
            self.model.geom_friction[geom_id, 0] = friction

    def set_slope(self, value: float) -> None:
        slope_deg = float(np.clip(value, -10.0, 10.0))
        self.config.slope_deg = slope_deg
        half_angle = math.radians(slope_deg) / 2.0
        self.model.geom_quat[self.ground_id] = (
            math.cos(half_angle),
            0.0,
            math.sin(half_angle),
            0.0,
        )
        self._terrain_dirty = True

    def set_imu_noise(self, value: float) -> None:
        self.config.imu_noise_deg_s = float(np.clip(value, 0.0, 10.0))

    def set_imu_delay(self, value: float) -> None:
        self.config.imu_delay_ms = float(np.clip(value, 0.0, 120.0))

    def set_motor_delay(self, value: float) -> None:
        self.config.motor_delay_ms = float(np.clip(value, 0.0, 120.0))

    def schedule_push(self, current_time: float, force_x: float, duration: float = 0.15) -> None:
        self._push_force_x = float(np.clip(force_x, -80.0, 80.0))
        self._push_until = current_time + duration
        self._recovery_start = self._push_until
        self._recovery_pending = True

    def apply_model_changes(self, data: mujoco.MjData) -> None:
        """Commit slider-driven terrain changes before stepping."""
        if self._terrain_dirty:
            mujoco.mj_setConst(self.model, data)
            mujoco.mj_forward(self.model, data)
            self._terrain_dirty = False

    def prepare_step(self, data: mujoco.MjData) -> None:
        self.apply_model_changes(data)
        data.xfrc_applied[:] = 0.0
        if data.time < self._push_until:
            data.xfrc_applied[self.chassis_id, 0] = self._push_force_x

    def observe(self, truth: RobotState) -> RobotState:
        """Apply selected IMU noise and whole-state sensor delay to controller input."""
        noise_rad = math.radians(self.config.imu_noise_deg_s)
        if noise_rad > 0.0:
            noisy = replace(
                truth,
                forward_speed=truth.forward_speed + self._rng.normal(0.0, noise_rad * 0.05),
                pitch_rate=truth.pitch_rate + self._rng.normal(0.0, noise_rad),
                yaw_rate=truth.yaw_rate + self._rng.normal(0.0, noise_rad),
            )
        else:
            noisy = truth
        self._sensor_history.append(noisy)
        max_samples = max(
            8,
            int(round(self.config.imu_delay_ms / (self.model.opt.timestep * 1_000.0))) + 8,
        )
        while len(self._sensor_history) > max_samples:
            self._sensor_history.popleft()
        delay_steps = int(round(self.config.imu_delay_ms / (self.model.opt.timestep * 1_000.0)))
        delayed = self._sensor_history[max(0, len(self._sensor_history) - 1 - delay_steps)]
        return replace(delayed, sim_time=truth.sim_time)

    def delay_motors(self, desired: tuple[float, float]) -> tuple[float, float]:
        self._motor_history.append(desired)
        delay_steps = int(round(self.config.motor_delay_ms / (self.model.opt.timestep * 1_000.0)))
        max_samples = max(8, delay_steps + 8)
        while len(self._motor_history) > max_samples:
            self._motor_history.popleft()
        if delay_steps >= len(self._motor_history):
            return (0.0, 0.0)
        return self._motor_history[len(self._motor_history) - 1 - delay_steps]

    def observe_metrics(self, truth: RobotState) -> None:
        self.metrics.max_pitch_deg = max(self.metrics.max_pitch_deg, abs(math.degrees(truth.pitch)))
        self.metrics.max_speed = max(self.metrics.max_speed, abs(truth.forward_speed))
        if (
            self._recovery_pending
            and truth.sim_time >= self._recovery_start
            and abs(truth.pitch) < math.radians(2.0)
            and abs(truth.forward_speed) < 0.08
        ):
            self.metrics.last_recovery_s = truth.sim_time - self._recovery_start
            self.metrics.recovery_count += 1
            self._recovery_pending = False


class LogBuffer:
    """Keeps only a bounded recent telemetry window (default: three seconds)."""

    def __init__(self, seconds: float = 3.0) -> None:
        if not 1.0 <= seconds <= 5.0:
            raise ValueError("LogBuffer window must be within 1--5 seconds")
        self.seconds = seconds
        self._frames: Deque[TelemetryFrame] = deque()

    def clear(self) -> None:
        self._frames.clear()

    def append(self, state: RobotState, command: Command) -> None:
        self._frames.append(
            TelemetryFrame(
                sim_time=state.sim_time,
                pitch=state.pitch,
                pitch_rate=state.pitch_rate,
                forward_speed=state.forward_speed,
                yaw_rate=state.yaw_rate,
                left_torque=state.left_torque,
                right_torque=state.right_torque,
                target_speed=command.forward_speed,
                target_yaw_rate=command.yaw_rate,
            )
        )
        oldest_time = state.sim_time - self.seconds
        while self._frames and self._frames[0].sim_time < oldest_time:
            self._frames.popleft()

    def frames(self) -> tuple[TelemetryFrame, ...]:
        return tuple(self._frames)


class ShortWindowAnalyzer:
    """Short-window safety monitor corresponding to the diagram's analyzer.

    It receives no model/data handle and therefore cannot write motor commands
    or alter the learned feedback matrix.  Its bounded output is applied only
    after hard command clamping.
    """

    def __init__(self, analysis_period: float = 1.0) -> None:
        self.analysis_period = analysis_period
        self._last_analysis_time = -math.inf
        self.last_summary = "collecting a 1--5 s telemetry window"

    def reset(self) -> None:
        self._last_analysis_time = -math.inf
        self.last_summary = "collecting a 1--5 s telemetry window"

    def analyze(
        self,
        frames: tuple[TelemetryFrame, ...],
        limits: SafetyLimits,
    ) -> TuningUpdate | None:
        if len(frames) < 2:
            return None
        newest = frames[-1].sim_time
        if newest - self._last_analysis_time < self.analysis_period:
            return None
        self._last_analysis_time = newest

        duration = newest - frames[0].sim_time
        if duration < 1.0:
            self.last_summary = f"collecting window ({duration:.1f}/1.0 s)"
            return None
        pitches = np.asarray([frame.pitch for frame in frames])
        rates = np.asarray([frame.pitch_rate for frame in frames])
        torques = np.asarray(
            [max(abs(frame.left_torque), abs(frame.right_torque)) for frame in frames]
        )
        pitch_rms = float(np.sqrt(np.mean(np.square(pitches))))
        peak_pitch = float(np.max(np.abs(pitches)))
        peak_rate = float(np.max(np.abs(rates)))
        torque_ratio = float(np.max(torques) / max(limits.max_torque, 1e-6))
        self.last_summary = (
            f"{duration:.1f}s window | pitch rms {math.degrees(pitch_rms):.2f} deg, "
            f"peak {math.degrees(peak_pitch):.2f} deg, "
            f"rate {math.degrees(peak_rate):.1f} deg/s"
        )

        if peak_pitch > math.radians(6.0) or (
            pitch_rms > math.radians(3.0) and torque_ratio > 0.85
        ):
            self.last_summary += " -> guarded command limits"
            return TuningUpdate(
                speed_limit=limits.max_speed * 0.85,
                yaw_rate_limit=limits.max_yaw_rate * 0.85,
                reason="large pitch excursion in recent telemetry",
            )
        return TuningUpdate(reason="window is stable; learned policy unchanged")


class SafetySupervisor:
    """Clamps targets and moves the robot into recovery before a fall."""

    def __init__(self, limits: SafetyLimits | None = None) -> None:
        self.default_limits = limits or SafetyLimits()
        self.limits = replace(self.default_limits)
        self.mode = "NORMAL"
        self.reason = "within limits"

    def reset(self) -> None:
        self.limits = replace(self.default_limits)
        self.mode = "NORMAL"
        self.reason = "within limits"

    def set_stability_mode(self) -> None:
        self.limits.max_speed = min(self.limits.max_speed, 0.75)
        self.limits.max_yaw_rate = min(self.limits.max_yaw_rate, 0.90)
        self.mode = "STABILITY"
        self.reason = "conservative command limits enabled"

    def clamp_command(self, command: Command) -> None:
        command.forward_speed = float(
            np.clip(command.forward_speed, -self.limits.max_speed, self.limits.max_speed)
        )
        command.yaw_rate = float(
            np.clip(command.yaw_rate, -self.limits.max_yaw_rate, self.limits.max_yaw_rate)
        )

    def enforce(self, state: RobotState, command: Command) -> None:
        """Freeze motion targets during a large tilt without resetting simulation state."""
        self.clamp_command(command)
        pitch_magnitude = abs(state.pitch)
        if pitch_magnitude >= self.limits.warning_pitch:
            command.forward_speed = 0.0
            command.yaw_rate = 0.0
            command.reference_x = state.x
            self.mode = "RECOVERY"
            self.reason = "motion targets frozen until pitch returns below 10 deg"
        elif self.mode == "RECOVERY":
            self.mode = "NORMAL"
            self.reason = "recovered"


class BalanceController:
    """Learned LQR balance policy with safety, delays, and short-window monitoring."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self.velocity_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_velocity"
        )
        self.left_qpos = model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")
        ]
        self.right_qpos = model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")
        ]
        self.command = Command()
        self.last_torques = (0.0, 0.0)
        self.estimator = StateEstimator()
        self.scenario = SimulationScenario(model)
        self.policy = LearnedLQRPolicy(model)
        self._policy_ticks = self.policy.control_interval - 1
        self._desired_torques = (0.0, 0.0)
        self.learning_session: OnlineLQRSession | None = None
        self._learning_previous_state: np.ndarray | None = None
        self._learning_previous_torques = (0.0, 0.0)
        self._last_online_learning_summary: str | None = None
        self.log_buffer = LogBuffer(seconds=3.0)
        self.analyzer = ShortWindowAnalyzer()
        self.supervisor = SafetySupervisor()
        self.last_update = "analyzer inactive"

    def reset(self, data: mujoco.MjData, clear_metrics: bool = True) -> None:
        mujoco.mj_resetData(self.model, data)
        mujoco.mj_forward(self.model, data)
        self.command = Command(reference_x=float(data.qpos[0]))
        self.last_torques = (0.0, 0.0)
        self._policy_ticks = self.policy.control_interval - 1
        self._desired_torques = (0.0, 0.0)
        self._learning_previous_state = None
        self._learning_previous_torques = (0.0, 0.0)
        data.ctrl[:] = 0.0
        self.estimator.reset()
        self.scenario.reset(clear_metrics=clear_metrics)
        self.log_buffer.clear()
        self.analyzer.reset()
        self.supervisor.reset()
        self.last_update = "controller reset"

    def raw_state(self, data: mujoco.MjData) -> RobotState:
        _, pitch, yaw = quaternion_to_euler(data.qpos[3:7])
        gyro = data.sensordata[
            self.model.sensor_adr[self.gyro_id] : self.model.sensor_adr[self.gyro_id] + 3
        ]
        velocity = data.sensordata[
            self.model.sensor_adr[self.velocity_id] : self.model.sensor_adr[self.velocity_id] + 3
        ]
        return RobotState(
            sim_time=float(data.time),
            x=float(data.qpos[0]),
            y=float(data.qpos[1]),
            forward_speed=float(velocity[0]),
            pitch=pitch,
            pitch_rate=float(gyro[1]),
            yaw=yaw,
            yaw_rate=float(gyro[2]),
            left_wheel_angle=float(data.qpos[self.left_qpos]),
            right_wheel_angle=float(data.qpos[self.right_qpos]),
            left_torque=self.last_torques[0],
            right_torque=self.last_torques[1],
        )

    def state(self, data: mujoco.MjData) -> RobotState:
        """Return a current raw snapshot without advancing estimator state."""
        return self.raw_state(data)

    def apply_tuning_update(self, update: TuningUpdate) -> None:
        """Analyzer may narrow targets, but cannot alter learned policy weights online."""
        if update.speed_limit is not None:
            self.supervisor.limits.max_speed = float(
                np.clip(update.speed_limit, 0.15, self.supervisor.default_limits.max_speed)
            )
        if update.yaw_rate_limit is not None:
            self.supervisor.limits.max_yaw_rate = float(
                np.clip(update.yaw_rate_limit, 0.15, self.supervisor.default_limits.max_yaw_rate)
            )
        self.last_update = update.reason

    @staticmethod
    def _policy_state(state: RobotState) -> np.ndarray:
        return np.asarray(
            (state.pitch, state.pitch_rate, state.forward_speed, state.yaw_rate), dtype=float
        )

    def _begin_learning_episode(self, data: mujoco.MjData) -> None:
        """Reset into a small perturbation so exploration is visible in the viewer."""
        if self.learning_session is None:
            return
        self.reset(data, clear_metrics=False)
        pitch = self.learning_session.begin_episode()
        half_pitch = pitch / 2.0
        data.qpos[3:7] = (math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0)
        mujoco.mj_forward(self.model, data)
        self._policy_ticks = self.learning_session.control_interval - 1
        self.last_update = self.learning_session.phase

    def start_visible_learning(self, data: mujoco.MjData) -> None:
        """Start an online, episode-based learning run that is visible in MuJoCo."""
        self.learning_session = OnlineLQRSession(
            timestep=float(self.model.opt.timestep),
            control_interval=self.policy.control_interval,
        )
        self._last_online_learning_summary = None
        self._begin_learning_episode(data)
        self.last_update = "visible online learning started"

    def stop_visible_learning(self) -> None:
        """Keep the newest valid online policy, if any, and resume normal control."""
        if self.learning_session is not None and self.learning_session.policy is not None:
            self.policy = self.learning_session.policy
            self.last_update = "kept the latest online-learned LQR policy"
        elif self.learning_session is not None:
            self.last_update = "online learning stopped before a valid policy was found"
        self.learning_session = None
        self._learning_previous_state = None

    def learning_summary(self, language: str = "en") -> str:
        session = self.learning_session
        if session is None:
            if self._last_online_learning_summary is not None:
                if language == "en":
                    report = self.policy.report
                    return (
                        f"Online learning complete: {report.samples} measured transitions, "
                        f"spectral radius {report.spectral_radius:.3f}"
                    )
                return self._last_online_learning_summary
            report = self.policy.report
            return bilingual(
                language,
                f"离线 LQR：{report.samples} 条采样，谱半径 {report.spectral_radius:.3f}",
                f"Offline LQR: {report.samples} samples, spectral radius {report.spectral_radius:.3f}",
            )
        progress = session.progress
        radius = "--" if progress.spectral_radius is None else f"{progress.spectral_radius:.3f}"
        return bilingual(
            language,
            f"在线学习：第 {progress.episodes} 回合，{progress.samples} 条转移，"
            f"更新 {progress.policy_updates} 次，谱半径 {radius}\n{progress.phase}",
            f"Online learning: episode {progress.episodes}, {progress.samples} transitions, "
            f"{progress.policy_updates} updates, spectral radius {radius}\n"
            f"{localized_learning_phase(progress.phase, language)}",
        )

    def _advance_learning_episode(self, data: mujoco.MjData, reason: str) -> RobotState:
        """Use accumulated live data to update, then visibly restart the next rollout."""
        assert self.learning_session is not None
        self.learning_session.phase = reason
        self.learning_session.update_policy()
        self._begin_learning_episode(data)
        return self.state(data)

    def step(self, data: mujoco.MjData) -> RobotState:
        """Estimate state, enforce safety, tune from history, and write motor torque."""
        self.scenario.prepare_step(data)
        truth = self.raw_state(data)
        self.scenario.observe_metrics(truth)
        observed = self.scenario.observe(truth)
        state = self.estimator.update(observed)
        session = self.learning_session
        policy_state = self._policy_state(state)
        self.supervisor.enforce(truth, self.command)

        self._policy_ticks += 1
        if self._policy_ticks >= self.policy.control_interval:
            if session is not None:
                if self._learning_previous_state is not None:
                    session.record_transition(
                        self._learning_previous_state,
                        self._learning_previous_torques,
                        policy_state,
                    )
                if session.should_finish_episode(policy_state):
                    return self._advance_learning_episode(
                        data, "当前回合结束，利用真实轨迹进行在线辨识"
                    )
                if session.complete and session.policy is not None:
                    self.policy = session.policy
                    self.last_update = session.phase
                    progress = session.progress
                    self._last_online_learning_summary = (
                        f"在线学习完成：第 {progress.episodes} 回合，"
                        f"{progress.samples} 条真实转移，更新 {progress.policy_updates} 次，"
                        f"谱半径 {progress.spectral_radius:.3f}"
                    )
                    self.learning_session = None
                    session = None
                else:
                    self._desired_torques = session.choose_action(
                        policy_state, self.supervisor.limits.max_torque
                    )
                    self._learning_previous_state = policy_state.copy()
            self._policy_ticks = 0
            if session is None:
                self._desired_torques = self.policy.act(
                    policy_state,
                    self.command.forward_speed,
                    self.command.yaw_rate,
                    self.supervisor.limits.max_torque,
                )
        actual_torques = self.scenario.delay_motors(self._desired_torques)
        data.ctrl[0] = actual_torques[0]
        data.ctrl[1] = actual_torques[1]
        self.last_torques = actual_torques
        if session is not None:
            self._learning_previous_torques = actual_torques
        state = replace(
            state,
            left_torque=actual_torques[0],
            right_torque=actual_torques[1],
        )
        self.log_buffer.append(state, self.command)

        update = self.analyzer.analyze(self.log_buffer.frames(), self.supervisor.limits)
        if update is not None:
            self.apply_tuning_update(update)
        return state


class Harness:
    """Natural-language command layer, keyboard fallback, and status renderer."""

    speed_step = 0.15
    yaw_step = 0.25

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        controller: BalanceController,
        language: str = "en",
    ) -> None:
        self.model = model
        self.data = data
        self.controller = controller
        self.set_language(language)

    def set_language(self, language: str) -> None:
        if language not in SUPPORTED_LANGUAGES:
            raise ValueError(f"unsupported language: {language}")
        self.language = language

    def tr(self, chinese: str, english: str) -> str:
        return bilingual(self.language, chinese, english)

    def reset(self) -> None:
        self.controller.reset(self.data)

    @staticmethod
    def _number_after(text: str, phrases: tuple[str, ...], default: float) -> float:
        number_pattern = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
        for phrase in phrases:
            search_from = 0
            while True:
                phrase_start = text.find(phrase, search_from)
                if phrase_start < 0:
                    break
                tail = text[phrase_start + len(phrase) :]
                # Connector words ("at", "set to", "设置为") and units are
                # accepted, but a number from the next command is not borrowed.
                boundary = re.search(r"\band\b|并且|并|同时|[，,;；|]", tail)
                clause = tail if boundary is None else tail[: boundary.start()]
                match = re.search(number_pattern, clause)
                if match:
                    return float(match.group(0))
                search_from = phrase_start + len(phrase)
        return default

    def _set_speed(self, requested: float) -> str:
        command = self.controller.command
        limit = self.controller.supervisor.limits.max_speed
        applied = float(np.clip(requested, -limit, limit))
        command.forward_speed = applied
        if not math.isclose(requested, applied, rel_tol=0.0, abs_tol=1e-9):
            return self.tr(
                f"请求速度 {requested:+.2f} m/s；受当前限幅 ±{limit:.2f} m/s 影响，"
                f"实际写入目标为 {applied:+.2f} m/s（测量速度会动态跟随）。",
                f"Requested speed {requested:+.2f} m/s; the current ±{limit:.2f} m/s limit "
                f"clipped the applied target to {applied:+.2f} m/s (measured speed follows dynamically).",
            )
        return self.tr(
            f"请求速度 {requested:+.2f} m/s；实际写入目标为 {applied:+.2f} m/s"
            "（测量速度会动态跟随）。",
            f"Requested speed {requested:+.2f} m/s; applied target is {applied:+.2f} m/s "
            "(measured speed follows dynamically).",
        )

    def _set_yaw_rate(self, requested: float) -> str:
        command = self.controller.command
        limit = self.controller.supervisor.limits.max_yaw_rate
        applied = float(np.clip(requested, -limit, limit))
        command.yaw_rate = applied
        if not math.isclose(requested, applied, rel_tol=0.0, abs_tol=1e-9):
            return self.tr(
                f"请求转向速率 {requested:+.2f} rad/s；受当前限幅 ±{limit:.2f} rad/s 影响，"
                f"实际写入目标为 {applied:+.2f} rad/s（测量值会动态跟随）。",
                f"Requested yaw rate {requested:+.2f} rad/s; the current ±{limit:.2f} rad/s limit "
                f"clipped the applied target to {applied:+.2f} rad/s (measured value follows dynamically).",
            )
        return self.tr(
            f"请求转向速率 {requested:+.2f} rad/s；实际写入目标为 {applied:+.2f} rad/s"
            "（测量值会动态跟随）。",
            f"Requested yaw rate {requested:+.2f} rad/s; applied target is {applied:+.2f} rad/s "
            "(measured value follows dynamically).",
        )

    def execute_instruction(self, text: str) -> str:
        """Interpret compact Chinese/English motion instructions deterministically."""
        normalized = text.strip().lower()
        if not normalized:
            return self.tr(
                "空指令，未改变控制目标。",
                "Empty instruction; control targets were not changed.",
            )
        if any(word in normalized for word in ("帮助", "help", "指令列表")):
            return self.help_text()
        if any(word in normalized for word in ("重置", "reset", "重新开始")):
            self.reset()
            return self.tr(
                "已重置机器人、状态估计器和短时日志。",
                "Robot, state estimator, and short-window log were reset.",
            )
        if any(word in normalized for word in ("状态", "status", "state")):
            return self.format_state(self.controller.state(self.data))
        if any(word in normalized for word in ("停止", "停下", "stop", "halt")):
            command = self.controller.command
            command.forward_speed = 0.0
            command.yaw_rate = 0.0
            command.reference_x = float(self.data.qpos[0])
            return self.tr(
                "已停止；当前位置已设为保持参考点。",
                "Stopped; the current position is now the hold reference.",
            )
        if any(word in normalized for word in ("稳定", "平衡", "stable", "balance")):
            self.controller.supervisor.set_stability_mode()
            command = self.controller.command
            command.forward_speed = 0.0
            command.yaw_rate = 0.0
            command.reference_x = float(self.data.qpos[0])
            return self.tr(
                "已进入稳定模式：速度上限 0.75 m/s，转向上限 0.90 rad/s。",
                "Stability mode enabled: speed limit 0.75 m/s and yaw-rate limit 0.90 rad/s.",
            )

        responses: list[str] = []
        forward_words = ("前进", "forward", "go")
        backward_words = ("后退", "backward", "reverse")
        left_words = ("左转", "turn left", "left")
        right_words = ("右转", "turn right", "right")
        if any(word in normalized for word in forward_words):
            amount = abs(self._number_after(normalized, forward_words, 0.30))
            responses.append(self._set_speed(amount))
        elif any(word in normalized for word in backward_words):
            amount = abs(self._number_after(normalized, backward_words, 0.30))
            responses.append(self._set_speed(-amount))
        elif "速度" in normalized or "speed" in normalized:
            amount = self._number_after(normalized, ("速度", "speed"), 0.0)
            responses.append(self._set_speed(amount))

        if any(word in normalized for word in left_words):
            amount = abs(self._number_after(normalized, left_words, 0.40))
            responses.append(self._set_yaw_rate(amount))
        elif any(word in normalized for word in right_words):
            amount = abs(self._number_after(normalized, right_words, 0.40))
            responses.append(self._set_yaw_rate(-amount))
        elif "转向" in normalized or "yaw" in normalized:
            amount = self._number_after(normalized, ("转向", "yaw"), 0.0)
            responses.append(self._set_yaw_rate(amount))

        if responses:
            separator = "; " if self.language == "en" else "；"
            terminator = "." if self.language == "en" else "。"
            return separator.join(responses) + terminator
        return self.tr(
            "未识别指令。可输入：前进 0.3、后退 0.2、左转 0.4、停止、稳定、状态、重置、帮助。",
            "Instruction not recognized. Try: forward 0.3, backward 0.2, turn left 0.4, stop, balance, status, reset, or help.",
        )

    def handle_key(self, keycode: int) -> None:
        """Official viewer callback: arrow keys change only robot targets."""
        command = self.controller.command
        if keycode == glfw.KEY_UP:
            self._set_speed(command.forward_speed + self.speed_step)
        elif keycode == glfw.KEY_DOWN:
            self._set_speed(command.forward_speed - self.speed_step)
        elif keycode == glfw.KEY_LEFT:
            self._set_yaw_rate(command.yaw_rate + self.yaw_step)
        elif keycode == glfw.KEY_RIGHT:
            self._set_yaw_rate(command.yaw_rate - self.yaw_step)

    def help_text(self) -> str:
        return self.tr(
            "运动：前进/forward 0.3，后退/backward 0.2，速度/speed -0.2，"
            "左转/turn left 0.4，右转/turn right 0.4，转向/yaw -0.3。\n"
            "系统：停止/stop，稳定/balance，状态/status，重置/reset，帮助/help。\n"
            "键盘：↑/↓ 每次增加/减少 0.15 m/s，←/→ 每次增加/减少 0.25 rad/s。",
            "Motion: forward/前进 0.3, backward/后退 0.2, speed/速度 -0.2, "
            "turn left/左转 0.4, turn right/右转 0.4, yaw/转向 -0.3.\n"
            "System: stop/停止, balance/稳定, status/状态, reset/重置, help/帮助.\n"
            "Keyboard: ↑/↓ changes speed by 0.15 m/s; ←/→ changes yaw rate by 0.25 rad/s.",
        )

    def format_state(self, state: RobotState, display_time: float | None = None) -> str:
        command = self.controller.command
        supervisor = self.controller.supervisor
        shown_time = state.sim_time if display_time is None else display_time
        summary = localized_analyzer_summary(
            self.controller.analyzer.last_summary, self.language
        )
        if self.language == "en":
            return (
                f"t {shown_time:7.2f}s | {supervisor.mode:10s} | "
                f"position ({state.x:+6.2f}, {state.y:+6.2f}) m | "
                f"speed {state.forward_speed:+5.2f}/{command.forward_speed:+5.2f} m/s | "
                f"pitch {math.degrees(state.pitch):+6.2f} deg "
                f"({math.degrees(state.pitch_rate):+6.1f} deg/s) | "
                f"yaw {math.degrees(state.yaw):+6.1f} deg | "
                f"yaw rate {state.yaw_rate:+5.2f}/{command.yaw_rate:+5.2f} rad/s | "
                f"torque {state.left_torque:+4.2f}/{state.right_torque:+4.2f} Nm | "
                f"window: {summary}"
            )
        return (
            f"时间 {shown_time:7.2f}s | 模式 {localized_mode(supervisor.mode, self.language)} | "
            f"位置 ({state.x:+6.2f}, {state.y:+6.2f}) m | "
            f"速度 {state.forward_speed:+5.2f}/{command.forward_speed:+5.2f} m/s | "
            f"俯仰 {math.degrees(state.pitch):+6.2f}° "
            f"({math.degrees(state.pitch_rate):+6.1f}°/s) | "
            f"航向 {math.degrees(state.yaw):+6.1f}° | "
            f"转向速率 {state.yaw_rate:+5.2f}/{command.yaw_rate:+5.2f} rad/s | "
            f"力矩 {state.left_torque:+4.2f}/{state.right_torque:+4.2f} N·m | "
            f"窗口：{summary}"
        )


class InstructionPanel:
    """In-process controls for commands, disturbances, and robustness tests."""

    def __init__(self, harness: Harness) -> None:
        self.harness = harness
        self.scenario = harness.controller.scenario
        self.closed = False
        self.root = tk.Tk()
        for font_name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkFixedFont"):
            tkfont.nametofont(font_name).configure(
                family="Microsoft YaHei UI", size=11
            )
        self.root.geometry("1040x940")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.scale_labels: dict[str, tk.Label] = {}

        language_row = tk.Frame(self.root)
        language_row.pack(fill="x", padx=10, pady=(10, 0))
        tk.Label(language_row, text="Languages", width=12, anchor="w").pack(side="left")
        initial_option = "English" if self.harness.language == "en" else "中文"
        self.language_var = tk.StringVar(value=initial_option)
        self.language_menu = tk.OptionMenu(
            language_row,
            self.language_var,
            *LANGUAGE_OPTIONS.keys(),
            command=self.set_language,
        )
        self.language_menu.configure(width=10)
        self.language_menu.pack(side="left")

        self.command_box = tk.LabelFrame(self.root, padx=10, pady=8)
        self.command_box.pack(fill="x", padx=10, pady=(6, 5))
        command_left = tk.Frame(self.command_box)
        command_left.pack(side="left", fill="both", expand=True)
        self.command_reference_box = tk.LabelFrame(self.command_box, padx=8, pady=5)
        self.command_reference_box.pack(side="right", fill="y", padx=(12, 0))
        self.command_reference = tk.Label(
            self.command_reference_box,
            justify="left",
            anchor="nw",
            font=("Microsoft YaHei UI", 11),
        )
        self.command_reference.pack(fill="both", expand=True)
        self.command_example = tk.Label(command_left)
        self.command_example.pack(anchor="w")
        row = tk.Frame(command_left)
        row.pack(fill="x", pady=(6, 0))
        self.entry = tk.Entry(row, font=("Microsoft YaHei UI", 12))
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", self.submit)
        self.send_button = tk.Button(row, command=self.submit, width=9)
        self.send_button.pack(side="left", padx=(8, 0))
        self.result = tk.StringVar()
        tk.Label(
            command_left,
            textvariable=self.result,
            justify="left",
            anchor="w",
            wraplength=570,
            fg="#1f3b6d",
        ).pack(fill="x", pady=(6, 0))

        self.experiment_box = tk.LabelFrame(self.root, padx=10, pady=8)
        self.experiment_box.pack(fill="x", padx=10, pady=5)
        self._add_scale(
            self.experiment_box,
            "friction",
            "地面与轮胎摩擦",
            "Ground / tire friction",
            self.scenario.config.friction,
            0.20,
            2.50,
            0.05,
            self.scenario.set_friction,
        )
        self._add_scale(
            self.experiment_box,
            "slope",
            "地面坡度 (deg)",
            "Ground slope (deg)",
            self.scenario.config.slope_deg,
            -10.0,
            10.0,
            0.5,
            self.scenario.set_slope,
        )
        self._add_scale(
            self.experiment_box,
            "imu_noise",
            "IMU 噪声 (deg/s)",
            "IMU noise (deg/s)",
            self.scenario.config.imu_noise_deg_s,
            0.0,
            10.0,
            0.1,
            self.scenario.set_imu_noise,
        )
        self._add_scale(
            self.experiment_box,
            "imu_delay",
            "IMU 延迟 (ms)",
            "IMU delay (ms)",
            self.scenario.config.imu_delay_ms,
            0.0,
            120.0,
            5.0,
            self.scenario.set_imu_delay,
        )
        self._add_scale(
            self.experiment_box,
            "motor_delay",
            "电机延迟 (ms)",
            "Motor delay (ms)",
            self.scenario.config.motor_delay_ms,
            0.0,
            120.0,
            5.0,
            self.scenario.set_motor_delay,
        )

        self.action_box = tk.LabelFrame(self.root, padx=10, pady=8)
        self.action_box.pack(fill="x", padx=10, pady=5)
        row = tk.Frame(self.action_box)
        row.pack(fill="x")
        self.push_forward_button = tk.Button(row, command=lambda: self.push(25.0))
        self.push_forward_button.pack(side="left", padx=(0, 6))
        self.push_backward_button = tk.Button(row, command=lambda: self.push(-25.0))
        self.push_backward_button.pack(side="left", padx=6)
        self.clear_metrics_button = tk.Button(row, command=self.clear_metrics)
        self.clear_metrics_button.pack(side="left", padx=6)
        second_row = tk.Frame(self.action_box)
        second_row.pack(fill="x", pady=(6, 0))
        self.reset_button = tk.Button(second_row, command=self.reset_experiment)
        self.reset_button.pack(side="left")
        learning_row = tk.Frame(self.action_box)
        learning_row.pack(fill="x", pady=(6, 0))
        self.start_learning_button = tk.Button(
            learning_row, command=self.relearn_current_situation
        )
        self.start_learning_button.pack(side="left")
        self.stop_learning_button = tk.Button(learning_row, command=self.stop_visible_learning)
        self.stop_learning_button.pack(side="left", padx=6)
        self.camera_button = tk.Button(row, command=self.toggle_camera_follow)
        self.camera_button.pack(side="right")
        self._update_camera_button()

        self.metrics_box = tk.LabelFrame(self.root, padx=10, pady=8)
        self.metrics_box.pack(fill="both", expand=True, padx=10, pady=(5, 10))
        self.state = tk.StringVar()
        self.metrics_text = tk.StringVar()
        tk.Label(self.metrics_box, textvariable=self.state, justify="left", anchor="w").pack(
            fill="x", pady=(0, 6)
        )
        tk.Label(
            self.metrics_box,
            textvariable=self.metrics_text,
            justify="left",
            anchor="nw",
            fg="#245b32",
        ).pack(fill="both", expand=True)
        self._apply_language(initial=True)
        self.entry.focus_force()

    def _add_scale(
        self,
        parent: tk.Misc,
        key: str,
        chinese_label: str,
        english_label: str,
        initial: float,
        lower: float,
        upper: float,
        resolution: float,
        callback: Callable[[float], None],
    ) -> None:
        row = tk.Frame(parent)
        row.pack(fill="x", pady=1)
        label = tk.Label(row, width=22, anchor="w")
        label.pack(side="left")
        label.localized_text = (chinese_label, english_label)
        self.scale_labels[key] = label
        value = tk.DoubleVar(value=initial)
        tk.Scale(
            row,
            from_=lower,
            to=upper,
            resolution=resolution,
            orient="horizontal",
            variable=value,
            command=lambda raw: callback(float(raw)),
            length=290,
            showvalue=True,
        ).pack(side="left", fill="x", expand=True)

    def _t(self, chinese: str, english: str) -> str:
        return bilingual(self.harness.language, chinese, english)

    def set_language(self, selection: str) -> None:
        language = LANGUAGE_OPTIONS.get(selection, "zh")
        self.harness.set_language(language)
        self._apply_language()
        self.result.set(
            self._t("语言已切换为中文。", "Language switched to English.")
        )

    def _apply_language(self, initial: bool = False) -> None:
        self.root.title(self._t("BalanceBot 仿真实验台", "BalanceBot Simulation Workbench"))
        self.command_box.configure(text=self._t("自然语言指令", "Natural-language commands"))
        self.command_example.configure(
            text=self._t(
                "示例：前进 0.3｜左转 0.4｜停止｜稳定｜状态｜重置",
                "Examples: forward 0.3 | turn left 0.4 | stop | balance | status | reset",
            )
        )
        self.send_button.configure(text=self._t("发送", "Send"))
        self.command_reference_box.configure(
            text=self._t("所有可用指令", "All available commands")
        )
        self._update_command_reference()
        self.experiment_box.configure(
            text=self._t("抗扰动实验参数", "Robustness test parameters")
        )
        for label in self.scale_labels.values():
            chinese, english = label.localized_text
            label.configure(text=self._t(chinese, english))
        self.action_box.configure(
            text=self._t("交互扰动与视角", "Interactive disturbances and view")
        )
        self.push_forward_button.configure(
            text=self._t("轻推前方 (+25 N)", "Forward push (+25 N)")
        )
        self.push_backward_button.configure(
            text=self._t("轻推后方 (-25 N)", "Backward push (-25 N)")
        )
        self.clear_metrics_button.configure(text=self._t("清零指标", "Clear metrics"))
        self.reset_button.configure(text=self._t("重置实验", "Reset experiment"))
        self.start_learning_button.configure(
            text=self._t("重新学习当前环境", "Relearning current situation")
        )
        self.stop_learning_button.configure(
            text=self._t("停止重新学习", "Stop relearning")
        )
        self.metrics_box.configure(
            text=self._t("实时稳定性指标", "Real-time stability metrics")
        )
        self._update_camera_button()
        if initial:
            self.result.set(self._t("准备就绪。", "Ready."))
            self.state.set(self._t("等待仿真状态…", "Waiting for simulation state..."))
            self.metrics_text.set(self._t("等待实验数据…", "Waiting for experiment data..."))

    def _update_camera_button(self) -> None:
        if self.scenario.config.camera_follow:
            label = self._t("摄像头跟随：开", "Camera follow: On")
        else:
            label = self._t("摄像头跟随：关", "Camera follow: Off")
        self.camera_button.configure(text=label)

    def _update_command_reference(self) -> None:
        limits = self.harness.controller.supervisor.limits
        commands = (
            "forward / go / 前进 | backward / reverse / 后退 <m/s>\n"
            "speed / 速度 <signed m/s>\n"
            "turn left / left / 左转 | turn right / right / 右转 <rad/s>\n"
            "yaw / 转向 <signed rad/s>\n"
            "stop / halt / 停止 / 停下 | balance / stable / 稳定 / 平衡\n"
            "status / state / 状态 | reset / 重置 / 重新开始\n"
            "help / 帮助 / 指令列表\n"
            "↑ ↓: speed    ← →: yaw\n"
        )
        limit_text = self._t(
            f"当前限幅：速度 ±{limits.max_speed:.2f} m/s；转向 ±{limits.max_yaw_rate:.2f} rad/s",
            f"Current limits: speed ±{limits.max_speed:.2f} m/s; yaw ±{limits.max_yaw_rate:.2f} rad/s",
        )
        self.command_reference.configure(text=commands + limit_text)

    def toggle_camera_follow(self) -> None:
        self.scenario.config.camera_follow = not self.scenario.config.camera_follow
        self._update_camera_button()
        if self.scenario.config.camera_follow:
            self.result.set(self._t("摄像头已开始跟随机器人。", "Camera follow enabled."))
        else:
            self.result.set(self._t("摄像头跟随已关闭。", "Camera follow disabled."))

    def push(self, force_x: float) -> None:
        self.scenario.schedule_push(float(self.harness.data.time), force_x)
        self.result.set(
            self._t(
                f"已施加 {force_x:+.0f} N、持续 0.15 s 的水平推力。",
                f"Applied a {force_x:+.0f} N horizontal push for 0.15 s.",
            )
        )

    def clear_metrics(self) -> None:
        self.scenario.metrics.clear()
        self.result.set(self._t("稳定性指标已清零。", "Stability metrics cleared."))

    def reset_experiment(self) -> None:
        self.harness.reset()
        self.result.set(
            self._t(
                "已重置机器人与实验指标；当前地面、传感器和电机参数保留。",
                "Robot and experiment metrics reset; current terrain, sensor, and motor settings were preserved.",
            )
        )

    def relearn_current_situation(self) -> None:
        self.harness.controller.start_visible_learning(self.harness.data)
        self.result.set(
            self._t(
                "当前环境重新学习已开始：请观察探索、回合重启、辨识和稳定验证过程。",
                "Relearning current situation started: observe exploration, episode restarts, identification, and stability validation.",
            )
        )

    def stop_visible_learning(self) -> None:
        self.harness.controller.stop_visible_learning()
        self.result.set(
            self._t(
                "重新学习已停止；若已得到有效策略，会继续使用最新策略。",
                "Relearning stopped; the latest valid policy remains active if one was found.",
            )
        )

    def submit(self, _event: object | None = None) -> None:
        if self.closed:
            return
        text = self.entry.get().strip()
        if text:
            self.result.set(self.harness.execute_instruction(text))
            self.entry.delete(0, tk.END)

    def update(self, state: RobotState) -> None:
        if self.closed:
            return
        metrics = self.scenario.metrics
        recovery = "--" if metrics.last_recovery_s is None else f"{metrics.last_recovery_s:.2f} s"
        command = self.harness.controller.command
        self._update_command_reference()
        if self.harness.language == "en":
            self.state.set(
                f"t={state.sim_time:.1f}s | pitch={math.degrees(state.pitch):+.2f} deg | "
                f"speed measured/target={state.forward_speed:+.2f}/{command.forward_speed:+.2f} m/s | "
                f"yaw measured/target={state.yaw_rate:+.2f}/{command.yaw_rate:+.2f} rad/s | "
                f"mode={self.harness.controller.supervisor.mode}"
            )
            self.metrics_text.set(
                f"Maximum pitch: {metrics.max_pitch_deg:.2f} deg\n"
                f"Maximum speed: {metrics.max_speed:.2f} m/s\n"
                f"Latest recovery time: {recovery}\n"
                f"Recoveries: {metrics.recovery_count}\n"
                f"Current slope: {self.scenario.config.slope_deg:+.1f} deg    "
                f"Friction: {self.scenario.config.friction:.2f}\n"
                f"IMU noise / delay: {self.scenario.config.imu_noise_deg_s:.1f} deg/s / "
                f"{self.scenario.config.imu_delay_ms:.0f} ms    "
                f"Motor delay: {self.scenario.config.motor_delay_ms:.0f} ms"
                f"\n{self.harness.controller.learning_summary('en')}"
            )
        else:
            self.state.set(
                f"t={state.sim_time:.1f}s | 俯仰={math.degrees(state.pitch):+.2f}° | "
                f"速度 测量/目标={state.forward_speed:+.2f}/{command.forward_speed:+.2f} m/s | "
                f"转向 测量/目标={state.yaw_rate:+.2f}/{command.yaw_rate:+.2f} rad/s | "
                f"模式={localized_mode(self.harness.controller.supervisor.mode, 'zh')}"
            )
            self.metrics_text.set(
                f"最大俯仰：{metrics.max_pitch_deg:.2f}°\n"
                f"最大速度：{metrics.max_speed:.2f} m/s\n"
                f"最近恢复时间：{recovery}\n"
                f"恢复次数：{metrics.recovery_count}\n"
                f"当前坡度：{self.scenario.config.slope_deg:+.1f}°    "
                f"摩擦：{self.scenario.config.friction:.2f}\n"
                f"IMU 噪声/延迟：{self.scenario.config.imu_noise_deg_s:.1f} deg/s / "
                f"{self.scenario.config.imu_delay_ms:.0f} ms    "
                f"电机延迟：{self.scenario.config.motor_delay_ms:.0f} ms"
                f"\n{self.harness.controller.learning_summary('zh')}"
            )
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            self.root.destroy()


def simulate(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: BalanceController,
    status: Callable[[RobotState], None],
    duration: float | None,
    realtime: bool,
) -> None:
    """Run an optionally bounded headless simulation."""
    wall_start = time.perf_counter()
    status_ticker = ResetAwareTicker(0.25)
    while duration is None or data.time < duration:
        state = controller.step(data)
        mujoco.mj_step(model, data)
        if status_ticker.is_due(float(data.time)):
            status(state)
        if realtime:
            remaining = data.time - (time.perf_counter() - wall_start)
            if remaining > 0.0:
                time.sleep(remaining)


def run_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    harness: Harness,
    controller: BalanceController,
) -> None:
    """Run the official viewer plus an in-process graphical command panel."""
    panel = InstructionPanel(harness)
    overlay_ticker = ResetAwareTicker(0.20)
    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=harness.handle_key,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            with viewer.lock():
                viewer.cam.distance = 2.2
                viewer.cam.azimuth = 120
                viewer.cam.elevation = -18
            while viewer.is_running():
                step_start = time.perf_counter()
                state = controller.step(data)
                mujoco.mj_step(model, data)
                if controller.scenario.config.camera_follow:
                    with viewer.lock():
                        target = np.array((data.qpos[0], data.qpos[1], 0.25))
                        viewer.cam.lookat[:] = 0.88 * viewer.cam.lookat + 0.12 * target
                viewer.sync()
                panel.update(state)
                if overlay_ticker.is_due(float(data.time)):
                    if harness.language == "en":
                        overlay_title = "BalanceBot  |  Arrow keys control robot"
                        overlay_body = (
                            "Use the BalanceBot command panel to type instructions\n"
                            + harness.format_state(
                                state, display_time=overlay_ticker.elapsed_time
                            )
                        )
                    else:
                        overlay_title = "BalanceBot  |  方向键控制机器人"
                        overlay_body = (
                            "请在 BalanceBot 指令面板中输入指令\n"
                            + harness.format_state(
                                state, display_time=overlay_ticker.elapsed_time
                            )
                        )
                    viewer.set_texts(
                        (
                            None,
                            None,
                            overlay_title,
                            overlay_body,
                        )
                    )
                remaining = model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        panel.close()
    print(
        bilingual(
            harness.language,
            f"查看器已关闭，累计仿真 {data.time:.2f} s。",
            f"Viewer closed after {data.time:.2f}s simulated.",
        )
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run without the MuJoCo OpenGL viewer")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="headless duration in seconds (default: 10; ignored by viewer)",
    )
    parser.add_argument("--no-realtime", action="store_true", help="do not pace headless simulation")
    parser.add_argument(
        "--relearn-current-situation",
        "--visible-learning",
        dest="visible_learning",
        action="store_true",
        help="relearn the current situation from visible, episode-based trajectories",
    )
    parser.add_argument(
        "--command",
        action="append",
        default=[],
        help="initial natural-language instruction; may be passed more than once",
    )
    parser.add_argument(
        "--language",
        choices=SUPPORTED_LANGUAGES,
        default="en",
        help="interface language: zh or en (default: en)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    controller = BalanceController(model)
    harness = Harness(model, data, controller, language=args.language)
    harness.reset()
    for instruction in args.command:
        label = "Initial command" if harness.language == "en" else "初始指令"
        print(f"[{label}] {harness.execute_instruction(instruction)}")
    if args.visible_learning:
        controller.start_visible_learning(data)
        print(
            bilingual(
                harness.language,
                "[重新学习当前环境] 已开始从环境中的真实轨迹辨识控制策略。",
                "[Relearning current situation] Started identifying a control policy from measured environment trajectories.",
            )
        )

    if args.headless:
        print(harness.help_text())
        simulate(
            model,
            data,
            controller,
            lambda state: print(harness.format_state(state)),
            duration=args.duration,
            realtime=not args.no_realtime,
        )
    else:
        run_viewer(model, data, harness, controller)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(130)
