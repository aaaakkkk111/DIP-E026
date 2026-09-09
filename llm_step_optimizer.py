#!/usr/bin/env python3
"""Free-control BalanceBot recorder and structured tuning interface.

This entry point deliberately leaves all existing project files untouched.  It
loads the original flat-ground model and preserves arrow-key control, natural
language commands, camera follow, pushes, and disturbance sliders.  Operational
speed/pitch/yaw limits and automatic fall reset are disabled; motor torque
remains a finite, tunable physical actuator parameter.

Each recording session produces machine-readable telemetry, events, parameter
and environment snapshots, MuJoCo camera keyframes, and high-resolution plots.
The generated ``llm_input.json`` is the compact hand-off artifact for a later
human/LLM tuning pass.  New parameters are applied only by loading the single
structured ``controller_params.json`` file.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Deque

import glfw
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import mujoco
import mujoco.viewer
import numpy as np

import balance_bot as base
from learned_lqr import LearningReport


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "balance_bot.xml"
DEFAULT_PARAMETER_PATH = ROOT / "controller_params.json"
DEFAULT_OUTPUT_ROOT = ROOT / "experiments"


EMPTY_PARAMETER_DOCUMENT: dict[str, Any] = {}


# This is a schema/example for the first tuning response.  It is deliberately
# not written to controller_params.json at startup: an empty object means that
# no controller has been configured yet.
PARAMETER_TEMPLATE: dict[str, Any] = {
    "schema_version": 1,
    "revision": 1,
    "notes": "Edit numeric values, save this file, then click '导入参数 JSON'.",
    "controller": {
        "q_pitch": 75.0,
        "q_pitch_rate": 4.0,
        "q_forward_speed": 6.0,
        "q_yaw_rate": 1.5,
        "r_common_torque": 0.22,
        "r_differential_torque": 0.35,
        "motor_torque_limit_nm": 4.0,
        "control_interval_steps": 10,
        "estimator_alpha": 0.82,
        "identification_samples": 800,
        "identification_seed": 7,
    },
    "initial_command": {
        "forward_speed_m_s": 0.0,
        "yaw_rate_rad_s": 0.0,
    },
    "environment": {
        "friction": 1.5,
        "slope_deg": 0.0,
        "imu_noise_deg_s": 0.0,
        "imu_delay_ms": 0.0,
        "motor_delay_ms": 0.0,
    },
    "recording": {
        "default_duration_s": 10.0,
        "sample_period_s": 0.01,
        "camera_period_s": 0.50,
        "realtime_plot_window_s": 10.0,
    },
}


@dataclass(frozen=True)
class ControllerParameters:
    q_pitch: float
    q_pitch_rate: float
    q_forward_speed: float
    q_yaw_rate: float
    r_common_torque: float
    r_differential_torque: float
    motor_torque_limit_nm: float
    control_interval_steps: int
    estimator_alpha: float
    identification_samples: int
    identification_seed: int


@dataclass(frozen=True)
class RecordingParameters:
    default_duration_s: float
    sample_period_s: float
    camera_period_s: float
    realtime_plot_window_s: float


@dataclass(frozen=True)
class AppConfiguration:
    document: dict[str, Any]
    controller: ControllerParameters | None
    initial_speed: float
    initial_yaw_rate: float
    environment: dict[str, float]
    recording: RecordingParameters


@dataclass(frozen=True)
class IdentifiedDynamics:
    a: np.ndarray
    b: np.ndarray
    equilibrium: np.ndarray
    samples: int
    controllability_rank: int


def json_write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def ensure_parameter_file(path: Path) -> None:
    if not path.exists():
        json_write(path, EMPTY_PARAMETER_DOCUMENT)


def finite_number(raw: Any, name: str, lower: float, upper: float) -> float:
    value = float(raw)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须位于 [{lower}, {upper}]，当前值为 {raw!r}")
    return value


def integer_value(raw: Any, name: str, lower: int, upper: int) -> int:
    value = int(raw)
    if value != float(raw) or not lower <= value <= upper:
        raise ValueError(f"{name} 必须是 [{lower}, {upper}] 内的整数，当前值为 {raw!r}")
    return value


def load_configuration(path: Path) -> AppConfiguration:
    content = path.read_text(encoding="utf-8").strip()
    raw = json.loads(content) if content else {}
    if raw == {}:
        environment = PARAMETER_TEMPLATE["environment"].copy()
        recording = PARAMETER_TEMPLATE["recording"]
        return AppConfiguration(
            document={},
            controller=None,
            initial_speed=0.0,
            initial_yaw_rate=0.0,
            environment=environment,
            recording=RecordingParameters(
                default_duration_s=float(recording["default_duration_s"]),
                sample_period_s=float(recording["sample_period_s"]),
                camera_period_s=float(recording["camera_period_s"]),
                realtime_plot_window_s=float(recording["realtime_plot_window_s"]),
            ),
        )
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("参数文件必须是空对象 {}，或 schema_version=1 的完整 JSON 对象")
    controller = raw["controller"]
    command = raw["initial_command"]
    environment_raw = raw["environment"]
    recording = raw["recording"]
    params = ControllerParameters(
        q_pitch=finite_number(controller["q_pitch"], "q_pitch", 0.001, 10_000.0),
        q_pitch_rate=finite_number(
            controller["q_pitch_rate"], "q_pitch_rate", 0.001, 10_000.0
        ),
        q_forward_speed=finite_number(
            controller["q_forward_speed"], "q_forward_speed", 0.001, 10_000.0
        ),
        q_yaw_rate=finite_number(controller["q_yaw_rate"], "q_yaw_rate", 0.001, 10_000.0),
        r_common_torque=finite_number(
            controller["r_common_torque"], "r_common_torque", 0.0001, 1_000.0
        ),
        r_differential_torque=finite_number(
            controller["r_differential_torque"],
            "r_differential_torque",
            0.0001,
            1_000.0,
        ),
        motor_torque_limit_nm=finite_number(
            controller["motor_torque_limit_nm"], "motor_torque_limit_nm", 0.01, 50.0
        ),
        control_interval_steps=integer_value(
            controller["control_interval_steps"], "control_interval_steps", 1, 100
        ),
        estimator_alpha=finite_number(
            controller["estimator_alpha"], "estimator_alpha", 0.0, 0.9999
        ),
        identification_samples=integer_value(
            controller["identification_samples"], "identification_samples", 100, 10_000
        ),
        identification_seed=integer_value(
            controller["identification_seed"], "identification_seed", 0, 2_147_483_647
        ),
    )
    environment = {
        "friction": finite_number(environment_raw["friction"], "friction", 0.20, 2.50),
        "slope_deg": finite_number(environment_raw["slope_deg"], "slope_deg", -10.0, 10.0),
        "imu_noise_deg_s": finite_number(
            environment_raw["imu_noise_deg_s"], "imu_noise_deg_s", 0.0, 10.0
        ),
        "imu_delay_ms": finite_number(
            environment_raw["imu_delay_ms"], "imu_delay_ms", 0.0, 120.0
        ),
        "motor_delay_ms": finite_number(
            environment_raw["motor_delay_ms"], "motor_delay_ms", 0.0, 120.0
        ),
    }
    return AppConfiguration(
        document=raw,
        controller=params,
        initial_speed=finite_number(
            command["forward_speed_m_s"], "forward_speed_m_s", -100.0, 100.0
        ),
        initial_yaw_rate=finite_number(
            command["yaw_rate_rad_s"], "yaw_rate_rad_s", -100.0, 100.0
        ),
        environment=environment,
        recording=RecordingParameters(
            default_duration_s=finite_number(
                recording["default_duration_s"], "default_duration_s", 0.1, 86_400.0
            ),
            sample_period_s=finite_number(
                recording["sample_period_s"], "sample_period_s", 0.002, 1.0
            ),
            camera_period_s=finite_number(
                recording["camera_period_s"], "camera_period_s", 0.02, 60.0
            ),
            realtime_plot_window_s=finite_number(
                recording["realtime_plot_window_s"], "realtime_plot_window_s", 1.0, 120.0
            ),
        ),
    )


class RobotAddresses:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self.velocity_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_velocity"
        )
        self.left_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")
        ]
        self.right_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")
        ]

    def compact_state(self, data: mujoco.MjData) -> np.ndarray:
        _, pitch, _ = base.quaternion_to_euler(data.qpos[3:7])
        gyro_adr = self.model.sensor_adr[self.gyro_id]
        velocity_adr = self.model.sensor_adr[self.velocity_id]
        gyro = data.sensordata[gyro_adr : gyro_adr + 3]
        velocity = data.sensordata[velocity_adr : velocity_adr + 3]
        return np.asarray((pitch, gyro[1], velocity[0], gyro[2]), dtype=float)


def solve_dare(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    p = q.copy()
    for _ in range(2_000):
        middle = r + b.T @ p @ b
        next_p = a.T @ p @ a - a.T @ p @ b @ np.linalg.solve(
            middle, b.T @ p @ a
        ) + q
        if np.max(np.abs(next_p - p)) < 1e-10:
            p = next_p
            break
        p = next_p
    return np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)


def identify_dynamics(
    model: mujoco.MjModel,
    addresses: RobotAddresses,
    samples: int,
    seed: int,
    control_interval: int,
) -> IdentifiedDynamics:
    rng = np.random.default_rng(seed)
    states: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    next_states: list[np.ndarray] = []
    data = mujoco.MjData(model)
    for _ in range(samples):
        mujoco.mj_resetData(model, data)
        pitch = rng.uniform(math.radians(-6.0), math.radians(6.0))
        half = pitch / 2.0
        data.qpos[3:7] = (math.cos(half), 0.0, math.sin(half), 0.0)
        data.qvel[4] = rng.uniform(-0.7, 0.7)
        speed = rng.uniform(-0.35, 0.35)
        data.qvel[0] = speed
        data.qvel[addresses.left_dof] = speed / 0.16 + rng.uniform(-0.2, 0.2)
        data.qvel[addresses.right_dof] = speed / 0.16 + rng.uniform(-0.2, 0.2)
        data.qvel[5] = rng.uniform(-0.4, 0.4)
        mujoco.mj_forward(model, data)
        state = addresses.compact_state(data)
        common = rng.uniform(-1.8, 1.8)
        differential = rng.uniform(-0.5, 0.5)
        data.ctrl[:] = (common - differential, common + differential)
        for _ in range(control_interval):
            mujoco.mj_step(model, data)
        states.append(state)
        actions.append(np.asarray((common, differential)))
        next_states.append(addresses.compact_state(data))

    x = np.asarray(states)
    u = np.asarray(actions)
    y = np.asarray(next_states)
    regressors = np.column_stack((x, u, np.ones(samples)))
    coefficients, *_ = np.linalg.lstsq(regressors, y, rcond=None)
    a = coefficients[:4].T
    b = coefficients[4:6].T
    offset = coefficients[-1]
    stationary = np.column_stack((np.eye(4) - a, -b))
    equilibrium, *_ = np.linalg.lstsq(stationary, offset, rcond=None)
    controllability = np.column_stack((b, a @ b, a @ a @ b, a @ a @ a @ b))
    return IdentifiedDynamics(
        a=a,
        b=b,
        equilibrium=equilibrium[:4],
        samples=samples,
        controllability_rank=int(np.linalg.matrix_rank(controllability)),
    )


class TunableLQRPolicy:
    """Same policy interface as the existing controller, with JSON-driven Q/R."""

    def __init__(self, dynamics: IdentifiedDynamics, parameters: ControllerParameters) -> None:
        self.control_interval = parameters.control_interval_steps
        q = np.diag(
            (
                parameters.q_pitch,
                parameters.q_pitch_rate,
                parameters.q_forward_speed,
                parameters.q_yaw_rate,
            )
        )
        r = np.diag((parameters.r_common_torque, parameters.r_differential_torque))
        self.gain = solve_dare(dynamics.a, dynamics.b, q, r)
        self.equilibrium = dynamics.equilibrium.copy()
        radius = float(np.max(np.abs(np.linalg.eigvals(dynamics.a - dynamics.b @ self.gain))))
        self.report = LearningReport(
            samples=dynamics.samples,
            spectral_radius=radius,
            controllability_rank=dynamics.controllability_rank,
        )

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
        return (
            float(np.clip(common - differential, -torque_limit, torque_limit)),
            float(np.clip(common + differential, -torque_limit, torque_limit)),
        )


class UnconfiguredPolicy:
    """Zero-output policy used while the parameter JSON is empty."""

    control_interval = 1
    gain = np.zeros((2, 4), dtype=float)
    report = LearningReport(samples=0, spectral_radius=0.0, controllability_rank=0)

    @staticmethod
    def act(
        _state: np.ndarray,
        _target_speed: float,
        _target_yaw_rate: float,
        _torque_limit: float,
    ) -> tuple[float, float]:
        return (0.0, 0.0)


class FreeSupervisor:
    """No command/pitch limits and no automatic reset; only physical torque remains."""

    def __init__(self, torque_limit: float) -> None:
        self.torque_limit = torque_limit
        self.mode = "FREE"
        self.reason = "speed, yaw and pitch safety restrictions disabled"
        self._refresh_limits()

    def _refresh_limits(self) -> None:
        self.default_limits = base.SafetyLimits(
            max_speed=math.inf,
            max_yaw_rate=math.inf,
            max_torque=self.torque_limit,
            warning_pitch=math.inf,
            reset_pitch=math.inf,
        )
        self.limits = replace(self.default_limits)

    def set_torque_limit(self, torque_limit: float) -> None:
        self.torque_limit = torque_limit
        self._refresh_limits()

    def reset(self) -> None:
        self.mode = "FREE"
        self.reason = "manual reset only"
        self._refresh_limits()

    def set_stability_mode(self) -> None:
        self.mode = "FREE-STABLE"
        self.reason = "targets stopped; no numerical safety limits were enabled"

    @staticmethod
    def clamp_command(_command: base.Command) -> None:
        return

    @staticmethod
    def enforce(_state: base.RobotState, _command: base.Command) -> bool:
        return False


class PassiveMonitor:
    last_summary = "passive recording only; learned gains are never changed automatically"

    def reset(self) -> None:
        return

    @staticmethod
    def analyze(_frames: Any, _limits: Any) -> None:
        return None


class FreeTunableController(base.BalanceController):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: AppConfiguration) -> None:
        # Do not call BalanceController.__init__: it constructs a ready-made
        # LearnedLQRPolicy, which would make an empty-JSON run balance before
        # the first user/LLM iteration.
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
        self.command = base.Command()
        self.last_torques = (0.0, 0.0)
        self.estimator = base.StateEstimator()
        self.scenario = base.SimulationScenario(model)
        self.policy = UnconfiguredPolicy()
        self._policy_ticks = 0
        self._desired_torques = (0.0, 0.0)
        self.learning_session = None
        self._learning_previous_state = None
        self._learning_previous_torques = (0.0, 0.0)
        self._last_online_learning_summary = None
        self.log_buffer = base.LogBuffer(seconds=3.0)
        self.addresses = RobotAddresses(model)
        self.current_config = config
        initial_torque_limit = (
            config.controller.motor_torque_limit_nm if config.controller is not None else 1.0
        )
        self.supervisor = FreeSupervisor(initial_torque_limit)
        self.analyzer = PassiveMonitor()
        self.install_configuration(data, config, set_initial_command=True)

    @property
    def is_configured(self) -> bool:
        return self.current_config.controller is not None

    def install_configuration(
        self,
        data: mujoco.MjData,
        config: AppConfiguration,
        set_initial_command: bool,
    ) -> None:
        params = config.controller
        self.scenario.set_friction(config.environment["friction"])
        self.scenario.set_slope(config.environment["slope_deg"])
        self.scenario.set_imu_noise(config.environment["imu_noise_deg_s"])
        self.scenario.set_imu_delay(config.environment["imu_delay_ms"])
        self.scenario.set_motor_delay(config.environment["motor_delay_ms"])
        self.scenario.apply_model_changes(data)
        if params is None:
            self.policy = UnconfiguredPolicy()
            self.estimator = base.StateEstimator()
            self.supervisor.set_torque_limit(1.0)
            self._policy_ticks = 0
            self._desired_torques = (0.0, 0.0)
            self.current_config = config
            if set_initial_command:
                self.command.forward_speed = 0.0
                self.command.yaw_rate = 0.0
            self.last_update = "empty JSON: controller unconfigured, motor output disabled"
            return
        self.model.actuator_ctrlrange[:, 0] = -params.motor_torque_limit_nm
        self.model.actuator_ctrlrange[:, 1] = params.motor_torque_limit_nm
        dynamics = identify_dynamics(
            self.model,
            self.addresses,
            samples=params.identification_samples,
            seed=params.identification_seed,
            control_interval=params.control_interval_steps,
        )
        policy = TunableLQRPolicy(dynamics, params)
        if policy.report.controllability_rank < 4:
            raise ValueError("辨识模型不可控：请增加 identification_samples 或更换 seed")
        if not math.isfinite(policy.report.spectral_radius):
            raise ValueError("LQR 求解得到非有限闭环谱半径")
        self.policy = policy
        self.estimator = base.StateEstimator(alpha=params.estimator_alpha)
        self.supervisor.set_torque_limit(params.motor_torque_limit_nm)
        self._policy_ticks = policy.control_interval - 1
        self._desired_torques = (0.0, 0.0)
        self.current_config = config
        if set_initial_command:
            self.command.forward_speed = config.initial_speed
            self.command.yaw_rate = config.initial_yaw_rate
        self.last_update = (
            f"loaded JSON revision {config.document.get('revision', '?')}; "
            f"rho={policy.report.spectral_radius:.3f}"
        )

    def relearn_policy(self, data: mujoco.MjData | None = None) -> None:
        if data is None:
            raise ValueError("relearning requires the active MjData")
        if self.current_config.controller is None:
            raise ValueError("控制器尚未配置：请先导入完整的参数 JSON")
        self.install_configuration(data, self.current_config, set_initial_command=False)


class RecordingHarness(base.Harness):
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        controller: FreeTunableController,
    ) -> None:
        super().__init__(model, data, controller)
        self.recorder: ExperimentRecorder | None = None

    def execute_instruction(self, text: str) -> str:
        normalized = text.strip().lower()
        if any(word in normalized for word in ("稳定", "平衡", "stable", "balance")):
            self.controller.command.forward_speed = 0.0
            self.controller.command.yaw_rate = 0.0
            self.controller.command.reference_x = float(self.data.qpos[0])
            self.controller.supervisor.set_stability_mode()
            result = "已将运动目标归零；自由模式未启用速度、转向、倾角或自动重置限制。"
        else:
            result = super().execute_instruction(text)
        if self.recorder is not None:
            self.recorder.add_event("text_instruction", {"text": text, "result": result})
        return result

    def handle_key(self, keycode: int) -> None:
        before = asdict(self.controller.command)
        super().handle_key(keycode)
        after = asdict(self.controller.command)
        if self.recorder is not None and before != after:
            key_name = {
                glfw.KEY_UP: "UP",
                glfw.KEY_DOWN: "DOWN",
                glfw.KEY_LEFT: "LEFT",
                glfw.KEY_RIGHT: "RIGHT",
            }.get(keycode, str(keycode))
            self.recorder.add_event("keyboard", {"key": key_name, "command": after})

    def reset(self) -> None:
        super().reset()
        if not self.controller.is_configured:
            # A mathematically exact upright pose can remain motionless even
            # with zero torque. This deterministic perturbation makes the
            # cold-start failure observable and repeatable.
            pitch = math.radians(4.0)
            half_pitch = pitch / 2.0
            self.data.qpos[3:7] = (math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0)
            mujoco.mj_forward(self.model, self.data)
        if self.recorder is not None:
            self.recorder.add_event(
                "manual_reset",
                {
                    "controller_configured": self.controller.is_configured,
                    "cold_start_pitch_deg": 4.0 if not self.controller.is_configured else 0.0,
                },
            )


class CameraRecorder:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.renderer: mujoco.Renderer | None = None
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = 2.4
        self.camera.azimuth = 125.0
        self.camera.elevation = -18.0
        self.error: str | None = None
        try:
            # The base XML's offscreen framebuffer is 640x480.  Keep the
            # capture inside that limit without mutating the shared model file.
            self.renderer = mujoco.Renderer(model, height=360, width=640)
        except Exception as exc:
            self.error = str(exc)

    def capture(self, data: mujoco.MjData, path: Path) -> bool:
        if self.renderer is None:
            return False
        try:
            self.camera.lookat[:] = (float(data.qpos[0]), float(data.qpos[1]), 0.25)
            self.renderer.update_scene(data, camera=self.camera)
            image = self.renderer.render()
            from PIL import Image

            path.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(image).save(path)
            return True
        except Exception as exc:
            self.error = str(exc)
            self.close()
            return False

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class ExperimentRecorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        controller: FreeTunableController,
        parameter_path: Path,
        output_root: Path,
        recording_config: RecordingParameters,
    ) -> None:
        self.model = model
        self.data = data
        self.controller = controller
        self.addresses = controller.addresses
        self.parameter_path = parameter_path
        self.output_root = output_root
        self.config = recording_config
        self.active = False
        self.duration_s = recording_config.default_duration_s
        self.elapsed_s = 0.0
        self.next_sample_s = 0.0
        self.next_camera_s = 0.0
        self.samples: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.recent: Deque[dict[str, Any]] = deque()
        self.session_dir: Path | None = None
        self.started_at = ""
        self.camera: CameraRecorder | None = None
        self.camera_files: list[str] = []
        self.last_export: Path | None = None
        self.status = "等待开始记录"
        self._last_environment = self.environment_snapshot()
        self._environment_at_start = self._last_environment
        self._parameter_at_start = self.controller.current_config.document

    def environment_snapshot(self) -> dict[str, Any]:
        config = self.controller.scenario.config
        return {
            "flat_ground_model": True,
            "friction": config.friction,
            "slope_deg": config.slope_deg,
            "imu_noise_deg_s": config.imu_noise_deg_s,
            "imu_delay_ms": config.imu_delay_ms,
            "motor_delay_ms": config.motor_delay_ms,
            "gravity_m_s2": self.model.opt.gravity.tolist(),
            "physics_timestep_s": float(self.model.opt.timestep),
        }

    def start(self, duration_s: float) -> None:
        if self.active:
            raise ValueError("记录已经开始，请先停止当前记录")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = self.output_root / f"experiment_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.duration_s = duration_s
        self.elapsed_s = 0.0
        self.next_sample_s = 0.0
        self.next_camera_s = 0.0
        self.samples.clear()
        self.events.clear()
        self.recent.clear()
        self.camera_files.clear()
        self.started_at = datetime.now().isoformat(timespec="seconds")
        self.camera = CameraRecorder(self.model)
        self._last_environment = self.environment_snapshot()
        self._environment_at_start = self._last_environment
        self._parameter_at_start = json.loads(
            json.dumps(self.controller.current_config.document, ensure_ascii=False)
        )
        self.active = True
        self.last_export = None
        self.status = f"正在记录：0.0/{duration_s:.1f} s"
        self.add_event(
            "recording_started",
            {
                "duration_s": duration_s,
                "parameter_file": str(self.parameter_path.resolve()),
                "parameter_document": self._parameter_at_start,
            },
        )

    def stop(self, reason: str = "manual") -> None:
        if not self.active:
            return
        self.add_event("recording_stopped", {"reason": reason})
        self.active = False
        if self.camera is not None:
            self.camera.close()
        self.status = f"记录停止：{len(self.samples)} 条样本；点击“导出”生成文件"

    def clear(self) -> None:
        self.samples.clear()
        self.events.clear()
        self.recent.clear()
        self.elapsed_s = 0.0
        self.next_sample_s = 0.0
        self.next_camera_s = 0.0
        self.camera_files.clear()
        self.status = "当前缓冲已清空；历史实验目录未删除"

    def add_event(self, event_type: str, payload: dict[str, Any]) -> None:
        if not self.active:
            return
        self.events.append(
            {
                "record_time_s": self.elapsed_s,
                "sim_time_s": float(self.data.time),
                "type": event_type,
                "payload": payload,
            }
        )

    def observe(self, estimated: base.RobotState) -> None:
        if not self.active:
            return
        self.elapsed_s += float(self.model.opt.timestep)
        environment = self.environment_snapshot()
        if environment != self._last_environment:
            self.add_event(
                "environment_changed",
                {"before": self._last_environment, "after": environment},
            )
            self._last_environment = environment
        if self.elapsed_s + 1e-12 >= self.next_sample_s:
            truth = self.controller.raw_state(self.data)
            _, truth_pitch, truth_yaw = base.quaternion_to_euler(self.data.qpos[3:7])
            row = {
                "record_time_s": self.elapsed_s,
                "sim_time_s": float(self.data.time),
                "target_forward_speed_m_s": self.controller.command.forward_speed,
                "target_yaw_rate_rad_s": self.controller.command.yaw_rate,
                "position_x_m": float(self.data.qpos[0]),
                "position_y_m": float(self.data.qpos[1]),
                "position_z_m": float(self.data.qpos[2]),
                "estimated_pitch_deg": math.degrees(estimated.pitch),
                "truth_pitch_deg": math.degrees(truth_pitch),
                "estimated_pitch_rate_deg_s": math.degrees(estimated.pitch_rate),
                "truth_pitch_rate_deg_s": math.degrees(truth.pitch_rate),
                "truth_yaw_deg": math.degrees(truth_yaw),
                "estimated_yaw_rate_deg_s": math.degrees(estimated.yaw_rate),
                "truth_yaw_rate_deg_s": math.degrees(truth.yaw_rate),
                "estimated_forward_speed_m_s": estimated.forward_speed,
                "truth_forward_speed_m_s": truth.forward_speed,
                "left_wheel_speed_rad_s": float(self.data.qvel[self.addresses.left_dof]),
                "right_wheel_speed_rad_s": float(self.data.qvel[self.addresses.right_dof]),
                "left_motor_torque_nm": estimated.left_torque,
                "right_motor_torque_nm": estimated.right_torque,
                "contact_count": int(self.data.ncon),
                "controller_mode": (
                    self.controller.supervisor.mode
                    if self.controller.is_configured
                    else "UNCONFIGURED"
                ),
            }
            self.samples.append(row)
            self.recent.append(row)
            self.next_sample_s += self.config.sample_period_s
        window_start = self.elapsed_s - self.config.realtime_plot_window_s
        while self.recent and self.recent[0]["record_time_s"] < window_start:
            self.recent.popleft()
        if self.camera is not None and self.elapsed_s + 1e-12 >= self.next_camera_s:
            assert self.session_dir is not None
            frame_path = self.session_dir / "camera_frames" / f"frame_{len(self.camera_files):04d}.png"
            if self.camera.capture(self.data, frame_path):
                self.camera_files.append(str(frame_path.resolve()))
            self.next_camera_s += self.config.camera_period_s
        self.status = f"正在记录：{self.elapsed_s:.1f}/{self.duration_s:.1f} s，{len(self.samples)} 条"
        if self.elapsed_s >= self.duration_s:
            self.stop(reason="duration_reached")

    def metrics(self) -> dict[str, Any]:
        if not self.samples:
            return {
                "sample_count": 0,
                "duration_s": self.elapsed_s,
            }
        pitch = np.asarray([row["truth_pitch_deg"] for row in self.samples])
        speed = np.asarray([row["truth_forward_speed_m_s"] for row in self.samples])
        wheel = np.asarray(
            [
                max(abs(row["left_wheel_speed_rad_s"]), abs(row["right_wheel_speed_rad_s"]))
                for row in self.samples
            ]
        )
        torque = np.asarray(
            [
                max(abs(row["left_motor_torque_nm"]), abs(row["right_motor_torque_nm"]))
                for row in self.samples
            ]
        )
        controller_parameters = self.controller.current_config.controller
        limit = (
            controller_parameters.motor_torque_limit_nm
            if controller_parameters is not None
            else None
        )
        return {
            "sample_count": len(self.samples),
            "duration_s": self.elapsed_s,
            "pitch_rms_deg": float(np.sqrt(np.mean(np.square(pitch)))),
            "pitch_peak_abs_deg": float(np.max(np.abs(pitch))),
            "forward_speed_peak_abs_m_s": float(np.max(np.abs(speed))),
            "wheel_speed_peak_abs_rad_s": float(np.max(wheel)),
            "motor_torque_peak_abs_nm": float(np.max(torque)),
            "motor_saturation_ratio": (
                float(np.mean(torque >= 0.98 * limit)) if limit is not None else None
            ),
        }

    @staticmethod
    def _plot_series(
        rows: list[dict[str, Any]],
        output: Path,
        title: str,
        ylabel: str,
        series: list[tuple[str, str, str]],
    ) -> None:
        figure = Figure(figsize=(12.0, 4.8), dpi=170, constrained_layout=True)
        axis = figure.add_subplot(111)
        times = [row["record_time_s"] for row in rows]
        for key, label, color in series:
            axis.plot(times, [row[key] for row in rows], label=label, color=color, linewidth=1.4)
        axis.axhline(0.0, color="#7a8494", linewidth=0.7)
        axis.set_title(title)
        axis.set_xlabel("record time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        output.parent.mkdir(parents=True, exist_ok=True)
        FigureCanvasAgg(figure).print_png(str(output))

    def _export_plots(self, plot_dir: Path) -> list[str]:
        specifications = [
            (
                "pitch.png",
                "Pitch: estimator vs MuJoCo truth",
                "pitch (deg)",
                [
                    ("estimated_pitch_deg", "estimated", "#1976d2"),
                    ("truth_pitch_deg", "truth", "#d32f2f"),
                ],
            ),
            (
                "wheel_speed.png",
                "Left and right wheel speed",
                "wheel speed (rad/s)",
                [
                    ("left_wheel_speed_rad_s", "left", "#1565c0"),
                    ("right_wheel_speed_rad_s", "right", "#ef6c00"),
                ],
            ),
            (
                "motor_torque.png",
                "Left and right motor torque",
                "torque (Nm)",
                [
                    ("left_motor_torque_nm", "left", "#2e7d32"),
                    ("right_motor_torque_nm", "right", "#8e24aa"),
                ],
            ),
            (
                "forward_speed.png",
                "Forward speed: target, estimator and truth",
                "speed (m/s)",
                [
                    ("target_forward_speed_m_s", "target", "#455a64"),
                    ("estimated_forward_speed_m_s", "estimated", "#1976d2"),
                    ("truth_forward_speed_m_s", "truth", "#d32f2f"),
                ],
            ),
        ]
        paths: list[str] = []
        for filename, title, ylabel, series in specifications:
            path = plot_dir / filename
            self._plot_series(self.samples, path, title, ylabel, series)
            paths.append(str(path.resolve()))
        return paths

    def export(self) -> Path:
        if not self.samples or self.session_dir is None:
            raise ValueError("没有可导出的记录数据")
        if self.active:
            self.stop(reason="export_requested")
        telemetry_csv = self.session_dir / "telemetry.csv"
        telemetry_jsonl = self.session_dir / "telemetry.jsonl"
        events_jsonl = self.session_dir / "events.jsonl"
        with telemetry_csv.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.samples[0]))
            writer.writeheader()
            writer.writerows(self.samples)
        telemetry_jsonl.write_text(
            "\n".join(json.dumps(row, ensure_ascii=False) for row in self.samples) + "\n",
            encoding="utf-8",
        )
        events_jsonl.write_text(
            "\n".join(json.dumps(row, ensure_ascii=False) for row in self.events) + "\n",
            encoding="utf-8",
        )
        plot_files = self._export_plots(self.session_dir / "plots")
        metrics = self.metrics()
        experiment = {
            "schema_version": 1,
            "experiment_id": self.session_dir.name,
            "started_at": self.started_at,
            "exported_at": datetime.now().isoformat(timespec="seconds"),
            "parameter_file": str(self.parameter_path.resolve()),
            "parameter_at_start": self._parameter_at_start,
            "parameter_at_export": self.controller.current_config.document,
            "controller_configured": self.controller.is_configured,
            "learned_feedback_matrix_K": (
                self.controller.policy.gain.tolist() if self.controller.is_configured else None
            ),
            "learning_report": (
                asdict(self.controller.policy.report) if self.controller.is_configured else None
            ),
            "environment_at_start": self._environment_at_start,
            "environment_at_export": self.environment_snapshot(),
            "final_command": asdict(self.controller.command),
            "metrics": metrics,
            "files": {
                "telemetry_csv": str(telemetry_csv.resolve()),
                "telemetry_jsonl": str(telemetry_jsonl.resolve()),
                "events_jsonl": str(events_jsonl.resolve()),
                "plots": plot_files,
                "camera_frames": self.camera_files,
            },
            "camera_capture_error": None if self.camera is None else self.camera.error,
        }
        json_write(self.session_dir / "experiment.json", experiment)
        llm_input = {
            "task": "Analyse this BalanceBot experiment and return an updated parameter JSON.",
            "constraints": {
                "preserve_schema_version": 1,
                "return_complete_parameter_document": True,
                "no_speed_yaw_pitch_safety_limits": True,
                "motor_torque_limit_is_a_physical_parameter": True,
            },
            "cold_start": not self.controller.is_configured,
            "current_parameters": self.controller.current_config.document,
            "parameter_template": PARAMETER_TEMPLATE,
            "experiment_summary": experiment,
            "requested_output_file": str(self.parameter_path.resolve()),
        }
        json_write(self.session_dir / "llm_input.json", llm_input)
        self.last_export = self.session_dir
        self.status = f"已导出：{self.session_dir}"
        return self.session_dir


class RecorderPanel:
    COLORS = {
        "pitch": "#d32f2f",
        "left": "#1565c0",
        "right": "#ef6c00",
        "torque_left": "#2e7d32",
        "torque_right": "#8e24aa",
        "speed": "#00838f",
    }

    def __init__(
        self,
        harness: RecordingHarness,
        recorder: ExperimentRecorder,
        parameter_path: Path,
    ) -> None:
        self.harness = harness
        self.controller = harness.controller
        self.scenario = self.controller.scenario
        self.recorder = recorder
        self.parameter_path = parameter_path
        self.closed = False
        self.camera_follow = False
        self.root = tk.Tk()
        self.root.title("BalanceBot 自由控制、记录与结构化调优")
        self.root.geometry("1220x820")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        left = tk.Frame(self.root, width=400)
        left.pack(side="left", fill="y", padx=8, pady=8)
        right = tk.Frame(self.root)
        right.pack(side="right", fill="both", expand=True, padx=(0, 8), pady=8)
        notebook = ttk.Notebook(left, width=390, height=745)
        notebook.pack(fill="both", expand=True)
        control_tab = tk.Frame(notebook)
        data_tab = tk.Frame(notebook)
        notebook.add(control_tab, text="控制与环境")
        notebook.add(data_tab, text="记录与参数")
        self._build_control_tab(control_tab)
        self._build_data_tab(data_tab)

        self.canvas = tk.Canvas(right, background="#10151d", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.root.bind_all("<Up>", lambda _event: self._arrow(glfw.KEY_UP))
        self.root.bind_all("<Down>", lambda _event: self._arrow(glfw.KEY_DOWN))
        self.root.bind_all("<Left>", lambda _event: self._arrow(glfw.KEY_LEFT))
        self.root.bind_all("<Right>", lambda _event: self._arrow(glfw.KEY_RIGHT))

    def _build_control_tab(self, parent: tk.Misc) -> None:
        command_box = tk.LabelFrame(parent, text="自然语言指令", padx=8, pady=7)
        command_box.pack(fill="x", padx=7, pady=7)
        tk.Label(command_box, text="前进 0.3｜左转 0.4｜停止｜稳定｜状态｜重置").pack(anchor="w")
        row = tk.Frame(command_box)
        row.pack(fill="x", pady=(5, 0))
        self.command_entry = tk.Entry(row)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", self.submit_command)
        tk.Button(row, text="发送", command=self.submit_command).pack(side="left", padx=(6, 0))
        self.command_result = tk.StringVar(value="自由模式：无速度、转向、倾角和自动重置限制。")
        tk.Label(
            command_box,
            textvariable=self.command_result,
            justify="left",
            anchor="w",
            wraplength=350,
            fg="#1f3b6d",
        ).pack(fill="x", pady=(5, 0))

        environment_box = tk.LabelFrame(parent, text="环境与传感器", padx=8, pady=7)
        environment_box.pack(fill="x", padx=7, pady=7)
        config = self.controller.current_config.environment
        self.scales: dict[str, tk.DoubleVar] = {}
        self._add_scale(environment_box, "friction", "摩擦", config["friction"], 0.20, 2.50, 0.05)
        self._add_scale(environment_box, "slope_deg", "坡度 (deg)", config["slope_deg"], -10, 10, 0.5)
        self._add_scale(
            environment_box,
            "imu_noise_deg_s",
            "IMU 噪声",
            config["imu_noise_deg_s"],
            0,
            10,
            0.1,
        )
        self._add_scale(
            environment_box,
            "imu_delay_ms",
            "IMU 延迟 (ms)",
            config["imu_delay_ms"],
            0,
            120,
            5,
        )
        self._add_scale(
            environment_box,
            "motor_delay_ms",
            "电机延迟 (ms)",
            config["motor_delay_ms"],
            0,
            120,
            5,
        )

        action_box = tk.LabelFrame(parent, text="动作与视角", padx=8, pady=7)
        action_box.pack(fill="x", padx=7, pady=7)
        row = tk.Frame(action_box)
        row.pack(fill="x")
        tk.Button(row, text="前推 +25 N", command=lambda: self.push(25.0)).pack(side="left")
        tk.Button(row, text="后推 -25 N", command=lambda: self.push(-25.0)).pack(
            side="left", padx=5
        )
        tk.Button(row, text="手动重置", command=self.manual_reset).pack(side="left")
        self.camera_button = tk.Button(action_box, text="摄像头跟随：关", command=self.toggle_camera)
        self.camera_button.pack(anchor="w", pady=(6, 0))

        self.live_state = tk.StringVar(value="等待仿真…")
        tk.Label(
            parent,
            textvariable=self.live_state,
            justify="left",
            anchor="nw",
            wraplength=365,
        ).pack(fill="x", padx=9, pady=8)

    def _build_data_tab(self, parent: tk.Misc) -> None:
        parameter_box = tk.LabelFrame(parent, text="结构化参数输入", padx=8, pady=7)
        parameter_box.pack(fill="x", padx=7, pady=7)
        self.parameter_var = tk.StringVar(value=str(self.parameter_path))
        tk.Entry(parameter_box, textvariable=self.parameter_var).pack(fill="x")
        row = tk.Frame(parameter_box)
        row.pack(fill="x", pady=(6, 0))
        tk.Button(row, text="导入参数 JSON", command=self.load_parameters).pack(side="left")
        tk.Button(row, text="选择文件", command=self.choose_parameter_file).pack(
            side="left", padx=6
        )
        self.parameter_status = tk.StringVar(value=self._parameter_summary())
        tk.Label(
            parameter_box,
            textvariable=self.parameter_status,
            justify="left",
            anchor="w",
            wraplength=350,
            fg="#245b32",
        ).pack(fill="x", pady=(6, 0))

        recording_box = tk.LabelFrame(parent, text="实验记录", padx=8, pady=7)
        recording_box.pack(fill="x", padx=7, pady=7)
        row = tk.Frame(recording_box)
        row.pack(fill="x")
        tk.Label(row, text="记录时长 (s)：").pack(side="left")
        self.duration_var = tk.StringVar(value=f"{self.recorder.config.default_duration_s:g}")
        duration = ttk.Combobox(
            row,
            textvariable=self.duration_var,
            values=("5", "10", "30", "60"),
            width=8,
        )
        duration.pack(side="left")
        duration.bind("<<ComboboxSelected>>", lambda _event: None)
        buttons = tk.Frame(recording_box)
        buttons.pack(fill="x", pady=(7, 0))
        tk.Button(buttons, text="开始记录", command=self.start_recording).pack(side="left")
        tk.Button(buttons, text="停止记录", command=self.stop_recording).pack(side="left", padx=4)
        tk.Button(buttons, text="清空", command=self.clear_recording).pack(side="left", padx=4)
        tk.Button(buttons, text="导出", command=self.export_recording).pack(side="left", padx=4)
        self.recording_status = tk.StringVar(value=self.recorder.status)
        tk.Label(
            recording_box,
            textvariable=self.recording_status,
            justify="left",
            anchor="w",
            wraplength=350,
            fg="#1f3b6d",
        ).pack(fill="x", pady=(7, 0))

        explanation = (
            "导出内容：telemetry.csv/jsonl、events.jsonl、experiment.json、"
            "llm_input.json、4 张高清曲线和 MuJoCo 关键帧。\n\n"
            "后续只需把实验目录交给我；我会分析后返回完整 controller_params.json。"
        )
        tk.Label(parent, text=explanation, justify="left", anchor="nw", wraplength=360).pack(
            fill="x", padx=10, pady=8
        )
        self.metrics_text = tk.StringVar(value="尚无记录数据")
        tk.Label(
            parent,
            textvariable=self.metrics_text,
            justify="left",
            anchor="nw",
            fg="#333333",
        ).pack(fill="x", padx=10, pady=8)

    def _add_scale(
        self,
        parent: tk.Misc,
        key: str,
        label: str,
        initial: float,
        lower: float,
        upper: float,
        resolution: float,
    ) -> None:
        row = tk.Frame(parent)
        row.pack(fill="x")
        tk.Label(row, text=label, width=14, anchor="w").pack(side="left")
        variable = tk.DoubleVar(value=initial)
        self.scales[key] = variable
        tk.Scale(
            row,
            variable=variable,
            from_=lower,
            to=upper,
            resolution=resolution,
            orient="horizontal",
            showvalue=True,
            length=240,
            command=lambda raw, item=key: self.change_environment(item, float(raw)),
        ).pack(side="left")

    def _arrow(self, key: int) -> str:
        self.harness.handle_key(key)
        return "break"

    def submit_command(self, _event: object | None = None) -> None:
        text = self.command_entry.get().strip()
        if text:
            self.command_result.set(self.harness.execute_instruction(text))
            self.command_entry.delete(0, tk.END)

    def change_environment(self, key: str, value: float) -> None:
        callbacks: dict[str, Callable[[float], None]] = {
            "friction": self.scenario.set_friction,
            "slope_deg": self.scenario.set_slope,
            "imu_noise_deg_s": self.scenario.set_imu_noise,
            "imu_delay_ms": self.scenario.set_imu_delay,
            "motor_delay_ms": self.scenario.set_motor_delay,
        }
        callbacks[key](value)

    def push(self, force: float) -> None:
        self.scenario.schedule_push(float(self.harness.data.time), force)
        self.recorder.add_event("external_push", {"force_x_n": force, "duration_s": 0.15})
        self.command_result.set(f"已施加 {force:+.0f} N、持续 0.15 s 的推力。")

    def manual_reset(self) -> None:
        self.harness.reset()
        self.command_result.set("已手动重置；程序不会因倾角自动重置。")

    def toggle_camera(self) -> None:
        self.camera_follow = not self.camera_follow
        self.scenario.config.camera_follow = self.camera_follow
        self.camera_button.configure(text=f"摄像头跟随：{'开' if self.camera_follow else '关'}")
        self.recorder.add_event("camera_follow", {"enabled": self.camera_follow})

    def choose_parameter_file(self) -> None:
        selected = filedialog.askopenfilename(
            title="选择控制器参数 JSON",
            filetypes=(("JSON", "*.json"), ("All files", "*.*")),
            initialdir=str(ROOT),
        )
        if selected:
            self.parameter_var.set(selected)

    def _parameter_summary(self) -> str:
        config = self.controller.current_config
        params = config.controller
        if params is None:
            return (
                "未配置（冷启动）｜JSON = {}\n"
                "电机控制输出为 0；导入完整参数后才启用 LQR。"
            )
        return (
            f"revision={config.document.get('revision', '?')} | "
            f"Q=({params.q_pitch:g}, {params.q_pitch_rate:g}, "
            f"{params.q_forward_speed:g}, {params.q_yaw_rate:g})\n"
            f"R=({params.r_common_torque:g}, {params.r_differential_torque:g}) | "
            f"torque={params.motor_torque_limit_nm:g} Nm | "
            f"interval={params.control_interval_steps} steps | alpha={params.estimator_alpha:g}"
        )

    def load_parameters(self) -> None:
        path = Path(self.parameter_var.get()).expanduser().resolve()
        try:
            config = load_configuration(path)
            self.controller.install_configuration(self.harness.data, config, set_initial_command=True)
            self.parameter_path = path
            self.recorder.parameter_path = path
            self.recorder.config = config.recording
            for key, variable in self.scales.items():
                variable.set(config.environment[key])
            self.duration_var.set(f"{config.recording.default_duration_s:g}")
            self.parameter_status.set(self._parameter_summary())
            self.recorder.add_event(
                "parameter_json_loaded",
                {
                    "path": str(path),
                    "revision": config.document.get("revision"),
                    "parameter_document": config.document,
                },
            )
            self.command_result.set("参数 JSON 已通过校验并加载；机器人状态未自动重置。")
        except Exception as exc:
            messagebox.showerror("参数导入失败", str(exc), parent=self.root)

    def start_recording(self) -> None:
        try:
            duration = finite_number(self.duration_var.get(), "记录时长", 0.1, 86_400.0)
            self.recorder.start(duration)
        except Exception as exc:
            messagebox.showerror("无法开始记录", str(exc), parent=self.root)

    def stop_recording(self) -> None:
        self.recorder.stop()

    def clear_recording(self) -> None:
        self.recorder.clear()

    def export_recording(self) -> None:
        try:
            directory = self.recorder.export()
            messagebox.showinfo("导出完成", f"结构化实验已保存到：\n{directory}", parent=self.root)
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc), parent=self.root)

    def _draw_plot(
        self,
        x0: float,
        y0: float,
        width: float,
        height: float,
        title: str,
        rows: list[dict[str, Any]],
        series: list[tuple[str, str, str]],
    ) -> None:
        canvas = self.canvas
        canvas.create_rectangle(x0, y0, x0 + width, y0 + height, fill="#18212c", outline="#37474f")
        canvas.create_text(x0 + 10, y0 + 10, text=title, anchor="nw", fill="#e5edf5")
        if len(rows) < 2:
            canvas.create_text(
                x0 + width / 2,
                y0 + height / 2,
                text="等待记录数据",
                fill="#8fa3b8",
            )
            return
        values = [float(row[key]) for row in rows for key, _label, _color in series]
        maximum = max(max(abs(value) for value in values), 1e-6)
        t0 = float(rows[0]["record_time_s"])
        t1 = float(rows[-1]["record_time_s"])
        left, right = x0 + 46, x0 + width - 12
        top, bottom = y0 + 32, y0 + height - 24
        middle = 0.5 * (top + bottom)
        canvas.create_line(left, middle, right, middle, fill="#52606d")
        canvas.create_text(left - 5, top, text=f"+{maximum:.2g}", anchor="e", fill="#8fa3b8")
        canvas.create_text(left - 5, bottom, text=f"-{maximum:.2g}", anchor="e", fill="#8fa3b8")
        for key, label, color in series:
            points: list[float] = []
            for row in rows:
                fraction = (float(row["record_time_s"]) - t0) / max(t1 - t0, 1e-9)
                x = left + fraction * (right - left)
                y = middle - float(row[key]) / maximum * (bottom - top) / 2.0
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)
        legend_x = left
        for _key, label, color in series:
            canvas.create_text(legend_x, bottom + 13, text=label, anchor="w", fill=color)
            legend_x += 88

    def redraw(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 600)
        height = max(self.canvas.winfo_height(), 600)
        gap = 8
        plot_height = (height - 3 * gap) / 4.0
        rows = list(self.recorder.recent)
        plots = [
            (
                "Pitch (deg): truth / estimated",
                [
                    ("truth_pitch_deg", "truth", self.COLORS["pitch"]),
                    ("estimated_pitch_deg", "estimate", "#64b5f6"),
                ],
            ),
            (
                "Wheel speed (rad/s)",
                [
                    ("left_wheel_speed_rad_s", "left", self.COLORS["left"]),
                    ("right_wheel_speed_rad_s", "right", self.COLORS["right"]),
                ],
            ),
            (
                "Motor torque (Nm)",
                [
                    ("left_motor_torque_nm", "left", self.COLORS["torque_left"]),
                    ("right_motor_torque_nm", "right", self.COLORS["torque_right"]),
                ],
            ),
            (
                "Forward speed (m/s): target / truth",
                [
                    ("target_forward_speed_m_s", "target", "#b0bec5"),
                    ("truth_forward_speed_m_s", "truth", self.COLORS["speed"]),
                ],
            ),
        ]
        for index, (title, series) in enumerate(plots):
            self._draw_plot(0, index * (plot_height + gap), width, plot_height, title, rows, series)

    def update(self, state: base.RobotState) -> None:
        if self.closed:
            return
        self.recording_status.set(self.recorder.status)
        metrics = self.recorder.metrics()
        self.metrics_text.set(
            f"样本：{metrics.get('sample_count', 0)}\n"
            f"记录时间：{metrics.get('duration_s', 0.0):.2f} s\n"
            f"俯仰峰值：{metrics.get('pitch_peak_abs_deg', 0.0):.2f}°\n"
            f"速度峰值：{metrics.get('forward_speed_peak_abs_m_s', 0.0):.2f} m/s\n"
            f"力矩峰值：{metrics.get('motor_torque_peak_abs_nm', 0.0):.2f} Nm"
        )
        self.live_state.set(
            f"t={state.sim_time:.2f}s | "
            f"{'FREE-LQR' if self.controller.is_configured else 'UNCONFIGURED'}\n"
            f"位置=({state.x:+.2f}, {state.y:+.2f}) m\n"
            f"速度={state.forward_speed:+.2f}/{self.controller.command.forward_speed:+.2f} m/s\n"
            f"俯仰={math.degrees(state.pitch):+.2f}° | 转向率={state.yaw_rate:+.2f} rad/s\n"
            f"左右力矩={state.left_torque:+.2f}/{state.right_torque:+.2f} Nm"
        )
        self.redraw()
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            self.recorder.stop(reason="ui_closed")
            self.root.destroy()


def run_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: FreeTunableController,
    harness: RecordingHarness,
    panel: RecorderPanel,
    recorder: ExperimentRecorder,
) -> None:
    next_overlay = 0.0
    last_plot_wall = 0.0
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
            while viewer.is_running() and not panel.closed:
                step_start = time.perf_counter()
                state = controller.step(data)
                mujoco.mj_step(model, data)
                recorder.observe(state)
                if controller.scenario.config.camera_follow:
                    with viewer.lock():
                        target = np.asarray((data.qpos[0], data.qpos[1], 0.25))
                        viewer.cam.lookat[:] = 0.88 * viewer.cam.lookat + 0.12 * target
                viewer.sync()
                now = time.perf_counter()
                if now - last_plot_wall >= 0.05:
                    panel.update(state)
                    last_plot_wall = now
                if data.time >= next_overlay:
                    mode = (
                        "FREE-LQR"
                        if controller.is_configured
                        else "UNCONFIGURED / ZERO TORQUE"
                    )
                    viewer.set_texts(
                        (
                            None,
                            None,
                            f"BalanceBot {mode} | Arrow keys control robot",
                            f"pitch={math.degrees(state.pitch):+.2f} deg | "
                            f"speed={state.forward_speed:+.2f} m/s\n{recorder.status}",
                        )
                    )
                    next_overlay += 0.20
                remaining = model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        panel.close()


def run_headless(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: FreeTunableController,
    recorder: ExperimentRecorder,
    duration: float,
    realtime: bool,
) -> Path:
    if not recorder.active:
        recorder.start(duration)
    wall_start = time.perf_counter()
    while recorder.active:
        state = controller.step(data)
        mujoco.mj_step(model, data)
        recorder.observe(state)
        if realtime:
            remaining = recorder.elapsed_s - (time.perf_counter() - wall_start)
            if remaining > 0.0:
                time.sleep(remaining)
    return recorder.export()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETER_PATH)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, help="headless recording duration")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--command", action="append", default=[])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    parameter_path = args.parameters.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    ensure_parameter_file(parameter_path)
    config = load_configuration(parameter_path)
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    controller = FreeTunableController(model, data, config)
    harness = RecordingHarness(model, data, controller)
    harness.reset()
    controller.command.forward_speed = config.initial_speed
    controller.command.yaw_rate = config.initial_yaw_rate
    recorder = ExperimentRecorder(
        model,
        data,
        controller,
        parameter_path,
        output_root,
        config.recording,
    )
    harness.recorder = recorder
    if args.headless:
        duration = args.duration or config.recording.default_duration_s
        recorder.start(duration)
        for command in args.command:
            print(harness.execute_instruction(command))
        exported = run_headless(
            model,
            data,
            controller,
            recorder,
            duration=duration,
            realtime=not args.no_realtime,
        )
        print(json.dumps({"status": "exported", "directory": str(exported)}, ensure_ascii=False))
    else:
        for command in args.command:
            print(harness.execute_instruction(command))
        panel = RecorderPanel(harness, recorder, parameter_path)
        run_viewer(model, data, controller, harness, panel, recorder)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyboardInterrupt, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "error", "type": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(2)
