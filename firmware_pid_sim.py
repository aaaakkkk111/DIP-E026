"""MuJoCo reproduction of the Yahboom STM32 balance-car PID firmware.

This module implements only migration step 1:

* a 1 ms MuJoCo plant with an exactly 5 ms firmware control period;
* the vendor Normal and Weight_M PID branches;
* encoder quantisation and the firmware's signed PWM/dead-zone/limit path;
* a 12 V geared-DC-motor approximation driven by the applied PWM value;
* an adjustable load fixed to a small upper platform.

No learned controller or neural policy is present in this file.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "firmware_pid_car.xml"

PHYSICS_PERIOD_S = 0.001
CONTROL_PERIOD_S = 0.005
CONTROL_STEPS = 5

WHEEL_DIAMETER_M = 0.067
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
ENCODER_LINES = 11
ENCODER_MULTIPLIER = 4
GEAR_RATIO = 30
ENCODER_COUNTS_PER_WHEEL_REV = ENCODER_LINES * ENCODER_MULTIPLIER * GEAR_RATIO

PWM_FREQUENCY_HZ = 25_000
PWM_PERIOD_COUNTS = 2_880
PWM_REGISTER_MAX = PWM_PERIOD_COUNTS - 1
PWM_COMMAND_LIMIT = 2_600
PWM_DEAD_ZONE_COMPENSATION = 1_300

BATTERY_NOMINAL_V = 12.0
BATTERY_CUTOFF_V = 9.6
MOTOR_NO_LOAD_RPM = 333.0
MOTOR_NO_LOAD_RAD_S = MOTOR_NO_LOAD_RPM * 2.0 * math.pi / 60.0
MOTOR_STALL_TORQUE_NM = 4.8 * 9.80665 / 100.0  # 4.8 kg cm
MOTOR_CURRENT_TIME_CONSTANT_S = 0.015

GYRO_COUNTS_PER_DEG_S = 16.4  # MPU6050 at +/-2000 deg/s
MAX_PAYLOAD_KG = 4.0
PAYLOAD_FULL_SIZE_M = np.asarray((0.120, 0.090, 0.050), dtype=float)

# Geometry and contact values.  Wheel diameter and track follow the firmware
# and supplied STEP assembly.  The rubber/hard-floor coefficient is not stated
# by the vendor, so it remains an explicit, user-adjustable engineering value.
WHEEL_TRACK_M = 0.126
DEFAULT_GROUND_FRICTION = 1.15
GROUND_TORSIONAL_FRICTION = 0.015
GROUND_ROLLING_FRICTION = 0.001

MOTION_SPEED_M_S = 0.15
MOTION_YAW_RATE_RAD_S = 0.45
MOTION_TARGETS = {
    "forward": (MOTION_SPEED_M_S, 0.0),
    "backward": (-MOTION_SPEED_M_S, 0.0),
    "left": (0.0, MOTION_YAW_RATE_RAD_S),
    "right": (0.0, -MOTION_YAW_RATE_RAD_S),
    "stop": (0.0, 0.0),
}
PUSH_DIRECTIONS = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}
PUSH_FORCE_LEVELS_N = {
    "轻 / Low (1 N)": 1.0,
    "中 / Medium (2 N)": 2.0,
    "强 / High (4 N)": 4.0,
}
PUSH_DURATION_S = 0.10
CAMERA_FOLLOW_SMOOTHING = 0.12
CAMERA_LOOKAT_HEIGHT_M = 0.10


def camera_follow_lookat(
    current_lookat: np.ndarray, chassis_position: np.ndarray
) -> np.ndarray:
    """Return a smoothed camera target centered on the moving chassis."""
    target = np.array(
        [chassis_position[0], chassis_position[1], CAMERA_LOOKAT_HEIGHT_M],
        dtype=float,
    )
    current = np.asarray(current_lookat, dtype=float)
    return current + CAMERA_FOLLOW_SMOOTHING * (target - current)


class ControlMode(str, Enum):
    NORMAL = "Normal"
    WEIGHT = "Weight_M"


@dataclass(frozen=True)
class FirmwareGains:
    balance_kp: float
    balance_kd: float
    velocity_kp: float
    velocity_ki: float
    turn_kp: float
    turn_kd: float
    balance_scale: float = 1.0
    velocity_scale: float = 1.0
    turn_scale: float = 1.0


# Final values assigned by the vendor's app_mode.c::Set_PID().  The Weight_M
# output multipliers are the vendor's Balance_K, Velocity_K and Turn_K values.
GAINS = {
    ControlMode.NORMAL: FirmwareGains(
        balance_kp=9600.0,
        balance_kd=48.0,
        velocity_kp=6200.0,
        velocity_ki=31.0,
        turn_kp=1700.0,
        turn_kd=20.0,
    ),
    ControlMode.WEIGHT: FirmwareGains(
        balance_kp=9600.0,
        balance_kd=75.0,
        velocity_kp=7000.0,
        velocity_ki=35.0,
        turn_kp=1400.0,
        turn_kd=20.0,
        balance_scale=2.0,
        velocity_scale=1.35,
        turn_scale=1.0,
    ),
}


@dataclass(frozen=True)
class RobotObservation:
    time_s: float
    pitch_rad: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float
    forward_speed_m_s: float
    left_wheel_angle_rad: float
    right_wheel_angle_rad: float


@dataclass(frozen=True)
class FirmwareTelemetry:
    control_tick: int
    mode: ControlMode
    encoder_left: int
    encoder_right: int
    movement_counts: float
    balance_pwm: int
    velocity_pwm: int
    turn_pwm: int
    pwm_left: int
    pwm_right: int
    stopped: bool


def _trunc(value: float) -> int:
    """Match C conversion from finite float to int (truncate toward zero)."""

    if not math.isfinite(value):
        return 0
    return int(value)


def _firmware_dead_zone_and_limit(value: int) -> int:
    if value > 0:
        value += PWM_DEAD_ZONE_COMPENSATION
    elif value < 0:
        value -= PWM_DEAD_ZONE_COMPENSATION
    return max(-PWM_COMMAND_LIMIT, min(PWM_COMMAND_LIMIT, value))


def quaternion_pitch(quaternion_wxyz: np.ndarray) -> float:
    w, x, y, z = map(float, quaternion_wxyz)
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


class FirmwarePIDController:
    """Line-for-line behavioural reproduction of the Normal/Weight PID path."""

    def __init__(self, mode: ControlMode = ControlMode.NORMAL) -> None:
        self.mode = ControlMode(mode)
        self.target_speed_m_s = 0.0
        self.target_yaw_rate_rad_s = 0.0
        self.battery_voltage = BATTERY_NOMINAL_V
        self.encoder_bias = 0.0
        self.encoder_integral = 0.0
        self._last_left_angle = 0.0
        self._last_right_angle = 0.0
        self.control_tick = 0
        self.last = FirmwareTelemetry(
            control_tick=0,
            mode=self.mode,
            encoder_left=0,
            encoder_right=0,
            movement_counts=0.0,
            balance_pwm=0,
            velocity_pwm=0,
            turn_pwm=0,
            pwm_left=0,
            pwm_right=0,
            stopped=False,
        )

    def reset(self, observation: RobotObservation) -> None:
        self.encoder_bias = 0.0
        self.encoder_integral = 0.0
        self._last_left_angle = observation.left_wheel_angle_rad
        self._last_right_angle = observation.right_wheel_angle_rad
        self.control_tick = 0
        self.last = FirmwareTelemetry(
            control_tick=0,
            mode=self.mode,
            encoder_left=0,
            encoder_right=0,
            movement_counts=0.0,
            balance_pwm=0,
            velocity_pwm=0,
            turn_pwm=0,
            pwm_left=0,
            pwm_right=0,
            stopped=False,
        )

    def set_mode(self, mode: ControlMode) -> None:
        mode = ControlMode(mode)
        if mode != self.mode:
            self.mode = mode
            self.encoder_bias = 0.0
            self.encoder_integral = 0.0

    def set_targets(self, speed_m_s: float, yaw_rate_rad_s: float) -> None:
        self.target_speed_m_s = float(np.clip(speed_m_s, -0.50, 0.50))
        self.target_yaw_rate_rad_s = float(np.clip(yaw_rate_rad_s, -1.0, 1.0))

    @staticmethod
    def _encoder_delta(current: float, previous: float) -> int:
        revolutions = (current - previous) / (2.0 * math.pi)
        return int(round(revolutions * ENCODER_COUNTS_PER_WHEEL_REV))

    def update(self, observation: RobotObservation) -> tuple[int, int]:
        gains = GAINS[self.mode]
        self.control_tick += 1

        encoder_left = self._encoder_delta(
            observation.left_wheel_angle_rad, self._last_left_angle
        )
        encoder_right = self._encoder_delta(
            observation.right_wheel_angle_rad, self._last_right_angle
        )
        self._last_left_angle = observation.left_wheel_angle_rad
        self._last_right_angle = observation.right_wheel_angle_rad

        pitch_deg = math.degrees(observation.pitch_rad)
        pitch_rate_counts = math.degrees(observation.pitch_rate_rad_s) * GYRO_COUNTS_PER_DEG_S
        yaw_rate_counts = math.degrees(observation.yaw_rate_rad_s) * GYRO_COUNTS_PER_DEG_S

        # Vendor Balance_PD with Mid_Angle == 0 for both selected modes.
        balance = (
            gains.balance_kp / 100.0 * pitch_deg
            + gains.balance_kd / 100.0 * pitch_rate_counts
        )
        balance_pwm = _trunc(balance * gains.balance_scale)

        # Movement is the expected sum of the two encoder deltas per 5 ms.
        movement = (
            self.target_speed_m_s
            * CONTROL_PERIOD_S
            * ENCODER_COUNTS_PER_WHEEL_REV
            / (math.pi * WHEEL_RADIUS_M)
        )
        encoder_least = -(encoder_left + encoder_right)
        self.encoder_bias = 0.84 * self.encoder_bias + 0.16 * encoder_least
        self.encoder_integral += self.encoder_bias
        self.encoder_integral += movement
        self.encoder_integral = float(np.clip(self.encoder_integral, -8000.0, 8000.0))
        velocity = (
            -self.encoder_bias * gains.velocity_kp / 100.0
            - self.encoder_integral * gains.velocity_ki / 100.0
        )
        velocity_pwm = _trunc(velocity * gains.velocity_scale)

        # Firmware positive Turn_PWM is a right turn; MuJoCo positive yaw is left.
        turn_target = float(np.clip(-self.target_yaw_rate_rad_s / 0.8 * 36.0, -36.0, 36.0))
        moving_straight = abs(self.target_speed_m_s) > 1e-4 and abs(turn_target) < 1e-4
        turn_kd = gains.turn_kd if moving_straight else 0.0
        turn = turn_target * gains.turn_kp / 100.0 + yaw_rate_counts * turn_kd / 100.0
        turn_pwm = _trunc(turn * gains.turn_scale)

        raw_left = balance_pwm + velocity_pwm + turn_pwm
        raw_right = balance_pwm + velocity_pwm - turn_pwm
        pwm_left = _firmware_dead_zone_and_limit(raw_left)
        pwm_right = _firmware_dead_zone_and_limit(raw_right)

        stopped = abs(pitch_deg) > 40.0 or self.battery_voltage < BATTERY_CUTOFF_V
        if stopped:
            pwm_left = 0
            pwm_right = 0
            self.encoder_integral = 0.0

        self.last = FirmwareTelemetry(
            control_tick=self.control_tick,
            mode=self.mode,
            encoder_left=encoder_left,
            encoder_right=encoder_right,
            movement_counts=movement,
            balance_pwm=balance_pwm,
            velocity_pwm=velocity_pwm,
            turn_pwm=turn_pwm,
            pwm_left=pwm_left,
            pwm_right=pwm_right,
            stopped=stopped,
        )
        return pwm_left, pwm_right


class PwmMotorPlant:
    """First-order geared DC motor model driven by applied PWM counts."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.left_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")
        ]
        self.right_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")
        ]
        self.actual_torque = np.zeros(2, dtype=float)
        self.command = (0, 0)

    def reset(self) -> None:
        self.actual_torque[:] = 0.0
        self.command = (0, 0)

    def set_pwm(self, left: int, right: int) -> None:
        self.command = (
            max(-PWM_COMMAND_LIMIT, min(PWM_COMMAND_LIMIT, int(left))),
            max(-PWM_COMMAND_LIMIT, min(PWM_COMMAND_LIMIT, int(right))),
        )

    def step(self, data: mujoco.MjData, battery_voltage: float) -> tuple[float, float]:
        wheel_speed = np.asarray((data.qvel[self.left_dof], data.qvel[self.right_dof]))
        duty = np.asarray(self.command, dtype=float) / PWM_REGISTER_MAX
        voltage_ratio = max(0.0, battery_voltage) / BATTERY_NOMINAL_V
        target = MOTOR_STALL_TORQUE_NM * (duty * voltage_ratio)
        target -= (MOTOR_STALL_TORQUE_NM / MOTOR_NO_LOAD_RAD_S) * wheel_speed
        max_torque = MOTOR_STALL_TORQUE_NM * max(0.6, voltage_ratio)
        target = np.clip(target, -max_torque, max_torque)
        alpha = 1.0 - math.exp(-self.model.opt.timestep / MOTOR_CURRENT_TIME_CONSTANT_S)
        self.actual_torque += alpha * (target - self.actual_torque)
        data.ctrl[:] = self.actual_torque
        return float(self.actual_torque[0]), float(self.actual_torque[1])


class FirmwarePidSimulation:
    def __init__(
        self,
        mode: ControlMode = ControlMode.NORMAL,
        payload_kg: float = 0.0,
        initial_pitch_deg: float = 1.5,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        if not math.isclose(float(self.model.opt.timestep), PHYSICS_PERIOD_S, abs_tol=1e-12):
            raise ValueError("model physics timestep must be 1 ms")
        self.data = mujoco.MjData(self.model)
        self.controller = FirmwarePIDController(mode)
        self.motor = PwmMotorPlant(self.model)
        self.payload_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload"
        )
        self.chassis_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis"
        )
        self.left_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge"
        )
        self.right_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge"
        )
        self.left_qpos = self.model.jnt_qposadr[self.left_joint_id]
        self.right_qpos = self.model.jnt_qposadr[self.right_joint_id]
        self.orientation_sensor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_orientation"
        )
        self.gyro_sensor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro"
        )
        self.velocity_sensor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_velocity"
        )
        self.ground_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground"
        )
        self.tire_geom_ids = tuple(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("left_tire", "right_tire")
        )
        self.physics_steps = 0
        self.payload_kg = 0.0
        self.initial_pitch_deg = float(np.clip(initial_pitch_deg, -15.0, 15.0))
        self.last_torque = (0.0, 0.0)
        self.ground_friction = DEFAULT_GROUND_FRICTION
        self._push_force_xy = np.zeros(2, dtype=float)
        self._push_end_time = -1.0
        self.set_payload_mass(payload_kg)
        self.set_ground_friction(DEFAULT_GROUND_FRICTION)
        self.reset()

    def _sensor(self, sensor_id: int) -> np.ndarray:
        start = self.model.sensor_adr[sensor_id]
        size = self.model.sensor_dim[sensor_id]
        return self.data.sensordata[start : start + size]

    def observe(self) -> RobotObservation:
        quaternion = self._sensor(self.orientation_sensor)
        gyro = self._sensor(self.gyro_sensor)
        velocity = self._sensor(self.velocity_sensor)
        return RobotObservation(
            time_s=float(self.data.time),
            pitch_rad=quaternion_pitch(quaternion),
            pitch_rate_rad_s=float(gyro[1]),
            yaw_rate_rad_s=float(gyro[2]),
            forward_speed_m_s=float(velocity[0]),
            left_wheel_angle_rad=float(self.data.qpos[self.left_qpos]),
            right_wheel_angle_rad=float(self.data.qpos[self.right_qpos]),
        )

    def set_payload_mass(self, mass_kg: float) -> None:
        mass = float(np.clip(mass_kg, 0.0, MAX_PAYLOAD_KG))
        if math.isclose(mass, self.payload_kg, abs_tol=1e-9):
            return
        dynamic_mass = max(mass, 1e-6)
        x, y, z = PAYLOAD_FULL_SIZE_M
        inertia = dynamic_mass / 12.0 * np.asarray(
            (y * y + z * z, x * x + z * z, x * x + y * y)
        )
        self.model.body_mass[self.payload_body_id] = dynamic_mass
        self.model.body_inertia[self.payload_body_id] = inertia
        self.payload_kg = mass
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def set_mode(self, mode: ControlMode) -> None:
        self.controller.set_mode(mode)

    def set_targets(self, speed_m_s: float, yaw_rate_rad_s: float) -> None:
        self.controller.set_targets(speed_m_s, yaw_rate_rad_s)

    def set_battery_voltage(self, voltage: float) -> None:
        self.controller.battery_voltage = float(np.clip(voltage, 8.0, 12.6))

    def set_ground_friction(self, coefficient: float) -> None:
        coefficient = float(np.clip(coefficient, 0.40, 1.60))
        friction = np.asarray(
            (
                coefficient,
                GROUND_TORSIONAL_FRICTION,
                GROUND_ROLLING_FRICTION,
            ),
            dtype=float,
        )
        self.model.geom_friction[self.ground_geom_id] = friction
        for geom_id in self.tire_geom_ids:
            self.model.geom_friction[geom_id] = friction
        self.ground_friction = coefficient

    def command_direction(self, direction: str) -> None:
        try:
            speed, yaw_rate = MOTION_TARGETS[direction]
        except KeyError as exc:
            raise ValueError(f"unknown motion direction: {direction}") from exc
        self.set_targets(speed, yaw_rate)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        angle = math.radians(self.initial_pitch_deg)
        self.data.qpos[3:7] = (math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0)
        mujoco.mj_forward(self.model, self.data)
        self.physics_steps = 0
        self.motor.reset()
        self.last_torque = (0.0, 0.0)
        self._push_force_xy[:] = 0.0
        self._push_end_time = -1.0
        self.controller.reset(self.observe())

    def push(
        self,
        force_x_n: float,
        force_y_n: float = 0.0,
        duration_s: float = PUSH_DURATION_S,
    ) -> None:
        # Stored as end time; apply_external_force clears it after expiry.
        force = np.asarray((force_x_n, force_y_n), dtype=float)
        magnitude = float(np.linalg.norm(force))
        if magnitude > 40.0:
            force *= 40.0 / magnitude
        self._push_force_xy[:] = force
        self._push_end_time = float(self.data.time + max(0.0, duration_s))

    def push_direction(
        self, direction: str, force_n: float, duration_s: float = PUSH_DURATION_S
    ) -> None:
        try:
            unit_x, unit_y = PUSH_DIRECTIONS[direction]
        except KeyError as exc:
            raise ValueError(f"unknown push direction: {direction}") from exc
        magnitude = float(np.clip(abs(force_n), 0.0, 40.0))
        self.push(unit_x * magnitude, unit_y * magnitude, duration_s)

    def _apply_external_force(self) -> None:
        self.data.xfrc_applied[self.chassis_body_id] = 0.0
        if self._push_end_time > self.data.time:
            self.data.xfrc_applied[self.chassis_body_id, 0:2] = self._push_force_xy

    @property
    def active_push_force_xy(self) -> tuple[float, float]:
        if self._push_end_time <= self.data.time:
            return (0.0, 0.0)
        return float(self._push_force_xy[0]), float(self._push_force_xy[1])

    def step(self) -> RobotObservation:
        if self.physics_steps % CONTROL_STEPS == 0:
            pwm = self.controller.update(self.observe())
            self.motor.set_pwm(*pwm)
        self._apply_external_force()
        self.last_torque = self.motor.step(self.data, self.controller.battery_voltage)
        mujoco.mj_step(self.model, self.data)
        self.physics_steps += 1
        return self.observe()

    @property
    def total_mass_kg(self) -> float:
        return float(np.sum(self.model.body_mass))


class ControlPanel:
    """Large, readable control panel kept separate from the render thread."""

    def __init__(self, simulation: FirmwarePidSimulation) -> None:
        import tkinter as tk
        from tkinter import font as tkfont, ttk

        self.tk = tk
        self.simulation = simulation
        self.root = tk.Tk()
        self.root.title("STM32 Firmware PID - MuJoCo Hardware Baseline")
        self.root.geometry("1220x820")
        self.root.minsize(1188, 760)
        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            tkfont.nametofont(name).configure(
                family="Microsoft YaHei UI", size=12
            )
        tkfont.nametofont("TkFixedFont").configure(family="Consolas", size=12)
        style = ttk.Style(self.root)
        style.configure("TButton", padding=(10, 7))
        style.configure(
            "TLabelframe.Label", font=("Microsoft YaHei UI", 12, "bold")
        )

        frame = ttk.Frame(self.root, padding=14)
        frame.grid(sticky="nsew")
        frame.columnconfigure(0, weight=1, uniform="panel")
        frame.columnconfigure(1, weight=1, uniform="panel")
        frame.rowconfigure(2, weight=1)
        self.mode = tk.StringVar(value=simulation.controller.mode.value)
        self.payload = tk.DoubleVar(value=simulation.payload_kg)
        self.speed = tk.DoubleVar(value=simulation.controller.target_speed_m_s)
        self.yaw = tk.DoubleVar(value=simulation.controller.target_yaw_rate_rad_s)
        self.battery = tk.DoubleVar(value=simulation.controller.battery_voltage)
        self.friction = tk.DoubleVar(value=simulation.ground_friction)
        self.push_level = tk.StringVar(value="中 / Medium (2 N)")
        self.camera_follow = tk.BooleanVar(value=True)
        self.camera_button_text = tk.StringVar()
        self.model_info = tk.StringVar()
        self.status = tk.StringVar()

        settings = ttk.LabelFrame(frame, text="运行与环境参数 / Runtime parameters", padding=10)
        settings.grid(row=0, column=0, padx=(0, 7), pady=(0, 8), sticky="nsew")
        settings.columnconfigure(1, weight=1)
        ttk.Label(settings, text="Firmware mode").grid(row=0, column=0, sticky="w")
        mode_box = ttk.Combobox(
            settings,
            values=[mode.value for mode in ControlMode],
            textvariable=self.mode,
            state="readonly",
            width=14,
        )
        mode_box.grid(row=0, column=1, sticky="ew")
        mode_box.bind("<<ComboboxSelected>>", lambda _event: self._apply())

        self._scale(settings, 1, "Payload / 负载 (kg)", self.payload, 0.0, MAX_PAYLOAD_KG, 0.1)
        self._scale(settings, 2, "Target speed / 目标速度 (m/s)", self.speed, -0.50, 0.50, 0.01)
        self._scale(settings, 3, "Target yaw / 目标转向 (rad/s)", self.yaw, -1.0, 1.0, 0.02)
        self._scale(settings, 4, "Battery / 电池电压 (V)", self.battery, 8.0, 12.6, 0.1)
        self._scale(settings, 5, "Tire friction / 轮胎摩擦 μ*", self.friction, 0.40, 1.60, 0.05)

        motion = ttk.LabelFrame(frame, text="小车方向控制 / Robot motion", padding=10)
        motion.grid(row=1, column=0, padx=(0, 7), pady=(0, 8), sticky="nsew")
        for column in range(3):
            motion.columnconfigure(column, weight=1)
        ttk.Button(motion, text="↑  前 / Forward", command=lambda: self._drive("forward")).grid(
            row=0, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(motion, text="←  左 / Left", command=lambda: self._drive("left")).grid(
            row=1, column=0, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(motion, text="停止 / Stop", command=lambda: self._drive("stop")).grid(
            row=1, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(motion, text="右 / Right  →", command=lambda: self._drive("right")).grid(
            row=1, column=2, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(motion, text="↓  后 / Backward", command=lambda: self._drive("backward")).grid(
            row=2, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(
            motion,
            textvariable=self.camera_button_text,
            command=self._toggle_camera_follow,
        ).grid(row=2, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(motion, text="重置仿真 / Reset", command=simulation.reset).grid(
            row=2, column=2, padx=4, pady=4, sticky="ew"
        )

        disturbance = ttk.LabelFrame(frame, text="四向外力 / External disturbance", padding=10)
        disturbance.grid(row=0, column=1, padx=(7, 0), pady=(0, 8), sticky="nsew")
        for column in range(3):
            disturbance.columnconfigure(column, weight=1)
        ttk.Label(disturbance, text="力度档位 / Force level").grid(
            row=0, column=0, padx=4, pady=(0, 8), sticky="w"
        )
        ttk.Combobox(
            disturbance,
            values=list(PUSH_FORCE_LEVELS_N),
            textvariable=self.push_level,
            state="readonly",
            width=21,
        ).grid(row=0, column=1, columnspan=2, padx=4, pady=(0, 8), sticky="ew")
        ttk.Button(disturbance, text="↑  前推", command=lambda: self._push("forward")).grid(
            row=1, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(disturbance, text="←  左推", command=lambda: self._push("left")).grid(
            row=2, column=0, padx=4, pady=4, sticky="ew"
        )
        ttk.Label(
            disturbance,
            text=f"作用 {PUSH_DURATION_S:.2f} s",
            anchor="center",
        ).grid(
            row=2, column=1, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(disturbance, text="右推  →", command=lambda: self._push("right")).grid(
            row=2, column=2, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(disturbance, text="↓  后推", command=lambda: self._push("backward")).grid(
            row=3, column=1, padx=4, pady=4, sticky="ew"
        )

        model_box = ttk.LabelFrame(frame, text="硬件对应关系 / Hardware model", padding=10)
        model_box.grid(row=1, column=1, padx=(7, 0), pady=(0, 8), sticky="nsew")
        ttk.Label(
            model_box,
            textvariable=self.model_info,
            justify="left",
            font=("Consolas", 11),
        ).grid(sticky="nw")

        status_box = ttk.LabelFrame(frame, text="实时控制状态 / Live controller state", padding=10)
        status_box.grid(row=2, column=0, columnspan=2, sticky="nsew")
        ttk.Label(
            status_box,
            textvariable=self.status,
            justify="left",
            font=("Consolas", 12),
        ).grid(sticky="nw")

        self.root.bind("<Up>", lambda _event: self._drive("forward"))
        self.root.bind("<Down>", lambda _event: self._drive("backward"))
        self.root.bind("<Left>", lambda _event: self._drive("left"))
        self.root.bind("<Right>", lambda _event: self._drive("right"))
        self.root.bind("<space>", lambda _event: self._drive("stop"))

        ttk.Label(
            frame,
            text="* 摩擦系数、精确质心和电机动态缺少厂家标定，当前为可调工程估计值。",
            foreground="#7a4b00",
        ).grid(row=3, column=0, columnspan=2, pady=(6, 0), sticky="w")

        self._apply()
        self._refresh_camera_button()
        self._refresh_model_info()

    @property
    def camera_follow_enabled(self) -> bool:
        return bool(self.camera_follow.get())

    def _toggle_camera_follow(self) -> None:
        self.camera_follow.set(not self.camera_follow_enabled)
        self._refresh_camera_button()

    def _refresh_camera_button(self) -> None:
        state = "开 / On" if self.camera_follow_enabled else "关 / Off"
        self.camera_button_text.set(f"镜头跟随 / Follow: {state}")

    def _drive(self, direction: str) -> None:
        self.simulation.command_direction(direction)
        self.speed.set(self.simulation.controller.target_speed_m_s)
        self.yaw.set(self.simulation.controller.target_yaw_rate_rad_s)

    def _push(self, direction: str) -> None:
        force_n = PUSH_FORCE_LEVELS_N[self.push_level.get()]
        self.simulation.push_direction(direction, force_n)

    def _refresh_model_info(self) -> None:
        self.model_info.set(
            f"physics / control : {PHYSICS_PERIOD_S * 1000:.0f} / {CONTROL_PERIOD_S * 1000:.0f} ms\n"
            f"wheel diameter    : {WHEEL_DIAMETER_M * 1000:.0f} mm\n"
            f"wheel track       : {WHEEL_TRACK_M * 1000:.0f} mm (STEP)\n"
            f"encoder           : {ENCODER_COUNTS_PER_WHEEL_REV} count/rev\n"
            f"motor             : 12 V, 30:1, {MOTOR_NO_LOAD_RPM:.0f} rpm\n"
            f"PWM               : {PWM_FREQUENCY_HZ / 1000:.0f} kHz, ±{PWM_COMMAND_LIMIT}\n"
            f"ground            : rigid / 2 ms contact\n"
            f"friction μ*       : {self.simulation.ground_friction:.2f}\n"
            f"model total mass  : {self.simulation.total_mass_kg:.2f} kg"
        )

    def _scale(self, parent, row, label, variable, low, high, resolution) -> None:
        from tkinter import ttk

        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        scale = self.tk.Scale(
            parent,
            variable=variable,
            from_=low,
            to=high,
            resolution=resolution,
            orient="horizontal",
            length=315,
            command=lambda _value: self._apply(),
        )
        scale.grid(row=row, column=1, sticky="ew")

    def _apply(self) -> None:
        if not self.root.winfo_exists():
            return
        self.simulation.set_mode(ControlMode(self.mode.get()))
        self.simulation.set_payload_mass(self.payload.get())
        self.simulation.set_targets(self.speed.get(), self.yaw.get())
        self.simulation.set_battery_voltage(self.battery.get())
        self.simulation.set_ground_friction(self.friction.get())
        if hasattr(self, "model_info"):
            self._refresh_model_info()

    def update(self, observation: RobotObservation) -> bool:
        try:
            telemetry = self.simulation.controller.last
            left_torque, right_torque = self.simulation.last_torque
            gain = GAINS[telemetry.mode]
            force_x, force_y = self.simulation.active_push_force_xy
            self.status.set(
                f"t={observation.time_s:7.3f} s   control={telemetry.control_tick:7d}\n"
                f"mode={telemetry.mode.value:8s} payload={self.simulation.payload_kg:4.1f} kg "
                f"total={self.simulation.total_mass_kg:4.2f} kg  "
                f"battery={self.simulation.controller.battery_voltage:4.1f} V  "
                f"friction={self.simulation.ground_friction:.2f}\n"
                f"pitch={math.degrees(observation.pitch_rad):+7.3f} deg  "
                f"speed actual/target={observation.forward_speed_m_s:+6.3f}/"
                f"{self.simulation.controller.target_speed_m_s:+5.2f} m/s  "
                f"yaw actual/target={observation.yaw_rate_rad_s:+6.3f}/"
                f"{self.simulation.controller.target_yaw_rate_rad_s:+5.2f} rad/s\n"
                f"gains B(P/D)=({gain.balance_kp:.0f}/{gain.balance_kd:.0f})  "
                f"V(P/I)=({gain.velocity_kp:.0f}/{gain.velocity_ki:.0f})  "
                f"T(P/D)=({gain.turn_kp:.0f}/{gain.turn_kd:.0f})\n"
                f"enc=({telemetry.encoder_left:+4d},{telemetry.encoder_right:+4d})  "
                f"PID=({telemetry.balance_pwm:+5d},{telemetry.velocity_pwm:+5d},{telemetry.turn_pwm:+5d})\n"
                f"PWM=({telemetry.pwm_left:+5d},{telemetry.pwm_right:+5d})  "
                f"torque=({left_torque:+.3f},{right_torque:+.3f}) N m  "
                f"external Fxy=({force_x:+.0f},{force_y:+.0f}) N"
            )
            self.root.update_idletasks()
            self.root.update()
            return True
        except self.tk.TclError:
            return False


def run_headless(simulation: FirmwarePidSimulation, duration_s: float) -> RobotObservation:
    state = simulation.observe()
    for _ in range(round(duration_s / simulation.model.opt.timestep)):
        state = simulation.step()
    return state


def run_viewer(simulation: FirmwarePidSimulation) -> None:
    panel = ControlPanel(simulation)
    with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
        with viewer.lock():
            viewer.cam.distance = 0.65
            viewer.cam.azimuth = 120
            viewer.cam.elevation = -18
            viewer.cam.lookat[:] = np.array(
                [simulation.data.qpos[0], simulation.data.qpos[1], CAMERA_LOOKAT_HEIGHT_M]
            )
        while viewer.is_running():
            started = time.perf_counter()
            state = simulation.step()
            if panel.camera_follow_enabled:
                with viewer.lock():
                    viewer.cam.lookat[:] = camera_follow_lookat(
                        viewer.cam.lookat, simulation.data.qpos[:3]
                    )
            viewer.sync()
            if not panel.update(state):
                break
            remaining = simulation.model.opt.timestep - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--mode", choices=[mode.value for mode in ControlMode], default="Normal")
    parser.add_argument("--payload-kg", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=0.0, help="target speed in m/s")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="target yaw rate in rad/s")
    parser.add_argument("--battery-v", type=float, default=12.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    simulation = FirmwarePidSimulation(
        mode=ControlMode(args.mode),
        payload_kg=args.payload_kg,
        initial_pitch_deg=args.initial_pitch_deg,
    )
    simulation.set_targets(args.speed, args.yaw_rate)
    simulation.set_battery_voltage(args.battery_v)
    if args.headless:
        state = run_headless(simulation, max(0.0, args.duration))
        telemetry = simulation.controller.last
        print(
            f"mode={telemetry.mode.value} payload={simulation.payload_kg:.2f}kg "
            f"t={state.time_s:.3f}s pitch={math.degrees(state.pitch_rad):+.3f}deg "
            f"speed={state.forward_speed_m_s:+.3f}m/s "
            f"pwm=({telemetry.pwm_left:+d},{telemetry.pwm_right:+d}) "
            f"stopped={telemetry.stopped}"
        )
    else:
        run_viewer(simulation)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
