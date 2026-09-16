"""Firmware-derived constants and explicitly labelled plant assumptions.

The only firmware source for Route A is Yahboom's
``4.Balanced_Car_base/04.bluetooth_control`` project. Values whose hardware
accuracy cannot be established from that project are labelled accordingly.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from enum import Enum
from math import isfinite
from typing import Any, ClassVar, Mapping


PID_KEYS = ("AP", "AD", "VP", "VI", "TP", "TD")
STAGE_KEYS = {
    "balance": ("AP", "AD"),
    "velocity": ("VP", "VI"),
    "turn": ("TP", "TD"),
}


@dataclass(frozen=True)
class PIDValues:
    AP: float
    AD: float
    VP: float
    VI: float
    TP: float
    TD: float

    def __post_init__(self) -> None:
        for name in PID_KEYS:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a number")
            if not isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, float(value))

    def as_dict(self) -> dict[str, float]:
        return {key: getattr(self, key) for key in PID_KEYS}

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any], *, strict: bool = True) -> "PIDValues":
        keys = set(data)
        expected = set(PID_KEYS)
        if strict and keys != expected:
            missing = sorted(expected - keys)
            extra = sorted(keys - expected)
            raise ValueError(f"PID fields mismatch; missing={missing}, extra={extra}")
        return cls(**{key: data[key] for key in PID_KEYS})

    def interpolate(self, other: "PIDValues", fraction: float) -> "PIDValues":
        fraction = max(0.0, min(1.0, float(fraction)))
        return PIDValues(**{
            key: getattr(self, key) + (getattr(other, key) - getattr(self, key)) * fraction
            for key in PID_KEYS
        })


DEFAULT_PID = PIDValues(AP=96.0, AD=48.0, VP=62.0, VI=31.0, TP=14.0, TD=20.0)


class ProfileMode(str, Enum):
    HARDWARE_FAITHFUL = "hardware-faithful"
    EXPLORATION = "exploration"


@dataclass
class EnvironmentConfig:
    """Environment/model settings. Firmware-locked constants live below."""

    profile_mode: str = ProfileMode.HARDWARE_FAITHFUL.value
    slope_deg: float = 0.0
    friction: float = 1.15
    payload_mass_kg: float = 0.0
    payload_height_m: float = 0.183
    payload_offset_x_m: float = 0.0
    battery_voltage_v: float = 12.0
    left_motor_gain: float = 1.0
    right_motor_gain: float = 1.0
    motor_time_constant_s: float = 0.015
    motor_static_pwm: int = 1300
    motor_viscous_friction_nm_per_rad_s: float = 0.0015
    motor_coulomb_friction_nm: float = 0.012
    actuator_delay_ms: float = 0.0
    pwm_deadzone: int = 1300
    imu_noise_deg: float = 0.08
    imu_bias_deg: float = 0.0
    imu_delay_ms: float = 5.0
    kalman_process_noise: float = 1.0e-10
    kalman_measurement_noise: float = 1.0e-4
    encoder_quantization: bool = True
    encoder_noise_counts: float = 0.0
    simulation_speed: float = 1.0
    seed: int = 20260912
    push_force_n: float = 1.0
    push_duration_s: float = 0.10
    push_height_m: float = 0.145

    _RANGES: ClassVar[dict[str, tuple[float, float]]] = {
        "slope_deg": (-15.0, 15.0), "friction": (0.2, 2.0),
        "payload_mass_kg": (0.0, 2.0), "payload_height_m": (0.16, 0.50),
        "payload_offset_x_m": (-0.10, 0.10), "battery_voltage_v": (8.0, 12.6),
        "left_motor_gain": (0.5, 1.5), "right_motor_gain": (0.5, 1.5),
        "motor_time_constant_s": (0.003, 0.20), "motor_static_pwm": (0.0, 2000.0),
        "motor_viscous_friction_nm_per_rad_s": (0.0, 0.02),
        "motor_coulomb_friction_nm": (0.0, 0.10), "actuator_delay_ms": (0.0, 100.0),
        "pwm_deadzone": (0.0, 2400.0), "imu_noise_deg": (0.0, 5.0),
        "imu_bias_deg": (-10.0, 10.0), "imu_delay_ms": (0.0, 100.0),
        "kalman_process_noise": (1e-14, 1e-3), "kalman_measurement_noise": (1e-8, 1.0),
        "encoder_noise_counts": (0.0, 10.0), "simulation_speed": (0.05, 20.0),
        "push_force_n": (0.0, 20.0), "push_duration_s": (0.01, 1.0),
        "push_height_m": (0.02, 0.30),
    }

    def validate(self) -> None:
        if self.profile_mode not in {mode.value for mode in ProfileMode}:
            raise ValueError("unknown profile_mode")
        for name, (low, high) in self._RANGES.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            if not low <= float(value) <= high:
                raise ValueError(f"{name} must be in [{low}, {high}]")
        if not isinstance(self.seed, int) or isinstance(self.seed, bool):
            raise ValueError("seed must be an integer")
        if self.profile_mode == ProfileMode.HARDWARE_FAITHFUL.value and self.pwm_deadzone != 1300:
            raise ValueError("hardware-faithful mode locks pwm_deadzone to 1300")
        if self.motor_static_pwm > self.pwm_deadzone:
            raise ValueError("motor_static_pwm cannot exceed firmware pwm_deadzone")

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "EnvironmentConfig":
        allowed = {item.name for item in fields(cls)}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"unknown environment fields: {sorted(unknown)}")
        result = cls(**dict(values))
        result.validate()
        return result


FIRMWARE_CONSTANTS: dict[str, Any] = {
    "physics_timestep_s": 0.001, "control_period_s": 0.005, "control_frequency_hz": 200,
    "uart": {"port": "UART5", "baud": 9600, "data_bits": 8, "stop_bits": 1, "parity": "none", "flow_control": "none"},
    "rx_buffer_bytes": 80, "wheel_diameter_m": 0.067,
    "encoder_counts_per_wheel_rev": 11 * 4 * 30, "battery_nominal_v": 12.0,
    "battery_cutoff_v": 9.6, "mid_angle_deg": 1.0, "fall_angle_deg": 40.0,
    "pwm_limit": 2600, "pwm_deadzone": 1300, "pwm_frequency_hz": 25_000,
    "movement_target": 25.0, "turn_target": 30.0, "pivot_turn_target": 50.0,
}

SOURCE_TRACE = {
    "control_period": "APP/app_control.c:Get_Angle comment + APP/PID/pid_control.c integral comment",
    "pid_defaults_and_movement": "APP/PID/pid_control.c",
    "pid_protocol_scaling_and_ack": "BSP/Bluetooth/app_bluetooth.c:Protocol",
    "uart_and_framing": "BSP/Bluetooth/bsp_bluetooth.c + app_bluetooth.c:deal_bluetooth",
    "pwm_and_protection": "APP/app_control.c + APP/app_motor.c",
    "encoder_geometry": "APP/app_motor.h + APP/app_motor.c", "mid_angle": "USER/main.c",
}

MODEL_ASSUMPTIONS = {
    "friction": "estimated", "base_mass_and_inertia": "estimated_from_existing_model; calibration_required",
    "wheel_track": "estimated_from_existing_STEP-derived model",
    "motor_stall_torque": "estimated_from_existing project documentation",
    "motor_time_constant": "estimated; calibration_required",
    "motor_static_pwm_and_friction": "behavior-matched estimate; calibration_required",
    "imu_noise_bias_delay": "estimated; calibration_required",
    "left_right_motor_gain": "estimated; calibration_required",
    "payload_inertia": "computed as a box; calibration_required",
    "push_application_height": "behavior-matched estimate; calibration_required",
}
