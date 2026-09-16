"""A 200 Hz virtual STM32 executing the 04.bluetooth_control PID path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .firmware_profile import DEFAULT_PID, FIRMWARE_CONSTANTS, PIDValues
from .protocol import (
    FrameStreamParser, ProtocolError, build_checked_frame, build_pid_report,
    parse_control_request,
)


@dataclass(frozen=True)
class SensorInputs:
    pitch_deg: float = 1.0
    pitch_rate_deg_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    encoder_left: int = 0
    encoder_right: int = 0
    acceleration_z: float = 9.81
    battery_v: float = 12.0
    raw_imu: bool = False
    accel_y_g: float = 0.0
    accel_z_g: float = 1.0
    gyro_balance_raw: float = 0.0


class FirmwareKalmanPitch:
    """The 2-state KF_X used by the burned 04.bluetooth_control firmware."""

    PERIOD_S = 0.005

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.angle_rad = 0.0
        self.bias_rad_s = 0.0
        self.p00, self.p01, self.p10, self.p11 = 1.0, 0.0, 0.0, 1.0

    def step(self, accel_y_g: float, accel_z_g: float, gyro_rad_s: float, *,
             process_noise: float = 1.0e-10, measurement_noise: float = 1.0e-4) -> float:
        dt = self.PERIOD_S
        angle_minus = self.angle_rad + dt * (gyro_rad_s - self.bias_rad_s)
        bias_minus = self.bias_rad_s
        p00m = self.p00 - dt*(self.p01+self.p10) + dt*dt*self.p11 + process_noise
        p01m = self.p01 - dt*self.p11
        p10m = self.p10 - dt*self.p11
        p11m = self.p11 + process_noise
        innovation = math.atan2(-accel_y_g, accel_z_g) - angle_minus
        inv_s = 1.0 / (p00m + measurement_noise)
        k0, k1 = p00m*inv_s, p10m*inv_s
        self.angle_rad = angle_minus + k0*innovation
        self.bias_rad_s = bias_minus + k1*innovation
        self.p00 = (1.0-k0)*p00m
        self.p01 = (1.0-k0)*p01m
        self.p10 = p10m-k1*p00m
        self.p11 = p11m-k1*p01m
        return self.angle_rad


@dataclass(frozen=True)
class ControlTelemetry:
    tick_ms: int = 0
    encoder_left: int = 0
    encoder_right: int = 0
    encoder_bias: float = 0.0
    encoder_integral: float = 0.0
    movement: float = 0.0
    turn_target: float = 0.0
    balance_pwm: int = 0
    velocity_pwm: int = 0
    turn_pwm: int = 0
    pwm_left: int = 0
    pwm_right: int = 0
    flags: int = 0
    command: int = 0


class VirtualSTM32:
    """Firmware behavior isolated behind bytes and sensor/actuator pins.

    PC code must access this object only through a Transport. Plant code may
    provide sensors and consume the returned PWM, matching a hardware boundary.
    """

    CONTROL_PERIOD_S = 0.005

    def __init__(self, pid: PIDValues = DEFAULT_PID, *, extended_telemetry: bool = True) -> None:
        self.pid = pid
        self.original_pid = DEFAULT_PID
        self.command = 0
        self.pivot = 0
        self.auto_report = False
        self.extended_telemetry = extended_telemetry
        self.stop_flag = False
        self.rx_parser = FrameStreamParser()
        self.rx_errors = 0
        self.tx_drops = 0
        self.control_overruns = 0
        self.tick_ms = 0
        self.encoder_bias = 0.0
        self.encoder_integral = 0.0
        self.telemetry_encoder_left = 0
        self.telemetry_encoder_right = 0
        self.pitch_filter = FirmwareKalmanPitch()
        self.last_pitch_deg = FIRMWARE_CONSTANTS["mid_angle_deg"]
        self.last_pitch_rate_deg_s = 0.0
        self.last_yaw_rate_deg_s = 0.0
        self.last = ControlTelemetry()

    @property
    def internal_pid(self) -> dict[str, float]:
        """Internal values; only virtual firmware tests may inspect this."""
        return {"AP": self.pid.AP * 100.0, "AD": self.pid.AD,
                "VP": self.pid.VP * 100.0, "VI": self.pid.VI,
                "TP": self.pid.TP * 100.0, "TD": self.pid.TD}

    def reset_runtime(self) -> None:
        self.tick_ms = 0
        self.encoder_bias = 0.0
        self.encoder_integral = 0.0
        self.telemetry_encoder_left = 0
        self.telemetry_encoder_right = 0
        self.pitch_filter.reset()
        self.last_pitch_deg = FIRMWARE_CONSTANTS["mid_angle_deg"]
        self.last_pitch_rate_deg_s = 0.0
        self.last_yaw_rate_deg_s = 0.0
        self.stop_flag = False
        self.command = 0
        self.pivot = 0
        self.last = ControlTelemetry()

    def report_tx_drop(self) -> None:
        self.tx_drops += 1

    def receive(self, chunk: bytes) -> list[bytes]:
        replies: list[bytes] = []
        before = self.rx_parser.discarded_frames
        frames = self.rx_parser.feed(chunk)
        if self.rx_parser.discarded_frames > before:
            self.rx_errors += self.rx_parser.discarded_frames - before
            replies.append(b"$ReceivePackError#")
        for frame in frames:
            try:
                request = parse_control_request(frame)
            except ProtocolError:
                self.rx_errors += 1
                replies.append(b"$ReceivePackError#")
                continue
            self.command = request.movement
            self.pivot = request.pivot
            if request.pid_operation == 1:
                # Executed C sends the same query reply twice, 5 ms apart.
                if any(value < 0 for value in self.pid.as_dict().values()):
                    replies.append(b"$GetPIDError#")
                else:
                    report = build_pid_report(self.pid)
                    replies.extend((report, report))
            elif request.pid_operation == 2:
                self.pid = self.original_pid
                replies.extend((build_pid_report(self.pid), b"$OK#"))
            if request.auto_report == 1:
                self.auto_report = True; replies.append(b"$OK#")
            elif request.auto_report == 2:
                self.auto_report = False; replies.append(b"$OK#")
            if request.groups:
                assert request.pid is not None
                old = self.pid.as_dict(); new = request.pid.as_dict()
                pairs = {"balance": ("AP", "AD"), "velocity": ("VP", "VI"), "turn": ("TP", "TD")}
                for group in request.groups:
                    for key in pairs[group]: old[key] = new[key]
                    replies.append(b"$OK#")
                self.pid = PIDValues.from_mapping(old)
        return replies

    @staticmethod
    def _deadzone_limit(value: int, deadzone: int) -> int:
        if value > 0: value += deadzone
        elif value < 0: value -= deadzone
        return max(-FIRMWARE_CONSTANTS["pwm_limit"], min(FIRMWARE_CONSTANTS["pwm_limit"], value))

    def control_step(self, sensor: SensorInputs, *, deadzone: int = 1300,
                     kalman_process_noise: float = 1.0e-10,
                     kalman_measurement_noise: float = 1.0e-4) -> tuple[tuple[int, int], list[bytes]]:
        self.tick_ms += 5
        internal = self.internal_pid
        if sensor.raw_imu:
            pitch_deg = math.degrees(self.pitch_filter.step(
                sensor.accel_y_g, sensor.accel_z_g, sensor.gyro_balance_raw/939.8,
                process_noise=kalman_process_noise,
                measurement_noise=kalman_measurement_noise,
            ))
            gyro_raw = sensor.gyro_balance_raw
            pitch_rate_deg_s = gyro_raw/16.4
        else:
            pitch_deg = sensor.pitch_deg
            pitch_rate_deg_s = sensor.pitch_rate_deg_s
            gyro_raw = pitch_rate_deg_s * 16.4
        self.last_pitch_deg = pitch_deg
        self.last_pitch_rate_deg_s = pitch_rate_deg_s
        self.last_yaw_rate_deg_s = sensor.yaw_rate_deg_s
        self.telemetry_encoder_left += sensor.encoder_left
        self.telemetry_encoder_right += sensor.encoder_right
        angle_bias = FIRMWARE_CONSTANTS["mid_angle_deg"] - pitch_deg
        gyro_bias = -gyro_raw
        balance = int(-internal["AP"] / 100.0 * angle_bias - gyro_bias * internal["AD"] / 100.0)

        if self.command == 1: movement = FIRMWARE_CONSTANTS["movement_target"]
        elif self.command == 2: movement = -FIRMWARE_CONSTANTS["movement_target"]
        else: movement = 0.0
        encoder_least = -(sensor.encoder_left + sensor.encoder_right)
        self.encoder_bias = self.encoder_bias * 0.84 + encoder_least * 0.16
        self.encoder_integral += self.encoder_bias + movement
        self.encoder_integral = max(-8000.0, min(8000.0, self.encoder_integral))
        velocity = int(-self.encoder_bias * internal["VP"] / 100.0 - self.encoder_integral * internal["VI"] / 100.0)

        if self.command == 3: turn_target = -FIRMWARE_CONSTANTS["turn_target"]
        elif self.command == 4: turn_target = FIRMWARE_CONSTANTS["turn_target"]
        elif self.pivot == 1: turn_target = -FIRMWARE_CONSTANTS["pivot_turn_target"]
        elif self.pivot == 2: turn_target = FIRMWARE_CONSTANTS["pivot_turn_target"]
        else: turn_target = 0.0
        turn_kd = internal["TD"] if self.command in (1, 2) else 0.0
        turn = int(turn_target * internal["TP"] / 100.0 + sensor.yaw_rate_deg_s * 16.4 * turn_kd / 100.0)

        left = self._deadzone_limit(balance + velocity + turn, deadzone)
        right = self._deadzone_limit(balance + velocity - turn, deadzone)
        fall = abs(pitch_deg) > FIRMWARE_CONSTANTS["fall_angle_deg"]
        low_voltage = sensor.battery_v < FIRMWARE_CONSTANTS["battery_cutoff_v"]
        flags = (1 if fall else 0) | (2 if low_voltage else 0) | (4 if self.stop_flag else 0)
        if flags:
            left = right = 0
            self.encoder_integral = 0.0
            flags |= 8
        self.last = ControlTelemetry(self.tick_ms, sensor.encoder_left, sensor.encoder_right,
            self.encoder_bias, self.encoder_integral, movement, turn_target, balance, velocity,
            turn, left, right, flags, self.command if not self.pivot else 4 + self.pivot)
        frames: list[bytes] = []
        if self.extended_telemetry and self.tick_ms % 100 == 0:
            frames.append(self._fast_frame(sensor))
        if self.extended_telemetry and self.tick_ms % 500 == 0:
            frames.append(self._slow_frame(sensor))
        if self.auto_report and self.tick_ms % 2000 == 0:
            frames.append(self._legacy_frame(sensor))
        return (left, right), frames

    def _fast_frame(self, sensor: SensorInputs) -> bytes:
        t = self.last
        values = (t.tick_ms, round(self.last_pitch_deg*100), round(self.last_pitch_rate_deg_s*100),
                  round(self.last_yaw_rate_deg_s*100), self.telemetry_encoder_left, self.telemetry_encoder_right,
                  t.balance_pwm, t.velocity_pwm, t.turn_pwm, t.pwm_left, t.pwm_right,
                  t.flags, t.command)
        frame = build_checked_frame("E1F," + ",".join(map(str, values)))
        self.telemetry_encoder_left = 0
        self.telemetry_encoder_right = 0
        return frame

    def _slow_frame(self, sensor: SensorInputs) -> bytes:
        return build_checked_frame(f"E1S,{self.tick_ms},{round(sensor.battery_v*1000)},{self.rx_errors},{self.tx_drops},{self.control_overruns}")

    def _legacy_frame(self, sensor: SensorInputs) -> bytes:
        circumference_mm = math.pi * 67.0
        scale = 200.0 / 1320.0 * circumference_mm / 10.0
        lv = sensor.encoder_left * scale; rv = sensor.encoder_right * scale
        return f"$LV{lv:.2f},RV{rv:.2f},AC{sensor.acceleration_z/100:.2f},GY{sensor.pitch_rate_deg_s*16.4:.2f},CSB0.00,VT{sensor.battery_v:.2f}#".encode("ascii")
