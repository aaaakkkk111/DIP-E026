"""MuJoCo plant and sensor bridge. Oracle values never cross telemetry."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from .firmware_profile import EnvironmentConfig, FIRMWARE_CONSTANTS
from .virtual_stm32 import SensorInputs, VirtualSTM32


MODEL_PATH = Path(__file__).resolve().parents[1] / "firmware_pid_car.xml"


@dataclass(frozen=True)
class OracleSample:
    time_s: float; pitch_deg: float; pitch_rate_deg_s: float; yaw_rate_deg_s: float
    speed_m_s: float; left_wheel_rad_s: float; right_wheel_rad_s: float
    left_torque_nm: float; right_torque_nm: float; contact_force_n: float
    applied_push_x_n: float; applied_push_y_n: float
    longitudinal_position_m: float; lateral_position_m: float
    def as_dict(self) -> dict[str, float]: return asdict(self)


def quaternion_pitch(quaternion_wxyz: np.ndarray) -> float:
    w, x, y, z = map(float, quaternion_wxyz)
    return math.asin(max(-1.0, min(1.0, 2.0*(w*y-z*x))))


class MotorPlant:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.left_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")]
        self.right_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")]
        self.target_pwm = (0, 0); self.applied_pwm = (0, 0)
        self.actual_torque = np.zeros(2); self.delay_queue: deque[tuple[float, tuple[int, int]]] = deque()

    def reset(self) -> None:
        self.target_pwm = self.applied_pwm = (0, 0); self.actual_torque[:] = 0; self.delay_queue.clear()

    def set_pwm(self, pwm: tuple[int, int], now: float, delay_ms: float) -> None:
        self.target_pwm = tuple(map(int, pwm)); self.delay_queue.append((now+delay_ms/1000.0, self.target_pwm))

    def step(self, data: mujoco.MjData, config: EnvironmentConfig) -> tuple[float, float]:
        while self.delay_queue and self.delay_queue[0][0] <= data.time + 1e-12:
            _, self.applied_pwm = self.delay_queue.popleft()
        pwm = np.asarray(self.applied_pwm, dtype=float)
        static_pwm = min(float(config.motor_static_pwm), float(FIRMWARE_CONSTANTS["pwm_limit"])-1.0)
        effective = np.sign(pwm) * np.clip(
            (np.abs(pwm)-static_pwm)/(float(FIRMWARE_CONSTANTS["pwm_limit"])-static_pwm), 0.0, 1.0)
        duty = effective * np.asarray((config.left_motor_gain, config.right_motor_gain))
        voltage_ratio = max(0.0, config.battery_voltage_v) / 12.0
        stall_torque = 4.8 * 9.80665 / 100.0
        no_load = 333.0 * 2*math.pi/60
        wheel_speed = np.asarray((data.qvel[self.left_dof], data.qvel[self.right_dof]))
        electromagnetic = stall_torque*(duty*voltage_ratio - np.where(np.abs(duty) > 0, wheel_speed/no_load, 0.0))
        mechanical = (config.motor_viscous_friction_nm_per_rad_s*wheel_speed
                      + config.motor_coulomb_friction_nm*np.tanh(wheel_speed/0.5))
        target = electromagnetic-mechanical
        target = np.clip(target, -stall_torque*voltage_ratio, stall_torque*voltage_ratio)
        alpha = 1.0-math.exp(-self.model.opt.timestep/config.motor_time_constant_s)
        self.actual_torque += alpha*(target-self.actual_torque)
        data.ctrl[:] = self.actual_torque
        return float(self.actual_torque[0]), float(self.actual_torque[1])


class MuJoCoPlant:
    PHYSICS_TIMESTEP_S = 0.001

    def __init__(self, config: EnvironmentConfig | None = None) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH)); self.data = mujoco.MjData(self.model)
        if not math.isclose(float(self.model.opt.timestep), self.PHYSICS_TIMESTEP_S, abs_tol=1e-12):
            raise ValueError("physics timestep must be 1 ms")
        self.config = config or EnvironmentConfig(); self.config.validate()
        self.chassis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.payload_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "payload")
        self.ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.tire_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, x) for x in ("left_tire", "right_tire")]
        self.orientation_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_orientation")
        self.gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self.velocity_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_velocity")
        self.acceleration_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_acceleration")
        self.left_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_hinge")
        self.right_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_hinge")
        self.left_qpos = self.model.jnt_qposadr[self.left_joint]; self.right_qpos = self.model.jnt_qposadr[self.right_joint]
        self.left_dof = self.model.jnt_dofadr[self.left_joint]; self.right_dof = self.model.jnt_dofadr[self.right_joint]
        self.motor = MotorPlant(self.model); self.rng = np.random.default_rng(self.config.seed)
        self.physics_steps = 0; self.encoder_origin_angles = (0.0, 0.0)
        self.last_encoder_totals = (0, 0)
        self.sensor_history: deque[tuple[float, float, float, float]] = deque(maxlen=501)
        self.push_force = np.zeros(2); self.push_until = -1.0; self.last_torque = (0.0, 0.0)
        self.configure(self.config); self.reset(self.config.seed)

    def _sensor(self, sensor_id: int) -> np.ndarray:
        start = self.model.sensor_adr[sensor_id]; return self.data.sensordata[start:start+self.model.sensor_dim[sensor_id]]

    def configure(self, config: EnvironmentConfig) -> None:
        config.validate(); self.config = config
        slope = math.radians(config.slope_deg)
        self.model.opt.gravity[:] = (-9.81*math.sin(slope), 0, -9.81*math.cos(slope))
        friction = np.asarray((config.friction, 0.015, 0.001))
        self.model.geom_friction[self.ground_id] = friction
        for geom_id in self.tire_ids: self.model.geom_friction[geom_id] = friction
        mass = max(1e-6, config.payload_mass_kg)
        self.model.body_mass[self.payload_id] = mass
        sx, sy, sz = 0.12, 0.09, 0.05
        self.model.body_inertia[self.payload_id] = mass/12*np.asarray((sy*sy+sz*sz, sx*sx+sz*sz, sx*sx+sy*sy))
        self.model.body_pos[self.payload_id] = (config.payload_offset_x_m, 0, config.payload_height_m-0.025)
        mujoco.mj_setConst(self.model, self.data); mujoco.mj_forward(self.model, self.data)

    def reset(self, seed: int | None = None, initial_pitch_deg: float = 3.0) -> None:
        if seed is not None: self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        angle = math.radians(initial_pitch_deg); self.data.qpos[3:7] = (math.cos(angle/2), 0, math.sin(angle/2), 0)
        mujoco.mj_forward(self.model, self.data); self.physics_steps = 0; self.motor.reset()
        self.encoder_origin_angles = (float(self.data.qpos[self.left_qpos]), float(self.data.qpos[self.right_qpos]))
        self.last_encoder_totals = (0, 0)
        self.sensor_history.clear(); self.push_force[:] = 0; self.push_until = -1; self.last_torque = (0, 0)

    def apply_push(self, direction: str, force_n: float | None = None) -> None:
        vectors = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}
        if direction not in vectors: raise ValueError("invalid push direction")
        force = self.config.push_force_n if force_n is None else max(0.0, min(20.0, force_n))
        self.push_force[:] = np.asarray(vectors[direction])*force
        self.push_until = self.data.time+self.config.push_duration_s

    def observe_oracle(self) -> OracleSample:
        quat = self._sensor(self.orientation_id); gyro = self._sensor(self.gyro_id); velocity = self._sensor(self.velocity_id)
        contact_force = 0.0
        for i in range(self.data.ncon):
            force = np.zeros(6); mujoco.mj_contactForce(self.model, self.data, i, force); contact_force += abs(float(force[0]))
        active = self.push_force if self.data.time < self.push_until else np.zeros(2)
        return OracleSample(float(self.data.time), math.degrees(quaternion_pitch(quat)), math.degrees(float(gyro[1])),
            math.degrees(float(gyro[2])), float(velocity[0]), float(self.data.qvel[self.left_dof]),
            float(self.data.qvel[self.right_dof]), self.last_torque[0], self.last_torque[1], contact_force,
            float(active[0]), float(active[1]), float(self.data.qpos[0]), float(self.data.qpos[1]))

    def sensor_inputs(self) -> SensorInputs:
        oracle = self.observe_oracle()
        acceleration = self._sensor(self.acceleration_id)
        accel_y_g, accel_z_g = float(acceleration[0]/9.81), float(acceleration[2]/9.81)
        accel_angle = math.atan2(-accel_y_g, accel_z_g)
        accel_magnitude = max(1e-6, math.hypot(accel_y_g, accel_z_g))
        accel_angle += math.radians(self.config.imu_bias_deg+self.rng.normal(0, self.config.imu_noise_deg))
        accel_y_g = -accel_magnitude*math.sin(accel_angle)
        accel_z_g = accel_magnitude*math.cos(accel_angle)
        gyro_raw = oracle.pitch_rate_deg_s*16.4 + self.rng.normal(0, self.config.imu_noise_deg*4*16.4)
        yaw_rate = oracle.yaw_rate_deg_s+self.rng.normal(0, self.config.imu_noise_deg*4)
        self.sensor_history.append((accel_y_g, accel_z_g, gyro_raw, yaw_rate))
        sample_period_ms = VirtualSTM32.CONTROL_PERIOD_S*1000.0
        delay_steps = min(len(self.sensor_history)-1, round(self.config.imu_delay_ms/sample_period_ms))
        accel_y_g, accel_z_g, gyro_raw, yaw_rate = list(self.sensor_history)[-(delay_steps+1)]
        current = (float(self.data.qpos[self.left_qpos]), float(self.data.qpos[self.right_qpos]))
        totals = [((current[i]-self.encoder_origin_angles[i])/(2*math.pi))*1320 for i in range(2)]
        noisy_totals = [value+self.rng.normal(0, self.config.encoder_noise_counts) for value in totals]
        quantized = [int(round(value)) if self.config.encoder_quantization else int(value) for value in noisy_totals]
        enc = [quantized[i]-self.last_encoder_totals[i] for i in range(2)]
        self.last_encoder_totals = tuple(quantized)
        return SensorInputs(
            pitch_deg=oracle.pitch_deg, pitch_rate_deg_s=gyro_raw/16.4,
            yaw_rate_deg_s=yaw_rate, encoder_left=enc[0], encoder_right=enc[1],
            acceleration_z=float(acceleration[2]), battery_v=self.config.battery_voltage_v,
            raw_imu=True, accel_y_g=accel_y_g, accel_z_g=accel_z_g,
            gyro_balance_raw=gyro_raw,
        )

    def step(self, stm32: VirtualSTM32) -> tuple[OracleSample, list[bytes]]:
        frames: list[bytes] = []
        if self.physics_steps % 5 == 0:
            pwm, frames = stm32.control_step(
                self.sensor_inputs(), deadzone=self.config.pwm_deadzone,
                kalman_process_noise=self.config.kalman_process_noise,
                kalman_measurement_noise=self.config.kalman_measurement_noise,
            )
            self.motor.set_pwm(pwm, self.data.time, self.config.actuator_delay_ms)
        self.data.xfrc_applied[self.chassis_id] = 0
        if self.data.time < self.push_until:
            force = np.asarray((self.push_force[0], self.push_force[1], 0.0))
            rotation = self.data.xmat[self.chassis_id].reshape(3, 3)
            application_point = self.data.xpos[self.chassis_id] + rotation @ np.asarray((0.0, 0.0, self.config.push_height_m))
            torque = np.cross(application_point-self.data.xipos[self.chassis_id], force)
            self.data.xfrc_applied[self.chassis_id, :3] = force
            self.data.xfrc_applied[self.chassis_id, 3:] = torque
        self.last_torque = self.motor.step(self.data, self.config)
        mujoco.mj_step(self.model, self.data); self.physics_steps += 1
        return self.observe_oracle(), frames
