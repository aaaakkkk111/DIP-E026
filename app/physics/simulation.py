from __future__ import annotations
from dataclasses import dataclass, replace
import random, math, time
from app.types import RobotState, MotorCommand

@dataclass
class PhysicsConfig:
    # Null/default values deliberately represent tunable system-ID parameters, not manufacturer facts.
    body_mass_kg: float | None = None; wheel_mass_kg: float | None = None
    com_height_m: float | None = None; wheel_radius_m: float | None = None; track_width_m: float | None = None
    motor_gain: float = 1.0; friction: float = .08; battery_voltage: float = 7.4
    left_right_mismatch: float = 0.; deadband: float = .03; sensor_noise: float = .002
    sensor_bias: float = 0.; sensor_latency_s: float = .01; actuator_latency_s: float = .01
    encoder_quantization: float = .02

class SimulationBackend:
    """Reduced two-wheel inverted-pendulum model; parameters are deliberately configurable/system-ID values."""
    def __init__(self, config: PhysicsConfig | None = None, seed: int = 0):
        self.config, self.rng = config or PhysicsConfig(), random.Random(seed); self.reset(seed=seed)
    def reset(self, seed: int | None = None) -> RobotState:
        if seed is not None: self.rng.seed(seed)
        self.true = RobotState(pitch=self.rng.uniform(-.02,.02), timestamp=time.monotonic(), battery=self.config.battery_voltage)
        self._command = MotorCommand(); self._cmd_delay: list[tuple[float, MotorCommand]]=[]; self._sensor_delay: list[tuple[float,RobotState]]=[]
        return self.observe()
    def set_pose(self, position: float = 0., pitch: float = 0., velocity: float = 0., pitch_rate: float = 0.) -> RobotState:
        """User-defined simulation reset pose; values remain state inputs, not hardware commands."""
        self.true.position=float(position); self.true.pitch=float(pitch); self.true.velocity=float(velocity); self.true.pitch_rate=float(pitch_rate); self.true.pitch_accel=0.
        self._sensor_delay.clear(); return self.observe()
    def apply(self, command: MotorCommand) -> None:
        self._cmd_delay.append((time.monotonic()+self.config.actuator_latency_s, command))
    def step(self, dt: float, external_force: float = 0.) -> RobotState:
        now=time.monotonic()
        while self._cmd_delay and self._cmd_delay[0][0] <= now: self._command=self._cmd_delay.pop(0)[1]
        c=self._command; l=0. if abs(c.left)<self.config.deadband else c.left; r=0. if abs(c.right)<self.config.deadband else c.right
        avg=(l+r)/2*self.config.motor_gain*(self.true.battery/max(self.config.battery_voltage,.1)); asym=(r-l)*self.config.left_right_mismatch
        # Coupled pitch/translation dynamics. Coefficients are simulation parameters, not claimed robot measurements.
        old_rate=self.true.pitch_rate
        pitch_accel=7.5*math.sin(self.true.pitch) - 3.0*avg - .35*self.true.pitch_rate + external_force
        self.true.pitch_rate += pitch_accel*dt; self.true.pitch += self.true.pitch_rate*dt
        acceleration=2.0*avg - self.config.friction*self.true.velocity - .2*math.sin(self.true.pitch)
        self.true.velocity += acceleration*dt; self.true.position += self.true.velocity*dt
        self.true.left_speed=self.true.velocity-.5*asym; self.true.right_speed=self.true.velocity+.5*asym
        self.true.pitch_accel=(self.true.pitch_rate-old_rate)/dt; self.true.disturbance=external_force; self.true.timestamp=now
        self.true.battery=max(5.5, self.true.battery-abs(avg)*dt*.0008)
        self._sensor_delay.append((now+self.config.sensor_latency_s,replace(self.true)))
        return self.observe()
    def observe(self) -> RobotState:
        now=time.monotonic(); source=self.true
        while self._sensor_delay and self._sensor_delay[0][0] <= now: source=self._sensor_delay.pop(0)[1]
        n=self.config.sensor_noise; q=self.config.encoder_quantization
        return replace(source, pitch=source.pitch+self.config.sensor_bias+self.rng.gauss(0,n), pitch_rate=source.pitch_rate+self.rng.gauss(0,n), left_speed=round(source.left_speed/q)*q, right_speed=round(source.right_speed/q)*q)
