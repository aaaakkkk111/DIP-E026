"""传感器噪声、执行器故障和外部冲击。
Sensor noise, actuator faults and external impulses.

这里每一个旋钮都接到了控制面板上的一个滑块或按钮；同时每一个旋钮在训练时都会
被随机化，好逼着 PPO 学出能扛住它们的增益，而不是只在理想世界里管用的增益。

Every knob here is wired to a slider or a button in the control panel, and
every knob is also randomised during training so the PPO agent is forced to
learn gains that survive them rather than gains that only work in a perfect
world.
"""
from __future__ import annotations

from collections import deque
from dataclasses import replace
import numpy as np

from .dynamics import ExternalWrench
from .params import DisturbanceConfig


class Measurement:
    """控制器被允许看到的东西的容器。
    Container for what the controller is allowed to see."""

    __slots__ = ("theta", "theta_dot", "v", "yaw_rate")

    def __init__(self, theta, theta_dot, v, yaw_rate):
        self.theta = theta
        self.theta_dot = theta_dot
        self.v = v
        self.yaw_rate = yaw_rate

    def as_array(self):
        return np.array([self.theta, self.theta_dot, self.v, self.yaw_rate])


class DisturbanceModel:
    """施加传感器噪声/延迟，并产生外部力旋量。
    Applies sensor noise / latency and produces external wrenches."""

    MAX_LATENCY = 20

    def __init__(self, cfg: DisturbanceConfig | None = None,
                 rng: np.random.Generator | None = None):
        self.cfg = cfg or DisturbanceConfig()
        self.rng = rng or np.random.default_rng()
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        self.gyro_bias = 0.0
        self._buf = deque(maxlen=self.MAX_LATENCY + 1)
        self._impulse_left = 0.0        # seconds remaining on a manual kick
        self._impulse_vec = (0.0, 0.0)
        self._torque_scale = 1.0 + self.rng.normal(0.0, self.cfg.torque_scale_err)

    def set_config(self, cfg: DisturbanceConfig):
        # 只有在*设置*变化时才重新抽电机增益误差。它是一个固定的制造偏差，
        # 不是逐拍噪声——每次 UI 推滑块值就重抽一次，会把它变成第二个噪声源，
        # 那个滑块的含义也就变了。
        #
        # Only redraw the motor gain error when the *setting* changes.  It is
        # a fixed manufacturing offset, not per-tick noise -- resampling it
        # every time the UI pushes the slider values would turn it into a
        # second noise source and make the slider mean something else.
        prev = getattr(self.cfg, "torque_scale_err", None)
        self.cfg = cfg
        if prev is None or abs(prev - cfg.torque_scale_err) > 1e-12:
            self._torque_scale = 1.0 + self.rng.normal(
                0.0, max(cfg.torque_scale_err, 0.0))

    # ------------------------------------------------------------------
    def measure(self, theta, theta_dot, v, yaw_rate, dt) -> Measurement:
        c = self.cfg
        if c.imu_bias_walk > 0.0:
            self.gyro_bias += self.rng.normal(0.0, c.imu_bias_walk * np.sqrt(dt))
        m = Measurement(
            theta + self.gyro_bias + self.rng.normal(0.0, c.noise_pitch),
            theta_dot + self.rng.normal(0.0, c.noise_pitch_rate),
            v + self.rng.normal(0.0, c.noise_vel),
            yaw_rate + self.rng.normal(0.0, c.noise_yaw_rate),
        )
        # 传输延迟 / transport delay
        self._buf.append(m)
        k = int(np.clip(c.latency_steps, 0, self.MAX_LATENCY))
        if k == 0 or len(self._buf) <= k:
            return self._buf[0] if len(self._buf) <= k else m
        return self._buf[-(k + 1)]

    # ------------------------------------------------------------------
    def corrupt_torque(self, tau_l, tau_r, tau_max):
        c = self.cfg
        s = self._torque_scale
        n = c.torque_noise * tau_max
        if n > 0.0:
            tau_l += self.rng.normal(0.0, n)
            tau_r += self.rng.normal(0.0, n)
        return tau_l * s, tau_r * s

    # ------------------------------------------------------------------
    def fire_impulse(self, force: float | None = None,
                     direction: float | None = None,
                     duration: float | None = None):
        """触发一次推击。``direction`` 在车体系下（0 = 正前方）。
        Trigger a push.  ``direction`` is in the body frame (0 = forward)."""
        c = self.cfg
        f = c.impulse_force if force is None else force
        d = c.impulse_dir if direction is None else direction
        t = c.impulse_duration if duration is None else duration
        self._impulse_left = float(t)
        self._impulse_vec = (f * np.cos(d), f * np.sin(d))

    def maybe_random_impulse(self, dt):
        c = self.cfg
        if c.random_impulse_hz <= 0.0 or c.random_impulse_max <= 0.0:
            return False
        if self.rng.random() < c.random_impulse_hz * dt:
            self.fire_impulse(
                force=self.rng.uniform(0.3, 1.0) * c.random_impulse_max,
                direction=self.rng.uniform(-np.pi, np.pi),
                duration=self.rng.uniform(0.03, 0.12),
            )
            return True
        return False

    # ------------------------------------------------------------------
    def wrench(self, heading: float, dt: float, com_height: float) -> ExternalWrench:
        """当前拍在车体系下的外部力旋量。
        External wrench in the body frame for the current tick."""
        c = self.cfg
        fx = fy = 0.0

        # 在世界坐标系里定义的恒定「风」-> 转到车体系
        # constant "wind" defined in world coordinates -> body frame
        if c.wind_force != 0.0:
            rel = c.wind_dir - heading
            fx += c.wind_force * np.cos(rel)
            fy += c.wind_force * np.sin(rel)

        # 脚本化/手动冲击（已经在车体系下）
        # scripted / manual impulse (already in the body frame)
        if self._impulse_left > 0.0:
            fx += self._impulse_vec[0]
            fy += self._impulse_vec[1]
            self._impulse_left = max(0.0, self._impulse_left - dt)

        # 在质心高度上的水平推力同时会把机器人推倒：真正让一次推搡变得有
        # 意思的，是它带来的俯仰力矩。
        # A horizontal push at COM height also tips the robot: the pitch
        # moment is what actually makes a shove interesting.
        return ExternalWrench(fx=fx, fy=fy,
                              pitch_moment=-fx * com_height,
                              yaw_moment=0.0)

    @property
    def impulse_active(self) -> bool:
        return self._impulse_left > 0.0


# ----------------------------------------------------------------------
def sample_training_disturbance(rng: np.random.Generator,
                                difficulty: float = 1.0) -> DisturbanceConfig:
    """训练时用的域随机化。
    Domain randomisation used while training.

    ``difficulty`` 取值 [0, 1]，由课程学习逐步抬高，让智能体先在干净条件下学会
    站起来，然后才去学如何在一个有噪声、有延迟、有风的世界里活下来。

    ``difficulty`` in [0, 1] is ramped up by the curriculum so the agent first
    learns to stand up in clean conditions and only then learns to survive a
    noisy, laggy, wind-blown world.
    """
    d = float(np.clip(difficulty, 0.0, 1.0))
    return DisturbanceConfig(
        noise_pitch=rng.uniform(0.0, 0.012) * d,
        noise_pitch_rate=rng.uniform(0.0, 0.10) * d,
        noise_vel=rng.uniform(0.0, 0.05) * d,
        noise_yaw_rate=rng.uniform(0.0, 0.06) * d,
        imu_bias_walk=rng.uniform(0.0, 0.004) * d,
        torque_noise=rng.uniform(0.0, 0.06) * d,
        torque_scale_err=rng.uniform(0.0, 0.15) * d,
        latency_steps=int(rng.integers(0, 1 + int(4 * d))),
        wind_force=rng.uniform(0.0, 1.2) * d,
        wind_dir=rng.uniform(-np.pi, np.pi),
        ground_slip=rng.uniform(0.0, 0.25) * d,
        # 刻意调得足够大，让固定的标称 PID 有时候会摔——如果基线从不失败，
        # 增益调度就没有任何东西可赢。
        # deliberately large enough that the fixed nominal PID falls over
        # sometimes -- if the baseline never fails there is nothing for gain
        # scheduling to win
        random_impulse_hz=rng.uniform(0.0, 0.8) * d,
        random_impulse_max=rng.uniform(4.0, 20.0) * d,
    )
