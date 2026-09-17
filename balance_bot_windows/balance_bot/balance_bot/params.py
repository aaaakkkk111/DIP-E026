"""机器人 / 仿真 / 控制器的参数定义。
Robot / simulation / controller parameter definitions.

项目其余部分需要知道的一切——实体机器人、PID 增益搜索空间、扰动模型、奖励
权重——都放在这里，好让解析模型、MuJoCo 模型和 Gazebo 模型能从同一个事实源
保持一致。

Everything the rest of the project needs to know about the physical robot,
the PID gain search space, the disturbance model and the reward weights
lives here so that the analytic model, the MuJoCo model and the Gazebo
model can be kept consistent from a single source of truth.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict, replace
import numpy as np

GRAVITY = 9.81


# --------------------------------------------------------------------------
# 机器人 / Robot
# --------------------------------------------------------------------------
@dataclass
class RobotParams:
    """两轮倒立摆（「平衡车式」）机器人。
    Two-wheeled inverted pendulum ("Segway style") robot.

    俯仰角 ``theta`` 以竖直方向为零，正值表示前倾（朝车体系的 +x 方向）。
    Pitch angle ``theta`` is measured from the vertical, positive = leaning
    forward (towards +x of the body frame).
    """

    # 车体（轮轴以上的一切）/ body (everything above the wheel axle)
    m_body: float = 1.50          # kg
    l_com: float = 0.12           # m，轮轴到车体质心 / axle -> body COM
    I_body: float = 0.0080        # kg·m^2，绕质心的俯仰惯量 / about the COM, pitch axis
    I_yaw: float = 0.0100         # kg·m^2，绕竖直轴 / about the vertical axis

    # 轮子（单个）/ wheels (per wheel)
    m_wheel: float = 0.10         # kg
    # 电机转子惯量，**已折算到输出端**（乘了减速比的平方）。0 = 不建模。
    # Rotor inertia referred to the output shaft (times the gear ratio squared).
    I_rotor: float = 0.0          # kg·m^2
    r_wheel: float = 0.050        # m
    track: float = 0.160          # m，两个轮子触地点的间距 / distance between contacts

    # 驱动 / actuation
    tau_max: float = 0.60         # 单轮 N·m / N m per wheel
    wheel_speed_max: float = 60.0 # rad/s，电机空载转速（反电动势限制）/ free speed

    # 阻尼与损耗 / damping / losses
    b_wheel: float = 0.010        # N·m·s，轮端粘性 / viscous at the wheel
    b_pitch: float = 0.002        # N·m·s，车体<->轮子的关节摩擦 / joint friction
    b_yaw: float = 0.004          # N·m·s
    rolling_mu: float = 0.02      # 滚动阻力系数 / rolling resistance coefficient

    # 碰撞与渲染用的几何 / geometry used for collision + rendering
    body_height: float = 0.24     # m，长方体高度 / box height
    body_width: float = 0.10      # m，左右 / lateral
    body_depth: float = 0.08      # m，前后 / fore-aft
    collision_radius: float = 0.11
    # 方块中心相对轮轴的高度。0 表示沿用 l_com（老行为）。
    # 形心和质心不是一个点：质心取决于质量怎么分布，形心只看外形。用真实
    # 外形时这个差别决定方块底面伸到哪里，也就决定托不托底。
    # Box centre above the axle; 0 keeps the old behaviour of reusing l_com.
    # Centroid and centre of mass are different points, and with a real-sized
    # box the difference decides whether the chassis bottoms out.
    body_center: float = 0.0
    # UI 侧视图用的分层轮廓 ((名字, 前后 y0, y1, 高 z0, z1), ...)，原点在轮轴，
    # 单位米。只有这台 STM32 车有（从 STEP 解出来），通用机器人留空。不参与物理。
    # Per-part rectangles for the side view; STM32 car only, not used by physics.
    side_profile: tuple = ()
    # MuJoCo 视图用的零件盒子 ((名字, x0,x1, y0,y1, z0,z1), ...)，原点在轮轴，
    # 单位米。**只是外观**：不参与碰撞、不贡献质量和惯量。空则退回单个方块。
    # Part boxes for the MuJoCo view: appearance only, no collision or mass.
    part_boxes: tuple = ()

    # ---- 派生量 / derived ----------------------------------------------
    @property
    def box_center(self) -> float:
        """碰撞方块中心离轮轴的高度，米。/ Box centre above the axle, m."""
        return self.body_center if self.body_center > 0.0 else self.l_com

    def bottoms_out(self, pitch: float) -> bool:
        """车身方块的最低角有没有低于地面。/ Does the box corner reach ground?

        方块在车体系里中心 (0, c)、半长 d、半高 h，绕轮轴转 theta 之后最低点

            z_min = c*cos - d*|sin| - h*cos

        地面在轮轴下方 r_wheel 处。真车的底盘确实会托底，这不是仿真瑕疵：
        前后半长 75.8 mm 而轮子半径只有 33.5 mm。
        """
        import numpy as _np
        c = self.box_center
        d = 0.5 * self.body_depth
        h = 0.5 * self.body_height
        ct = _np.cos(pitch)
        st_ = abs(_np.sin(pitch))
        return bool(c * ct - d * st_ - h * ct < -self.r_wheel)

    @property
    def I_wheel(self) -> float:
        """单轮绕自身轴的转动惯量：轮盘 + 折算到输出端的电机转子。
        Wheel inertia about its axle: the disc plus the reflected rotor.

        转子那一项以前只写在 A 的注释里、公式里没有。它不是小量：520 电机的
        转子惯量约 7e-7 kg·m^2，经 30:1 折算到输出端要乘 30^2 = 900，得
        6.6e-4——是轮盘自身 2e-5 的**三十倍**。缺了它，轮子对高频力矩的响应
        比真车快得多，俯仰到极点、死区补偿让 PWM 整块翻转时，车身就会抖，而
        用户实测真车在极点不抖。
        Reflected rotor inertia was named in A's docstring but missing from the
        formula.  It dominates: ~7e-7 kg m^2 at the rotor times 30^2 is 6.6e-4,
        thirty times the disc's own 2e-5.
        """
        return 0.5 * self.m_wheel * self.r_wheel ** 2 + self.I_rotor

    @property
    def A(self) -> float:
        """等效平动质量（两个轮子 + 车体 + 转子惯量）。
        Effective translational mass (both wheels + body + rotor inertia)."""
        return (2.0 * self.m_wheel + self.m_body
                + 2.0 * self.I_wheel / self.r_wheel ** 2)

    @property
    def B(self) -> float:
        """俯仰/平动的耦合项 m*l。 / Pitch/translation coupling term m*l."""
        return self.m_body * self.l_com

    @property
    def C(self) -> float:
        """绕轮轴的俯仰惯量。 / Pitch inertia about the axle."""
        return self.I_body + self.m_body * self.l_com ** 2

    @property
    def omega_n(self) -> float:
        """倒立摆的自然（不稳定）频率，rad/s。
        Natural (unstable) frequency of the inverted pendulum, rad/s."""
        return float(np.sqrt(self.m_body * GRAVITY * self.l_com / self.C))

    def with_payload(self, mass: float, height: float | None = None):
        """挂上载重，返回重新算过质量/质心/惯量的一份参数。
        Attach a payload and return params with mass, COM and inertia redone.

        实车规格：空车约 1.0 kg，**最大负重 4 kg**——也就是总质量最多变 5 倍。
        载重通常压在顶层那块板上（雷达固定板，轮轴上方约 105 mm），所以它不只
        是加重量，还会把质心大幅抬高：4 kg 压在 105 mm 处，质心会被拉到接近
        载重自己的高度，而电机力矩一点没变。这是这台车最真实、也最被忽略的
        鲁棒性维度——之前的域随机化只动噪声、风和打滑，质量和质心是钉死的。

        质心按加权平均，俯仰惯量按平行轴定理搬到新质心：
            I_new = I_body + m_body*(l_body - l_new)^2 + m_pay*(l_pay - l_new)^2
        偏航惯量把载重当集中质量，只加它绕竖直轴的部分（假设载重不宽，
        对 I_yaw 的贡献小，这里保守地按底盘半宽估一个）。

        The car weighs ~1 kg empty and is rated for 4 kg, so a full load is 5x
        the mass -- and sitting on the top plate it also drags the centre of
        mass up to nearly its own height while the motor torque is unchanged.
        """
        m_pay = max(0.0, float(mass))
        if m_pay <= 0.0:
            return self
        l_pay = float(self.body_height + self.body_center - 0.5 *
                      self.body_height) if height is None else float(height)
        m_tot = self.m_body + m_pay
        l_new = (self.m_body * self.l_com + m_pay * l_pay) / m_tot
        i_new = (self.I_body
                 + self.m_body * (self.l_com - l_new) ** 2
                 + m_pay * (l_pay - l_new) ** 2)
        # 载重绕竖直轴：按底盘半宽当回转半径，宁可估小也不虚高
        r_gyr = 0.5 * self.body_width
        i_yaw = self.I_yaw + m_pay * r_gyr ** 2 / 3.0
        return replace(self, m_body=m_tot, l_com=l_new,
                       I_body=i_new, I_yaw=i_yaw)

    def randomized(self, rng: np.random.Generator, scale: float = 1.0):
        """域随机化：返回一份被扰动过的参数副本。
        Domain randomisation: returns a perturbed copy of the parameters."""
        if scale <= 0.0:
            return self

        def jitter(value, pct):
            return float(value * (1.0 + rng.uniform(-pct, pct) * scale))

        return replace(
            self,
            m_body=jitter(self.m_body, 0.25),
            l_com=jitter(self.l_com, 0.20),
            I_body=jitter(self.I_body, 0.30),
            I_yaw=jitter(self.I_yaw, 0.25),
            m_wheel=jitter(self.m_wheel, 0.20),
            r_wheel=jitter(self.r_wheel, 0.05),
            tau_max=jitter(self.tau_max, 0.15),
            b_wheel=jitter(self.b_wheel, 0.60),
            b_pitch=jitter(self.b_pitch, 0.60),
            b_yaw=jitter(self.b_yaw, 0.60),
            rolling_mu=jitter(self.rolling_mu, 0.60),
        )


# --------------------------------------------------------------------------
# 仿真时序 / Simulation timing
# --------------------------------------------------------------------------
@dataclass
class SimParams:
    dt_phys: float = 0.0025       # 400 Hz 积分（RK4）/ integration (RK4)
    dt_ctrl: float = 0.005        # 200 Hz PID
    dt_agent: float = 0.040       # 25 Hz，PPO 更新增益的节拍 / gain update by PPO
    episode_seconds: float = 20.0 # 一局时长（秒）/ episode length
    pitch_fail: float = 0.60      # rad，约 34 度 -> 判定为摔倒 / considered fallen

    @property
    def phys_per_ctrl(self) -> int:
        return max(1, int(round(self.dt_ctrl / self.dt_phys)))

    @property
    def dt_sub(self) -> float:
        """实际积分步长，保证能整除 ``dt_ctrl``。
        Actual integration step, guaranteed to tile ``dt_ctrl`` exactly.

        直接用 ``dt_phys``，一旦 ``dt_ctrl / dt_phys`` 不是整数，就会悄悄地
        丢掉（或重复）一段时间。

        Using ``dt_phys`` directly would silently lose (or duplicate) time
        whenever ``dt_ctrl / dt_phys`` is not an integer.
        """
        return self.dt_ctrl / self.phys_per_ctrl

    @property
    def ctrl_per_agent(self) -> int:
        return max(1, int(round(self.dt_agent / self.dt_ctrl)))

    @property
    def max_agent_steps(self) -> int:
        return int(round(self.episode_seconds / self.dt_agent))


# --------------------------------------------------------------------------
# PID 增益 / PID gains
# --------------------------------------------------------------------------
GAIN_NAMES = (
    "kp_pitch", "ki_pitch", "kd_pitch",
    "kp_vel", "ki_vel", "kd_vel",
    "kp_yaw", "ki_yaw", "kd_yaw",
)


@dataclass
class GainSpace:
    """PPO 智能体活动的搜索空间。
    Search space the PPO agent moves inside.

    智能体输出一个 ``[-1, 1]^9`` 的动作，含义是相对手调标称增益的乘性偏移
    （见下）。映射取指数形式，是因为控制增益天然就是乘性的——差两倍远比差一个
    绝对量重要得多。

    The agent emits an action in ``[-1, 1]^9`` which is a multiplicative
    deviation from the hand-tuned nominal gains (see below).  The mapping is
    exponential because control gains are naturally multiplicative -- a factor
    of two matters far more than an absolute offset.
    """

    # 标称值来自在线性化纵向模型上求解的连续时间 LQR（见
    # controller.lqr_reference_gains），再换算成等价的串级 PID 增益。
    #
    # 动作被参数化为**相对标称值**的乘性偏移：
    #
    #     gain = clip(nominal * span ** action,  low, high),  action ∈ [-1, 1]
    #
    # 所以 ``action = 0`` 恰好就是那套手调 PID。锚定在标称值上（而不是锚在
    # 对数区间的中点）比看上去重要得多：它让「什么都不做」成为那个好控制器，
    # 让奖励里的偏移惩罚有实际意义（||action||^2 = 离标称值多远），并且给一个
    # 半吊子策略能造成的破坏划了上界。
    #
    # The nominal values come from a continuous-time LQR solved on the
    # linearised longitudinal model (see controller.lqr_reference_gains) and
    # translated into the equivalent cascade-PID gains.
    #
    # The action is parameterised as a multiplicative deviation FROM NOMINAL:
    #
    #     gain = clip(nominal * span ** action,  low, high),  action in [-1, 1]
    #
    # so ``action = 0`` is exactly the hand-tuned PID.  Anchoring on nominal
    # rather than on the midpoint of a log range matters more than it looks:
    # it makes "do nothing" the good controller, it makes the deviation
    # penalty in the reward meaningful (||action||^2 = how far from nominal),
    # and it bounds the damage a half-trained policy can do.
    low: np.ndarray = field(default_factory=lambda: np.array([
        1.20, 0.00, 0.100,      # 俯仰环 pitch loop:    kp, ki, kd
        0.030, 0.000, 0.000,    # 速度环 velocity loop: kp, ki, kd
        0.010, 0.000, 0.000,    # 偏航环 yaw loop:      kp, ki, kd
    ]))
    high: np.ndarray = field(default_factory=lambda: np.array([
        9.00, 8.00, 1.200,
        0.350, 0.800, 0.060,
        0.200, 0.600, 0.012,
    ]))
    nominal: np.ndarray = field(default_factory=lambda: np.array([
        3.80, 2.00, 0.350,
        0.160, 0.150, 0.012,
        0.080, 0.200, 0.003,
    ]))
    # 智能体最多能把每个增益推多远，以倍率计
    # how far the agent may move each gain, as a multiplicative factor
    span: float = 2.5

    # 增益会低通滤波地趋向指令值，这样智能体没法把控制器搞到抖
    # gains are low-pass filtered towards the commanded value so the agent
    # cannot make the controller chatter
    slew_tau: float = 0.12        # s

    def action_to_gains(self, action: np.ndarray) -> np.ndarray:
        """把 [-1, 1]^9 的动作映射到物理增益。0 对应标称值。
        Map action in [-1, 1]^9 to physical gains.  0 -> nominal."""
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        return np.clip(self.nominal * self.span ** a, self.low, self.high)

    def gains_to_action(self, gains: np.ndarray) -> np.ndarray:
        g = np.clip(np.asarray(gains, dtype=float), self.low, self.high)
        r = np.maximum(g, 1e-9) / np.maximum(self.nominal, 1e-9)
        return np.clip(np.log(r) / np.log(self.span), -1.0, 1.0)

    def normalize(self, gains: np.ndarray) -> np.ndarray:
        """增益 -> [-1, 1]，供观测向量使用。
        Gains -> [-1, 1] for use inside the observation vector."""
        return self.gains_to_action(gains)

    @property
    def nominal_action(self) -> np.ndarray:
        """按构造就是零——见上面关于边界的说明。
        Zero, by construction -- see the note on the bounds above."""
        return np.zeros_like(self.nominal)


# --------------------------------------------------------------------------
# 扰动——这里每一个字段都在 UI 上有对应控件
# Disturbances -- every field here is exposed as a UI control
# --------------------------------------------------------------------------
@dataclass
class DisturbanceConfig:
    # --- 传感器噪声（高斯，每控制拍）/ sensor noise (gaussian, per tick) ---
    noise_pitch: float = 0.0          # rad，俯仰角 / pitch
    noise_pitch_rate: float = 0.0     # rad/s，俯仰角速度 / pitch rate
    noise_vel: float = 0.0            # m/s，速度 / velocity
    noise_yaw_rate: float = 0.0       # rad/s，偏航角速度 / yaw rate
    imu_bias_walk: float = 0.0        # rad/sqrt(s)，陀螺零偏缓慢漂移 / bias drift

    # --- 驱动 / actuation ---
    torque_noise: float = 0.0         # 相对 tau_max 的比例，加性 / additive
    torque_scale_err: float = 0.0     # 乘性增益误差，比例 / multiplicative error
    # 传感器测量延迟（固件拍）。**不是**那一拍控制延迟——控制延迟在
    # twin_baseline 的 PWM 输出侧，见 PWM_DELAY_TICKS。这里保持 0：
    # 编码器是定时器直接读的，不走 I2C，没有理由跟着 IMU 一起延迟。
    # Sensor-measurement delay, NOT the control delay: that one sits on
    # the PWM output (see PWM_DELAY_TICKS) and must not drag the encoder
    # path with it.
    latency_steps: int = 0            # 测量延迟 / measurement delay

    # --- 环境 / environment ---
    wind_force: float = 0.0           # N，沿世界系 +x 的恒定推力 / constant push
    wind_dir: float = 0.0             # rad，世界系 / world frame
    ground_slip: float = 0.0          # 0 = 完全抓地，1 = 完全打滑 / 0 grip, 1 none

    # --- 冲击（事件驱动，由 UI 或随机触发）/ impulses (event driven) ---
    impulse_force: float = 0.0        # N，一次脚本化踢击的幅值 / scripted kick
    impulse_duration: float = 0.05    # s，持续时间 / duration
    impulse_dir: float = 0.0          # rad，相对车头方向 / relative to heading
    random_impulse_hz: float = 0.0    # 自动踢击的泊松频率 / Poisson rate
    random_impulse_max: float = 0.0   # N，最大幅值 / max magnitude

    def to_dict(self):
        return asdict(self)


# --------------------------------------------------------------------------
# 奖励整形 / Reward shaping
# --------------------------------------------------------------------------
@dataclass
class RewardWeights:
    upright: float = 2.0          # 直立 / uprightness
    alive: float = 0.5            # 存活 / staying up
    vel_track: float = 2.5        # 速度跟踪。原为 1.2——稳态跟踪被存活/直立两项
    yaw_track: float = 0.6        # 淹没了 / was 1.2, drowned out by alive+upright
    torque: float = 0.05          # 力矩代价 / torque cost
    gain_rate: float = 0.40       # 原为 0.10——要认真罚增益抖动 / punish chatter
    gain_dev: float = 0.05        # 偏离标称值本身的代价，这样策略只在划算时
                                  # 才去动增益 / so gains move only when it pays
    pitch_rate: float = 0.05      # 俯仰角速度 / pitch rate
    obstacle: float = 1.0         # 障碍物 / obstacle
    fall_penalty: float = 40.0    # 摔倒惩罚 / falling
    collision_penalty: float = 15.0  # 碰撞惩罚 / collision


# --------------------------------------------------------------------------
# 场地 / Arena
# --------------------------------------------------------------------------
@dataclass
class ArenaParams:
    half_x: float = 4.0
    half_y: float = 4.0
    n_obstacles: int = 6
    obstacle_r_min: float = 0.15
    obstacle_r_max: float = 0.40
    n_rays: int = 8
    ray_max: float = 2.5
    safe_dist: float = 0.35       # start penalising below this clearance


DEFAULT_ROBOT = RobotParams()
DEFAULT_SIM = SimParams()
DEFAULT_GAINS = GainSpace()
DEFAULT_ARENA = ArenaParams()
