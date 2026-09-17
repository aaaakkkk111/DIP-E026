"""MPU6050 的信号链路，按固件真正在跑的样子。
The MPU6050 signal chain, as the firmware actually runs it.

在此之前孪生用一个单极点（``imu_filter_tau = 10 ms``）近似板载姿态滤波。那只是
个占位实现，读了出厂源码才发现它连形状都是错的。

``main.c`` 设了 ``GET_Angle_Way = 2``，所以 ``Get_Angle()`` 走的是这一支::

    Pitch = KF_X(accel_y, accel_z, -gyro_x) / PI * 180;

``KF_X`` 在 ``6.LQR/STM32_code/APP/KF/KF.c``，是一个教科书式的两状态卡尔曼
滤波器——角度和陀螺零偏——以 Ts = 5 ms 积分::

    A = [[1, -Ts], [0, 1]]     B = [[Ts], [0]]      C = [[1, 0]]
    Q = 1e-10 * I              R = 1e-4             y = atan2(-accel_y, accel_z)

注意这个调参。``Q`` 比 ``R`` 小*七个数量级*，所以滤波器几乎完全不信加速度计。
把 Riccati 递推迭代到收敛，得到

    K_inf = [0.003311, -0.000998]        估计器极点 |z| = 0.998343
                                         时间常数     = 3.0 s

这才是关键数字。对任何快于几秒的东西，这个滤波器就是一个**纯陀螺积分器**；
加速度计只修正缓慢的零偏。由此有三个后果，旧的单极点模型三个全搞错了：

1.  俯仰角上根本没有值得一提的滞后。那个 10 ms 极点是在凭空发明一个真板子
    并不存在的相位损失。
2.  因此 ``app_control.c`` 里的 ``gyro_x = (angle_x - last_angle) * 200``
    实际上*几乎把陀螺本身还原出来了*，而不是在对一个滞后信号求导。LQR 版本
    那个「假的」俯仰角速度，比看上去好得多。
3.  这个滤波器真正差的地方是陀螺零偏：3 秒的修正时间意味着漂移的陀螺会直接
    走进角度估计，固件于是绕着一个错误的零点去平衡。这——而不是滞后——才是
    这颗 IMU 真正的失效模式。

下面复现的是精确的递推（连瞬态一起——``P`` 从单位阵开始，和 C 的静态初始化
完全一致，头几拍的增益接近 1，这就是为什么上电时估计值是「啪」地贴到加速度计
读数上，而不是慢慢爬上去）。

Until now the twin approximated the on-board attitude filter with a single
pole (``imu_filter_tau = 10 ms``).  That was a placeholder, and reading the
shipped source shows it was the wrong shape entirely.

``main.c`` sets ``GET_Angle_Way = 2``, so ``Get_Angle()`` takes this branch::

    Pitch = KF_X(accel_y, accel_z, -gyro_x) / PI * 180;

``KF_X`` lives in ``6.LQR/STM32_code/APP/KF/KF.c`` and is a textbook two-state
Kalman filter -- angle and gyro bias -- integrated at Ts = 5 ms::

    A = [[1, -Ts], [0, 1]]     B = [[Ts], [0]]      C = [[1, 0]]
    Q = 1e-10 * I              R = 1e-4             y = atan2(-accel_y, accel_z)

Note the tuning.  ``Q`` is *seven orders of magnitude* below ``R``, so the
filter barely believes the accelerometer at all.  Iterating the Riccati
recursion to convergence gives

    K_inf = [0.003311, -0.000998]        estimator poles |z| = 0.998343
                                         time constant   = 3.0 s

which is the number that matters.  The filter is, for anything faster than a
few seconds, a **pure gyro integrator**; the accelerometer only trims slow
bias.  Three consequences, all of which the old one-pole model got wrong:

1.  There is no meaningful lag on the pitch angle.  The 10 ms pole was
    inventing a phase loss the real board does not have.
2.  ``app_control.c``'s ``gyro_x = (angle_x - last_angle) * 200`` therefore
    very nearly *recovers the gyro itself*, rather than differentiating a
    lagged signal.  The LQR build's "fake" pitch rate is much better than it
    looks.
3.  What the filter is genuinely bad at is gyro bias: with a 3 s correction,
    a drifting gyro walks straight into the angle estimate and the firmware
    then balances to a wrong zero.  That, not lag, is this IMU's real failure
    mode.

The exact recursion is reproduced below (transient included -- ``P`` starts at
the identity exactly as the C static initialiser does, and the first few ticks
have gains near 1, which is why the estimate snaps to the accelerometer on
power-up instead of ramping).
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------
# 量程标度，来自 mpu6050.c：MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000)
# 和 MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2)，在 Get_Angle() 里以
# ``gyro/939.8`` 和 ``accel/1671.84`` 读回。
#
# Scaling, from mpu6050.c: MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000)
# and MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2), read back in
# Get_Angle() as ``gyro/939.8`` and ``accel/1671.84``.
# --------------------------------------------------------------------------
GYRO_LSB_PER_RAD_S = 939.8       # +-2000 deg/s  ->  16.4 LSB/(deg/s)
ACCEL_LSB_PER_MS2 = 1671.84      # +-2 g         ->  16384 LSB/g, g = 9.8
GYRO_MAX_LSB = 32767
ACCEL_MAX_LSB = 32767

# --------------------------------------------------------------------------
# 片上数字低通滤波器。
#
# ``MPU6050_initialize()`` 从没碰过 CONFIG 寄存器，但 bsp.c 里紧接着就跑
# ``DMP_Init()``，它调用 ``mpu_set_sample_rate(200)``（DEFAULT_MPU_HZ = 200，
# mpu6050.c:10）。在 inv_mpu.c 内部这最终变成
#
#     mpu_set_lpf(st.chip_cfg.sample_rate >> 1)     -> mpu_set_lpf(100)
#     ... 100 >= 98  ->  INV_FILTER_98HZ
#
# 所以 5 ms 中断读到的寄存器**不是原始值**：陀螺和加速度计都在片上被滤过
# （98 Hz / 94 Hz，按数据手册群延迟 2.8 ms / 3.0 ms），并以 200 Hz 更新。
# 孪生原来两者都没建模，这就是为什么它仿真出的加速度计比台架上的噪得多——
# 所有 100 Hz 以上的机架抖动都直接混叠进了卡尔曼滤波器的新息，而不是被滤掉。
#
# The on-chip digital low-pass filter.
#
# ``MPU6050_initialize()`` never touches the CONFIG register, but ``DMP_Init()``
# runs straight afterwards in bsp.c and calls ``mpu_set_sample_rate(200)``
# (DEFAULT_MPU_HZ = 200, mpu6050.c:10).  Inside inv_mpu.c that ends with
#
#     mpu_set_lpf(st.chip_cfg.sample_rate >> 1)     -> mpu_set_lpf(100)
#     ... 100 >= 98  ->  INV_FILTER_98HZ
#
# so the registers the 5 ms interrupt reads are NOT raw: gyro and accelerometer
# are both filtered on-chip (98 Hz / 94 Hz, 2.8 ms / 3.0 ms group delay per the
# datasheet) and updated at 200 Hz.  The twin had modelled neither, which is
# why its simulated accelerometer used to be far noisier than the bench one --
# every bit of chassis chatter above 100 Hz aliased straight into the Kalman
# filter's innovation instead of being filtered away.
# --------------------------------------------------------------------------
DLPF_GYRO_HZ = 98.0
DLPF_ACCEL_HZ = 94.0
DLPF_GYRO_DELAY_S = 0.0028
DLPF_ACCEL_DELAY_S = 0.0030
MPU_SAMPLE_HZ = 200.0

# KF.c
KF_TS = 0.005
KF_Q = 1e-10
KF_R = 1e-4


class MPU6050Kalman:
    """KF.c 里的 ``KF_X()``，逐个状态复现。
    ``KF_X()`` from KF.c, state for state.

    ``update(accel_angle_rad, gyro_rad_s)`` 是一拍 5 ms，返回后验角度（弧度）
    ——也就是固件转成度数后存进 ``Angle_Balance`` 的那个值。

    ``update(accel_angle_rad, gyro_rad_s)`` is one 5 ms tick and returns the
    posterior angle in radians -- the value the firmware converts to degrees
    and stores in ``Angle_Balance``.
    """

    __slots__ = ("ts", "x", "P", "A", "B", "C", "Q", "R", "_k_last",
                 "warm_start", "_P0")

    def __init__(self, ts: float = KF_TS, q: float = KF_Q, r: float = KF_R,
                 warm_start: bool = True):
        self.ts = float(ts)
        self.A = np.array([[1.0, -self.ts], [0.0, 1.0]])
        self.B = np.array([[self.ts], [0.0]])
        self.C = np.array([[1.0, 0.0]])
        self.Q = np.eye(2) * q
        self.R = np.array([[r]])
        # C 代码把 P 初始化为单位阵，从那里开始增益要大约 5 秒挂钟时间才稳定。
        # 在真板子上这个瞬态只发生一次——上电那一下，车还平躺在台面上；等有人
        # 去开它的时候滤波器早就收敛了。如果每局复位都重启它，就等于给仿真里的
        # 车一个「短暂地信任加速度计」的滤波器，而真车从来没有过这种状态，所以
        # ``warm_start`` 直接把 P 播种到收敛值。想看上电瞬态本身就设成 False。
        #
        # The C code initialises P to the identity, and from there the gain
        # takes about 5 s of wall clock to settle.  On the real board that
        # transient happens once, at power-up, while the car is still lying
        # on the bench -- by the time anyone drives it the filter has long
        # converged.  Restarting it at every episode reset would hand the
        # simulated car a briefly accelerometer-trusting filter that the real
        # one never has, so ``warm_start`` seeds P at its converged value.
        # Set it False to watch the power-up transient itself.
        self.warm_start = bool(warm_start)
        self._P0 = self._converged_P() if warm_start else np.eye(2)
        self.reset()

    def _converged_P(self, iters: int = 20000) -> np.ndarray:
        P = np.eye(2)
        C = self.C
        for _ in range(iters):
            P_minus = self.A @ P @ self.A.T + self.Q
            S = C @ P_minus @ C.T + self.R
            K = P_minus @ C.T / S[0, 0]
            P = (np.eye(2) - K @ C) @ P_minus
        return P

    def reset(self):
        # 对应 C 里的 / C: static float x_hat[2][1] = {0};  p_hat = {{1,0},{0,1}}
        self.x = np.zeros((2, 1))
        self.P = self._P0.copy()
        self._k_last = np.zeros(2)

    def seed(self, angle: float, bias: float = 0.0):
        """把滤波器放到一个已经收敛的状态该在的位置。
        Put the filter where a converged one would already be.

        一局复位不等于断电重启：在台架上，板子早在车被扶起来之前就一直在跑
        （滤波器也一直在跟踪）。如果改成从零开始估计，就等于塞给控制器一个
        几度的姿态误差、还要 3 秒才能修回来——这是个仿真假象，而且很大，
        光这一条就足以把 LQR 那版掀翻。

        Episode reset is not a power cycle: on the bench the board has been
        running (and the filter tracking) long before the car is stood up.
        Starting the estimate at zero instead would hand the controller a
        several-degree attitude error with a 3 s correction, which is a
        simulator artefact and a large one -- it is enough on its own to tip
        the LQR build over.
        """
        self.x[0, 0] = float(angle)
        self.x[1, 0] = float(bias)

    # ------------------------------------------------------------------
    def update(self, accel_angle: float, gyro: float) -> float:
        u = np.array([[float(gyro)]])
        x_minus = self.A @ self.x + self.B @ u
        P_minus = self.A @ self.P @ self.A.T + self.Q

        S = self.C @ P_minus @ self.C.T + self.R
        K = P_minus @ self.C.T / S[0, 0]

        innov = float(accel_angle) - float((self.C @ x_minus)[0, 0])
        self.x = x_minus + K * innov
        self.P = (np.eye(2) - K @ self.C) @ P_minus
        self._k_last = K.ravel().copy()
        return float(self.x[0, 0])

    # ------------------------------------------------------------------
    @property
    def angle(self) -> float:
        return float(self.x[0, 0])

    @property
    def bias(self) -> float:
        """滤波器自己估计的陀螺零偏，rad/s。
        The filter's own estimate of the gyro bias, rad/s."""
        return float(self.x[1, 0])

    @property
    def gain(self) -> np.ndarray:
        return self._k_last

    # ------------------------------------------------------------------
    @classmethod
    def steady_state(cls, ts: float = KF_TS, q: float = KF_Q, r: float = KF_R,
                     iters: int = 20000):
        """(K_inf, |极点|, 时间常数) —— C 滤波器最终收敛到的值。
        (K_inf, |pole|, tau) -- what the C filter converges to."""
        f = cls(ts, q, r)
        C = f.C
        K = np.zeros((2, 1))
        for _ in range(iters):
            P_minus = f.A @ f.P @ f.A.T + f.Q
            S = C @ P_minus @ C.T + f.R
            K = P_minus @ C.T / S[0, 0]
            f.P = (np.eye(2) - K @ C) @ P_minus
        pole = float(np.max(np.abs(np.linalg.eigvals(
            (np.eye(2) - K @ C) @ f.A))))
        return K.ravel(), pole, -ts / np.log(pole)


# --------------------------------------------------------------------------
def accel_pitch_angle(theta: float, theta_dot: float = 0.0,
                      theta_ddot: float = 0.0, v_dot: float = 0.0,
                      l_com: float = 0.0383, g: float = 9.81) -> float:
    """``atan2(-accel_y, accel_z)`` 实际读到什么，含比力项。
    What ``atan2(-accel_y, accel_z)`` reads, including specific force.

    MPU6050 装在轮轴上方 ``l_com`` 处，量的是比力（proper acceleration），所以
    它*并不*只报告重力矢量。把 IMU 的加速度写成「轮轴的加速度 + 摆绕轮轴转动
    的加速度」，再投影到车体轴上，固件用到的那两路就是

        accel_y = v_dot*cos(theta) + l*theta_ddot - g*sin(theta)
        accel_z = v_dot*sin(theta) - l*theta_dot**2 + g*cos(theta)

    也就是说，加速度计对「哪边是下」的判断，恰恰在车正在加速时是错的——而在
    平衡车上，加速的时候正是它在做修正的时候。这就是加速度计必须被「慢慢
    相信」的经典原因，也是为什么固件那个 ``R >> Q`` 站得住脚，哪怕由此得到的
    3 秒时间常数在纸面上看着荒唐。

    当 ``theta_ddot = v_dot = 0`` 时，这个式子退化成 ``atan2(g sin, g cos)``
    = ``theta``，即一个静止、装配完美的 IMU。

    The MPU6050 sits ``l_com`` above the axle and measures proper
    acceleration, so it does *not* report the gravity vector alone.  Writing
    the IMU's acceleration as the axle's plus the pendulum's rotation about
    it, and projecting onto the body axes, the two channels the firmware uses
    come out as

        accel_y = v_dot*cos(theta) + l*theta_ddot - g*sin(theta)
        accel_z = v_dot*sin(theta) - l*theta_dot**2 + g*cos(theta)

    so the accelerometer's idea of "down" is wrong exactly when the car is
    accelerating -- which, on a balance car, is precisely when it is
    correcting.  This is the classic reason the accelerometer must be trusted
    slowly, and it is why the firmware's ``R >> Q`` is defensible even though
    the resulting 3 s time constant looks absurd on paper.

    With ``theta_ddot = v_dot = 0`` this collapses to ``atan2(g sin, g cos)``
    = ``theta``, i.e. a static, perfectly-mounted IMU.
    """
    s, c = np.sin(theta), np.cos(theta)
    a_y = v_dot * c + l_com * theta_ddot - g * s
    a_z = v_dot * s - l_com * theta_dot ** 2 + g * c
    return float(np.arctan2(-a_y, a_z))


def accel_pitch_angle_from_sensor(a_fwd: float, a_up: float) -> float:
    """同一个读数，但直接来自实测的车体系加速度计。
    Same reading, but straight from a measured body-frame accelerometer.

    ``a_fwd`` / ``a_up`` 是沿车体前向轴和上向轴的比力分量（也就是 MuJoCo 在
    ``imu`` 站点的 ``accelerometer`` 传感器返回的 ``acc[0]`` 和 ``acc[2]``）。
    固件的 ``accel_y`` 对应前向那一路，``accel_z`` 对应上向那一路。

    ``a_fwd`` / ``a_up`` are the specific-force components along the chassis'
    forward and up axes (what MuJoCo's ``accelerometer`` sensor at the ``imu``
    site returns as ``acc[0]`` and ``acc[2]``).  The firmware's ``accel_y`` is
    the forward channel and ``accel_z`` the up channel.
    """
    return float(np.arctan2(-a_fwd, a_up))


# --------------------------------------------------------------------------
def quantize_gyro(rate_rad_s: float) -> int:
    """rad/s -> 固件读到的带符号 16 位寄存器值。
    rad/s -> the signed 16-bit register value the firmware reads.

    这里用四舍五入而不是截断：寄存器里存的是 ADC 结果，而 ADC 是舍入的。向零
    截断会让每个样本的幅值系统性地偏小最多半个 LSB，而 LQR 那版会对这一路做
    *积分*（两次——先经卡尔曼滤波器，再作为 x_pose 积一次），半个 LSB 的系统
    误差在 20 秒一局里值大约一度。向零截断对 C 里那些对**计算值**做的 ``(int)``
    强制转换是对的（见 EncoderQuantizer），但对**传感器读数**是错的。

    Round to nearest, not truncate: the register holds an ADC result, and an
    ADC rounds.  Truncating toward zero would bias every sample's magnitude
    down by up to half an LSB, and since the LQR build *integrates* this
    channel (twice, via the Kalman filter and then again as x_pose) a
    half-LSB systematic error is worth about a degree over a 20 s episode.
    Truncation toward zero is right for C's ``(int)`` casts on computed
    values -- see EncoderQuantizer -- but wrong for a sensor reading.
    """
    raw = int(np.rint(rate_rad_s * GYRO_LSB_PER_RAD_S))
    return int(np.clip(raw, -GYRO_MAX_LSB - 1, GYRO_MAX_LSB))


def dequantize_gyro(raw_lsb: int) -> float:
    """固件里的 ``Gyro_X / 939.8``。 / The firmware's ``Gyro_X / 939.8``."""
    return raw_lsb / GYRO_LSB_PER_RAD_S


def quantize_accel(a_ms2: float) -> int:
    raw = int(np.rint(a_ms2 * ACCEL_LSB_PER_MS2))
    return int(np.clip(raw, -ACCEL_MAX_LSB - 1, ACCEL_MAX_LSB))
# --------------------------------------------------------------------------
class OnePole:
    """离散单极点低通，按传进来的 dt 步进。
    Discrete one-pole low pass, stepped at whatever dt it is handed.

    用来建模 MPU6050 的片上 DLPF。芯片里的滤波器阶数比这个高，但一个按数据
    手册群延迟匹配过的单极点，把两件真正要紧的事做对了——它给控制回路带来的
    相位损失，以及机架抖动根本到不了寄存器这一事实。

    Used for the MPU6050's on-chip DLPF.  The chip's filter is higher order
    than this, but a single pole matched to the datasheet's group delay gets
    the two things that matter right -- the phase it costs the control loop,
    and the fact that chassis chatter never reaches the register at all.
    """

    __slots__ = ("tau", "y")

    def __init__(self, cutoff_hz: float = DLPF_GYRO_HZ,
                 delay_s: float | None = None):
        self.tau = float(delay_s) if delay_s is not None else             1.0 / (2.0 * np.pi * float(cutoff_hz))
        self.y = None

    def reset(self, y0=None):
        self.y = None if y0 is None else float(y0)

    def update(self, x: float, dt: float) -> float:
        x = float(x)
        if self.y is None:
            self.y = x
        else:
            a = dt / max(dt + self.tau, 1e-12)
            self.y += a * (x - self.y)
        return self.y

    @property
    def value(self) -> float:
        return 0.0 if self.y is None else self.y


def make_dlpf():
    """按芯片实际配置生成 (前向加速度, 上向加速度, 陀螺) 三个滤波器。
    (accel_fwd, accel_up, gyro) filters as the chip is configured."""
    return (OnePole(delay_s=DLPF_ACCEL_DELAY_S),
            OnePole(delay_s=DLPF_ACCEL_DELAY_S),
            OnePole(delay_s=DLPF_GYRO_DELAY_S))


# --------------------------------------------------------------------------
# 固件的另外两种取角方式
# The firmware's other two attitude algorithms
# --------------------------------------------------------------------------
# app_control.c 的 Get_Angle(way)：1 = DMP，2 = 卡尔曼，3 = 互补滤波。
# main.c 里 GET_Angle_Way = 2，所以**出厂默认是卡尔曼**——上面那个
# MPU6050Kalman 就是它，逐状态复现自 KF.c。
#
# Get_Angle(way) in app_control.c: 1 = DMP, 2 = Kalman, 3 = complementary.
# main.c ships GET_Angle_Way = 2, so Kalman is the factory default.
COMP_K1 = 0.02          # filter.c: float K1 = 0.02
COMP_DT = 0.005         # filter.c: float dt = 0.005，每 5 ms 一拍


class MPU6050Complementary:
    """filter.c 的 ``Complementary_Filter_x()``，一行公式照抄。

        angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt)

    K1 = 0.02、dt = 0.005，等效时间常数 dt*(1-K1)/K1 = **0.245 秒**——
    比卡尔曼那条 3.02 秒的快了十二倍。这不是实现差异，是这两种滤波在这台
    车上本来就有的取舍：互补跟得快但更信加速度计，car 一加速/刹车，加速度计
    读到的「重力方向」就被平移加速度污染，角度会跟着晃；卡尔曼慢但对那种
    污染不敏感。停车振荡那类工况下两者的差别会很明显。

    One-liner from filter.c.  Its effective time constant is 0.245 s against
    the Kalman's 3.02 s: it tracks far faster but leans on the accelerometer,
    whose "gravity" direction is corrupted by the car's own linear
    acceleration -- exactly what happens when braking.
    """

    __slots__ = ("k1", "dt", "angle")

    def __init__(self, k1: float = COMP_K1, dt: float = COMP_DT):
        self.k1 = float(k1)
        self.dt = float(dt)
        self.angle = None

    def reset(self):
        # C 里是 static float angle，上电为 0；这里用 None 表示「还没收到第一拍」，
        # 第一拍直接吃加速度角，免得从 0 慢慢爬上来。
        # The C static starts at 0; seeding from the first sample avoids an
        # artificial ramp that the real board only ever sees once at power-up.
        self.angle = None

    def seed(self, angle: float, bias: float = 0.0):
        self.angle = float(angle)

    def update(self, accel_angle: float, gyro: float) -> float:
        if self.angle is None:
            self.angle = float(accel_angle)
            return self.angle
        self.angle = (self.k1 * float(accel_angle)
                      + (1.0 - self.k1) * (self.angle + float(gyro) * self.dt))
        return self.angle


class MPU6050DMP:
    """MPU6050 片上 DMP 的**近似**，不是逐位复现。

    必须说清楚：DMP 是 InvenSense 的固件 blob，被 ``dmp_load_motion_driver_
    firmware()`` 灌进芯片，源码里只有那段二进制。所以这个类和 MPU6050Kalman
    的性质不同——后者是从 KF.c 一行行抄下来的，前者是按 DMP 的**已知行为**
    建的模型：

      * 200 Hz 六轴四元数融合（mpu6050.c: DEFAULT_MPU_HZ = 200，
        dmp_set_fifo_rate(200)），输出角度比互补滤波干净
      * 开了 DMP_FEATURE_GYRO_CAL / SEND_CAL_GYRO，**陀螺零偏被持续估计并扣掉**
        ——这是 DMP 相对另外两种最实质的优势，另外两种都直接吃原始陀螺
      * 芯片内定点运算 + FIFO 取数，有约一拍的固有延迟

    这里用「带零偏估计的互补融合」来近似，时间常数取 0.5 秒（介于互补的
    0.245 和卡尔曼的 3.02 之间，符合 DMP 实测的响应），零偏用慢积分估计。
    **凡是拿 DMP 做出来的结论都要记住这一层近似**，不能当成和卡尔曼同等可信。

    An approximation, NOT a bit-level reproduction: the DMP is a binary blob
    uploaded to the chip, so only its documented behaviour can be modelled --
    200 Hz 6-axis quaternion fusion, continuous gyro bias calibration (the one
    real advantage over the other two, which both eat the raw gyro), and about
    one tick of pipeline delay.
    """

    __slots__ = ("dt", "tau", "tau_bias", "angle", "bias", "_delayed")

    def __init__(self, dt: float = COMP_DT, tau: float = 0.5,
                 tau_bias: float = 20.0):
        self.dt = float(dt)
        self.tau = float(tau)
        self.tau_bias = float(tau_bias)
        self.angle = None
        self.bias = 0.0
        self._delayed = None

    def reset(self):
        self.angle = None
        self.bias = 0.0
        self._delayed = None

    def seed(self, angle: float, bias: float = 0.0):
        self.angle = float(angle)
        self.bias = float(bias)
        self._delayed = float(angle)

    def update(self, accel_angle: float, gyro: float) -> float:
        a_ang = float(accel_angle)
        if self.angle is None:
            self.angle = a_ang
            self._delayed = a_ang
            return a_ang
        # 零偏估计：把「加速度角和积分角的长期差」慢慢归到陀螺零偏上
        err = a_ang - self.angle
        self.bias -= (self.dt / self.tau_bias) * err
        rate = float(gyro) - self.bias
        k = self.dt / (self.dt + self.tau)
        self.angle = (1.0 - k) * (self.angle + rate * self.dt) + k * a_ang
        out = self._delayed          # FIFO 取数的一拍延迟 / one-tick pipeline
        self._delayed = self.angle
        return out
