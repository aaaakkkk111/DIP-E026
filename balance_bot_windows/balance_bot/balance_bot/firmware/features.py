"""情境特征：策略靠这 8 个数判断「现在是什么局面」。
Situation features -- the eight numbers a gain-scheduling policy reads.

**这个文件的每一行都要能原样搬到 STM32 上。** 所以：

* 只用 IIR（一阶 EMA + 一个二阶带通），不用 FFT、不留历史缓冲；
* 状态一共 9 个标量，定点化之后 18 字节 RAM；
* 每拍的算力是十几次乘加，200 Hz 下可以忽略。

固件那边看得见的原始量只有四个：卡尔曼角度（度）、陀螺原始 LSB、两个加速度
通道、编码器计数，外加自己写出去的 CCR。特征全部从这些量导出，不依赖任何
仿真才有的真值。

Every line here must port to the STM32 verbatim: IIR only (one biquad plus
first-order EMAs), nine scalars of state, a dozen multiply-accumulates per
tick.  The features are derived only from what the firmware can actually see:
the Kalman angle, the raw gyro, the two accelerometer channels, the encoder
counts and the CCR it wrote itself.

**为什么是这 8 个**（顺序即 `NAMES`）：

1. ``bp31``   —— 31 Hz 带通能量。项目早先的结论：负重本身测不出来，但「该用
   哪档增益」能测，而且**必须带通，高通会把死区极限环一起收进来**。
2. ``gyro``   —— |陀螺| 的慢 EMA，激励水平。静止约 9 °/s，被撞时几十上百。
3. ``shock``  —— |陀螺| 的快峰值保持，冲击/磕碰的即时指示。
4. ``freefall`` —— |a|/g - 1。悬空时趋近 -1，落地瞬间冲正。从台阶掉下去
   这一类事件只有这个通道看得见。
5. ``tilt``   —— 卡尔曼角度的慢 EMA。坡道不会直接显形（加速度计测的是比力，
   在坡上静止时"倾角"依然相对重力），但爬坡时车要前倾才能推得动，这个慢
   平均就是坡度的代理量。
6. ``venc``   —— 编码器速度 EMA（m/s），在跑还是站着。
7. ``vint``   —— 速度环积分归一化。坡道、持续外力、负载拖拽都在这里积起来，
   是"车正在被什么东西拽着"的唯一直接证据。
8. ``duty``   —— |CCR| 的 EMA 除以满量程，当前用掉多少力矩权限。负重和爬坡
   都会把它顶上去。

Why these eight: load is not directly identifiable but the *gain mode* is, via
a band-pass (never a high-pass -- that would fold in the dead-band limit
cycle); the rest cover excitation level, impact, free fall, slope, motion,
accumulated external load and torque headroom.
"""
from __future__ import annotations

import numpy as np

NAMES = ("bp31", "gyro", "shock", "freefall", "tilt", "venc", "vint", "duty")
N_FEATURES = len(NAMES)

# 200 Hz 控制拍 / the 200 Hz control tick
FS = 200.0
# 31 Hz 带通，Q=3。项目里「该用哪档」的检测器就用这个频点，沿用它。
# 31 Hz band-pass, Q = 3 -- the frequency the load-mode detector already uses.
BP_F0 = 31.0
BP_Q = 3.0
# EMA 时间常数（秒）/ EMA time constants
TAU_SLOW = 0.50        # bp31 / gyro / duty / venc
TAU_TILT = 1.50        # 坡度代理量要更慢，别把一次摆动当成坡
TAU_SHOCK = 0.12       # 峰值保持的衰减
# 归一化尺度：让每个特征在常见工况下落在 [-1, 1] 附近，PPO 不用再学缩放。
# Normalisation so every feature sits near [-1, 1] in ordinary conditions.
SCALE_BP = 40.0        # °/s
SCALE_GYRO = 60.0      # °/s
SCALE_SHOCK = 200.0    # °/s
SCALE_TILT = 10.0      # 度 / deg
SCALE_VENC = 0.8       # m/s
SCALE_VINT = 8000.0    # 固件的积分限幅 / the firmware's integral clamp
PWM_FULL = 2880.0
G = 9.81


def _biquad_bandpass(f0: float, q: float, fs: float):
    """RBJ 常数峰值增益带通。返回 (b0, b1, b2, a1, a2)，a0 已归一。
    RBJ constant-peak-gain band-pass; a0 normalised out."""
    w0 = 2.0 * np.pi * f0 / fs
    alpha = np.sin(w0) / (2.0 * q)
    b0, b1, b2 = alpha, 0.0, -alpha
    a0, a1, a2 = 1.0 + alpha, -2.0 * np.cos(w0), 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)


BP_COEF = _biquad_bandpass(BP_F0, BP_Q, FS)


class SituationFeatures:
    """8 个情境特征的在线提取器。一个实例 = 一台车。
    Online extractor; one instance per car."""

    def __init__(self, fs: float = FS):
        self.fs = float(fs)
        dt = 1.0 / self.fs
        self.a_slow = float(np.exp(-dt / TAU_SLOW))
        self.a_tilt = float(np.exp(-dt / TAU_TILT))
        self.a_shock = float(np.exp(-dt / TAU_SHOCK))
        self.reset()

    def reset(self):
        # 带通的两个延迟单元 / the band-pass delay line
        self._x1 = self._x2 = 0.0
        self._y1 = self._y2 = 0.0
        # 五个 EMA + 一个峰值保持 / five EMAs plus a peak hold
        self._bp = 0.0
        self._gyro = 0.0
        self._shock = 0.0
        self._tilt = 0.0
        self._venc = 0.0
        self._duty = 0.0

    # ------------------------------------------------------------------
    def update(self, gyro_dps: float, accel_fwd: float, accel_up: float,
               angle_deg: float, v_enc: float, ccr: float,
               vel_integral: float) -> np.ndarray:
        """喂一拍固件量，返回归一化后的 8 个特征。

        gyro_dps     俯仰角速度，度/秒（固件的 Gyro_Balance / 16.4）
        accel_fwd    加速度计前向通道，m/s^2
        accel_up     加速度计竖直通道，m/s^2
        angle_deg    卡尔曼角度，度（Angle_Balance）
        v_enc        编码器速度，m/s
        ccr          本拍写出去的比较值（两轮平均）
        vel_integral 速度环积分器当前值（固件的 Encoder_Integral）
        """
        g = float(gyro_dps)
        # 1) 31 Hz 带通 -> 整流 -> 慢 EMA
        x0 = g
        b0, b1, b2, a1, a2 = BP_COEF
        y0 = b0 * x0 + b1 * self._x1 + b2 * self._x2 - a1 * self._y1 - a2 * self._y2
        self._x2, self._x1 = self._x1, x0
        self._y2, self._y1 = self._y1, y0
        self._bp = self.a_slow * self._bp + (1.0 - self.a_slow) * abs(y0)

        # 2) 激励水平
        self._gyro = self.a_slow * self._gyro + (1.0 - self.a_slow) * abs(g)

        # 3) 冲击：峰值保持，只允许按 TAU_SHOCK 衰减
        self._shock = max(abs(g), self.a_shock * self._shock)

        # 4) 悬空 / 落地：比力模长相对 1 g 的偏差。悬空 -> -1，落地 -> 冲正。
        amag = float(np.hypot(accel_fwd, accel_up))
        freefall = amag / G - 1.0

        # 5) 坡度代理量：角度的慢平均
        self._tilt = self.a_tilt * self._tilt + (1.0 - self.a_tilt) * float(angle_deg)

        # 6) 走多快
        self._venc = self.a_slow * self._venc + (1.0 - self.a_slow) * float(v_enc)

        # 7) 速度环积分（不滤波，它自己已经是积分量）
        vint = float(vel_integral)

        # 8) 力矩权限用掉多少
        self._duty = (self.a_slow * self._duty
                      + (1.0 - self.a_slow) * abs(float(ccr)) / PWM_FULL)

        return np.array([
            self._bp / SCALE_BP,
            self._gyro / SCALE_GYRO,
            self._shock / SCALE_SHOCK,
            freefall,
            self._tilt / SCALE_TILT,
            self._venc / SCALE_VENC,
            vint / SCALE_VINT,
            self._duty,
        ], dtype=np.float32)

    # ------------------------------------------------------------------
    @staticmethod
    def zeros() -> np.ndarray:
        return np.zeros(N_FEATURES, dtype=np.float32)

    def c_constants(self) -> dict[str, float]:
        """导出到 C 的常数，`scripts/export_nn_c.py` 用。
        The constants the generated C needs."""
        b0, b1, b2, a1, a2 = BP_COEF
        return dict(bp_b0=b0, bp_b1=b1, bp_b2=b2, bp_a1=a1, bp_a2=a2,
                    a_slow=self.a_slow, a_tilt=self.a_tilt,
                    a_shock=self.a_shock,
                    s_bp=SCALE_BP, s_gyro=SCALE_GYRO, s_shock=SCALE_SHOCK,
                    s_tilt=SCALE_TILT, s_venc=SCALE_VENC, s_vint=SCALE_VINT,
                    pwm_full=PWM_FULL, g=G)
