"""串级 PID 平衡控制器。
Cascade PID balance controller.

三个嵌套回路、九个增益——正是 PPO 智能体被允许动的那九个数：

    速度环（外环）  v_ref - v              -> 俯仰角参考
    俯仰环（内环）  theta - theta_ref      -> 总轮端力矩
    偏航角速度环    psi_dot_ref - psi_dot  -> 差动力矩

符号约定（很重要，而且很容易搞反）：

*   车体前倾时 ``theta`` 为正。
*   俯仰误差是 ``e = theta - theta_ref``，力矩是 ``tau = +Kp * e``。所以前倾
    会指令一个向前的轮端力矩，把轮子驱到正在倒下的车体*下方*。对轮式倒立摆
    来说，这才是正确的镇定符号。
*   要往前走必须先前倾，所以一个正的速度误差会要求一个正的俯仰角参考，内环
    随后把轮子追向前。这就是经典的非最小相位响应：机器人会先短暂后退，然后
    才向前加速。

Three nested loops, nine gains -- exactly the nine numbers the PPO agent is
allowed to move:

    velocity loop  (outer)  v_ref - v          -> pitch reference
    pitch loop     (inner)  theta - theta_ref  -> total wheel torque
    yaw-rate loop           psi_dot_ref - psi_dot -> differential torque

Sign convention (matters, and is easy to get backwards):

*   ``theta`` is positive when the body leans forward.
*   The pitch error is ``e = theta - theta_ref`` and the torque is
    ``tau = +Kp * e``.  Leaning forward therefore commands forward wheel
    torque, which drives the wheels *under* the falling body.  That is the
    correct stabilising sign for an inverted pendulum on wheels.
*   To move forward you must first lean forward, so a positive velocity
    error asks for a positive pitch reference.  The inner loop then chases
    the wheels forward.  This is the classic non-minimum-phase response:
    the robot briefly moves backwards before accelerating forward.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .params import GainSpace, GAIN_NAMES


@dataclass
class ControllerLimits:
    pitch_ref_max: float = 0.22       # rad, how far the robot may lean
    # 积分限幅是用*输出*单位表达的（也就是这一项实际贡献出去的单位），而不是
    # 加在原始累加器上。这一点很重要：如果直接钳原始累加器，一个很小的 Ki 会
    # 让积分项饱和在一个远不足以消除稳态误差的值上，PPO 智能体看到的就是一条
    # 莫名其妙、非单调的 Ki 响应曲线。
    #
    # Integral limits are expressed in *output* units (the units the term
    # actually contributes to), not on the raw accumulator.  This matters:
    # with a raw clamp, a small Ki makes the integral term saturate at a
    # value far too small to remove the steady-state error, and the PPO
    # agent then sees a nonsensical, non-monotonic response to Ki.
    i_pitch_out_max: float = 0.70     # N m of total wheel torque
    i_vel_out_max: float = 0.20       # rad of pitch reference
    i_yaw_out_max: float = 0.30       # N m of differential torque
    d_filter_tau: float = 0.02        # s, low-pass on the derivative terms
    vel_filter_tau: float = 0.06      # s, low-pass on the measured speed
    yaw_filter_tau: float = 0.02      # s, low-pass on the measured yaw rate


@dataclass
class ControlOutput:
    tau_l: float
    tau_r: float
    tau_sum: float
    tau_diff: float
    pitch_ref: float
    e_pitch: float
    e_vel: float
    e_yaw: float
    saturated: bool


class CascadePID:
    """有状态的串级 PID。每个机器人一个实例。
    Stateful cascade PID.  One instance per robot."""

    def __init__(self, gain_space: GainSpace | None = None,
                 limits: ControllerLimits | None = None,
                 tau_max: float = 0.6):
        self.gs = gain_space or GainSpace()
        self.lim = limits or ControllerLimits()
        self.tau_max = tau_max
        self.gains = self.gs.nominal.copy()
        self.gains_cmd = self.gs.nominal.copy()
        self.reset()

    # ------------------------------------------------------------------
    def reset(self, gains: np.ndarray | None = None):
        if gains is not None:
            self.gains = np.asarray(gains, dtype=float).copy()
            self.gains_cmd = self.gains.copy()
        self.i_pitch = 0.0
        self.i_vel = 0.0
        self.i_yaw = 0.0
        self.d_pitch = 0.0
        self.d_vel = 0.0
        self.d_yaw = 0.0
        self._prev_e_pitch = None
        self._prev_e_vel = None
        self._prev_e_yaw = None
        self._prev_pitch_ref = 0.0
        self.v_filt = 0.0
        self.yaw_filt = 0.0
        self._filt_init = False

    # ------------------------------------------------------------------
    def set_gain_command(self, gains: np.ndarray):
        """目标增益；实际增益会朝它们变化率受限地靠拢（防抖）。
        Target gains; the actual gains slew towards them (anti-chatter)."""
        self.gains_cmd = np.clip(np.asarray(gains, dtype=float),
                                 self.gs.low, self.gs.high)

    def slew_gains(self, dt: float):
        tau = max(self.gs.slew_tau, 1e-6)
        alpha = 1.0 - np.exp(-dt / tau)
        self.gains += alpha * (self.gains_cmd - self.gains)

    # ------------------------------------------------------------------
    @property
    def gain_dict(self):
        return {n: float(g) for n, g in zip(GAIN_NAMES, self.gains)}

    # ------------------------------------------------------------------
    def step(self, theta: float, theta_dot: float, v: float, yaw_rate: float,
             v_ref: float, yaw_rate_ref: float, dt: float) -> ControlOutput:
        g = self.gains
        kp_p, ki_p, kd_p, kp_v, ki_v, kd_v, kp_y, ki_y, kd_y = g
        lim = self.lim
        a_d = dt / max(dt + lim.d_filter_tau, 1e-9)   # derivative LPF alpha
        a_v = dt / max(dt + lim.vel_filter_tau, 1e-9)
        a_y = dt / max(dt + lim.yaw_filter_tau, 1e-9)

        if not self._filt_init:
            self.v_filt, self.yaw_filt = v, yaw_rate
            self._filt_init = True
        self.v_filt += a_v * (v - self.v_filt)
        self.yaw_filt += a_y * (yaw_rate - self.yaw_filt)

        # ---------------- 外环：速度 -> 俯仰角参考 -----------------------
        # ---------------- outer: velocity -> pitch reference -------------
        e_v = v_ref - self.v_filt
        self.i_vel = clamp_integral(self.i_vel + e_v * dt, ki_v, lim.i_vel_out_max)
        if self._prev_e_vel is None:
            self._prev_e_vel = e_v
        raw_d = (e_v - self._prev_e_vel) / max(dt, 1e-9)
        self.d_vel += a_d * (raw_d - self.d_vel)
        self._prev_e_vel = e_v

        pitch_ref = kp_v * e_v + ki_v * self.i_vel + kd_v * self.d_vel
        pitch_ref = float(np.clip(pitch_ref, -lim.pitch_ref_max, lim.pitch_ref_max))

        # ---------------- 内环：俯仰 -> 总力矩 ---------------------------
        # ---------------- inner: pitch -> total torque -------------------
        e_p = theta - pitch_ref
        self.i_pitch = clamp_integral(self.i_pitch + e_p * dt, ki_p,
                                      lim.i_pitch_out_max)
        # 微分作用在*测量值*上，避免 pitch_ref 跳变时产生一次冲击
        # derivative on the *measurement* to avoid a kick when pitch_ref jumps
        self.d_pitch += a_d * (theta_dot - self.d_pitch)
        self._prev_e_pitch = e_p

        tau_sum = kp_p * e_p + ki_p * self.i_pitch + kd_p * self.d_pitch

        # ---------------- 偏航角速度 -> 差动力矩 -------------------------
        # ---------------- yaw rate -> differential torque ----------------
        e_y = yaw_rate_ref - self.yaw_filt
        self.i_yaw = clamp_integral(self.i_yaw + e_y * dt, ki_y,
                                    lim.i_yaw_out_max)
        if self._prev_e_yaw is None:
            self._prev_e_yaw = e_y
        raw_dy = (e_y - self._prev_e_yaw) / max(dt, 1e-9)
        self.d_yaw += a_d * (raw_dy - self.d_yaw)
        self._prev_e_yaw = e_y

        tau_diff = kp_y * e_y + ki_y * self.i_yaw + kd_y * self.d_yaw

        # ---------------- 混合 + 饱和 / mix + saturate --------------------
        tau_l_raw = 0.5 * tau_sum - tau_diff
        tau_r_raw = 0.5 * tau_sum + tau_diff
        tau_l = float(np.clip(tau_l_raw, -self.tau_max, self.tau_max))
        tau_r = float(np.clip(tau_r_raw, -self.tau_max, self.tau_max))
        saturated = (tau_l != tau_l_raw) or (tau_r != tau_r_raw)

        # 反算式抗积分饱和：执行器一旦削顶就把积分器往回退，否则机器人会
        # 「记住」一个它根本满足不了的需求，在一次大冲击之后严重过冲。
        # back-calculation anti-windup: unwind the integrators when the
        # actuators are clipping, otherwise the robot "remembers" a demand it
        # could never satisfy and overshoots badly after a big kick.
        if saturated:
            excess = (tau_l_raw + tau_r_raw) - (tau_l + tau_r)
            if ki_p > 1e-9:
                self.i_pitch = clamp_integral(
                    self.i_pitch - 0.5 * excess / ki_p * dt,
                    ki_p, lim.i_pitch_out_max)
            self.i_vel *= 0.98

        self._prev_pitch_ref = pitch_ref
        return ControlOutput(tau_l, tau_r, tau_sum, tau_diff, pitch_ref,
                             e_p, e_v, e_y, saturated)


# ----------------------------------------------------------------------
def clamp_integral(value: float, gain: float, out_max: float) -> float:
    """钳住积分器，使 ``gain * value`` 永不超过 ``out_max``。
    Clamp an integrator so that ``gain * value`` never exceeds ``out_max``.

    用输出单位来钳，能让 Ki 的作用在 PPO 被允许探索的整个范围内保持单调。

    Clamping in output units keeps the effect of Ki monotonic across the whole
    range the PPO agent is allowed to explore.
    """
    if gain <= 1e-9:
        return 0.0
    lim = out_max / gain
    return float(np.clip(value, -lim, lim))


def lqr_reference_gains(dynamics, q_diag=(1.0, 60.0, 2.0), r=8.0):
    """在线性化模型上求解连续时间 LQR。
    Continuous-time LQR on the linearised model.

    仅用作自检 / 标称 PID 增益的起点——``scipy`` 是可选依赖，装不上时这里
    返回 ``None``。

    Only used as a sanity check / a starting point for the nominal PID gains
    -- ``scipy`` is optional and this returns ``None`` when it is missing.
    """
    try:
        from scipy.linalg import solve_continuous_are
    except Exception:
        return None
    Ac, Bc = dynamics.linearize()
    Q = np.diag(q_diag)
    R = np.array([[r]])
    P = solve_continuous_are(Ac, Bc, Q, R)
    K = np.linalg.solve(R, Bc.T @ P)      # u = -K x, x = [v, th, th_dot]
    return K.ravel()
