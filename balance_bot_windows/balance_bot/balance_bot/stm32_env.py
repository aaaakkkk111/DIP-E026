"""专为 STM32 平衡车孪生定制的 Gymnasium 环境。
Gymnasium environment specifically tailored for the STM32 Balance Car twin.

特点 / Features:
- 既能包快速解析孪生，也能包高保真 MuJoCo 孪生。
  Wraps either the fast analytic twin or the high-fidelity MuJoCo twin.
- 原生支持 Yahboom 370 电机特性（45% 死区、0.20 N·m 堵转）。
  Native support for Yahboom 370 motor characteristics (45% deadband, 0.20 Nm stall).
- 增益调度锚定在固件基线（LQR 或 PID）上，所以训练是从一个本来就能站住的
  基线出发，学的是对扰动的鲁棒适应，而不是从零学平衡。
  Anchored gain scheduling on firmware baseline (LQR or PID) so training starts
  from a functioning baseline and learns robust adaptation to disturbances.
- 可 pickle 的环境工厂，供 Windows 多进程（SubprocVecEnv）使用。
  Picklable environment factory for Windows multi-processing (SubprocVecEnv).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import gymnasium as gym
from gymnasium import spaces
import numpy as np

from .dynamics import IX, IY, IPSI, IPSID, IS, IV, ITH, ITHD
from .firmware.controllers import LQR_GAINS, PID_GAINS, HYBRID_GAINS
from .firmware.twin_baseline import (STM32Twin, make_mujoco_twin,
                                     MODE_STM32_HYBRID,
                           MODE_STM32_LQR, MODE_STM32_PID,
                           sample_stm32_disturbance)
from .params import ArenaParams, DisturbanceConfig
from .env import CommandScript


# --------------------------------------------------------------------------
# 奖励权重（v6）/ Reward weights (v6)
# --------------------------------------------------------------------------
# 七项，每项对应一个能看见的现象，全部线性。数值是这样定的：先用原厂增益
# （action = 0）在难度 0.6 下实测每一项的典型量级，再把权重调到各项大致可比，
# 且总惩罚明显小于存活奖励——否则策略会算出「早点摔了少受罚」。
#
# Seven terms, one per visible symptom, all linear.  The numbers come from
# measuring each term's typical magnitude at the factory gains (action = 0) and
# scaling them to be comparable, with the total staying well under the alive
# bonus so falling never pays.
# 标定过程：先用原厂增益（action = 0）在难度 0.6 下实测每项的典型量级，
# 第一版权重下静止局平均奖励是 **-0.791/步**——活着 500 步得 -395，而第一步
# 就摔只有 -50，策略会正确地算出「早点摔更划算」。权重整体缩小 2.5 倍、存活
# 奖励抬到 4.0 之后才为正。任何时候改权重都要重跑这个检查。
#
# Calibration: measured each term at the factory gains, difficulty 0.6.  The
# first cut averaged -0.791 per step standing still -- surviving 500 steps
# scored -395 while falling immediately scored -50, so falling paid.  Scaled
# down 2.5x with the alive bonus at 4.0 to fix it.  Re-run this check after
# any weight change.
R_ALIVE = 4.0              # 每步活着的基础奖励 / per-step bonus for staying up

W_PITCH = 10.0             # |俯仰角| rad —— 车身摆动 / body sway
# 阻尼项。#9 从 0.15 提到 0.45：#8 在「开完再停」那一档俯仰 RMS 1.65 度，
# 比 #7 的 0.81 度差一倍，而 W_PITCH/W_PRATE = 67 倍的悬殊让角速度几乎不计费。
#
# 注意这里罚的是**俯仰角速度本身**，不是 balance_kd。两者不是一回事：kd 乘在
# 角速度**测量值**上，而测量值里有陀螺噪声（训练难度拉满时 0.12 rad/s，外加
# 0.004 rad/sqrt(s) 的零偏游走），噪声经 kd 放大后撞上死区就变成 bang-bang。
# #8 的实测证据是 balance_kd 在三种工况下**全部贴在下限 0.160 不动**——PPO
# 已经独立发现这台车的阻尼要往下调。所以这一项只说「别晃」，不说「加 kd」，
# 用什么手段做到交给策略；抖动惩罚 W_CHATTER 会替我们否决掉靠 kd 硬压的解。
#
# This penalises pitch *rate*, not balance_kd -- not the same thing.  kd
# multiplies the measured rate, and the measurement carries gyro noise, which
# kd amplifies into dead-band bang-bang.  #8 pinned balance_kd at its floor in
# all three conditions, so PPO already found that this car wants less damping,
# not more.  The term says "stop swaying" and leaves the means to the policy;
# W_CHATTER vetoes any solution that just cranks kd.
# LQR 静止摇晃 4.264 度 @ 2.59 Hz，对应俯仰角速度幅值 0.074*16 = 1.2 rad/s。
# W_PRATE=0.15 时这只值 0.18/步，几乎不计费；提到 1.0 让它和 W_PITCH 的
# 0.74/步 同量级，摇晃才真正进入优化目标。这一项专打「静止前后摇晃」和
# 「停车振荡」，两者都是俯仰方向的往复。
# The rocking is a 2.59 Hz pitch oscillation whose rate amplitude is ~1.2
# rad/s; at 0.15 that scored 0.18/step and was effectively free.
W_PRATE = 1.0              # |俯仰角速度| rad/s —— 阻尼 / damping
# #10 把这三项一起提价，理由是实测出来的一个 0.07/步的账：
# #9 能调死区补偿之后把它从 1300 砍到 780，于是无扰动时位置 1.4 mm、PWM 换向
# 0.0 次/秒——三个靶全穿。但侧风下位置漂到 0.425 m（原厂 0.008），速度 0.20
# 指令只跟到 0.115。消融证实是同一个常量造成的：补偿低 = 小指令不产生动作 =
# 安静但没劲。
#
# 用奖励函数自己算这笔账：强制 1300 在姿态+位置上比 780 好 0.23/步，但抖动
# 多付 0.30/步，净输 0.07。**PPO 找到的是我这个奖励的最优解，是权重错了。**
#
# 修法不是压低 W_CHATTER（那会毁掉无扰动时那个 1.4 mm 的结果），而是给
# 位置和跟踪提价。关键在于无风时低死区在位置和抖动上是**双赢**、不冲突，
# 所以提价不影响 A 组，只在真有外力时才改变行为。
#
# Measured 0.07/step ledger: #9 cut the dead-band compensation 1300 -> 780 and
# won everything with no disturbance (1.4 mm, 0.0 reversals/s) but drifts
# 0.425 m under wind and tracks 0.20 m/s at 0.57.  Forcing 1300 back is worth
# +0.23/step on attitude and position and costs 0.30/step in chatter, so the
# policy optimised my reward correctly -- the weights were wrong.  Raising the
# position and tracking prices (rather than lowering W_CHATTER) is surgical:
# with no wind, low compensation wins on position *and* chatter, so nothing
# changes there; only the loaded cases move.
W_TRACK = 2.5              # |速度误差| m/s —— 输入延迟 / command lag
W_POS = 4.0                # |位置误差| m —— 自己跑出去 / drifting off
W_YAW = 3.0                # |航向误差| rad —— 方向变了 / heading drift
# 偏航率跟踪。#11 新增，补的是结构缺口不是调权重：原来的 W_YAW 写在
# `if holding` 里，只在松手时计费，**转向过程中偏航率跟得准不准一分钱不罚**。
# 速度有 W_TRACK 管着，偏航率什么都没有，于是 #8 的偏航率过冲 21.1%，
# 原厂 13.7%——策略没有理由做得更干净。
#
# 这一项无条件计费，因此同时管两件事：
#   转向时  |yaw_ref - yaw_rate| 大 -> 罚，压过冲和振铃
#   静止时  yaw_ref = 0，车自己转起来 -> 罚，压「静止时自己旋转」
# 一项覆盖用户的第 2 条和第 3 条目标。
#
# Yaw-rate tracking, new in #11 and a structural gap rather than a reweight:
# W_YAW sat inside `if holding`, so nothing at all penalised yaw tracking
# while turning.  Applied unconditionally it covers both goals: overshoot when
# turning, and spinning in place when the command is zero.
W_YAWRATE = 1.5            # |偏航率误差| rad/s —— 转向振荡 + 静止自转
W_JERK = 1.0               # 动作变化的 RMS —— 策略自己乱改增益
# 这一项是我自己加的，不在用户提的三个问题里，而且它是 #7~#10 位置和跟踪
# 一路退步的直接原因。
#
# 用户说的是「无输入时没停稳、输入延迟大、自调整时振荡明显」——三条都是
# **看得见的车身行为**，由 W_POS / W_TRACK / W_PITCH 管着。我把「振荡」读成了
# 死区极限环（听得见的电机嗡嗡声），于是加了这一项。那是个误读。
#
# 代价是实打实的：在 45% 死区的执行器上，**低速控制权限就是靠左右交替换向
# 换来的**。原厂 0.0076 m 的抗风位置，物理上就是用 16.6 次/秒买的。消融数据
# （同一个网络，只改死区补偿 780 -> 1300）：位置 0.4252 -> 0.0497 m，换向
# 0.3 -> 8.5 次/秒。这个交换是硬件决定的，罚换向就等于罚控制权限。
#
# 三个模型的单调证据：抖动 16.7 -> 12.2 -> 5.8 -> 0.0，同时风中位置
# 0.0076 -> 0.1256 -> 0.1068 -> 0.4252，跟踪 1.00 -> 0.92 -> 0.79 -> 0.38。
#
# This term was mine, not the user's.  The three reported symptoms are all
# visible body behaviour, already covered by W_POS / W_TRACK / W_PITCH; I read
# "oscillation" as the audible dead-band limit cycle, which was wrong.  On a
# 45 %-dead-band actuator, low-speed authority *is* bought with reversals --
# stock's 0.0076 m in wind costs 16.6/s -- so penalising them penalises
# authority.  Kept as a switch rather than deleted because #9 proved it works
# when nothing is pushing: 1.4 mm and 0.0 reversals with no disturbance.
W_CHATTER = 0.8            # |逐拍 PWM 变化| / 2880 —— 默认开，--no-chatter 关

PWM_FULL = 2880.0          # 定时器周期；|ccr| 就在这个尺度上 / TIM period

# 观测归一化 / obs normalisation, metres
OBS_POS_SCALE = 0.3

# 训练指令的上限必须是这台车**真跑得住**的速度，不是固件嘴上发的那个。
# 指令上限必须是车真的撑得住的速度，否则大部分驾驶段都在要求策略做办不到的
# 事，学到的是「别听速度指令」。
#
# 【2026-09-10 重测，MuJoCo，斜坡 0.9 m/s2，5 种子 x 12 s，站住的种子数】
#            0.50  0.60  0.70  0.80  0.90  1.00  1.15
#   PID 原厂   5/5   5/5   5/5   5/5   0/5   0/5   0/5   <- 卡在 0.80
#   PID 修正   5/5   5/5   5/5   5/5   0/5   0/5   0/5   <- 一模一样
#   LQR 原厂   5/5   5/5   5/5   5/5   5/5   5/5   5/5
#   LQR 修正   5/5   5/5   5/5   5/5   5/5   5/5   5/5
#
# 能持续跑住的前进速度。2026-09-10 实测（MuJoCo 斜坡、5 种子）：PID 卡在
# 0.80 m/s，是它 90.3% 占空比上限的结果（app_control.c 先补死区后钳位）；
# LQR 能到 1.15，因为它先钳位后补死区、比较值到 3900、顶满 100% 占空比。
# 取 0.65 按较弱的 PID 那一支定，留约 20% 余量。
#
# 【2026-09-17】原来这里是两个常数（修不修编码器各一个），同一次实测显示修正
# 对可持续速度毫无影响，两个数早就一样了；开关删掉后合并成一个。
# One sustainable speed; the encoder-fix variant was removed once measured to
# make no difference.
V_SUSTAINABLE = 0.65


def command_script() -> dict:
    return dict(v_max=V_SUSTAINABLE,
                yaw_max=2.0, hold_min=1.5, hold_max=4.0, p_zero=0.35)


@dataclass
class STM32CommandScript(CommandScript):
    """指令脚本，站定段单独给一套时长。
    Command script with its own duration for stand-still segments.

    和基类的两点不同：
    1) **联合置零。** 基类对 v 和 w 分别掷骰子，于是「两个都为零」只有
       p_zero^2 = 12% 的机会，位置/航向惩罚大部分时间根本不计费。
    2) **站定段单独计时。** 行驶段还是 1.5~4 秒，站定段拉到 6~16 秒，
       因为要抓的那个低频游荡周期就有 10~15 秒。

    Two differences from the base class: v and w are zeroed *together* (the
    base rolls them independently, so a genuine hold happens only p_zero^2 of
    the time), and hold segments get their own, much longer duration -- the
    drift being penalised has a 10-15 s period, so a 4 s window cannot see it.
    """
    zero_hold_min: float = 6.0
    zero_hold_max: float = 16.0

    def sample(self, rng):
        if rng.random() < self.p_zero:
            return 0.0, 0.0, float(rng.uniform(self.zero_hold_min,
                                               self.zero_hold_max))
        v = rng.uniform(-self.v_max, self.v_max)
        w = 0.0 if rng.random() < 0.3 else rng.uniform(-self.yaw_max,
                                                       self.yaw_max)
        return float(v), float(w), float(rng.uniform(self.hold_min,
                                                     self.hold_max))

# 站定外环的两个增益，实测在原厂 PID 上把航向漂移从 20.8 度压到 1.9 度。
# PPO 在它们周围搜，所以 action = 0 仍然是「实测最好的手调值」。
# The two station-hold gains.  Measured on the factory PID they cut heading
# drift from 20.8 deg to 1.9; PPO searches around them, so action = 0 is still
# the best hand-tuned setting rather than an arbitrary midpoint.
HOLD_NOMINAL = (1.5, 4.0)          # kpos, kpsi
HOLD_LOW = (0.2, 0.5)
HOLD_HIGH = (8.0, 15.0)


# 逐增益跨度。数字来自 #8 的贴边诊断，不是拍脑袋：
#   balance_kp   3   没贴边（216/96 = 2.25 倍），够用
#   balance_kd   6   贴在下限。噪声进来乘在角速度上再撞死区，这台车的阻尼要
#                    往**下**调；给它往下走的余地，不是往上。
#   velocity_kp  3   没贴边
#   velocity_ki  6   贴在下限
#   turn_kp      6   贴在**上限**，航向还想要更多权力
#   turn_kd      6   接近上限；同样乘噪声，两头都要留路
# Per-gain spans, from #8's saturation diagnosis rather than guesswork: four
# of six gains sat pinned on a box bound, which means the search space was the
# binding constraint, not convergence.
# #9 之后又有三个维度贴边：balance_kp 顶 288（3 倍上限）、velocity_kp 顶
# 185（3 倍上限）、hold_kpsi 顶 15。放宽到 5 倍 / 6 倍。
# Three more dims saturated in #9; widen them.
# #11 退回 #8 的统一 3 倍。放宽跨度是为配合死区维度做的（#9 砍死区后需要
# 更大的增益权限去补偿），死区放弃了，跨度也该跟着退回来。
# turn_kp（索引 4）的跨度从 3.0 放到 7.0。
# 【2026-09-12 实测依据】第一次定点行驶训练学到 turn_kp = 42.0000，
# 恰好 = 14 x 3，一分不多——**顶死在跨度上，不是顶在物理上**（high = 100）。
# 不重训、只手工把 turn_kp 抬上去，到达半径 5cm 的胜率（6 局/档）：
#         0kg  1kg  2kg  3kg  4kg
#   42     5    6    0    0    0     <- 跨度钳住的值
#   60     6    6    6    1    0
#   80     6    6    6    6    0
#   100    6    6    6    6    4     <- 硬上限
# 这对得上真车实测的「高负重难转向」：4kg 时偏航惯量涨 11.7 倍，近目标要重新
# 指向时转不动，cos 门控又把前进掐掉，于是差最后 10~20 cm 进不去。
# 7.0 给到 14*7 = 98，贴合 high = 100。
#
# Measured: the first goto run learned turn_kp = 42.0 = 14 x 3 exactly, i.e.
# pinned by the SPAN, not by the bound (high = 100).  Raising it by hand to
# 100 takes the win rate from 5/6/0/0/0 to 6/6/6/6/4 across the payload
# ladder with no retraining.
PID_SPANS = (3.0, 3.0, 3.0, 3.0, 7.0, 3.0)
LQR_SPANS = (3.0,) * 6
HOLD_SPANS = (3.0, 3.0)

@dataclass
class STM32GainSpace:
    """围绕固件标称增益的搜索 / 调制空间。
    Gain search / modulation space around the firmware nominal gains."""
    firmware: str = MODE_STM32_LQR
    span: float = 3.0  # 遗留的标量跨度；per_gain=False 时全维度共用
    hold: bool = False                 # 是否把站定外环的两个增益也交给策略
    per_gain: bool = False             # 用 *_SPANS 的逐增益跨度

    def __post_init__(self):
        if self.firmware == MODE_STM32_HYBRID:
            # LQR 的六个增益 + 两个 PID 辅助权重。后两个标称 0，乘性缩放对 0
            # 无效，所以它们走线性映射（见 action_to_gains 里的 self.linear）。
            self.names = ("K1", "K2", "K3", "K4", "K5", "K6",
                          "w_gyro", "w_pid_vel")
            self.nominal = np.array([
                HYBRID_GAINS["K1"], HYBRID_GAINS["K2"], HYBRID_GAINS["K3"],
                HYBRID_GAINS["K4"], HYBRID_GAINS["K5"], HYBRID_GAINS["K6"],
                0.0, 0.0], dtype=np.float64)
            self.low = np.array([-250.0, -300.0, -1200.0, -150.0,
                                 0.0, 0.0, 0.0, 0.0])
            self.high = np.array([0.0, 0.0, -100.0, -2.0,
                                  60.0, 60.0, 1.0, 1.0])
            self.linear = np.array([False] * 6 + [True] * 2)
        elif self.firmware == MODE_STM32_LQR:
            self.names = ("K1", "K2", "K3", "K4", "K5", "K6")
            self.nominal = np.array([
                LQR_GAINS["K1"], LQR_GAINS["K2"], LQR_GAINS["K3"],
                LQR_GAINS["K4"], LQR_GAINS["K5"], LQR_GAINS["K6"]
            ], dtype=np.float64)
            # 围绕标称值的边界（保持正确的正负号）
            # Bounds around the nominal values (preserving proper signs)
            self.low = np.array([-250.0, -300.0, -1200.0, -150.0, 0.0, 0.0])
            self.high = np.array([0.0, 0.0, -100.0, -2.0, 60.0, 60.0])
        else:
            self.names = ("balance_kp", "balance_kd", "velocity_kp",
                          "velocity_ki", "turn_kp", "turn_kd")
            self.nominal = np.array([
                PID_GAINS["balance_kp"], PID_GAINS["balance_kd"],
                PID_GAINS["velocity_kp"], PID_GAINS["velocity_ki"],
                PID_GAINS["turn_kp"], PID_GAINS["turn_kd"]
            ], dtype=np.float64)
            # turn_kp 上限从 60 提到 100：span 6 给到 14*6 = 84，卡在 60
            # 就等于没放开。turn_kp's ceiling goes 60 -> 100 so span 6 is real.
            self.low = np.array([20.0, 0.05, 10.0, 0.01, 2.0, 0.0])
            self.high = np.array([400.0, 2.50, 250.0, 2.00, 100.0, 1.5])
        if self.firmware == MODE_STM32_HYBRID:
            base_spans = LQR_SPANS + (1.0, 1.0)      # 后两个走线性，span 不用
        elif self.firmware == MODE_STM32_LQR:
            base_spans = LQR_SPANS
        else:
            base_spans = PID_SPANS
        n_base = len(base_spans)
        self.spans = np.array(base_spans if self.per_gain
                              else (self.span,) * n_base, dtype=np.float64)
        if self.firmware == MODE_STM32_HYBRID:
            self.spans[6:8] = 1.0
        if not hasattr(self, "linear"):
            self.linear = np.zeros(n_base, dtype=bool)
        if self.hold:
            self.names = self.names + ("hold_kpos", "hold_kpsi")
            self.nominal = np.concatenate([self.nominal, HOLD_NOMINAL])
            self.low = np.concatenate([self.low, HOLD_LOW])
            self.high = np.concatenate([self.high, HOLD_HIGH])
            self.spans = np.concatenate([self.spans, np.array(
                HOLD_SPANS if self.per_gain else (self.span,) * 2)])
            self.linear = np.concatenate([self.linear, [False, False]])
    @property
    def dim(self) -> int:
        return len(self.nominal)

    def action_to_gains(self, action: np.ndarray) -> dict[str, float]:
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        # 围绕标称值的乘性缩放 / Multiplicative scaling around nominal
        scale = self.spans ** a
        gains_arr = self.nominal * scale
        # 标称为 0 的维度（混合权重）乘性缩放没有意义，改成 -1..+1 -> low..high
        # 的线性映射。Dimensions whose nominal is 0 cannot be scaled
        # multiplicatively; map the action linearly onto [low, high] instead.
        if np.any(self.linear):
            lin = self.linear
            gains_arr[lin] = (self.low[lin]
                              + (a[lin] + 1.0) * 0.5
                              * (self.high[lin] - self.low[lin]))
        if self.firmware == MODE_STM32_LQR:
            # LQR 增益：K1..K4 为负，K5..K6 为正
            # LQR gains: K1..K4 are negative, K5..K6 positive
            gains_arr = np.clip(gains_arr, self.low, self.high)
        else:
            gains_arr = np.clip(gains_arr, self.low, self.high)
        return {name: float(val) for name, val in zip(self.names, gains_arr)}

    def gains_to_action(self, gains_dict: dict[str, float]) -> np.ndarray:
        arr = np.array([gains_dict.get(k, self.nominal[i]) for i, k in enumerate(self.names)])
        ratio = np.maximum(np.abs(arr), 1e-6) / np.maximum(np.abs(self.nominal), 1e-6)
        return np.clip(np.log(ratio) / np.log(self.spans), -1.0, 1.0)


class STM32Env(gym.Env):
    """把 STM32Twin 包成 Gymnasium 环境，供强化学习训练用。
    Gymnasium environment wrapping STM32Twin for RL training."""
    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self,
                 firmware: str = MODE_STM32_LQR,
                 backend: str = "analytic",
                 randomize: bool = True,
                 episode_seconds: float = 20.0,
                 imu_filter: str = "kalman",
                 command_prob: float = 0.5,
                 hold_station: bool = False,
                 per_gain_span: bool = False,
                 penalise_chatter: bool = True,
                 seed: int | None = None):
        super().__init__()
        self.firmware_name = firmware
        self.backend = backend
        # 孪生跑的是哪套姿态滤波。这个必须记录下来、并在评测时对齐：
        # "kalman"（KF.c，板子实际在跑的）和 "lag"（旧的占位实现）产生的观测
        # 分布有可测量的差别，所以在一套下训出来的策略，不能拿去和另一套下
        # 测出来的基线比。
        #
        # Which attitude filter the twin runs.  It has to be recorded and
        # matched at evaluation time: "kalman" (KF.c, what the board does) and
        # "lag" (the old placeholder) produce measurably different observation
        # distributions, so a policy trained under one is not comparable
        # against a baseline measured under the other.
        self.imu_filter = imu_filter
        self.randomize = randomize
        self.episode_seconds = episode_seconds
        self.difficulty = 0.0
        self.hold_station = bool(hold_station)
        # 训练必须和部署跑同一台车。这个字段会写进 npz，评测和 UI 跟着它走，
        # 和 imu_filter 一个道理。
        # Training must run the same car as deployment.  Recorded in the npz so
        # the bench and the UI follow it, exactly as imu_filter already does.
        self.w_chatter = W_CHATTER if penalise_chatter else 0.0
        self.gain_space = STM32GainSpace(firmware=firmware,
                                         hold=self.hold_station,
                                         per_gain=bool(per_gain_span))

        # 构造底层孪生 / Factory for core
        self._build_core(seed=seed)

        # 动作：每个增益在 [-1, 1] 上的乘性偏移
        # Action: multiplicative deviation in [-1, 1] for each gain
        act_dim = self.gain_space.dim
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(act_dim,), dtype=np.float32
        )

        # 观测 / Observation:
        # [sin(俯仰), cos(俯仰), 俯仰角速度/10, 速度/2, 偏航角速度/10,
        #  速度指令/2, 偏航指令/10, 上一步动作...]
        # 观测 v2：核心量从 7 个变成 10 个，多的三个是「相对锚点的前向误差、
        # 横向误差、航向误差」。没有它们策略看不见漂移，也就学不会站定。
        # Observation v2: 10 core terms instead of 7, the extra three being the
        # forward / lateral / heading error against the anchor.  Without them
        # the policy cannot see the drift it is being asked to remove.
        obs_dim = (10 if self.hold_station else 7) + act_dim
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self._last_action = np.zeros(act_dim, dtype=np.float64)
        self._current_gains = self.gain_space.action_to_gains(self._last_action)
        self.command_prob = float(command_prob)
        self._rng = np.random.default_rng(seed)
        self._prev_pitch = 0.0
        self._prev_pitch_rate = 0.0
        self._prev_ccr = 0.0

    def _build_core(self, seed: int | None = None):
        kw = dict(
            firmware=self.firmware_name,
            hold_station=self.hold_station,
            randomize=self.randomize,
            episode_seconds=self.episode_seconds,
            imu_filter=self.imu_filter,
            seed=seed or 0,
            gains=None,
        )
        if self.backend == "mujoco":
            self.core = make_mujoco_twin(**kw)
        else:
            self.core = STM32Twin(**kw)
        self.core.difficulty = self.difficulty

    def set_difficulty(self, d: float):
        self.difficulty = float(np.clip(d, 0.0, 1.0))
        self.core.difficulty = self.difficulty

    def _get_obs(self) -> np.ndarray:
        st = self.core.state
        pitch = st[ITH]
        pitch_rate = st[ITHD]
        v = st[IV]
        yaw_rate = st[IPSID]
        v_ref = self.core.v_ref
        yaw_ref = self.core.yaw_ref

        obs_core = np.array([
            np.sin(pitch),
            np.cos(pitch),
            pitch_rate / 10.0,
            v / 2.0,
            yaw_rate / 10.0,
            v_ref / 2.0,
            yaw_ref / 10.0,
        ], dtype=np.float32)

        if self.hold_station:
            e_f, e_l, e_p = self.core.station_error()
            obs_core = np.concatenate([obs_core, np.array([
                e_f / OBS_POS_SCALE, e_l / OBS_POS_SCALE, e_p,
            ], dtype=np.float32)])

        obs = np.concatenate([obs_core, self._last_action.astype(np.float32)])
        return np.clip(obs, -20.0, 20.0)

    def reset(self, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self.core.reset(seed=seed)
        else:
            self.core.reset()
        # v1 把每一局都钉死在 (0, 0)，于是整整 300 万步里 v_ref 恒等于零：策略
        # 从没见过一条指令，过冲那一项也就永远不会触发。现在一半的局会开车；
        # 另一半保持静止，因为极限环就住在那里。
        #
        # v1 pinned every episode to (0, 0), so v_ref was identically zero for
        # all 3M steps: the policy never saw a command, and the overshoot term
        # could never fire.  Half the episodes now drive; standstill keeps the
        # other half, because that is where the limit cycle lives.
        if self._rng.random() < self.command_prob:
            self.core.script = STM32CommandScript(
                **command_script())
            (self.core._script_v, self.core._script_w,
             self.core._cmd_left) = self.core.script.sample(self.core.rng)
            self.core.set_command(None, None)
        else:
            self.core.set_command(0.0, 0.0)
        self._last_action = np.zeros(self.gain_space.dim, dtype=np.float64)
        self._update_firmware_gains(self._last_action)
        # v2 的过冲/抖动项需要的历史量。用复位后的状态播种，这样第一步不会
        # 记到一次子虚乌有的过零。
        # History the v2 overshoot / chatter terms need.  Seeded from the
        # post-reset state so the first step cannot register a bogus crossing.
        self._prev_pitch = float(self.core.state[ITH])
        self._prev_pitch_rate = float(self.core.state[ITHD])
        self._prev_ccr = 0.0
        return self._get_obs(), {}

    def _update_firmware_gains(self, action: np.ndarray):
        gains = self.gain_space.action_to_gains(action)
        if self.firmware_name in (MODE_STM32_LQR, MODE_STM32_HYBRID):
            self.core.fw.K = np.array([
                gains["K1"], gains["K2"], gains["K3"],
                gains["K4"], gains["K5"], gains["K6"]
            ], dtype=float)
            if self.firmware_name == MODE_STM32_HYBRID:
                self.core.fw.w_gyro = float(gains["w_gyro"])
                self.core.fw.w_pid_vel = float(gains["w_pid_vel"])
        else:
            self.core.fw.g.update({k: v for k, v in gains.items()
                                   if not k.startswith("hold_")
                                   })
        if self.hold_station:
            self.core.hold_kpos = gains["hold_kpos"]
            self.core.hold_kpsi = gains["hold_kpsi"]
        self._current_gains = gains

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        # 变化率限制，保证稳定 / Slew rate limit for stability
        smoothed_action = 0.7 * self._last_action + 0.3 * action
        self._update_firmware_gains(smoothed_action)
        action_diff = np.mean((smoothed_action - self._last_action) ** 2)
        self._last_action = smoothed_action

        # 推进底层孪生一步 / Step core
        _, raw_reward, term, trunc, info = self.core.step()

        # ------------------------------------------------------------------
        # 奖励 v6：七项，全部线性
        # Reward v6: seven terms, all linear
        # ------------------------------------------------------------------
        # v2~v5 一路加到十一项、每项带上限，结果是项与项互相拮抗，改一个权重
        # 就在另一个指标上退步（#3 压住抖动丢了位置，#4 拿回位置丢了姿态，
        # #5 提了跟踪权重毫无效果）。这一版推倒重来，两条原则：
        #
        # 1) **线性，不用指数。** 之前 exp(-15*theta^2) 是为「别摔倒」调的，
        #    在 0.2~1.5 度这个真正干活的区间里几乎是平的——0.29 度和 1.03 度
        #    之间只差 0.0007，等于没有激励。线性惩罚全程梯度恒定。
        # 2) **项要少。** 每一项对应一个用户能看见的现象，不重复计费。
        #
        # v2..v5 grew to eleven capped terms that fought each other: fixing one
        # metric regressed another every time.  This version restarts from two
        # rules -- linear penalties (the old exp(-15*theta^2) was flat across
        # the 0.2-1.5 deg band where regulation actually happens, 0.0007 of
        # reward between 0.29 and 1.03 deg) and one term per visible symptom.
        st = self.core.state
        pitch = float(st[ITH])
        pitch_rate = float(st[ITHD])
        v = float(st[IV])
        yaw_ref = self.core.yaw_ref
        holding = (abs(self.core.v_ref) < 1e-6 and abs(yaw_ref) < 1e-6)

        # 站稳 / upright -- 车身摆动，就是「自调整时振荡明显」
        r_pitch = -W_PITCH * abs(pitch) - W_PRATE * abs(pitch_rate)

        # 跟指令 / tracking -- 「输入延迟」
        r_track = -W_TRACK * abs(self.core.v_ref - v)

        # 跟转向指令 / yaw-rate tracking -- 「左转右转会振荡」「静止时自己转」
        r_yawrate = -W_YAWRATE * abs(yaw_ref - float(st[IPSID]))

        # 不漂、不转 / station + heading -- 只在松手时计费，
        # 有指令时车本来就该动 / only while commanded to hold
        r_pos = r_yaw = 0.0
        if holding and self.hold_station:
            e_f, e_l, e_p = self.core.station_error()
            r_pos = -W_POS * float(np.hypot(e_f, e_l))
            r_yaw = -W_YAW * abs(e_p)

        # 不抖 / no chatter -- 两层各罚一次，因为它们是两回事：
        #   动作层 = 策略自己乱改增益；执行器层 = 死区极限环，真车听得见的那个
        # Two layers, two different things: the policy thrashing its own gains,
        # and the dead-band limit cycle that is what you actually hear.
        r_jerk = -W_JERK * float(np.sqrt(action_diff))
        ccr = float(np.mean(info["ccr"]))
        r_chatter = -self.w_chatter * abs(ccr - self._prev_ccr) / PWM_FULL
        self._prev_ccr = ccr

        reward = (R_ALIVE + r_pitch + r_track + r_yawrate + r_pos + r_yaw
                  + r_jerk + r_chatter)

        if self.core.fell:
            reward -= 50.0

        obs = self._get_obs()
        info["gains"] = self._current_gains
        return obs, float(reward), term, trunc, info


class STM32EnvFactory:
    """可 pickle 的环境工厂，供 Windows 的 SubprocVecEnv 多进程使用。
    Picklable environment factory for Windows SubprocVecEnv multiprocessing."""
    def __init__(self, rank: int, seed: int, firmware: str, backend: str,
                 difficulty: float, imu_filter: str = "kalman",
                 episode_seconds: float = 20.0, command_prob: float = 0.5,
                 hold_station: bool = False,
                 per_gain_span: bool = False, penalise_chatter: bool = True):
        self.rank = rank
        self.seed = seed
        self.firmware = firmware
        self.backend = backend
        self.difficulty = difficulty
        self.imu_filter = imu_filter
        self.episode_seconds = episode_seconds
        self.command_prob = command_prob
        self.hold_station = hold_station
        self.per_gain_span = per_gain_span
        self.penalise_chatter = penalise_chatter

    def __call__(self):
        env = STM32Env(
            firmware=self.firmware,
            backend=self.backend,
            randomize=True,
            episode_seconds=self.episode_seconds,
            imu_filter=self.imu_filter,
            command_prob=self.command_prob,
            hold_station=self.hold_station,
            per_gain_span=self.per_gain_span,
            penalise_chatter=self.penalise_chatter,
            seed=self.seed + self.rank
        )
        env.set_difficulty(self.difficulty)
        return env
