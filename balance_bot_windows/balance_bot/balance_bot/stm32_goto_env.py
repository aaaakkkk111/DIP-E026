# -*- coding: utf-8 -*-
"""「开到指定点并停稳」的训练环境，产出**能烧进板子的静态 PID 增益**。

Drive-to-a-point training env whose output is a static, flashable gain set.

设计上的三条硬约束
------------------

1. **产物必须是静态的，而且 PPO 的信用分配要对得上。**
   现有 ``STM32Env`` 每个智能体拍都输出一次增益倍数，那是个自适应增益调度器，
   烧不进板子。这里**一个 PPO 步 = 一整局 25 秒试验**：``step()`` 拿到一组增益，
   把整局跑完，返回整局的累计奖励，然后 ``terminated=True``。

   第一版不是这样：那时 ``step()`` 只推进一拍、动作在开局冻结。结果 PPO 每局
   采样 625 个动作，其中 624 个对奖励**毫无因果作用**，却照样被分配了整局的
   优势——信噪比 1:624，梯度全是噪声。实测 1.44M 步之后
   ``clip_fraction ~ 0``、``explained_variance ~ 0``，策略纹丝不动。
   写成 bandit 之后每个动作恰好对应它自己产生的那一份回报。

   One PPO step is one full 25 s trial.  The first version stepped one tick at
   a time with the action frozen after t=0, so 624 of every 625 sampled actions
   had no causal effect on the reward yet still received the episode's
   advantage; after 1.44M steps nothing had moved.

2. **导航外环必须是能翻成 C 的。** 固件 PID 只有速度环和积分，没有位置反馈，
   转向是纯前馈——**没有任何增益作用在航向误差上**，所以再怎么调参也开不到
   指定坐标。缺的那条通路在这里补上，只用两个增益、一段除法和一个 atan2，
   见 ``navigate()``；``export_c()`` 直接把它打印成 C。
   The stock PID has no heading feedback, so no gain can make it reach a
   point.  ``navigate()`` is that missing loop, kept small enough to hand-
   translate; ``export_c()`` prints it.

3. **一套增益要吃下 0~4 kg。** 载重按回合分层轮换（不是随机），保证每个
   rollout 里五档载重都出现，否则回报方差大到学不动。
   Payload is stratified across episodes, not sampled i.i.d.

为什么观测是「这一局有多难」而策略看不见它
------------------------------------------
观测是回合上下文（载重、目标距离、初始方位角），**整局恒定**。它只喂给
critic：不同载重、远近目标的回报差着量级，价值函数看不见上下文就只能预测一个
全局均值，优势退化成纯噪声——实测 ``explained_variance = 6e-8``、
``clip_fraction = 0``，策略几乎不动。

actor 那一侧的输入在 ``ConstActorPolicy`` 里被强制置零（见训练脚本），所以策略
输出与状态无关，产物**保证**是单一一组数。若让 actor 看见载重，它会给不同载重
不同增益——那又变成「切换模型」，正是用户要避免的。

The observation is per-episode context fed to the critic only; the actor's
input is zeroed by ConstActorPolicy, so the policy stays state-independent
while the value function can still tell a hard episode from an easy one.
"""
from __future__ import annotations

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:                                   # pragma: no cover
    import gym
    from gym import spaces

from .dynamics import ITH, ITHD, IV, IPSI, IPSID
from .firmware.twin_baseline import (STM32Twin, make_mujoco_twin,
                                     MODE_STM32_PID)
from .stm32_env import STM32GainSpace
from .params import DisturbanceConfig

# --------------------------------------------------------------------------
# 导航外环的两个增益。这就是固件缺的那条通路。
# The two navigator gains -- the feedback path the firmware lacks.
# --------------------------------------------------------------------------
# nav_kv: 距离 -> 速度指令，1/s。1.0 表示离目标 1 m 时要 1 m/s（会被 V_MAX 截）
# nav_kw: 方位角误差 -> 偏航指令，1/s。
NAV_NOMINAL = (0.9, 2.0)
NAV_LOW = (0.2, 0.4)
NAV_HIGH = (3.0, 8.0)
NAV_SPANS = (3.0, 3.0)

# 外环自己的限幅。V_MAX 按 2026-09-10 实测的可持续速度定（PID 支撑到
# 0.80 m/s，取 0.55 留三成余量）；YAW_MAX 沿用站定外环的 1.2 rad/s。
# Ceilings for the outer loop, from the measured sustainable speed.
NAV_V_MAX = 0.55
NAV_YAW_MAX = 1.2

# 到达半径。原来 5cm 是随手定的，比车自己还小得多（车长 194mm），加上死区
# 极限环，要求停在 5cm 圈内静止 3 秒偏苛刻。10cm 约半个车长，是站得住的规格。
# 2026-09-12 实测（turn_kp=100，6 局/档的胜利数）：
#    5cm -> 6/6/6/6/4     10cm -> 6/6/6/6/5
# 注意放大半径**代替不了**修 turn_kp：跨度钳在 42 时，25cm 也只到 6/6/6/6/1。
ARRIVE_R = 0.10          # m，进这个圈算「到了」
ARRIVE_SWAY = np.deg2rad(2.0)   # rad，到了还要摆得够小才算稳
HOLD_SECONDS = 3.0       # s，连续稳这么久算赢

TARGET_R_MIN = 0.6       # m，目标点距离范围
TARGET_R_MAX = 1.8

PAYLOAD_LADDER = (0.0, 1.0, 2.0, 3.0, 4.0)   # 分层轮换的载重档

# --------------------------------------------------------------------------
# 奖励权重
# --------------------------------------------------------------------------
R_ALIVE = 2.0            # 活着
W_PITCH = 8.0            # |俯仰|，行驶中和停稳后都计费 —— 「期间稳定程度」
W_PRATE = 0.8            # |俯仰角速度|
W_DIST = 1.5             # 到目标距离 m —— 隐式奖励快
R_ARRIVE = 60.0          # 首次到达的一次性奖励，按剩余时间打折 —— 「到达时间」
W_HOLD_POS = 12.0        # 到达后的位置误差 m —— 「到达后稳定程度」
W_HOLD_PITCH = 6.0       # 到达后的额外姿态惩罚
R_WIN = 120.0            # 连续稳住 HOLD_SECONDS
R_FELL = -150.0          # 摔了
# 逐拍 |dCCR|/2880 —— 电机来回猛踹，真车上听得见也看得见。
# 【2026-09-13 补】这一项原来**漏了**：奖励里只有 |俯仰| 和 |俯仰角速度|，
# 一个 1.68° 的 8 Hz 振荡几乎不扣分，优化器没有任何理由避开它。v2 上车高频
# 剧烈振荡，这是两个原因之一（另一个是 PWM_DELAY_TICKS=0）。
# 权重沿用 stm32_env.py 里验证过的 W_CHATTER = 0.8。
# This term was missing: a small-amplitude 8 Hz cycle cost almost nothing
# under W_PITCH alone, so nothing discouraged it.
W_CHATTER = 0.8
PWM_FULL = 2880.0


def navigate(dx, dy, psi, kv, kw):
    """位置误差 -> (速度指令, 偏航指令)。**这段要原样翻成 C。**

    Position error -> (v_ref, yaw_ref).  This is the block to port to C.

    差速平衡车只能沿车头走，所以横向误差必须先转成航向误差：先算目标方位角，
    再用 cos 门控前进——没对准目标就先转、少走，对准了才全速。
    A differential drive can only move along its nose, so lateral error has to
    become heading error first; the cosine gate stops it driving off-axis.

    **偏航指令的符号是反的，这是真固件的约定，不是孪生的毛病。**
    ``app_control.c``: ``Motor_Left = ...+Turn_Pwm``、``Motor_Right = ...-Turn_Pwm``,
    正的 turn 让左轮快、车顺时针转，而世界系的 psi 是逆时针为正——所以
    「指令为正」= 「psi 减小」。实测：指令 +1.5 得到 psi 速率 -0.104 rad/s。
    C 代码里也必须是这个符号，写反了车会朝相反方向越转越远。

    The yaw command sign is inverted, and this is the real firmware's
    convention: a positive Turn_Pwm speeds the left wheel, turning clockwise,
    while world psi grows counter-clockwise.  Measured: +1.5 command gives
    -0.104 rad/s.  The C port must keep this sign.
    """
    dist = float(np.hypot(dx, dy))
    bearing = float(np.arctan2(dy, dx) - psi)
    bearing = float(np.arctan2(np.sin(bearing), np.cos(bearing)))

    yaw_ref = float(np.clip(-kw * bearing, -NAV_YAW_MAX, NAV_YAW_MAX))
    if dist <= ARRIVE_R:
        # 到了就别再动，交给固件自己的速度环收住
        return 0.0, 0.0
    gate = max(0.0, np.cos(bearing))
    v_ref = float(np.clip(kv * dist, -NAV_V_MAX, NAV_V_MAX)) * gate
    return v_ref, yaw_ref


class STM32GotoEnv(gym.Env):
    """一局 = 一个随机目标点 + 一档载重。策略只在开局决策一次。"""

    metadata = {"render_modes": []}

    def __init__(self, seed: int = 0, backend: str = "mujoco",
                 imu_filter: str = "kalman", episode_seconds: float = 25.0,
                 randomize: bool = False,
                 payload_phase: int = 0):
        super().__init__()
        self.backend = backend
        self.imu_filter = imu_filter
        self.episode_seconds = float(episode_seconds)
        self.randomize = bool(randomize)
        self.difficulty = 0.0

        # 固件的 6 个 PID 增益 + 导航外环的 2 个 = 8 维，全部静态
        self.gain_space = STM32GainSpace(firmware=MODE_STM32_PID,
                                         per_gain=True)
        self.gain_space.names = self.gain_space.names + ("nav_kv", "nav_kw")
        self.gain_space.nominal = np.concatenate(
            [self.gain_space.nominal, np.array(NAV_NOMINAL)])
        self.gain_space.low = np.concatenate(
            [self.gain_space.low, np.array(NAV_LOW)])
        self.gain_space.high = np.concatenate(
            [self.gain_space.high, np.array(NAV_HIGH)])
        self.gain_space.spans = np.concatenate(
            [self.gain_space.spans, np.array(NAV_SPANS)])
        self.gain_space.linear = np.concatenate(
            [self.gain_space.linear, np.zeros(2, dtype=bool)])
        # dim 是 len(nominal) 的只读属性，上面扩了 nominal 就已经是 8
        act_dim = self.gain_space.dim

        self.action_space = spaces.Box(-1.0, 1.0, shape=(act_dim,),
                                       dtype=np.float32)
        # 回合上下文，只给 critic 用（见模块说明）：
        # [载重/4, 目标距离/R_MAX, sin(初始方位角), cos(初始方位角)]
        self.observation_space = spaces.Box(-2.0, 2.0, shape=(4,),
                                            dtype=np.float32)

        self._rng = np.random.default_rng(seed)
        self._ep = int(payload_phase)
        self._frozen = np.zeros(act_dim, dtype=np.float64)
        self._build_core(seed)

    # ------------------------------------------------------------------
    def _build_core(self, seed):
        # randomize=True 时孪生每局按 difficulty 采一组扰动（传感器噪声、
        # 力矩误差、风、打滑），见 sample_stm32_disturbance。在完全干净的世界里
        # 训出来的增益只保证「理想条件下能赢」，不保证稳。
        # Domain randomisation is where robustness actually comes from.
        kw = dict(firmware=MODE_STM32_PID, hold_station=False,
                  randomize=self.randomize,
                  episode_seconds=self.episode_seconds,
                  imu_filter=self.imu_filter,
                  seed=int(seed) or 0, gains=None)
        self.core = (make_mujoco_twin(**kw) if self.backend == "mujoco"
                     else STM32Twin(**kw))
        if not self.randomize:
            self.core.set_disturbance(DisturbanceConfig())
        self.core.difficulty = self.difficulty

    def set_difficulty(self, d: float):
        self.difficulty = float(np.clip(d, 0.0, 1.0))
        self.core.difficulty = self.difficulty

    def _apply_gains(self, action):
        g = self.gain_space.action_to_gains(np.asarray(action, dtype=float))
        self.core.fw.g.update({k: v for k, v in g.items()
                               if not k.startswith("nav_")})
        self.nav_kv = float(g["nav_kv"])
        self.nav_kw = float(g["nav_kw"])
        self._current_gains = g
        return g

    # ------------------------------------------------------------------
    def reset(self, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        # 载重分层轮换：保证一个 rollout 里五档都出现
        kg = PAYLOAD_LADDER[self._ep % len(PAYLOAD_LADDER)]
        self._ep += 1
        self.core.set_payload(kg)
        self.core.reset(seed=seed)
        self.payload = kg

        r = self._rng.uniform(TARGET_R_MIN, TARGET_R_MAX)
        th = self._rng.uniform(-np.pi, np.pi)
        self.target = (float(r * np.cos(th)), float(r * np.sin(th)))
        self.t = 0.0
        self._arrived_at = None
        self._hold_run = 0.0
        self._prev_ccr = 0.0
        self._frozen = np.zeros(self.gain_space.dim, dtype=np.float64)
        self._apply_gains(self._frozen)
        st = self.core.state
        b = float(np.arctan2(self.target[1] - st[1], self.target[0] - st[0])
                  - st[IPSI])
        self._ctx = np.array([kg / 4.0, r / TARGET_R_MAX,
                              np.sin(b), np.cos(b)], dtype=np.float32)
        return self._ctx.copy(), {}

    # ------------------------------------------------------------------
    def step(self, action):
        """一个动作 = 一组增益 = 一整局试验。返回整局累计奖励。"""
        self._frozen = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._apply_gains(self._frozen)
        total, info = 0.0, {}
        best_d, max_sway = 9.9, 0.0
        n_max = int(round(self.episode_seconds / 0.04))
        for _ in range(n_max):
            r, info, done = self._tick()
            total += r
            best_d = min(best_d, info["dist"])
            max_sway = max(max_sway, info["sway"])
            if done:
                break
        info = dict(info)
        info.update(payload=self.payload, gains=dict(self._current_gains),
                    best_dist=best_d, max_sway=max_sway)
        return self._ctx.copy(), float(total), True, False, info

    def _tick(self):
        """推进一个智能体拍，返回 (奖励, info, 是否结束)。"""
        st = self.core.state
        dx = self.target[0] - float(st[0])
        dy = self.target[1] - float(st[1])
        v_ref, yaw_ref = navigate(dx, dy, float(st[IPSI]),
                                  self.nav_kv, self.nav_kw)
        self.core.set_command(v_ref, yaw_ref)
        _, _, term, trunc, info = self.core.step()
        self.t += 0.04

        st = self.core.state
        dist = float(np.hypot(self.target[0] - st[0], self.target[1] - st[1]))
        pitch = abs(float(st[ITH]))
        prate = abs(float(st[ITHD]))

        rew = R_ALIVE - W_PITCH * pitch - W_PRATE * prate - W_DIST * dist
        ccr = float(np.mean(np.abs(np.array(self.core.fw_ccr, dtype=float))))
        rew -= W_CHATTER * abs(ccr - self._prev_ccr) / PWM_FULL
        self._prev_ccr = ccr

        steady = dist <= ARRIVE_R and pitch <= ARRIVE_SWAY
        if self._arrived_at is None and dist <= ARRIVE_R:
            # 用时只统计到**第一次**到达。之后的震荡由保持段惩罚承担，
            # 所以「冲过去再荡回来」自己就不划算，不需要再加一项。
            # Time is scored only to first arrival; overshoot pays for itself
            # through the hold-phase penalty.
            self._arrived_at = self.t
            rew += R_ARRIVE * max(0.0, 1.0 - self.t / self.episode_seconds)

        if self._arrived_at is not None:
            rew -= W_HOLD_POS * dist + W_HOLD_PITCH * pitch
            self._hold_run = self._hold_run + 0.04 if steady else 0.0

        won = self._hold_run >= HOLD_SECONDS
        if won:
            rew += R_WIN
        fell = bool(term) or bool(getattr(self.core, "fell", False))
        if fell:
            rew += R_FELL

        info = dict(info)
        info.update(dist=dist, won=won, arrived=self._arrived_at,
                    fell=fell, sway=abs(float(st[ITH])))
        return float(rew), info, bool(fell or won or trunc)

    # ------------------------------------------------------------------
    def gains_from_action(self, action):
        return self.gain_space.action_to_gains(
            np.clip(np.asarray(action, dtype=float), -1.0, 1.0))
