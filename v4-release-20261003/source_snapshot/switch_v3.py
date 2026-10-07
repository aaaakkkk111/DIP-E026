# -*- coding: utf-8 -*-
"""切换器 v3：**判定锁存** —— 修 v2 那个把自己判定推翻的闭环。

v2 的结构缺陷（2026-09-30 实测，驻停 14 s 逐拍跟踪）：

    负载   探测窗末(33拍)   1个时间常数(61拍)      1.0s        3.0s        末了
    0kg    14.35/0.30      **17.44/0.00**    5.90/0.65   9.13/0.44   9.24/0.43
    1kg     5.16/0.75        6.35/0.62       2.50/0.87   3.53/0.81   3.34/0.82
    2kg     4.42/0.78        4.87/0.72       2.00/0.91   0.38/1.00   2.66/0.86
    4kg     4.18/0.79        4.24/0.76       1.72/0.93   2.22/0.89   2.25/0.89
    （格式：抖振读数 / 当时的 f）

**判定在 61 拍那一刻是对的**：空载 f = 0.00，1/2/4 kg = 0.62/0.72/0.76，单调。
然后闭环把它毁掉 —— f 降到 0，增益变软，抖振从 17.44 掉到 5.90，而映射按
「5.9 = 有载」又把 f 推回 0.65，最后稳在 0.43。空载因此一直在用半重载增益，
抖振 9.0（原厂 NORMAL 只有 0.6）。

根因：**映射是在 HEAVY 增益下标定的，却被用在任意当前增益上。**
`chatter_switch.py` 的检测表整张都是「HEAVY 档上抖振随负载单调下降」，
那张表只有在 f=1 时成立。这不是参数没调好——再怎么搜 OSC_EMPTY 都修不了，
因为标定基准本身在动。CEM 搜了两轮（70.77 -> 44.88）也没能把空载拉下来。

v3 的两条改动：

1. **判定锁存。** 在 f=1（HEAVY，标定基准所在的那一档）下探测满
   PROBE_S，算一次 want，跳过去之后**不再更新判定**。载重在一局里不会变，
   持续重估没有信息增益，只有闭环污染。

2. **单向安全逃生。** 锁存之后只允许 f **往上**走，而且只在车明显吃力时。
   理由同 v1 就有的升降不对称：漏判负载会摔（不可恢复），误判重载只是抖
   （可恢复）。逃生不触发时 f 保持锁存值 —— 满足「0 = 什么都不做」。

   吃力的判据用倾角偏离的低通（相对重力，坡上才不会误触发）。

PROBE_S 默认取 0.305 s = 抖振能量那级 EMA 的时间常数（1/0.016393 = 61 拍）。
v2 被 CEM 压到 0.164 s 是因为那时判定会被后面持续修正，锁存之后就必须让
读数收敛了 —— **结构变了，最优参数也得重搜**（v1->v2 时同样如此）。

    python scripts/rl/switch_v3.py        # 和 v2、原厂两档对比
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for q in (ROOT, HERE):
    if q not in sys.path:
        sys.path.insert(0, q)

from balance_bot.firmware.controllers import pid_gains          # noqa: E402

G_N = dict(pid_gains("Normal"))
try:
    G_H = dict(pid_gains("Weight_M"))
except Exception:
    G_H = dict(G_N, balance_kp=192.0, balance_kd=1.50,
               velocity_kp=94.5, velocity_ki=0.47, turn_kp=14.0)
KEYS = ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki",
        "turn_kp", "turn_kd")

# --- CEM 可搜的参数（**这里就是最终值，唯一真值**）---
# 来源：cem_v3.py 第一轮（runs_e2e/cem_v3.log）最后一代的精英均值。
# 手挑初值依次是 0.305 / 15.0 / 3.0 / 1.0 / 0.05 / 6.0。
#
# 2026-09-30 按 realcar/PARAMS.md 修正孪生（反向死区补偿 1480 -> 1455）后
# 重搜过一次（runs_e2e/cem_v3_revdb.log），得到
#     0.3222 / 9.0058 / 1.6273 / 0.8796 / 0.0256 / 7.1686
# 搜索集上只好 2.9%（噪声量级），全表上**被否决**（final_table4.log）：
#   2kg +8N 从 4/4 变 3/4（2 kg 被判得更轻，f 0.39 -> 0.23，挨推扛不住），
#   总成功 36 -> 35/48；空载三格新判据收敛各慢约 0.4 s。
#   这一格不在搜索的 5 个工况里 —— 典型的对搜索集过拟合。
# 所以保留下面这组。它在修正后的孪生上的全表：36/48（final_table3_revdb.log）。
# 以后重搜，搜索集要加「带载 + 推」的工况。
PROBE_S = 0.2042     # 探测时长（秒，只算驻停期）
OSC_EMPTY = 11.9606  # 探测末的抖振（°/s）高于此 -> 判空车
OSC_FULL = 2.8626    # 低于此 -> 判满载
JUMP_FRAC = 0.8804   # 锁存时一次性跳到目标的比例
RATE_UP = 0.022      # 安全逃生的升档速率（每拍）
STRUGGLE_DEG = 7.1031  # 倾角偏离低通超过这个值（度）算「吃力」
STRUGGLE_LPF = 0.01  # 那个低通的系数
# 逃生提升的衰减时间常数（秒）。**必须有**：第一版逃生是单向覆盖 f，
# 结果空载挨一下 3 N 冲激就被永久钉在重载档，抖振 33.5（v2 是 9.1）——
# 一次瞬时扰动把空车变成「人拨错档」那个失效模式。
# 正确结构是：逃生 = 叠在锁存值之上的**临时**提升，事后衰减回锁存值。
# 和 L2 的冲击门同一个哲学：门一关就回到基准。
BOOST_TAU = 1.5
RELATCH_S = 0.0      # >0 则驻停这么久后重新探测一次（0 = 永不重探）

# --- 探测何时结束（2026-09-30 加）---
# "fixed"：旧逻辑，累计静止满 PROBE_S 就锁存。
# "converge"：**连续**静止满 PROBE_MIN_S，且最近 CONV_WIN_S 秒读数的
#   (max-min)/mean <= CONV_TOL 才锁存；连续静止满 PROBE_MAX_S 强制锁存。
#
# 为什么改：GUI 里车上电就站着，抖振读数（能量 EMA 时间常数 0.305 s，
# 要 ~1.2 s 才到 2% 以内）还在变就被 0.204 s 的探测锁死，1/2/4 kg 全判成
# 半载。而 factory_vs_auto 的测试一直是「开局就走、停了才探测」，读数在
# 行驶中早就跑热了 —— 门限是配着那个时序调出来的。另外旧逻辑的 n_stop 是
# **累计**的（车动了也不清零），几次短停可以拼成一次「探测」。
# 默认 converge（2026-09-30 起）。旧行为：PROBE_MODE="fixed" + MAP_MODE="linear"。
PROBE_MODE = "converge"
PROBE_MIN_S = 1.2    # 连续静止至少这么久（约 4 个能量 EMA 时间常数）
CONV_WIN_S = 0.3     # 判稳窗口（约 1 个时间常数）
CONV_TOL = 0.15      # 窗口内 (max-min)/mean 不超过这个才算稳
PROBE_MAX_S = 3.0    # 连续静止满这么久强制锁存（别一直停在硬档上抖）

# --- 读数 -> f 的映射（2026-09-30 加）---
# "linear"：旧逻辑，OSC_EMPTY / OSC_FULL 之间线性插值。
# "two_level"：读数 >= OSC_SPLIT 判轻载用 F_LIGHT，否则判重载用 F_HEAVY。
#
# 为什么：读数收敛以后（修好积分器的孪生，HEAVY 档上电站立 2 s）只剩两团：
#     0 kg 33.4   0.5 kg 22.9   1 kg 20.5   |   2 kg 0.57   4 kg 0.47
# ≤1 kg 在硬增益下持续自激，≥2 kg 被压住，两团差 40 倍以上，团内不可分。
# 在只有两个取值的信号上做线性插值，f 实际只取两个值，却要靠两个门限的
# 位置去间接编码它们 —— 不如直接写成两档，每个数都有含义。
# OSC_SPLIT 取 3.0：离重团（~0.5）和轻团（~20）都有 5 倍以上余量。
MAP_MODE = "two_level"

# --- 行驶中粗判「轻」（2026-10-01）---
# 只做「判轻」这一个方向：判错的代价不对称，把重车判轻会摔。HEAVY 档行驶中
# 读数（匀速 0.3 m/s / 来回加减速，2 种子，scratchpad/moving_osc.py）：
#     0 kg 26.0-34.9   0.5 kg 16.0-23.5   1 kg 4.8-20.5   2 kg ≤13.2   4 kg ≤11.2
# 分界 20：对重车最大值 13.2 留 1.5 倍；空车最小 26 一定判轻。窗口内**每个**
# 读数都要 ≥ 分界才判（防尖峰）。为什么要：开局就走时空车整段都在 HEAVY 上
# 探测不了，全表「先开」空车抖振 4.6-5.9（站立开局只有 0.5）。
MOVE_PROBE = True
MOVE_SPLIT = 20.0
# 事件触发的重新探测（推击/升档之后，由 v4 置 event_probe）判轻也用保守分界，
# 且窗口内每个读数都要过。实测（2026-10-01）：2 kg 推 15 N 后 stiff 升档 ->
# 重新探测，推后 1.22 s 就锁存，读数 8.32 -> 判轻 f=0.12（车上有 2 kg！）。
# 2 kg 稳态读数 0.3-0.6，8.32 是推击后没衰减完的残余，在一个暂时平坦的位置
# 骗过了「读数稳定」判据。推后残余最高见到 14.4，取 20 和行驶中同一依据。
EVENT_SPLIT = 20.0

# 换档时按速度环积分增益的新旧比例缩放积分（无扰切换），和固件一致。
BUMPLESS = True
# （2026-10-01 试过升档时不缩放：空载 6N 推击 8 次里 4/8 -> 4/8，无差别，没保留。
#   6N 是所有控制器的悬崖，4 次一组的表里 4/4 和 2/4 都是抽样波动。）
MOVE_MIN_S = 1.0     # 连续行驶至少这么久（读数先跑热）
MOVE_WIN_S = 0.5
# 3.0 -> 23.0（2026-10-01）。3.0 是按标称车定的（带载 0.3-0.6、空载 20-33）。随机化
# 16 台车（scratchpad/probe_split.py）：空载 24.3-37.5，但 2 kg 有 3 台读 18-19、
# 4 kg 有 1 台读 8.2、种子 4 的 2 kg 读 2.9 —— 全被判轻，其中一台下台阶摔了。
# 两种错判代价不对等：重判轻 = 轻档带重物会摔；轻判重 = 档位偏硬，照样收敛。
# 所以「有把握才判轻」：分界放在空载那团正下方（空载最低 24.3，带载最高 22.1）。
# 实车读数尺度不同，按同一规则标定：分界 = 空载最低读数下方留余量。
OSC_SPLIT = 23.0
# grid_v3.py（修好积分器的孪生，两种开局 + 2kg+8N，runs_e2e/grid_v3.log）：
#   F_LIGHT 0.12 明显最好（0 -> 总扣分 40，1 kg 用近 NORMAL 会出事）；
#   F_HEAVY 0.3/0.4 打平（28.80/28.78），取 0.4 往硬的一侧留余量。
#   对照 F_HEAVY 1.0（=纯 HEAVY）：2kg 抖振 0.8->0.4、4kg 收敛 3.24->3.01 s。
# 2026-10-01 分界改 23 后「轻」只剩空载、「重」覆盖 1-4 kg，重新网格过
# （tune_v4_map.py，随机场地 64 局 + 行驶 8 工况）：重档 0.55/0.70 少摔 4/64（在波动内），
# 但持续抖振 8 -> ~50、不收敛 8 -> ~20（1 kg 在硬档上抖）；轻档 0/0.06 空载过冲小 ~1 cm，
# 摔倒多 1-3 局。差别不大或得不偿失 -> 保留原值（用户规则：没进步就用原来的）。
F_LIGHT = 0.12
F_HEAVY = 0.4

_HP, _LP, _EN = 0.200849, 0.334511, 0.016393


class SwitchV3:
    """接口同 PIDActor：ctl(env) -> action。"""

    def __init__(self, gain_hook=None):
        from calibrate_reward import PIDActor
        self.pid = PIDActor(dict(G_H))      # 从安全端（标定基准）起步
        self.gain_hook = gain_hook
        self.f = 1.0
        self.hp = self.bp = self.en = 0.0
        self.osc = 0.0
        self.n_stop = 0
        self.latched = False
        self.f_latched = 1.0
        self.struggle = 0.0
        self.boost = 0.0               # 叠在锁存值之上的临时提升
        self.escape_ticks = 0          # 逃生开了多少拍（证据列）
        self.boost_max = 0.0           # 提升过的最大量（证据列）
        self.osc_at_latch = 0.0        # 锁存时的读数（证据列）
        self.n_still = 0               # **连续**静止拍数（converge 模式用）
        self.win = []                  # 最近 CONV_WIN_S 秒的读数
        self.t_latch = None            # 在第几拍锁存（证据列）
        self.n_tick = 0
        self.n_move = 0                # 连续行驶拍数
        self.mwin = []                 # 行驶中最近 MOVE_WIN_S 秒的读数
        self.move_latched = False      # 证据列：是不是在行驶中判的
        self.event_probe = False       # 由事件（推击/升档）触发的重新探测
        self._sref = None              # v3 单用时慢速逃生的平衡角基准

    def _gains(self):
        return {k: G_N.get(k, 0.0)
                + (G_H.get(k, 0.0) - G_N.get(k, 0.0)) * self.f for k in KEYS}

    def __call__(self, env, moving=None):
        from balance_bot.rl.e2e_env import FS
        tw = env.core
        g = float(tw._gyro_lsb)
        self.hp += _HP * (g - self.hp)
        self.bp += _LP * ((g - self.hp) - self.bp)
        self.en += _EN * (self.bp * self.bp - self.en)
        self.osc = float(np.sqrt(max(self.en, 0.0))) / 16.4

        if moving is None:
            # 「有指令」= 前后速度或左右转向任一非零（2026-10-01 用户：转向也算指令）。
            # 原地转弯时车身会晃，之前只看速度指令，转弯被当成静止：探测窗口在
            # 转弯中照样计时、v4 的静止噪声估计把转弯晃动算进去（门限抬到 16.8°）。
            moving = abs(float(env.v_ref)) > 1e-6 or abs(float(getattr(env, "yaw_ref", 0.0))) > 1e-6
        self.n_tick += 1
        if not moving:
            self.n_stop += 1
            self.n_still += 1
            self.win.append(self.osc)
            nw = max(int(CONV_WIN_S * FS), 1)
            if len(self.win) > nw:
                del self.win[0]
        else:
            self.n_still = 0
            self.win = []
        if moving:
            self.n_move += 1
            self.mwin.append(self.osc)
            if len(self.mwin) > max(int(MOVE_WIN_S * FS), 1):
                del self.mwin[0]
        else:
            self.n_move = 0
            self.mwin = []

        if (not self.latched and MOVE_PROBE and MAP_MODE == "two_level"
                and abs(self.f - 1.0) < 1e-3      # 只在标定点 f=1 判（f>1 是紧急档）
                and self.n_move >= int(MOVE_MIN_S * FS)
                and len(self.mwin) >= max(int(MOVE_WIN_S * FS), 1)
                and min(self.mwin) >= MOVE_SPLIT):
            # 行驶中只能判「轻」；判不了就继续留在 HEAVY，停下再细判
            self.t_latch = self.n_tick
            self.osc_at_latch = self.osc
            self.f_latched = float(np.clip(F_LIGHT, 0.0, 1.0))
            self.f = self.f_latched
            self.latched = True
            self.move_latched = True

        if not self.latched:
            # --- 探测期：f 保持在 1，也就是检测表标定的那一档 ---
            if PROBE_MODE == "converge":
                ready = False
                if self.n_still >= int(PROBE_MAX_S * FS):
                    ready = True
                elif (self.n_still >= int(PROBE_MIN_S * FS)
                      and len(self.win) >= max(int(CONV_WIN_S * FS), 1)):
                    lo, hi = min(self.win), max(self.win)
                    mean = max(sum(self.win) / len(self.win), 1e-6)
                    ready = (hi - lo) / mean <= CONV_TOL
            else:
                ready = self.n_stop >= int(PROBE_S * FS)
            if ready:
                self.t_latch = self.n_tick
                self.osc_at_latch = self.osc
                if MAP_MODE == "two_level":
                    # 两档：直接到位，不走 JUMP_FRAC（它只对插值有意义）
                    if self.event_probe:
                        light = bool(self.win) and min(self.win) >= max(OSC_SPLIT, EVENT_SPLIT)
                    else:
                        light = self.osc >= OSC_SPLIT
                    self.f_latched = float(np.clip(
                        F_LIGHT if light else F_HEAVY, 0.0, 1.0))
                    self.event_probe = False
                else:
                    want = float(np.clip(
                        (OSC_EMPTY - self.osc)
                        / max(OSC_EMPTY - OSC_FULL, 1e-6), 0.0, 1.0))
                    self.f_latched = float(np.clip(
                        self.f + JUMP_FRAC * (want - self.f), 0.0, 1.0))
                self.f = self.f_latched
                self.latched = True
        else:
            # --- 锁存后：只有安全逃生能动 f，而且只能往上 ---
            # 倾角偏离要相对**平衡姿态**，否则坡上的正常倾斜会被当成吃力
            # （同一个基准踩过三次，见 BUGS.md 第十节）。
            # 【2026-10-01】原来用孪生的 gravity_pitch —— 真车读不到这个量。
            # 改用实测得到的平衡角：v4 的 th_ref（v4 每拍先算好），v3 单独用
            # 时用自己的同规则慢低通。C 版（v4_core.c）同样用 th_ref。
            ref = getattr(self, "th_ref", None)
            if ref is None:
                a_ = float(tw._angle_filt)
                if self._sref is None:
                    self._sref = a_
                elif abs(a_ - self._sref) < 4.0:
                    self._sref += (a_ - self._sref) / (2.0 * FS)
                ref = self._sref
            err = abs(float(tw._angle_filt) - ref)
            self.struggle += STRUGGLE_LPF * (err - self.struggle)
            if self.struggle > STRUGGLE_DEG:
                self.boost = min(self.boost + RATE_UP, 1.0)
                self.escape_ticks += 1
                self.boost_max = max(self.boost_max, self.boost)
            else:
                # 衰减回锁存值。boost=0 时 f 逐位等于 f_latched ——
                # 「什么都不做」仍然是这一层的恒等元。
                self.boost *= float(np.exp(-1.0 / (BOOST_TAU * FS)))
                if self.boost < 1e-4:
                    self.boost = 0.0
            self.f = float(np.clip(self.f_latched + self.boost, 0.0, 1.0))
            if RELATCH_S > 0 and self.n_stop >= int(RELATCH_S * FS):
                self.latched = False
                self.n_stop = 0

        base = self._gains()
        if self.gain_hook is not None:
            base = self.gain_hook(env, base)
        # 无扰切换（同固件 load_adapt.c la_apply / v4_adapt.c apply）：速度环输出
        # -enc_int*vki，vki 一变而积分不动，输出就跳一下（最多 ~1280 PWM，偏偏
        # 发生在换档那一刻）。按新旧比例缩放积分，让输出连续。
        if BUMPLESS:
            ki_old = float(self.pid.g.get("velocity_ki", 0.0))
            ki_new = float(base.get("velocity_ki", ki_old))
            if ki_old > 0.0 and ki_new > 0.0 and ki_new != ki_old:
                self.pid.enc_int *= ki_old / ki_new
        self.pid.g.update(base)
        return self.pid(env)


def make(gain_hook=None):
    return SwitchV3(gain_hook)


def main():
    import factory_vs_auto as FA
    import switch_v2 as SV2

    V2P = (0.1644, 15.834, 0.588, 0.7839, 0.0216, 0.0756)
    (SV2.PROBE_S, SV2.OSC_EMPTY, SV2.OSC_FULL,
     SV2.JUMP_FRAC, SV2.RATE_DOWN, SV2.RATE_UP) = V2P

    base = FA.Ctl

    class Ctl2(base):
        def __init__(self, kind, kg):
            if kind in ("v2", "v3"):
                self.kind, self.kg = kind, kg
                self.sw = SV2.SwitchV2() if kind == "v2" else SwitchV3()
                self.pid = self.sw.pid
                self.kg_hat, self.est, self.sched = 0.0, None, None
            else:
                base.__init__(self, kind, kg)

        def __call__(self, env):
            if self.kind in ("v2", "v3"):
                a = self.sw(env)
                self.kg_hat = self.sw.f
                return a
            return base.__call__(self, env)

    FA.Ctl = Ctl2
    cases = [("空载 平地", 0.0, 0.0, 0.0, 0.0),
             ("1kg 平地", 1.0, 0.0, 0.0, 0.0),
             ("2kg 平地", 2.0, 0.0, 0.0, 0.0),
             ("4kg 平地", 4.0, 0.0, 0.0, 0.0),
             ("空载 上坡8度", 0.0, 0.0, 0.0, 8.0),
             ("2kg 上坡8度", 2.0, 0.0, 0.0, 8.0)]
    print("v3（判定锁存）vs v2（持续重估）vs 原厂两固定档  —— 3 种子")
    print("%-14s %-13s %6s %9s %9s %7s %6s"
          % ("控制器", "工况", "成功", "旧变稳s", "新变稳s", "抖振", "末f"))
    print("-" * 74)
    for kn, kind in (("基线 NORMAL", "normal"), ("基线 HEAVY", "heavy"),
                     ("切换 v2", "v2"), ("**切换 v3**", "v3")):
        for cn, kg, imp, st, sl in cases:
            rs = [FA.run(kind, kg, imp, st, sd, sl) for sd in (0, 1, 2)]
            p = lambda k: [r[k] for r in rs if r.get(k) is not None]
            ss, qq = p("settle"), p("quiet")
            oo = [r["osc"] for r in rs if r["ok"]]
            hh = p("hat")
            print("%-14s %-13s %4d/3 %9s %9s %7s %6s"
                  % (kn, cn, sum(r["ok"] for r in rs),
                     ("%.2f" % np.mean(ss)) if ss else "  — ",
                     ("%.2f" % np.mean(qq)) if qq else "  — ",
                     ("%.1f" % np.mean(oo)) if oo else "  — ",
                     ("%.2f" % np.mean(hh)) if hh else "  — "))
        print()


if __name__ == "__main__":
    main()
