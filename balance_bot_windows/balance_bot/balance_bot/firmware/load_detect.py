# -*- coding: utf-8 -*-
"""自动档位切换：不测负重，测症状。 / Automatic mode switching on symptoms.

=== 为什么不是「辨识负重」 ===
负重本身在这台车上测不出来。主动抖振那条路走死了（抖振本身会把车推倒，
而且特征跨 0~4 kg 只差 1.29 倍，见 load_sched 模块头）。被动信号里，
「装了多少」同样测不出来 —— 静止陀螺 rms 在 0.5~4 kg 全是 3.1~3.8 deg/s，
起步加速峰值也是平的（M_eff 的 2.6 倍被速度环调节掉了）。

**但「空 / 不空」分得极开，10 倍余量，而且两个档位下都成立：**
        载重      NORMAL 下      HEAVY 下
        0 kg      39.5 deg/s    118.3 deg/s
        0.5 kg     3.8            14.3
        1 kg       3.1             8.8
        2 kg       3.5             7.1
        4 kg       3.6             7.2
所以档位边界跟着**可检测性**走，不是跟着最优性走：NORMAL = 空车，
HEAVY = 装了货（不管多少）。

=== 这个状态机是自纠错的 ===
                抖动持续偏低（装了货）
      NORMAL ───────────────────────> HEAVY
             <───────────────────────
                抖动偏高（车空了）

关键：**误判进 HEAVY 的后果是抖，不是摔** —— 空车用 HEAVY 存活 5/5，只是
118 deg/s 的 31 Hz 抖动。而那个抖动恰好就是退回 NORMAL 的触发信号，所以
错误会立刻自己暴露并纠正。反方向同样安全：该重却停在 NORMAL，而 NORMAL
在 0~3 kg 都是 5/5。

=== 代价 ===
1~1.5 kg 时车跑在 HEAVY 上，比用 NORMAL 吵（8.8 vs 3.1 deg/s）。但 8.8 比
原厂空车的 20.8 还安静，不算退步。

=== 板子上怎么实现 ===
只要陀螺原始值，200 Hz 中断里做，不需要 FFT：
    高通（减掉慢变均值）-> 平方 -> 一阶低通 -> 得到抖动能量
31 Hz 远高于平衡环带宽，慢变的驾驶动作进不来。
"""
from __future__ import annotations

from dataclasses import dataclass, field

# 阈值 = 各自「空车值」与「装货最大值」的几何中点（带通后实测）：
#     NORMAL 下  空车 20.9  装货最大 2.7  ->  7.5
#     HEAVY  下  空车 57.4  装货最大 8.4  ->  21.9
# 余量 2.62x。**这两个数只对电机死区 1300 成立**，死区变了必须重量 ——
# 死区改变 31 Hz 模态的幅值，死区 650 版的阈值是 19.4/36.7，差 3 倍。
# 各死区版本见 keil_out/db650 db900 db1300，生成脚本 scripts/export_keil.py。**这比真车需要的要紧**，所以上车前必须按实测重标，
# 步骤见模块末尾。旧版本写着「2 倍以上余量」，那是在全带宽统计量下算的，
# 而那个统计量对载重非单调（见 update() 里的注释），余量再大也是假的。
LOADED_BELOW = 7.5      # deg/s rms，低于这个判定「装了货」
EMPTY_ABOVE = 21.9       # deg/s rms，高于这个判定「车空了」

# 判定必须持续这么久才算数。31 Hz 的能量估计本身要几百毫秒才稳，
# 而且短暂的路面冲击不该触发换档。
CONFIRM_SECONDS = 2.0

# 换档之后锁定，避免在边界上来回跳（每次换档都是一次闭环参数突变）。
LOCKOUT_SECONDS = 3.0

# 带通的两端。HP 去掉 ~16 Hz 以下（4 kg 撑不住时的低频晃动），
# LP 去掉 ~40 Hz 以上（陀螺噪声）。中心落在空车快模态的 30 Hz 附近。
# 带宽扫过五组，这一组的最窄余量最大（1.77x）。收窄能从 1.55x 提到
# 1.77x 就到顶了 —— 五组都在 1.8x 附近，这是这台车能给的上限。
HP_TAU = 0.006      # 高通拐点 26.5 Hz
LP_TAU = 0.0035     # 低通拐点 45.5 Hz

# 抖动能量的低通时间常数
ENERGY_TAU = 0.30

GYRO_LSB_PER_DEG_S = 16.4


@dataclass
class LoadDetector:
    """逐固件拍（200 Hz）喂原始陀螺 LSB，返回当前该用哪一档。

    ``heavy`` 为 True 表示应该用 HEAVY 增益。**开机从 HEAVY 起**，因为
    风险是不对称的：
        NORMAL 用在 4 kg 上 -> 行驶时摔（5 次只活 1 次）
        HEAVY  用在空车上   -> 抖 121 deg/s，但 5/5 不摔
    HEAVY 在任何载重下都不摔，而空车那个抖恰好是最响亮的检测信号
    （121 vs <=45，2 倍余量）。默认 NORMAL 等于把「可能摔车」放在检测
    失败的那一侧，方向是反的。代价是空车开机会抖一两秒再切回 NORMAL。
    """
    hz: float = 200.0
    heavy: bool = True
    _mean: float = 0.0
    _band: float = 0.0
    _energy: float = 0.0
    _hold: float = 0.0        # 当前判定已经持续了多久
    _lock: float = 0.0        # 换档后的锁定剩余时间
    _primed: bool = False

    @property
    def rms_deg_s(self) -> float:
        return (self._energy ** 0.5) / GYRO_LSB_PER_DEG_S

    def reset(self):
        self._mean = self._band = self._energy = 0.0
        self._hold = self._lock = 0.0
        self._primed = False
        self.heavy = True

    def update(self, gyro_lsb: float, moving: bool = False) -> bool:
        """``moving`` = 车正在被指令行驶。行驶时**冻结判决**。

        抖动统计量只在静止时可标定：行驶时驾驶动作会污染它，实测 0 kg 在
        行驶段会掉进「装了货」的区间，导致多余的换档（默认 NORMAL 两次、
        默认 HEAVY 四次）。而载重只会在有人停下来装卸货时改变，所以行驶中
        根本不需要更新判决。能量估计照常跑，只是不下结论。
        """
        dt = 1.0 / self.hz
        # 带通，不是高通。**这里踩过坑**：原来用 0.5 s 的高通（0.32 Hz 以上
        # 全放行），结果统计量对载重是非单调的 ——
        #     NORMAL 下：0kg 44.8 / 1kg 6.2 / 2kg 8.8 / 3kg 30.1 / 4kg 35.5
        # 两头都高，中间低。因为抖动有两个来源，机制完全相反：
        #     空车  kd 太大 -> ~31 Hz 快模态限幅振荡
        #     4 kg  kd 太小 -> 车撑不住，低频大幅晃动
        # 全带宽能量把两者混成一个数，于是 4 kg 被判成空车、一直留在 NORMAL、
        # 必摔（实测 0/8）。
        #
        # 带通只留 ~16-40 Hz，低频晃动落在带外，快模态留下。两级一阶滤波器，
        # 板子上就是 4 个乘加。
        a_hp = dt / (HP_TAU + dt)
        self._mean += a_hp * (gyro_lsb - self._mean)
        hp = gyro_lsb - self._mean
        a_lp = dt / (LP_TAU + dt)
        self._band += a_lp * (hp - self._band)
        ac = self._band
        a_e = dt / (ENERGY_TAU + dt)
        self._energy += a_e * (ac * ac - self._energy)

        if not self._primed:
            # 能量估计稳定之前不判决，否则开机瞬间的暂态会误触发
            self._hold += dt
            if self._hold >= 3.0 * ENERGY_TAU:
                self._primed, self._hold = True, 0.0
            return self.heavy

        if moving:
            # 能量继续估，判决冻结，计时清零（不让行驶段的片段累积成判决）
            self._hold = 0.0
            return self.heavy

        if self._lock > 0.0:
            self._lock -= dt
            self._hold = 0.0
            return self.heavy

        r = self.rms_deg_s
        want = self.heavy
        if not self.heavy and r < LOADED_BELOW:
            want = True
        elif self.heavy and r > EMPTY_ABOVE:
            want = False

        if want != self.heavy:
            self._hold += dt
            if self._hold >= CONFIRM_SECONDS:
                self.heavy = want
                self._hold = 0.0
                self._lock = LOCKOUT_SECONDS
        else:
            self._hold = 0.0
        return self.heavy


# ---------------------------------------------------------------- 标定
# 阈值是孪生里的数，真车的陀螺噪声底、机械间隙、电机响应都不一样，绝对值
# 几乎肯定不同。余量只有 1.8x，**不标一定不准**。两次读数就够：
#
#   1) 空车，立起来站稳，读 rms_deg_s，记为 R_empty_N（此时在 NORMAL）
#   2) 装 2 kg，站稳，读 rms_deg_s，记为 R_load_N
#          LOADED_BELOW = sqrt(R_empty_N * R_load_N)
#   3) 手动切到 HEAVY，仍装 2 kg，读数记为 R_load_H
#   4) 卸货变空车，**扶住**（这一步车会抖），读数记为 R_empty_H
#          EMPTY_ABOVE = sqrt(R_empty_H * R_load_H)
#
# 不标的后果是安全的：真车 rms 通常比孪生高，检测器会一直判「空车」，
# 结果是一直停在 NORMAL。那在 4 kg 行驶时会摔 —— **所以这一步不能省**。
# （早先我写过「不标只是空车时吵」，那是默认档还是 HEAVY 时的结论，
#   现在默认档由 boot 策略决定，见 twin_baseline.enable_load_detect。）
