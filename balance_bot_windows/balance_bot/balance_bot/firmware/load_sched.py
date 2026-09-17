# -*- coding: utf-8 -*-
"""负载模式与增益。 / Load modes and gains.

=== 一套打不了 0~4 kg，冲突在 D 项 ===
    空车      需要 kd <= 0.48，否则 ~31 Hz 剧烈抖（真车 v2 的症状）
    4 kg 行驶 需要 kd >= 0.96，否则起步加速时摔

中间没有安全值。kd 从 0.56 到 0.72，空车抖动 23.5 -> 107.3 deg/s，
**4.6 倍的跳变发生在 0.16 的区间里**：
    kd    空车抖动   主频     4kg行驶  0kg行驶
    0.48   23.6      49.2Hz    0/5     5/5
    0.56   23.5      46.7Hz    0/5     5/5
    0.64   42.4      35.7Hz    4/5     5/5   <- 唯一两头都沾边的，正在悬崖上
    0.72  107.3      31.7Hz    5/5     5/5
    0.96  114.4      30.8Hz    5/5     5/5
0.64 两侧一边抖 5 倍、一边 4 kg 开不动，没有余量。孪生里能站的针尖点到真车
上参数一偏就掉下去 —— v2 就是这么翻的。所以不取 0.64，分两档。

=== 为什么必须测行驶 ===
只测静止会得出「一套通吃」的错误结论：kd=0.48 静止时 0~4 kg 全部 10/10，
但 4 kg 一起步就摔（1/5，全摔在加速段）。存活率也会藏住抖动 —— kd>=0.72
在空车下行驶存活 5/5，但陀螺 rms 是原厂的 5 倍。
**存活率是必要条件，不是充分条件。** 抖动要单独在 200 Hz 上看频谱。

=== 两个模型对这件事是一致的 ===
离散极点判据说 kd>=0.72 空车下有个 33~36 Hz 的不稳模态；MuJoCo 实测主频
30~32 Hz、幅值 5 倍。线性模型算成无界发散，非线性里被死区限幅成有界振荡。
同一件事，两种说法。而且判据说 kd=0.96 从 1.0 kg 起全程收敛 —— 这就是
分档能成立的原因：**31 Hz 那个模态只在空车时存在**。

=== 为什么切换是手动的 ===
负重在这台车上基本不可观，试过抖振辨识，两个问题都是硬的：
1. 抖振本身会把车推倒。400 counts（满量程 5.6%）在 4 kg 下直接摔，
   3000 counts 在 1 kg 以上全摔。可用幅值窗口是空的。
2. 往车顶加重既加质量也抬质心，轮轴反冲把多出来的俯仰惯量抵消掉大半：
       C_eff = C - (M*l)^2 / (M + 2*I_wheel/r^2)
   3.5->4.0 kg 时 C 变 13.4%，C_eff 只变 3.5%。实测特征跨 0~4 kg 全程只差
   1.29 倍，每 0.5 kg 约 3%。
平衡车本来就是被设计成对质量不敏感的，这是结构性质，不是测量精度问题。

误判的代价是不对称的：判成 HEAVY 而车是空的 -> 立刻 31 Hz 剧烈抖。
所以宁可手动，和原厂 weight_mode_flag 一个用法。
见 scratchpad/ident3.py、kdcurve.py、chatter.py、twobin.py。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GainMode:
    name: str
    balance_kp: float
    balance_kd: float
    velocity_kp: float
    velocity_ki: float
    turn_kp: float = 14.0
    turn_kd: float = 0.20
    # 0.Large program Normal 模式的 Set_Mid_Angle() 给 0。原来的 1.0 抄自
    # 04.bluetooth_control/main.c，已不是 baseline。load_ctrl 本身不写中值，
    # 所以真车上负载档跑的也是固件的 0。
    mid_angle_deg: float = 0.0
    lo: float = 0.0               # 适用载重下限 kg
    hi: float = 4.25              # 上限 kg
    note: str = ""

    def gains(self) -> dict:
        return dict(balance_kp=self.balance_kp, balance_kd=self.balance_kd,
                    velocity_kp=self.velocity_kp, velocity_ki=self.velocity_ki,
                    turn_kp=self.turn_kp, turn_kd=self.turn_kd,
                    mid_angle_deg=self.mid_angle_deg)


# 行驶存活/5 与该载重下静止抖动 deg/s（原厂空车参照 20.8）：
#   0kg 5/5 22.5 | 1kg 5/5 1.2 | 1.5kg 5/5 1.0 | 2kg 5/5 0.9 | 3kg 5/5 8.1
#   4kg 1/5 10.2  <- 上限就在这里，所以分界往下留一整 kg 的余量
# 位置环 = Velocity_Ki。固件的「速度环」其实是 P(速度) + I(位置)：
#     Encoder_Integral += Encoder_bias        # 速度的积分 = 位置
#     velocity = -Encoder_bias*Vkp - Encoder_Integral*Vki
# 所以 Vki 是位置增益，Vkp 是它的阻尼。
#
# vkp 93->110 / vki 0.46->0.92：空车站定漂移 233->128 mm、游走 74->48 mm，
# 行驶存活仍 6/6。**而且检测余量没掉**（2.72x -> 2.68x），这一点是选它而不是
# 选更激进的 124/1.20 的原因 —— 后者游走 37 mm 更好看，但余量掉到 1.63x。
#
# 【这里有个结构性权衡】检测器靠「空车在 NORMAL 下会抖」工作，而位置阻尼
# 正好压的就是那个抖。位置控制做得越好，负载越不可观：
#     vkp  vki  | 0kg游走 | 检测间隔 余量
#      93 0.46  |  74mm   |  7.4x  2.72x
#     110 0.92  |  48mm   |  7.2x  2.68x   <- 选这个，免费
#     124 0.92  |  42mm   |  4.4x  2.11x
#     124 1.20  |  37mm   |  2.7x  1.63x
# 真车上所有量都会偏，余量是唯一能吸收偏差的东西，别为 6 mm 去换 22% 余量。
NORMAL = GainMode(
    "normal", 384.0, 0.48, 110.0, 0.92, lo=0.0, hi=2.0,
    note="默认，含空车。3 kg 还是 5/5，分界留了 1 kg 余量")

# 0kg 5/5 122.0(!) | 1kg 5/5 9.3 | 1.5kg 5/5 8.0 | 2kg 5/5 6.9
# 3kg 5/5 2.2 | 4kg 5/5 11.2  —— 从 1.0 kg 起就比 NORMAL 还安静
# kd 从 0.96 提到 1.20：0.96 在 4 kg 行驶下是 9/10，1.20 是 10/10。
# 更要紧的是**邻域**——kd +-20% x vkp -17% 那一片全是 6/6，空车也全 6/6：
#     kd \ vkp     129        155        186
#     1.00      6/6(6/6)   6/6(6/6)   0/6(6/6)   <- 悬崖在这个角
#     1.20      6/6(6/6)   6/6(6/6)   5/6(6/6)
#     1.44      6/6(6/6)   6/6(6/6)   6/6(6/6)
#     （4kg 存活/6，括号内 0kg 存活/6）
# 只看中心点满分就选，正是 v2 那次的错法；这里选的是一片高地的中间。
# 注意悬崖在 **vkp 往上** 的方向：vkp 186 + kd 1.0 直接 0/6。别再往上抬 vkp。
# vki 0.78->1.20 是纯赚：4 kg 游走 35->28 mm，而**空车下的检测信号一点没动**
# （117.4 deg/s，两者相同）。因为 HEAVY 的检测靠的是它用在空车上时那个失稳
# 模态的幅度，由 kd 决定，与 vki 无关。NORMAL 那边没有这个便宜可占。
HEAVY = GainMode(
    "heavy", 384.0, 1.20, 155.0, 1.20, lo=2.0, hi=4.25,
    note="仅 >=2 kg。空车开启会立刻 31 Hz 剧烈抖（>120 deg/s，原厂的 6 倍）")

# 原厂 = 0.Large program 的 Normal 模式（app_mode.c Set_PID），增益从
# controllers.PID_GAIN_SETS 取，不在这里再抄一遍。
# 下面 note 里的存活数是 2026-09-10 在旧 baseline（04.bluetooth_control 增益、
# 死区补偿 1300）上测的，没有在新 baseline 上重测。
def _factory():
    from .controllers import pid_gains
    g = pid_gains("Normal")
    return GainMode("factory", g["balance_kp"], g["balance_kd"],
                    g["velocity_kp"], g["velocity_ki"],
                    turn_kp=g["turn_kp"], turn_kd=g["turn_kd"],
                    mid_angle_deg=g["mid_angle_deg"],
                    note="原厂 0.Large program Normal 模式。旧 baseline 上"
                         "行驶时 1 kg 起 0/5；静止时 2.5 kg 以上摔（未重测）")


FACTORY = _factory()

MODES = {m.name: m for m in (NORMAL, HEAVY, FACTORY)}
DEFAULT = NORMAL

# 分界。重叠区是 1.0~3.0 kg（两档在这一段都是 5/5 且都安静），分界取中间的
# 2.0 kg，两边各留 1 kg。迟滞是为了避免在分界上来回跳 —— 但这只在有人手动
# 反复拨的时候才用得上，没有自动辨识。
BOUNDARY_KG = 2.0
HYSTERESIS_KG = 0.5


def select(weight_mode: bool = False) -> GainMode:
    """对应固件的 weight_mode_flag。手动，不自动辨识 —— 原因见模块头。"""
    return HEAVY if weight_mode else NORMAL


def bin_for(payload_kg: float) -> GainMode:
    """按真实载重取档。仿真和标定用；车上没有这个数。"""
    return HEAVY if payload_kg >= BOUNDARY_KG else NORMAL
