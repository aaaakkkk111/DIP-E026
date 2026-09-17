"""真车的物理参数，取自固件。 / Physical parameters of the real car, taken from the firmware.

下面每一个数字都引自出厂源码里某一具体行。固件自己前后矛盾的地方，这里如实记录
而不是抹平——参考实现里的矛盾是基线的一部分性质，不是孪生该偷偷修掉的东西。

Every number below is quoted from a specific line of the shipped source.
Where the firmware itself is inconsistent, that is recorded rather than
smoothed over -- an inconsistency in the reference is a property of the
baseline, not something a twin should quietly fix.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from ..params import RobotParams, SimParams

# --------------------------------------------------------------------------
# 来自 6.LQR/Matlab/parameter_LQR.m / From 6.LQR/Matlab/parameter_LQR.m
# --------------------------------------------------------------------------
#   m         = 0.035            单个轮子质量 kg / wheel mass, kg (each)
#   r         = 0.0672/2         轮子半径 m / wheel radius, m
#   inertia   = 0.5*m*r^2        轮子绕自身轴的转动惯量 / wheel inertia about its axle
#   M         = 1.000 - 2*m      车体质量（整车 1.000 kg）/ body mass
#   L         = 0.5*0.0766       轮轴到质心 m / axle -> centre of mass, m
#   J_centroid= (1/12)*M*(0.0766^2 + 0.0575^2)   绕质心的俯仰惯量 / pitch inertia
#   d         = 0.1612           轮距 m / track width, m
#   J_Y_delta = 同 J_centroid    偏航惯量 / yaw inertia
# 以下两个值改用 3D 模型实测，不再用 parameter_LQR.m 的标称值。
# 来源：STM322/10.attch/3Dmodel/STM32_Balance_V2.STEP（SolidWorks 2021 导出，
# AP214 装配体，16 个零件 / 45 个装配实例）。
#
# 轮距 0.1612 -> 0.1670
#   两个「平衡车轮」实例的平移相距 140.00 mm，但轮子的局部原点在**端面**上
#   （局部 Y 范围 0..27 mm），不在中平面。解出两实例的旋转矩阵后，两轮的
#   局部 +Y 分别指向 (-1,0,0) 和 (+1,0,0)——镜像朝外，所以中平面间距是
#   140.00 + 27.00 = 167.00 mm。两轮的 Y、Z 中点完全相同（轴高 Z=99.50），
#   确认同轴。
#
# 轮子半径 0.0672/2 -> 0.0670/2
#   CAD 里唯一直径 67.00 的圆柱面（r=33.5，两个面，正是两个轮子）。这同时
#   和固件 app_motor.h 的 Diameter_67 一致；Matlab 的 67.2 才是那个异类。
#
# 注意**不要**改 FirmwareConstants.wheel_spacing_mm（=161.0）。那是固件自己
# 相信的常数，偏航率 bug 就建立在它上面，必须保持原样才能复现真车行为。
# 物理用 167.0，固件用 161.0，两者本来就不必相等。
#
# Track and wheel radius now come from the CAD model rather than the Matlab
# nominal values.  The wheel part's local origin sits on a face, not the mid
# plane, so the 140.00 mm between instance origins becomes a 167.00 mm track
# once the mirrored placements are resolved.  Do NOT touch
# FirmwareConstants.wheel_spacing_mm: that is what the firmware believes, and
# the yaw-rate bug is built on it.
#
# 质量、质心高度、转动惯量仍来自 parameter_LQR.m —— STEP 只有几何，没有材料
# 和密度，算不出质量。Mass, COM height and inertia still come from the Matlab
# script: a STEP file carries geometry, not materials.
_M_WHEEL = 0.035
_R_WHEEL = 0.0670 / 2.0
# 整车 942 g，官方规格（yahboom 产品页）。原来写 1.000 是 parameter_LQR.m 的
# 标称值。Official kerb mass is 942 g, not the Matlab script's 1.000 kg.
_M_BODY = 0.942 - 2.0 * _M_WHEEL
# 质心与俯仰惯量，**全部由硬数据推出**（2026-09-13）。
#
# 用户用悬挂法量得整车质心 **离地 65 mm**（法线与中轴线交点）。
#   轮轴离地 = 轮半径 33.5 mm  ->  整车质心在轮轴以上 65 - 33.5 = 31.5 mm
#   两个轮子共 70 g 就在轴心（0 mm），剥出来才得车体质心：
#       l_body = 942 g * 31.5 mm / 872 g = 34.0 mm
# 注意这和 parameter_LQR.m 原来的 0.5*0.0766 = 38.3 mm 很接近——**质心本来
# 就大体是对的，错的是惯量**。中途曾把它当拟合参数调到 65/75 mm，都已作废。
#
# I_body（绕车体质心）按部件算，不再用 Matlab 那个等效方块：
#   电机 JGB37-520 两个共 300 g，装在**轮轴上**，即车体质心下方 34.0 mm
#       -> 300 g * 0.034^2 = 3.47e-4
#   其余 572 g，形心由质心约束定死在 51.9 mm（车身高 106.1 mm，合理）
#       -> 平行轴 572 g * (51.9-34.0)^2 + 自身展开 (1/12)*572g*(106.1^2+84^2)
#   合计 1.403e-3，是 Matlab 等效方块 6.67e-4 的 **2.1 倍**。
#   那个方块是 76.6 x 57.5 mm，而真车外形是 106 x 84 mm ——尺寸小一倍，
#   惯量就少一倍。这是纯建模错误，不是拟合问题。
#
# COM from the user's plumb-line measurement (65 mm above the ground; the axle
# sits at the 33.5 mm wheel radius, and the 70 g of wheels at the axle must be
# taken out to get the body's own COM).  I_body is now built from the parts --
# 300 g of motors at the axle plus the remainder at the height the COM
# constraint requires -- instead of the Matlab equivalent box, which was sized
# 76.6 x 57.5 mm against a real 106 x 84 mm body and so understated it 2.1x.
_L_COM = 0.0340
_M_MOTORS = 0.300                     # JGB37-520 x2，每个 150 g，装在轮轴上
_M_REST = _M_BODY - _M_MOTORS
_H_REST = _M_BODY * _L_COM / _M_REST  # 其余质量的形心，由质心约束定死
_J_PITCH = (_M_MOTORS * _L_COM ** 2
            + _M_REST * (_H_REST - _L_COM) ** 2
            + (1.0 / 12.0) * _M_REST * (0.1061 ** 2 + 0.084 ** 2))
_TRACK = 0.1670

# 偏航惯量：Matlab 脚本对车体取 J_Y_delta = J_centroid，而两个轮子位于 +-d/2 处，
# 所以它们的贡献在这里补上。（脚本是把这一项折进了 B 矩阵；物理相同，只是记账
# 方式不同。）
#
# Yaw inertia: the Matlab script uses J_Y_delta = J_centroid for the body, and
# the two wheels sit at +-d/2, so their contribution is added here.  (The
# script folds this into its B matrix instead; same physics, different
# bookkeeping.)
_J_YAW = _J_PITCH + 2.0 * (_M_WHEEL * (_TRACK / 2.0) ** 2
                           + 0.25 * _M_WHEEL * _R_WHEEL ** 2)


@dataclass(frozen=True)
class FirmwareConstants:
    """固件自己用的常数，保持固件的单位。
    Constants the firmware itself uses, in the firmware's own units."""

    # --- app_motor.h ---
    control_hz: float = 200.0          # 控制频率 / Control_Frequency
    diameter_mm: float = 67.0          # 轮径（注意 != Matlab 的 67.2）/ Diameter_67
    wheel_spacing_mm: float = 161.0    # 轮距（注意 != Matlab 的 161.2）/ Wheel_spacing
    encoder_multiples: float = 4.0     # 编码器倍频 / EncoderMultiples
    encoder_lines: float = 11.0        # 编码器线数 / Encoder_precision
    reduction_ratio: float = 30.0      # 减速比 / Reduction_Ratio

    # --- app_control.c ---
    # x_speed = (encL+encR)/2 * PI * Diameter_67/1000/1560 * Control_Frequency
    #
    # 1560 是 LQR 回路假定的「每圈计数」。但 app_motor.h 写的是 4 * 11 * 30 =
    # 1320。固件自相矛盾：LQR 回路因此把速度读小了 1320/1560 = 0.846 倍。孪生
    # 复现固件的 1560（真车就是这么跑的），而*物理*用真值 1320，于是由此产生的
    # 速度误差就和实机台架上一样，成为基线的一部分。
    #
    # 1560 is the counts-per-wheel-revolution the LQR loop assumes.  But
    # app_motor.h says 4 * 11 * 30 = 1320.  The firmware contradicts itself:
    # the LQR loop therefore under-reads speed by 1320/1560 = 0.846.  The twin
    # reproduces the firmware's 1560 (that is what the real car does) while
    # the *physics* uses the true 1320, so the resulting speed error is part
    # of the baseline exactly as it is on the bench.
    lqr_counts_per_rev: float = 1560.0
    true_counts_per_rev: float = 4.0 * 11.0 * 30.0        # = 1320

    # --- bsp.c: BalanceCar_PWM_Init(2880, 0) -> 72 MHz / 2880 = 25 kHz ---
    pwm_period: int = 2880             # 满占空比对应的定时器计数 / full duty in timer counts
    pwm_limit: int = 2600              # app_control.c PWM_Limit(.., 2600, -2600)
    # 这里有两个 1300，含义完全不同，不要合并：
    #   pwm_deadband      —— 固件**假设**的电机静摩擦，1300，只用于对照属性
    #                        （duty_dead / v_cmd_at_full_duty），不进力矩链。
    #                        电机**实际**的占空比死区是另一个数，在
    #                        MotorCalibration.motor_deadband（拟合 665）。
    #                        2026-09-13 我一度把这两个合并了，结果四个描述
    #                        固件行为的测试全红——正是下面这条警告说的事。
    #   pwm_deadband_comp —— **固件的**补偿量，app_motor.c 里的
    #                        MOTOR_IGNORE_PULSE，一个 #define。固件假设它等于
    #                        静摩擦，于是给每个非零指令加上它。原厂两者相等，
    #                        所以最小非零输出就是 1301/2880 = 45% 占空比，
    #                        电机几乎从不真正断电——这就是那个嗡嗡响的死区极限环。
    #
    # Two 1300s with different meanings; do not merge them.  pwm_deadband was
    # ASSUMED equal to the compensation and is no longer part of the torque
    # chain; pwm_deadband_comp is the firmware's MOTOR_IGNORE_PULSE, quoted
    # from app_motor.c:5, whose own comment says the value "needs to be
    # fine-tuned in static state" (and cites 1450 for 25 kHz).  The smallest
    # non-zero output is therefore 45 % duty.
    pwm_deadband: int = 1300           # 固件假设值 / firmware's assumption
    # app_motor.c:5  #define MOTOR_IGNORE_PULSE (1300)
    #   厂家注释：「死区 1450 25Khz 这个值需要在静止状态微调」
    #   -> 1300 是出厂起点，不是这台车的实测值
    # 【2026-09-16 实测，用户定义的 baseline】开环扫描测得电机死区 ~1500。
    # **本项目的原厂 baseline = 用户手里这台真车 + MOTOR_IGNORE_PULSE 1500 的
    # 固件**，所有增益集（包括原厂那 7 套）都跑在这个值上。源码里的 1300 是
    # 亚博发布时的起点，不再是 baseline。
    # 改成 1500 之后同一套增益从「能站但振荡大」变成「原地几乎不动」（倾角
    # std 0.25°）。
    # Measured 2026-09-16.  The project's baseline is the user's real car with
    # MOTOR_IGNORE_PULSE 1500; every gain set, factory ones included, runs on
    # it.  The 1300 in Yahboom's source is no longer the baseline.
    pwm_deadband_comp: int = 1500      # 固件补偿 / MOTOR_IGNORE_PULSE
    ratio_accel: float = 2400.0        # 加速度->PWM 系数 / app_control.c Ratio_accel

    # --- 安全保护 / safety, app_motor.c Turn_Off() ---
    fail_angle_deg: float = 40.0       # 超过这个角度切断电机 / motors cut beyond this
    battery_cutoff_v: float = 9.6      # 低压保护 / low-battery cutoff
    battery_nominal_v: float = 12.0    # 标称电压 / nominal battery voltage

    # --- app_control.c: 俯仰轴的机械零点 / mechanical zero of the pitch axis ---
    target_angle_rad: float = 0.0349   # Target_angle_x (= 2.0 度 / deg)

    @property
    def dt(self) -> float:
        return 1.0 / self.control_hz

    @property
    def r_wheel_firmware(self) -> float:
        """*固件所以为的*轮子半径，米。
        Wheel radius as the *firmware* believes it, metres."""
        return 0.5 * self.diameter_mm / 1000.0

    @property
    def counts_to_metres_lqr(self) -> float:
        """一个编码器计数对应多少米，按固件的 1560 算。
        One encoder count -> metres of travel, using the firmware's 1560."""
        return (np.pi * self.diameter_mm / 1000.0) / self.lqr_counts_per_rev

    @property
    def counts_to_metres_true(self) -> float:
        return (np.pi * self.diameter_mm / 1000.0) / self.true_counts_per_rev


STM32_FIRMWARE_CONST = FirmwareConstants()


# --------------------------------------------------------------------------
# 同一台车，用仿真器的 RobotParams 表达
# The same car expressed in the simulator's RobotParams
# --------------------------------------------------------------------------
STM32_CAR = RobotParams(
    m_body=_M_BODY,
    l_com=_L_COM,
    # 零件估算值 x1.015：2026-09-16 模式 1 静止基准拟合（见 motor.py MotorCalibration）。
    # 质心高度是铅垂线实测的，拟合给出 x0.9988（0.04 mm），不改。
    # Parts-based estimate scaled by the replay fit; the plumb-line COM stays.
    I_body=_J_PITCH * 1.0152,
    I_yaw=_J_YAW,
    m_wheel=_M_WHEEL,
    r_wheel=_R_WHEEL,
    track=_TRACK,

    # 电机转子惯量，**已折算到输出端**（乘 30^2）。RobotParams.A 的注释一直
    # 写着「两个轮子 + 车体 + 转子惯量」，公式里却漏了这一项。
    # 它是轮轴上的主导惯量：轮盘自身只有 0.5*0.035*0.0335^2 = 1.96e-5。
    #
    # 取 6.6e-4 = 几何估算：转子约 30 g、有效半径约 7 mm -> 7e-7，经 30:1 乘 900。
    #
    # 【2026-09-13 解除旧否决】此前这里是 3.0e-4，理由是「6.6e-4 以上原厂 LQR
    # 会倒」。那个否决是在**错的** I_body (6.67e-4) 和 l_com (38.3mm) 下判的；
    # 两者换成硬数据（1.40e-3 / 34.0mm）之后重扫，6.6e-4 一路到 4.2e-3 都是
    # LQR 稳、4kg 可用。**旧结论作废，几何值可以直接用。**
    #
    # 效果：空载静止的俯仰峰峰 17.87° -> 1.35°，对上用户实测的「倾角很小」。
    # 代价：位置摆动掉到 2.3 mm（用户实测是肉眼可见的来回），见下面的已知缺口。
    #
    # The geometric estimate, restored.  The earlier rejection ("LQR falls
    # above 6.6e-4") was made under the wrong I_body and l_com; with both now
    # from hard data, everything up to 4.2e-3 keeps LQR and the 4 kg load
    # working.  Peak pitch drops 17.87 -> 1.35 deg, matching the real car.
    # 【2026-09-16】回放拟合 6.33e-4（几何估算 6.6e-4，差 4%）。
    # 注意它现在挂在关节 armature 上（mjcf.py），弹性传动开启时由转子自由度
    # 承担（twin_baseline.py），不再是轮子刚体的一部分。
    I_rotor=1.202e-03,

    # 12 V 下单轮堵转力矩。这是固件*唯一*没有钉死的数——见 motor.py，其余全部
    # 是从固件自己的 PWM 标定里辨识出来的。0.20 N·m 是这类套件常配的 12 V /
    # 30:1 减速电机的典型值；哪天你实测了真车，这是第一个该重新拟合的参数。
    #
    # Stall torque per wheel at 12 V.  This is the ONE number the firmware
    # does not pin down -- see motor.py, where everything else is identified
    # from the firmware's own PWM calibration.  0.20 N m is typical of the
    # 12 V / 30:1 gearmotors these kits ship with; it is the first thing to
    # re-fit if you ever log the real car.
    # 0.40 N·m。在 MuJoCo 上重划的（用户只用 MuJoCo，analytic 不再当约束）：
    #   力矩  LQR空载   空载增益+0kg（=GUI 默认）   负载+0kg   负载+4kg行驶
    #   0.35    稳    摆0.55° 位 10mm 离地 0.0%   摆 7.5°     倒   <- 带不动
    #   0.40    稳    摆2.66° 位125mm 离地 4.0%   摆 6.6°     可   <- 用这个
    #   0.45    稳    摆5.97° 位148mm 离地10.4%   摆11.9°     可
    #   0.50    稳    摆6.61° 位107mm 离地 6.2%   摆14.8°     可
    #   0.60    稳    摆7.86° 位108mm 离地 6.6%   摆15.7°     可
    # 四条真车行为（LQR 稳 / 空载增益小幅摆且不漂 / 负载增益空载来回振荡 /
    # 4kg 可用）在 0.40~0.60 都满足，而 0.40 是这段里空载振荡最小的。
    #
    # 【上表是在**旧**电机模型下测的】同日 motor.py 换成了三参数直流直线
    # （见那里的模块说明）。0.40 在新模型下复核过，四条依旧全过，而且空载
    # 默认从 摆2.66°/位125mm/离地4.0% 变成 摆0.06°/位0.6mm/离地0.0%。
    # 但**整个窗口没有在新模型下重扫**——0.40 是「验证可用」，不是「验证最优」。
    # 要动这个数，先在新模型下把表重画一遍。
    # The table above was measured under the OLD motor model.  0.40 was
    # re-verified under the new one (all four behaviours still hold, and the
    # no-load default improved to 0.06 deg / 0.6 mm / 0 % airborne), but the
    # window itself was NOT re-swept.  Redraw the table before changing this.
    #
    # 下界是「4kg 要能开动」卡的：0.35 只是能驻车，一给前进指令就倒。
    # 注意这条约束必须用**行驶**测，只测站立会把下界误判到 0.35。
    #
    # 上界是用户报告的「空载默认又开始振荡」卡的。之前这里是 0.60，那个值
    # 来自 analytic 后端上「4kg 带不动」的扫描（tau4.log）；MuJoCo 上 0.40 就够。
    # 0.60 时空载停车后振荡发散（停后 6 s 的摆幅/刚停时 = 1.37），0.40 收敛（0.69）。
    #
    # 量级上：JGB37-520 的 19:1 档 datasheet 线性外推到 30:1 约 1.47 N·m，
    # 0.40 是它的 27%。
    #
    # 【测量方法更正】旧注释说「不要用少于 3 个种子判这个参数」。实测发现：
    # 在 randomize=False + DisturbanceConfig() 全零的配置下，8 个种子的结果
    # **逐位相同**——种子根本不起作用，所谓「N 种子」是同一次运行跑了 N 遍。
    # 要真的统计重复，必须开 randomize=True 或给非零扰动。
    #
    # Re-pinned on MuJoCo (the analytic backend is no longer a constraint).
    # The lower bound is "must DRIVE with its rated 4 kg" -- testing only
    # standstill misplaces it at 0.35.  The upper bound is the user's report
    # that the no-load default oscillates: at 0.60 the post-stop transient
    # diverges (1.37), at 0.40 it converges (0.69).  Note: with randomize off
    # and zero disturbance the seed does nothing -- eight seeds are bit
    # identical, so "N seeds" in the old sweeps was one run repeated N times.
    # 【2026-09-13 拟合尝试，已回退，别再试】300 组五参数联合搜索
    # （tau_damp / 电机死区 / tau_max / b_pitch / solref）对着用户实测的四条：
    #     前后峰峰 40mm / 换向间隔 1.4s / 倾角 1~3° / 无高频振荡
    # 最优解 tau_damp 0.065、死区 665、tau_max 0.406、b_pitch 0.0001、
    # solref 0.038 装上之后：峰峰 58.4mm、倾角 1.50° 两项对上，但
    #   - 换向间隔仍是 0.12s，比真车 1.4s **快 12 倍**（300 组里最慢也才 0.16s）
    #   - 40 秒净漂移 56.5mm，而真车不漂
    #   - analytic 后端跑到 0.7mm，和 mujoco 差 83 倍
    # 用户判定「确实也不符合」，已全部回退。**结论是缺结构不是缺参数**：
    # 五个参数任意组合都到不了 1.4s 的周期，那个时间尺度在模型里不存在。
    # A five-parameter fit reproduced amplitude and tilt but never the 1.4 s
    # period (best of 300 was 0.16 s), so it was reverted.  The missing thing
    # is a structure with that time scale, not a parameter value.
    tau_max=0.40,
    # 官方规格：电机转速 333±10 rpm（1:30 减速后的输出端）。
    # 333 rpm = 34.9 rad/s = 1.17 m/s 轮缘速度。原来写 25.0（239 rpm）低了 28%，
    # 那是从固件 Ratio_accel 的标定反推的「固件以为的」速度，不是电机能力。
    # Official 333 rpm at the output shaft; the old 25.0 came from the
    # firmware's own PWM calibration, which is what the firmware believes, not
    # what the motor does.
    wheel_speed_max=34.9,        # 轮端 rad/s = 333 rpm = 1.17 m/s

    # 摩擦在这里不是独立参数：固件那个 1300/2880 = 45% 的死区已经隐含了大部分。
    # 见 motor.py。
    # Friction is not free-standing here: the firmware's 1300/2880 = 45 %
    # dead band implies most of it.  See motor.py.
    b_wheel=0.0,
    b_pitch=0.0005,
    b_yaw=0.0008,
    rolling_mu=0.010,

    # 外形尺寸。官方 194 x 84 x 139.59 mm，和 CAD 装配三处独立对上：
    #   194    = 轮距 167 + 轮宽 27                    （精确）
    #   139.59 <-> 轮半径 33.5 + 车顶 105.2 = 138.70   （差 0.6%）
    #   84     <-> 底盘前后 83.5                        （匹配）
    #
    # CAD 的 X 是左右（两轮位置差在这个轴上），所以 body_width 对应 X。
    # 前后尺寸用官方的 84，不用点云取的 151.6——那个数是 B 样条控制点撑出来的
    # 假值（控制点落在实际曲面之外），比整车官方尺寸还大。
    #
    # 更早还用过 76.6 x 60.0 x 75.0，那是 parameter_LQR.m 算转动惯量用的等效
    # 方块，不是车的外形，拿来渲染就画成一台又小又矮的车。
    #
    # **I_body 仍然按那个等效方块算（见上面的 _J_PITCH），没有按外形重算。**
    # 外形大不等于惯量大——惯量取决于质量怎么分布，而 STEP 里没有材料密度。
    # 两者不一致是有意的，别顺手「统一」。
    #
    # Official envelope, cross-checked three ways against the CAD.  I_body
    # still comes from the Matlab inertia box on purpose: envelope size is not
    # inertia, and STEP carries no densities.
    body_height=0.1061,          # 139.59 - 33.5 轮半径
    body_width=0.194,            # 含两个轮子
    body_depth=0.084,
    body_center=0.0473,
    collision_radius=0.12,
    side_profile=(),   # 见文件末尾，构造后回填 / filled in below
)


def stm32_sim_params(episode_seconds: float = 20.0) -> SimParams:
    """和固件 200 Hz 回路对齐的仿真时序。
    Simulation timing that lines up with the firmware's 200 Hz loop.

    ``dt_ctrl`` 正好是固件的 5 ms 一拍，积分步长能整除它。智能体那一拍保持
    25 Hz，这样 RL 侧可以和通用机器人直接对比。

    ``dt_ctrl`` is exactly the firmware's 5 ms tick, and the integrator step
    divides it evenly.  The agent tick stays at 25 Hz so the RL side is
    directly comparable with the generic robot.
    """
    return SimParams(
        dt_phys=0.00125,          # 800 Hz：这个摆约 13 rad/s / this pendulum is ~13 rad/s
        dt_ctrl=1.0 / STM32_FIRMWARE_CONST.control_hz,
        dt_agent=0.040,
        episode_seconds=episode_seconds,
        # 固件在 40 度切断电机（Turn_Off），所以判定失败的是这个阈值，而不是
        # 仿真器随手定的一个数。
        # The firmware cuts the motors at 40 deg (Turn_Off), so that -- not an
        # arbitrary simulator threshold -- is when the real car has failed.
        pitch_fail=np.deg2rad(STM32_FIRMWARE_CONST.fail_angle_deg),
    )


def describe() -> str:
    p = STM32_CAR
    c = STM32_FIRMWARE_CONST
    return "\n".join([
        "STM32 balance car (Yahboom), from the shipped firmware",
        f"  body mass        {p.m_body:.3f} kg   (+2 x {p.m_wheel:.3f} kg wheels"
        f" = {p.m_body + 2 * p.m_wheel:.3f} kg total)",
        f"  COM height       {p.l_com * 1000:.1f} mm above the axle",
        f"  wheel radius     {p.r_wheel * 1000:.2f} mm   track {p.track * 1000:.1f} mm",
        f"  pitch inertia    {p.I_body:.3e} kg m^2   yaw {p.I_yaw:.3e}",
        f"  natural freq     {p.omega_n:.2f} rad/s  (time constant"
        f" {1 / p.omega_n * 1000:.0f} ms)",
        f"  control loop     {c.control_hz:.0f} Hz",
        f"  PWM              25 kHz, period {c.pwm_period}, limit +-{c.pwm_limit},"
        f" dead band {c.pwm_deadband} ({c.pwm_deadband / c.pwm_period * 100:.1f} %)",
        f"  encoder          {c.true_counts_per_rev:.0f} counts/rev"
        f"  (LQR loop assumes {c.lqr_counts_per_rev:.0f} -- firmware bug, reproduced)",
        f"  fails at         {c.fail_angle_deg:.0f} deg (motors cut by Turn_Off)",
    ])


# --------------------------------------------------------------------------
# 侧面轮廓：给 UI 画的，不参与物理
# Side profile for the UI only; not used by the physics
# --------------------------------------------------------------------------
# 从 STM322/10.attch/3Dmodel/STM32_Balance_V2.STEP 解出来的：45 个装配实例
# 变换到整车坐标系后，按零件取包围盒，同名的多件（铜柱 x4）合并成一个包络，
# 螺丝略去。单位米，**原点在轮轴**，前后 = CAD 的 Y，高 = CAD 的 Z。
#
# 为什么要有这个：仿真器把车体当成**一个长方体**，而 UI 的侧视图以前更是把
# 前后尺寸写死成 0.09 m、矩形从轮轴往上画。真车的车身其实从轮轴**下方**
# 11.5 mm 一直到上方 106.1 mm，中间是底盘、电池仓、铜柱、雷达板这么几层。
# 要真的和 CAD 一模一样得把 STEP 转成网格（需要 CAD 内核，这里没有），
# 用各零件的投影矩形拼是最接近的折中。
#
# The simulator models the body as one box, and the side view used to hard
# code its fore-aft size to 0.09 m and draw it from the axle up.  The real
# body runs from 11.5 mm *below* the axle to 106.1 mm above it in several
# layers.  Matching the CAD exactly would need a mesh (and a CAD kernel);
# per-part projected rectangles are the closest thing available here.
STM32_SIDE_PROFILE = (
    # (名字, 前后 y0, y1, 高 z0, z1)
    ('联轴器', -0.0074, +0.0067, -0.0068, +0.0068),
    ('减速电机L', -0.0155, +0.0155, -0.0115, +0.0255),
    ('平衡车V2底盘', -0.0426, +0.0409, -0.0115, +0.0347),
    ('电池仓', -0.0758, +0.0758, +0.0305, +0.0575),
    ('电池仓盖', -0.0388, +0.0388, +0.0483, +0.0655),
    ('铜柱M3-32+6', -0.0221, +0.0321, +0.0602, +0.1022),
    ('蝙蝠车超声波', -0.0403, -0.0213, +0.0693, +0.0960),
    ('超声波亚克力挡板', -0.0243, -0.0143, +0.0738, +0.1052),
    ('雷达固定板', -0.0400, +0.0441, +0.0952, +0.1052),
)


# STM32_CAR 是 frozen dataclass，构造时还拿不到下面定义的轮廓，所以在这里回填。
# STM32_CAR is a frozen dataclass, so the profile is patched in afterwards.
object.__setattr__(STM32_CAR, "side_profile", STM32_SIDE_PROFILE)


# 零件盒子：给 MuJoCo 视图画的，**只是外观，不参与碰撞和惯量**。
# 从同一份 STEP 解出来，逐实例（左右两个电机、四根铜柱各自一个），轮子跳过
# ——MJCF 里轮子是独立 body，有自己的转动关节。
# 坐标原点在轮轴，MuJoCo 的 x=前后 y=左右 z=上（CAD 的 X 是左右、Y 是前后，
# 这里已经换过轴）。
#
# Per-instance part boxes for the MuJoCo view: appearance only, no collision
# and no inertia.  Wheels are skipped -- they are separate bodies with hinges.
STM32_PART_BOXES = (
    ('coupler_l', -0.0074, +0.0066, -0.0820, -0.0640, -0.0068, +0.0068),
    ('coupler_r', -0.0074, +0.0067, +0.0640, +0.0820, -0.0067, +0.0067),
    ('motor_l', -0.0155, +0.0155, -0.0790, -0.0042, -0.0115, +0.0255),
    ('motor_r', -0.0155, +0.0155, +0.0042, +0.0790, -0.0115, +0.0255),
    ('chassis_plate', -0.0426, +0.0409, -0.0600, +0.0600, -0.0115, +0.0347),
    ('battery_box', -0.0758, +0.0758, -0.0916, +0.0916, +0.0305, +0.0575),
    ('battery_lid', -0.0388, +0.0388, -0.0458, +0.0458, +0.0483, +0.0655),
    ('post_1', -0.0221, -0.0169, -0.0315, -0.0265, +0.0602, +0.1022),
    ('post_2', +0.0269, +0.0321, -0.0315, -0.0265, +0.0602, +0.1022),
    ('post_3', +0.0271, +0.0319, +0.0264, +0.0316, +0.0602, +0.1022),
    ('post_4', -0.0219, -0.0171, +0.0264, +0.0316, +0.0602, +0.1022),
    ('ultrasonic', -0.0403, -0.0213, -0.0228, +0.0224, +0.0693, +0.0960),
    ('acrylic_guard', -0.0243, -0.0143, -0.0330, +0.0330, +0.0738, +0.1052),
    ('radar_plate', -0.0400, +0.0441, -0.0700, +0.0460, +0.0952, +0.1052),
)
object.__setattr__(STM32_CAR, "part_boxes", STM32_PART_BOXES)
