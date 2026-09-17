"""轮端力矩模型：PWM 计数进，力矩出。
Wheel-torque model: PWM counts in, torque out.

分界线画在 **CCR 之后**。CCR 之前全是固件，逐行复刻，一个字都不简化：
死区补偿 +1300、钳位 +-2600、LQR「先钳后补」而 PID「先补后钳」、以及
2400*v+1300 超过 2880 周期时的饱和。见 ``firmware_pwm`` 和 controllers.py。

CCR 之后是电机，用一条直流电机直线，三个数：

    tau = tau_stall * duty - k_v * omega - tau_damp * tanh(omega / 0.35)
    k_v = (tau_stall - tau_damp) / omega_noload

  omega_noload  34.9 rad/s = 333 rpm  <- **唯一实测值**（官方规格）
  tau_stall     0.40 N·m              <- 自由参数，按真车四条行为定
  tau_damp      0.15 N·m              <- 自由参数，阻尼

``k_v`` 不是自由的：它由「duty=1 时净力矩恰好在 omega_noload 归零」定死，
所以官方那个 333 rpm 是**约束**而不是摆设。四个象限都正确——高速时给反向
占空比会得到更大的制动力矩，而不是更小。

为什么换掉旧模型
----------------
旧模型写成 ``tau = a*(V - b*omega) - tau_coulomb*sign(omega)``，声称 a、b、
tau_coulomb 是从固件的 ``Ratio_accel = 2400`` 标定里「辨识」出来的。实际上
那条链上有七个数，没有一个是量出来的：

    stall_torque   拟合行为（三天内 0.20 -> 0.60 -> 0.40）
    b_back_emf     从固件的 Ratio_accel 推的
    tau_coulomb    假设「静摩擦 = 固件死区补偿」，2026-09-10 实测证伪
    tau_elec       为拟合「真车极点不抖」加的，但默认 0，从没生效过
    I_rotor        拟合「LQR 必须站得住」
    tanh 的 0.35   随手写的
    钳位的 1.2     随手写的

七个数全部是拿「要复现的行为」反推的——那不是物理模型，是套着物理外壳的
七参数曲线拟合。而且它和唯一一个真实电机数据自相矛盾：它的空载轮速是
**187.7 rpm**，官方规格是 **333 rpm**，差 44%。

关于 Ratio_accel 的取舍
-----------------------
旧模型把固件的 ``Ratio_accel = 2400``（满占空比 = 0.658 m/s）当成电机的
实测值。新模型不这么认：**它是固件的一厢情愿，不是电机的能力。**
本项目已经记录了同类的固件错常数——偏航角速度小 10^6 倍、LQR 按
1560 counts/rev 读而真值是 1320。Ratio_accel 是第三个：固件以为满占空比
跑 0.658 m/s，电机实际能跑 1.169 m/s，所以固件的速度指令整体偏小 44%。
这条差异现在由 test_motor_identification 显式钉住。

The dividing line is drawn **after the CCR**.  Everything upstream is
firmware and is reproduced exactly.  Downstream is one straight DC-motor
line with three numbers, of which only omega_noload is measured (the
official 333 rpm).  k_v is not free: it is fixed by requiring the net
torque to cross zero at omega_noload under full duty, which makes the
official figure a constraint rather than decoration.

The old model claimed to "identify" its constants from the firmware's
Ratio_accel calibration.  In fact none of its seven numbers was measured;
all were fitted to the same behaviours they were meant to reproduce, and
the result contradicted the one real motor figure available (its free
speed was 187.7 rpm against the official 333).  Ratio_accel is treated
here as another firmware misbelief, alongside the yaw rate that is 10^6
too small and the LQR reading 1560 counts/rev against a true 1320.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .robot import STM32_FIRMWARE_CONST, STM32_CAR


@dataclass
class MotorCalibration:
    """轮端力矩直线的三个数，外加固件那边的常数。
    Three numbers for the wheel-torque line, plus the firmware constants."""

    # ---- 自由参数（拟合真车行为）/ free, fitted to real-car behaviour ----
    # 【2026-09-16 真车回放拟合】以下 stall_torque / tau_damp / 两个死区 /
    # 三个传动参数，是用 e026 keil/deadband_test 里的真车录数（模式 1 静止、
    # 幅值 300/500 扫频、Kd120、Kp38400）原样回放、交叉熵法拟合出来的，
    # 稳定状态权重 x3。拟合结果（静止，真车 / 孪生）：
    #   倾角 std 0.25 / 0.26°   陀螺 RMS 8.58 / 8.76 °/s   |PWM| 1532 / 1534
    #   饱和 0 / 0%   扫频共振 12.5 / 12.0 Hz   扫频期间陀螺逐拍相关 0.76-0.85
    # 仍未对上：PWM 换向率 27 / 15 次/s；Kp 38400 真车失稳、孪生不失稳。
    #
    # 堵转力矩和驱动器电流限是两回事：电机能出 0.57 N·m，驱动器把它截在
    # RobotParams.tau_max（0.40）。以前两者强行相等。
    # Stall torque (what the motor makes) is no longer tied to tau_max (what
    # the driver lets through).
    # 【2026-09-17 第十二轮，模式 1 静止基准拟合】用户定的最标准基准（原厂增益、
    # 补偿 1500、空车静止）。8 个没参与拟合的新噪声种子验证，综合误差 0.017
    # （第九轮 0.048）。已进真车范围（孪生/真车）：倾角 std 0.26/0.25、陀螺峰值
    # 27.3/28.1、换向 26.6/27.1、|PWM| 均值 24/32、位移峰峰 2.82/3.27 mm、
    # 位移 std 0.54/0.64、位移主频 4.28/4.29 Hz。
    # 关键结构：**左右电机不是一对**（死区差 99 计数）。真车左右编码器幅值差
    # 20%，两边不同时踢才凑得出真车 3-6 / 6-12 Hz 对半开的频谱；对称孪生的能量
    # 全挤在 4.3 Hz 基频上。仍略高：12-100 Hz 4.61/3.81、位移超低频 0.27/0.17。
    # 上面那段「第一次回放拟合」的数值已被本组取代。
    stall_torque: float = 0.5679              # 12 V 满占空比、零转速时的轮端力矩
    tau_damp: float = 0.00251                 # 阻尼力矩，N·m
    # 电机**实际**的 PWM 死区计数，进力矩链（见 duty()）。和固件**假设**的
    # pwm_deadband 是两个不同的量，别合并——2026-09-13 我合并过一次，四个
    # 描述固件行为的测试当场全红。
    # 【2026-09-16 实测】1500。依据两条独立的真车测量：
    #   1. 开环扫描（离地、补偿关掉、PWM 逐步加）：~1500 轮子开始转
    #   2. 闭环静止时 |PWM| 均值 1532，编码器只有 0.39 计数/5ms —— 刚过死区
    # 旧值 1300 是固件的设计意图（补偿恰好抵消死区），不是测量。
    # 2026-09-13 拟合到 665 的尝试是过补偿区，已回退，别再用。
    # The motor's ACTUAL dead band, measured 2026-09-16 on the real car
    # (open-loop scan, confirmed by |PWM| 1532 giving 0.39 counts/5 ms).
    # 【2026-09-16 回放拟合】正向 1522、反向 1529：比固件补偿 1500 略大，
    # 小指令留下一段很窄的零力矩区。开环扫描测的 ~1500 是轮子离地的值，
    # 着地带载后的等效死区稍高，符合预期。
    # 模式 1 基准拟合：正向 1494、反向 1458，都**低于**补偿 1500——补偿零点是
    # 真正的跳变，小输出也会得到一脚力矩，这就是 4.3 Hz 继电器极限环的来源。
    motor_deadband: int = 1480
    # 反向死区，None = 和正向相同。用户怀疑的「正反不对称」拟合出来只差 7 计数。
    motor_deadband_neg: float | None = 1454.51

    # ---- 传动链弹性 / drivetrain compliance（2026-09-16 回放拟合）--------
    # 证据：真车静止时车身在晃（陀螺 8.6 °/s）而编码器几乎不动（0.39 计数/拍）；
    # 闭环扫频 12.5 Hz 有轻阻尼共振；Kd120 极限环锁在 14 Hz。刚性传动都复现
    # 不了。模型：转子 --(扭簧 k、阻尼 c、间隙 b)--> 轮子，每侧一套。
    # gear_stiffness=None 关掉，回到刚性传动（转子惯量走关节 armature）。
    gear_stiffness: float | None = 1.7183     # N·m/rad，折算到轮端
    gear_damping: float = 0.0745              # N·m·s/rad
    gear_backlash_deg: float = 0.8398         # 单边间隙，轮端角度
    # 左右死区之差（计数，右 - 左）。0 = 两边一样。
    # Left/right dead-band split in counts (right minus left); 0 = matched.
    motor_deadband_lr_split: float = 98.6
    gear_friction: float = 0.00192            # 轮子关节库仑摩擦 N·m（减速箱）

    # ---- 实测 / measured -------------------------------------------------
    # 官方规格「电机转速 333±10 rpm」（1:30 减速后的输出端）= 34.9 rad/s。
    # 这是整条链上唯一一个厂家给的电机数据。
    omega_noload: float = STM32_CAR.wheel_speed_max

    supply_v: float = 12.0
    r_wheel: float = STM32_CAR.r_wheel

    # ---- 来自固件，不要手改 / from the firmware, do not edit by hand -----
    pwm_period: int = STM32_FIRMWARE_CONST.pwm_period
    pwm_deadband: int = STM32_FIRMWARE_CONST.pwm_deadband
    ratio_accel: float = STM32_FIRMWARE_CONST.ratio_accel

    # ---- 派生量 / derived ------------------------------------------------
    @property
    def k_v(self) -> float:
        """转速衰减系数，N·m/(rad/s)。**不是自由参数**：由「duty=1 时净力矩
        恰好在 omega_noload 归零」定死，官方 333 rpm 是约束。
        Speed droop; pinned by the no-load speed, not free."""
        return (self.stall_torque - self.tau_damp) / max(self.omega_noload, 1e-9)

    @property
    def no_load_speed(self) -> float:
        """满占空比空载轮速，rad/s。按构造就等于 omega_noload。
        Free-running wheel speed at full duty; equals omega_noload by design."""
        return self.omega_noload

    # ---- 固件自己的标定，留着做对照，**不参与力矩计算** ------------------
    # The firmware's own calibration, kept only for comparison; it does NOT
    # feed the torque model any more.
    @property
    def duty_dead(self) -> float:
        """固件**假设**的死区占空比 1300/2880 = 45.1%，只做对照。
        The dead band the firmware assumes; comparison only."""
        return self.pwm_deadband / self.pwm_period

    @property
    def duty_dead_motor(self) -> float:
        """电机**实际**的正向死区占空比，进力矩链。
        The motor's actual (forward) dead band, used by the torque model."""
        return self.motor_deadband / self.pwm_period

    def duty_dead_for(self, sign: float, side: int = 0) -> float:
        """按方向（和左右）取死区占空比。
        Dead-band duty for the given direction and wheel."""
        if sign < 0 and self.motor_deadband_neg is not None:
            dead = float(self.motor_deadband_neg)
        else:
            dead = float(self.motor_deadband)
        # 左右电机不是一对：真车模式 1 静止时左右编码器幅值差 20%
        # （0.39 / 0.47 计数），两边死区不同就会在一个周期里踢两次，是真车
        # 陀螺 8.3 Hz 二次谐波的候选来源。side<0 = 左，>0 = 右。
        # The two motors are not a matched pair; a left/right dead-band split
        # makes the relay kick twice per cycle.
        if self.motor_deadband_lr_split:
            dead += 0.5 * float(self.motor_deadband_lr_split) * (1.0 if side > 0 else -1.0)
        return dead / self.pwm_period

    @property
    def v_dead(self) -> float:
        return self.duty_dead * self.supply_v

    @property
    def v_cmd_at_full_duty(self) -> float:
        """固件**以为**满占空比能跑多快（m/s）：(2880-1300)/2400 = 0.658。
        实际电机能跑 omega_noload * r = 1.169 m/s，所以固件低估了 44%。
        What the firmware BELIEVES full duty gives; the motor does 1.169 m/s."""
        return (self.pwm_period - self.pwm_deadband) / self.ratio_accel

    @property
    def firmware_speed_error(self) -> float:
        """固件标定相对电机真实能力的比值。1.0 = 固件是对的。
        Ratio of what the firmware believes to what the motor does."""
        return self.v_cmd_at_full_duty / max(
            self.omega_noload * self.r_wheel, 1e-9)

    def summary(self) -> str:
        return "\n".join([
            "wheel torque line: tau = stall*duty - k_v*w - damp*tanh(w/0.35)",
            f"  stall torque     {self.stall_torque:.3f} N m per wheel"
            f"   <-- free, fitted",
            f"  damping          {self.tau_damp:.3f} N m per wheel"
            f"   <-- free, fitted",
            f"  no-load speed    {self.omega_noload:.1f} rad/s"
            f"  ({self.omega_noload * self.r_wheel:.3f} m/s, "
            f"{self.omega_noload * 30.0 / np.pi:.0f} rpm)   <-- MEASURED",
            f"  k_v              {self.k_v:.5f} N m s / rad"
            f"   (pinned by the no-load speed, not free)",
            "",
            "firmware side (reproduced exactly, not part of the torque model)",
            f"  dead-band comp   {self.duty_dead * 100:.1f} % duty (1300/2880)",
            f"  firmware believes full duty = "
            f"{self.v_cmd_at_full_duty:.3f} m/s, motor does "
            f"{self.omega_noload * self.r_wheel:.3f} m/s"
            f"  ({self.firmware_speed_error * 100:.0f} %)",
        ])


class IdealMotor:
    """理想电机：轮端力矩与补偿以上的 PWM 严格成正比。

    固件对非零输出 u 加上补偿 C 得到 CCR = u ± C，这里原样扣回：
        tau = stall * u / (period - C)
    即死区补偿恰好抵消死区，且没有反电动势、没有阻尼、没有占空比饱和、
    不受电池电压影响。和 scripts/ideal_pid.py 的执行器是同一个式子。
    Torque strictly proportional to the PID output above the compensation;
    no back-EMF, damping, duty saturation or supply dependence.
    """

    def __init__(self, stall_torque: float = 0.5679, comp: int | None = None):
        c = STM32_FIRMWARE_CONST.pwm_deadband_comp if comp is None else int(comp)
        # 只为了让读 cal 的代码（GUI 面板、弹性传动开关）照常工作
        self.cal = MotorCalibration(stall_torque=stall_torque, tau_damp=0.0,
                                    motor_deadband=c, motor_deadband_neg=None,
                                    gear_stiffness=None, gear_backlash_deg=0.0,
                                    gear_friction=0.0)
        self.stall = float(stall_torque)

    def reset(self):
        pass

    def duty(self, ccr: float, side: int = 0) -> float:
        return float(ccr) / self.cal.pwm_period

    def torque(self, ccr: float, omega_wheel: float,
               supply_v: float | None = None, dt: float | None = None,
               side: int = 0) -> float:
        if ccr == 0:
            return 0.0
        c = self.cal.motor_deadband
        u = float(ccr) - np.sign(ccr) * c
        return float(self.stall * u / (self.cal.pwm_period - c))


class Yahboom370Motor:
    """输入 PWM 计数，输出轮端力矩——固件的各种毛病一并保留。
    PWM counts in, wheel torque out -- with the firmware's quirks intact."""

    def __init__(self, cal: MotorCalibration | None = None):
        self.cal = cal or MotorCalibration()

    def reset(self):
        pass

    # ------------------------------------------------------------------
    def duty(self, ccr: float, side: int = 0) -> float:
        """定时器比较值 -> [-1, 1] 区间的带符号占空比。
        Timer compare value -> signed duty in [-1, 1].

        写一个比定时器周期还大的 CCR，输出就直接钉在高电平；固件正是靠这一点，
        因为只要指令速度超过 0.66 m/s，2400*v + 1300 就会超过 2880 的周期。

        Writing a CCR larger than the timer period simply pins the output
        high, which the firmware relies on.
        """
        d = float(np.clip(ccr / self.cal.pwm_period, -1.0, 1.0))
        # **电机自身的占空比死区**，必须留着——固件那句
        # `PWM_Ignore: pulse += 1300` 存在的唯一理由就是抵消它。
        #
        # 【2026-09-13 修回】2026-09-10 重写电机模型时我删掉了电机侧死区
        # （旧的 tau_coulomb），却保留了固件的 +1300 补偿。后果：
        #     需求 1 计数 -> 占空比 (1+1300)/2880 = 45.2% -> **45.2% 的净力矩**
        #     正确应为     -> 45.1% 被死区吃掉 -> 净 0.1% -> 力矩 ~= 0
        # 差 450 倍。于是任何微小需求都变成全力猛踹，孪生退化成继电器系统：
        # CCR 从不为零、平均占空比 66%、折算电流 2555 mA（真车约 440 mA）、
        # 并锁进一个 33.3 Hz = 200/6 的极限环——这个环对质心、惯量、转子惯量、
        # Kd 全都免疫，因为它根本不是物理模态，是继电器反馈的产物。
        # 修回之后：静止倾角 1.35° -> 0.09°，33 Hz 环消失，CCR 开始出现零拍。
        #
        # The firmware's +1300 exists solely to cancel the motor's own duty
        # dead band.  Removing the latter while keeping the former made a
        # 1-count demand produce 45 % duty of NET torque (450x too much) and
        # turned the twin into a relay system locked at 200/6 Hz, immune to
        # every physical parameter because it was not a physical mode.
        # 扣掉死区后**重新归一**：满占空比仍然给出规格的堵转力矩，
        # 只是小指令那一段被摩擦吃掉。不归一的话车的总权限凭空少一半。
        # Re-normalised so full duty still yields the rated stall torque.
        dd = self.cal.duty_dead_for(d, side)
        return float(np.sign(d) * max(0.0, abs(d) - dd) / (1.0 - dd))

    # ------------------------------------------------------------------
    def torque(self, ccr: float, omega_wheel: float,
               supply_v: float | None = None, dt: float | None = None,
               side: int = 0) -> float:
        """给定 PWM 指令和当前轮速，算轮端力矩。
        Wheel torque for a PWM command and the current wheel speed.

            tau = stall*duty - k_v*omega - tau_damp*tanh(omega/0.35)

        阻尼项用 tanh 在 0.35 rad/s 的窄带里平滑，免得积分器在过零点抖。
        它是**必须**留的：2026-09-10 实测把阻尼从 0.27 降到 0.02，静止摆幅
        7.3° -> 10.3°、离地占比 13.8% -> 32.8%。阻尼是阻尼器，不是激励源。

        ``dt`` 和 ``side`` 保留只为兼容调用点；新模型没有内部状态。
        ``dt`` and ``side`` are accepted for call-site compatibility; this
        model is stateless.
        """
        c = self.cal
        duty = self.duty(ccr, side)
        if supply_v is not None:
            duty *= float(supply_v) / c.supply_v
        drive = c.stall_torque * duty - c.k_v * omega_wheel
        return float(drive - c.tau_damp * np.tanh(omega_wheel / 0.35))

    # ------------------------------------------------------------------
    def steady_state_speed(self, ccr: float,
                           supply_v: float | None = None) -> float:
        """这个 PWM 最终会稳到多少轮速（m/s）。
        Wheel speed (m/s) this PWM settles at.

        解 stall*duty - k_v*w - damp = 0（转起来之后 tanh -> 1）。
        驱动力矩顶不过阻尼就不动。
        """
        c = self.cal
        duty = self.duty(ccr)
        if supply_v is not None:
            duty *= float(supply_v) / c.supply_v
        duty = float(np.sign(duty) * max(0.0, abs(duty) - c.duty_dead_motor)
                     / (1.0 - c.duty_dead_motor))
        drive = c.stall_torque * duty
        if abs(drive) <= c.tau_damp:
            return 0.0
        w = (abs(drive) - c.tau_damp) / max(c.k_v, 1e-9)
        return float(np.sign(drive) * w * c.r_wheel)


# ----------------------------------------------------------------------
def firmware_pwm(v_cmd: float, const=STM32_FIRMWARE_CONST) -> int:
    """固件的「速度 -> PWM」通路，原样复现。
    The firmware's speed -> PWM path, reproduced exactly.

    ``app_control.c``::

        velocity_L = (int)(Ratio_accel * (x_speed + L_accel/Control_Frequency));
        Motor_Left = PWM_Limit(velocity_L, 2600, -2600);
        Motor_Left = PWM_Ignore(Motor_Left);          // += 1300 * sign

    注意顺序：钳位发生在加死区偏置*之前*，所以实际的比较值能到 3900——远超
    2880 的定时器周期。这不是移植时的 bug，板子上就是这么跑的。

    Note the order: the clamp happens *before* the dead-band offset is added,
    so the actual compare value reaches 3900 -- well past the 2880 timer
    period.  That is not a bug in this port; it is what the board does.
    """
    p = int(v_cmd * const.ratio_accel)
    p = int(np.clip(p, -const.pwm_limit, const.pwm_limit))
    if p > 0:
        p += const.pwm_deadband_comp
    elif p < 0:
        p -= const.pwm_deadband_comp
    return p
