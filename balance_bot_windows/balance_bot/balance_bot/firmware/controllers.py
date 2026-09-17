"""固件里的两套控制器，按位复现。 / The two shipped controllers, reproduced at the bit level.

两套都以 200 Hz 跑在整数编码器计数上、输出一个 PWM 比较值，和板子上完全一样。
固件哪里做错了，这份代码就照样做错——一个偷偷修好参考实现的基线不叫基线。
每一处都标了 ``FIRMWARE BUG``，可以单独关掉看它值多少分。

Both run at 200 Hz on integer encoder counts and emit a PWM compare value,
exactly like the board.  Where the firmware does something wrong, this code
does the same wrong thing on purpose -- a baseline that quietly fixes the
reference is not a baseline.  Each such case is marked ``FIRMWARE BUG`` and
can be switched off individually to see what it costs.

读任何一个基准数字之前，有两个 bug 值得先知道。
Two of them are worth knowing about before you read a single benchmark number:

1.  **LQR 回路的偏航角速度估计小了约 10^6 倍。**
    **The LQR loop's yaw-rate estimate is ~10^6 too small.**
    ``app_control.c`` 算的是 / it computes:

        gyro_z = (encR - encL) / Wheel_spacing / 1000 * PI*Diameter_67/1000/1560 * f

    ``Wheel_spacing`` 是 161.0 *毫米*，换成米应该除以 1000——但代码是
    *先除 Wheel_spacing、再除 1000*，等于除了 161000 而不是 0.161。于是偏航
    反馈项形同虚设；转向仍然能用，但纯粹是开环，走的是同一个表达式里
    ``Target_gyro_z`` 那一半。

    ``Wheel_spacing`` is 161.0 *millimetres*, so converting to metres means
    dividing by 1000 -- but the code divides *by Wheel_spacing and then by
    1000*, i.e. by 161000 instead of by 0.161.  The yaw feedback term is
    therefore dead; steering still works, but purely open loop, through the
    ``Target_gyro_z`` half of the same expression.

2.  **LQR 回路把速度读小了 15%。** 它用 1560 计数/圈去除编码器计数，而
    ``app_motor.h`` 定义的编码器是 4 x 11 x 30 = 1320 计数/圈。LQR 用到速度和
    位置的地方全都被缩放了 1320/1560 = 0.846 倍。

    **The LQR loop reads speed 15 % low.**  It divides encoder counts by
    1560 counts/rev, while ``app_motor.h`` defines the encoder as
    4 x 11 x 30 = 1320 counts/rev.  Everything the LQR does with speed and
    position is scaled by 1320/1560 = 0.846.

两个工程之间还有一处更隐蔽的差别：LQR 回路是先钳位到 +-2600、*再*加 1300 计数
的死区偏置（所以比较值能到 3900，输出钉在 100% 占空比），而 PID 回路是*先*加
偏置、后钳位（所以永远超不过 90.3% 占空比）。LQR 那台车就是有更大的控制权限。
这一点也照样复现了。

There is also a subtler difference between the two projects: the LQR loop
clamps to +-2600 *before* adding the 1300-count dead-band offset (so the
compare value reaches 3900 and the output pins at 100 % duty), while the PID
loop adds the offset *first* and clamps after (so it never exceeds 90.3 %
duty).  The LQR car simply has more authority.  That is reproduced too.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .robot import STM32_FIRMWARE_CONST as FC

# --------------------------------------------------------------------------
# 增益，逐字抄自固件 / Gains, quoted from the firmware
# --------------------------------------------------------------------------
# 6.LQR/STM32_code/APP/app_control.c
LQR_GAINS = dict(
    K1=-62.0484,    # 位移 / x_pose
    K2=-73.3232,    # 速度误差 / x_speed error
    K3=-361.4617,   # 俯仰角误差 / pitch error
    K4=-35.9024,    # 俯仰角速度 / pitch rate
    K5=15.8114,     # 航向角 / yaw angle
    K6=15.8114,     # 航向角速度误差 / yaw rate error
)

# --------------------------------------------------------------------------
# 模式与增益：唯一来源是 0.Large program
# Modes and gains: the one source is 0.Large program
# --------------------------------------------------------------------------
# 2026-09-16 用户定：真车刷的是 sourcecode/0.Large program 的 hex，**所有模式
# 和参数集都以它为准**。之前按 04.bluetooth_control、05.weight_control 等 7
# 个单功能例程建的增益集，以及本项目的 load_normal / load_heavy，已从这里
# 移除（负载档的增益仍在 load_sched.py）。
#
# 核对方式（程序化，不是人工抄）：
#   - 以标准库 Keil 版 1.standard libraries(keil)/stm32_Balance_Car_L 为准，
#     它的 hex 最新（2025-01-10）。HAL Keil 版逐项一致；HAL IDE 版只是
#     app_mode.c 里少处理了 Diff_Line_track 一个模式。
#   - 源码的 19 个浮点常数有 17 个原样出现在 Keil hex 的字面量池里。缺的
#     6000 和 1.35 只作为 RW 初值出现，Keil 压缩 RW 初值；gcc 编的 HAL IDE
#     hex 里两个都在。对照组（app_mode.c 里被注释掉的 32.5）三份 hex 都没有。
#
# 固件怎么定参数（开机拧轮子选模式、按键确认后各执行一次）：
#   pid_control.c 初值      9600/75/6000/30/1400/30（Set_PID 不管的模式就用它）
#   app_mode.c Set_PID()    只重设下面 5 组
#   app_mode.c Set_Mid_Angle() / Set_angle() / Set_control_speed()
#   pid_control.c           mode == Weight_M 时 balance*2.0 / velocity*1.35 /
#                           turn*1.0（孪生把这三个系数折进增益）
#   app_motor.c Turn_Off()  angle < -40 || angle > angle_max  —— 上下不对称
#
# 死区补偿：hex 里 MOTOR_IGNORE_PULSE 是 1300；本项目 baseline 是用户真车改成
# 的 1500（robot.py pwm_deadband_comp）。模式里不带补偿值，全部跑在 1500 上。
#
# 孪生只模拟遥控那条指令通路。巡线 / CCD / 电磁 / K210 / 雷达这些模式的转向
# 和速度由传感器写入，孪生没有那些传感器；选这些模式得到的是它们的平衡环和
# 速度环参数，外加遥控指令幅值 0（固件的 Set_control_speed 不给它们设值）。
_LP = "0.Large program/1.standard libraries(keil)"

# (Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd)，固件 x100 单位
_RAW_INIT = (9600, 75, 6000, 30, 1400, 30)       # pid_control.c:15-26 初值
_RAW_NORMAL = (9600, 48, 6200, 31, 1700, 20)     # Set_PID: Normal / PS2_Control / LiDar_Patrol
_RAW_WEIGHT = (9600, 75, 7000, 35, 1400, 20)     # Set_PID: Weight_M / K210_Follow
_RAW_ELE = (9900, 72, 7000, 35, 2500, 20)        # Set_PID: ElE_Mode
_RAW_K210_LINE = (12000, 72, 8000, 40, 2500, 20)  # Set_PID: K210_Line
_RAW_LIDAR_LINE = (10200, 75, 9000, 45, 2500, 20)  # Set_PID: LiDar_Line / LiDar_wall_Line
_LOAD_K = (2.0, 1.35, 1.0)                        # pid_control.c:4-6 Balance_K/Velocity_K/Turn_K

# 这几个键是说明，不是增益，pid_gains() 会去掉
_MODE_META = ("source", "label", "mode_index", "turn_loop")


def _mode(index, enum, cn, raw, mid, angle_max, speed=0.0, turn_amp=0.0,
          turn_loop="Turn_PD", load_k=(1.0, 1.0, 1.0)):
    bk, vk, tk = load_k
    return dict(
        balance_kp=raw[0] / 100.0 * bk, balance_kd=raw[1] / 100.0 * bk,
        velocity_kp=raw[2] / 100.0 * vk, velocity_ki=raw[3] / 100.0 * vk,
        turn_kp=raw[4] / 100.0 * tk, turn_kd=raw[5] / 100.0 * tk,
        mid_angle_deg=float(mid),            # Set_Mid_Angle()
        angle_max_deg=float(angle_max),      # Set_angle()，Turn_Off 的前倾上限
        car_target_velocity=float(speed),    # Set_control_speed()
        car_turn_amplitude=float(turn_amp),
        mode_index=index,                    # OLED 上的编号 = 枚举值 + 1
        label=f"{index}.{cn}",
        turn_loop=turn_loop,
        source=f"{_LP} · mode {enum}")


# 顺序 = myenum.h 里 Car_mode_t 的顺序（开机拧轮子时的切换顺序）
PID_GAIN_SETS = {m["source"].rsplit(" ", 1)[1]: m for m in (
    _mode(1, "Normal", "标准模式（蓝牙遥控）", _RAW_NORMAL, 0, 40, 30, 36),
    _mode(2, "U_Follow", "超声波跟随", _RAW_INIT, 0, 40),
    _mode(3, "U_Avoid", "超声波避障", _RAW_INIT, 0, 40),
    _mode(4, "Weight_M", "负重模式", _RAW_WEIGHT, 0, 40, 30, 36, load_k=_LOAD_K),
    _mode(5, "PS2_Control", "PS2 手柄", _RAW_NORMAL, 0, 40, 30, 48),
    _mode(6, "Line_Track", "四路巡线", _RAW_INIT, 0, 16, turn_loop="Turn_IRTrack_PD"),
    _mode(7, "Diff_Line_track", "高难度四路巡线", _RAW_INIT, 0, 16,
          turn_loop="Turn_IRTrack_PD"),
    _mode(8, "K210_QR", "K210 二维码", _RAW_INIT, -1, 30),
    _mode(9, "K210_Line", "K210 巡线", _RAW_K210_LINE, -1, 30, turn_loop="Turn_K210_PD"),
    _mode(10, "K210_Follow", "K210 颜色跟随", _RAW_WEIGHT, -1, 30),
    _mode(11, "K210_SelfLearn", "K210 自主学习", _RAW_INIT, -1, 30),
    _mode(12, "K210_mnist", "K210 数字识别", _RAW_INIT, -1, 30),
    _mode(13, "LiDar_avoid", "雷达避障", _RAW_INIT, 0, 40),
    _mode(14, "LiDar_Follow", "雷达跟随", _RAW_INIT, 0, 40),
    _mode(15, "LiDar_aralm", "雷达警卫", _RAW_INIT, 0, 40),
    _mode(16, "LiDar_Patrol", "雷达巡逻", _RAW_NORMAL, 0, 40),
    _mode(17, "LiDar_Line", "雷达巡墙直线", _RAW_LIDAR_LINE, 0, 40),
    _mode(18, "LiDar_wall_Line", "雷达沿墙", _RAW_LIDAR_LINE, 0, 40),
    _mode(19, "CCD_Mode", "CCD 巡线", _RAW_INIT, 1, 25, turn_loop="Turn_CCD_PD"),
    _mode(20, "ElE_Mode", "电磁巡线", _RAW_ELE, -4, 16, turn_loop="Turn_ELE_PD"),
)}

# 真车用的「1 档」= OLED 上的 1.Standard Mode
PID_DEFAULT_SET = "Normal"


def pid_mode(name: str = PID_DEFAULT_SET) -> dict:
    """One Large program mode with its description fields (label, source...)."""
    return dict(PID_GAIN_SETS[name])


def pid_gains(name: str = PID_DEFAULT_SET) -> dict:
    """One Large program mode's parameters, in the twin's units (/100 folded in).

    The description fields are stripped so the result drops straight into
    ``STM32_CascadePID(gains=...)``.
    """
    g = pid_mode(name)
    for k in _MODE_META:
        g.pop(k, None)
    g.setdefault("integral_limit", 8000.0)
    g.setdefault("encoder_lpf", 0.84)
    return g


# 默认那一套 = Normal 模式。/100 已折算：固件把增益存成 x100 的数「方便调」，
# 用的时候再除以 100。
# The default = Large program Normal mode, with the firmware's /100 folded in.
PID_GAINS = pid_gains(PID_DEFAULT_SET)


# --------------------------------------------------------------------------
# app_control.c 在原地旋转时抬高偏航增益、在超声波模式下把它们清零。所以
# K5/K6 不是编译期常数，而是驾驶状态的函数——这一点很重要，因为
# TWIN_BASELINE.md 4.1 指出偏航通道正是固件最严重的 bug 所在。（在 10^6
# 标度 bug 还在的时候这四组是等价的；只有开了 ``fix_yaw_scale=True`` 才会
# 开始有区别。）
#
# app_control.c raises the yaw gains for spin-in-place and zeroes them for the
# ultrasonic modes.  K5/K6 are therefore not constants of the build, they are
# functions of the drive state -- which matters because TWIN_BASELINE.md 4.1 shows the
# yaw path is where the firmware's worst bug lives.  (With the 10^6 scaling bug
# in place all four of these are equivalent; they only start to differ once
# ``fix_yaw_scale=True``.)
LQR_YAW_GAINS_BY_MODE = {
    "run": (15.8114, 15.8114),        # 站立默认值 / K5OLD / K6OLD, the standing default
    "spin": (22.3607, 22.3607),       # 原地旋转 / enTLEFT / enTRIGHT
    "follow": (0.0, 0.0),             # 超声波跟随 / enFollow -- ultrasonic follow
    "avoid": (0.0, 0.0),              # 超声波避障 / enAvoid
}

# 出厂遥控实际发出的指令，用固件自己的单位。孪生的 UI 把摇杆和 WASD
# 映射到这些值上，而不是自己编一个范围——所以仿真里的「满前进」就是
# 真车上的满前进。
#
# What the shipped remote actually commands, in the firmware's own units.
# The twin's UI maps a joystick or WASD onto these rather than inventing a
# range, so "full forward" in the simulator is full forward on the real car.
FIRMWARE_COMMANDS = dict(
    # 6.LQR/STM32_code/APP/app_control.c —— 注意 Flag_velocity = 2 会把后退和
    # 转向减半，但*不*影响前进，前进是直接写死的 0.5f。
    # note Flag_velocity = 2 halves reverse and turn but NOT forward, which is
    # written as a bare 0.5f.
    lqr_forward_ms=0.5,
    lqr_reverse_ms=-0.6 / 2.0,
    lqr_turn_rad_s=4.0 / 2.0,
    lqr_flag_velocity=2.0,
    # 4.Balanced_Car_base/04.bluetooth_control/APP/PID/pid_control.c
    pid_target_velocity=25.0,         # 编码器计数/拍 / Car_Target_Velocity, counts/tick
    pid_turn_amplitude=30.0,          # 转向幅度 / Car_Turn_Amplitude_speed
    pid_spin_amplitude=50.0,          # 原地旋转幅度 / enTLEFT / enTRIGHT
)

# MPU6050 的标度，来自 app_control.c 里的 Get_Angle()：
#   gyro_x = Gyro_X / 939.8   -> 每 rad/s 对应 939.8 个原始 LSB（+-2000 deg/s 量程）
#   accel  = Accel  / 1671.84 -> +-2 g 量程
#
# MPU6050 scaling, from Get_Angle() in app_control.c:
#   gyro_x = Gyro_X / 939.8   -> 939.8 raw LSB per rad/s  (+-2000 deg/s range)
#   accel  = Accel  / 1671.84 -> +-2 g range
GYRO_LSB_PER_RAD_S = 939.8


# --------------------------------------------------------------------------
@dataclass
class FirmwareState:
    """Everything the firmware carries between 5 ms ticks."""
    x_pose: float = 0.0
    last_angle: float = 0.0
    angle_z: float = 0.0
    x_speed: float = 0.0
    gyro_x: float = 0.0
    gyro_z: float = 0.0
    # PID project
    encoder_bias: float = 0.0
    encoder_integral: float = 0.0
    # outputs, kept for telemetry
    ccr_l: int = 0
    ccr_r: int = 0
    motors_off: bool = False

    def reset(self):
        for f in ("x_pose", "last_angle", "angle_z", "x_speed", "gyro_x",
                  "gyro_z", "encoder_bias", "encoder_integral"):
            setattr(self, f, 0.0)
        self.ccr_l = self.ccr_r = 0
        self.motors_off = False


@dataclass
class FirmwareCommand:
    """What the remote / autonomy layer asks for, in the firmware's units."""
    v_ref: float = 0.0          # m/s   (LQR: Target_x_speed)
    yaw_ref: float = 0.0        # rad/s (LQR: Target_gyro_z)
    pid_move: float = 0.0       # PID project: Movement, encoder counts/tick
    pid_turn: float = 0.0       # PID project: Turn_Target


# --------------------------------------------------------------------------
class _FirmwareBase:
    """Shared plumbing: safety cut-out and PWM output stage."""

    def __init__(self, const=FC, fix_yaw_scale=False):
        self.c = const
        self.fix_yaw_scale = fix_yaw_scale
        self.deadband_comp = const.pwm_deadband_comp
        self.st = FirmwareState()

    def reset(self):
        self.st.reset()

    # ------------------------------------------------------------------
    def _deadband(self, p: int, side: int = 0) -> int:
        """app_motor.c PWM_Ignore()：非零输出都加上 1300 计数的死区补偿。

        于是最小的非零指令就是 1300/2880 = 45% 占空比——不存在「轻轻推一下」，
        电机几乎从不关闭，只能左右交替地踹。这是真实固件行为，也是静止晃动的来源。
        The stock dead-band compensation; the smallest non-zero command is
        45 % duty, which is why the motors are essentially never off.
        """
        if p > 0:
            return p + self.deadband_comp
        if p < 0:
            return p - self.deadband_comp
        return 0

    # ------------------------------------------------------------------
    def turn_off(self, angle_deg: float, battery_v: float, stop_flag: bool) -> bool:
        """app_motor.c Turn_Off(): the real failure condition of this car."""
        return bool(abs(angle_deg) > self.c.fail_angle_deg
                    or battery_v < self.c.battery_cutoff_v
                    or stop_flag)

    # ------------------------------------------------------------------
    @property
    def counts_per_rev(self) -> float:
        """LQR 环自己算速度用的线数——固件写的 1560，真值是 1320，所以它把
        速度读小 15.4%。这是照抄的固件 bug，不是笔误。

        【2026-09-17 删掉了 fix_encoder_cpr 开关】它只改这一个数，而 2026-09-10
        实测修不修对可持续速度毫无影响（PID 都卡 0.80、LQR 都到 1.15 m/s），
        串级 PID 那一路根本不用这个换算。真板子有这个 bug，孪生就照着有。
        The LQR loop's own counts/rev: the firmware says 1560 where the true
        value is 1320, so it under-reads speed by 15.4%.  Reproduced on
        purpose; the opt-out switch was removed once it was shown to change
        nothing measurable.
        """
        return self.c.lqr_counts_per_rev


# --------------------------------------------------------------------------
class STM32_LQR(_FirmwareBase):
    """6.LQR: full state feedback, 200 Hz, in EXTI15_10_IRQHandler."""

    def __init__(self, gains: dict | None = None, **kw):
        super().__init__(**kw)
        g = dict(LQR_GAINS)
        g.update(gains or {})
        self.K = np.array([g["K1"], g["K2"], g["K3"],
                           g["K4"], g["K5"], g["K6"]], dtype=float)
        self.target_angle = self.c.target_angle_rad

    # ------------------------------------------------------------------
    def step(self, enc_l: int, enc_r: int, angle_deg: float,
             cmd: FirmwareCommand, battery_v: float = 12.0,
             stop_flag: bool = False):
        """One 5 ms tick.  Encoder counts are integers, as on the board."""
        c = self.c
        st = self.st
        f = c.control_hz
        K1, K2, K3, K4, K5, K6 = self.K

        # ---- speed and odometry (app_control.c) --------------------------
        # x_speed = (encL+encR)/2 * PI*Diameter/1000 / cpr * f
        m_per_count = (np.pi * c.diameter_mm / 1000.0) / self.counts_per_rev
        st.x_speed = (enc_l + enc_r) / 2.0 * m_per_count * f
        st.x_pose += st.x_speed / f

        # ---- attitude ---------------------------------------------------
        angle_x = angle_deg / 180.0 * np.pi
        # FIRMWARE QUIRK: the pitch *rate* is a one-step difference of the
        # filtered angle, not the gyro.  It inherits the filter's lag and
        # multiplies its noise by f = 200.
        st.gyro_x = (angle_x - st.last_angle) * f
        st.last_angle = angle_x

        # ---- yaw --------------------------------------------------------
        if self.fix_yaw_scale:
            yaw_scale = m_per_count / (c.wheel_spacing_mm / 1000.0) * f
        else:
            # FIRMWARE BUG: /Wheel_spacing/1000 instead of /(Wheel_spacing/1000)
            yaw_scale = m_per_count / c.wheel_spacing_mm / 1000.0 * f
        st.gyro_z = (enc_r - enc_l) * yaw_scale
        st.angle_z += st.gyro_z / f

        # ---- command dispatch (app_control.c lines 46-110) ---------------
        # The shipped code zeroes the matching integrator on every tick that
        # a command is held: x_pose under enRUN/enBACK (and the ultrasonic
        # modes), angle_z under the four turn states.  Without this the
        # position and heading terms pull against the command and the car
        # cannot hold a steady speed or a steady turn.
        if cmd.v_ref != 0.0:
            st.x_pose = 0.0
        if cmd.yaw_ref != 0.0:
            st.angle_z = 0.0

        # ---- state feedback ---------------------------------------------
        common = (K1 * st.x_pose
                  + K2 * (st.x_speed - cmd.v_ref)
                  + K3 * (angle_x - self.target_angle)
                  + K4 * st.gyro_x)
        differential = K5 * st.angle_z + K6 * (st.gyro_z - cmd.yaw_ref)
        l_accel = -(common + differential)
        r_accel = -(common - differential)

        # ---- acceleration -> speed -> PWM --------------------------------
        v_l = int(c.ratio_accel * (st.x_speed + l_accel / f))
        v_r = int(c.ratio_accel * (st.x_speed + r_accel / f))

        # LQR project order: clamp, THEN add the dead band (so the compare
        # value can reach 3900 against a 2880 period -> pinned at 100 % duty)
        ccr_l = self._deadband(int(np.clip(v_l, -c.pwm_limit, c.pwm_limit)), 0)
        ccr_r = self._deadband(int(np.clip(v_r, -c.pwm_limit, c.pwm_limit)), 1)

        st.motors_off = self.turn_off(angle_deg, battery_v, stop_flag)
        if st.motors_off:
            ccr_l = ccr_r = 0
        st.ccr_l, st.ccr_r = ccr_l, ccr_r
        return ccr_l, ccr_r



# --------------------------------------------------------------------------
class STM32_CascadePID(_FirmwareBase):
    """0.Large program: Balance_PD + Velocity_PI + Turn_PD, summed."""

    def __init__(self, gains: dict | None = None, **kw):
        super().__init__(**kw)
        g = dict(PID_GAINS)
        g.update(gains or {})
        self.g = g
        # 负载辨识用的抖振，单位是 Motor 的 counts。每拍由外部写入，加在
        # **死区补偿之前**——固件里也只能加在这儿（Set_Pwm 之前一行），否则
        # 那 1300 的补偿盖不住抖振，小幅抖振根本出不了电机死区。
        # Load-ID dither in Motor counts, written per tick from outside and
        # added BEFORE the dead-band compensation -- the only place it can go,
        # or the +1300 would not cover it and a small dither never moves.
        self.dither = 0.0

    # ------------------------------------------------------------------
    def turn_off(self, angle_deg: float, battery_v: float, stop_flag: bool) -> bool:
        """0.Large program app_motor.c Turn_Off()::

            if(angle<-40||angle>angle_max || battery<9.6 || Stop_Flag==1)

        后仰固定 -40，前倾上限随模式（Set_angle()：巡线/电磁 16、CCD 25、
        K210 30、其余 40）。Backward limit is a literal -40; the forward limit
        is per mode."""
        up = self.g.get("angle_max_deg", self.c.fail_angle_deg)
        return bool(angle_deg < -self.c.fail_angle_deg or angle_deg > up
                    or battery_v < self.c.battery_cutoff_v or stop_flag)

    # ------------------------------------------------------------------
    def step(self, enc_l: int, enc_r: int, angle_deg: float,
             cmd: FirmwareCommand, gyro_pitch: float = 0.0,
             gyro_yaw: float = 0.0, battery_v: float = 12.0,
             stop_flag: bool = False, moving: bool = False):
        """One 5 ms tick.

        ``gyro_pitch`` / ``gyro_yaw`` are in RAW MPU6050 LSB, because that is
        what the firmware feeds its gains: ``Gyro_Balance = -Gyro_X`` is the
        unscaled register value, so ``Balance_Kd`` is "PWM counts per LSB"
        (0.48 LSB^-1 = 7.9 counts per deg/s).  Passing rad/s here would be
        off by a factor of 940.
        """
        c = self.c
        st = self.st
        g = self.g

        # ---- Balance_PD --------------------------------------------------
        angle_bias = g["mid_angle_deg"] - angle_deg
        gyro_bias = -gyro_pitch
        balance = -g["balance_kp"] * angle_bias - gyro_bias * g["balance_kd"]

        # ---- Velocity_PI -------------------------------------------------
        # NB the sign: "speed feedback is positive feedback" (firmware comment)
        encoder_least = -(enc_l + enc_r)
        st.encoder_bias *= g["encoder_lpf"]
        st.encoder_bias += encoder_least * (1.0 - g["encoder_lpf"])
        st.encoder_integral += st.encoder_bias
        st.encoder_integral += cmd.pid_move
        st.encoder_integral = float(np.clip(st.encoder_integral,
                                            -g["integral_limit"],
                                            g["integral_limit"]))
        velocity = (-st.encoder_bias * g["velocity_kp"]
                    - st.encoder_integral * g["velocity_ki"])

        # ---- Turn_PD -----------------------------------------------------
        kd = g["turn_kd"] if moving else 0.0     # firmware: Kd=0 unless driving
        turn = cmd.pid_turn * g["turn_kp"] + gyro_yaw * kd

        # ---- mix ---------------------------------------------------------
        motor_l = balance + velocity + turn + self.dither
        motor_r = balance + velocity - turn + self.dither

        # PID project order: dead band FIRST, then clamp (so 2600 is a hard
        # ceiling -> 90.3 % duty, unlike the LQR project)
        ccr_l = int(np.clip(self._deadband(int(motor_l), 0),
                            -c.pwm_limit, c.pwm_limit))
        ccr_r = int(np.clip(self._deadband(int(motor_r), 1),
                            -c.pwm_limit, c.pwm_limit))

        st.motors_off = self.turn_off(angle_deg, battery_v, stop_flag)
        if st.motors_off:
            ccr_l = ccr_r = 0
            st.encoder_integral = 0.0          # firmware clears it when off
        st.ccr_l, st.ccr_r = ccr_l, ccr_r
        return ccr_l, ccr_r


# --------------------------------------------------------------------------
class EncoderQuantizer:
    """True wheel rotation -> integer counts per tick, as TIM3/TIM4 see it.

    A fractional remainder is carried across ticks; without it the truncation
    would bias the measured speed low by up to half a count every tick, which
    at 200 Hz is a systematic error of several cm/s.
    """

    def __init__(self, counts_per_rev: float = FC.true_counts_per_rev):
        self.cpr = counts_per_rev
        self.residual = 0.0

    def reset(self):
        self.residual = 0.0

    def __call__(self, d_wheel_angle: float) -> int:
        exact = d_wheel_angle / (2.0 * np.pi) * self.cpr + self.residual
        counts = int(exact)              # C truncates toward zero
        self.residual = exact - counts
        return counts


class IdealEncoder(EncoderQuantizer):
    """理想编码器：返回精确的（非整数）计数增量，没有量化。
    Exact, unquantised counts per tick -- for the ideal twin only."""

    def __call__(self, d_wheel_angle: float) -> float:
        return float(d_wheel_angle / (2.0 * np.pi) * self.cpr)


# --------------------------------------------------------------------------
# 混合：LQR 为主，PID 为辅 —— **超出出厂固件**，两个混合权重默认 0，
# 也就是逐位等同于原厂 LQR。
# Hybrid: LQR primary with PID auxiliaries.  BEYOND the shipped firmware; both
# blend weights default to 0, which is the stock LQR bit for bit.
# --------------------------------------------------------------------------
class STM32_HybridLQR(STM32_LQR):
    """原厂 LQR，外加两个可调的 PID 辅助项。

    **为什么是这两个辅助项**，实测数据（scripts/bench_stop_transient.py，
    无扰动 8 种子）：

        减速停车 0.35 m/s   俯仰过零次数   结果
          原厂 PID              ——        8 局全摔
          原厂 LQR              51        全过
        静止前后摇晃          俯仰摆幅    频率
          原厂 PID             0.442°    8.32 Hz
          原厂 LQR             4.264°    2.59 Hz

    LQR 在停车瞬态上明显更好（全状态反馈，K 矩阵里有速度到俯仰的交叉项），
    但静止摇晃是 PID 的十倍，而且是低频大幅。

    根因在 ``STM32_LQR.step`` 里那一行::

        st.gyro_x = (angle_x - st.last_angle) * f      # f = 200

    LQR 的俯仰角速度**不是读陀螺**，是把卡尔曼滤波后的角度做一阶差分。而那个
    滤波器时间常数 3 秒（见 KF.c），差分出来的「角速度」既滞后又把量化噪声
    放大 200 倍。阻尼项 K4*gyro_x 建立在一个很差的估计上，低频摇晃由此而来。
    PID 版用的是真陀螺，所以它静止时只摇 0.44 度。

    所以辅助项一是**阻尼来源的混合**：把差分角速度和真陀螺按 w_gyro 混合。
    辅助项二是 PID 的速度环 PI 输出按 w_pid_vel 叠加——LQR 的 K1*x_pose 是
    位置反馈，遇到恒定外力会有稳态偏差，PID 的积分项正好补这一块。

    两个权重都是 0 时，本类的输出和 STM32_LQR 完全一致（有单元测试锁定）。

    Both auxiliaries are zero by default, making this bit-identical to stock
    LQR.  w_gyro blends the real gyro into the damping term, which stock LQR
    derives by differencing a 3-second-filtered angle at 200 Hz; w_pid_vel adds
    the PID velocity-PI output, supplying the integral action LQR lacks.
    """

    def __init__(self, gains: dict | None = None, **kw):
        super().__init__(gains=gains, **kw)
        g = dict(gains or {})
        self.w_gyro = float(g.get("w_gyro", 0.0))
        self.w_pid_vel = float(g.get("w_pid_vel", 0.0))
        # 辅助速度环用的 PID 增益，取原厂那一套
        self.pg = dict(PID_GAINS)
        for k in ("velocity_kp", "velocity_ki", "encoder_lpf",
                  "integral_limit"):
            if k in g:
                self.pg[k] = g[k]

    # ------------------------------------------------------------------
    def step(self, enc_l: int, enc_r: int, angle_deg: float,
             cmd: FirmwareCommand, gyro_pitch: float = 0.0,
             gyro_yaw: float = 0.0, battery_v: float = 12.0,
             stop_flag: bool = False, moving: bool = False):
        c = self.c
        st = self.st
        f = c.control_hz
        K1, K2, K3, K4, K5, K6 = self.K

        m_per_count = (np.pi * c.diameter_mm / 1000.0) / self.counts_per_rev
        st.x_speed = (enc_l + enc_r) / 2.0 * m_per_count * f
        st.x_pose += st.x_speed / f

        angle_x = angle_deg / 180.0 * np.pi
        st.gyro_x = (angle_x - st.last_angle) * f
        st.last_angle = angle_x

        # ---- 辅助 1：阻尼来源混合 / damping source blend -----------------
        # gyro_pitch 是 MPU6050 原始 LSB，和 PID 版拿到的是同一个量。
        rate_meas = float(gyro_pitch) / GYRO_LSB_PER_RAD_S
        rate_used = (1.0 - self.w_gyro) * st.gyro_x + self.w_gyro * rate_meas

        if self.fix_yaw_scale:
            yaw_scale = m_per_count / (c.wheel_spacing_mm / 1000.0) * f
        else:
            yaw_scale = m_per_count / c.wheel_spacing_mm / 1000.0 * f
        st.gyro_z = (enc_r - enc_l) * yaw_scale
        st.angle_z += st.gyro_z / f

        if cmd.v_ref != 0.0:
            st.x_pose = 0.0
        if cmd.yaw_ref != 0.0:
            st.angle_z = 0.0

        common = (K1 * st.x_pose
                  + K2 * (st.x_speed - cmd.v_ref)
                  + K3 * (angle_x - self.target_angle)
                  + K4 * rate_used)
        differential = K5 * st.angle_z + K6 * (st.gyro_z - cmd.yaw_ref)
        l_accel = -(common + differential)
        r_accel = -(common - differential)

        v_l = int(c.ratio_accel * (st.x_speed + l_accel / f))
        v_r = int(c.ratio_accel * (st.x_speed + r_accel / f))

        # ---- 辅助 2：PID 速度环 PI / PID velocity-PI assist ---------------
        # 只有权重非零时才推进积分器，否则关掉辅助项还会留下状态。
        # Only advance the integrator when the assist is on, so switching it
        # off leaves no stale state behind.
        if self.w_pid_vel != 0.0:
            pg = self.pg
            encoder_least = -(enc_l + enc_r)
            st.encoder_bias *= pg["encoder_lpf"]
            st.encoder_bias += encoder_least * (1.0 - pg["encoder_lpf"])
            st.encoder_integral += st.encoder_bias
            st.encoder_integral += cmd.pid_move
            st.encoder_integral = float(np.clip(st.encoder_integral,
                                                -pg["integral_limit"],
                                                pg["integral_limit"]))
            velocity = (-st.encoder_bias * pg["velocity_kp"]
                        - st.encoder_integral * pg["velocity_ki"])
            v_l += int(self.w_pid_vel * velocity)
            v_r += int(self.w_pid_vel * velocity)

        # 输出级保持 LQR 工程的顺序：先钳位，再加死区
        # LQR project order preserved: clamp, THEN dead band
        ccr_l = self._deadband(int(np.clip(v_l, -c.pwm_limit, c.pwm_limit)), 0)
        ccr_r = self._deadband(int(np.clip(v_r, -c.pwm_limit, c.pwm_limit)), 1)

        st.motors_off = self.turn_off(angle_deg, battery_v, stop_flag)
        if st.motors_off:
            ccr_l = ccr_r = 0
        st.ccr_l, st.ccr_r = ccr_l, ccr_r
        return ccr_l, ccr_r


HYBRID_GAINS = dict(LQR_GAINS)
HYBRID_GAINS.update(w_gyro=0.0, w_pid_vel=0.0)
