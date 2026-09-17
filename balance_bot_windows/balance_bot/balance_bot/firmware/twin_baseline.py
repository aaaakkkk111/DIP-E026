"""孪生本体：固件控制器 + 电机模型驱动仿真器。
The twin itself: firmware controller + motor model driving the simulator.

``STM32Twin`` 是解析被控对象版本（快，用于基准测试），``STM32TwinMujoco`` 是
同一套东西跑在 MuJoCo 物理上。两者复用项目里同一份扰动注入、场地、奖励和
回合逻辑，所以固件基线和 RL 策略是用完全同一把尺子量出来的。

关于机械零点要专门说一句，因为它直接决定孪生看起来正不正常。固件是把*测量到*
的角度平衡到一个非零设定点上——LQR 版是 0.0349 rad，PID 版是 1 度——因为真车
上 IMU 并没有装得完全水平。而在仿真里真实的平衡点是 0，所以如果把真实角度喂给
控制器，它就会一直保持一个倾角然后开走。因此孪生把安装误差显式建模了：传感器
报告的是 ``true_angle + mount_offset``，而 ``mount_offset`` 默认就取固件自己的
设定点——正是这一点让真车能站直。想看一台标定不准的底盘要付多少代价，就故意把
这两个值设得不一样。

``STM32Twin`` is the analytic-plant version (fast, for benchmarking) and
``STM32TwinMujoco`` is the same thing on MuJoCo physics.  Both reuse the
project's disturbance injection, arena, reward and episode logic, so a
firmware baseline and an RL policy are measured by the identical yardstick.

A note on the mechanical zero, because it decides whether the twin looks
sane at all.  The firmware balances the *measured* angle to a non-zero
setpoint -- 0.0349 rad in the LQR build, 1 deg in the PID build -- because
the IMU is not mounted exactly level on the real chassis.  In simulation the
true balance point is 0, so feeding the controller the true angle would make
it hold a permanent lean and drive away.  The twin therefore models the
mounting error explicitly: the sensor reports ``true_angle + mount_offset``,
and ``mount_offset`` defaults to the firmware's own setpoint, which is what
makes the real car stand up straight.  Set them apart deliberately if you
want to see what a mis-calibrated chassis costs.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from ..disturb import DisturbanceModel
from ..dynamics import IX, IY, IPSI, IPSID, IS, IV, ITH, ITHD
from ..env import BalanceCore, activity_gate
from ..params import DisturbanceConfig
from . import imu as imu_mod
from . import motor as motor_mod
from .controllers import (STM32_LQR, STM32_CascadePID, STM32_HybridLQR,
                          FirmwareCommand,
                          EncoderQuantizer, IdealEncoder, GYRO_LSB_PER_RAD_S, PID_GAINS)
from .motor import IdealMotor, MotorCalibration, Yahboom370Motor
from .robot import STM32_CAR, STM32_FIRMWARE_CONST as FC, stm32_sim_params

MODE_STM32_LQR = "stm32_lqr"
MODE_STM32_PID = "stm32_pid"
# LQR 为主 + PID 辅助。**超出出厂固件**：两个混合权重为 0 时逐位等同原厂 LQR，
# 见 controllers.STM32_HybridLQR 的注释和 test_hybrid_is_lqr_at_zero。
# Hybrid, beyond the shipped firmware; identical to stock LQR at zero weights.
MODE_STM32_HYBRID = "stm32_hybrid"

# 回路总延迟，单位是 200 Hz 的固件拍（一拍 5 ms）。
#
# 物理下界就是 1：固件在中断里读 IMU、算完、写比较寄存器，新值下一个 PWM
# 周期才输出。**这个值曾经被设成 0**，注释里标着「非物理、掩盖一个没找到的
# 结构性缺陷」——2026-09-13 真车实测证明那个代价是实打实的：在零延迟被控对象
# 上搜出来的增益（balance_kp x2.31 / balance_kd x0.69）上车后高频剧烈振荡。
#
# 延迟从 0 改到 1，同一套增益的逐拍 |dCCR| 从 6 涨到 3275（546 倍），
# 俯仰频率跳到 8.3 Hz。**优化器会精确地钻进模型里每一个不物理的简化**：
# 没有延迟时 D 项纯粹是负担，所以它把 D 砍掉换 P。
#
# The physical floor is one tick: the CCR written in this interrupt takes
# effect on the next PWM period.  This was set to 0 and documented as
# non-physical; on 2026-09-13 that cost a full training run and a real-car
# test -- gains searched against a delay-free plant chattered at 8.3 Hz on the
# board.  An optimiser will exploit every unphysical simplification.
#
# 【2026-09-16 真车回放拟合】2.03 拍（约 10 ms）。拟合时放开到 0-10 拍（0-50 ms），
# 数据选的是 2 拍：扫频相位 3.5->18 Hz 那 -271° 里有很大一部分是 12.5 Hz 传动
# 共振贡献的，不是纯延迟；把纯延迟加到 20 ms 以上，静止和扫频全都对不上。
# Replay fit, free over 0-50 ms, chose ~10 ms.
PWM_DELAY_TICKS = 1.7538      # 模式 1 静止基准拟合（第一次回放拟合是 2.03）

# IMU 安装零偏（度），真车静止倾角均值 +0.31°，回放拟合 0.28。
# IMU mounting error, from the real car's standstill mean tilt.
IMU_MOUNT_ERROR_DEG = 0.1660

# 陀螺本底噪声（rad/s，每个控制拍高斯）。无扰动配置下也存在——真车的传感器
# 从来不是干净的。回放拟合值，约 0.05 °/s。
# Intrinsic gyro noise, present even with a zero DisturbanceConfig.
GYRO_NOISE_RAD_S = 0.000607

# 陀螺零偏（rad/s）。真车模式 1 静止三次实测 +1.98 / +2.04 / +2.05 °/s，车并没有
# 持续转动（否则 30 秒会累积 60°），所以是 MPU6050 自身的零偏，非常稳定。
# 卡尔曼靠加速度计把它从角度里修掉，但固件的 D 项直接用陀螺原始值：
#   0.48 x 2.02 °/s x 16.4 LSB/(°/s) ≈ 16 计数的恒定偏移叠加在 PID 输出上。
# Measured MPU6050 gyro bias, identical across three recordings.
GYRO_BIAS_RAD_S = float(np.radians(2.02))

# 陀螺高频振动（rad/s）：白噪声的一阶差分，直流附近几乎没有能量，代表电机和
# 齿轮工作时的机械振动。和普通白噪声的区别：不会被卡尔曼积分成角度漂移、不会
# 推着位置环游走，但会让 D 项在相邻两拍之间跳，把 PID 输出在零点附近来回拨——
# 真车模式 1 静止时 32% 的换向间隔只有 1-2 拍，40-100 Hz 陀螺能量也比孪生多。
# High-frequency gyro vibration (first-differenced white noise, no DC content).
GYRO_VIB_RAD_S = 0.01621      # 模式 1 静止基准拟合；0 = 关
# 振动里随轮速变化的那部分（rad/s 每 轮端 rad/s）：齿轮啮合振动只在轮子转的时候
# 才有。0 = 振动幅值恒定。
GYRO_VIB_MOTION = 0.01265

# 轮胎滚动阻力（m，MuJoCo 滚动摩擦系数：阻力矩 = 系数 x 法向力）。MJCF 里写了
# 滚动摩擦 0.0001，但接触维度 condim 默认 3，滚动项根本不生效——轮子等于在没有
# 任何阻力的地面上纯滚动。> 0 时把轮子和地面改成 condim 6 并用这个系数。
# 0 = 保持原样。和减速箱摩擦不同：它阻碍轮子相对**地面**滚动，不牵连车身俯仰。
# Tyre rolling resistance; the MJCF's value was inert because condim was 3.
ROLLING_RESIST_M = 0.000802      # 模式 1 静止基准拟合（第九轮）
# 转子 Stribeck 摩擦（轮端等效 N·m）：静止时要克服 ROTOR_FRIC_STATIC 才能起转，
# 转起来以后按 exp(-(w/vs)^2) 降到 ROTOR_FRIC_KINETIC。真车模式 1 静止录波里
# 看得到：编码器停住、PWM 一路爬到 ±70 才突然脱开，一脚尖峰后再振一下
# （8.3 Hz 二次谐波的来源）。0 = 关。
# Rotor Stribeck friction (wheel-equivalent N·m); 0 disables it.
ROTOR_FRIC_STATIC = 0.00271
ROTOR_FRIC_KINETIC = 0.00090
ROTOR_STRIBECK_VEL = 0.6552         # rad/s，轮端

# 弹性传动积分时把 MuJoCo 子步再细分几份：轮子本体惯量只有 2e-5，接上
# 11 N·m/rad 的扭簧，1.25 ms 的步长会数值发散（实测扭转角飞到 1.4 rad）。
GEAR_SUBSTEPS = 5

# --------------------------------------------------------------------------
# 站定外环 —— 这是**超出出厂固件**的一个添加，默认关闭
# Station-hold outer loop -- an addition BEYOND the shipped firmware, off by
# default so every existing baseline number is untouched.
# --------------------------------------------------------------------------
# 为什么必须加：出厂的两套控制器都没有航向状态可用。
#   PID:  turn = pid_turn*turn_kp + gyro_yaw*turn_kd
#         指令为 0 时只剩阻尼项，缩放这两个增益能改变转多快，永远无法把车
#         **转回**原朝向。
#   LQR:  K5 作用于 st.angle_z，但 angle_z 是 10^6 标度 bug 下积出来的，恒等
#         于 0；打开 fix_yaw_scale 会让偏航跟踪整体崩掉（实测扫了 8 个 K5/K6
#         值，没有一个跟得上指令）。
# 所以「松手后朝向不变」不是调参能解决的，必须补一个作用在航向误差上的项。
# 它在真板子上是可实现的：angle_z 已经算出来了，修掉标度再加三行 C 即可。
#
# Neither shipped controller has a usable heading state, so "holds its heading
# after you let go" cannot be reached by scaling gains -- there is no gain that
# acts on heading error.  This loop adds one.  It is implementable on the real
# board: app_control.c already accumulates angle_z; fix its scale and add three
# lines.
HOLD_V_MAX = 0.12        # m/s，位置保持能自己下的最大速度指令
HOLD_YAW_MAX = 1.2       # rad/s，航向保持能自己下的最大偏航指令
HOLD_DEADBAND_M = 0.01   # m，位置死区，免得在原地来回蹭
HOLD_DEADBAND_RAD = 0.017  # rad (1 度)，航向死区
# --------------------------------------------------------------------------
# 原厂蓝牙遥控：**离散状态**，不是连续速度
# The factory Bluetooth control: DISCRETE states, not a continuous velocity
# --------------------------------------------------------------------------
# 0.Large program 的 BSP/Bluetooth/app_bluetooth.c 把收到的字节解成这几个状态
# （和 04.bluetooth_control 同一套）：
#     run_car -> enRUN    back_car -> enBACK   left_car -> enLEFT
#     right_car -> enRIGHT   stop_car -> enSTOP
# 另有两个「原地转」状态 enTLEFT / enTRIGHT。
#
# pid_control.c 把状态映射成两个固定幅值，**和按键按多深无关**：
#     Velocity_PI:  enRUN  -> Movement = +Car_Target_Velocity（Normal 30）
#                   enBACK -> Movement = -Car_Target_Velocity
#                   其余   -> Movement = Move_X（遥控不用，恒 0）
#     Turn_PD:      enLEFT  -> Turn_Target = -Car_Turn_Amplitude_speed（Normal 36）
#                   enRIGHT -> Turn_Target = +Car_Turn_Amplitude_speed
#                   enTLEFT/enTRIGHT -> -+50
#                   其余    -> 0
#     Turn_PD 的 Kd 只在 enRUN/enBACK 下取 Turn_Kd，其余状态取 0。
#
# 三个后果，孪生原来的连续指令路径都不具备：
#   1) 幅值是固定的。摇杆推到底和轻推给出同一个 Movement。
#   2) **左/右转不设 Movement**，所以原厂的左右转是在零前进指令下转的；
#      要边走边转只能靠 enRUN 期间的偏航阻尼项，没有「同时给前进和转向」。
#   3) 没有斜坡。指令是阶跃的，唯一的平滑来自速度环自己的积分。
#
# Five discrete states with fixed magnitudes; the command steps, it does not
# ramp, and left/right leaves Movement at zero.
CAR_STOP = "stop"
CAR_RUN = "run"
CAR_BACK = "back"
CAR_LEFT = "left"
CAR_RIGHT = "right"
CAR_TLEFT = "tleft"        # 原地左转 / spin left
CAR_TRIGHT = "tright"      # 原地右转 / spin right
CAR_STATES = (CAR_STOP, CAR_RUN, CAR_BACK, CAR_LEFT, CAR_RIGHT,
              CAR_TLEFT, CAR_TRIGHT)

# 0.Large program Normal 模式（app_mode.c Set_control_speed()）。其它模式的幅值
# 在 PID_GAIN_SETS 里（car_target_velocity / car_turn_amplitude），运行时按
# 当前模式取，见 STM32TwinMixin._car_state_cmd()。旧值 25/30 来自
# 04.bluetooth_control，已不是 baseline。
CAR_TARGET_VELOCITY = 30.0      # app_mode.c Set_control_speed: Normal -> 30
CAR_TURN_AMPLITUDE = 36.0       # app_mode.c Set_control_speed: Normal -> 36
CAR_SPIN_AMPLITUDE = 50.0       # pid_control.c Turn_PD: enTLEFT/enTRIGHT -> -+50

# 状态 -> (Movement, Turn_Target, Kd 是否启用)
CAR_STATE_CMD = {
    CAR_STOP:   (0.0, 0.0, False),
    CAR_RUN:    (+CAR_TARGET_VELOCITY, 0.0, True),
    CAR_BACK:   (-CAR_TARGET_VELOCITY, 0.0, True),
    CAR_LEFT:   (0.0, -CAR_TURN_AMPLITUDE, False),
    CAR_RIGHT:  (0.0, +CAR_TURN_AMPLITUDE, False),
    CAR_TLEFT:  (0.0, -CAR_SPIN_AMPLITUDE, False),
    CAR_TRIGHT: (0.0, +CAR_SPIN_AMPLITUDE, False),
}

FIRMWARE_MODES = (MODE_STM32_LQR, MODE_STM32_PID)


def sample_stm32_disturbance(rng, difficulty: float = 1.0) -> DisturbanceConfig:
    """按*这台*车缩放的扰动阶梯，不是通用机器人那套。
    Disturbance ladder scaled to *this* car, not the generic robot.

    ``disturb.py`` 里的通用配置是按 1.7 kg、0.6 N·m 轮子的机器人配的。这台车
    只有 0.942 kg：峰值轮端推力 2*tau_max/r = 23.9 N，其中 2*tau_damp/r = 9.0 N
    在任何东西动起来之前就被阻尼吃掉了，净剩约 14.9 N 的控制权限，而整车重量
    只有 9.2 N。所以 20 N 的推搡不叫「难」，它是算术意义上救不回来；而一个满是
    不可救回合的基准，什么也没测到。

    下面这个阶梯的上限卡在执行器能力附近——那正是控制器好坏真正显出来的地方。

    The generic profile in ``disturb.py`` was sized for a 1.7 kg robot with
    0.6 N m wheels.  This car weighs 0.942 kg: peak wheel force is
    2*tau_max/r = 23.9 N, of which 2*tau_damp/r = 9.0 N is eaten by damping
    before anything moves, leaving roughly 14.9 N of net authority against
    9.2 N of weight.  A 20 N shove is therefore not "hard", it is arithmetically
    unrecoverable, and a benchmark full of unrecoverable episodes measures
    nothing.

    The ladder below tops out near the actuator's authority, which is where
    controller quality actually shows up.
    """
    d = float(np.clip(difficulty, 0.0, 1.0))
    return DisturbanceConfig(
        noise_pitch=rng.uniform(0.0, 0.010) * d,
        noise_pitch_rate=rng.uniform(0.0, 0.12) * d,
        noise_vel=rng.uniform(0.0, 0.03) * d,
        noise_yaw_rate=rng.uniform(0.0, 0.06) * d,
        imu_bias_walk=rng.uniform(0.0, 0.004) * d,
        torque_noise=rng.uniform(0.0, 0.05) * d,
        torque_scale_err=rng.uniform(0.0, 0.12) * d,
        latency_steps=int(rng.integers(0, 1 + int(3 * d))),
        wind_force=rng.uniform(0.0, 0.5) * d,
        wind_dir=rng.uniform(-np.pi, np.pi),
        ground_slip=rng.uniform(0.0, 0.25) * d,
        random_impulse_hz=rng.uniform(0.0, 0.8) * d,
        random_impulse_max=rng.uniform(2.0, 10.0) * d,
    )


class STM32TwinMixin:
    """把串级 PID 的控制拍替换成固件自己的那一拍。
    Replaces the cascade-PID control tick with the firmware's own."""

    def _init_firmware(self, firmware=MODE_STM32_LQR,
                       motor_cal: MotorCalibration | None = None,
                       imu_filter: str = "kalman",
                       imu_warm_start: bool = True,
                       mount_offset_deg: float | None = None,
                       fix_yaw_scale: bool = False,
                       hold_station: bool = False,
                       hold_kpos: float = 1.5,
                       hold_kpsi: float = 4.0,
                       battery_v: float = 12.0,
                       gains: dict | None = None,
                       ideal: bool = False):
        self.firmware_name = firmware
        # ---- 理想模式 / ideal mode ------------------------------------------
        # 同一个 MuJoCo 车体、同一套固件 PID 公式，把所有非理想因素关掉：
        #   电机  力矩 ∝ 补偿以上的 PWM（IdealMotor），无反电动势/阻尼/饱和
        #   信号  无 PWM 延迟、无 PWM 限幅、无驱动器电流限
        #   传动  刚性（无弹性、无间隙），转子惯量挂关节 armature
        #   传感  角度 / 角速度取真值（无卡尔曼、DLPF、量化、噪声、安装零偏），
        #         编码器不截断
        # 和 scripts/ideal_pid.py 的线性模型是同一个理想化，只是被控对象用 MuJoCo。
        # Same body and firmware PID, every non-ideal effect switched off.
        self.ideal = bool(ideal)
        self.pwm_delay_ticks = 0.0 if self.ideal else PWM_DELAY_TICKS
        self.gyro_noise = 0.0 if self.ideal else GYRO_NOISE_RAD_S
        self.gyro_bias_rad_s = 0.0 if self.ideal else GYRO_BIAS_RAD_S
        self.gyro_vib = 0.0 if self.ideal else GYRO_VIB_RAD_S
        self._vib_prev = 0.0
        if self.ideal:
            imu_filter = "ideal"
        fw_const = (dataclasses.replace(FC, pwm_limit=10 ** 9) if self.ideal else FC)
        # 站定外环 / station-hold outer loop (beyond the firmware -- see above)
        self.hold_station = bool(hold_station)
        self.hold_kpos = float(hold_kpos)
        self.hold_kpsi = float(hold_kpsi)
        self._anchor = None
        # 原厂蓝牙那条离散指令通路；None = 用连续的 set_command
        # The factory discrete-state path; None means use set_command
        self.car_state = None
        # 逐拍钩子，签名 hook(twin, tick_index)；见 step()
        self.tick_hook = None
        self._tick_n = 0
        self._enc_last = (0, 0)
        # 自动档位切换。None = 关掉（默认），走固定增益。
        # 打开的话每个固件拍喂一次陀螺，检测器决定用 NORMAL 还是 HEAVY。
        # Automatic load-mode switching; None disables it (the default).
        self.load_detect = None
        self.load_mode = None
        # 堵转力矩和空载转速都必须从 RobotParams 传进电机标定。这两个数以前是
        # 各写各的：RobotParams.tau_max 只是动力学的力矩钳位，而真正产生力矩的
        # 是 MotorCalibration.stall_torque。改了前者不改后者，车的实际出力纹丝
        # 不动——我就这样误报过一次「力矩已改成 0.60」，实际跑的还是 0.20。
        # stall_torque must follow RobotParams.tau_max: the former is what the
        # motor actually produces, the latter only clamps it.  Editing one
        # without the other silently changes nothing.
        # 【2026-09-16】堵转力矩不再等于 tau_max：电机能出 0.57，驱动器截在 0.40。
        # 两者由真车回放分别拟合，见 motor.py MotorCalibration。
        if self.ideal:
            self.motor = IdealMotor(comp=FC.pwm_deadband_comp)
        else:
            self.motor = Yahboom370Motor(motor_cal or MotorCalibration(
                r_wheel=self.robot_nominal.r_wheel,
                omega_noload=self.robot_nominal.wheel_speed_max))
        self.battery_v = battery_v

        if firmware == MODE_STM32_LQR:
            self.fw = STM32_LQR(gains=gains, fix_yaw_scale=fix_yaw_scale,
                                const=fw_const)
            default_offset = np.rad2deg(FC.target_angle_rad)
        elif firmware == MODE_STM32_HYBRID:
            self.fw = STM32_HybridLQR(gains=gains,
                                      fix_yaw_scale=fix_yaw_scale,
                                      const=fw_const)
            default_offset = np.rad2deg(FC.target_angle_rad)
        elif firmware == MODE_STM32_PID:
            self.fw = STM32_CascadePID(gains=gains,
                                       fix_yaw_scale=fix_yaw_scale,
                                        const=fw_const)
            # 固件的机械中值 + 真车 IMU 实际装歪的那 0.28°
            default_offset = (gains or {}).get(
                "mid_angle_deg", PID_GAINS["mid_angle_deg"]) + (
                    0.0 if self.ideal else IMU_MOUNT_ERROR_DEG)
        else:
            raise ValueError(f"unknown firmware {firmware!r}")

        self.mount_offset_deg = (default_offset if mount_offset_deg is None
                                 else float(mount_offset_deg))

        # 控制器看到的角度来自 KF.c 里的 KF_X()——一个两状态卡尔曼滤波器
        # （角度 + 陀螺零偏），由 ``main.c`` 用 GET_Angle_Way = 2 选中。现在
        # 已经精确复现；它的 Q/R 调参到底换来了什么见 imu.py（3 秒时间常数，
        # 也就是一个近乎纯粹的陀螺积分器，**不是**孪生以前假设的 10 ms 滞后）。
        #
        # The angle the controller sees comes out of KF_X() in KF.c -- a
        # two-state Kalman filter (angle + gyro bias) that ``main.c`` selects
        # with GET_Angle_Way = 2.  It is now reproduced exactly; see imu.py
        # for what its Q/R tuning actually buys (a 3 s time constant, i.e. a
        # near-pure gyro integrator, NOT the 10 ms lag the twin used to
        # assume).
        self.imu_filter = str(imu_filter)
        # 固件 app_control.c 的 Get_Angle(way)：1=DMP 2=卡尔曼 3=互补。
        # main.c 里 GET_Angle_Way = 2，所以 "kalman" 是出厂默认。
        # "dmp" 是**近似**：芯片里那段是二进制 blob，只能照行为近似。
        # Get_Angle(way): 1=DMP 2=Kalman 3=complementary; the board ships 2.
        if self.imu_filter == "kalman":
            self.imu = imu_mod.MPU6050Kalman(ts=1.0 / FC.control_hz,
                                             warm_start=imu_warm_start)
        elif self.imu_filter == "complementary":
            self.imu = imu_mod.MPU6050Complementary(dt=1.0 / FC.control_hz)
        elif self.imu_filter == "dmp":
            self.imu = imu_mod.MPU6050DMP(dt=1.0 / FC.control_hz)
        elif self.imu_filter == "ideal":
            self.imu = None                  # 角度和角速度直接取真值
        else:
            raise ValueError(f"unknown imu_filter {imu_filter!r}")
        self._dlpf_af, self._dlpf_au, self._dlpf_g = imu_mod.make_dlpf()
        self._angle_filt = None
        self._gyro_filt = 0.0
        self._gyro_raw = 0
        self._gyro_lsb = 0.0
        self._yaw_lsb = 0.0
        self._sub_thd = 0.0
        self._sub_v = 0.0


        self._ccr_pipe = [(0.0, 0.0)] * (int(np.ceil(self.pwm_delay_ticks)) + 1)

        enc_cls = IdealEncoder if self.ideal else EncoderQuantizer
        self.enc_l = enc_cls(FC.true_counts_per_rev)
        self.enc_r = enc_cls(FC.true_counts_per_rev)
        self._last_wheel = (0.0, 0.0)
        self.fw_ccr = (0, 0)
        self.fw_off = False
        # MuJoCo 版要按模式重设模型（armature / 执行器范围 / 步长）
        hook = getattr(self, "_apply_model_mode", None)
        if hook is not None:
            hook()

    # ------------------------------------------------------------------
    def _wheel_angles(self):
        """由行程和航向算出每个轮子的角度（弧度）。
        Per-wheel angle (rad) from travel and heading."""
        st = self.state
        half = 0.5 * self.robot.track
        r = self.robot.r_wheel
        left = (st[IS] - half * st[IPSI]) / r
        right = (st[IS] + half * st[IPSI]) / r
        return left, right

    def _wheel_speeds(self):
        """每个电机实际感受到的转子转速，rad/s。
        Rotor speed each motor actually sees, rad/s.

        解析被控对象没有轮子这个自由度，只能从底盘反推：
        ``omega = (v -+ track/2 * yaw_rate) / r``，这悄悄假设了轮子纯滚动不打滑。
        地面抓地时这是对的，也无害。可一旦地面变滑就完全不成立了，而且后果一点
        不微妙——电机模型的反电动势是从这个转速推出来的，于是一个实际以 80 rad/s
        空转、而车只以 0.2 m/s 蠕行的轮子，会被告知自己在跑 6 rad/s，从而继续
        输出接近堵转的力矩。它随后无限加速，接触求解器开始抖。MuJoCo 孪生用
        轮子关节自身的速度覆盖掉这个函数。

        The analytic plant has no wheel degree of freedom, so it has to infer
        this from the chassis: ``omega = (v -+ track/2 * yaw_rate) / r``, which
        silently assumes the wheels are rolling without slipping.  On a grippy
        floor that is true and harmless.  It is very much not true once the
        floor gets slippery, and the consequence is not subtle -- the motor
        model derives its back-EMF from this speed, so a wheel that is really
        spinning at 80 rad/s while the car creeps along at 0.2 m/s gets told
        it is doing 6 rad/s and keeps delivering near-stall torque.  It then
        spins up without limit and the contact solver chatters.  The MuJoCo
        twin overrides this with the wheel joints' own velocities.
        """
        st = self.state
        p = self.robot
        return ((st[IV] - 0.5 * p.track * st[IPSID]) / p.r_wheel,
                (st[IV] + 0.5 * p.track * st[IPSID]) / p.r_wheel)

    # ------------------------------------------------------------------
    # Get_Angle()：5 ms 一拍里属于 MPU6050 的那一半
    # Get_Angle(): the MPU6050 half of the 5 ms tick
    # ------------------------------------------------------------------
    def _imu_raw(self, dt):
        """一个 DLPF 之前的采样：(前向加速度, 上向加速度, 陀螺)，SI 单位，车体系。
        One pre-DLPF sample: (accel_fwd, accel_up, gyro), SI, body frame.

        加速度计量的是 IMU 处的比力，而 IMU 装在轮轴上方 ``l_com`` 处，所以它
        会被车自身的修正加速度污染——代数推导见 imu.accel_pitch_angle。
        MuJoCo 孪生会覆盖这个函数，改成直接读模型自己的传感器。

        The accelerometer measures specific force at the IMU, which sits
        ``l_com`` above the axle, so it is contaminated by the car own
        correction accelerations -- see imu.accel_pitch_angle for the algebra.
        The MuJoCo twin overrides this to read the model own sensors.
        """
        st = self.state
        thd = float(st[ITHD])
        v = float(st[IV])
        thdd = (thd - self._sub_thd) / dt
        v_dot = (v - self._sub_v) / dt
        self._sub_thd, self._sub_v = thd, v
        th = float(st[ITH])
        sn, cs = np.sin(th), np.cos(th)
        l = self.robot.l_com
        g = 9.81
        return (v_dot * cs + l * thdd - g * sn,
                v_dot * sn - l * thd ** 2 + g * cs,
                thd)

    def _on_substep(self, dt):
        """按积分步长跑片上 DLPF，而不是按控制回路的节拍。
        Run the chip DLPF at the integration rate, not the loop rate."""
        # 三种真实滤波（DMP / 卡尔曼 / 互补）都读同一颗 MPU6050，片上 DLPF
        # 在传感器里，和用哪种融合算法无关。只有 "lag"/"none" 那两条遗留通路
        # 绕开它。
        # 这里以前写 != "kalman"，加进互补和 DMP 之后它们拿到的加速度角和
        # 陀螺全是 0——传感器是死的，车两步就摔。
        # All three real filters read the same MPU6050; the DLPF lives in the
        # sensor, not in the fusion.  This used to test != "kalman", which fed
        # the two new filters nothing but zeros.
        if self.imu_filter not in ("kalman", "complementary", "dmp"):
            return
        a_f, a_u, gyro = self._imu_raw(dt)
        self._dlpf_af.update(a_f, dt)
        self._dlpf_au.update(a_u, dt)
        self._dlpf_g.update(gyro, dt)

    def _read_imu(self, meas, st, dt):
        """更新 ``Angle_Balance``（度）和 ``Gyro_Balance``（原始 LSB）。
        Update ``Angle_Balance`` (deg) and ``Gyro_Balance`` (raw LSB).

        安装误差是整个传感器坐标系绕俯仰轴的一次旋转，所以它应该直接加在
        加速度计报告的倾角上，而不是加在滤波器的输出上——对停着的车来说两者
        一样，对正在加速的车来说前者才是对的。

        The mounting error is a rotation of the whole sensor frame about the
        pitch axis, so it adds straight onto the tilt the accelerometer
        reports rather than onto the filter output -- the same thing for a
        parked car, the right thing for an accelerating one.
        """
        off = self.mount_offset_deg

        if self.imu_filter == "ideal":
            rate = float(st[ITHD])
            angle_deg = float(np.rad2deg(float(st[ITH]))) + off
            self._gyro_raw = rate * GYRO_LSB_PER_RAD_S        # 不量化
            self._gyro_lsb = float(self._gyro_raw)
            self._yaw_lsb = float(st[IPSID]) * GYRO_LSB_PER_RAD_S
            self._gyro_filt = rate
            self._angle_filt = angle_deg
            return angle_deg, self._gyro_raw

        # DisturbanceModel 把 ``imu_bias_walk`` 加在角度上，因为它没有滤波器
        # 可以把它喂进去。但陀螺零偏本来就该加在角速度上——KF_X 的第二个状态
        # 存在的意义正是估计它——所以在这里把它挪过去。
        #
        # DisturbanceModel puts ``imu_bias_walk`` on the angle because it has
        # no filter to feed it through.  A gyro bias belongs on the rate --
        # that is what the second state of KF_X exists to estimate -- so move
        # it across here.
        bias = self.dist.gyro_bias
        # 扰动模型加进来的所有东西（噪声，以及它通过缓冲实现的传输延迟）都是
        # 以「相对真值的差」的形式带过来的，这样 DLPF 的输出仍然是干净信号，
        # 而配置好的传感器故障含义也丝毫不变。
        #
        # Everything the disturbance model adds (noise, and the transport
        # delay it applies by buffering) is carried across as a difference
        # against the truth, so the DLPF output stays the clean signal and the
        # configured sensor faults keep meaning exactly what they meant.
        d_rate = float(meas.theta_dot) - float(st[ITHD])
        noise = getattr(self, "gyro_noise", GYRO_NOISE_RAD_S)
        if noise > 0.0:
            d_rate += float(self.dist.rng.normal(0.0, noise))
        d_rate += getattr(self, "gyro_bias_rad_s", GYRO_BIAS_RAD_S)
        vib = getattr(self, "gyro_vib", GYRO_VIB_RAD_S)
        vm = 0.0 if getattr(self, "ideal", False) else float(GYRO_VIB_MOTION)
        if vm > 0.0:
            try:
                wl, wr = self._wheel_speeds()
                vib = vib + vm * 0.5 * (abs(wl) + abs(wr))
            except Exception:
                pass
        if vib > 0.0:
            w = float(self.dist.rng.normal())
            d_rate += vib * (w - getattr(self, "_vib_prev", 0.0)) / np.sqrt(2.0)
            self._vib_prev = w
        d_angle = float(meas.theta) - bias - float(st[ITH])

        # 片上 DLPF -> 陀螺量化 -> 加速度计算倾角，三种滤波共用这一条
        # 传感器通路，差别只在融合那一步。
        # One sensor path (DLPF, gyro quantisation, accel-derived angle);
        # the three filters differ only in the fusion step.
        gyro_rate = self._dlpf_g.value + d_rate + bias
        self._gyro_raw = imu_mod.quantize_gyro(gyro_rate)
        gyro_q = imu_mod.dequantize_gyro(self._gyro_raw)
        accel_angle = imu_mod.accel_pitch_angle_from_sensor(
            self._dlpf_af.value, self._dlpf_au.value) + d_angle
        angle_deg = np.rad2deg(self.imu.update(accel_angle, gyro_q)) + off
        # Gyro_Balance 是原始寄存器值，所以 PID 版拿到的是整数——而且
        # 一并拿到产生它的那个片上 DLPF。
        # Gyro_Balance is the raw register, so the PID build gets the
        # integer -- and it gets the chip DLPF that produced it.
        self._gyro_lsb = float(self._gyro_raw)
        self._yaw_lsb = float(imu_mod.quantize_gyro(meas.yaw_rate))

        self._gyro_filt = gyro_q
        self._angle_filt = angle_deg
        return angle_deg, self._gyro_raw

    # ------------------------------------------------------------------
    def reset(self, *args, **kwargs):
        obs = super().reset(*args, **kwargs)
        if self.randomize and not self._fixed_disturbance:
            # 用按这台车配的扰动阶梯替换掉通用配置
            # replace the generic profile with the one sized for this car
            self.dist.set_config(
                sample_stm32_disturbance(self.rng, self.ep_difficulty))
            self.dist.reset()
        self.fw.reset()
        if hasattr(self.fw, "dither"):
            self.fw.dither = 0.0
        if self.load_detect is not None:
            self.enable_load_detect(True)
        self._tick_n = 0
        self.motor.reset()
        self._ccr_pipe = [(0.0, 0.0)] * (int(np.ceil(
            getattr(self, "pwm_delay_ticks", PWM_DELAY_TICKS))) + 1)
        self.enc_l.reset()
        self.enc_r.reset()
        self._last_wheel = self._wheel_angles()
        self._sub_thd = float(self.state[ITHD])
        self._sub_v = float(self.state[IV])
        for f in (self._dlpf_af, self._dlpf_au, self._dlpf_g):
            f.reset()
        if self.imu is not None:
            self.imu.reset()
            if self.imu_filter == "kalman":
                # 真车上电时 P = 单位阵，静止那几秒里 KF 的第二个状态已经把陀螺
                # 零偏学到位；之后 Q=1e-10 让增益趋零，零偏估计就冻结在那儿。
                # warm_start 给的是收敛后的 P，所以零偏估计也必须给收敛后的值
                # （= 传感器零偏），否则学不进去，角度估计一直偏，车追着错误的
                # 倾角跑——2026-09-16 实测游走 40 mm、12.5 Hz 振荡就是这个。
                # With a converged P the bias can no longer be learned, so seed
                # the bias the filter would have learned at power-up.
                self.imu.seed(float(self.state[ITH]),
                              bias=float(getattr(self, "gyro_bias_rad_s", GYRO_BIAS_RAD_S)))
            elif self.imu_filter in ("complementary", "dmp"):
                self.imu.seed(float(self.state[ITH]))
        self._angle_filt = None
        self._gyro_filt = 0.0
        self._gyro_raw = 0
        self._gyro_lsb = 0.0
        self._yaw_lsb = 0.0
        self.fw_ccr = (0, 0)
        self.fw_off = False
        self._anchor = None
        return obs

    # ------------------------------------------------------------------
    def _auto_load_mode(self):
        """把检测器的判决写进固件增益。每个固件拍一次。

        判决只在静止时更新：抖动统计量只有静止时可标定，行驶时驾驶动作会
        污染它（实测空车在行驶段会掉进「装了货」的区间）。载重只在有人停下
        装卸货时才变，所以行驶中冻结判决没有任何损失。
        """
        from .load_sched import NORMAL, HEAVY
        moving = self.car_state not in (None, CAR_STOP)
        heavy = self.load_detect.update(float(self._gyro_lsb), moving=moving)
        mode = HEAVY if heavy else NORMAL
        if mode is not self.load_mode:
            self.load_mode = mode
            if hasattr(self.fw, "g"):
                self.fw.g.update(mode.gains())

    def enable_load_detect(self, on=True):
        """打开/关掉自动档位切换。打开时立刻按检测器的默认档写一次增益。"""
        from .load_detect import LoadDetector
        from .load_sched import NORMAL, HEAVY
        if not on:
            self.load_detect = self.load_mode = None
            return
        self.load_detect = LoadDetector(hz=float(FC.control_hz))
        self.load_detect.reset()
        self.load_mode = HEAVY if self.load_detect.heavy else NORMAL
        if hasattr(self.fw, "g"):
            self.fw.g.update(self.load_mode.gains())

    # ------------------------------------------------------------------
    def step(self, action=None):
        """一个智能体拍，由 ``ctrl_per_agent`` 个固件拍组成。
        One agent tick, made of ``ctrl_per_agent`` firmware ticks."""
        sim = self.sim
        rw = self.rw
        p = self.robot
        dt = sim.dt_ctrl

        acc_theta = acc_ev = acc_ey = acc_tau = acc_thd = 0.0
        obstacle_pen = 0.0
        n_ticks = sim.ctrl_per_agent

        for _ in range(n_ticks):
            # 逐拍钩子：负载辨识要在 200 Hz 上注入抖振并采样，智能体拍
            # （25 Hz）太粗，8 Hz 的抖振在那个采样率下直接混叠。
            # Per-tick hook: load ID injects and samples at 200 Hz; the 25 Hz
            # agent tick would alias an 8 Hz dither outright.
            if self.load_detect is not None:
                self._auto_load_mode()
            if self.tick_hook is not None:
                self.tick_hook(self, self._tick_n)
            self._tick_n += 1
            self._advance_command(dt)
            self.dist.maybe_random_impulse(dt)
            st = self.state

            # ---- 传感器 / sensors ---------------------------------------
            meas = self.dist.measure(st[ITH], st[ITHD], st[IV], st[IPSID], dt)
            self._read_imu(meas, st, dt)

            wl, wr = self._wheel_angles()
            enc_l = self.enc_l(wl - self._last_wheel[0])
            enc_r = self.enc_r(wr - self._last_wheel[1])
            self._last_wheel = (wl, wr)
            # 这一拍的编码器增量，给 tick_hook 用（板子上就是 Encoder_Least）
            self._enc_last = (enc_l, enc_r)

            # ---- 固件 / firmware ----------------------------------------
            v_ref, yaw_ref = self._station_hold()
            cmd = FirmwareCommand(v_ref=v_ref, yaw_ref=yaw_ref)
            if self.firmware_name == MODE_STM32_LQR:
                ccr_l, ccr_r = self.fw.step(
                    enc_l, enc_r, self._angle_filt, cmd,
                    battery_v=self.battery_v)
            elif self.firmware_name == MODE_STM32_HYBRID:
                # 混合版两样都要：LQR 的角度路径，加上 PID 那条原始陀螺
                # LSB 通道（辅助阻尼用的就是它），以及 PID 单位的速度指令
                # （辅助速度环用）。
                # The hybrid needs both paths: LQR's angle, plus the raw gyro
                # LSB channel the damping assist reads and the PID-unit move
                # command the velocity assist integrates.
                cmd.pid_move = v_ref * self._pid_move_scale()
                cmd.pid_turn = yaw_ref * self._pid_turn_scale()
                ccr_l, ccr_r = self.fw.step(
                    enc_l, enc_r, self._angle_filt, cmd,
                    gyro_pitch=self._gyro_lsb,
                    gyro_yaw=self._yaw_lsb,
                    battery_v=self.battery_v,
                    moving=abs(v_ref) > 1e-6)
            else:
                # PID 版要的是 MPU6050 的原始 LSB，不是 rad/s
                # the PID build wants raw MPU6050 LSB, not rad/s
                if self.car_state is not None:
                    # 原厂蓝牙通路：固定幅值，见 CAR_STATE_CMD
                    mv, tt, kd_on = self._car_state_cmd(self.car_state)
                    cmd.pid_move, cmd.pid_turn = mv, tt
                    pid_moving = kd_on
                else:
                    cmd.pid_move = v_ref * self._pid_move_scale()
                    cmd.pid_turn = yaw_ref * self._pid_turn_scale()
                    pid_moving = abs(v_ref) > 1e-6
                # Gyro_Balance / Gyro_Turn 是 MPU6050 的**原始**寄存器值——
                # PID 版从不对它们做滤波，所以这里也不做。
                # Gyro_Balance / Gyro_Turn are the RAW MPU6050 registers --
                # the PID build never filters them, so neither does this.
                ccr_l, ccr_r = self.fw.step(
                    enc_l, enc_r, self._angle_filt, cmd,
                    gyro_pitch=self._gyro_lsb,
                    gyro_yaw=self._yaw_lsb,
                    battery_v=self.battery_v,
                    moving=pid_moving)
            # 固件在 5 ms 中断里读 IMU、算完、写 PWM，**写进去的值下一拍才
            # 生效**。孪生原来是同一拍算、同一拍作用，物理上不可能。
            #
            # 这一拍不是小事：用户 2026-09-09 实测真车静止时摆幅 >=10 度、
            # 约 1 Hz，而零延迟的孪生给 0.44 度、8.3 Hz；补上这一拍之后是
            # 8.94 度、0.48 Hz，一步合上了大半差距。
            #
            # 延迟必须加在**输出侧**：DisturbanceConfig.latency_steps 延迟的
            # 是所有测量，会把编码器也一起拖慢，而编码器是定时器直接读的、
            # 不走 I2C，没有理由延迟。
            #
            # The board writes PWM that takes effect on the *next* tick.  This
            # must go on the output: delaying the measurements instead would
            # also drag the encoder path, which the timer reads directly.
            dly = self.pwm_delay_ticks
            if dly > 0.0:
                # 小数拍：延迟本来就不是整数。CCR 写进比较寄存器的时刻相对
                # PWM 周期是任意的，IMU 样本的年龄也取决于 I2C 读取相对控制拍
                # 的时序。整数拍给出的是分岔式的两个极端——1 拍是 10 Hz 的死区
                # 极限环、2 拍是 0.5 Hz 的慢摆，而真车的 1 Hz 落在中间。
                # Fractional ticks: the delay is not an integer.  Whole ticks
                # only offer the two extremes either side of the real car.
                self._ccr_pipe.append((float(ccr_l), float(ccr_r)))
                old_l, old_r = self._ccr_pipe.pop(0)
                new_l, new_r = self._ccr_pipe[0]
                f = dly - int(dly)
                ccr_l = int(round(f * old_l + (1.0 - f) * new_l))
                ccr_r = int(round(f * old_r + (1.0 - f) * new_r))
            self.fw_ccr = (ccr_l, ccr_r)
            self.fw_off = self.fw.st.motors_off

            # ---- 电机模型 / motor model ---------------------------------
            omega_l, omega_r = self._wheel_speeds()
            tau_l = self.motor.torque(ccr_l, omega_l, self.battery_v,
                                      dt=dt, side=0)
            tau_r = self.motor.torque(ccr_r, omega_r, self.battery_v,
                                      dt=dt, side=1)
            tau_l, tau_r = self.dist.corrupt_torque(tau_l, tau_r, p.tau_max)

            wrench = self.dist.wrench(st[IPSI], dt, p.l_com)
            self._plant_step(tau_l, tau_r, wrench, self.dist.cfg.ground_slip)

            self.t += dt
            st = self.state
            acc_theta += st[ITH] ** 2
            acc_thd += st[ITHD] ** 2
            acc_ev += abs(self.v_ref - st[IV])
            acc_ey += abs(self.yaw_ref - st[IPSID])
            acc_tau += 0.5 * ((tau_l / p.tau_max) ** 2 + (tau_r / p.tau_max) ** 2)
            obstacle_pen = max(obstacle_pen,
                               self.arena.obstacle_penalty(st[IX], st[IY],
                                                           p.collision_radius))
            # 两条摔倒判据：固件自己的角度阈值，以及车身触地。
            # 只看角度是不够的——车身用真实外形之后（前后 151.6 mm，而轮子
            # 半径只有 33.5 mm），车会先托底被地面撑住，永远到不了 40 度，
            # 于是明明趴在地上却被算成「存活」。
            # Two fall criteria: the firmware's own angle cut-out, and the
            # chassis touching the ground.  With a real-sized body the car
            # bottoms out before it can ever reach 40 degrees.
            if abs(st[ITH]) > sim.pitch_fail or p.bottoms_out(float(st[ITH])):
                self.fell = True
                break
            if self.use_obstacles and self.arena.in_collision(
                    st[IX], st[IY], p.collision_radius):
                self.collided = True
                break

        k = max(1, n_ticks)
        reward = (rw.alive
                  + rw.upright * (1.0 - (acc_theta / k) / sim.pitch_fail ** 2)
                  - rw.vel_track * (acc_ev / k)
                  - rw.yaw_track * (acc_ey / k)
                  - rw.torque * (acc_tau / k)
                  - rw.pitch_rate * (acc_thd / k)
                  - rw.obstacle * obstacle_pen)
        terminated = False
        if self.fell:
            reward -= rw.fall_penalty
            terminated = True
        elif self.collided:
            reward -= rw.collision_penalty
            terminated = True

        self.step_count += 1
        truncated = self.step_count >= sim.max_agent_steps
        self.info = {
            "t": self.t, "fell": self.fell, "collided": self.collided,
            "pitch": float(self.state[ITH]), "v": float(self.state[IV]),
            "yaw_rate": float(self.state[IPSID]),
            "v_ref": self.v_ref, "yaw_ref": self.yaw_ref,
            "ccr": self.fw_ccr, "motors_off": self.fw_off,
            "obstacle_pen": obstacle_pen,
            "firmware": self.firmware_name,
        }
        return self._observe(), float(reward), terminated, truncated, self.info

    # ------------------------------------------------------------------
    def set_car_state(self, state):
        """按原厂蓝牙的方式下指令：一个**离散状态**，不是速度。

        ``state`` 取 ``CAR_STATES`` 之一；``None`` 退回连续的
        ``set_command(v, yaw)``（训练和基准要连续量，保持可用）。
        只对 PID 底座生效——LQR 工程的指令通路是另一套（Target_x_speed /
        Target_gyro_z），不是蓝牙那五个状态。

        Issue commands the way the factory Bluetooth build does: one discrete
        state.  ``None`` restores the continuous path.  PID base only.
        """
        if state is not None and state not in CAR_STATE_CMD:
            raise ValueError(f"unknown car state {state!r}; "
                             f"expected one of {CAR_STATES}")
        self.car_state = state

    def _station_hold(self):
        """把「保持位置和朝向」翻译成固件听得懂的速度/偏航指令。
        Translate "hold this spot and this heading" into the commands the
        firmware already understands.

        有指令时锚点跟着车走（等于不干预）；指令一撤，锚点冻结，外环开始把车
        往回拉。这和固件自己在 app_control.c 里逐拍清零 x_pose / angle_z 的
        做法是同一个思路，只是反过来用。

        While a command is held the anchor follows the car, so the loop does
        nothing.  The moment the command drops to zero the anchor freezes and
        the loop starts pulling the car back to it.
        """
        v_ref, yaw_ref = self.v_ref, self.yaw_ref
        if not self.hold_station:
            return v_ref, yaw_ref
        st = self.state
        pose = (float(st[IX]), float(st[IY]), float(st[IPSI]))
        if abs(v_ref) > 1e-6 or abs(yaw_ref) > 1e-6 or self._anchor is None:
            self._anchor = pose
            return v_ref, yaw_ref

        ax, ay, apsi = self._anchor
        # 位置误差投影到车头方向：差速车只能沿车头走，横向误差得先转过去
        # Project onto the heading -- a differential drive can only move along
        # its own nose, so lateral error has to be turned into heading error.
        c, sn = np.cos(pose[2]), np.sin(pose[2])
        e_fwd = (pose[0] - ax) * c + (pose[1] - ay) * sn
        d = pose[2] - apsi
        e_psi = float(np.arctan2(np.sin(d), np.cos(d)))

        if abs(e_fwd) > HOLD_DEADBAND_M:
            v_ref = float(np.clip(-self.hold_kpos * e_fwd,
                                  -HOLD_V_MAX, HOLD_V_MAX))
        if abs(e_psi) > HOLD_DEADBAND_RAD:
            yaw_ref = float(np.clip(self.hold_kpsi * e_psi,
                                    -HOLD_YAW_MAX, HOLD_YAW_MAX))
        return v_ref, yaw_ref

    def station_error(self):
        """(前向误差 m, 横向误差 m, 航向误差 rad) —— 给观测向量用。
        (forward, lateral, heading) error against the anchor, for the obs."""
        st = self.state
        if self._anchor is None:
            return 0.0, 0.0, 0.0
        ax, ay, apsi = self._anchor
        psi = float(st[IPSI])
        c, sn = np.cos(psi), np.sin(psi)
        dx, dy = float(st[IX]) - ax, float(st[IY]) - ay
        d = psi - apsi
        return (dx * c + dy * sn, -dx * sn + dy * c,
                float(np.arctan2(np.sin(d), np.cos(d))))

    # ------------------------------------------------------------------
    def _car_state_cmd(self, state):
        """离散遥控状态 -> (Movement, Turn_Target, Kd 是否启用)，幅值按当前模式。

        0.Large program 的 Set_control_speed() 按模式给 Car_Target_Velocity /
        Car_Turn_Amplitude_speed（Normal 30/36、PS2 30/48、其余不设即 0），
        原地转的 +-50 是 Turn_PD 里写死的。
        Amplitudes follow the selected mode; spin-in-place is a literal 50."""
        mv, tt, kd_on = CAR_STATE_CMD[state]
        g = getattr(self.fw, "g", None) or {}
        v = g.get("car_target_velocity", CAR_TARGET_VELOCITY)
        a = g.get("car_turn_amplitude", CAR_TURN_AMPLITUDE)
        if state in (CAR_RUN, CAR_BACK):
            mv = float(np.sign(mv)) * v
        elif state in (CAR_LEFT, CAR_RIGHT):
            tt = float(np.sign(tt)) * a
        return mv, tt, kd_on

    def _pid_move_scale(self) -> float:
        """m/s -> PID 版的 ``Movement``（每拍编码器计数）。
        m/s -> the PID build's ``Movement`` (encoder counts per tick).

        ``Car_Target_Velocity = 25`` 对应固件所理解的「前进」，而它喂进去的
        那个积分器累加的是 ``encoder_least = -(enc_l + enc_r)``——**两个轮子
        的和**，不是单轮，也不是平均。所以指令必须用同一个单位，1 m/s 等于
        ``2 * counts_per_rev / (2*pi*r) / f`` 个计数。

        少乘这个 2 会让指令恒定只兑现一半：实测指令 0.10/0.20/0.30/0.60 得到
        0.051/0.100/0.151/0.302，比值全量程锁在 0.50。表现出来就是「踩下去
        车跑不到指令速度」，而 LQR 底座没有这个问题（比值 0.97）。

        The integrator this feeds accumulates ``encoder_least =
        -(enc_l + enc_r)`` -- the SUM of both wheels, not one wheel and not
        their mean -- so the command has to be in the same unit: one m/s is
        ``2 * counts_per_rev / (2*pi*r) / f`` counts.  Dropping the 2 delivers
        exactly half of every command (measured ratio 0.50 across the range),
        which reads as "the car never reaches the commanded speed".
        """
        return (2.0 * FC.true_counts_per_rev
                / (2.0 * np.pi * self.robot.r_wheel) / FC.control_hz)

    def _pid_turn_scale(self) -> float:
        """rad/s -> PID 版的 ``Turn_Target``（约 30 个单位 = 满转）。
        rad/s -> the PID build's ``Turn_Target`` (~30 units = full turn)."""
        return 30.0 / 4.0        # 固件把 +-4 rad/s 对应到 +-30 个单位
                                 # firmware pairs +-4 rad/s with +-30 units


# --------------------------------------------------------------------------
class STM32Twin(STM32TwinMixin, BalanceCore):
    """跑在解析被控对象上的固件孪生。
    Firmware twin on the analytic plant."""

    def __init__(self, firmware=MODE_STM32_LQR, motor_cal=None,
                 imu_filter="kalman",
                 imu_warm_start=True, mount_offset_deg=None,
                 fix_yaw_scale=False,
                 hold_station=False, hold_kpos=1.5, hold_kpsi=4.0,
                 battery_v=12.0, episode_seconds=20.0, gains=None, ideal=False, **kw):
        kw.setdefault("robot", STM32_CAR)
        kw.setdefault("sim", stm32_sim_params(episode_seconds))
        kw.setdefault("randomize", False)
        kw.setdefault("obstacles", False)
        # 只要传了 DisturbanceConfig，就会置上 BalanceCore 的
        # _fixed_disturbance 标志，从而**关掉**逐局随机化——这对交互使用是
        # 对的（滑块归 UI 管），对基准测试则恰恰是错的。
        #
        # Passing a DisturbanceConfig at all sets BalanceCore's
        # _fixed_disturbance flag, which switches OFF per-episode
        # randomisation -- right for interactive use (the UI owns the
        # sliders), exactly wrong for benchmarking.
        if not kw["randomize"]:
            kw.setdefault("disturbance", DisturbanceConfig())
        self._fw_kw = dict(firmware=firmware, motor_cal=motor_cal,
                           imu_filter=imu_filter,
                           imu_warm_start=imu_warm_start,
                           mount_offset_deg=mount_offset_deg,
                           fix_yaw_scale=fix_yaw_scale,
                           hold_station=hold_station, hold_kpos=hold_kpos,
                           hold_kpsi=hold_kpsi,
                           battery_v=battery_v,
                           gains=gains, ideal=ideal)
        self._fw_ready = False
        super().__init__(mode=firmware, **kw)

    def reset(self, *args, **kwargs):
        if not self._fw_ready:
            # BalanceCore.__init__ 会在我们来得及初始化固件之前就调用
            # reset()，所以第一趟在这里补上。
            # BalanceCore.__init__ calls reset() before we can set the
            # firmware up, so do it on the first pass.
            obs = BalanceCore.reset(self, *args, **kwargs)
            self._init_firmware(**self._fw_kw)
            self._fw_ready = True
            self._last_wheel = self._wheel_angles()
            return obs
        return super().reset(*args, **kwargs)


def make_mujoco_twin(**kw):
    """同一个孪生，换成 MuJoCo 物理。延迟导入，让 mujoco 保持可选依赖。
    Same twin, MuJoCo physics.  Imported lazily so mujoco stays optional."""
    from ..backends.mujoco_backend import MujocoCore

    class STM32TwinMujoco(STM32TwinMixin, MujocoCore):
        def __init__(self, firmware=MODE_STM32_LQR, motor_cal=None,
                     imu_filter="kalman",
                     imu_warm_start=True, mount_offset_deg=None,
                     fix_yaw_scale=False,
                     hold_station=False, hold_kpos=1.5, hold_kpsi=4.0,
                     battery_v=12.0, episode_seconds=20.0, gains=None,
                     ideal=False, **kwargs):
            kwargs.setdefault("robot", STM32_CAR)
            kwargs.setdefault("sim", stm32_sim_params(episode_seconds))
            kwargs.setdefault("randomize", False)
            kwargs.setdefault("obstacles", False)
            if not kwargs["randomize"]:
                kwargs.setdefault("disturbance", DisturbanceConfig())
            self._imu_adr_cache = None
            self._fw_kw = dict(firmware=firmware, motor_cal=motor_cal,
                               imu_filter=imu_filter,
                               imu_warm_start=imu_warm_start,
                               mount_offset_deg=mount_offset_deg,
                               fix_yaw_scale=fix_yaw_scale,
                                   hold_station=hold_station,
                               hold_kpos=hold_kpos, hold_kpsi=hold_kpsi,
                               battery_v=battery_v,
                           gains=gains, ideal=ideal)
            self._fw_ready = False
            super().__init__(mode=firmware, **kwargs)

        # ---- 弹性传动 / drivetrain compliance -----------------------------
        # 【2026-09-16 真车回放拟合】转子 --(扭簧 k、阻尼 c、间隙 b)--> 轮子。
        # 证据全部来自真车录数：静止时车身在晃而编码器几乎不动；闭环扫频
        # 12.5 Hz 轻阻尼共振；Kd120 极限环锁在 14 Hz。刚性传动复现不了。
        # 转子惯量（RobotParams.I_rotor）从 MuJoCo 的关节 armature 挪到这里的
        # 转子自由度上；反电动势用转子转速，编码器读转子角（编码器在电机轴上）。
        # MotorCalibration.gear_stiffness = None 时整段不生效，回到刚性传动。
        # Series-elastic drivetrain; off when gear_stiffness is None.
        def _gear_cal(self):
            m = getattr(self, "motor", None)
            if m is not None:
                return m.cal
            return self._fw_kw.get("motor_cal") or MotorCalibration()

        def _ideal_on(self):
            if hasattr(self, "ideal"):
                return bool(self.ideal)
            return bool(self._fw_kw.get("ideal", False))

        def _gear_on(self):
            if self._ideal_on():
                return False                  # 理想模式：刚性传动
            k = self._gear_cal().gear_stiffness
            return k is not None and k > 0.0

        def _apply_model_mode(self):
            """按当前模式重设 MuJoCo 模型。模型是缓存的，GUI 在「真车孪生」和
            「理想模型」之间切换时不会重新编译，所以每次换固件都要重设一遍。
            Re-apply per-mode model settings; the compiled model is cached and
            survives controller switches in the GUI."""
            if getattr(self, "model", None) is None:
                return
            self._rotor = None
            fl = 0.0 if self._ideal_on() else float(getattr(self._gear_cal(), "gear_friction", 0.0))
            self.model.dof_frictionloss[6] = fl
            self.model.dof_frictionloss[7] = fl
            # 实例属性优先，GUI 的摩擦滑块改的就是它（不改全局，免得动了
            # 一个孪生就把同进程里别的孪生一起改了）。
            # Instance attribute wins; the GUI friction sliders set it.
            rr = 0.0 if self._ideal_on() else float(
                getattr(self, "rolling_resist_m", ROLLING_RESIST_M))
            mj = self._mj
            for name in ("gw_l", "gw_r", "floor"):
                gid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name)
                if gid < 0:
                    continue
                if rr > 0.0:
                    self.model.geom_condim[gid] = 6
                    self.model.geom_friction[gid, 2] = rr
                else:
                    self.model.geom_condim[gid] = 3
                    self.model.geom_friction[gid, 2] = 0.0001
            if self._gear_on():
                # 转子惯量由转子自由度承担；关节上不再重复算
                self.model.dof_armature[6] = 0.0
                self.model.dof_armature[7] = 0.0
                # 扭簧瞬时力矩可以超过驱动器电流限，电流限改为作用在电机力矩上
                self.model.actuator_ctrlrange[:, 0] = -50.0
                self.model.actuator_ctrlrange[:, 1] = 50.0
                self.model.opt.timestep = self.sim.dt_sub / GEAR_SUBSTEPS
            else:
                self.model.dof_armature[6] = self.robot.I_rotor
                self.model.dof_armature[7] = self.robot.I_rotor
                lim = 50.0 if self._ideal_on() else self.robot.tau_max
                self.model.actuator_ctrlrange[:, 0] = -lim
                self.model.actuator_ctrlrange[:, 1] = lim
                self.model.opt.timestep = self.sim.dt_sub

        def _build_model(self):
            super()._build_model()
            self._apply_model_mode()

        def _plant_reset(self, *args, **kw2):
            self._rotor = None
            return super()._plant_reset(*args, **kw2)

        def _rotor_state(self):
            if getattr(self, "_rotor", None) is None:
                self._rotor = (
                    np.array([self.data.qpos[7], self.data.qpos[8]], dtype=float),
                    np.array([self.data.qvel[6], self.data.qvel[7]], dtype=float))
            return self._rotor

        def _wheel_angles(self):
            if hasattr(self, "data") and len(self.data.qpos) > 8:
                if self._gear_on():
                    phi, _ = self._rotor_state()
                    return float(phi[0]), float(phi[1])
                return float(self.data.qpos[7]), float(self.data.qpos[8])
            return super()._wheel_angles()

        def _wheel_speeds(self):
            if hasattr(self, "data") and len(self.data.qvel) > 7:
                if self._gear_on():
                    _, wm = self._rotor_state()
                    return float(wm[0]), float(wm[1])
                return float(self.data.qvel[6]), float(self.data.qvel[7])
            return super()._wheel_speeds()

        def _plant_step(self, tau_l, tau_r, wrench, slip):
            if self._ideal_on():
                # 理想模式：没有驱动器电流限，力矩原样作用
                grip = float(np.clip(1.0 - slip, 0.0, 1.0))
                self.data.ctrl[0] = tau_l * grip
                self.data.ctrl[1] = tau_r * grip
                psi = self.state[IPSI]
                fx_w = wrench.fx * np.cos(psi) - wrench.fy * np.sin(psi)
                fy_w = wrench.fx * np.sin(psi) + wrench.fy * np.cos(psi)
                self.data.xfrc_applied[self._bid, :] = 0.0
                self.data.xfrc_applied[self._bid, 0] = fx_w
                self.data.xfrc_applied[self._bid, 1] = fy_w
                self.data.xfrc_applied[self._bid, 5] = wrench.yaw_moment
                for _ in range(self.sim.phys_per_ctrl):
                    self._mj.mj_step(self.model, self.data)
                    self._sync_state()
                    self._on_substep(self.sim.dt_sub)
                return
            if not self._gear_on():
                return super()._plant_step(tau_l, tau_r, wrench, slip)
            cal = self._gear_cal()
            k, c = float(cal.gear_stiffness), float(cal.gear_damping)
            bl = np.radians(float(cal.gear_backlash_deg))
            I_r = max(float(self.robot.I_rotor), 1e-7)
            mj = self._mj
            grip = float(np.clip(1.0 - slip, 0.0, 1.0))
            lim = self.robot.tau_max            # 驱动器电流限
            tm = np.clip(np.array([tau_l, tau_r], dtype=float) * grip, -lim, lim)
            psi = self.state[IPSI]
            fx_w = wrench.fx * np.cos(psi) - wrench.fy * np.sin(psi)
            fy_w = wrench.fx * np.sin(psi) + wrench.fy * np.cos(psi)
            self.data.xfrc_applied[self._bid, :] = 0.0
            self.data.xfrc_applied[self._bid, 0] = fx_w
            self.data.xfrc_applied[self._bid, 1] = fy_w
            self.data.xfrc_applied[self._bid, 5] = wrench.yaw_moment
            phi, wm = self._rotor_state()
            fs = float(getattr(self, "rotor_fric_static", ROTOR_FRIC_STATIC))
            fk = float(getattr(self, "rotor_fric_kinetic", ROTOR_FRIC_KINETIC))
            vs = max(float(getattr(self, "rotor_stribeck_vel",
                                   ROTOR_STRIBECK_VEL)), 1e-6)
            H = self.sim.dt_sub
            h = H / GEAR_SUBSTEPS
            for sub in range(self.sim.phys_per_ctrl * GEAR_SUBSTEPS):
                th = np.array([self.data.qpos[7], self.data.qpos[8]])
                ww = np.array([self.data.qvel[6], self.data.qvel[7]])
                dl = phi - th
                if bl > 0.0:
                    de = np.sign(dl) * np.maximum(np.abs(dl) - bl, 0.0)
                    eng = (np.abs(dl) > bl).astype(float)
                else:
                    de, eng = dl, np.ones(2)
                ts = k * de + c * eng * (wm - ww)
                self.data.ctrl[0] = ts[0]
                self.data.ctrl[1] = ts[1]
                mj.mj_step(self.model, self.data)
                if (sub + 1) % GEAR_SUBSTEPS == 0:
                    self._sync_state()
                    self._on_substep(H)
                # 转子：阻尼项隐式积分，保证稳定
                den = 1.0 + h * c * eng / I_r
                drive = tm - k * de + c * eng * ww
                w_free = (wm + h * drive / I_r) / den
                if fs > 0.0 or fk > 0.0:
                    # 粘滞：静止且驱动力矩不够静摩擦就保持静止；运动时摩擦只减速、
                    # 不越过零（越过就停住）。
                    fr = fk + (fs - fk) * np.exp(-(wm / vs) ** 2)
                    dirn = np.where(wm != 0.0, np.sign(wm), np.sign(drive))
                    w_new = w_free - dirn * h * fr / I_r / den
                    crossed = np.sign(w_new) != dirn
                    stuck = (wm == 0.0) & (np.abs(drive) <= fs)
                    wm = np.where(stuck | crossed, 0.0, w_new)
                else:
                    wm = w_free
                phi = phi + h * wm
            self._rotor = (phi, wm)

        def _imu_raw(self, dt):
            """直接读模型自己的 IMU，而不是对状态求导。
            Read the model own IMU instead of differentiating the state.

            ``imu`` 站点位于质心，用的是底盘自身的坐标轴（x 向前、z 向上），
            所以 ``s_acc[0]`` 就是固件的 ``accel_y`` 通道、``s_acc[2]`` 是它的
            ``accel_z``、``s_gyro[1]`` 是绕车体 y 轴的俯仰角速度。走传感器这条
            路，意味着一次踢击、或者一个轮子打滑后重新抓地，抵达姿态滤波器的
            方式和真车上一样——而且完全绕开了自由关节角速度的坐标系约定问题。

            The ``imu`` site sits at the COM with the chassis own axes
            (x forward, z up), so ``s_acc[0]`` is the firmware ``accel_y``
            channel, ``s_acc[2]`` its ``accel_z`` and ``s_gyro[1]`` the pitch
            rate about the body y axis.  Going through the sensors means a
            kick, or a wheel slipping and grabbing again, reaches the attitude
            filter the way it reaches the real one -- and it sidesteps the
            free joint angular-velocity frame convention entirely.
            """
            adr = self._imu_adr()
            if adr is None:
                return super()._imu_raw(dt)
            a_adr, g_adr = adr
            sd = self.data.sensordata
            return (float(sd[a_adr]), float(sd[a_adr + 2]),
                    float(sd[g_adr + 1]))

        def _imu_adr(self):
            adr = getattr(self, "_imu_adr_cache", None)
            if adr is not None:
                return adr
            mj = self._mj
            if mj is None:
                return None
            ia = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SENSOR, "s_acc")
            ig = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SENSOR, "s_gyro")
            if ia < 0 or ig < 0:
                return None
            self._imu_adr_cache = (int(self.model.sensor_adr[ia]),
                                   int(self.model.sensor_adr[ig]))
            return self._imu_adr_cache

        def reset(self, *args, **kw2):
            if not self._fw_ready:
                obs = MujocoCore.reset(self, *args, **kw2)
                self._init_firmware(**self._fw_kw)
                self._fw_ready = True
                self._last_wheel = self._wheel_angles()
                return obs
            return super().reset(*args, **kw2)

    return STM32TwinMujoco(**kw)
