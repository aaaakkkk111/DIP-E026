# -*- coding: utf-8 -*-
"""参数 -> 孪生构造（第二版，建在当前源码之上，不重复叠加源码已有的量）。

源码已经有：弹性传动、正反死区、陀螺噪声、陀螺零偏、PWM 延迟、IMU 零偏。
这里把它们全部显式覆盖成 p 里的值：
  stall damp dpos dneg k c bl        -> MotorCalibration
  irot ib                            -> RobotParams（I_body 相对当前源码值的倍率）
  delay gnoise                       -> 模块全局（_init_firmware 在首次 reset 时读取）
  gbias_dps                          -> 陀螺零偏，默认实测 +2.02 °/s，不拟合
  mnt                                -> IMU 安装零偏（度）
  fric                               -> 轮子关节库仑摩擦 N·m（MuJoCo frictionloss，减速箱）
"""
import dataclasses
import numpy as np


def build(p):
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.firmware.motor import MotorCalibration
    from balance_bot.firmware.robot import STM32_CAR
    from balance_bot.params import DisturbanceConfig
    TB.PWM_DELAY_TICKS = float(p["delay"])
    TB.GYRO_NOISE_RAD_S = float(p["gnoise"])
    TB.GYRO_BIAS_RAD_S = float(np.radians(p.get("gbias_dps", 2.02)))
    TB.GYRO_VIB_RAD_S = float(p.get("vib", 0.0))
    TB.ROLLING_RESIST_M = float(p.get("roll", 0.0))
    TB.GYRO_VIB_MOTION = float(p.get("vibm", 0.0))
    TB.ROTOR_FRIC_STATIC = float(p.get("fs", 0.0))
    TB.ROTOR_FRIC_KINETIC = float(p.get("fs", 0.0)) * float(p.get("fkr", 1.0))
    TB.ROTOR_STRIBECK_VEL = float(p.get("vs", 1.0))
    cal = MotorCalibration(
        stall_torque=p["stall"], tau_damp=p["damp"],
        motor_deadband=int(round(p["dpos"])), motor_deadband_neg=float(p["dneg"]),
        gear_stiffness=p["k"], gear_damping=p["c"], gear_backlash_deg=p["bl"],
        gear_friction=float(p.get("fric", 0.0)),
        motor_deadband_lr_split=float(p.get("lrs", 0.0)),
        r_wheel=STM32_CAR.r_wheel, omega_noload=STM32_CAR.wheel_speed_max)
    robot = dataclasses.replace(STM32_CAR, I_rotor=p["irot"], I_body=STM32_CAR.I_body * p.get("ib", 1.0))
    kw = dict(motor_cal=cal, robot=robot, disturbance=DisturbanceConfig(),
              mount_offset_deg=p.get("mnt", 0.2804))
    fric = float(p.get("fric", 0.0))

    def post(core):
        core.model.dof_frictionloss[6] = fric
        core.model.dof_frictionloss[7] = fric
    return kw, post
