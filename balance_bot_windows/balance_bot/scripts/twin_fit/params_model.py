# -*- coding: utf-8 -*-
"""参数 -> 孪生构造。所有「没实测验证」的量都在这里当自由参数。

  stall   堵转力矩（电机力矩常数）          dpos/dneg 正 / 反向电机死区
  damp    电机阻尼力矩                      irot     折算转子惯量（挂关节 armature）
  delay   PWM 生效延迟（拍，0-10 = 0-50 ms） gnoise   陀螺噪声 rad/s
  bl      齿轮间隙（度）                    ib / lc  车身俯仰惯量 / 质心高度 倍率
  enc     编码器计数/圈 倍率                mnt      IMU 安装零偏（度）
  k / c   （可选）串联弹性传动刚度 / 阻尼
"""
import dataclasses
import numpy as np


def build(p):
    if p.get("v2"):                      # 第二版：建在当前源码之上
        import params_model2
        return params_model2.build(p)
    if p.get("source"):                  # 直接用源码默认值，什么都不覆盖
        return {}, (lambda core: None)
    import sea
    import fit as FT
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.firmware.motor import MotorCalibration
    from balance_bot.firmware.robot import STM32_CAR, STM32_FIRMWARE_CONST as FC
    from balance_bot.params import DisturbanceConfig
    TB.PWM_DELAY_TICKS = p.get("delay", 1.0)
    dpos = p.get("dpos", p.get("dead", 1500.0))
    dneg = p.get("dneg", dpos)
    cal = MotorCalibration(motor_deadband=int(round(dpos)), r_wheel=STM32_CAR.r_wheel,
                           stall_torque=p.get("stall", 0.40), tau_damp=p.get("damp", 0.15),
                           omega_noload=STM32_CAR.wheel_speed_max)
    on_wheel = p.get("rotor_on_wheel", False)     # True = 原孪生的（错误）挂法
    rob = dict(I_rotor=(p.get("irot", 6.6e-4) if on_wheel else 1e-6),
               I_body=STM32_CAR.I_body * p.get("ib", 1.0),
               l_com=STM32_CAR.l_com * p.get("lc", 1.0))
    sea_on = "k" in p
    if sea_on:
        rob["tau_max"] = 50.0
    robot = dataclasses.replace(STM32_CAR, **rob)
    kw = dict(motor_cal=cal, robot=robot,
              disturbance=DisturbanceConfig(noise_pitch_rate=p.get("gnoise", 0.0)),
              mount_offset_deg=p.get("mnt", 0.0))
    bl = np.radians(p.get("bl", 0.0))
    cpr = FC.true_counts_per_rev * p.get("enc", 1.0)

    def post(core):
        m = core.motor
        period = m.cal.pwm_period
        ddp, ddn = dpos / period, dneg / period

        def duty(ccr):
            d = float(np.clip(ccr / period, -1.0, 1.0))
            dd = ddp if d > 0 else ddn
            return float(np.sign(d) * max(0.0, abs(d) - dd) / (1.0 - dd))
        m.duty = duty
        core.enc_l.cpr = cpr
        core.enc_r.cpr = cpr
        if sea_on:
            sea.install(core, k=p["k"], c=p["c"], bl_rad=bl, I_r=p["irot"])
        else:
            if not on_wheel:
                core.model.dof_armature[6] = p.get("irot", 6.6e-4)
                core.model.dof_armature[7] = p.get("irot", 6.6e-4)
            FT.wrap_backlash(m, bl)
    return kw, post
