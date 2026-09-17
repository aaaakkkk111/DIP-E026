# -*- coding: utf-8 -*-
"""单参数灵敏度：每次只动一个，看哪个把孪生往真车的静止指标推。"""
import sys, time, dataclasses
import numpy as np
import metrics as M, twinrun as T
import balance_bot.firmware.twin_baseline as TB
from balance_bot.firmware.motor import MotorCalibration
from balance_bot.firmware.robot import STM32_CAR

r = M.load("base_empty.txt")
REAL = M.standstill(r["gyro"], r["ang"], r["ml"], r["el"], r["er"])
KEYS = ("ang_std", "gyro_rms", "demand", "enc_abs", "pos_std", "peak_hz")


def feat(m):
    m = dict(m); m["demand"] = max(m["pwm_abs"] - 1500.0, 1.0); return m


def score(m):
    a, b = feat(REAL), feat(m)
    return float(np.mean([abs(np.log(max(b[k], 1e-3) / max(a[k], 1e-3))) for k in KEYS]))


def one(label, cal=None, robot=None, delay=None):
    old = TB.PWM_DELAY_TICKS
    if delay is not None: TB.PWM_DELAY_TICKS = delay
    kw = {}
    if robot is not None: kw["robot"] = robot
    t0 = time.time()
    try:
        d = T.run(32.0, motor_cal=cal, **kw)
    finally:
        TB.PWM_DELAY_TICKS = old
    if d["fell"] or len(d["gyro"]) < 6000:
        print("%-24s 摔了 (%d 拍)" % (label, len(d["gyro"]))); sys.stdout.flush(); return
    m = M.standstill(d["gyro"], d["ang"], d["ml"], d["el"], d["er"])
    f = feat(m)
    print("%-24s " % label + " ".join("%9.2f" % f[k] for k in KEYS)
          + "  sat%5.1f%%  score %.3f  (%.0fs)" % (m["pwm_sat"], score(m), time.time() - t0))
    sys.stdout.flush()


def cal(**kw):
    base = dict(motor_deadband=1500, r_wheel=STM32_CAR.r_wheel,
                stall_torque=STM32_CAR.tau_max, omega_noload=STM32_CAR.wheel_speed_max)
    base.update(kw); return MotorCalibration(**base)


def rob(**kw):
    return dataclasses.replace(STM32_CAR, **kw)


print("%-24s " % "配置" + " ".join("%9s" % k for k in KEYS))
print("%-24s " % "真车" + " ".join("%9.2f" % feat(REAL)[k] for k in KEYS))
one("孪生现状")
for s in (0.25, 0.5, 2.0):
    one(f"stall x{s}", cal=cal(stall_torque=0.40 * s))
for s in (0.2, 3.0, 8.0):
    one(f"tau_damp x{s}", cal=cal(tau_damp=0.15 * s))
for s in (0.3, 3.0, 6.0):
    one(f"I_rotor x{s}", robot=rob(I_rotor=6.6e-4 * s))
for dl in (0, 2, 4):
    one(f"delay {dl} 拍", delay=dl)
for s in (0.1, 10.0):
    one(f"b_wheel x{s}", robot=rob(b_wheel=max(STM32_CAR.b_wheel, 1e-4) * s))
    one(f"b_pitch x{s}", robot=rob(b_pitch=max(STM32_CAR.b_pitch, 1e-4) * s))
