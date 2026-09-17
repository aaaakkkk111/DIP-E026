# -*- coding: utf-8 -*-
"""每个真车录波里轮速到过多少（占空载转速的比例），以及 PWM 用到哪一档。"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)
import replay as R
from balance_bot.firmware.motor import MotorCalibration
from balance_bot.firmware.robot import STM32_CAR as P
cal = MotorCalibration()
v_free = cal.omega_noload * P.r_wheel
CPR = 1320.0
print("空载车速 %.3f m/s；1 计数/5ms = %.4f m/s" % (v_free, 0.2105 * 200 / CPR))
for name in ("base_empty.txt", "kd120.txt", "sweep_m1_300.txt", "sweep_A_300.txt",
             "sweep_A_500.txt", "sweep_300.txt"):
    d = R.parse(name)["real"]
    el = np.nan_to_num(np.asarray(d["el"], float))
    v = el * 200.0 / CPR * 0.2105
    ml = np.nan_to_num(np.asarray(d["ml"], float))
    print("%-17s 轮速 rms %.3f 峰 %.3f m/s = 峰值占空载 %4.1f%%   |PWM-1500| 中位 %4.0f 峰 %4.0f"
          % (name, v.std(), np.abs(v).max(), 100 * np.abs(v).max() / v_free,
             np.median(np.abs(ml[ml != 0]) - 1500), np.abs(np.abs(ml).max() - 1500)))
