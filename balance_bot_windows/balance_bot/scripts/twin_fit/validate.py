# -*- coding: utf-8 -*-
"""拟合结果检验。

1. 拟合用过的数据：静止 + 两次扫频，现状 vs 拟合后，逐项对真车
2. **留出数据** kd120.txt（没参与拟合）：Kd 改成 120、电池 11.1 V 的静止录数。
   拟合出的参数要能预测另一套增益下的表现，才说明学到的是这台车，不是
   凑出了一组只对 kd=48 成立的数。
"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
import fit as FT
import metrics as M

NOW = dict(stall=0.40, damp=0.15, irot=6.6e-4, dead=1500.0, delay=1.0, gnoise=1e-4, bl=0.0)
R = json.load(open(os.path.join(HERE, "fit_result.json")))
FIT = R["params"]
print("拟合参数:", {k: round(v, 5) for k, v in FIT.items()})

rs, rfr = FT.real()
KEYS = ("ang_std", "ang_pp", "gyro_rms", "gyro_pk", "pwm_abs", "pwm_sat", "pwm_rev",
        "enc_abs", "pos_pp", "pos_std", "peak_hz")


def table(title, real, a, b):
    print("\n=== %s ===" % title)
    print("%-10s %10s %10s %10s" % ("指标", "真车", "现状", "拟合后"))
    for k in KEYS:
        va = "摔了" if a is None else "%.2f" % a[k]
        vb = "摔了" if b is None else "%.2f" % b[k]
        print("%-10s %10.2f %10s %10s" % (k, real[k], va, vb))
    for i, (lo, hi) in enumerate(M.BANDS):
        va = "-" if a is None else "%.2f" % a["bands"][i]
        vb = "-" if b is None else "%.2f" % b["bands"][i]
        print("%-10s %10.2f %10s %10s" % ("%g-%gHz" % (lo, hi), real["bands"][i], va, vb))


table("拟合数据：静止 kd=48", rs, FT.simulate(NOW, "still"), FT.simulate(FIT, "still"))

for amp in (300, 500):
    a = FT.simulate(NOW, "sweep", amp); b = FT.simulate(FIT, "sweep", amp)
    print("\n=== 拟合数据：扫频 幅值 %d（增益 °/s per 1000 PWM / 相位）===" % amp)
    print("%5s %14s %14s %14s" % ("Hz", "真车", "现状", "拟合后"))
    for i, f in enumerate(FT.F):
        r = rfr[amp][i]
        sa = "摔了" if a is None else "%5.0f %5.0f°" % (a[i][1], a[i][2])
        sb = "摔了" if b is None else "%5.0f %5.0f°" % (b[i][1], b[i][2])
        print("%5.1f %6.0f %5.0f° %14s %14s" % (f, r[1], r[2], sa, sb))

k = M.load("kd120.txt")
rk = M.standstill(k["gyro"], k["ang"], k["ml"], k["el"], k["er"])
g120 = dict(balance_kd=1.20)
table("留出数据（未参与拟合）：静止 kd=120，电池 11.1 V", rk,
      FT.simulate(NOW, "still", gains=g120, battery=11.1, seconds=30.0),
      FT.simulate(FIT, "still", gains=g120, battery=11.1, seconds=30.0))
