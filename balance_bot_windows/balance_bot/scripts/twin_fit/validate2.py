# -*- coding: utf-8 -*-
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
import fit as FT
import metrics as M

structs = sys.argv[1:] or ["A", "B"]
PS = {s: json.load(open(os.path.join(HERE, "fit2_%s.json" % s))) for s in structs}
import fit2 as F2


def sim(s, *a, **k):
    F2.SPACE = F2.COMMON + ([("k", 1, 1, True), ("c", 1, 1, True)] if s == "B" else [])
    return F2.simulate(PS[s]["params"], *a, **k)


for s in structs:
    print("结构 %s 总误差 %.3f  %s" % (s, PS[s]["best"], {k: round(v, 3) for k, v in PS[s]["parts"].items()}))
    print("   参数", {k: round(v, 5) for k, v in PS[s]["params"].items()})

rs, rfr = FT.real()
KEYS = ("ang_mean", "ang_std", "ang_pp", "gyro_rms", "gyro_pk", "pwm_abs", "pwm_sat", "pwm_rev",
        "enc_abs", "pos_pp", "pos_std", "peak_hz")


def table(title, real, res):
    print("\n=== %s ===" % title)
    print("%-12s %10s" % ("指标", "真车") + "".join("%10s" % ("结构" + s) for s in structs))
    for k in KEYS:
        print("%-12s %10.2f" % (k, real[k]) + "".join(
            "%10s" % ("摔了" if res[s] is None else "%.2f" % res[s][k]) for s in structs))
    for i, (lo, hi) in enumerate(M.BANDS):
        print("%-12s %10.2f" % ("%g-%gHz" % (lo, hi), real["bands"][i]) + "".join(
            "%10s" % ("-" if res[s] is None else "%.2f" % res[s]["bands"][i]) for s in structs))


table("拟合数据：静止 kd=48", rs, {s: sim(s, "still") for s in structs})
for amp in (300, 500):
    res = {s: sim(s, "sweep", amp) for s in structs}
    print("\n=== 拟合数据：扫频 幅值 %d ===" % amp)
    print("%5s %13s" % ("Hz", "真车") + "".join("%14s" % ("结构" + s) for s in structs))
    for i, f in enumerate(F2.F):
        r = rfr[amp][i]
        print("%5.1f %6.0f %5.0f°" % (f, r[1], r[2]) + "".join(
            "%14s" % ("摔了" if res[s] is None else "%6.0f %5.0f°" % (res[s][i][1], res[s][i][2]))
            for s in structs))
k = M.load("kd120.txt")
rk = M.standstill(k["gyro"], k["ang"], k["ml"], k["el"], k["er"])
table("留出数据（未参与拟合）：静止 kd=120，电池 11.1 V", rk,
      {s: sim(s, "still", gains=dict(balance_kd=1.20), battery=11.1, seconds=30.0) for s in structs})
