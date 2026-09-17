# -*- coding: utf-8 -*-
"""结构假设网格：摩擦锁轮 x 死区相对补偿 x 传动刚度。其余参数取第三轮 + 实测陀螺零偏。"""
import os, sys, itertools
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BASE = dict(stall=0.5717, damp=0.0105, irot=6.33e-4, delay=2.0253, gnoise=0.000842,
            c=0.0778, bl=0.5806, ib=1.0, mnt=0.2804, v2=True)
KEYS = ("disp_pp", "disp_std", "f_disp", "f_gyro", "rev", "enc_abs", "ang_std", "gyro_rms", "demand")


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    fric, dead, k = args
    import replay as R, bench as B
    p = dict(BASE, fric=fric, dpos=dead, dneg=dead, k=k)
    ms = []
    for sd in (0, 1):
        r = R.replay("base_empty.txt", p, seed=sd, settle=5.0)
        if r["fell_at"] is not None:
            return args, None
        ms.append(B.metrics(r["twin"]))
    m = B.mean_metrics(ms)
    return args, (m, B.score(m)[0])


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    grid = list(itertools.product((0.002, 0.01, 0.03, 0.06), (1460.0, 1500.0, 1540.0), (2.0, 11.0)))
    with Pool(18, initializer=init) as pool:
        res = pool.map(one, grid)
    print("%-6s %-6s %-5s " % ("摩擦", "死区", "刚度") + "".join("%9s" % k for k in KEYS) + "%8s" % "综合")
    print("%-19s " % "真车" + "".join("%9.2f" % real[k] for k in KEYS) + "%8.3f" % B.score(real)[0])
    for (fric, dead, k), r in sorted(res, key=lambda x: (9 if x[1] is None else x[1][1])):
        if r is None:
            print("%-6g %-6.0f %-5g  摔了" % (fric, dead, k)); continue
        m, sc = r
        print("%-6g %-6.0f %-5g " % (fric, dead, k) + "".join("%9.2f" % m[kk] for kk in KEYS) + "%8.3f" % sc)
