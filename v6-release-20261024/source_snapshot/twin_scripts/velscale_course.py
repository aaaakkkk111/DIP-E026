# -*- coding: utf-8 -*-
"""Velocity-loop scale as a second gear axis (2026-10-24).  Fixed gains: v4 ladder at f, with the
velocity kp/ki multiplied by s.  Benchmark course with load kg at CoM height h.
cell = penalty (lower better) / falls.   python scripts/rl/velscale_course.py"""
import os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import v52_score as V
FS_ = (0.4, 0.7, 1.0)
SS = (0.4, 0.6, 0.8, 1.0)
HS = (0.105, 0.20, 0.30)
KGS = (0.0, 1.0, 3.0)


def job(a):
    f, s, h, kg, sd = a
    import chatter_twin as C
    import switch_v3 as SV3
    V.set_height(h)
    base = C.FA.Ctl

    class Fix(base):
        def __init__(self, kind, kg_true):
            base.__init__(self, "heavy", kg_true)
            gn, gh = dict(SV3.G_N), dict(SV3.G_H)
            g = {k: gn[k] + (gh[k] - gn[k]) * f for k in ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki", "turn_kp", "turn_kd")}
            g["velocity_kp"] *= s; g["velocity_ki"] *= s
            self.pid.g.update(g)
    C.FA.Ctl = Fix
    try:
        r = C.FA.run("heavy", kg, 0.0, 0.0, sd, 0.0)
    finally:
        C.FA.Ctl = base
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else ((r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return a, dict(pen=pen, ok=r["ok"])


if __name__ == "__main__":
    jobs = [(f, s, h, kg, sd) for f in FS_ for s in SS for h in HS for kg in KGS for sd in (0, 1)]
    with Pool(16, initializer=V._init) as pool:
        res = pool.map(job, jobs)
    print("course, fixed gains: ladder f, velocity x s; 2 seeds; cell = penalty / falls")
    for h in HS:
        print("== load CoM %.3f m" % h)
        print("  %-10s" % "f / s" + "".join("%14s" % ("%g kg" % kg) for kg in KGS))
        for f in FS_:
            for s in SS:
                cells = []
                for kg in KGS:
                    rs = [r for (ff, ss, hh, g, sd), r in res if (ff, ss, hh, g) == (f, s, h, kg)]
                    cells.append("%.1f/%d" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs)))
                print("  %-10s" % ("%g / %g" % (f, s)) + "".join("%14s" % c for c in cells))
