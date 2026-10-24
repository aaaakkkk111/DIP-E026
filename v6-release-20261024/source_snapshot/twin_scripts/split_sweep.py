# -*- coding: utf-8 -*-
"""Light/middle split for the v5 three-level probe (2026-10-21): benchmark + 0/0.5/1/1.5 kg
course, per light split.  python scripts/rl/split_sweep.py"""
import ctypes, os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chatter_twin as C
import mc_twin_test as M
SPLITS = (23.0, 25.0, 27.0, 30.0)
C.DLLS = {"v5 (on)": "v5core_sw.dll"}


def _init(light):
    C._init()
    lib = C.LIBS["v5 (on)"]
    lib.v4_splits.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.v4_splits(light, 20.0)


def job(a):
    light, kg, imp, sd = a
    r = C.FA.run("v5 (on)", kg, imp, 0.0, sd, 0.0)
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else ((r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return a, pen, r["ok"]


if __name__ == "__main__":
    out = {}
    for light in SPLITS:
        jobs = [(light, kg, imp, sd) for _cn, kg, imp, _w in M.CASES for sd in M.SEEDS]
        jobs += [(light, kg, 0.0, sd) for kg in (0.5, 1.5) for sd in M.SEEDS]
        with Pool(16, initializer=_init, initargs=(light,)) as pool:
            out[light] = pool.map(job, jobs)
    print("v5 three-level probe, middle split 20; benchmark total (lower better) and extra loads")
    for light, res in out.items():
        per = {}
        for (l, kg, imp, sd), pen, ok in res:
            per.setdefault((kg, imp), []).append((pen, ok))
        tot = sum(w * np.mean([p for p, _ in per[(kg, imp)]]) for _cn, kg, imp, w in M.CASES)
        def cell(kg, imp=0.0):
            v = per[(kg, imp)]
            return "%.2f/%d" % (np.mean([p for p, _ in v]), sum(not o for _, o in v))
        print("  light split %4.1f | benchmark %6.2f | empty %s  0.5kg %s  1kg %s  1.5kg %s  2kg %s  4kg %s  empty+3N %s" % (
            light, tot, cell(0.0), cell(0.5), cell(1.0), cell(1.5), cell(2.0), cell(4.0), cell(0.0, 3.0)))
