# -*- coding: utf-8 -*-
"""Score v5.1 (v4_core.c with the small-load oscillation fixes) against v4 / v5, 2026-10-21.

Controllers: factory Normal, factory Weight_M, v4 release (v5core_off.dll), v5 previous
(v5core_sw.dll, splits 25/20), v5.1 (v51core.dll, v4_opt from OPT).
  bench  : release benchmark (5 cases x 3 seeds, lower is better)
  static : course with 0..4 kg in 0.5 kg steps (load_matrix.job_static)
  swap   : loads 0/1/2/3/4 swapped on a standing car (load_matrix.job_swap)
  cycle  : small load (0.2..1 kg) put on a standing empty car with a nudge, 20 s, gear
           backlash BL: oscillation episodes (calm -> oscillating), seconds oscillating,
           gear jumps (|df| > 0.05), final gear, falls
    python scripts/rl/v51_score.py bench|static|swap|cycle [opt]
"""
import ctypes
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chatter_twin as C
import load_matrix as L

C.DLLS = {"v4 rel": "v5core_off.dll", "v5 prev": "v5core_sw.dll", "v5.1": "v51core.dll"}
KINDS = ("normal", "heavy", "v4 rel", "v5 prev", "v5.1")
L.KINDS = KINDS
L.NAMES = {"normal": "factory Normal", "heavy": "factory Weight_M", "v4 rel": "v4 release",
           "v5 prev": "v5 previous", "v5.1": "v5.1"}
OPT = [31]


def _init(opt=31):
    C._init()
    lib = C.LIBS["v5 prev"]
    lib.v4_splits.argtypes = [ctypes.c_float, ctypes.c_float]
    lib.v4_splits(25.0, 20.0)
    lib = C.LIBS["v5.1"]
    lib.v4_set_opt.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.v4_set_opt(int(opt), float(os.environ.get("V51_WAMP", "40")))


class Fixed:
    """factory Normal / Weight_M with fixed gains."""
    def __init__(self, kind):
        import switch_v3 as SV3
        self.pid = SV3.SwitchV3().pid
        g = SV3.G_N if kind == "normal" else SV3.G_H
        self.pid.g.update({k: g[k] for k in ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki", "turn_kp", "turn_kd")})
        self.f = 0.0 if kind == "normal" else 1.0

    def __call__(self, env):
        return self.pid(env)


def job_cycle(a):
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    kind, kg, bl, sd = a
    env = C._env(sd)
    c = env.core
    c.motor.cal.gear_backlash_deg = bl
    if kind in C.DLLS:
        ctl = C.V5C(C.LIBS[kind])
    else:
        ctl = Fixed(kind)
    for _ in range(int(4.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True, pre=True)
    env._apply_payload(kg)
    T.set_tilt(env, 3.0, 60.0)
    glp, win, eps, osc_t, was = 0.0, [], 0, 0.0, False
    fs, jumps = [], 0
    for n in range(int(20.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True, pre=False, t=n / FS)
        g = float(c._gyro_lsb) / 16.4
        glp += 0.030928 * (g - glp)
        win.append(abs(g - glp))
        if len(win) == 50:                       # 0.25 s windows
            on = np.sqrt(np.mean(np.square(win))) > 12.0
            if on:
                osc_t += 0.25
                if not was:
                    eps += 1
            was = on
            win = []
        f = float(getattr(ctl, "f", np.nan))
        if fs and abs(f - fs[-1]) > 0.05:
            jumps += 1
        fs.append(f)
    return a, dict(fell=False, eps=eps, osc_t=osc_t, jumps=jumps, f_end=fs[-1])


def main():
    what = sys.argv[1]
    opt = int(sys.argv[2]) if len(sys.argv) > 2 else 31
    if what == "bench":
        import mc_twin_test as M
        only = sys.argv[3:] == ["only"]
        jobs = [(k, ci, sd) for k in KINDS if k in C.DLLS and (not only or k == "v5.1")
                for ci in range(len(M.CASES)) for sd in M.SEEDS]
        with Pool(16, initializer=_init, initargs=(opt,)) as pool:
            res = pool.map(C.job_bench, jobs)
        print("release benchmark (lower is better), v5.1 opt %d" % opt)
        for k in KINDS:
            if k not in C.DLLS or (only and k != "v5.1"):
                continue
            per = {}
            for (kk, ci, sd), r in res:
                if kk == k:
                    per.setdefault(ci, []).append(r)
            tot = sum(M.CASES[ci][3] * np.mean([r["pen"] for r in v]) for ci, v in per.items())
            print("  %-12s total %6.2f  falls %d  per case %s" % (
                L.NAMES[k], tot, sum(1 for v in per.values() for r in v if not r["ok"]),
                " ".join("%.2f" % np.mean([r["pen"] for r in per[ci]]) for ci in sorted(per))))
    elif what == "static":
        jobs = [(k, kg, sd) for k in KINDS for kg in L.LOADS for sd in (0, 1, 2)]
        with Pool(16, initializer=_init, initargs=(opt,)) as pool:
            res = pool.map(L.job_static, jobs)
        print("course with a fixed load, 3 seeds; cell = penalty / falls / chatter; v5.1 opt %d" % opt)
        print("  %-18s" % "kg" + "".join("%13.1f" % kg for kg in L.LOADS))
        for k in KINDS:
            cells = []
            for kg in L.LOADS:
                rs = [r for (kk, g, sd), r in res if kk == k and g == kg]
                cells.append("%5.1f/%d/%4.1f" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs),
                                                 np.mean([r["osc"] for r in rs])))
            print("  %-18s" % L.NAMES[k] + "".join("%13s" % c for c in cells))
    elif what == "swap":
        pairs = [(x, y) for x in L.SWAP_SET for y in L.SWAP_SET if x != y]
        jobs = [(k, x, y, sd) for k in KINDS if k in C.DLLS for x, y in pairs for sd in (0, 1, 2)]
        with Pool(16, initializer=_init, initargs=(opt,)) as pool:
            res = pool.map(L.job_swap, jobs)
        print("load swaps 0/1/2/3/4 kg, 20 pairs x 3 seeds; v5.1 opt %d" % opt)
        for k in KINDS:
            if k not in C.DLLS:
                continue
            rs = [r for (kk, x, y, sd), r in res if kk == k]
            ok = [r for r in rs if not r["fell"]]
            print("  %-12s falls %d/%d | shaking ends after %.2f s (mean) | tilt sd %.3f | gyro %.2f" % (
                L.NAMES[k], sum(r["fell"] for r in rs), len(rs), np.mean([r["shake"] for r in ok]),
                np.mean([r["th_sd"] for r in ok]), np.mean([r["gy"] for r in ok])))
    else:
        loads = (0.2, 0.3, 0.5, 0.8, 1.0)
        bls = (0.84, 1.5)
        jobs = [(k, kg, bl, sd) for k in KINDS for kg in loads for bl in bls for sd in (0, 1, 2)]
        with Pool(16, initializer=_init, initargs=(opt,)) as pool:
            res = pool.map(job_cycle, jobs)
        print("small load put on a standing empty car (nudge), 20 s; 3 seeds; v5.1 opt %d" % opt)
        print("cell = oscillation episodes / s oscillating (of 20) / gear jumps / final gear  (FELL n)")
        for bl in bls:
            print("== gear backlash %.2f deg%s" % (bl, " (twin default)" if bl < 1 else " (more sensitive)"))
            print("  %-18s" % "kg" + "".join("%22.1f" % kg for kg in loads))
            for k in KINDS:
                cells = []
                for kg in loads:
                    rs = [r for (kk, g, b, sd), r in res if kk == k and g == kg and b == bl]
                    ok = [r for r in rs if not r["fell"]]
                    nf = len(rs) - len(ok)
                    if not ok:
                        cells.append("FELL %d" % nf)
                        continue
                    cells.append("%.1f/%4.1f/%4.1f/%.2f%s" % (np.mean([r["eps"] for r in ok]), np.mean([r["osc_t"] for r in ok]),
                                                         np.mean([r["jumps"] for r in ok]), np.mean([r["f_end"] for r in ok]),
                                                         (" F%d" % nf) if nf else ""))
                print("  %-18s" % L.NAMES[k] + "".join("%22s" % c for c in cells))


if __name__ == "__main__":
    main()
