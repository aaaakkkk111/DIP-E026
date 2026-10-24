# -*- coding: utf-8 -*-
"""Score v5.2 (shift by frequency: low-frequency sway up, 8-16 Hz chatter down, calm = no
change) against factory / v4 / v5.1, 2026-10-24.

  bench   release benchmark (load CoM at the plate, 0.105 m)
  high    load put on a standing car, CoM 0.15 / 0.20 m above the axle (a real load sits on
          the top plate, its CoM above it -- this reproduces the real car's growing sway),
          0.5 / 1 / 2 kg, 15 s: falls, sway amplitude over the last 3 s, final gear, gear
          changes over the last 5 s (calm -> no change)
  hcourse benchmark course with those high loads
  static  course with 0..4 kg (CoM at the plate)
  chatter gear forced too stiff (empty car, f=1 latched): time until the 8-16 Hz chatter is
          gone (< 5 deg/s for 0.25 s), final gear, and whether it stays there
    python scripts/rl/v52_score.py bench|high|hcourse|static|chatter
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
import v51_score as V

C.DLLS = {"v4 rel": "v5core_off.dll", "v5.1": "v51core.dll", "v5.2": os.environ.get("V52_DLL", "v52core.dll")}
KINDS = ("normal", "heavy", "v4 rel", "v5.1", "v5.2")
NAMES = {"normal": "factory Normal", "heavy": "factory Weight_M", "v4 rel": "v4 release",
         "v5.1": "v5.1", "v5.2": "v5.2"}
L.KINDS, L.NAMES = KINDS, NAMES
H = [None]


def _init(h=None):
    C._init()
    lib = C.LIBS["v5.1"]
    lib.v4_set_opt.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.v4_set_opt(30, 40.0)
    lib = C.LIBS["v5.2"]
    lib.v4_set_opt.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.v4_set_opt(int(os.environ.get("V52_OPT", "50")), 40.0)
    lib.v4_set_swup.argtypes = [ctypes.c_float]
    lib.v4_set_swup(float(os.environ.get("V52_SWUP", "1.5")))
    lib.v4_sw.restype = ctypes.c_float
    if h is not None:
        set_height(h)


def set_height(h):
    from balance_bot.rl import payload as PL
    import balance_bot.rl.e2e_env as E
    orig = PL.with_payload.__wrapped__ if hasattr(PL.with_payload, "__wrapped__") else PL.with_payload
    def wp(robot, m, h_p=h):
        return orig(robot, m, h)
    wp.__wrapped__ = orig
    PL.with_payload = wp
    E.with_payload = wp
    H[0] = h


def ctl_for(kind):
    return C.V5C(C.LIBS[kind]) if kind in C.DLLS else V.Fixed(kind)


def job_high(a):
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    kind, h, kg, sd = a
    set_height(h)
    env = C._env(sd)
    c = env.core
    ctl = ctl_for(kind)
    for _ in range(int(4.0 * FS)):
        env.step(ctl(env))
    env._apply_payload(kg)
    T.set_tilt(env, 3.0, 60.0)
    lp, buf, fs = 0.0, [], []
    for n in range(int(15.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True, t=n / FS)
        th = float(c._angle_filt)
        lp += 0.17 * (th - lp)
        if n >= int(12.0 * FS):
            buf.append(lp)
        fs.append(float(getattr(ctl, "f", np.nan)))
    tail = fs[int(10.0 * FS):]
    changes = sum(1 for i in range(1, len(tail)) if abs(tail[i] - tail[i - 1]) > 0.05)
    return a, dict(fell=False, sway=0.5 * (max(buf) - min(buf)), f_end=fs[-1], changes=changes)


def job_hcourse(a):
    kind, h, kg, sd = a
    set_height(h)
    r = C.FA.run(kind, kg, 0.0, 0.0, sd, 0.0)
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else ((r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return a, dict(pen=pen, ok=r["ok"])


def job_chatter(a):
    from balance_bot.rl.e2e_env import FS
    kind, kg, sd = a
    env = C._env(sd, kg)
    c = env.core
    lib = C.LIBS[kind]
    ctl = C.V5C(lib)
    for _ in range(int(4.0 * FS)):
        env.step(ctl(env))
    lib.v4_force(1.0)
    osc = L.Osc()
    calm, t_calm, fs = 0, None, []
    hp = bp = en = 0.0
    fcalm, t_fast, on = 0, None, False
    for n in range(int(8.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True)
        o = osc(float(c._gyro_lsb))
        g = float(c._gyro_lsb)                         # fast 8-16 Hz meter (~0.1 s), same filter as the firmware oscf
        hp += 0.200849 * (g - hp); bp += 0.334511 * ((g - hp) - bp); en += 0.05 * (bp * bp - en)
        of = en ** 0.5 / 16.4
        on = on or of >= 15.0
        if on:
            fcalm = fcalm + 1 if of < 5.0 else 0
            if fcalm >= 50 and t_fast is None:
                t_fast = (n - 50) / FS
        if n > 20:
            calm = calm + 1 if o < 5.0 else 0
            if calm >= 50 and t_calm is None:
                t_calm = (n - 50) / FS
        fs.append(ctl.f)
    return a, dict(fell=False, t_calm=t_calm, t_fast=t_fast, f_end=fs[-1], f_min=min(fs))


def main():
    what = sys.argv[1]
    if what == "bench":
        import mc_twin_test as M
        jobs = [(k, ci, sd) for k in C.DLLS for ci in range(len(M.CASES)) for sd in M.SEEDS]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(C.job_bench, jobs)
        print("release benchmark (lower is better)")
        for k in C.DLLS:
            per = {}
            for (kk, ci, sd), r in res:
                if kk == k:
                    per.setdefault(ci, []).append(r)
            tot = sum(M.CASES[ci][3] * np.mean([r["pen"] for r in v]) for ci, v in per.items())
            print("  %-12s total %6.2f  falls %d  per case %s" % (
                NAMES[k], tot, sum(1 for v in per.values() for r in v if not r["ok"]),
                " ".join("%.2f" % np.mean([r["pen"] for r in per[ci]]) for ci in sorted(per))))
    elif what == "high":
        jobs = [(k, h, kg, sd) for k in KINDS for h in (0.15, 0.20) for kg in (0.5, 1.0, 2.0) for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_high, jobs)
        print("load put on a standing car, load CoM high; 15 s; 3 seeds")
        print("cell = falls / sway amplitude deg (last 3 s) / final gear / gear changes in the last 5 s")
        for h in (0.15, 0.20):
            print("== load CoM %.2f m above the axle" % h)
            for k in KINDS:
                cells = []
                for kg in (0.5, 1.0, 2.0):
                    rs = [r for (kk, hh, g, sd), r in res if kk == k and hh == h and g == kg]
                    ok = [r for r in rs if not r["fell"]]
                    nf = len(rs) - len(ok)
                    if not ok:
                        cells.append("FELL 3/3")
                        continue
                    cells.append("%d / %.2f / %.1f / %.1f" % (nf, np.mean([r["sway"] for r in ok]),
                                                            np.mean([r["f_end"] for r in ok]), np.mean([r["changes"] for r in ok])))
                print("  %-17s " % NAMES[k] + " | ".join("%gkg %-22s" % (kg, c) for kg, c in zip((0.5, 1.0, 2.0), cells)))
    elif what == "hcourse":
        jobs = [(k, h, kg, sd) for k in KINDS for h in (0.15, 0.20) for kg in (0.5, 1.0, 2.0) for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_hcourse, jobs)
        print("benchmark course with a high load; 3 seeds; cell = penalty / falls")
        for h in (0.15, 0.20):
            print("== load CoM %.2f m" % h)
            for k in KINDS:
                cells = []
                for kg in (0.5, 1.0, 2.0):
                    rs = [r for (kk, hh, g, sd), r in res if kk == k and hh == h and g == kg]
                    cells.append("%.1f/%d" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs)))
                print("  %-17s " % NAMES[k] + "  ".join("%gkg %-9s" % (kg, c) for kg, c in zip((0.5, 1.0, 2.0), cells)))
    elif what == "static":
        jobs = [(k, kg, sd) for k in KINDS for kg in L.LOADS for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(L.job_static, jobs)
        print("course with a fixed load (CoM at the plate), 3 seeds; cell = penalty / falls / chatter")
        print("  %-18s" % "kg" + "".join("%13.1f" % kg for kg in L.LOADS))
        for k in KINDS:
            cells = []
            for kg in L.LOADS:
                rs = [r for (kk, g, sd), r in res if kk == k and g == kg]
                cells.append("%5.1f/%d/%4.1f" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs),
                                                 np.mean([r["osc"] for r in rs])))
            print("  %-18s" % NAMES[k] + "".join("%13s" % c for c in cells))
    elif what == "swap":
        pairs = [(x, y) for x in L.SWAP_SET for y in L.SWAP_SET if x != y]
        jobs = [(k, x, y, sd) for k in C.DLLS for x, y in pairs for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(L.job_swap, jobs)
        print("load swaps 0/1/2/3/4 kg (CoM at the plate), 20 pairs x 3 seeds")
        for k in C.DLLS:
            rs = [r for (kk, x, y, sd), r in res if kk == k]
            ok = [r for r in rs if not r["fell"]]
            fell = {}
            for (kk, x, y, sd), r in res:
                if kk == k and r["fell"]:
                    fell[(x, y)] = fell.get((x, y), 0) + 1
            print("  %-12s falls %d/%d %s | shaking ends after %.2f s | tilt sd %.3f" % (
                NAMES[k], sum(r["fell"] for r in rs), len(rs), fell or "", np.mean([r["shake"] for r in ok]),
                np.mean([r["th_sd"] for r in ok])))
    else:
        jobs = [(k, kg, sd) for k in C.DLLS for kg in (0.0, 0.3) for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_chatter, jobs)
        print("gear forced to f=1 (too stiff), 8 s; 3 seeds; chatter gone = 8-16 Hz < 5 deg/s for 0.25 s")
        for kg in (0.0, 0.3):
            for k in C.DLLS:
                rs = [r for (kk, g, sd), r in res if kk == k and g == kg]
                ok = [r for r in rs if not r.get("fell")]
                tc = [r["t_calm"] for r in ok if r["t_calm"] is not None]
                tf = [r["t_fast"] for r in ok if r.get("t_fast") is not None]
                print("  %.1f kg %-12s | chatter gone after %s (%d/3) | fast meter %s | final gear %s" % (
                    kg, NAMES[k], ("%.2f s" % np.mean(tc)) if tc else "never", len(tc),
                    ("%.2f s" % np.mean(tf)) if tf else "-",
                    "/".join("%.1f" % r["f_end"] for r in ok)))


if __name__ == "__main__":
    main()
