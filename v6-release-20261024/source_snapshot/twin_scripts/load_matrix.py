# -*- coding: utf-8 -*-
"""Loads 0..4 kg in 0.5 kg steps, and swapping loads, 2026-10-21.

Controllers (same twin): factory Normal (mode 1, fixed gains), factory Weight_M (fixed),
v4 = v4_core.c with CHATTER_DOWN=0 (v5core_off.dll), v5c = with chatter downshift
(v5core.dll, candidate, not released).
  static : release-benchmark course (stand, drive to the waypoints, stop) with load KG,
           3 seeds; penalty = benchmark rule (lower is better), falls, chatter
  swap   : boot with load A, stand 6 s, load B put on (with a nudge, as a hand does), 10 s.
           falls, final gear, re-probes / downshifts, time until the shaking stops,
           jitter over the last 3 s
    python scripts/rl/load_matrix.py static|swap
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chatter_twin as C

KINDS = ("normal", "heavy", "v4 (off)", "v5 (on)")
NAMES = {"normal": "factory Normal", "heavy": "factory Weight_M", "v4 (off)": "v4 release", "v5 (on)": "v5 new"}
LOADS = tuple(np.round(np.arange(0.0, 4.01, 0.5), 1))
SWAP_SET = (0.0, 1.0, 2.0, 3.0, 4.0)


class Osc:
    """8-16 Hz chatter, same filters as v4_core (deg/s)."""
    def __init__(self):
        self.hp = self.bp = self.en = 0.0

    def __call__(self, g):
        self.hp += 0.200849 * (g - self.hp)
        self.bp += 0.334511 * ((g - self.hp) - self.bp)
        self.en += 0.016393 * (self.bp * self.bp - self.en)
        return np.sqrt(max(self.en, 0.0)) / 16.4


def job_static(a):
    kind, kg, sd = a
    r = C.FA.run(kind, kg, 0.0, 0.0, sd, 0.0)
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else (
        (r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return a, dict(pen=pen, ok=r["ok"], osc=r["osc"], settle=r["settle"])


def job_swap(a):
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    kind, ka, kb, sd = a
    env = C._env(sd, ka)
    c = env.core
    ctl = C.FA.Ctl(kind, ka)
    lib = C.LIBS.get(kind)
    for _ in range(int(6.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell_before=True, fell=True)
    f_a = float(getattr(getattr(ctl, "sw", None), "f", np.nan))
    r0 = lib.v4_rearms() if lib else 0
    d0 = lib.v4_downs() if lib else 0
    env._apply_payload(kb)
    T.set_tilt(env, 3.0, 60.0)
    osc, rows, oscs, last_shake, fs = Osc(), [], [], 0.0, []
    for n in range(int(10.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell_before=False, fell=True)
        g = float(c._gyro_lsb) / 16.4
        o = osc(float(c._gyro_lsb))
        if abs(g) > 15.0:
            last_shake = n / FS
        fs.append(float(getattr(getattr(ctl, "sw", None), "f", np.nan)))
        if n >= int(7.0 * FS):
            rows.append((float(c._angle_filt), g))
            oscs.append(o)
    r = np.array(rows)
    return a, dict(fell_before=False, fell=False, f_a=f_a, f_end=fs[-1], f_max=float(np.nanmax(fs)) if lib else np.nan,
                   reprobes=(lib.v4_rearms() - r0) if lib else 0, downs=(lib.v4_downs() - d0) if lib else 0,
                   shake=last_shake, th_sd=float(r[:, 0].std()),
                   gy=float(np.sqrt(np.mean((r[:, 1] - r[:, 1].mean()) ** 2))), osc=float(np.mean(oscs)))


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "static":
        jobs = [(k, kg, sd) for k in KINDS for kg in LOADS for sd in (0, 1, 2)]
        with Pool(16, initializer=C._init) as pool:
            res = pool.map(job_static, jobs)
        print("benchmark course with a fixed load; 3 seeds; cell = penalty (lower better) / falls / 8-16 Hz chatter")
        print("  %-20s" % "kg" + "".join("%13.1f" % kg for kg in LOADS))
        for k in KINDS:
            cells = []
            for kg in LOADS:
                rs = [r for (kk, g, sd), r in res if kk == k and g == kg]
                cells.append("%5.1f/%d/%4.1f" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs),
                                                 np.mean([r["osc"] for r in rs])))
            print("  %-20s" % NAMES[k] + "".join("%13s" % c for c in cells))
    else:
        pairs = [(a_, b_) for a_ in SWAP_SET for b_ in SWAP_SET if a_ != b_]
        jobs = [(k, a_, b_, sd) for k in KINDS for a_, b_ in pairs for sd in (0, 1, 2)]
        with Pool(16, initializer=C._init) as pool:
            res = pool.map(job_swap, jobs)
        print("load swapped A -> B on a standing car (with a nudge); 3 seeds")
        print("cell = falls / shaking ends after s / tilt sd deg (last 3 s) / gear before->after (v4, v5c)")
        hdr = "  %-11s" % "A->B" + "".join("%-30s" % NAMES[k] for k in KINDS)
        print(hdr)
        for a_, b_ in pairs:
            cells = []
            for k in KINDS:
                rs = [r for (kk, x, y, sd), r in res if kk == k and x == a_ and y == b_]
                ok = [r for r in rs if not r["fell"]]
                if not ok:
                    cells.append("FELL 3/3")
                    continue
                gear = ""
                if k in ("v4 (off)", "v5 (on)"):
                    gear = " %.1f->%s" % (np.mean([r["f_a"] for r in ok]), "/".join("%.1f" % r["f_end"] for r in ok))
                cells.append("%d / %.1fs / %.2f%s" % (sum(r["fell"] for r in rs), np.mean([r["shake"] for r in ok]),
                                                    np.mean([r["th_sd"] for r in ok]), gear))
            print("  %3.0f -> %-3.0f " % (a_, b_) + "".join("%-30s" % c for c in cells))
        for k in ("v4 (off)", "v5 (on)"):
            rs = [r for (kk, x, y, sd), r in res if kk == k and not r["fell"]]
            print("  %s: re-probes per swap %.2f, downshifts per swap %.2f, falls %d/%d" % (
                NAMES[k], np.mean([r["reprobes"] for r in rs]), np.mean([r["downs"] for r in rs]),
                sum(r["fell"] for (kk, x, y, sd), r in res if kk == k), len(pairs) * 3))
