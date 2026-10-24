# -*- coding: utf-8 -*-
"""High CoM (0.25 / 0.30 m): is anything stable?  Fixed gains swept beyond the stock Weight_M set
(2026-10-24).  Load put on a standing car (Weight_M gains while placing), +3 deg nudge, 15 s.
cell: fell / sway deg (last 3 s, < 6 Hz) / share of ticks at the PWM limit.
    python scripts/rl/hgain_sweep.py"""
import os, sys, itertools
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import v52_score as V
HS = (0.20, 0.25, 0.30)
KGS = (1.0, 2.0, 3.0)
BKP = (192.0, 260.0, 340.0)
BKD = (1.0, 1.5, 2.2)
VEL = (0.5, 1.0, 1.5)          # x Weight_M velocity kp/ki


def job(a):
    h, kg, bkp, bkd, vs, sd = a
    import switch_v3 as SV3
    import tilt_envelope as T
    import chatter_twin as C
    from balance_bot.rl.e2e_env import FS
    V.set_height(h)
    env = C._env(sd); c = env.core
    pid = SV3.SwitchV3().pid
    gh = dict(SV3.G_H)
    pid.g.update({k: gh[k] for k in ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki", "turn_kp", "turn_kd")})
    for _ in range(int(3.0 * FS)):
        env.step(pid(env))
    pid.g.update(balance_kp=bkp, balance_kd=bkd, velocity_kp=gh["velocity_kp"] * vs, velocity_ki=gh["velocity_ki"] * vs)
    env._apply_payload(kg)
    T.set_tilt(env, 3.0, 60.0)
    lp, buf, sat = 0.0, [], 0
    for n in range(int(15.0 * FS)):
        act = pid(env)
        if np.max(np.abs(np.asarray(act, dtype=float))) >= 0.99:
            sat += 1
        _o, _r, d, _t, _ = env.step(act)
        if d:
            return a, dict(fell=True, t=n / FS, sat=sat / max(n, 1))
        lp += 0.17 * (float(c._angle_filt) - lp)
        if n >= int(12.0 * FS):
            buf.append(lp)
    return a, dict(fell=False, sway=0.5 * (max(buf) - min(buf)), sat=sat / (15.0 * FS))


if __name__ == "__main__":
    gains = list(itertools.product(BKP, BKD, VEL))
    jobs = [(h, kg, p, d, v, sd) for h in HS for kg in KGS for p, d, v in gains for sd in (0, 1)]
    with Pool(16, initializer=V._init) as pool:
        res = pool.map(job, jobs)
    for h in HS:
        print("== load CoM %.2f m" % h)
        print("  %-28s" % "bkp/bkd/vel x" + "".join("%22s" % ("%g kg" % kg) for kg in KGS))
        for p, d, v in gains:
            cells = []
            for kg in KGS:
                rs = [r for (hh, g, pp, dd, vv, sd), r in res if hh == h and g == kg and (pp, dd, vv) == (p, d, v)]
                nf = sum(r["fell"] for r in rs); ok = [r for r in rs if not r["fell"]]
                cells.append(("fell %d/2" % nf) if not ok else "%d / %.2f / %.0f%%" % (nf, np.mean([r["sway"] for r in ok]), 100 * np.mean([r["sat"] for r in ok])))
            tag = "  <- Weight_M" if (p, d, v) == (192.0, 1.5, 1.0) else ""
            print("  %-28s" % ("%g / %g / %g" % (p, d, v)) + "".join("%22s" % c for c in cells) + tag)
