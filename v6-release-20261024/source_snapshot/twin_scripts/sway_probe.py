# -*- coding: utf-8 -*-
"""Real car 10-24: with a load the body sways VISIBLY back and forth a few times a second, the
swing grows, and the car falls -- even on a good gear.  Can the twin show it if the load sits
higher than the 0.105 m above the axle it assumed?  Fixed gear f (v4 ladder), load kg at height
h, 10 s standing after a small nudge: low-frequency (< 6 Hz) tilt amplitude per 2 s window.
    python scripts/rl/sway_probe.py"""
import os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import load_place as LP
HS = (0.105, 0.15, 0.20, 0.25)
KGS = (0.5, 1.0, 2.0)
FS_ = (0.0, 0.2, 0.4, 0.7, 1.0)


def job(a):
    h, kg, fv, sd = a
    import switch_v3 as SV3
    import tilt_envelope as T
    from balance_bot.rl import payload as PL
    from balance_bot.rl.e2e_env import FS
    PL.PAYLOAD_H = h
    orig = PL.with_payload
    import balance_bot.rl.e2e_env as E
    E.with_payload = lambda robot, m, h_p=h: orig(robot, m, h_p)
    env = LP._env(sd)
    env._apply_payload(kg)
    c = env.core
    pid = SV3.SwitchV3().pid
    gn, gh = dict(SV3.G_N), dict(SV3.G_H)
    pid.g.update({k: gn[k] + (gh[k] - gn[k]) * fv for k in ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki", "turn_kp", "turn_kd")})
    for _ in range(int(1.0 * FS)):
        _o, _r, d, _t, _ = env.step(pid(env))
        if d:
            return a, dict(fell=True, t=0.0, amp=[])
    T.set_tilt(env, 2.0, 30.0)
    lp, amp, buf = 0.0, [], []
    for n in range(int(10.0 * FS)):
        _o, _r, d, _t, _ = env.step(pid(env))
        if d:
            return a, dict(fell=True, t=n / FS, amp=amp)
        th = float(c._angle_filt)
        lp += 0.17 * (th - lp)            # ~6 Hz low-pass: the visible sway
        buf.append(lp)
        if len(buf) == 400:
            amp.append(0.5 * (max(buf) - min(buf)))
            buf = []
    return a, dict(fell=False, amp=amp)


if __name__ == "__main__":
    import score_release_v4 as S
    jobs = [(h, kg, fv, 0) for h in HS for kg in KGS for fv in FS_]
    with Pool(16, initializer=S._init) as pool:
        res = pool.map(job, jobs)
    print("low-frequency sway amplitude (deg, half peak-to-peak) per 2 s window over 10 s, or FELL at t")
    for h in HS:
        print("== load height %.3f m above the axle" % h)
        for kg in KGS:
            cells = []
            for fv in FS_:
                r = [r for a, r in res if a[0] == h and a[1] == kg and a[2] == fv][0]
                s = " ".join("%.1f" % x for x in r["amp"])
                cells.append(("FELL %.1fs [%s]" % (r["t"], s)) if r["fell"] else "[%s]" % s)
            print("  %.1f kg  " % kg + " | ".join("f=%.1f %s" % (fv, c) for fv, c in zip(FS_, cells)))
