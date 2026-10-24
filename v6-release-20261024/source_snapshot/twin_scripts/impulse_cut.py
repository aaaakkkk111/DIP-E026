# -*- coding: utf-8 -*-
"""Does a larger motor cut-off angle let the car catch faster / larger falls? (2026-10-21)

Firmware logic = v4 + recovery + get-up (rec_core.dll), cut-off CUT in {45, 60, 75} deg
(STEP model: the body touches the ground at 78.5 deg).  Start: tilt TH0 already falling
at W0 deg/s (filter seeded), 8 s.
  caught = the body never touched the ground (peak < 78 deg) AND upright+stopped at the end
  final  = upright+stopped at the end (may have fallen and got up)
    python scripts/rl/impulse_cut.py
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import impulse_twin as I
import rec_twin as R

CUTS = (45.0, 60.0, 75.0)
THETAS = (10, 15, 20, 25, 30, 35, 40)
OMEGAS = (0, 100, 200, 300, 400)


def run(cut, th0, w0, sg, sd):
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    R.FORCE[0] = 0
    R.CUT = cut
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=14, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: 0.0
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    c = env.core
    R.VAR["imp"] = (1, 20.0, 2.0, 30.0, 4.0, 1, I.SR)
    ctl = R.RecCtl("imp")
    for _ in range(int(3.5 * FS)):
        env.step(ctl(env))
    ref = np.degrees(float(c.state[ITH]))
    T.set_tilt(env, sg * th0, sg * w0)
    peak, tail_v, tail_th = 0.0, [], []
    for n in range(int(8.0 * FS)):
        env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        el, er = c._enc_last
        peak = max(peak, abs(th))
        if n >= int(7.5 * FS):
            tail_v.append(abs(0.5 * (el + er)))
            tail_th.append(abs(th))
    final = max(tail_th) < 3 and max(tail_v) < 4
    return dict(caught=(peak < 78.0 and final), final=final, peak=peak)


def _job(a):
    return a, run(*a)


if __name__ == "__main__":
    import tilt_envelope as T
    T._path()
    s = T.selftest()
    jobs = [(cut, th, w, sg, sd) for cut in CUTS for th in THETAS for w in OMEGAS
            for sg in (1, -1) for sd in (0, 1)]
    with Pool(16, initializer=I._init, initargs=(s,)) as pool:
        res = pool.map(_job, jobs)
    acc = {}
    for (cut, th, w, sg, sd), r in res:
        acc.setdefault((cut, th, w), []).append(r)
    print("empty car, 12 V; cell = caught without touching the ground /4 (fwd+back x 2 seeds), mean peak tilt")
    for cut in CUTS:
        print("\n== cut-off %.0f deg" % cut)
        print("   w0 deg/s " + "".join("  th0=%-8d" % th for th in THETAS))
        for w in OMEGAS:
            cells = []
            for th in THETAS:
                rs = acc[(cut, th, w)]
                cells.append("%d/4 %4.0f" % (sum(r["caught"] for r in rs), np.mean([r["peak"] for r in rs])))
            print("   %5d    " % w + "".join("  %-12s" % c for c in cells))
        best = []
        for w in OMEGAS:
            ok = [th for th in THETAS if all(r["caught"] for r in acc[(cut, th, w)])]
            best.append("%d deg/s: %s" % (w, ("%d deg" % max(ok)) if ok else "<10 deg"))
        print("   largest angle caught 4/4: " + " | ".join(best))
        print("   final upright (incl. get-up): %d/%d" % (
            sum(r["final"] for (c_, _t, _w), v in acc.items() if c_ == cut for r in v), len(THETAS) * len(OMEGAS) * 4))
