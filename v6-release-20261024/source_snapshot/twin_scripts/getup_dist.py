# -*- coding: utf-8 -*-
"""Get-up: how much floor does it need, and can a flywheel start shorten it? (2026-10-21)

rec_core.c with sr_rev (rec_core_fly.dll): sr_rev = 1 makes the first push go AWAY from
the lean.  Lying on the chassis only ~7 % of the weight is on the wheels, so they mostly
spin in place; reversing them hard then hands the stored rotor momentum to the body.
Same fall as getup_twin.run (shove past the point of no return, the car lies at ~79 deg).
Distances from the TRUE chassis position (the encoders lie when the wheels slip):
  peak  = largest distance from where the car lay down, during the get-up
  final = distance from there after 8 s
    python scripts/rl/getup_dist.py
"""
import ctypes
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import getup_twin as G
import rec_twin as R

VARS = {   # (rev, push1, coast, push2) in 5 ms ticks
    "now: .20/.15/<=.60":   (0, 40, 30, 120),
    "push2 <=.40":          (0, 40, 30, 80),
    "push2 <=.30":          (0, 40, 30, 60),
    "push2 <=.20":          (0, 40, 30, 40),
    ".25/.10/<=.60":        (0, 50, 20, 120),
    ".25/.10/<=.30":        (0, 50, 20, 60),
}


def _init(sign=1.0):
    G._init(sign)
    lib = ctypes.CDLL(os.path.join(HERE, "rec_core_fly.dll"))
    lib.rec_set.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.rec_exit.argtypes = [ctypes.c_float] * 3
    lib.rec_step.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    lib.rec_sr.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float]
    lib.rec_sr_rev.argtypes = [ctypes.c_int]
    R.LIB = lib


def run(var, vb, sg, sd):
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    rev, p1, co, p2 = VARS[var]
    R.FORCE[0] = 0
    R.VAR["fw"] = (1, 20.0, 2.0, 30.0, 4.0, 1, (1, p1, co, p2, 30.0))
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=16, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: 0.0
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    c = env.core
    if hasattr(c, "battery_v"):
        c.battery_v = vb
    ctl = R.RecCtl("fw")
    R.LIB.rec_sr_rev(rev)
    for _ in range(int(3.5 * FS)):
        env.step(ctl(env))
    ref = np.degrees(float(c.state[ITH]))
    T.set_tilt(env, sg * 50.0, sg * 150.0)
    x0, peak, up, tail_v, tail_th = None, 0.0, None, [], []
    for n in range(int(8.0 * FS)):
        env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        x = float(c.data.qpos[0])
        el, er = c._enc_last
        if x0 is None and abs(th) > 60:
            x0 = x
        if x0 is not None:
            peak = max(peak, abs(x - x0))
            if up is None and abs(th) < 3:
                up = n / FS
        if n >= int(7.5 * FS):
            tail_v.append(abs(0.5 * (el + er)))
            tail_th.append(abs(th))
    ok = max(tail_th) < 3 and max(tail_v) < 4
    final = abs(float(c.data.qpos[0]) - x0) if x0 is not None else float("nan")
    return dict(ok=ok, peak=peak, final=final, up=up, att=R.LIB.rec_sr_attempts())


def _job(a):
    return a, run(*a)


if __name__ == "__main__":
    import tilt_envelope as T
    T._path()
    s = T.selftest()
    jobs = [(v, vb, sg, sd) for v in VARS for vb in (12.0, 10.5) for sg in (1, -1) for sd in (0, 1, 2, 3)]
    with Pool(16, initializer=_init, initargs=(s,)) as pool:
        res = pool.map(_job, jobs)
    print("fall -> lie -> get up, empty car; 8 runs per row (4 nose-down, 4 tail-down)")
    print("  distances in cm from where the car lay down (true chassis position)")
    for v in VARS:
        for vb in (12.0, 10.5):
            rs = [r for a, r in res if a[0] == v and a[1] == vb]
            ok = [r for r in rs if r["ok"]]
            print("  %-20s %4.1f V | up+stopped %d/8 | attempts %.1f | peak %5.1f cm (max %5.1f) | final %5.1f cm | t_up %s" % (
                v, vb, len(ok), np.mean([r["att"] for r in rs]),
                100 * np.mean([r["peak"] for r in rs]), 100 * max(r["peak"] for r in rs),
                100 * np.mean([r["final"] for r in rs]),
                ("%.2f s" % np.mean([r["up"] for r in ok if r["up"] is not None])) if ok else "  -  "))
