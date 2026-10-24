# -*- coding: utf-8 -*-
"""Fall -> lie -> get up, with the firmware's rec_core.c (via rec_core.dll), 2026-10-21.
Geometry from the STEP model (collision hull; the chassis rests at ~79 deg).

Each run: balance 3.5 s, then a shove the controller cannot save (tilt 50 deg, 150 deg/s
toward the fall).  The motors cut at 45 deg, the car falls and lies on its chassis, and
the firmware logic does the rest (wait until still, push / coast / push, hand over to
v4 + recovery, retries, lock).  8 s.
  ok      = upright (|tilt| < 3 deg) and stopped at the end
  drive   = seconds with the motors driving while |tilt| > 45 deg (wheels turning on a
            fallen car) -- the safety number
    python scripts/rl/getup_twin.py [sweep]
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rec_twin as R

FW = (1, 40, 30, 120, 30.0)          # firmware defaults: on, push1 0.2 s, coast 0.15 s, push2 <= 0.6 s, hand 30


def _init(sign=1.0):
    R._init(sign)
    import recovery_bound40 as RB40
    RB40.CUT = np.deg2rad(179.0)      # the twin's own termination: none (we want to see it lie)
    from balance_bot import params as P
    for name in dir(P):
        cls = getattr(P, name)
        if isinstance(cls, type) and "bottoms_out" in cls.__dict__:
            cls.bottoms_out = lambda self, pitch: False


def run(sr, kg, vb, sg, sd):
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    R.FORCE[0] = 0
    R.VAR["fw"] = (1, 20.0, 2.0, 30.0, 4.0, 1, sr)
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=16, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: kg
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    c = env.core
    if hasattr(c, "battery_v"):
        c.battery_v = vb
    ctl = R.RecCtl("fw")
    for _ in range(int(3.5 * FS)):
        env.step(ctl(env))
    ref = np.degrees(float(c.state[ITH]))
    T.set_tilt(env, sg * 50.0, sg * 150.0)
    drive, peak, up, tail_v, tail_th = 0.0, 0.0, None, [], []
    lay = False
    for n in range(int(8.0 * FS)):
        env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        el, er = c._enc_last
        peak = max(peak, abs(th))
        if abs(th) > 60:
            lay = True
        if abs(th) > 45 and not c.fw.st.motors_off and (abs(c.fw.st.ccr_l) > 0 or abs(c.fw.st.ccr_r) > 0):
            drive += 1.0 / FS
        if lay and up is None and abs(th) < 3:
            up = n / FS
        if n >= int(7.5 * FS):
            tail_v.append(abs(0.5 * (el + er)))
            tail_th.append(abs(th))
    ok = max(tail_th) < 3 and max(tail_v) < 4
    return dict(ok=ok, lay=lay, up=up, drive=drive, peak=peak, att=R.LIB.rec_sr_attempts(),
                sr_ok=R.LIB.rec_sr_ok(), locked=R.LIB.rec_locked(), end_th=float(np.mean(tail_th)))


def run_start(sr, kg, vb, sg, sd):
    """Car already lying when the 2nd KEY1 is pressed: fresh firmware state (REC_Stopped
    ran while the motors were off), then the motors are switched on."""
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=16, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: kg
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    c = env.core
    if hasattr(c, "battery_v"):
        c.battery_v = vb
    ref = np.degrees(float(c.state[ITH]))
    T.set_tilt(env, sg * 20.0, 0.0, seed_filter=False)
    R.FORCE[0] = 1
    for _ in range(int(2.0 * FS)):              # Stop_Flag = 1: motors off, the car lies down
        env.step(np.array([0.0, 0.0]))
    rest = np.degrees(float(c.state[ITH])) - ref
    R.FORCE[0] = 0
    R.VAR["fw"] = (1, 20.0, 2.0, 30.0, 4.0, 1, sr)
    ctl = R.RecCtl("fw")                        # V4_Reset / REC_Init at the 2nd KEY1 ...
    R.LIB.rec_stopped()                         # ... after REC_Stopped ran while stopped
    drive, up, tail_v, tail_th = 0.0, None, [], []
    for n in range(int(8.0 * FS)):
        env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        el, er = c._enc_last
        if abs(th) > 45 and not c.fw.st.motors_off and (abs(c.fw.st.ccr_l) > 0 or abs(c.fw.st.ccr_r) > 0):
            drive += 1.0 / FS
        if up is None and abs(th) < 3:
            up = n / FS
        if n >= int(7.5 * FS):
            tail_v.append(abs(0.5 * (el + er)))
            tail_th.append(abs(th))
    ok = max(tail_th) < 3 and max(tail_v) < 4
    return dict(ok=ok, lay=abs(rest) > 60, up=up, drive=drive, peak=0.0, att=R.LIB.rec_sr_attempts(),
                sr_ok=R.LIB.rec_sr_ok(), locked=R.LIB.rec_locked(), end_th=float(np.mean(tail_th)))


def _job(a):
    if a[0] == "start":
        return a, run_start(*a[1:])
    return a, run(*a)


def table(res, keys, label):
    for k in keys:
        rs = [r for a, r in res if a[:3] == k[:3] and a[0] == k[0]]
        ok = [r for r in rs if r["ok"]]
        print("  %-40s lay %d/%d | up+stopped %d/%d | t_up %s | attempts %.1f | locked %d | drive on a fallen car %.2f s (max %.2f)" % (
            label(k), sum(r["lay"] for r in rs), len(rs), len(ok), len(rs),
            ("%.2f s" % np.mean([r["up"] for r in ok if r["up"] is not None])) if ok else "  -  ",
            np.mean([r["att"] for r in rs]), sum(r["locked"] for r in rs),
            np.mean([r["drive"] for r in rs]), max(r["drive"] for r in rs)))


if __name__ == "__main__":
    import tilt_envelope as T
    T._path()
    s = T.selftest()
    sweep = len(sys.argv) > 1 and sys.argv[1] == "sweep"
    if len(sys.argv) > 1 and sys.argv[1] == "start":
        cases = [(FW, 0.0, 12.0), (FW, 0.0, 10.5), (FW, 1.0, 12.0), ((0, 40, 30, 120, 30.0), 0.0, 12.0)]
        jobs = [("start", sr, kg, vb, sg, sd) for sr, kg, vb in cases for sg in (1, -1) for sd in (0, 1, 2, 3)]
        with Pool(16, initializer=_init, initargs=(s,)) as pool:
            res = pool.map(_job, jobs)
        print("already lying when started (2nd KEY1); 8 runs per row (4 nose-down, 4 tail-down)")
        for sr, kg, vb in cases:
            rs = [r for a, r in res if a[1] == sr and a[2] == kg and a[3] == vb]
            ok = [r for r in rs if r["ok"]]
            print("  get-up %s | %g kg %4.1f V | lying at start %d/8 | up+stopped %d/8 | t_up %s | attempts %.1f | locked %d | drive on a fallen car %.2f s (max %.2f)" % (
                "ON " if sr[0] else "OFF", kg, vb, sum(r["lay"] for r in rs), len(ok),
                ("%.2f s" % np.mean([r["up"] for r in ok if r["up"] is not None])) if ok else "  -  ",
                np.mean([r["att"] for r in rs]), sum(r["locked"] for r in rs),
                np.mean([r["drive"] for r in rs]), max(r["drive"] for r in rs)))
        sys.exit(0)
    if sweep:
        srs = [(1, p1, co, 120, 30.0) for p1 in (30, 40, 50) for co in (20, 30, 40)]
        cases = [(sr, 0.0, vb) for sr in srs for vb in (12.0, 10.5)]
    else:
        cases = [(FW, 0.0, 12.0), (FW, 0.0, 10.5), (FW, 1.0, 12.0),
                 ((0, 40, 30, 120, 30.0), 0.0, 12.0)]
    jobs = [(sr, kg, vb, sg, sd) for sr, kg, vb in cases for sg in (1, -1) for sd in (0, 1, 2, 3)]
    with Pool(16, initializer=_init, initargs=(s,)) as pool:
        res = pool.map(_job, jobs)
    print("fall -> lie -> get up; 8 runs per row (4 nose-down, 4 tail-down)")
    for sr, kg, vb in cases:
        rs = [r for a, r in res if a[0] == sr and a[1] == kg and a[2] == vb]
        ok = [r for r in rs if r["ok"]]
        print("  get-up %s push1 %3d coast %3d | %g kg %4.1f V | lay %d/8 | up+stopped %d/8 | t_up %s | attempts %.1f | locked %d | drive on a fallen car %.2f s (max %.2f)" % (
            "ON " if sr[0] else "OFF", sr[1], sr[2], kg, vb, sum(r["lay"] for r in rs), len(ok),
            ("%.2f s" % np.mean([r["up"] for r in ok if r["up"] is not None])) if ok else "  -  ",
            np.mean([r["att"] for r in rs]), sum(r["locked"] for r in rs),
            np.mean([r["drive"] for r in rs]), max(r["drive"] for r in rs)))
