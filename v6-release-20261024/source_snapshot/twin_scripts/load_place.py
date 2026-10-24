# -*- coding: utf-8 -*-
"""Putting a load on a standing car: what does v4's gear logic do? (2026-10-21)

Two parts:
  place : v4 (SwitchV4 == firmware v4_core logic) stands 4 s empty (latches light), then
          a load of KG kg is put on at once (optionally with a downward/forward nudge),
          10 s more.  Logged: gear f over time, number of upshifts (re-probes), final f,
          tilt / gyro jitter over the last 3 s.
  fixed : the same loads with f held FIXED at a ladder of values -- which f is actually
          best for each load (jitter, 8-16 Hz chatter, tilt spread)?
    python scripts/rl/load_place.py place|fixed
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for q in (ROOT, HERE):
    if q not in sys.path:
        sys.path.insert(0, q)

LOADS = (0.0, 0.3, 0.5, 1.0, 1.5, 2.0)
F_LADDER = (0.0, 0.1, 0.2, 0.3, 0.4, 0.6, 1.0)


def _env(sd):
    import goto_hold as GH
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=20, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: 0.0
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    return env


def _jitter(rows):
    a = np.array(rows)
    th, gy = a[:, 0], a[:, 1]
    return float(th.std()), float(np.sqrt(np.mean((gy - gy.mean()) ** 2)))


def place(kg, nudge, sd):
    import switch_v4 as V4
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    from balance_bot.dynamics import ITH
    env = _env(sd)
    c = env.core
    ctl = V4.SwitchV4()
    for _ in range(int(4.0 * FS)):
        env.step(ctl(env))
    f_before = float(ctl.f)
    env._apply_payload(kg)
    if nudge:
        T.set_tilt(env, 3.0, 60.0)          # the hand that puts it down tips the car a little
    fs, rows, falls = [], [], False
    rearm0 = getattr(ctl, "rearms", 0)
    for n in range(int(10.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            falls = True
            break
        fs.append(float(ctl.f))
        if n >= int(7.0 * FS):
            rows.append((float(c._angle_filt), float(c._gyro_lsb) / 16.4))
    th_sd, gy = _jitter(rows) if rows else (float("nan"), float("nan"))
    f = np.array(fs) if fs else np.array([np.nan])
    t_hi = float(np.mean(f >= 0.99)) * 10.0     # seconds spent at the stiffest gear
    return dict(f0=f_before, f_end=float(f[-1]), f_max=float(np.nanmax(f)), t_hi=t_hi,
                rearms=getattr(ctl, "rearms", 0) - rearm0, th_sd=th_sd, gy=gy, fell=falls)


def fixed(kg, fval, sd):
    import switch_v4 as V4
    from balance_bot.rl.e2e_env import FS
    env = _env(sd)
    env._apply_payload(kg)
    c = env.core
    sw = V4.SwitchV4()
    pid = sw.pid
    import switch_v3 as SV3
    gn, gh = dict(SV3.G_N), dict(SV3.G_H)       # f=0 Normal, f=1 Weight_M (same as v4_core GN/GH)
    gains = {k: gn[k] + (gh[k] - gn[k]) * fval for k in ("balance_kp", "balance_kd", "velocity_kp",
                                                        "velocity_ki", "turn_kp", "turn_kd")}
    pid.g.update(gains)
    rows = []
    hp = bp = en = 0.0
    oscs = []
    for n in range(int(8.0 * FS)):
        _o, _r, d, _t, _ = env.step(pid(env))
        if d:
            return dict(fell=True)
        g = float(c._gyro_lsb)
        hp += 0.200849 * (g - hp)
        bp += 0.334511 * ((g - hp) - bp)
        en += 0.016393 * (bp * bp - en)
        if n >= int(3.0 * FS):
            rows.append((float(c._angle_filt), g / 16.4))
            oscs.append(np.sqrt(max(en, 0.0)) / 16.4)
    th_sd, gy = _jitter(rows)
    return dict(fell=False, th_sd=th_sd, gy=gy, osc=float(np.mean(oscs)))


def _job(a):
    kind = a[0]
    return a, (place(*a[1:]) if kind == "place" else fixed(*a[1:]))


if __name__ == "__main__":
    import score_release_v4 as S
    what = sys.argv[1] if len(sys.argv) > 1 else "place"
    if what == "place":
        jobs = [("place", kg, nd, sd) for kg in LOADS for nd in (0, 1) for sd in (0, 1, 2)]
    else:
        jobs = [("fixed", kg, fv, sd) for kg in LOADS for fv in F_LADDER for sd in (0, 1)]
    with Pool(16, initializer=S._init) as pool:
        res = pool.map(_job, jobs)
    if what == "place":
        print("load put on a standing car (v4 logic); 3 seeds each; f: 0 light .. 0.40 heavy .. 1 stiffest")
        for kg in LOADS:
            for nd in (0, 1):
                rs = [r for a, r in res if a[1] == kg and a[2] == nd]
                print("  %4.1f kg %-9s | f before %.2f | f max %.2f | f end %s | s at f=1 %.1f | re-probes %.1f | tilt sd %.2f deg | gyro %.1f deg/s | falls %d" % (
                    kg, "nudge" if nd else "no nudge", np.mean([r["f0"] for r in rs]), np.mean([r["f_max"] for r in rs]),
                    "/".join("%.2f" % r["f_end"] for r in rs), np.mean([r["t_hi"] for r in rs]),
                    np.mean([r["rearms"] for r in rs]), np.nanmean([r["th_sd"] for r in rs]),
                    np.nanmean([r["gy"] for r in rs]), sum(r["fell"] for r in rs)))
    else:
        print("load held, gear f FIXED; 2 seeds; jitter over 5 s standing (lower = calmer)")
        print("   kg   " + "".join("     f=%-12.1f" % f for f in F_LADDER))
        for kg in LOADS:
            cells = []
            for fv in F_LADDER:
                rs = [r for a, r in res if a[1] == kg and a[2] == fv]
                if any(r["fell"] for r in rs):
                    cells.append("FELL")
                else:
                    cells.append("%.2f/%.1f/%.1f" % (np.mean([r["th_sd"] for r in rs]), np.mean([r["gy"] for r in rs]),
                                                     np.mean([r["osc"] for r in rs])))
            print("  %4.1f  " % kg + "".join("  %-16s" % c for c in cells))
        print("  cell = tilt sd deg / gyro rms deg/s / 8-16 Hz chatter deg/s")
