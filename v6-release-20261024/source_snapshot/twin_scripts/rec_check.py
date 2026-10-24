# -*- coding: utf-8 -*-
"""Mode-27 recovery: regression + lock checks (2026-10-20), same rec_core.dll as rec_twin.

  bench : the release benchmark (score_release_v4 rules, 5 cases x 3 seeds) with
          (a) the release v4, cut-off 40          -- must reproduce 31.83
          (b) cut-off 45 + rec_core               -- should be unchanged; recovery entries counted
  lying : car resting on its chassis (~44.6 deg, motors held off 1 s), then motors on,
          0 kg and 1 kg -> got up?  lock fired?  (chassis-ground fall test disabled)
    python scripts/rl/rec_check.py bench|lying
"""
import ctypes
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mc_twin_test as M     # CASES / SEEDS / weights of the benchmark
import rec_twin as R

STATE = {"rec": False}
FA = None


def _init_bench():
    global FA
    import score_release_v4 as S
    S._init()
    import factory_vs_auto as FA_
    FA = FA_
    R.LIB = ctypes.CDLL(os.path.join(HERE, "rec_core.dll"))
    R.LIB.rec_set.argtypes = [ctypes.c_int, ctypes.c_float]
    R.LIB.rec_step.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    R.LIB.rec_sr.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float]
    from balance_bot.firmware import controllers as C
    for name in dir(C):
        cls = getattr(C, name)
        if isinstance(cls, type) and "turn_off" in cls.__dict__:
            def turn_off(self, angle_deg, battery_v, stop_flag=False, _o=cls.turn_off):
                if not STATE["rec"]:
                    return _o(self, angle_deg, battery_v, stop_flag)
                if R.FORCE[0] == 2:
                    return bool(battery_v < 9.6 or stop_flag)
                return bool(R.FORCE[0] == 1 or abs(angle_deg) > 45.0 or battery_v < 9.6 or stop_flag)
            cls.turn_off = turn_off
    base = FA.Ctl

    class Ctl3(base):
        def __init__(self, kind, kg):
            base.__init__(self, kind, kg)
            if STATE["rec"]:
                R.LIB.rec_init()
                R.LIB.rec_set(1, 20.0)
                R.LIB.rec_sr(1, 40, 30, 120, 30.0)     # firmware defaults: get-up on
                R.FORCE[0] = 0
                self._u = ctypes.c_int(0)

        def __call__(self, env):
            if not STATE["rec"]:
                return base.__call__(self, env)
            tw = env.core
            pid = self.sw.pid
            el, er = tw._enc_last
            r = R.LIB.rec_step(float(tw._angle_filt) - float(pid.g.get("mid_angle_deg", 0.0)),
                               float(tw._gyro_lsb), int(el), int(er), ctypes.byref(self._u))
            R.FORCE[0] = 2 if r == 2 else (1 if R.LIB.rec_locked() else 0)
            a = base.__call__(self, env)     # v4 keeps deciding
            if r:
                pid.enc_int = 0.0              # speed integral held at 0 while recovering
            if r == 2:
                return np.array([self._u.value / env.u_span, 0.0])
            return a
    FA.Ctl = Ctl3


def _job_bench(j):
    rec, ci, sd = j
    STATE["rec"] = bool(rec)
    _cn, kg, imp, _w = M.CASES[ci]
    r = FA.run("v4py", kg, imp, 0.0, sd, 0.0)
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else (
        (r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return (rec, ci, sd, pen, r["ok"], R.LIB.rec_entries() if rec else 0,
            R.LIB.rec_locks() if rec else 0)


def bench():
    jobs = [(rec, ci, sd) for rec in (0, 1) for ci in range(len(M.CASES)) for sd in M.SEEDS]
    acc = {}
    with Pool(16, initializer=_init_bench) as pool:
        for r in pool.imap_unordered(_job_bench, jobs):
            acc.setdefault(r[0], []).append(r)
    print("release benchmark (lower is better)")
    for rec in (0, 1):
        rs = acc[rec]
        per = {}
        for r in rs:
            per.setdefault(r[1], []).append(r[3])
        tot = sum(M.CASES[ci][3] * np.mean(v) for ci, v in per.items())
        print("  %-22s total %6.2f  falls %d  recovery entries %d  locks %d  per case %s" % (
            "release v4 (cut 40)" if rec == 0 else "cut 45 + recovery", tot,
            sum(1 for r in rs if not r[4]), sum(r[5] for r in rs), sum(r[6] for r in rs),
            " ".join("%.2f" % np.mean(per[ci]) for ci in sorted(per))))


def _init_lying(sign=1.0):
    R._init(sign)
    from balance_bot import params as P
    for name in dir(P):
        cls = getattr(P, name)
        if isinstance(cls, type) and "bottoms_out" in cls.__dict__:
            cls.bottoms_out = lambda self, pitch: False
    import recovery_bound40 as RB40
    RB40.CUT = np.deg2rad(80.0)       # only the firmware cut-off (45) and the lock stop the motors


def _job_lying(j):
    var, kg, sd = j
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    R.FORCE[0] = 0
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=12, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: kg
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    c = env.core
    ctl = R.RecCtl(var)
    for _ in range(int(3.5 * FS)):
        env.step(ctl(env))
    ref = np.degrees(float(c.state[ITH]))
    sg = 1 if sd % 2 == 0 else -1
    T.set_tilt(env, sg * 44.0, 0.0, seed_filter=False)
    # motors held off 1 s (like Stop_Flag before the 2nd KEY1): the car settles on its
    # chassis and the angle filter converges; the firmware resets rec_core meanwhile
    for _ in range(int(1.0 * FS)):
        R.FORCE[0] = 1
        env.step(ctl(env))
    cfg = R.VAR[var]
    R.LIB.rec_init()
    R.LIB.rec_set(cfg[0], cfg[1])
    R.LIB.rec_exit(cfg[2], cfg[3], cfg[4])
    R.FORCE[0] = 0
    rest = np.degrees(float(c.state[ITH])) - ref
    t_lock, tail, drive = None, [], 0
    for n in range(int(5.0 * FS)):
        env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        if t_lock is None and R.LIB.rec_locked():
            t_lock = n / FS
        if not c.fw.st.motors_off and abs(c.fw.st.ccr_l) > 0:
            drive += 1
        if n >= int(4.5 * FS):
            tail.append(abs(th))
    return var, kg, sd, rest, max(tail) < 5.0, t_lock, drive / FS


def lying():
    import tilt_envelope as T
    T._path()
    s = T.selftest()
    jobs = [(v, kg, sd) for v in R.VAR for kg in (0.0, 1.0) for sd in (0, 1, 2, 3)]
    with Pool(16, initializer=_init_lying, initargs=(s,)) as pool:
        res = pool.map(_job_lying, jobs)
    print("car resting on its chassis, then motors on; 4 runs each (2 nose-down, 2 tail-down)")
    for v in R.VAR:
        for kg in (0.0, 1.0):
            rs = [r for r in res if r[0] == v and r[1] == kg]
            print("  %-17s %g kg: rests at %.1f deg | got up %d/4 | lock fired %d/4 (at %s s) | "
                  "motors driving %.1f s of 5" % (
                      v, kg, np.mean([abs(r[3]) for r in rs]), sum(r[4] for r in rs),
                      sum(r[5] is not None for r in rs),
                      "/".join("%.2f" % r[5] for r in rs if r[5] is not None) or "-",
                      np.mean([r[6] for r in rs])))


def _job_start(j):
    """Car placed at TH0 and at rest, THEN the motors are switched on (2nd KEY1):
    fresh v4 (boot gains, no balance reference yet), fresh PID state."""
    var, th0, sg, vb, sd = j
    import goto_hold as GH
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    R.FORCE[0] = 0
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=10, taps=(0,), payload_p_empty=1.0,
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
    ref = np.degrees(float(c.state[ITH]))
    T.set_tilt(env, sg * th0, 0.0)          # filter seeded: the car was at rest
    ctl = R.RecCtl(var)                       # created now = state at the 2nd KEY1
    tail_v, tail_th = [], []
    up = None
    for n in range(int(4.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        el, er = c._enc_last
        if d:
            return var, th0, vb, False, None
        if up is None and abs(th) < 3.0:
            up = n / FS
        if n >= int(3.5 * FS):
            tail_v.append(abs(0.5 * (el + er)))
            tail_th.append(abs(th))
    return var, th0, vb, (max(tail_v) < 4 and max(tail_th) < 3), up


def start():
    import tilt_envelope as T
    T._path()
    s = T.selftest()
    ths = (30, 35, 38, 40)
    jobs = [(v, th, sg, vb, sd) for v in R.VAR for th in ths for sg in (1, -1)
            for vb in (12.0, 10.5) for sd in (0, 1)]
    with Pool(16, initializer=R._init, initargs=(s,)) as pool:
        res = pool.map(_job_start, jobs)
    print("placed at rest, then motors on (fresh v4); cell = upright and stopped at 4 s /4, time to <3 deg")
    for vb in (12.0, 10.5):
        for v in R.VAR:
            cells = []
            for th in ths:
                rs = [r for r in res if r[0] == v and r[1] == th and r[2] == vb]
                ok = [r for r in rs if r[3]]
                tu = [r[4] for r in ok if r[4] is not None]
                cells.append("%2d deg %d/4 %s" % (th, len(ok), ("%.2f s" % np.mean(tu)) if tu else "  -   "))
            print("  %4.1f V %-17s " % (vb, v) + " | ".join(cells))


if __name__ == "__main__":
    {"bench": bench, "lying": lying, "start": start}[sys.argv[1]]()
