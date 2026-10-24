# -*- coding: utf-8 -*-
"""Twin test of the mode-27 large-tilt recovery (firmware rec_core.c via rec_core.dll).

Firmware changes being tested: cut-off 40 -> 45 deg both ways (mode 27), rec_core on
top of v4 (SwitchV4 == release v4 logic in the twin).  While recovering (|tilt| > 20
until upright, slow and calm): v4 decision logic skipped (gains kept), speed integral
held at 0.  Lock: >40 deg for 1 s -> motors off until < 10 deg.
(An earlier version also tried a separate recovery law u = KP*tilt + KD*rate + KV*speed:
it lifted but never stopped the car, 0/4 everywhere; removed.)

"static": stand 3.5 s, tip to TH0 with zero rate, filter seeded (car was at rest),
4 s.  ok = no fall (chassis touching the ground or > 45 deg), up = time to |tilt| <
3 deg, settle = |tilt|<1 & |rate|<20 for 0.25 s, travel = wheel distance at the end.
    python scripts/rl/rec_twin.py
"""
import ctypes, os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tilt_envelope as T

CUT = 45.0
FORCE = [0]
LIB = None
# (mode, on_deg, exit_deg, exit_dps, exit_v, v4 keeps deciding while recovering)
# Firmware (2026-10-20) = "v4 + recovery".  Variants tried and dropped (rec_twin2.log,
# rec_start2.log): pausing the v4 decision while recovering fails 0/4 when the car is
# STARTED at a tilt (v4 boots on stiff loaded gains and must relax them); the loose exit
# test (no wheel-speed condition) settled slower.
VAR = {
    "v4 (cut 45 only)":   (0, 20.0, 2.0, 30.0, 4.0, 1),
    "v4 + recovery":      (1, 20.0, 2.0, 30.0, 4.0, 1),
}
THETAS = (30, 35, 38, 40, 42)
VBS = (12.0, 10.5)


def _init(sign=1.0):
    global LIB
    T._init(sign)
    import recovery_bound40 as RB40      # raises the twin's own pitch_fail (was 40 deg)
    RB40.CUT = np.deg2rad(CUT)
    RB40._init()
    LIB = ctypes.CDLL(os.path.join(HERE, "rec_core.dll"))
    LIB.rec_set.argtypes = [ctypes.c_int, ctypes.c_float]
    LIB.rec_exit.argtypes = [ctypes.c_float] * 3
    LIB.rec_step.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    LIB.rec_sr.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float]
    from balance_bot.firmware import controllers as C
    def turn_off(self, angle_deg, battery_v, stop_flag=False):
        # FORCE 1 = locked (off), 2 = get-up attempt (angle cut-off and lock ignored)
        if FORCE[0] == 2:
            return bool(battery_v < 9.6 or stop_flag)
        return bool(FORCE[0] == 1 or abs(angle_deg) > CUT or battery_v < 9.6 or stop_flag)
    for name in dir(C):
        cls = getattr(C, name)
        if isinstance(cls, type) and "turn_off" in cls.__dict__:
            cls.turn_off = turn_off


class RecCtl:
    def __init__(self, var):
        import switch_v4 as V4
        self.v4 = V4.SwitchV4()
        cfg = VAR[var]
        LIB.rec_init(); LIB.rec_set(cfg[0], cfg[1]); LIB.rec_exit(cfg[2], cfg[3], cfg[4])
        self.mode, self.v4_on = cfg[0], cfg[5]
        self.u = ctypes.c_int(0)
        if len(cfg) > 6:
            LIB.rec_sr(*cfg[6])
        else:
            LIB.rec_sr(0, 40, 30, 120, 30.0)     # get-up off unless asked for

    def __call__(self, env):
        tw = env.core
        pid = self.v4.pid
        err = float(tw._angle_filt) - float(pid.g.get("mid_angle_deg", 0.0))
        el, er = tw._enc_last
        r = LIB.rec_step(err, float(tw._gyro_lsb), int(el), int(er), ctypes.byref(self.u))
        FORCE[0] = 2 if r == 2 else (1 if LIB.rec_locked() else 0)
        if r == 0:
            return self.v4(env)
        if r == 2:                   # get-up attempt: fixed push, PID filters keep running
            self.v4.pid(env)
            self.v4.pid.enc_int = 0.0
            return np.array([self.u.value / env.u_span, 0.0])
        act = self.v4(env) if self.v4_on else pid(env)   # v4 decision runs / paused (gains kept)
        pid.enc_int = 0.0            # V4_RecOverride: speed integral held at 0 (after Velocity_PI)
        return act


def run(var, th0, vb, sd):
    import goto_hold as GH
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig, FS
    from balance_bot.dynamics import ITH
    FORCE[0] = 0
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=10, taps=(0,), payload_p_empty=1.0, **dict(GH.BASE)))
    env._sample_payload = lambda: 0.0
    env.reset(seed=sd)
    c = env.core
    if hasattr(c, "battery_v"):
        c.battery_v = vb
    env.yaw_ref = 0.0; env._cmd_left = 10 ** 9; env.v_ref = 0.0; env.pos_ref = env.pos
    ctl = RecCtl(var)
    for _ in range(int(T.STAND_S * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return dict(stand=False)
    ref = np.degrees(float(c.state[ITH]))
    x0 = float(env.pos) if hasattr(env, "pos") else 0.0
    T.set_tilt(env, th0, 0.0)
    up = settle = None; calm = 0; vmax = 0.0; tail_v = []; tail_th = []
    for n in range(int(4.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        th = np.degrees(float(c.state[ITH])) - ref
        om = np.degrees(float(c.state[ITH + 1]))
        el, er = c._enc_last
        vmax = max(vmax, abs(0.5 * (el + er)) * 0.0319)
        if d:
            return dict(stand=True, ok=False, locks=LIB.rec_locked())
        if n >= int(3.5 * FS):
            tail_v.append(abs(0.5 * (el + er))); tail_th.append(abs(th))
        if up is None and abs(th) < 3.0:
            up = n / FS
        if abs(th) < 1.0 and abs(om) < 20.0:
            calm += 1
            if calm >= 50 and settle is None:
                settle = (n - 50) / FS
        else:
            calm = 0
    travel = abs(float(env.pos) - x0) if hasattr(env, "pos") else float("nan")
    stopped = max(tail_v) < 4 and max(tail_th) < 3
    return dict(stand=True, ok=True, stopped=stopped, up=up, settle=settle, vmax=vmax, travel=travel,
                entries=LIB.rec_entries(), locks=LIB.rec_locks())


def _job(a):
    var, th, sg, vb, sd = a
    return a, run(var, sg * th, vb, sd)


if __name__ == "__main__":
    T._path()
    s = T.selftest()
    jobs = [(v, th, sg, vb, sd) for v in VAR for th in THETAS for sg in (1, -1) for vb in VBS for sd in (0, 1)]
    acc = {}
    with Pool(16, initializer=_init, initargs=(s,)) as pool:
        for (v, th, sg, vb, sd), r in pool.imap_unordered(_job, jobs):
            acc.setdefault((v, th, vb), []).append(r)
    def m(x):
        x = [y for y in x if y is not None]
        return ("%.2f" % np.mean(x)) if x else " -- "
    for vb in VBS:
        print("\n=== battery %.1f V, empty car, released at rest; cell = upright AND stopped at 4 s /4  up s / settle s / travel m" % vb)
        for v in VAR:
            cells = []
            for th in THETAS:
                rs = acc[(v, th, vb)]
                ok = [r for r in rs if r.get("ok") and r.get("stopped")]
                cells.append("%d/4 %s/%s/%s" % (len(ok), m([r["up"] for r in ok]), m([r["settle"] for r in ok]),
                                                 m([r["travel"] for r in ok])))
            print("  %-17s " % v + "  ".join("%2d°:%-20s" % (th, c) for th, c in zip(THETAS, cells)))
