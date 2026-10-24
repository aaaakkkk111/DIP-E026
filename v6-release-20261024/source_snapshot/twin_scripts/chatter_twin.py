# -*- coding: utf-8 -*-
"""v5 chatter downshift (v4_core.c, CHATTER_DOWN) in the twin, 2026-10-21.

The C core is used through DLLs built from the same v4_core.c:
  v5core.dll      CHATTER_DOWN=1 (new)
  v5core_off.dll  CHATTER_DOWN=0 (= v4 behaviour)
Parts:
  bench : release benchmark (5 cases x 3 seeds, lower is better), off vs on
  stiff : car standing with load KG, gear FORCED to a too-stiff f (latched), 8 s:
          does it step down, does the chatter stop?
  place : load put on a standing empty car with a nudge (the real-car complaint),
          final gear / chatter / falls; heavy loads must not be stepped down
    python scripts/rl/chatter_twin.py bench|stiff|place
"""
import ctypes
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for q in (ROOT, HERE):
    if q not in sys.path:
        sys.path.insert(0, q)

DLLS = {"v4 (off)": "v5core_off.dll", "v5 (on)": "v5core.dll"}
FA = None
LIBS = {}


def _init():
    global FA
    import score_release_v4 as S
    S._init()
    import factory_vs_auto as FA_
    import switch_v3 as SV3
    FA = FA_
    for k, fn in DLLS.items():
        lib = ctypes.CDLL(os.path.join(HERE, fn))
        lib.v4_step.restype = ctypes.c_float
        lib.v4_step.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_int]
        lib.v4_osc.restype = ctypes.c_float
        lib.v4_force.argtypes = [ctypes.c_float]
        LIBS[k] = lib

    class V5C(SV3.SwitchV3):
        """Same wrapper as score_release_v4.V4C: the C core decides f, gains + bumpless."""
        def __init__(self, lib):
            super().__init__()
            self.lib = lib
            lib.v4_reset()
            self.prev = None

        def __call__(self, env, moving=None):
            tw = env.core
            v, w = float(env.v_ref), float(getattr(env, "yaw_ref", 0.0))
            cat = (0 if abs(v) < 1e-6 else int(np.sign(v)), 0 if abs(w) < 1e-6 else int(np.sign(w)))
            if self.prev is not None and cat != self.prev:
                self.lib.v4_cmd_edge()
            self.prev = cat
            self.f = float(self.lib.v4_step(float(tw._angle_filt), float(tw._gyro_lsb),
                                            0 if cat == (0, 0) else 1))
            base = self._gains()
            ki_old = float(self.pid.g.get("velocity_ki", 0.0))
            ki_new = float(base.get("velocity_ki", ki_old))
            if ki_old > 0.0 and ki_new > 0.0 and ki_new != ki_old:
                self.pid.enc_int *= ki_old / ki_new
            self.pid.g.update(base)
            return self.pid(env)

    globals()["V5C"] = V5C
    base = FA.Ctl

    class Ctl3(base):
        def __init__(self, kind, kg):
            if kind in DLLS:
                self.kind, self.kg = kind, kg
                self.sw = V5C(LIBS[kind])
                self.kg_hat, self.est, self.sched = 0.0, None, None
                self.pid = self.sw.pid
                return
            base.__init__(self, kind, kg)

        def __call__(self, env):
            if self.kind in DLLS:
                a = self.sw(env)
                self.kg_hat = self.sw.f
                return a
            return base.__call__(self, env)
    FA.Ctl = Ctl3


def _env(sd, kg=0.0):
    import goto_hold as GH
    from balance_bot.rl.e2e_env import E2EBalanceEnv, E2EConfig
    env = E2EBalanceEnv(E2EConfig(seed=sd, episode_seconds=24, taps=(0,), payload_p_empty=1.0,
                                  **dict(GH.BASE)))
    env._sample_payload = lambda: kg
    env.reset(seed=sd)
    env.yaw_ref = 0.0
    env._cmd_left = 10 ** 9
    env.v_ref = 0.0
    env.pos_ref = env.pos
    return env


def _tail(c, rows, n, n0):
    if n >= n0:
        rows.append((float(c._angle_filt), float(c._gyro_lsb) / 16.4))


def job_bench(a):
    import mc_twin_test as M
    kind, ci, sd = a
    _cn, kg, imp, _w = M.CASES[ci]
    r = FA.run(kind, kg, imp, 0.0, sd, 0.0)
    pen = 30.0 * (1.0 - r["legs"] / 3.0) if not r["ok"] else (
        (r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"])
    return a, dict(pen=pen, ok=r["ok"])


def job_stiff(a):
    from balance_bot.rl.e2e_env import FS
    kind, kg, f0, sd = a
    env = _env(sd, kg)
    c = env.core
    ctl = V5C(LIBS[kind])
    for _ in range(int(4.0 * FS)):
        env.step(ctl(env))
    LIBS[kind].v4_force(f0)
    rows, fs, osc_end = [], [], []
    d0, b0 = LIBS[kind].v4_downs(), LIBS[kind].v4_up_blocks()
    for n in range(int(8.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True)
        fs.append(ctl.f)
        _tail(c, rows, n, int(6.0 * FS))
        if n >= int(6.0 * FS):
            osc_end.append(float(LIBS[kind].v4_osc()))
    r = np.array(rows)
    return a, dict(fell=False, f_end=fs[-1], osc=float(np.mean(osc_end)), th_sd=float(r[:, 0].std()),
                   gy=float(np.sqrt(np.mean((r[:, 1] - r[:, 1].mean()) ** 2))),
                   downs=LIBS[kind].v4_downs() - d0, blocks=LIBS[kind].v4_up_blocks() - b0)


def job_place(a):
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    kind, kg, sd = a
    env = _env(sd)
    c = env.core
    ctl = V5C(LIBS[kind])
    for _ in range(int(4.0 * FS)):
        env.step(ctl(env))
    env._apply_payload(kg)
    T.set_tilt(env, 3.0, 60.0)
    rows, osc_end, fs = [], [], []
    for n in range(int(12.0 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        if d:
            return a, dict(fell=True)
        fs.append(ctl.f)
        _tail(c, rows, n, int(9.0 * FS))
        if n >= int(9.0 * FS):
            osc_end.append(float(LIBS[kind].v4_osc()))
    r = np.array(rows)
    return a, dict(fell=False, f_end=fs[-1], f_max=max(fs), osc=float(np.mean(osc_end)),
                   th_sd=float(r[:, 0].std()), downs=LIBS[kind].v4_downs())


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "bench":
        import mc_twin_test as M
        jobs = [(k, ci, sd) for k in DLLS for ci in range(len(M.CASES)) for sd in M.SEEDS]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_bench, jobs)
        print("release benchmark (lower is better)")
        for k in DLLS:
            per = {}
            for (kk, ci, sd), r in res:
                if kk == k:
                    per.setdefault(ci, []).append(r)
            tot = sum(M.CASES[ci][3] * np.mean([r["pen"] for r in v]) for ci, v in per.items())
            print("  %-10s total %6.2f  falls %d  per case %s" % (
                k, tot, sum(1 for v in per.values() for r in v if not r["ok"]),
                " ".join("%.2f" % np.mean([r["pen"] for r in per[ci]]) for ci in sorted(per))))
    elif what == "stiff":
        cases = [(0.0, 0.6), (0.0, 1.0), (0.3, 1.0), (0.5, 1.0), (1.0, 1.0), (1.5, 1.0), (2.0, 1.0), (2.0, 0.4), (4.0, 0.4)]
        jobs = [(k, kg, f0, sd) for k in DLLS for kg, f0 in cases for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_stiff, jobs)
        print("gear forced too stiff (latched), 8 s; 3 seeds; chatter = 8-16 Hz deg/s over the last 2 s (calm ~0.3)")
        for kg, f0 in cases:
            for k in DLLS:
                rs = [r for (kk, g, f, sd), r in res if kk == k and g == kg and f == f0]
                if any(r.get("fell") for r in rs):
                    print("  %3.1f kg forced f=%.1f %-9s FELL %d/3" % (kg, f0, k, sum(r.get("fell", False) for r in rs)))
                    continue
                print("  %3.1f kg forced f=%.1f %-9s | f end %s | chatter %5.1f | tilt sd %.2f | gyro %4.1f | downshifts %.1f | upshifts blocked %.0f" % (
                    kg, f0, k, "/".join("%.1f" % r["f_end"] for r in rs), np.mean([r["osc"] for r in rs]),
                    np.mean([r["th_sd"] for r in rs]), np.mean([r["gy"] for r in rs]),
                    np.mean([r["downs"] for r in rs]), np.mean([r["blocks"] for r in rs])))
    else:
        loads = (0.0, 0.3, 0.5, 1.0, 1.5, 2.0, 4.0)
        jobs = [(k, kg, sd) for k in DLLS for kg in loads for sd in (0, 1, 2)]
        with Pool(16, initializer=_init) as pool:
            res = pool.map(job_place, jobs)
        print("load put on a standing empty car with a nudge, 12 s; 3 seeds")
        for kg in loads:
            for k in DLLS:
                rs = [r for (kk, g, sd), r in res if kk == k and g == kg]
                if any(r.get("fell") for r in rs):
                    print("  %3.1f kg %-9s FELL %d/3" % (kg, k, sum(r.get("fell", False) for r in rs)))
                    continue
                print("  %3.1f kg %-9s | f max %.2f | f end %s | chatter %5.1f | tilt sd %.2f | downshifts %.1f" % (
                    kg, k, np.mean([r["f_max"] for r in rs]), "/".join("%.1f" % r["f_end"] for r in rs),
                    np.mean([r["osc"] for r in rs]), np.mean([r["th_sd"] for r in rs]), np.mean([r["downs"] for r in rs])))
