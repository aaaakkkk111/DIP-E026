# -*- coding: utf-8 -*-
"""Scoreboard for the v4 release (2026-10-03): factory NORMAL / factory HEAVY /
v4 (Python switch_v4, = 10-02 logic + fast light) / v4 release (the firmware C core
through v4rel.dll).  Same twin, same tasks, same rules as cem_v3.py and final_table.py.

    python scripts/rl/score_release_v4.py --procs 16

Score (cem_v3.py objective, lower is better): 5 cases x 3 seeds, stop-go-stop task
(3 waypoint legs, each reached within 5 cm and held 5 s).  Per episode:
  finished: settle time (s, 7 if it never settled) + 0.5 x chatter (deg/s)
  fell:     30 x (1 - legs done / 3)
averaged over seeds, weighted per case (empty 2, 1 kg 2, 2 kg 1, 4 kg 1.5, empty+3 N 1).
Table: final_table.py's 12 cases x 4 seeds, "stand 3 s first" and "go at once".
"""
import argparse
import ctypes
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
FA = None

OBJ_CASES = [("空载", 0.0, 0.0, 0.0, 0.0, 2.0), ("1kg", 1.0, 0.0, 0.0, 0.0, 2.0),
             ("2kg", 2.0, 0.0, 0.0, 0.0, 1.0), ("4kg", 4.0, 0.0, 0.0, 0.0, 1.5),
             ("空载+3N", 0.0, 3.0, 0.0, 0.0, 1.0)]
OBJ_SEEDS = [0, 1, 2]
TAB_CASES = [("空载 平地", 0.0, 0.0, 0.0, 0.0), ("1kg 平地", 1.0, 0.0, 0.0, 0.0),
             ("2kg 平地", 2.0, 0.0, 0.0, 0.0), ("4kg 平地", 4.0, 0.0, 0.0, 0.0),
             ("空载 下台阶10mm", 0.0, 0.0, 0.010, 0.0), ("2kg 下台阶10mm", 2.0, 0.0, 0.010, 0.0),
             ("空载 +3N", 0.0, 3.0, 0.0, 0.0), ("空载 +8N", 0.0, 8.0, 0.0, 0.0),
             ("2kg +8N", 2.0, 8.0, 0.0, 0.0), ("空载 上坡8度", 0.0, 0.0, 0.0, 8.0),
             ("2kg 上坡8度", 2.0, 0.0, 0.0, 8.0), ("2kg 下坡8度", 2.0, 0.0, 0.0, -8.0)]
TAB_SEEDS = [0, 1, 2, 3]
ROWS = [("原厂正常档 (模式1)", "normal"), ("原厂负重档", "heavy"),
        ("v4 Python 版", "v4py"), ("v4 正式版 (固件C核心)", "v4rel"),
        ("核对: C核心关掉10-03修改", "v4off")]


def _init():
    global FA
    for q in (ROOT, HERE):
        if q not in sys.path:
            sys.path.insert(0, q)
    os.chdir(ROOT)
    os.environ.setdefault("E026_REAL_DATA", "c:/Users/jiang li/Downloads/e026 keil/deadband_test")
    import factory_vs_auto as FA_
    import switch_v3 as SV3
    import switch_v4 as V4
    FA = FA_
    libs = {}
    for k, fn in (("v4rel", "v4rel.dll"), ("v4off", "v4rel_off.dll")):
        lib = ctypes.CDLL(os.path.join(HERE, fn))
        lib.v4_step.restype = ctypes.c_float
        lib.v4_step.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_int]
        libs[k] = lib

    class V4C(SV3.SwitchV3):
        """The firmware core decides f; gains and bumpless transfer as in v4_adapt.c."""
        def __init__(self, lib):
            super().__init__()
            self.lib = lib
            lib.v4_reset()
            self.prev = None

        def __call__(self, env, moving=None):
            tw = env.core
            v, w = float(env.v_ref), float(getattr(env, "yaw_ref", 0.0))
            # key-state analogue: (sign of speed command, sign of turn command)
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

    base = FA.Ctl

    class Ctl2(base):
        def __init__(self, kind, kg):
            if kind in ("v4py", "v4rel", "v4off"):
                self.kind, self.kg = kind, kg
                if kind == "v4py":
                    self.sw = V4.SwitchV4()
                    self.sw.l2 = None
                else:
                    self.sw = V4C(libs[kind])
                self.kg_hat, self.est, self.sched = 0.0, None, None
                self.pid = self.sw.pid
                return
            base.__init__(self, kind, kg)

        def __call__(self, env):
            if self.kind in ("v4py", "v4rel", "v4off"):
                a = self.sw(env)
                self.kg_hat = self.sw.f
                return a
            return base.__call__(self, env)

    FA.Ctl = Ctl2


def _job(t):
    tag, kind, kg, imp, st, sl, sd, stand = t
    r = FA.run(kind, kg, imp, st, sd, sl, stand_s=stand)
    return tag, kind, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", type=int, default=16)
    ap.add_argument("--rows", type=str, default="")
    ap.add_argument("--part", type=str, default="obj,tab")
    a = ap.parse_args()
    rows = [r for r in ROWS if not a.rows or r[1] in a.rows.split(",")]
    jobs = []
    for _rn, kind in rows:
        for ci, (cn, kg, imp, st, sl, w) in enumerate(OBJ_CASES):
            for sd in OBJ_SEEDS:
                if "obj" in a.part:
                    jobs.append((("obj", ci), kind, kg, imp, st, sl, sd, 0.0))
        for stand in ((3.0, 0.0) if "tab" in a.part else ()):
            for ci, (cn, kg, imp, st, sl) in enumerate(TAB_CASES):
                for sd in TAB_SEEDS:
                    jobs.append((("tab", stand, ci), kind, kg, imp, st, sl, sd, stand))
    acc = {}
    with Pool(a.procs, initializer=_init) as pool:
        for tag, kind, r in pool.imap_unordered(_job, jobs):
            acc.setdefault((kind,) + tag, []).append(r)

    print("== 跑分（cem_v3 目标函数，越低越好）==")
    for rn, kind in (rows if "obj" in a.part else []):
        tot, parts = 0.0, []
        for ci, (cn, kg, imp, st, sl, w) in enumerate(OBJ_CASES):
            pen = 0.0
            rs = acc[(kind, "obj", ci)]
            for r in rs:
                if not r["ok"]:
                    pen += 30.0 * (1.0 - r["legs"] / 3.0)
                else:
                    pen += (r["settle"] if r["settle"] is not None else 7.0) + 0.5 * r["osc"]
            pen /= len(rs)
            tot += w * pen
            parts.append("%s %.2f" % (cn, pen))
            fell = [r for r in rs if not r["ok"]]
            okr = [r for r in rs if r["ok"]]
            st_ = [(r["settle"] if r["settle"] is not None else 7.0) for r in okr]
            os_ = [r["osc"] for r in okr]
            fp = sum(30.0 * (1.0 - r["legs"] / 3.0) for r in fell) / len(rs)
            print("DETAIL|%s|%s|w=%.1f|falls=%d/%d|legs_fell=%s|settle_mean=%s|osc_mean=%s|settle_part=%.2f|osc_part=%.2f|fall_part=%.2f|pen=%.2f"
                  % (kind, cn, w, len(fell), len(rs), [r["legs"] for r in fell],
                     ("%.2f" % np.mean(st_)) if st_ else "-", ("%.2f" % np.mean(os_)) if os_ else "-",
                     sum(st_) / len(rs), 0.5 * sum(os_) / len(rs), fp, pen))
        print("%-26s 总分 %7.2f   | %s" % (rn, tot, "  ".join(parts)))

    for stand in ((3.0, 0.0) if "tab" in a.part else ()):
        print("== 12 工况完成局数 /4（%s）==" % ("先站 3 s 再走" if stand else "开局就走"))
        for rn, kind in rows:
            cells, tot = [], 0
            for ci, (cn, *_x) in enumerate(TAB_CASES):
                rs = acc[(kind, "tab", stand, ci)]
                ok = sum(r["ok"] for r in rs)
                tot += ok
                osc = [r["osc"] for r in rs if r["ok"]]
                cells.append("%s:%d%s" % (cn, ok, ("(抖%.1f)" % np.mean(osc)) if osc else ""))
            print("%-26s 合计 %2d/48 | %s" % (rn, tot, " ".join(cells)))


if __name__ == "__main__":
    main()
