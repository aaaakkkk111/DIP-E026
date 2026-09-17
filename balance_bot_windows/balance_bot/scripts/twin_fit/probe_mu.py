# -*- coding: utf-8 -*-
"""源码默认孪生上扫地面摩擦系数 μ 和传动摩擦倍率，看真车录波更像哪一个。"""
import os, sys
import numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SEEDS = (400, 401, 402, 403, 404, 405)
CASES = [("μ %.2f" % mu, dict(mu=mu)) for mu in (0.70, 0.85, 1.00, 1.20, 1.50)]
CASES += [("传动 %d%%" % int(k * 100), dict(kf=k)) for k in (0.5, 0.75, 1.5, 2.0)]
CASES += [("滚阻 c=%.3f" % c, dict(rr=c * 0.0335)) for c in (0.012, 0.036, 0.048)]
KEYS = ("ang_std", "gyro_rms", "demand", "rev", "enc_abs", "g3_6", "g6_12", "disp_std", "f_gyro")

def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)

def one(a):
    lab, over, seed = a
    import replay as R, bench as B, params_model as PM
    import balance_bot.firmware.twin_baseline as TB
    real_build = PM.build
    def build(p):
        kw, post0 = real_build(p)
        def post(core):
            post0(core)
            if "kf" in over:
                k = over["kf"]
                core.motor.cal.gear_friction *= k
                core.rotor_fric_static = TB.ROTOR_FRIC_STATIC * k
                core.rotor_fric_kinetic = TB.ROTOR_FRIC_KINETIC * k
                core.model.dof_frictionloss[6] = core.motor.cal.gear_friction
                core.model.dof_frictionloss[7] = core.motor.cal.gear_friction
            if "rr" in over:
                core.rolling_resist_m = over["rr"]
            mj = core._mj
            for name in ("gw_l", "gw_r", "floor"):
                gid = mj.mj_name2id(core.model, mj.mjtObj.mjOBJ_GEOM, name)
                if gid < 0:
                    continue
                if "mu" in over:
                    core.model.geom_friction[gid, 0] = over["mu"]
                if "rr" in over:
                    core.model.geom_friction[gid, 2] = over["rr"]
        return kw, post
    PM.build = build
    try:
        r = R.replay("base_empty.txt", dict(source=True), seed=seed, settle=5.0)
    finally:
        PM.build = real_build
    return lab, (None if r["fell_at"] is not None else B.metrics(r["twin"]))

if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    with Pool(12, initializer=init) as pool:
        res = pool.map(one, [(l, o, s) for l, o in CASES for s in SEEDS])
    print("%-14s" % "情形" + "".join("%9s" % k for k in KEYS) + "%8s" % "综合")
    print("%-14s" % "真车" + "".join("%9.2f" % real[k] for k in KEYS))
    for lab, _ in CASES:
        ms = [m for l, m in res if l == lab and m is not None]
        if not ms:
            print(lab, "全摔"); continue
        m = B.mean_metrics(ms)
        print("%-14s" % lab + "".join("%9.2f" % m[k] for k in KEYS) + "%8.3f" % B.score(m)[0]
              + ("" if len(ms) == len(SEEDS) else "  摔%d" % (len(SEEDS) - len(ms))))
