# -*- coding: utf-8 -*-
"""源码默认孪生（第六轮参数）上逐个加结构，看哪个能把 4.3 Hz 主振荡幅值压 ~25% 而不改频率。
4 个新种子。"""
import os, sys
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SEEDS = (200, 201, 202, 203)
CASES = [
    ("源码默认", {}),
    ("滚动阻力 0.0005 m", dict(roll=0.0005)),
    ("滚动阻力 0.002 m", dict(roll=0.002)),
    ("滚动阻力 0.005 m", dict(roll=0.005)),
    ("滚动阻力 0.01 m", dict(roll=0.01)),
]
KEYS = ("ang_std", "g3_6", "g6_12", "g12_100", "d0_03", "disp_std", "f_gyro", "rev")


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    label, over, seed = args
    import replay as R, bench as B
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.firmware.motor import MotorCalibration
    import dataclasses
    orig_apply = TB.STM32TwinMixin.__dict__.get("_apply_model_mode")

    def post_patch(core):
        m = core.model
        if "wdamp" in over:
            m.dof_damping[6] = over["wdamp"]; m.dof_damping[7] = over["wdamp"]
        if "roll" in over:
            import mujoco
            for name in ("gw_l", "gw_r", "floor"):
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
                if gid >= 0:
                    m.geom_condim[gid] = 6
                    m.geom_friction[gid, 2] = over["roll"]
        if "solref" in over:
            import mujoco
            for name in ("gw_l", "gw_r"):
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
                m.geom_solref[gid, 0] = over["solref"]

    changes = {}
    base = MotorCalibration()
    if "kv" in over:
        changes["omega_noload"] = base.omega_noload / over["kv"]
    if "dsym" in over:
        changes["motor_deadband"] = int(over["dsym"]); changes["motor_deadband_neg"] = float(over["dsym"])
    if "fric" in over:
        changes["gear_friction"] = over["fric"]
    if "delay" in over:
        TB.PWM_DELAY_TICKS = over["delay"]
    if "gnoise" in over:
        TB.GYRO_NOISE_RAD_S = over["gnoise"]
    if "k" in over:
        changes["gear_stiffness"] = over["k"]
    kw_extra = dict(motor_cal=dataclasses.replace(base, **changes)) if changes else {}

    import replay
    old_build = None
    import params_model as PM
    real_build = PM.build

    def build(p):
        if p.get("source"):
            return dict(kw_extra), post_patch
        return real_build(p)
    PM.build = build
    try:
        r = R.replay("base_empty.txt", dict(source=True), seed=seed, settle=5.0)
    finally:
        PM.build = real_build
    if r["fell_at"] is not None:
        return label, None
    m = B.metrics(r["twin"])
    m.update(extra_bands(r["twin"]))
    return label, m


def extra_bands(d):
    import bench as B
    def dec(x, bands):
        x = B._fill(x); y = x - x.mean()
        sp = np.abs(np.fft.rfft(y)) ** 2
        fr = np.fft.rfftfreq(len(y), 1 / 200.0); tot = sp[1:].sum()
        return [y.std() * np.sqrt(sp[(fr >= lo) & (fr < hi) & (fr > 0)].sum() / tot) for lo, hi in bands]
    g = dec(d["gyro"] / 16.4, ((3, 6), (6, 12), (12, 100)))
    disp = np.cumsum((B._fill(d["el"]) + B._fill(d["er"])) / 2.0) * B.MM_PER_COUNT
    dl = dec(disp, ((0.0, 0.3),))
    return dict(g3_6=g[0], g6_12=g[1], g12_100=g[2], d0_03=dl[0])


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    rr = R.parse("base_empty.txt")["real"]
    real = B.metrics(rr); real.update(extra_bands(rr))
    jobs = [(lab, ov, s) for lab, ov in CASES for s in SEEDS]
    with Pool(6, initializer=init) as pool:
        res = pool.map(one, jobs)
    print("%-20s" % "情形" + "".join("%9s" % k for k in KEYS) + "%8s" % "综合")
    print("%-20s" % "真车" + "".join("%9.2f" % real[k] for k in KEYS) + "%8.3f" % B.score(real)[0])
    for lab, _ in CASES:
        ms = [m for l, m in res if l == lab and m is not None]
        if len(ms) < len(SEEDS):
            print("%-20s 摔了 %d/%d" % (lab, len(SEEDS) - len(ms), len(SEEDS)))
            if not ms:
                continue
        m = B.mean_metrics(ms)
        print("%-20s" % lab + "".join("%9.2f" % m[k] for k in KEYS) + "%8.3f" % B.score(m)[0])
