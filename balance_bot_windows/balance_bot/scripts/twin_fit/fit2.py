# -*- coding: utf-8 -*-
"""第二轮拟合：转子惯量改挂关节 armature（物理上它随「轮子相对车身」转）。

结构 A  刚性传动 + 力矩层间隙
结构 B  串联弹性传动（转子 -- 弹簧 k / 阻尼 c / 间隙 -- 轮子）
两者都只用 kd=48 的三份录数拟合；kd120.txt 留出。
目标 = 静止指标（含 0.4-3 / 3-8 / 8-16 Hz 三个频段能量）+ 两次扫频增益 + 0.5*相位
"""
import os, sys, time, json, dataclasses
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
STRUCT = sys.argv[1] if len(sys.argv) > 1 else "A"

COMMON = [("stall", 0.15, 1.2, True), ("damp", 0.005, 1.0, True),
          ("irot", 1e-4, 5e-3, True), ("dead", 1400.0, 1800.0, False),
          ("delay", 0.0, 5.0, False), ("gnoise", 1e-4, 0.1, True),
          ("bl", 0.0, 3.0, False)]
SPACE = COMMON + ([("k", 1.0, 200.0, True), ("c", 1e-4, 0.3, True)] if STRUCT == "B" else [])
F = (1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0, 11.0, 12.0, 13.0, 14.0, 16.0, 18.0, 20.0, 24.0)
SKEYS = ("ang_std", "gyro_rms", "demand", "enc_abs", "pos_std", "peak_hz")


def init_worker():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def decode(z):
    out = {}
    for (k, lo, hi, lg), v in zip(SPACE, np.clip(z, 0.0, 1.0)):
        out[k] = float(np.exp(np.log(lo) + v * (np.log(hi) - np.log(lo)))) if lg else float(lo + v * (hi - lo))
    return out


def build_kw(p, gains=None, battery=None):
    import fit as FT, sea
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.firmware.motor import MotorCalibration
    from balance_bot.firmware.robot import STM32_CAR
    from balance_bot.params import DisturbanceConfig
    TB.PWM_DELAY_TICKS = p["delay"]
    cal = MotorCalibration(motor_deadband=int(round(p["dead"])), r_wheel=STM32_CAR.r_wheel,
                           stall_torque=p["stall"], tau_damp=p["damp"],
                           omega_noload=STM32_CAR.wheel_speed_max)
    bl = np.radians(p["bl"])
    if "k" in p:
        robot = dataclasses.replace(STM32_CAR, I_rotor=1e-6, tau_max=50.0)
        hook = lambda c: sea.install(c, k=p["k"], c=p["c"], bl_rad=bl, I_r=p["irot"])
    else:
        robot = dataclasses.replace(STM32_CAR, I_rotor=1e-6)

        def hook(c):
            c.model.dof_armature[6] = p["irot"]; c.model.dof_armature[7] = p["irot"]
            FT.wrap_backlash(c.motor, bl)
    kw = dict(motor_cal=cal, robot=robot, dist=DisturbanceConfig(noise_pitch_rate=p["gnoise"]),
              gains=gains, core_hook=hook, mount_offset_deg=0.31)
    if battery: kw["battery_v"] = battery
    return kw


def simulate(p, kind, amp=0, gains=None, battery=None, seconds=32.0):
    import metrics as M, twinrun as T, fit as FT
    kw = build_kw(p, gains, battery)
    if kind == "still":
        d = T.run(seconds, **kw)
        if d["fell"] or len(d["gyro"]) < 0.9 * seconds * 200:
            return None
        return M.standstill(d["gyro"], d["ang"], d["ml"], d["el"], d["er"])
    d = T.run(24.0, chirp_amp=amp, **kw)
    if d["fell"] or len(d["gyro"]) < 4500:
        return None
    return M.fresp(d["inj"], d["gyro"], f_of_t=F)


def feat(m):
    m = dict(m); m["demand"] = max(m["pwm_abs"] - 1500.0, 1.0); return m


def evaluate(z):
    import fit as FT
    p = decode(z)
    try:
        rs, rfr = FT.real()
        s = simulate(p, "still")
        f3 = simulate(p, "sweep", 300) if s is not None else None
        f5 = simulate(p, "sweep", 500) if f3 is not None else None
    except Exception as e:
        return 99.0, p, {"err": repr(e)}
    if s is None or f3 is None or f5 is None:
        return 50.0, p, {"fell": True}
    a, b = feat(rs), feat(s)
    es = [abs(np.log(max(b[k], 1e-3) / max(a[k], 1e-3))) for k in SKEYS]
    es += [abs(np.log(max(b["bands"][i], 0.05) / max(a["bands"][i], 0.05))) for i in (0, 1, 2)]
    e_still = float(np.mean(es))
    eg, ep = [], []
    for amp, tw in ((300, f3), (500, f5)):
        for (f, rg, rp), (_, tg, tp) in zip(rfr[amp], tw):
            eg.append(abs(np.log(max(tg, 1e-3) / max(rg, 1e-3))))
            ep.append(abs((tp - rp + 180.0) % 360.0 - 180.0) / 90.0)
    e_gain, e_phase = float(np.mean(eg)), float(np.mean(ep))
    return e_still + e_gain + 0.5 * e_phase, p, dict(still=e_still, gain=e_gain, phase=e_phase)


def main():
    rng = np.random.default_rng(7)
    dim = len(SPACE)
    mu = np.full(dim, 0.5); sig = np.full(dim, 0.3)
    POP, ELITE, ITERS = 36, 8, 18
    best = (1e9, None, None); hist = []
    t0 = time.time()
    with Pool(18, initializer=init_worker) as pool:
        for it in range(ITERS):
            Z = np.clip(rng.normal(mu, sig, size=(POP, dim)), 0, 1)
            res = pool.map(evaluate, list(Z))
            order = np.argsort([r[0] for r in res])
            el = Z[order[:ELITE]]
            mu = 0.7 * el.mean(0) + 0.3 * mu
            sig = np.maximum(0.7 * el.std(0) + 0.3 * sig, 0.02)
            if res[order[0]][0] < best[0]:
                best = res[order[0]]
            b = res[order[0]]
            nfell = sum(1 for r in res if r[0] >= 50)
            hist.append(dict(it=it, best=b[0], p=b[1], parts=b[2]))
            print("[%s] 迭代 %2d  本轮 %.3f  全局 %.3f  摔 %d/%d  (%.0fs)  %s  %s" % (
                STRUCT, it, b[0], best[0], nfell, POP, time.time() - t0,
                {k: (round(v, 4) if v < 10 else round(v, 1)) for k, v in b[1].items()},
                {k: round(v, 3) for k, v in b[2].items()}))
            sys.stdout.flush()
    json.dump(dict(struct=STRUCT, best=best[0], params=best[1], parts=best[2], hist=hist),
              open(os.path.join(HERE, "fit2_%s.json" % STRUCT), "w"), indent=1, default=float)
    print("[%s] 完成" % STRUCT)


if __name__ == "__main__":
    main()
