# -*- coding: utf-8 -*-
"""用真车录数拟合孪生的执行器 / 传感器参数（交叉熵法，16 进程并行）。

拟合数据（模式 1，kd=48，死区补偿 1500）：
    base_empty.txt   静止 32 s     -> 倾角 std、陀螺 rms、PWM 需求、编码器、位置、主频
    sweep_A_300.txt  扫频 幅值 300 -> 15 个频点的增益和相位
    sweep_A_500.txt  扫频 幅值 500 -> 同上
留出不拟合，只用来检验：kd120.txt（Kd 改成 120 的静止录数）。

参数（车体几何、质量、惯量是 STEP 实测的硬数据，不动）：
    stall     堵转力矩 N·m
    damp      电机阻尼力矩 N·m
    irot      反射转子惯量 kg·m²
    dead      电机实际死区（固件补偿固定 1500；>1500 = 欠补偿留下的零力矩区）
    delay     PWM 生效延迟（拍，可小数）
    gnoise    陀螺噪声 rad/s
    bl        齿轮回程间隙（输出轴单边，度）
"""
import os, sys, time, json, dataclasses
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))

# (名字, 下界, 上界, 对数?)
SPACE = [("stall", 0.15, 1.2, True), ("damp", 0.01, 1.0, True),
         ("irot", 1e-4, 5e-3, True), ("dead", 1400.0, 1800.0, False),
         ("delay", 0.0, 6.0, False), ("gnoise", 1e-4, 0.1, True),
         ("bl", 0.0, 3.0, False)]
F = (1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0, 11.0, 12.0, 13.0, 14.0, 16.0, 18.0, 20.0, 24.0)
SKEYS = ("ang_std", "gyro_rms", "demand", "enc_abs", "pos_std", "peak_hz")


def init_worker():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def decode(z):
    out = {}
    for (k, lo, hi, lg), v in zip(SPACE, np.clip(z, 0.0, 1.0)):
        out[k] = float(np.exp(np.log(lo) + v * (np.log(hi) - np.log(lo)))) if lg else float(lo + v * (hi - lo))
    return out


def wrap_backlash(motor, b_rad):
    raw = motor.torque
    st = {0: 0.0, 1: 0.0}

    def torque(ccr, omega_wheel, supply_v=None, dt=None, side=0):
        tau = raw(ccr, omega_wheel, supply_v, dt=dt, side=side)
        if b_rad <= 1e-9 or dt is None:
            return tau
        d = st[side]
        if abs(d) < b_rad:
            w_free = motor.duty(ccr) * motor.cal.omega_noload
            d = max(-b_rad, min(b_rad, d + (w_free - omega_wheel) * dt))
            st[side] = d
            return tau if (abs(d) >= b_rad and d * tau > 0.0) else 0.0
        if d * tau <= 0.0:
            st[side] = d - np.sign(d) * 1e-9
            return 0.0
        return tau
    motor.torque = torque


def simulate(p, kind, amp=0, gains=None, battery=None, seconds=None):
    import metrics as M, twinrun as T
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.firmware.motor import MotorCalibration
    from balance_bot.firmware.robot import STM32_CAR
    from balance_bot.params import DisturbanceConfig
    TB.PWM_DELAY_TICKS = p["delay"]
    cal = MotorCalibration(motor_deadband=int(round(p["dead"])), r_wheel=STM32_CAR.r_wheel,
                           stall_torque=p["stall"], tau_damp=p["damp"],
                           omega_noload=STM32_CAR.wheel_speed_max)
    robot = dataclasses.replace(STM32_CAR, I_rotor=p["irot"])
    dist = DisturbanceConfig(noise_pitch_rate=p["gnoise"])
    bl = np.radians(p["bl"])
    kw = dict(motor_cal=cal, robot=robot, dist=dist, gains=gains,
              core_hook=lambda c: wrap_backlash(c.motor, bl),
              mount_offset_deg=0.31)
    if battery: kw["battery_v"] = battery
    if kind == "still":
        d = T.run(seconds or 32.0, **kw)
        if d["fell"] or len(d["gyro"]) < 0.9 * (seconds or 32.0) * 200:
            return None
        return M.standstill(d["gyro"], d["ang"], d["ml"], d["el"], d["er"])
    d = T.run(24.0, chirp_amp=amp, **kw)
    if d["fell"] or len(d["gyro"]) < 4500:
        return None
    return M.fresp(d["inj"], d["gyro"], f_of_t=F)


_REAL = None


def real():
    global _REAL
    if _REAL is None:
        import metrics as M
        r = M.load("base_empty.txt")
        still = M.standstill(r["gyro"], r["ang"], r["ml"], r["el"], r["er"])
        fr = {}
        for nm, a in (("sweep_A_300.txt", 300), ("sweep_A_500.txt", 500)):
            d = M.load(nm); fr[a] = M.fresp(d["inj"], d["gyro"], f_of_t=F)
        _REAL = (still, fr)
    return _REAL


def feat(m):
    m = dict(m); m["demand"] = max(m["pwm_abs"] - 1500.0, 1.0); return m


def evaluate(z):
    p = decode(z)
    try:
        rs, rfr = real()
        s = simulate(p, "still")
        f3 = simulate(p, "sweep", 300)
        f5 = simulate(p, "sweep", 500)
    except Exception as e:
        return 99.0, p, {"err": repr(e)}
    if s is None or f3 is None or f5 is None:
        return 50.0, p, {"fell": True}
    a, b = feat(rs), feat(s)
    e_still = float(np.mean([abs(np.log(max(b[k], 1e-3) / max(a[k], 1e-3))) for k in SKEYS]))
    eg, ep = [], []
    for amp, tw in ((300, f3), (500, f5)):
        for (f, rg, rp), (_, tg, tp) in zip(rfr[amp], tw):
            eg.append(abs(np.log(max(tg, 1e-3) / max(rg, 1e-3))))
            ep.append(abs((tp - rp + 180.0) % 360.0 - 180.0) / 90.0)
    e_gain, e_phase = float(np.mean(eg)), float(np.mean(ep))
    total = e_still + e_gain + 0.5 * e_phase
    return total, p, dict(still=e_still, gain=e_gain, phase=e_phase, s=b)


def main():
    rng = np.random.default_rng(1)
    dim = len(SPACE)
    mu = np.full(dim, 0.5); sig = np.full(dim, 0.3)
    # 现状作为第一个样本（阳性对照：拟合必须比它好）
    now = dict(stall=0.40, damp=0.15, irot=6.6e-4, dead=1500.0, delay=1.0, gnoise=1e-4, bl=0.0)
    z_now = []
    for (k, lo, hi, lg) in SPACE:
        v = now[k]
        z_now.append((np.log(v) - np.log(lo)) / (np.log(hi) - np.log(lo)) if lg else (v - lo) / (hi - lo))
    z_now = np.clip(np.array(z_now), 0, 1)
    POP, ELITE, ITERS = 32, 7, 16
    best = (1e9, None, None)
    hist = []
    t0 = time.time()
    with Pool(16, initializer=init_worker) as pool:
        base = pool.apply(evaluate, (z_now,))
        print("现状: 总误差 %.3f  %s" % (base[0], {k: round(v, 3) for k, v in base[2].items() if k != "s"}))
        sys.stdout.flush()
        for it in range(ITERS):
            Z = np.clip(rng.normal(mu, sig, size=(POP, dim)), 0, 1)
            if it == 0: Z[0] = z_now
            res = pool.map(evaluate, list(Z))
            order = np.argsort([r[0] for r in res])
            el = Z[order[:ELITE]]
            mu = 0.7 * el.mean(0) + 0.3 * mu
            sig = np.maximum(0.7 * el.std(0) + 0.3 * sig, 0.02)
            if res[order[0]][0] < best[0]:
                best = res[order[0]]
            b = res[order[0]]
            hist.append(dict(it=it, best=b[0], p=b[1], parts={k: v for k, v in b[2].items() if k != "s"}))
            print("迭代 %2d  本轮最优 %.3f  全局最优 %.3f  (%.0fs)  %s  %s" % (
                it, b[0], best[0], time.time() - t0,
                {k: (round(v, 4) if v < 10 else round(v)) for k, v in b[1].items()},
                {k: round(v, 3) for k, v in b[2].items() if k != "s"}))
            sys.stdout.flush()
    json.dump(dict(best=best[0], params=best[1],
                   parts={k: v for k, v in best[2].items() if k != "s"},
                   still=best[2].get("s"), baseline=base[0], hist=hist),
              open(os.path.join(HERE, "fit_result.json"), "w"), indent=1, default=float)
    print("完成，结果写入 fit_result.json")


if __name__ == "__main__":
    main()
