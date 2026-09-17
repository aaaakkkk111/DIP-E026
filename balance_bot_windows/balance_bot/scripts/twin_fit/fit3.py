# -*- coding: utf-8 -*-
"""第三轮：全部基于原样回放的拟合。

拟合（5 份，都用文件自己的增益 / 死区补偿 / 电池 / inj 回放）：
权重：静止 x3（用户要求稳定状态优先）、扫频 x1、kd120 与 Kp38400 各 x0.5
  base_empty      静止指标（含 PWM 换向率、|PWM| 需求、各频段能量、倾角均值）
  sweep_A_300/500 扫频增益 + 相位 + 扫频期间陀螺逐拍相关
  kd120           高 Kd 极限环：倾角 std、陀螺、饱和、换向、主频、8-16 Hz 能量
  sweep_300       Kp 38400（开机 HEAVY 6 s 再切 NORMAL）：饱和、倾角 std、陀螺
留出检验：sweep_m1_300（另一次独立的幅值 300 扫频，扫到 50 Hz）

自由参数 = 所有没实测验证的量（延迟放开到 50 ms）。
用法：python fit3.py A|B
"""
import os, sys, time, json
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
STRUCT = sys.argv[1] if len(sys.argv) > 1 else "A"
ITERS = int(sys.argv[2]) if len(sys.argv) > 2 else 20

SPACE = [("stall", 0.15, 1.2, "log"), ("damp", 0.005, 1.0, "log"),
         ("irot", 1e-4, 5e-3, "log"), ("dpos", 1350.0, 1750.0, "lin"),
         ("dneg", 1350.0, 1750.0, "lin"), ("delay", 0.0, 10.0, "lin"),
         ("gnoise", 1e-4, 0.1, "log"), ("bl", 0.0, 3.0, "lin"),
         ("ib", 0.5, 2.0, "log"), ("lc", 0.6, 1.6, "log"),
         ("enc", 0.85, 1.15, "lin"), ("mnt", -1.0, 1.0, "lin")]
if STRUCT == "B":
    SPACE += [("k", 1.0, 200.0, "log"), ("c", 1e-4, 0.3, "log")]
FR_F = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
        15.0, 16.0, 18.0, 20.0, 22.0, 24.0)


def init_worker():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def decode(z):
    out = {}
    for (k, lo, hi, sc), v in zip(SPACE, np.clip(z, 0.0, 1.0)):
        out[k] = float(np.exp(np.log(lo) + v * (np.log(hi) - np.log(lo)))) if sc == "log" \
            else float(lo + v * (hi - lo))
    return out


def lerr(a, b, floor=1e-3):
    return abs(np.log(max(b, floor) / max(a, floor)))


_REAL = {}


def real(name):
    import replay as R, metrics as M
    if name not in _REAL:
        f = R.parse(name)
        d = dict(file=f, stats=M.standstill(f["real"]["gyro"], f["real"]["ang"], f["real"]["ml"],
                                            f["real"]["el"], f["real"]["er"]))
        if f["chirp"]:
            d["fr"] = M.fresp(f["inj"], f["real"]["gyro"], fmax=f["chirp"][2], f_of_t=FR_F)
        _REAL[name] = d
    return _REAL[name]


def still_terms(rs, ts, keys, bands=(0, 1, 2)):
    rs = dict(rs); ts = dict(ts)
    rs["demand"] = max(rs["pwm_abs"] - 1500, 1); ts["demand"] = max(ts["pwm_abs"] - 1500, 1)
    e = [lerr(rs[k], ts[k]) for k in keys]
    e += [lerr(rs["bands"][i], ts["bands"][i], 0.05) for i in bands]
    return float(np.mean(e))


def evaluate(z):
    import replay as R, metrics as M
    p = decode(z)
    if STRUCT == "B":
        pass
    parts = {}
    try:
        # 1) 静止
        r = R.replay("base_empty.txt", p)
        if r["fell_at"] is not None:
            return 60.0, p, {"fell": "base_empty"}
        tw = r["twin"]
        ts = M.standstill(tw["gyro"], tw["ang"], tw["ml"], tw["el"], tw["er"])
        rr = real("base_empty.txt")["stats"]
        parts["still"] = still_terms(rr, ts, ("ang_std", "gyro_rms", "demand", "pwm_rev",
                                              "enc_abs", "pos_std", "peak_hz")) \
            + abs(ts["ang_mean"] - rr["ang_mean"]) / 1.0 + ts["pwm_sat"] / 10.0
        # 2) 扫频
        eg, ep, ec = [], [], []
        for nm in ("sweep_A_300.txt", "sweep_A_500.txt"):
            r = R.replay(nm, p)
            if r["fell_at"] is not None:
                return 55.0, p, {"fell": nm}
            ref = real(nm)
            f = ref["file"]
            tfr = M.fresp(f["inj"], r["twin"]["gyro"], fmax=f["chirp"][2], f_of_t=FR_F)
            for (fq, rg, rp), (_, tg, tp) in zip(ref["fr"], tfr):
                eg.append(lerr(rg, tg)); ep.append(abs((tp - rp + 180) % 360 - 180) / 90.0)
            a = f["real"]["gyro"]; b = r["twin"]["gyro"]
            act = np.nonzero(f["inj"] != 0)[0]
            s = slice(act[0], act[-1])
            ok = ~np.isnan(a[s]) & ~np.isnan(b[s])
            ec.append(1.0 - float(np.corrcoef(a[s][ok], b[s][ok])[0, 1]))
        parts["gain"] = float(np.mean(eg))
        parts["phase"] = float(np.mean(ep))
        parts["corr"] = float(np.mean(ec))
        # 3) kd120
        r = R.replay("kd120.txt", p)
        n = r["fell_at"]
        if n is not None and n < 1000:
            parts["kd120"] = 3.0
        else:
            tw = r["twin"]
            ts = M.standstill(tw["gyro"][:n], tw["ang"][:n], tw["ml"][:n], tw["el"][:n], tw["er"][:n])
            rr = real("kd120.txt")["stats"]
            parts["kd120"] = still_terms(rr, ts, ("ang_std", "gyro_rms", "pwm_rev", "peak_hz"), bands=(2,)) \
                + abs(ts["pwm_sat"] - rr["pwm_sat"]) / 50.0 + (1.0 if n is not None else 0.0)
        # 4) Kp 38400
        r = R.replay("sweep_300.txt", p)
        n = r["fell_at"]
        tw = r["twin"]
        rf = real("sweep_300.txt")["file"]["real"]
        if n is not None and n < 400:
            parts["kp384"] = 3.0
        else:
            ml_t = tw["ml"][:n]; ml_t = ml_t[~np.isnan(ml_t)]
            ml_r = rf["ml"][~np.isnan(rf["ml"])]
            sat_t = np.mean(np.abs(ml_t) >= 2600) * 100; sat_r = np.mean(np.abs(ml_r) >= 2600) * 100
            parts["kp384"] = float(np.mean([
                abs(sat_t - sat_r) / 50.0,
                lerr(np.nanstd(rf["ang"]), np.nanstd(tw["ang"][:n])),
                lerr(np.nanstd(rf["gyro"]), np.nanstd(tw["gyro"][:n]))]))
    except Exception as e:
        return 99.0, p, {"err": repr(e)[:200]}
    # 用户要求：稳定状态优先。静止 x3；扫频（稳态附近小扰动）x1；
    # kd120 / Kp38400 只作约束 x0.5，不许为了凑失稳去牺牲稳态。
    total = 3.0 * parts["still"] + parts["gain"] + 0.5 * parts["phase"] + parts["corr"] \
        + 0.5 * parts["kd120"] + 0.5 * parts["kp384"]
    return float(total), p, parts


def main():
    rng = np.random.default_rng(11)
    dim = len(SPACE)
    mu = np.full(dim, 0.5); sig = np.full(dim, 0.3)
    # 用上一轮的最优解做起点
    prev = os.path.join(HERE, "fit2_%s.json" % STRUCT)
    if os.path.exists(prev):
        pp = json.load(open(prev))["params"]
        pp = dict(pp); pp["dpos"] = pp.get("dead", 1500); pp["dneg"] = pp.get("dead", 1500)
        pp.setdefault("ib", 1.0); pp.setdefault("lc", 1.0); pp.setdefault("enc", 1.0); pp.setdefault("mnt", 0.31)
        for i, (k, lo, hi, sc) in enumerate(SPACE):
            v = pp.get(k)
            if v is None:
                continue
            mu[i] = np.clip((np.log(v) - np.log(lo)) / (np.log(hi) - np.log(lo)) if sc == "log"
                            else (v - lo) / (hi - lo), 0.02, 0.98)
    POP, ELITE = 40, 8
    # 从上一轮最优解出发做局部精修：初始范围 +-8%。第一次用 +-30% 时，
    # 14 维里一半样本站不住，精英均值被拖离起点，第 1 轮就比起点差。
    sig = np.full(dim, 0.08)
    best = (1e9, None, None); hist = []
    best_z = mu.copy()
    t0 = time.time()
    with Pool(18, initializer=init_worker) as pool:
        for it in range(ITERS):
            Z = np.clip(rng.normal(mu, sig, size=(POP, dim)), 0, 1)
            Z[0] = best_z                  # 精英保留：当前最优解每轮都在
            res = pool.map(evaluate, list(Z))
            order = np.argsort([r[0] for r in res])
            el = Z[order[:ELITE]]
            mu = 0.7 * el.mean(0) + 0.3 * mu
            sig = np.maximum(0.7 * el.std(0) + 0.3 * sig, 0.01)
            if res[order[0]][0] < best[0]:
                best = res[order[0]]
                best_z = Z[order[0]].copy()
            b = res[order[0]]
            nbad = sum(1 for r in res if r[0] >= 50)
            hist.append(dict(it=it, best=b[0], p=b[1], parts=b[2]))
            print("[%s] 迭代 %2d  本轮 %.3f  全局 %.3f  失败 %d/%d  (%.0fs)\n      %s\n      %s" % (
                STRUCT, it, b[0], best[0], nbad, POP, time.time() - t0,
                {k: (round(v, 4) if abs(v) < 10 else round(v, 1)) for k, v in b[1].items()},
                {k: (round(v, 3) if isinstance(v, float) else v) for k, v in b[2].items()}))
            sys.stdout.flush()
            json.dump(dict(struct=STRUCT, best=best[0], params=best[1], parts=best[2], hist=hist),
                      open(os.path.join(HERE, "fit3_%s.json" % STRUCT), "w"), indent=1, default=float)
    print("[%s] 完成" % STRUCT)


if __name__ == "__main__":
    main()
