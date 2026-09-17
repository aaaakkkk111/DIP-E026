# -*- coding: utf-8 -*-
"""第四轮：只对模式 1 静止基准拟合（用户：最最最标准的小车基准）。

陀螺零偏固定为实测 +2.02 °/s，不拟合。
目标 = bench.score：各硬指标落在三次复现范围内为 0，范围外按对数距离。
每组参数 2 个噪声种子取指标平均。起点：第三轮最终参数，摩擦 0。
允许电机死区低于补偿 1500（补偿零点的真正跳变 / 继电器效应）。
"""
import os, sys, time, json
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
ITERS = int(sys.argv[1]) if len(sys.argv) > 1 else 25
SEEDS = tuple(int(x) for x in os.environ.get("FIT_SEEDS", "0,1").split(","))

SPACE = [("stall", 0.15, 1.2, "log"), ("damp", 0.002, 0.5, "log"),
         ("irot", 1e-4, 4e-3, "log"), ("dpos", 1350.0, 1650.0, "lin"),
         ("dneg", 1350.0, 1650.0, "lin"), ("delay", 0.0, 6.0, "lin"),
         ("gnoise", 1e-4, 0.05, "log"), ("k", 1.0, 200.0, "log"),
         ("c", 1e-4, 0.3, "log"), ("bl", 0.0, 2.0, "lin"),
         ("fric", 1e-4, 0.08, "log"), ("ib", 0.7, 1.4, "log"),
         ("mnt", -0.5, 1.0, "lin"), ("vib", 0.002, 0.06, "log"), ("roll", 0.00005, 0.004, "log"),
         ("fs", 1e-4, 0.02, "log"), ("fkr", 0.05, 1.0, "lin"), ("vs", 0.1, 5.0, "log"),
         ("vibm", 1e-3, 0.2, "log"),
         ("lrs", 0.0, 220.0, "lin")]
START = dict(stall=0.4598, damp=0.0043, irot=0.0008, dpos=1468.5, dneg=1501.9, delay=1.9322,
             gnoise=0.001, k=3.8795, c=0.1312, bl=0.9444, fric=0.0005, ib=0.9991, mnt=0.2855)
import json as _json, os as _os
_prev = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "fit4.json")
if _os.environ.get("FIT_START_FROM"):
    START = dict(_json.load(open(_os.environ["FIT_START_FROM"]))["params"]); START.pop("v2", None)
START.setdefault("vib", float(_os.environ.get("FIT_VIB0", "0.022")))
START.setdefault("roll", float(_os.environ.get("FIT_ROLL0", "0.0005")))


def init_worker():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def decode(z):
    out = {}
    for (k, lo, hi, sc), v in zip(SPACE, np.clip(z, 0.0, 1.0)):
        out[k] = float(np.exp(np.log(lo) + v * (np.log(hi) - np.log(lo)))) if sc == "log" \
            else float(lo + v * (hi - lo))
    return out


def encode(p):
    z = []
    for k, lo, hi, sc in SPACE:
        v = p[k]
        z.append((np.log(v) - np.log(lo)) / (np.log(hi) - np.log(lo)) if sc == "log" else (v - lo) / (hi - lo))
    return np.clip(np.array(z), 0.0, 1.0)


def evaluate(z):
    import replay as R, bench as B
    p = decode(z)
    p["v2"] = True
    ms = []
    try:
        for sd in SEEDS:
            r = R.replay("base_empty.txt", p, seed=sd, settle=5.0)
            if r["fell_at"] is not None:
                return 9.0, p, {"fell": True}
            ms.append(B.metrics(r["twin"]))
    except Exception as e:
        return 99.0, p, {"err": repr(e)[:200]}
    m = B.mean_metrics(ms)
    sc, parts = B.score(m)
    return float(sc), p, dict(m=m, parts=parts)


def main():
    rng = np.random.default_rng(21)
    dim = len(SPACE)
    mu = encode(START)
    sig = np.full(dim, float(os.environ.get("FIT_SIG", "0.15")))
    POP, ELITE = 36, 8
    best = (1e9, None, None); best_z = mu.copy(); hist = []
    t0 = time.time()
    with Pool(18, initializer=init_worker) as pool:
        for it in range(ITERS):
            Z = np.clip(rng.normal(mu, sig, size=(POP, dim)), 0, 1)
            Z[0] = best_z
            res = pool.map(evaluate, list(Z))
            order = np.argsort([r[0] for r in res])
            el = Z[order[:ELITE]]
            mu = 0.7 * el.mean(0) + 0.3 * mu
            sig = np.maximum(0.7 * el.std(0) + 0.3 * sig, 0.015)
            if res[order[0]][0] < best[0]:
                best = res[order[0]]; best_z = Z[order[0]].copy()
            b = res[order[0]]
            nfell = sum(1 for r in res if r[0] >= 9)
            miss = [k for k, v in b[2].get("parts", {}).items() if v > 0]
            hist.append(dict(it=it, best=b[0], p=b[1]))
            print("[%s] 迭代" % os.environ.get("FIT_TAG", "4") + " %2d  本轮 %.3f  全局 %.3f  摔 %d/%d  (%.0fs)  未达标: %s" % (
                it, b[0], best[0], nfell, POP, time.time() - t0, miss))
            print("      %s" % {k: (round(v, 4) if abs(v) < 10 else round(v, 1)) for k, v in b[1].items() if k != "v2"})
            sys.stdout.flush()
            json.dump(dict(best=best[0], params=best[1], detail=best[2], hist=hist),
                      open(os.path.join(HERE, os.environ.get("FIT_OUT", "fit4.json")), "w"), indent=1, default=float)
    print("[4] 完成")


if __name__ == "__main__":
    main()
