# -*- coding: utf-8 -*-
"""第六轮最优参数，用 8 个没参与拟合的新种子验证模式 1 基准。"""
import os, sys, json
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
FILE = sys.argv[1] if len(sys.argv) > 1 else "fit6.json"
P = dict(source=True) if FILE == "source" else dict(json.load(open(os.path.join(HERE, FILE)))["params"], v2=True)
SEEDS = tuple(range(100, 108))


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(seed):
    import replay as R, bench as B
    r = R.replay("base_empty.txt", P, seed=seed, settle=5.0)
    return None if r["fell_at"] is not None else B.metrics(r["twin"])


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    with Pool(8, initializer=init) as pool:
        ms = pool.map(one, SEEDS)
    ok = [m for m in ms if m is not None]
    print("参数:", {k: (round(v, 4) if abs(v) < 10 else round(v, 1)) for k, v in P.items() if k != "v2"})
    print("新种子 %d 个，摔倒 %d 个" % (len(SEEDS), len(SEEDS) - len(ok)))
    mean = B.mean_metrics(ok)
    lo = {k: float(np.min([m[k] for m in ok])) for k in ok[0]}
    hi = {k: float(np.max([m[k] for m in ok])) for k in ok[0]}
    B.table(real, [("孪生均值", mean), ("孪生最小", lo), ("孪生最大", hi)])
    print("各种子综合误差:", [round(B.score(m)[0], 3) for m in ok])
