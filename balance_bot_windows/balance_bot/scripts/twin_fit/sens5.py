# -*- coding: utf-8 -*-
"""当前最优点附近逐参数灵敏度，4 个种子。看谁能压住 4.6 Hz 主极限环幅值又不丢高频。"""
import os, sys, json
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BEST = json.load(open(os.path.join(HERE, "fit5.json")))["params"]
SEEDS = (10, 11, 12, 13)
SCAN = {
    "vib":    [0.0, 0.01, 0.02, 0.035, 0.05],
}
_UNUSED = {
    "dpos":   [1440, 1455, 1468.5, 1480, 1490],
    "dneg":   [1470, 1485, 1501.9, 1515, 1530],
    "delay":  [1.0, 1.5, 1.93, 2.5, 3.0],
    "k":      [2.5, 3.2, 3.88, 6.0, 11.0],
    "gnoise": [0.0005, 0.001, 0.004, 0.01, 0.02],
    "bl":     [0.3, 0.6, 0.94, 1.3, 1.8],
    "stall":  [0.30, 0.38, 0.46, 0.55, 0.65],
    "c":      [0.03, 0.07, 0.13, 0.25, 0.4],
}
KEYS = ("ang_std", "b_3_8", "b_8_16", "rev", "gyro_pk", "disp_std", "f_gyro")


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    key, val, seed = args
    import replay as R, bench as B
    p = dict(BEST, v2=True)
    p.setdefault("vib", 0.0)
    p[key] = val
    r = R.replay("base_empty.txt", p, seed=seed, settle=5.0)
    return args, (None if r["fell_at"] is not None else B.metrics(r["twin"]))


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    jobs = [(k, v, s) for k, vals in SCAN.items() for v in vals for s in SEEDS]
    with Pool(18, initializer=init) as pool:
        res = pool.map(one, jobs)
    print("%-8s %8s " % ("参数", "值") + "".join("%9s" % k for k in KEYS) + "%8s" % "综合")
    print("%-17s " % "真车" + "".join("%9.2f" % real[k] for k in KEYS) + "%8.3f" % B.score(real)[0])
    for k, vals in SCAN.items():
        for v in vals:
            ms = [m for (kk, vv, s), m in res if kk == k and vv == v and m is not None]
            if len(ms) < len(SEEDS):
                print("%-8s %8g  摔了 %d/%d" % (k, v, len(SEEDS) - len(ms), len(SEEDS)))
                if not ms:
                    continue
            m = B.mean_metrics(ms)
            mark = ""
            print("%-8s %8g " % (k, v) + "".join("%9.2f" % m[kk] for kk in KEYS) + "%8.3f%s" % (B.score(m)[0], mark))
        print()
