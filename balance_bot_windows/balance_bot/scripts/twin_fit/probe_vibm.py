# -*- coding: utf-8 -*-
"""第九轮（修正摩擦后 = 源码）上加转子 Stribeck 摩擦，网格扫一遍。4 个新种子。"""
import os, sys, json
import numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BASE = dict(json.load(open(os.path.join(HERE, "fit9b.json")))["params"], v2=True)
SEEDS = (300, 301, 302, 303, 304, 305)
CASES = [("基线 fit9b", {})]
for frac in (0.0, 0.3, 0.6):
    for vibm in (0.01, 0.03, 0.08):
        CASES.append(("恒定x%.1f 速度%.2f" % (frac, vibm), dict(vib=BASE["vib"] * frac, vibm=vibm)))
KEYS = ("ang_std", "gyro_pk", "demand", "rev", "enc_abs", "g3_6", "g6_12", "g12_100", "disp_std", "f_gyro")

def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)

def one(a):
    lab, over, seed = a
    import replay as R, bench as B
    r = R.replay("base_empty.txt", dict(BASE, **over), seed=seed, settle=5.0)
    return lab, (None if r["fell_at"] is not None else B.metrics(r["twin"]))

if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    keys = [k for k in KEYS if k in real]
    with Pool(16, initializer=init) as pool:
        res = pool.map(one, [(l, o, s) for l, o in CASES for s in SEEDS])
    print("%-18s" % "情形" + "".join("%9s" % k for k in keys) + "%8s" % "综合")
    print("%-18s" % "真车" + "".join("%9.2f" % real[k] for k in keys))
    for lab, _ in CASES:
        ms = [m for l, m in res if l == lab and m is not None]
        if not ms:
            print(lab, "全摔"); continue
        m = B.mean_metrics(ms)
        print("%-18s" % lab + "".join("%9.2f" % m[k] for k in keys) + "%8.3f" % B.score(m)[0] + ("" if len(ms) == len(SEEDS) else " 摔%d" % (len(SEEDS) - len(ms))))
