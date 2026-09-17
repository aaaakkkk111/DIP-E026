# -*- coding: utf-8 -*-
"""修正卡尔曼零偏播种之后，第三轮参数 + 实测零偏在模式 1 基准上的表现。"""
import os, sys
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BASE = dict(stall=0.5717, damp=0.0105, irot=6.33e-4, dpos=1522.0, dneg=1529.48, delay=2.0253,
            gnoise=0.000842, k=11.19, c=0.0778, bl=0.5806, fric=0.0, ib=1.0, mnt=0.2804,
            gbias_dps=2.02, v2=True)


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    label, over, seed = args
    import replay as R, bench as B
    r = R.replay("base_empty.txt", dict(BASE, **over), seed=seed, settle=5.0)
    return label, (None if r["fell_at"] is not None else B.metrics(r["twin"]))


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    cases = [("第三轮参数+零偏", {}), ("同上 去掉零偏", dict(gbias_dps=0.0))]
    jobs = [(lab, ov, s) for lab, ov in cases for s in range(4)]
    with Pool(8, initializer=init) as pool:
        res = pool.map(one, jobs)
    cols = []
    for lab, _ in cases:
        ms = [m for l, m in res if l == lab and m is not None]
        cols.append((lab, B.mean_metrics(ms)))
    B.table(real, cols)
