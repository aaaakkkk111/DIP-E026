# -*- coding: utf-8 -*-
"""现在的孪生在模式 1 静止基准上的表现（源码默认，多种子）。"""
import os, sys
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    seed, bias_on = args
    import replay as R, bench as B
    import balance_bot.firmware.twin_baseline as TB
    if not bias_on:
        TB.GYRO_BIAS_RAD_S = 0.0
    r = R.replay("base_empty.txt", dict(source=True), seed=seed, settle=5.0)
    return B.metrics(r["twin"])


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    f = R.parse("base_empty.txt")
    real = B.metrics(f["real"])
    with Pool(8, initializer=init) as pool:
        no_bias = B.mean_metrics(pool.map(one, [(s, False) for s in range(4)]))
        bias = B.mean_metrics(pool.map(one, [(s, True) for s in range(4)]))
    print("模式 1 静止基准：真车 vs 当前孪生（4 个噪声种子平均）")
    B.table(real, [("孪生无零偏", no_bias), ("孪生+零偏", bias)])
