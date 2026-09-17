# -*- coding: utf-8 -*-
"""阳性对照：源码默认值构造的孪生，base_empty 回放多个噪声种子，和真车、拟合时的单种子结果比。"""
import os, sys
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
KEYS = ("ang_std", "gyro_rms", "pwm_abs", "pwm_sat", "pwm_rev", "enc_abs", "pos_std", "peak_hz")


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(seed):
    import replay as R, metrics as M
    r = R.replay("base_empty.txt", dict(source=True), seed=seed)
    t = r["twin"]
    m = M.standstill(t["gyro"], t["ang"], t["ml"], t["el"], t["er"])
    return [m[k] for k in KEYS] + list(m["bands"][:3])


if __name__ == "__main__":
    init()
    import replay as R, metrics as M
    f = R.parse("base_empty.txt")
    real = M.standstill(f["real"]["gyro"], f["real"]["ang"], f["real"]["ml"], f["real"]["el"], f["real"]["er"])
    with Pool(8, initializer=init) as pool:
        rows = np.array(pool.map(one, range(8)))
    fit_single = [0.26, 8.76, 1533.87, 0.0, 14.67, 0.21, 35.88, 7.96, 0.76, 6.95, 4.58]
    names = list(KEYS) + ["0.4-3Hz", "3-8Hz", "8-16Hz"]
    print("源码默认孪生，base_empty 回放 8 个噪声种子")
    print("%-10s %9s %20s %18s %13s" % ("指标", "真车", "源码 均值±标准差", "最小-最大", "拟合时单种子"))
    for i, k in enumerate(names):
        rv = real[k] if i < len(KEYS) else real["bands"][i - len(KEYS)]
        col = rows[:, i]
        print("%-10s %9.2f %11.2f ± %-7.2f %8.2f - %-8.2f %11.2f" % (k, rv, col.mean(), col.std(), col.min(), col.max(), fit_single[i]))
