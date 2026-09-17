# -*- coding: utf-8 -*-
"""当前源码孪生对全部 5 个真车录波的核对（只有 base_empty 参与过拟合）。"""
import os, sys
import numpy as np
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
FILES = ["base_empty.txt", "kd120.txt", "sweep_m1_300.txt", "sweep_A_300.txt",
         "sweep_A_500.txt", "sweep_300.txt"]

def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)

def one(name):
    import replay as R, metrics as M
    f = R.parse(name)
    real = f["real"]
    try:
        r = R.replay(name, dict(source=True), seed=100, settle=5.0)
    except Exception as e:
        return name, f["hdr"], None, None, repr(e)[:80]
    tw = r["twin"]
    def st(d):
        s = M.standstill(d["gyro"], d["ang"], d["ml"], d["el"], d["er"])
        return s
    return name, f["hdr"], st(real), st(tw), ("孪生第 %.1f s 摔了" % r["fell_at"]) if r["fell_at"] is not None else ""

if __name__ == "__main__":
    init()
    with Pool(6, initializer=init) as pool:
        res = pool.map(one, FILES)
    KEYS = ("ang_std", "gyro_rms", "gyro_pk", "pwm_abs", "pwm_sat", "pwm_rev", "peak_hz")
    print("%-17s %-4s" % ("文件", "kp/kd") + "".join("%9s" % k for k in KEYS))
    for name, hdr, real, tw, note in res:
        tag = "%d/%d" % (hdr["kp"] / 100, hdr["kd"])
        if real is None:
            print("%-17s %-6s 失败 %s" % (name, tag, note)); continue
        print("%-17s %-6s" % (name, tag) + "真车" + "".join("%9.2f" % real[k] for k in KEYS))
        print("%-24s" % "" + "孪生" + "".join("%9.2f" % tw[k] for k in KEYS) + "  " + note)
