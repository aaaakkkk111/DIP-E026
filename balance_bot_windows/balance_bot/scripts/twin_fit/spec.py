# -*- coding: utf-8 -*-
"""真车 vs 源码孪生：陀螺 / PWM / 编码器的细分频谱（1 Hz 一格），看 6-12 Hz 能量是什么。"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)
import replay as R, bench as B

def bands(x, edges):
    x = B._fill(np.asarray(x, float)); y = x - x.mean()
    sp = np.abs(np.fft.rfft(y)) ** 2; fr = np.fft.rfftfreq(len(y), 1 / 200.0); tot = sp[1:].sum()
    return [y.std() * np.sqrt(sp[(fr >= lo) & (fr < hi)].sum() / tot) for lo, hi in zip(edges[:-1], edges[1:])]

def peaks(x, n=6):
    x = B._fill(np.asarray(x, float)); y = (x - x.mean()) * np.hanning(len(x))
    sp = np.abs(np.fft.rfft(y)); fr = np.fft.rfftfreq(len(y), 1 / 200.0)
    m = (fr > 1) & (fr < 30); f, s = fr[m], sp[m]
    idx = [i for i in range(1, len(s) - 1) if s[i] > s[i-1] and s[i] > s[i+1]]
    idx = sorted(idx, key=lambda i: -s[i])[:n]
    return sorted((round(f[i], 2), round(s[i] / s.max(), 2)) for i in idx)

edges = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 20, 30]
real = R.parse("base_empty.txt")["real"]
tw = R.replay("base_empty.txt", dict(source=True), seed=100, settle=5.0)["twin"]
for key, scale in (("gyro", 1 / 16.4), ("ml", 1.0)):
    print("==", key, "频带 RMS", edges)
    for lab, d in (("真车", real), ("孪生", tw)):
        print("%-4s" % lab, " ".join("%5.2f" % v for v in bands(np.asarray(d[key]) * scale, edges)))
    for lab, d in (("真车", real), ("孪生", tw)):
        print("%-4s 峰" % lab, peaks(np.asarray(d[key]) * scale))
print("real n", len(real["gyro"]))
# 时域片段：换向间隔分布
for lab, d in (("真车", real), ("孪生", tw)):
    ml = B._fill(np.asarray(d["ml"], float)); s = np.sign(ml); s = s[s != 0]
    runs = np.diff(np.flatnonzero(np.diff(s) != 0))
    h = np.bincount(np.minimum(runs, 30), minlength=31)
    print(lab, "同号段长度(拍)分布 1..30:", list(h[1:]))
