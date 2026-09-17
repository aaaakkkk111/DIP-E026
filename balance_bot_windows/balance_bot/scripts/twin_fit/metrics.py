# -*- coding: utf-8 -*-
"""真车 CSV 和孪生输出**用同一段代码**算指标。

真车文件：#seq,gyro,ang_c,ml,mr,el,er,inj（200 Hz 一拍）
  gyro  = Gyro_Balance 原始 LSB
  ang_c = Angle_Balance * 100
  ml/mr = 死区补偿 + 限幅之后写进 PWM 的值（电机关着时记 0）
  el/er = 本拍编码器增量
  inj   = 注入量（加在补偿之前）
丢包靠 seq 找回：缺的拍保留为 NaN，频域分析时只用连续段或插值。
"""
import os

import numpy as np

# 真车录波的位置。默认找工程同级的 real_data/，也可以用环境变量指过去：
#   set E026_REAL_DATA=<...>/deadband_test
# The real-car recordings; override with the E026_REAL_DATA env var.
DIR = os.environ.get("E026_REAL_DATA", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "real_data"))
if not DIR.endswith(("/", "\\")):
    DIR += "/"
FS = 200.0


def load(name):
    rows, hdr = [], ""
    for line in open(DIR + name, encoding="ascii", errors="replace"):
        line = line.strip()
        if line.startswith("#start"):
            hdr = line
        if not line or line[0] == "#":
            continue
        p = line.split(",")
        if len(p) != 8:
            continue
        try:
            rows.append([int(x) for x in p])
        except ValueError:
            continue
    a = np.array(rows, dtype=float)
    seq = a[:, 0].astype(int)
    # seq 是 16 位，可能回绕
    for i in range(1, len(seq)):
        while seq[i] < seq[i - 1]:
            seq[i:] += 65536
    n = seq[-1] - seq[0] + 1
    out = np.full((n, 7), np.nan)
    out[seq - seq[0]] = a[:, 1:]
    cols = dict(gyro=0, ang=1, ml=2, mr=3, el=4, er=5, inj=6)
    d = {k: out[:, i] for k, i in cols.items()}
    d["ang"] = d["ang"] / 100.0
    d["hdr"] = hdr
    d["drop"] = 1.0 - len(a) / n
    return d


def fill(x):
    """线性插值补丢包，只用于需要等间隔的频域计算。"""
    x = np.array(x, dtype=float)
    ok = ~np.isnan(x)
    if ok.all():
        return x
    idx = np.arange(len(x))
    x[~ok] = np.interp(idx[~ok], idx[ok], x[ok])
    return x


BANDS = ((0.4, 3), (3, 8), (8, 16), (16, 25), (26.5, 45.5))


def standstill(gyro, ang, ml, el, er):
    """静止站立指标。gyro 为原始 LSB，ang 为度。"""
    g = fill(gyro) / 16.4
    y = g - np.mean(g)
    sp = np.abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2
    fr = np.fft.rfftfreq(len(y), 1.0 / FS)
    tot = sp[1:].sum()
    bands = [float(y.std() * np.sqrt(sp[(fr >= lo) & (fr < hi)].sum() / tot))
             for lo, hi in BANDS]
    m = ~np.isnan(ml)
    mlv = ml[m]
    pos = np.nancumsum(fill(el) + fill(er))           # 两轮计数之和，累加 = 位置
    # 主频只在 0.5 Hz 以上找（避开位置漂移）
    k = (fr > 0.5)
    pk = float(fr[k][np.argmax(sp[k])])
    return dict(
        ang_mean=float(np.nanmean(ang)), ang_std=float(np.nanstd(ang)),
        ang_pp=float(np.nanmax(ang) - np.nanmin(ang)),
        gyro_rms=float(y.std()), gyro_pk=float(np.nanmax(np.abs(gyro)) / 16.4),
        pwm_abs=float(np.mean(np.abs(mlv))), pwm_sat=float(np.mean(np.abs(mlv) >= 2600) * 100),
        pwm_rev=float(np.count_nonzero(np.diff(np.sign(mlv[mlv != 0])) != 0) / (len(mlv) / FS)),
        enc_abs=float(np.nanmean(np.abs(el))),
        pos_pp=float(pos.max() - pos.min()),        # 计数（两轮之和）
        pos_std=float(pos.std()),
        peak_hz=pk, bands=bands)


def fresp(inj, gyro, fmin=0.5, fmax=25.0, f_of_t=None, win_s=1.0):
    """扫频注入 -> 陀螺的频响。按瞬时频率分段做单频相关。"""
    u = fill(inj); y = fill(gyro) / 16.4
    n = len(u)
    t = np.arange(n) / FS
    active = np.nonzero(u != 0)[0]
    t0 = active[0]
    res = []
    W = int(win_s * FS)
    for f in f_of_t:
        # 线性扫频：f = f0 + (f1-f0)*tau/20
        tau = (f - 0.5) / (fmax - 0.5) * 20.0
        c = t0 + int(tau * FS)
        a, b = max(t0, c - W // 2), min(n, c + W // 2)
        if b - a < W // 2:
            res.append((f, np.nan, np.nan)); continue
        # 用瞬时相位做参考：直接对 u 和 y 做同一复指数相关（u 本身就是正弦）
        k = np.exp(-2j * np.pi * f * t[a:b])
        U = (u[a:b] * k).sum(); Y = (y[a:b] * k).sum()
        G = Y / U
        res.append((f, abs(G) * 1000.0, float(np.degrees(np.angle(G)))))
    return res


if __name__ == "__main__":
    for nm in ("base_empty.txt", "sweep_A_300.txt", "sweep_A_500.txt", "sweep_m1_300.txt"):
        d = load(nm)
        print(nm, "丢包 %.1f%%" % (d["drop"] * 100), "拍数", len(d["gyro"]))
        print("  ", d["hdr"])
    d = load("base_empty.txt")
    m = standstill(d["gyro"], d["ang"], d["ml"], d["el"], d["er"])
    for k, v in m.items():
        print("  %-10s %s" % (k, np.round(v, 3)))
