# -*- coding: utf-8 -*-
"""模式 1 静止基准（用户 2026-09-16 给的最标准基准）。

条件：原厂增益 Kp 9600 / Kd 48 / Vkp 6200 / Vki 31，死区补偿 1500，空车，静止，无扰动。
硬指标 = 三次复现都一致的量，给的是范围；落在范围内误差为 0，范围外按对数距离计。
每次会变的量（倾角均值、带符号 PWM 均值）不作目标，只报。

指标定义照用户的表：
  陀螺 RMS  含均值（sqrt(mean(g²))）
  位移      两轮编码器平均累加，1320 计数/圈，周长 210.49 mm
  编码器    单轮 |每 5 ms 计数| 的均值
"""
import numpy as np

FS = 200.0
MM_PER_COUNT = 210.49 / 1320.0

# (键, 下限, 上限, 权重, 说明)
TARGETS = [
    ("ang_std",   0.22, 0.27, 1.0, "倾角标准差 °"),
    ("gyro_rms",  8.6, 10.3, 1.0, "陀螺 RMS °/s"),
    ("gyro_pk",   27.0, 28.0, 1.0, "陀螺峰值 °/s"),
    ("gyro_mean", 1.98, 2.05, 1.0, "陀螺均值 °/s"),
    ("demand",    18.0, 48.0, 1.0, "|PWM| 均值 - 1500"),
    ("median_d",  33.0, 45.0, 1.0, "|PWM| 中位数 - 1500"),
    ("nz_max_d",  158.0, 158.0, 0.5, "|PWM| 非零最大 - 1500"),
    ("sat",       0.0, 0.0, 1.0, "饱和 %"),
    ("rev",       22.0, 28.8, 1.0, "换向率 次/s"),
    ("enc_abs",   0.37, 0.50, 1.0, "编码器 |均值| 计数/5ms（两轮平均）"),
    ("enc_max",   2.0, 2.0, 0.5, "编码器单帧 |最大| 计数"),
    ("disp_pp",   2.1, 3.3, 1.0, "位移峰峰 mm"),
    ("disp_std",  0.5, 0.6, 1.0, "位移标准差 mm"),
    ("f_gyro",    4.29, 4.29, 1.0, "角速度主频 Hz"),
    ("f_disp",    4.29, 4.29, 1.0, "位移主频 Hz"),
    # 频段（base_empty 主数据 ±15%）。边界取在 6 Hz：真车能量在 3-6 与 6-12 Hz
    # 各占一半（次峰 5.8 / 8.3 Hz），孪生曾经全挤在 3-6 Hz；旧的 3-8 / 8-16 Hz
    # 边界恰好把这个差异遮住了。另加位移超低频漂移（孪生曾是真车 3 倍）。
    ("g3_6",      4.45, 6.03, 1.0, "陀螺 3-6 Hz °/s"),
    ("g6_12",     4.60, 6.22, 1.0, "陀螺 6-12 Hz °/s"),
    ("g12_100",   3.24, 4.38, 1.0, "陀螺 12-100 Hz °/s"),
    ("d0_03",     0.12, 0.23, 1.0, "位移 0-0.3 Hz mm"),
]
INFO = [("ang_mean", "倾角均值 °（每次会变）"), ("pwm_signed", "带符号 PWM 均值（每次会变）")]


def _fill(x):
    x = np.array(x, dtype=float)
    ok = ~np.isnan(x)
    if ok.all():
        return x
    i = np.arange(len(x))
    x[~ok] = np.interp(i[~ok], i[ok], x[ok])
    return x


def _peak(y, fmin=1.0):
    y = y - y.mean()
    sp = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), 1.0 / FS)
    k = fr > fmin
    return float(fr[k][np.argmax(sp[k])])


def metrics(d, n=None):
    s = slice(0, n)
    g = _fill(d["gyro"][s]) / 16.4
    a = d["ang"][s]
    ml = d["ml"][s]
    ml = ml[~np.isnan(ml)]
    el, er = _fill(d["el"][s]), _fill(d["er"][s])
    ab = np.abs(ml)
    nz = ab[ab > 0]
    disp = np.cumsum((el + er) / 2.0) * MM_PER_COUNT
    sg = np.sign(ml[ml != 0])
    return dict(
        ang_std=float(np.nanstd(a)), ang_mean=float(np.nanmean(a)),
        gyro_rms=float(np.sqrt(np.mean(g ** 2))), gyro_pk=float(np.max(np.abs(g))),
        gyro_mean=float(np.mean(g)),
        demand=float(np.mean(ab)) - 1500.0, median_d=float(np.median(ab)) - 1500.0,
        nz_max_d=(float(nz.max()) - 1500.0) if len(nz) else 0.0,
        sat=float(np.mean(ab >= 2600) * 100), pwm_signed=float(np.mean(ml)),
        rev=float(np.count_nonzero(np.diff(sg) != 0)) / (len(ml) / FS),
        enc_abs=float((np.mean(np.abs(el)) + np.mean(np.abs(er))) / 2.0),
        enc_max=float(max(np.max(np.abs(el)), np.max(np.abs(er)))),
        disp_pp=float(disp.max() - disp.min()), disp_std=float(disp.std()),
        f_gyro=_peak(g), f_disp=_peak(disp),
        **_bands(g, disp))


def _dec(x, bands):
    y = x - x.mean()
    sp = np.abs(np.fft.rfft(y)) ** 2
    fr = np.fft.rfftfreq(len(y), 1.0 / FS)
    tot = sp[1:].sum()
    return [float(y.std() * np.sqrt(sp[(fr >= lo) & (fr < hi) & (fr > 0)].sum() / tot)) for lo, hi in bands]


def _bands(g, disp):
    a, b, c = _dec(g, ((3, 6), (6, 12), (12, 100)))
    (d,) = _dec(disp, ((0.0, 0.3),))
    return dict(g3_6=a, g6_12=b, g12_100=c, d0_03=d)

def term(key, v, lo, hi):
    """范围内 0；范围外对数距离（饱和、下限为 0 的量用线性距离）。"""
    if lo <= v <= hi:
        return 0.0
    if key == "sat":
        return v / 5.0
    edge = lo if v < lo else hi
    return abs(np.log(max(v, 1e-3) / max(edge, 1e-3)))


def score(m):
    tot, w = 0.0, 0.0
    parts = {}
    for key, lo, hi, wt, _ in TARGETS:
        e = term(key, m[key], lo, hi)
        parts[key] = e
        tot += wt * e
        w += wt
    return tot / w, parts


def mean_metrics(ms):
    return {k: float(np.mean([m[k] for m in ms])) for k in ms[0]}


def table(real, cols):
    """cols: [(标签, 指标字典)]"""
    print("%-34s %14s %9s" % ("指标", "实测范围", "主数据") + "".join("%12s" % lab for lab, _ in cols))
    for key, lo, hi, wt, name in TARGETS:
        rng = ("%.2f" % lo) if lo == hi else ("%.2f-%.2f" % (lo, hi))
        line = "%-34s %14s %9.2f" % (name, rng, real[key])
        for _, m in cols:
            ok = lo <= m[key] <= hi or (lo == hi and abs(m[key] - lo) / max(lo, 1) < 0.1)
            line += "%11.2f%s" % (m[key], " " if ok else "✗")
        print(line)
    for key, name in INFO:
        print("%-34s %14s %9.2f" % (name, "-", real[key]) + "".join("%12.2f" % m[key] for _, m in cols))
    print("%-34s %14s %9s" % ("综合误差（0 = 全部在范围内）", "", "%.3f" % score(real)[0])
          + "".join("%12.3f" % score(m)[0] for _, m in cols))
