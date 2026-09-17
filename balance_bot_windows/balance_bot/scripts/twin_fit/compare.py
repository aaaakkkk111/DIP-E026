# -*- coding: utf-8 -*-
"""回放对比：一组或几组孪生参数，逐文件、逐项对真车。

验证清单（用户给的）：
  base_empty   倾角 std 0.25 / 陀螺 8.6 / 主频 4.29 / |PWM| 1532 / 换向 28.8 / 饱和 0
  kd120        倾角 std 3.95 / 陀螺 99.2 / 主频 13.96 / 饱和 65.9
  sweep_A      共振 12 Hz(300) 11 Hz(500)；增益比 1.5-3.5 Hz ≈1，4-15 Hz 压到 0.63，16 Hz 以上 2.8-3.0
  sweep_300    Kp 38400 必须失稳
"""
import numpy as np
import metrics as M
import replay as R

FR_F = tuple(np.arange(1.0, 24.5, 0.5))
STAT = ("ang_mean", "ang_std", "gyro_rms", "gyro_pk", "pwm_abs", "pwm_sat", "pwm_rev",
        "enc_abs", "pos_std", "peak_hz")


def stats(d, n=None):
    s = slice(0, n)
    return M.standstill(d["gyro"][s], d["ang"][s], d["ml"][s], d["el"][s], d["er"][s])


def fr(d, chirp, n=None):
    s = slice(0, n)
    return M.fresp(d["inj"][s], d["gyro"][s], fmax=chirp[2], f_of_t=FR_F)


def features(f3, f5):
    g3 = np.array([x[1] for x in f3]); g5 = np.array([x[1] for x in f5])
    F = np.array(FR_F)
    ratio = g5 / g3

    def rmean(lo, hi):
        m = (F >= lo) & (F <= hi)
        return float(np.nanmean(ratio[m]))
    m415 = (F >= 4) & (F <= 15)
    return dict(peak300=float(F[np.nanargmax(g3)]), peak500=float(F[np.nanargmax(g5)]),
                lin=rmean(1.5, 3.5), comp=float(np.nanmin(ratio[m415])), exp=rmean(16, 24))


def unstable(d, n=None):
    a = d["ang"][:n]
    over = np.nonzero(np.abs(np.nan_to_num(a)) > 30)[0]
    ml = d["ml"][:n]
    ml = ml[~np.isnan(ml)]
    return dict(t30=(float(over[0] / 200.0) if len(over) else None),
                sat=float(np.mean(np.abs(ml) >= 2600) * 100) if len(ml) else float("nan"),
                ang_std=float(np.nanstd(a)))


def run_all(p, files=("base_empty.txt", "sweep_A_300.txt", "sweep_A_500.txt", "kd120.txt", "sweep_300.txt")):
    out = {}
    for nm in files:
        out[nm] = R.replay(nm, p)
    return out


def report(sets):
    """sets: [(标签, 参数字典)]"""
    runs = [(lab, run_all(p)) for lab, p in sets]
    ref = runs[0][1]

    def col(v, w=11):
        return ("%*s" % (w, "-")) if v is None or (isinstance(v, float) and np.isnan(v)) else ("%*.2f" % (w, v))
    for nm in ("base_empty.txt", "kd120.txt"):
        f = ref[nm]["file"]
        rs = stats(f["real"])
        print("\n=== %s（kd %.0f，电池 %.1f V）===" % (nm, f["hdr"]["kd"], f["hdr"]["vbat"]))
        print("%-10s %11s" % ("指标", "真车") + "".join("%11s" % lab for lab, _ in runs))
        tws = []
        for lab, rr in runs:
            r = rr[nm]
            n = r["fell_at"]
            tws.append(None if (n is not None and n < 2000) else stats(r["twin"], n))
        for k in STAT:
            print("%-10s %11.2f" % (k, rs[k]) + "".join(col(None if t is None else t[k]) for t in tws))
        for i, (lo, hi) in enumerate(M.BANDS):
            print("%-10s %11.2f" % ("%g-%gHz" % (lo, hi), rs["bands"][i]) +
                  "".join(col(None if t is None else t["bands"][i]) for t in tws))
        print("%-10s %11s" % ("摔倒", "否") + "".join(
            "%11s" % ("否" if rr[nm]["fell_at"] is None else "%.1fs" % (rr[nm]["fell_at"] / 200)) for _, rr in runs))

    print("\n=== 扫频（幅值 300 / 500），回放 inj 列 ===")
    fr_real = {}
    fr_tw = {lab: {} for lab, _ in runs}
    corr = {lab: {} for lab, _ in runs}
    for nm in ("sweep_A_300.txt", "sweep_A_500.txt"):
        f = ref[nm]["file"]
        fr_real[nm] = fr(f["real"], f["chirp"])
        for lab, rr in runs:
            r = rr[nm]
            n = r["fell_at"]
            fr_tw[lab][nm] = None if n is not None else fr(r["twin"], f["chirp"])
            a = f["real"]["gyro"]; b = r["twin"]["gyro"]
            act = np.nonzero(f["inj"] != 0)[0]
            s = slice(act[0], act[-1])
            ok = ~np.isnan(a[s]) & ~np.isnan(b[s])
            corr[lab][nm] = float(np.corrcoef(a[s][ok], b[s][ok])[0, 1]) if ok.sum() > 100 else float("nan")
    print("%5s %13s" % ("Hz", "真车300") + "".join("%14s" % (lab + "300") for lab, _ in runs)
          + " | %13s" % "真车500" + "".join("%14s" % (lab + "500") for lab, _ in runs))
    for i, fq in enumerate(FR_F):
        if fq not in (1, 2, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16, 18, 20, 22, 24):
            continue
        line = "%5.1f %6.0f %5.0f°" % (fq, fr_real["sweep_A_300.txt"][i][1], fr_real["sweep_A_300.txt"][i][2])
        for lab, _ in runs:
            t = fr_tw[lab]["sweep_A_300.txt"]
            line += "%14s" % ("摔了" if t is None else "%6.0f %5.0f°" % (t[i][1], t[i][2]))
        line += " | %6.0f %5.0f°" % (fr_real["sweep_A_500.txt"][i][1], fr_real["sweep_A_500.txt"][i][2])
        for lab, _ in runs:
            t = fr_tw[lab]["sweep_A_500.txt"]
            line += "%14s" % ("摔了" if t is None else "%6.0f %5.0f°" % (t[i][1], t[i][2]))
        print(line)

    def err(a, b):
        g = np.nanmean([abs(np.log(max(y[1], 1e-3) / max(x[1], 1e-3))) for x, y in zip(a, b)])
        ph = np.nanmean([abs((y[2] - x[2] + 180) % 360 - 180) for x, y in zip(a, b)])
        return g, ph
    fe_r = features(fr_real["sweep_A_300.txt"], fr_real["sweep_A_500.txt"])
    print("\n%-26s %9s" % ("扫频特征", "真车") + "".join("%11s" % lab for lab, _ in runs))
    rows = (("共振频率 幅值300 (Hz)", "peak300"), ("共振频率 幅值500 (Hz)", "peak500"),
            ("增益比500/300 1.5-3.5Hz", "lin"), ("增益比 4-15Hz 最低", "comp"), ("增益比 16-24Hz 平均", "exp"))
    fes = []
    for lab, _ in runs:
        t3, t5 = fr_tw[lab]["sweep_A_300.txt"], fr_tw[lab]["sweep_A_500.txt"]
        fes.append(None if (t3 is None or t5 is None) else features(t3, t5))
    for title, k in rows:
        print("%-26s %9.2f" % (title, fe_r[k]) + "".join(col(None if fe is None else fe[k]) for fe in fes))
    for nm in ("sweep_A_300.txt", "sweep_A_500.txt"):
        line_g = "%-26s %9s" % ("增益误差 " + nm[6:-4], "")
        line_p = "%-26s %9s" % ("相位误差(度) " + nm[6:-4], "")
        line_c = "%-26s %9s" % ("陀螺逐拍相关 " + nm[6:-4], "")
        for lab, _ in runs:
            t = fr_tw[lab][nm]
            if t is None:
                line_g += "%11s" % "-"; line_p += "%11s" % "-"
            else:
                g, ph = err(fr_real[nm], t)
                line_g += "%11.3f" % g; line_p += "%11.1f" % ph
            line_c += "%11.3f" % corr[lab][nm]
        print(line_g); print(line_p); print(line_c)

    nm = "sweep_300.txt"
    f = ref[nm]["file"]
    ur = unstable(f["real"])
    print("\n=== 失稳边界 sweep_300（Kp 38400 / Vkp 11000 / Vki 92，电池 %.1f V）===" % f["hdr"]["vbat"])
    print("%-22s %11s" % ("", "真车") + "".join("%11s" % lab for lab, _ in runs))
    print("%-22s %11s" % ("首次 |倾角|>30° (s)", "%.1f" % ur["t30"] if ur["t30"] is not None else "从未")
          + "".join("%11s" % (lambda u: "从未" if u["t30"] is None else "%.1f" % u["t30"])(unstable(rr[nm]["twin"], rr[nm]["fell_at"])) for _, rr in runs))
    print("%-22s %11.1f" % ("饱和 %", ur["sat"]) + "".join(col(unstable(rr[nm]["twin"], rr[nm]["fell_at"])["sat"]) for _, rr in runs))
    print("%-22s %11.2f" % ("倾角 std", ur["ang_std"]) + "".join(col(unstable(rr[nm]["twin"], rr[nm]["fell_at"])["ang_std"]) for _, rr in runs))
    print("%-22s %11s" % ("孪生摔倒于 (s)", "-") + "".join(
        "%11s" % ("未摔" if rr[nm]["fell_at"] is None else "%.1f" % (rr[nm]["fell_at"] / 200)) for _, rr in runs))
    return runs
