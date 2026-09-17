# -*- coding: utf-8 -*-
"""从串口录真车数据，存成 .npz，顺便当个串口终端用。

为什么要这个：孪生至今复现不了真车那个 40 mm / 1.4 秒的来回摆（300 组
五参数联合搜索，幅值和倾角能对上，周期从来没低于 0.16 秒），结论是缺
**结构**不是缺参数。缺什么结构，光靠仿真猜不出来 —— 要真车的原始波形。

另外一件更紧的事：load_ctrl 的带通选在 26.5~45.5 Hz，是因为**孪生里**空车
抖动主频是 31 Hz。真车主频是多少没人量过。如果真车在 20 Hz 或 50 Hz，
那个带通会把信号滤掉，检测器直接失效。录一段做 FFT 就知道。

用法：
    # 列出串口
    python scripts/capture_uart.py --list

    # 交互终端（发命令、看文本遥测）
    python scripts/capture_uart.py COM5

    # 录 60 秒二进制原始数据（会自动发 "t 2"）
    python scripts/capture_uart.py COM5 --record 60 --out empty.npz

    # 看频谱
    python scripts/capture_uart.py --fft empty.npz

需要 pyserial：  pip install pyserial
"""
import argparse
import struct
import sys
import time

import numpy as np

BAUD = 115200
SYNC = b"\xa5\x5a"
REC = 10          # 每条记录 10 字节：2 同步 + 4 个 int16
EOL = bytes([13, 10])       # 字面写，别让转义层碰它
LF = bytes([10])
NEWLINE = chr(10)
GYRO_LSB_PER_DPS = 16.4


def list_ports():
    from serial.tools import list_ports as lp
    found = list(lp.comports())
    if not found:
        print("没找到串口")
        return
    for p in found:
        print(f"  {p.device:10s} {p.description}")


def decode(raw):
    """从字节流里按同步字提取记录。丢包后能自己重新对齐。"""
    out = []
    i = 0
    n = len(raw)
    while i + REC <= n:
        if raw[i] != 0xA5 or raw[i + 1] != 0x5A:
            i += 1                      # 失步，逐字节找回来
            continue
        g, a, ml, mr = struct.unpack_from("<hhhh", raw, i + 2)
        out.append((g, a / 100.0, ml, mr))
        i += REC
    return np.array(out, dtype=np.float64)


def record(port, seconds, out):
    import serial
    with serial.Serial(port, BAUD, timeout=0.1) as s:
        s.write(b"t 2\r\n")             # 切到二进制模式
        time.sleep(0.2)
        s.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            buf += s.read(4096)
            el = time.time() - t0
            print(f"\r  {el:5.1f}s / {seconds}s   {len(buf)/1024:7.1f} KB",
                  end="", flush=True)
        s.write(b"t 0\r\n")             # 关掉，免得一直刷
    print()
    d = decode(bytes(buf))
    if len(d) == 0:
        print("一条记录都没解出来。检查：板子上 dbg_mode 有没有生效、"
              "波特率是不是 115200、DBG_Poll() 有没有在 while(1) 里调")
        return
    lost = 1.0 - len(d) / (seconds * 200.0)
    np.savez(out, gyro_lsb=d[:, 0], pitch_deg=d[:, 1],
             motor_l=d[:, 2], motor_r=d[:, 3], fs=200.0)
    print(f"  {len(d)} 条记录 -> {out}   丢包率 {lost*100:.1f}%")
    if lost > 0.05:
        print("  丢包偏高：DBG_Poll() 可能调得不够勤，或者主循环里有阻塞的东西")
    summarize(out)


def summarize(path):
    z = np.load(path)
    g = z["gyro_lsb"]
    fs = float(z["fs"])
    y = g - g.mean()
    rms = y.std() / GYRO_LSB_PER_DPS
    sp = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), 1.0 / fs)
    k = int(np.argmax(sp[1:])) + 1
    print(f"\n  样本 {len(g)}  时长 {len(g)/fs:.1f}s")
    print(f"  陀螺 rms {rms:.1f} deg/s     主频 {fr[k]:.1f} Hz")
    band = (fr > 26.5) & (fr < 45.5)
    frac = sp[band].sum() / max(sp[1:].sum(), 1e-9)
    print(f"  26.5~45.5 Hz（load_ctrl 的带通）占总能量 {frac*100:.1f}%")
    if not (20.0 < fr[k] < 55.0):
        print(f"  ** 主频 {fr[k]:.1f} Hz 落在带通外。26.5~45.5 这个频段是按孪生的")
        print("     31 Hz 定的，真车不在这个位置的话，检测器的带通必须重算。")
    # --- 和孪生直接对得上的三个量 -------------------------------------
    # 录下来的 10 字节记录里本来就有 Motor_Left/Right，以前只用了陀螺。
    # 这三个量是孪生侧我已经在量的同一批，可以逐项对照：
    #   零力矩拍占比   孪生在「死区1500+补偿1300」下是 8%，而真车实测描述是
    #                  「电机基本安静、大部分拍恰好为 0」。差多少是硬指标。
    #   CCR 换向间隔   真车报的是 1.4s。孪生过补偿时 0.08s、不过补偿时 1.7~1.8s。
    #   CCR 幅值分布   继电器化的系统会两头跑，中间没有值。
    ml = z["motor_l"] if "motor_l" in z else None
    if ml is not None and len(ml):
        ml = ml.astype(float)
        print("")
        print("  --- 电机指令（和孪生逐项对照用）---")
        for dbv in (1300, 1500):
            frac0 = float(np.mean(np.abs(ml) < dbv)) * 100.0
            print(f"  |CCR| < {dbv}（电机不出力）占 {frac0:5.1f}%")
        d = np.diff(np.sign(ml))
        n_rev = int(np.count_nonzero(d != 0))
        if n_rev:
            print(f"  CCR 换向 {n_rev} 次，平均间隔 {len(ml)/fs/n_rev:.3f}s"
                  f"   （真车报 1.4s；孪生过补偿 0.08s、不过补偿 1.7~1.8s）")
        print(f"  |CCR| 中位数 {np.median(np.abs(ml)):.0f}"
              f"  95 分位 {np.percentile(np.abs(ml), 95):.0f}"
              f"  最大 {np.abs(ml).max():.0f}")
    print("\n  能量最强的五个频率：")
    for i in np.argsort(sp[1:])[::-1][:5] + 1:
        print(f"    {fr[i]:6.1f} Hz   {sp[i]/sp[1:].max()*100:5.1f}%")


# --- 真车上的增益阶梯 ---------------------------------------------------
# 孪生排不了「停振快慢、稳不稳」这种暂态质量 —— 实测上升时间差 3 倍、超调
# 从 0% 到 85%。所以最后一步必须在真车上做。这个函数就是把孪生里那套扫描
# 原样搬到真车上：每一格自动改增益、等它站定、收一段遥测、算同样的三个数。
# 区别只是数据来自真车。
#
# 收哪三个数，以及为什么：
#   rms  26~46Hz 陀螺能量  = 高频嗡嗡，也是自动切档的判据本身
#   osc  0.3~3Hz 位置能量  = 低频前后晃（要 LD_Pos 接上，否则恒 0）
#   rev  每秒换向次数      = 「来回窜」有多勤
# 只看 rms 会漏掉低频晃，只看 osc 会漏掉嗡嗡。两个都要。

LADDER = [          # (Balance_Kp, Velocity_Kp, Velocity_Ki)，物理单位
    (288.0, 82.0, 0.69),
    (256.0, 82.0, 0.69),
    (224.0, 82.0, 0.69),
    (208.0, 82.0, 0.69),
]


def parse_tele(line):
    """"<heavy> <rms> <osc> <rev> <pitch>" -> 五元组，解不出来返回 None。"""
    f = line.split()
    if len(f) != 5:
        return None
    try:
        return (int(f[0]), float(f[1]), float(f[2]), float(f[3]), float(f[4]))
    except ValueError:
        return None


def _cmd(ser, text):
    ser.write(text.encode("ascii") + EOL)
    time.sleep(0.05)


def tune(port, rungs, secs, force):
    """走一遍增益阶梯，每格收一段遥测。车站在地上，人在旁边看着。"""
    import serial

    print("连上 %s @ %d" % (port, BAUD))
    print("一共 %d 格，每格 4 秒站定 + %.0f 秒采集，合计约 %.1f 分钟"
          % (len(rungs), secs, len(rungs) * (secs + 6) / 60.0))
    print("档位锁在 %s（m %d）。自动切档这一步是关掉的 —— 先把一档调好，"
          "再验切换。" % ("NORMAL" if force == 1 else "HEAVY", force))
    print("车放地上扶稳，回车开始（Ctrl-C 随时停，停下会恢复第一格）：", end="")
    input()

    rows = []
    with serial.Serial(port, BAUD, timeout=0.5) as ser:
        try:
            for i, (kp, vkp, vki) in enumerate(rungs, 1):
                _cmd(ser, "t 0")
                _cmd(ser, "m %d" % force)
                _cmd(ser, "p %.0f" % (kp * 100))
                _cmd(ser, "v %.0f" % (vkp * 100))
                _cmd(ser, "i %.0f" % (vki * 100))
                print(NEWLINE + "[%d/%d] kp %.0f  vkp %.0f  vki %.2f   站定中..."
                      % (i, len(rungs), kp, vkp, vki), end="", flush=True)
                ser.reset_input_buffer()
                _cmd(ser, "t 1")
                time.sleep(4.0)                 # 让 osc/rms 的平滑器充上
                ser.reset_input_buffer()
                print(" 采集 %.0fs..." % secs, end="", flush=True)

                got, buf, t_end = [], b"", time.time() + secs
                while time.time() < t_end:
                    buf += ser.read(256)
                    while LF in buf:
                        one, buf = buf.split(LF, 1)
                        r = parse_tele(one.decode("ascii", "replace").strip())
                        if r:
                            got.append(r)
                if len(got) < secs * 5:         # 期望 10 Hz，收不到一半就是没站住
                    print(" 只收到 %d 行 —— 车摔了？串口掉了？" % len(got))
                    rows.append((kp, vkp, vki, None))
                    continue
                a = np.array(got, dtype=float)
                rows.append((kp, vkp, vki, dict(
                    rms=a[:, 1].mean(), osc=a[:, 2].mean(), rev=a[:, 3].mean(),
                    pitch=float(np.sqrt((a[:, 4] ** 2).mean())),
                    pk=float(np.abs(a[:, 4]).max()), n=len(got))))
                print(" 完成")
        except KeyboardInterrupt:
            print(NEWLINE + "中断")
        finally:
            _cmd(ser, "t 0")
            kp, vkp, vki = rungs[0]
            _cmd(ser, "p %.0f" % (kp * 100))
            _cmd(ser, "v %.0f" % (vkp * 100))
            _cmd(ser, "i %.0f" % (vki * 100))
            _cmd(ser, "m 0")
            print("已恢复到第一格 kp %.0f 并交回自动切档（m 0）" % kp)

    print(NEWLINE + "%5s%6s%6s |%8s%8s%7s |%9s%8s"
          % ("kp", "vkp", "vki", "rms", "osc", "rev/s", "倾角rms", "峰值"))
    for kp, vkp, vki, r in rows:
        if r is None:
            print("%5.0f%6.0f%6.2f |          没站住" % (kp, vkp, vki))
            continue
        print("%5.0f%6.0f%6.2f |%8.2f%8.1f%7.2f |%8.2f度%7.2f度"
              % (kp, vkp, vki, r["rms"], r["osc"], r["rev"], r["pitch"], r["pk"]))
    ok = [r for *_, r in rows if r is not None]
    if ok and all(r["osc"] == 0.0 for r in ok):
        print(NEWLINE + "osc 全是 0 —— LD_Pos() 没接上，低频晃这一列是空的。"
              "接法见 debug_uart.h 里那段 OPTIONAL。")
    print(NEWLINE + "把这张表贴回来，我按它选，而不是按孪生选。")


def _collect(ser, secs, force, label):
    """锁一档、等站定、收一段遥测，返回 ld_rms 均值。车摔了返回 None。"""
    _cmd(ser, "t 0")
    _cmd(ser, "m %d" % force)
    print("   %s 站定中..." % label, end="", flush=True)
    ser.reset_input_buffer()
    _cmd(ser, "t 1")
    time.sleep(4.0)                      # 能量平滑器 tau=0.30s，4s 足够充满
    ser.reset_input_buffer()
    print(" 采集 %.0fs..." % secs, end="", flush=True)
    got, buf, t_end = [], b"", time.time() + secs
    while time.time() < t_end:
        buf += ser.read(256)
        while LF in buf:
            one, buf = buf.split(LF, 1)
            r = parse_tele(one.decode("ascii", "replace").strip())
            if r:
                got.append(r)
    _cmd(ser, "t 0")
    if len(got) < secs * 5:
        print(" 只收到 %d 行 —— 摔了？" % len(got))
        return None
    a = np.array(got, dtype=float)
    v = float(a[:, 1].mean())
    print(" rms %.2f" % v)
    return v


def calibrate(port, secs, load_kg):
    """在真车上标定两个检测阈值。

    出厂的 LOADED_BELOW / EMPTY_ABOVE 全是孪生数，余量只有 1.7~2.6 倍。
    真车和孪生之间的差距比这个大得多 —— 电机死区一项就能让同一套增益从
    1/8 变 7/8。**所以自动切档上线前必须用真车的四个读数把它们换掉。**

    要的就是四个数：NORMAL/HEAVY 两档 x 空车/装货两种。
    """
    import serial

    print("连上 %s @ %d" % (port, BAUD))
    print("要采四段，每段 %.0f 秒，一共约 %.0f 分钟。"
          % (secs, 4 * (secs + 6) / 60.0 + 1))
    print("中途会让你装 %.1f kg 上去。" % load_kg)
    print("")
    print("注意：HEAVY 档用在空车上会**明显抖**（孪生里超过 120 deg/s，")
    print("原厂的 6 倍）。那是预期行为，也正是检测器要用的信号 —— 20/20")
    print("没摔过。但人要扶在旁边。")

    r = {}
    with serial.Serial(port, BAUD, timeout=0.5) as ser:
        try:
            print(NEWLINE + "[1/2] 空车。放地上扶稳，回车：", end="")
            input()
            r["n0"] = _collect(ser, secs, 1, "NORMAL 空车")
            r["h0"] = _collect(ser, secs, 2, "HEAVY  空车（会抖）")

            print(NEWLINE + "[2/2] 装上 %.1f kg，扶稳，回车：" % load_kg, end="")
            input()
            r["n2"] = _collect(ser, secs, 1, "NORMAL %.1fkg" % load_kg)
            r["h2"] = _collect(ser, secs, 2, "HEAVY  %.1fkg" % load_kg)
        except KeyboardInterrupt:
            print(NEWLINE + "中断")
        finally:
            _cmd(ser, "t 0")
            _cmd(ser, "m 0")
            print("已交回自动切档（m 0）")

    if any(r.get(k) is None for k in ("n0", "n2", "h0", "h2")):
        print(NEWLINE + "四个数没凑齐，不能算阈值。")
        return
    n0, n2, h0, h2 = r["n0"], r["n2"], r["h0"], r["h2"]
    print(NEWLINE + "%-22s %6.2f" % ("NORMAL 空车", n0))
    print("%-22s %6.2f" % ("NORMAL 装货", n2))
    print("%-22s %6.2f" % ("HEAVY  空车", h0))
    print("%-22s %6.2f" % ("HEAVY  装货", h2))

    # 单调性是这个方法能成立的前提，必须先验。不单调说明带通频段对这台车
    # 不对 —— 我在孪生里踩过一次：宽带高通下 4kg 的读数和空车一样高，车被
    # 判成「空」，留在 NORMAL，0/8 全摔。
    bad = []
    if not n0 > n2:
        bad.append("NORMAL 档装货后 rms 没降（%.2f -> %.2f）" % (n0, n2))
    if not h0 > h2:
        bad.append("HEAVY 档装货后 rms 没降（%.2f -> %.2f）" % (h0, h2))
    if bad:
        print(NEWLINE + "*** 不能用这组阈值 ***")
        for b in bad:
            print("  " + b)
        print("  判据本身在这台车上不成立。多半是 26~46 Hz 这个带通选错了")
        print("  —— 它是按孪生里 31 Hz 定的。先录频谱：")
        print("     python scripts" + chr(92) + "capture_uart.py %s --record 60 --out empty.npz"
              % (port,))
        print("     python scripts" + chr(92) + "capture_uart.py --fft empty.npz")
        return

    lo = (n0 * n2) ** 0.5          # 几何中点：两侧留同样比例的余量
    hi = (h0 * h2) ** 0.5
    mg = min(n0 / lo, lo / n2, h0 / hi, hi / h2)
    print(NEWLINE + "阈值取几何中点，余量 %.2fx" % mg)
    print("  敲这两条进去：")
    print("     l %.2f" % lo)
    print("     e %.2f" % hi)
    if mg < 1.5:
        print(NEWLINE + "  余量只有 %.2fx，偏薄。电池电压、地面、装货位置都会让"
              % mg)
        print("  这些读数漂。先在几种情况下各测一遍，取最坏的那组再算。")
    print(NEWLINE + "  改动不保存。测好了要写死，就改 load_ctrl.c 里的")
    print("  LD_LOADED_BELOW / LD_EMPTY_ABOVE 两行重新编译。")
    print(NEWLINE + "  然后 m 0 交回自动，装卸货各试几次，看 ? 里的 heavy 跟不跟得上。")


def terminal(port):
    import serial
    import threading
    print(f"连上 {port} @ {BAUD}。输入命令回车发送，? 看帮助，Ctrl-C 退出")
    with serial.Serial(port, BAUD, timeout=0.1) as s:
        def rx():
            while True:
                d = s.read(256)
                if d:
                    sys.stdout.write(d.decode("ascii", "replace"))
                    sys.stdout.flush()
        threading.Thread(target=rx, daemon=True).start()
        try:
            for line in sys.stdin:
                s.write(line.strip().encode("ascii") + b"\r\n")
        except KeyboardInterrupt:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", help="串口，比如 COM5")
    ap.add_argument("--list", action="store_true", help="列出串口")
    ap.add_argument("--record", type=float, metavar="秒", help="录二进制数据")
    ap.add_argument("--out", default="capture.npz")
    ap.add_argument("--fft", metavar="npz", help="只看已录文件的频谱")
    ap.add_argument("--tune", action="store_true",
                    help="走一遍增益阶梯，每格收一段遥测（在真车上做）")
    ap.add_argument("--rungs", metavar="kp:vkp:vki,...",
                    help="自定义阶梯，默认见 LADDER")
    ap.add_argument("--secs", type=float, default=20.0, help="每格采集秒数")
    ap.add_argument("--cal", action="store_true",
                    help="在真车上标定两个检测阈值（自动切档上线前必做）")
    ap.add_argument("--load", type=float, default=2.0,
                    help="标定时装多重，默认 2.0 kg（分界值）")
    ap.add_argument("--force", type=int, default=1, choices=(1, 2),
                    help="1=锁 NORMAL（默认）2=锁 HEAVY")
    a = ap.parse_args()

    if a.list:
        list_ports()
    elif a.fft:
        summarize(a.fft)
    elif a.cal:
        if not a.port:
            ap.error("标定要指定串口")
        calibrate(a.port, a.secs, a.load)
    elif a.tune:
        if not a.port:
            ap.error("调参要指定串口")
        rungs = LADDER
        if a.rungs:
            rungs = [tuple(float(x) for x in g.split(":"))
                     for g in a.rungs.split(",")]
            for g in rungs:
                if len(g) != 3:
                    ap.error("每一格要写成 kp:vkp:vki")
        tune(a.port, rungs, a.secs, a.force)
    elif a.record:
        if not a.port:
            ap.error("录数据要指定串口")
        record(a.port, a.record, a.out)
    elif a.port:
        terminal(a.port)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
