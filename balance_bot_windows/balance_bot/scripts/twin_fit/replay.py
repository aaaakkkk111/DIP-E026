# -*- coding: utf-8 -*-
"""原样回放：用文件自己记下的输入驱动孪生，输出和实测同一格式。

每个文件的输入：
  增益      #start 行 kp/kd/vkp/vki（x100）；转向和中值按模式取（模式 1 = Normal
            17/0.20、中值 0；其它模式 = load_ctrl 写死的 14/0.20、中值 0）
  死区补偿  #start 行 dead=
  电池      #start 行 vbat=
  扰动      inj 列，逐拍喂进 PID 输出（死区补偿之前，同 app_control.c:149）
            丢包的行按 tune_io.c TUNE_Inject() 的 float32 算法补回，
            并和文件里现存的值逐个核对
行对齐：文件第 i 行 = 固件第 i 拍：本拍读到的 Gyro_Balance / Angle_Balance /
编码器增量，本拍算出的 Motor_Left（含本拍 inj，补偿后限幅后，电机关时记 0）。
"""
import os
import re
import numpy as np

# 真车录波的位置。默认找工程同级的 real_data/，也可以用环境变量指过去：
#   set E026_REAL_DATA=<...>/deadband_test
# The real-car recordings; override with the E026_REAL_DATA env var.
DIR = os.environ.get("E026_REAL_DATA", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "real_data"))
if not DIR.endswith(("/", "\\")):
    DIR += "/"
FS = 200.0
COLS = ("gyro", "ang", "ml", "mr", "el", "er", "inj")


def parse(name):
    hdr, chirp, rows = {}, None, []
    for line in open(DIR + name, encoding="ascii", errors="replace"):
        line = line.strip()
        if line.startswith("#start"):
            hdr = {k: float(v) for k, v in re.findall(r"(\w+)=([-\d.]+)", line)}
            hdr["gear_is_normal"] = 1 if "gear=NORMAL" in line else 0
        m = re.match(r"#chirp amp=(\d+) ([\d.]+)-([\d.]+)Hz", line)
        if m:
            chirp = (int(m.group(1)), float(m.group(2)), float(m.group(3)))
        if not line or line[0] == "#":
            continue
        p = line.split(",")
        if len(p) == 8:
            try:
                rows.append([int(x) for x in p])
            except ValueError:
                pass
    a = np.array(rows, dtype=float)
    seq = a[:, 0].astype(int)
    # 板子中途复位时 seq 从 0 重来：只取第一段连续记录
    cut = len(seq)
    for i in range(1, len(seq)):
        if seq[i] < seq[i - 1] - 30000:
            seq[i:] += 65536
        elif seq[i] < seq[i - 1]:
            cut = i
            break
    a, seq = a[:cut], seq[:cut]
    n = seq[-1] - seq[0] + 1
    real = {k: np.full(n, np.nan) for k in COLS}
    for j, k in enumerate(COLS):
        real[k][seq - seq[0]] = a[:, j + 1]
    real["ang"] /= 100.0
    inj = real["inj"].copy()
    check = None
    if chirp:
        amp, f0, f1 = chirp
        s0 = int(np.nonzero(np.nan_to_num(inj) != 0)[0][0])
        gen = np.zeros(n)
        ph = np.float32(0.0)
        two_pi = np.float32(6.2831853)
        for t in range(4000):
            if s0 + t >= n:
                break
            f = np.float32(f0) + np.float32(f1 - f0) * (np.float32(t) / np.float32(4000))
            ph = np.float32(ph + two_pi * f / np.float32(FS))
            if ph > two_pi:
                ph = np.float32(ph - two_pi)
            gen[s0 + t] = int(np.float32(amp) * np.float32(np.sin(np.float64(ph))))
        have = ~np.isnan(inj)
        check = (int(np.sum(gen[have] == inj[have])), int(have.sum()))
        inj = gen
    else:
        inj = np.nan_to_num(inj)
    return dict(hdr=hdr, chirp=chirp, real=real, inj=inj, n=n, check=check, rows=cut)


def gains_for(hdr):
    from balance_bot.firmware.controllers import pid_gains
    g = pid_gains("Normal")
    g.update(balance_kp=hdr["kp"] / 100.0, balance_kd=hdr["kd"] / 100.0,
             velocity_kp=hdr["vkp"] / 100.0, velocity_ki=hdr["vki"] / 100.0)
    if int(hdr["mode"]) != 1:          # load_ctrl 模式：转向写死 1400/20
        g.update(turn_kp=14.0, turn_kd=0.20)
    return g


def replay(name, p, settle=3.0, max_ticks=None, boot_heavy=None, seed=0):
    """p：参数字典（见 params_model.py）。返回和 real 同格式的孪生输出。"""
    import params_model as PM
    from balance_bot.firmware.twin_baseline import make_mujoco_twin, MODE_STM32_PID, CAR_STOP
    from balance_bot.firmware.robot import STM32_FIRMWARE_CONST as FC
    f = parse(name)
    hdr = f["hdr"]
    object.__setattr__(FC, "pwm_deadband_comp", int(hdr["dead"]))
    n = f["n"] if max_ticks is None else min(f["n"], max_ticks)
    kw, post = PM.build(p)
    core = make_mujoco_twin(firmware=MODE_STM32_PID, gains=gains_for(hdr), imu_filter="kalman",
                            randomize=False, battery_v=hdr.get("vbat", 12.0),
                            episode_seconds=settle + n / FS + 10, **kw)
    post(core)
    core.reset(seed=seed)
    core.set_car_state(CAR_STOP)
    # 负载模式（22-25）：固件开机 LD_Reset() 默认 HEAVY，检测器之后才切档，
    # 录制期间检测器一直开着。孪生照同样的流程走，并把录制起点推迟到
    # 检测器切到文件头记录的档位（gear=）之后。
    load_mode = int(hdr["mode"]) != 1 if boot_heavy is None else boot_heavy
    from balance_bot.firmware.load_sched import NORMAL, HEAVY
    if load_mode:
        # 孪生的检测器阈值是按孪生的抖动标的，会判错档（实测：孪生一直留在
        # HEAVY）。所以不跑孪生的检测器，直接照文件的事实排档：开机 HEAVY
        # 6 秒 -> 切到文件头记录的档位 -> 2 秒后开始录，录制中不再切。
        settle = max(settle, 8.0)
        core.fw.g.update(HEAVY.gains())
    out = {k: np.full(n, np.nan) for k in COLS}
    ns = int(settle * FS)
    st = {"c": 0}

    def hook(tw, _t):
        c = st["c"]
        st["c"] += 1
        i = c - 1 - ns                      # 上一拍对应的文件行
        if 0 <= i < n:
            off = tw.fw.st.motors_off
            out["gyro"][i] = tw._gyro_lsb
            out["ang"][i] = tw._angle_filt
            out["ml"][i] = 0 if off else tw.fw.st.ccr_l
            out["mr"][i] = 0 if off else tw.fw.st.ccr_r
            out["el"][i], out["er"][i] = tw._enc_last
            out["inj"][i] = tw.fw.dither
        if load_mode and c == ns - int(2 * FS):
            target = NORMAL if hdr.get("gear_is_normal", 1) else HEAVY
            tw.fw.g.update(target.gains())
        if load_mode and c == ns:
            st["gear"] = "HEAVY" if abs(tw.fw.g["balance_kd"] - HEAVY.balance_kd) < 1e-9 else "NORMAL"
        j = c - ns                           # 本拍对应的文件行
        tw.fw.dither = float(f["inj"][j]) if 0 <= j < n else 0.0
    core.tick_hook = hook
    fell_at = None
    for _ in range(int((settle + n / FS) * 25) + 2):
        _, _, te, tr, info = core.step()
        if te or tr:
            fell_at = st["c"] - 1 - ns
            break
    return dict(twin=out, file=f, fell_at=fell_at, gear_log=st.get("gear"))
