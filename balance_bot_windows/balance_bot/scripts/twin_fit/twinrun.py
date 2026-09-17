# -*- coding: utf-8 -*-
"""孪生在和真车录数**完全相同**的条件下跑，输出和真车 CSV 一样的列。

条件：模式 1 Normal（9600/48/6200/31/1700/20，中值 0），死区 1500 / 补偿 1500，
无扰动。扫频照 tune_io.c 的 TUNE_Inject() 逐拍生成：
    f = 0.5 + 24.5 * t/4000;  ph += 2*pi*f/200;  out = (int)(amp*sin(ph))
加在 PID 输出、死区补偿之前（app_control.c:149）。
"""
import os, sys
import numpy as np
from balance_bot.firmware.twin_baseline import (make_mujoco_twin, MODE_STM32_PID,
                                                CAR_STOP)
from balance_bot.firmware.controllers import pid_gains
from balance_bot.params import DisturbanceConfig

FS = 200.0


def run(seconds=32.0, chirp_amp=0, settle=3.0, gains=None, motor_cal=None,
        pre=2.0, core_hook=None, dist=None, **kw):
    g = pid_gains("Normal")
    g.update(gains or {})
    core = make_mujoco_twin(firmware=MODE_STM32_PID, gains=g, motor_cal=motor_cal,
                            imu_filter="kalman", randomize=False,
                            disturbance=dist or DisturbanceConfig(),
                            episode_seconds=settle + pre + seconds + 5, **kw)
    if core_hook: core_hook(core)
    core.reset(seed=0)
    core.set_car_state(CAR_STOP)
    rec = {k: [] for k in ("gyro", "ang", "ml", "mr", "el", "er", "inj")}
    st = dict(n=0, t=0, ph=0.0, on=False)
    n_settle = int(settle * FS)
    n_pre = int(pre * FS)

    def hook(tw, _t):
        i = st["n"]; st["n"] += 1
        # 先记上一拍的输出（hook 在本拍传感器/固件之前跑）
        if i > n_settle:
            ccr = tw.fw_ccr if not tw.fw.st.motors_off else (0, 0)
            rec["gyro"].append(float(tw._gyro_lsb))
            rec["ang"].append(float(tw._angle_filt))
            rec["ml"].append(int(ccr[0])); rec["mr"].append(int(ccr[1]))
            rec["el"].append(int(tw._enc_last[0])); rec["er"].append(int(tw._enc_last[1]))
            rec["inj"].append(int(tw.fw.dither))
        # 再算本拍注入
        out = 0
        if chirp_amp and i >= n_settle + n_pre and st["t"] < 4000:
            f = 0.5 + (25.0 - 0.5) * (st["t"] / 4000.0)
            st["ph"] += 6.2831853 * f / FS
            if st["ph"] > 6.2831853: st["ph"] -= 6.2831853
            out = int(chirp_amp * np.sin(st["ph"]))
            st["t"] += 1
        tw.fw.dither = float(out)
    core.tick_hook = hook
    total = int((settle + pre + seconds) * 25) + 1
    fell = False
    for _ in range(total):
        _, _, te, tr, info = core.step()
        if te or tr:
            fell = bool(info.get("fell", te)); break
    d = {k: np.array(v, dtype=float) for k, v in rec.items()}
    d["fell"] = fell
    return d
