"""减速停车瞬态：用户报告「从行进中停下时振荡剧烈甚至倒下」。
Deceleration-to-stop transient -- the symptom every existing bench misses.

为什么要单独写：现有的三个基准**没有一个**覆盖这个动作。
  - `bench_three_goals.py` 第 3 条在 `n*DT > 8.0` 之后才采样，专门跳过了加减速段
  - 「C 组」测的是停下**之后** 10~30 秒的位置和航向，不是停车过程本身
  - 所有基准都跑 0.35 N 侧风，而用户是在**低扰动**下看到的问题

这和转向振荡是同一类漏测。2026-09-09 用户说 #8 左转右转振荡，而我的
「转向俯仰RMS」显示余量很大——因为测的是恒定指令下的稳态，振荡发生在指令
**跳变**那一下。加了摇杆式阶跃才量出 #8 过冲 21.1%、原厂 13.7%。

这里复现的动作：加速到 v，匀速跑一段，然后**速度指令一步归零**，量刹车过程。
扰动一律用 `DisturbanceConfig()`（无风无噪声），因为用户就是在这个条件下看到的。

    python scripts/bench_stop_transient.py
    python scripts/bench_stop_transient.py --policy policy_stm32_pid_v11.npz
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (STM32Twin,          # noqa: E402
                                                MODE_STM32_PID,
                                                MODE_STM32_LQR)
from balance_bot.firmware.controllers import (PID_GAINS,            # noqa: E402
                                              LQR_GAINS)
from balance_bot.stm32_policy import STM32InferencePolicy           # noqa: E402
from balance_bot.params import DisturbanceConfig                    # noqa: E402
from balance_bot.dynamics import IV, ITH, ITHD                      # noqa: E402

DT = 0.04
CALM = DisturbanceConfig()          # 用户就是在低扰动下看到的问题
CRUISE = 6.0                        # 巡航多久再刹 / seconds before the stop
AFTER = 10.0                        # 刹车后观察多久 / seconds observed after
SETTLE_DEG = 0.5                    # 认为「稳住了」的俯仰阈值


def make(firmware, gains, pol, secs):
    return STM32Twin(firmware=firmware, gains=gains, imu_filter="kalman",
                     randomize=False, sample_difficulty=False,
                     episode_seconds=secs,
                     hold_station=bool(pol and pol.hold_station))


def stop_test(firmware, gains, pol, v_cruise, seeds=range(8)):
    """加速 -> 巡航 -> 速度指令一步归零。量刹车瞬态。

    返回 (俯仰峰值 deg, 俯仰过零次数, 稳定时间 s, 摔倒率 %, 有效局数)。
    过零次数数的是**刹车之后**俯仰角速度的符号翻转——车前后摇一个来回算两次。
    """
    PK, ZC, TS, fell, ok = [], [], [], 0, 0
    secs = CRUISE + AFTER + 2.0
    for sd in seeds:
        c = make(firmware, gains, pol, secs)
        c.set_disturbance(CALM)
        c.reset(seed=sd)
        last = np.zeros(pol.gain_space.dim) if pol else None
        v, n, log, down = 0.0, 0, [], False
        while True:
            t_now = n * DT
            if t_now < CRUISE:
                v = min(v_cruise, v + 0.9 * DT)
            else:
                v = 0.0                      # 一步归零，不是缓降
            if pol:
                last = pol.drive(c, last, float(v), 0.0)
            c.set_command(float(v), 0.0)
            _, _, term, trunc, _ = c.step()
            n += 1
            log.append((t_now, float(c.state[ITH]), float(c.state[ITHD]),
                        float(c.state[IV])))
            if term:
                down = True
                break
            if trunc:
                break
        if down:
            fell += 1
            ok += 1
            continue
        if n * DT < CRUISE + AFTER:
            continue
        ok += 1
        L = np.array(log)
        m = L[:, 0] >= CRUISE
        seg = L[m]
        PK.append(np.degrees(np.abs(seg[:, 1]).max()))
        rate = seg[:, 2]
        ZC.append(int(np.sum(np.diff(np.sign(rate)) != 0)))
        # 稳定时间：最后一次 |pitch| 超过阈值之后多久
        big = np.abs(np.degrees(seg[:, 1])) > SETTLE_DEG
        TS.append(float(np.argmax(big[::-1] == True)) * DT   # noqa: E712
                  if big.any() else 0.0)
        idx = len(big) - 1
        while idx > 0 and not big[idx]:
            idx -= 1
        TS[-1] = (idx + 1) * DT
    if not PK:
        return None
    return (float(np.mean(PK)), float(np.mean(ZC)), float(np.mean(TS)),
            fell / max(ok, 1) * 100.0, ok)


def rock_test(firmware, gains, pol, seeds=range(8), secs=30.0):
    """静止前后摇晃：零指令、无扰动，量俯仰的摆动幅度和频率。"""
    AMP, FRQ = [], []
    for sd in seeds:
        c = make(firmware, gains, pol, secs)
        c.set_disturbance(CALM)
        c.reset(seed=sd)
        last = np.zeros(pol.gain_space.dim) if pol else None
        n, th = 0, []
        while True:
            if pol:
                last = pol.drive(c, last, 0.0, 0.0)
            c.set_command(0.0, 0.0)
            _, _, term, trunc, _ = c.step()
            n += 1
            if n * DT > 8.0:
                th.append(float(c.state[ITH]))
            if term or trunc:
                break
        if n < int(secs / DT) or len(th) < 100:
            continue
        a = np.degrees(np.array(th))
        a = a - a.mean()
        AMP.append(float(np.abs(a).max()))
        # 过零频率 -> 摇晃频率
        FRQ.append(float(np.sum(np.diff(np.sign(a)) != 0)) / (len(a) * DT) / 2)
    if not AMP:
        return None
    return float(np.mean(AMP)), float(np.mean(FRQ)), len(AMP)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", nargs="*", default=[])
    ap.add_argument("--speeds", nargs="*", type=float,
                    default=[0.15, 0.25, 0.35])
    args = ap.parse_args()

    cases = [("原厂 PID", MODE_STM32_PID, dict(PID_GAINS), None),
             ("原厂 LQR", MODE_STM32_LQR, dict(LQR_GAINS), None)]
    for p in args.policy:
        pol = STM32InferencePolicy(p)
        tag = pol.label().split(" · ")[0]
        cases.append((tag, pol.firmware, None, pol))

    print("减速停车瞬态   无扰动（低噪声）   巡航 6 秒后速度指令一步归零   8 种子")
    print("过零次数 = 刹车后俯仰角速度符号翻转次数，前后摇一个来回算两次\n")

    for v in args.speeds:
        print(f"--- 巡航 {v:.2f} m/s ---")
        print(f"  {'':12s}{'俯仰峰值':>10s}{'过零次数':>10s}"
              f"{'稳定时间':>10s}{'摔倒率':>9s}  局")
        for nm, fw, g, pol in cases:
            r = stop_test(fw, g, pol, v)
            if r is None:
                print(f"  {nm:12s}{'---- 无有效局 ----':>42s}")
                continue
            print(f"  {nm:12s}{r[0]:9.2f}°{r[1]:10.1f}{r[2]:9.2f}s"
                  f"{r[3]:8.0f}%  {r[4]}")
        print()

    print("--- 静止前后摇晃（零指令，无扰动，第 8~30 秒）---")
    print(f"  {'':12s}{'俯仰摆幅':>10s}{'摇晃频率':>10s}  局")
    for nm, fw, g, pol in cases:
        r = rock_test(fw, g, pol)
        if r is None:
            print(f"  {nm:12s}{'---- 摔 ----':>26s}")
            continue
        print(f"  {nm:12s}{r[0]:9.3f}°{r[1]:9.2f}Hz  {r[2]}")


if __name__ == "__main__":
    main()
