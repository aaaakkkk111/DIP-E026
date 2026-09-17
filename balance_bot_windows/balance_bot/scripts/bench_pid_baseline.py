"""基准车 + 蓝牙遥控 PID 的完整基线。
The complete baseline for the bare car running the bluetooth-control PID.

「基准车」= 什么外设都不装的那台：`6.LQR/Matlab/parameter_LQR.m` 里的
整车 1.000 kg、轮径 67.2 mm、质心离轴 38.3 mm、轮距 161.2 mm。跑在它上面的
出厂工程有两个，`6.LQR/STM32_code` 和 `4.Balanced_Car_base/04.bluetooth_control`，
APP 目录下都没有额外硬件驱动。这个脚本测后者的 PID。

"The bare car" is the one with nothing bolted on: 1.000 kg, 67.2 mm wheels,
38.3 mm COM height, 161.2 mm track, from 6.LQR/Matlab/parameter_LQR.m.  Two
shipped projects run on it unmodified; this measures the PID one.

为什么单独一个脚本：`bench_twin_baseline.py` 只测站着不动的存活，而
`--drive` 用的是基类那套给 1.7 kg 通用机器人配的指令脚本（0.9 m/s），对这台
车是不可达的。要拿这套 PID 当「训练目标」的对照，四类指标缺一不可：存活、
速度跟踪、静止品质、抗风站定。

`bench_twin_baseline.py` only scores standing still, and its `--drive` uses the
generic robot's 0.9 m/s command script, which this car cannot reach.  Judging a
tuned PID against this one needs all four: survival, tracking, standstill
quality, and station keeping under wind.

    python scripts/bench_pid_baseline.py
    python scripts/bench_pid_baseline.py --episodes 20 --quick
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (STM32Twin,          # noqa: E402
                                                MODE_STM32_PID)
from balance_bot.firmware.controllers import PID_GAINS, pid_gains  # noqa: E402
from balance_bot.params import DisturbanceConfig                    # noqa: E402
from balance_bot.dynamics import IX, IY, IPSI, IV, ITH              # noqa: E402

DT = 0.04
V_ACCEL = 0.9            # 和 sim_gui.py 的斜坡一致 / same ramp as the UI
PWM_FULL = 2880.0


def build(gains, secs, cfg=None, randomize=False, difficulty=0.0):
    c = STM32Twin(firmware=MODE_STM32_PID, gains=gains, imu_filter="kalman",
                  randomize=randomize, sample_difficulty=False,
                  episode_seconds=secs)
    if cfg is not None:
        c.set_disturbance(cfg)
    c.difficulty = difficulty
    return c


def survival(gains, episodes, levels):
    """站着不动的存活步数——和 bench_twin_baseline.py 同一个指标。"""
    out = []
    for d in levels:
        steps = []
        for i in range(episodes):
            c = build(gains, 20.0, randomize=d > 0.0, difficulty=d)
            c.reset(seed=20000 + i)
            c.set_command(0.0, 0.0)
            n = 0
            while True:
                _, _, t, tr, _ = c.step()
                n += 1
                if t or tr:
                    break
            steps.append(n)
        out.append(np.mean(steps))
    return out


def tracking(gains, cmds, seeds=(0, 1, 2), secs=12.0):
    """指令 -> 稳态速度、到 50 % 的时间。斜坡和 UI 一致。"""
    rows = []
    for cmd in cmds:
        T, V, ok = [], [], 0
        for sd in seeds:
            c = build(gains, secs, cfg=DisturbanceConfig())
            c.reset(seed=sd)
            v, n, vs, t50 = 0.0, 0, [], None
            while True:
                v = min(cmd, v + V_ACCEL * DT)
                c.set_command(float(v), 0.0)
                _, _, t, tr, _ = c.step()
                n += 1
                vv = float(c.state[IV])
                if t50 is None and vv >= 0.5 * cmd:
                    t50 = n * DT
                if n * DT > secs * 0.5:
                    vs.append(vv)
                if t or tr:
                    break
            if n < int(secs / DT):
                continue
            ok += 1
            T.append(t50 if t50 else np.nan)
            V.append(np.mean(vs))
        if not ok:
            rows.append((cmd, np.nan, np.nan, 0, len(seeds)))
        else:
            rows.append((cmd, np.nanmean(T), np.mean(V), ok, len(seeds)))
    return rows


def quality(gains, cfg, drive, seeds=range(6), secs=30.0):
    """静止品质：位置跨度、PWM 换向、俯仰 RMS、速度 RMS、航向漂移。"""
    W, Z, T, V, H, ok = [], [], [], [], [], 0
    for sd in seeds:
        c = build(gains, secs, cfg=cfg)
        c.reset(seed=sd)
        n, xy, cc, th, vs, ps = 0, [], [], [], [], []
        while True:
            vr, yr = (0.25, 1.0) if (drive and n * DT < 5.0) else (0.0, 0.0)
            c.set_command(vr, yr)
            _, _, t, tr, info = c.step()
            n += 1
            if n * DT > 10.0:
                xy.append((float(c.state[IX]), float(c.state[IY])))
                cc.append(info["ccr"][0])
                th.append(float(c.state[ITH]))
                vs.append(float(c.state[IV]))
                ps.append(float(c.state[IPSI]))
            if t or tr:
                break
        if n < int(secs / DT) or not xy:
            continue
        ok += 1
        P = np.array(xy)
        W.append(np.linalg.norm(P - P.mean(0), axis=1).max() * 2)
        C = np.array(cc, dtype=float)
        Z.append(np.sum(np.diff(np.sign(C)) != 0) / (len(C) * DT))
        T.append(np.degrees(np.sqrt(np.mean(np.square(th)))))
        V.append(np.sqrt(np.mean(np.square(vs))))
        u = np.unwrap(np.array(ps))
        H.append(np.degrees(abs(u[-1] - u[0])))
    if not ok:
        return None
    return (np.mean(W), np.mean(Z), np.mean(T), np.mean(V), np.mean(H), ok)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--set", default="Normal",
                    help="0.Large program 的模式名（PID_GAIN_SETS）；默认 Normal = 真车 1 档")
    ap.add_argument("--quick", action="store_true", help="少跑几档，快速冒烟")
    args = ap.parse_args()

    gains = pid_gains(args.set)   # 0.Large program 的模式名，见 PID_GAIN_SETS
    levels = (0.0, 0.5, 1.0) if args.quick else (0.0, 0.25, 0.5, 0.75, 1.0)
    seeds = range(3) if args.quick else range(6)

    print(f"基准车 + 原厂 PID [{args.set}]   "
          f"姿态 kalman   {args.episodes} 局/档")
    print(f"增益 " + "  ".join(
        f"{k}={gains[k]:g}" for k in ("balance_kp", "balance_kd",
                                      "velocity_kp", "velocity_ki",
                                      "turn_kp", "turn_kd")))

    print("\n[1] 存活步数（零指令，满 500 = 20 秒）")
    surv = survival(gains, args.episodes, levels)
    print("  " + "".join(f"{d:>10.2f}" for d in levels))
    print("  " + "".join(f"{v:>10.1f}" for v in surv))

    print("\n[2] 速度跟踪（斜坡 0.9 m/s²，和 UI 一致）")
    print(f"  {'指令':>8s}{'到50%':>9s}{'稳态':>9s}{'比值':>8s}   存活")
    for cmd, t50, vf, ok, tot in tracking(
            gains, (0.10, 0.20, 0.25, 0.30, 0.35)):
        if not ok:
            print(f"  {cmd:8.2f}{'—— 摔 ——':>26s}   0/{tot}")
        else:
            print(f"  {cmd:8.2f}{t50:8.2f}s{vf:9.3f}{vf / cmd:8.2f}   {ok}/{tot}")

    print("\n[3] 静止与抗扰品质（第 10~30 秒稳态）")
    print(f"  {'场景':22s}{'位置跨度':>10s}{'PWM换向':>10s}{'俯仰RMS':>9s}"
          f"{'速度RMS':>9s}{'航向漂移':>10s}  存活")
    for tag, cfg, drive in (
            ("无扰动", DisturbanceConfig(), False),
            ("侧风 0.35N@60°", DisturbanceConfig(wind_force=0.35,
                                               wind_dir=1.05), False),
            ("侧风+冲击", DisturbanceConfig(wind_force=0.35, wind_dir=1.05,
                                         random_impulse_hz=0.5,
                                         random_impulse_max=5.0), False),
            ("侧风+先开5秒带转向", DisturbanceConfig(wind_force=0.35,
                                            wind_dir=1.05), True)):
        r = quality(gains, cfg, drive, seeds=seeds)
        if r is None:
            print(f"  {tag:22s}{'—— 全摔 ——':>44s}")
        else:
            w, z, t, v, h, ok = r
            print(f"  {tag:22s}{w:9.4f}m{z:9.1f}/s{t:8.2f}°{v:9.3f}"
                  f"{h:9.1f}°  {ok}/{len(list(seeds))}")


if __name__ == "__main__":
    main()
