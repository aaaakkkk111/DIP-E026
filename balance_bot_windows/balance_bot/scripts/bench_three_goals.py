"""用户定的三条目标，逐条量化。其他指标一律不作数。
The user's three goals, quantified.  Nothing else counts.

    1. 没有输入指令时，小车能长期在原地静止不动
    2. 小车在静止不动时，也不会自己旋转
    3. 小车在行进或者转向时，没有或者几乎没有振荡

前两条以前测过但口径太短：站定只测 20 秒，而「长期」意味着要看漂移**会不会
随时间增长**——#8 就是在停止 12 秒之后才开始游荡的，20 秒的窗口看不见。这里
拉到 60 秒，并且把前后半段分开比。

第三条**从来没测过**。之前的 C 组测的是「开完停下之后」的位置和航向，而用户
问的是「行进**中**」抖不抖。这需要新指标：匀速直行和定速转向过程中的俯仰
RMS、速度波动、偏航率波动——都要去掉均值，因为要的是波动不是数值本身。

Goals 1 and 2 were measured before but over too short a window: a 20 s hold
cannot see drift that only starts after 12 s (which is exactly how #8 failed).
This uses 60 s and compares the two halves.  Goal 3 was never measured at all:
the old "C group" scored position and heading *after* stopping, whereas the
question is whether the car shakes *while moving*.

    python scripts/bench_three_goals.py                     # 原厂基准
    python scripts/bench_three_goals.py --policy p_v10.npz  # 加上模型
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (STM32Twin,          # noqa: E402
                                                MODE_STM32_PID)
from balance_bot.firmware.controllers import PID_GAINS              # noqa: E402
from balance_bot.stm32_policy import STM32InferencePolicy           # noqa: E402
from balance_bot.params import DisturbanceConfig                    # noqa: E402
from balance_bot.dynamics import (IX, IY, IPSI, IPSID, IV,          # noqa: E402
                                  ITH, ITHD)

DT = 0.04
CALM = DisturbanceConfig()
WIND = DisturbanceConfig(wind_force=0.35, wind_dir=1.05)

# 目标线。前两条按原厂的表现定，因为原厂在静止上本来就很好，要求是「不退步」；
# 第三条原厂很差（开完再停航向漂 33 度），所以定成明确的绝对值。
# 「长期」= 60 秒，并且后 30 秒不能比前 30 秒差 —— 这一条卡的是**漂移**，
# 一个只在 20 秒窗口里好看的模型过不了。
GOAL = {
    "1 静止位置(60s)":      (0.010, "<=", "m"),
    "1 后半段/前半段":      (1.20,  "<=", "x"),
    "2 静止自转(60s)":      (2.0,   "<=", "deg"),
    "3 直行俯仰RMS":        (1.00,  "<=", "deg"),
    "3 直行速度波动":       (0.030, "<=", "m/s"),
    "3 转向俯仰RMS":        (1.50,  "<=", "deg"),
    "3 转向偏航率波动":     (0.30,  "<=", "rad/s"),
}


def make(gains, pol, cfg, secs):
    c = STM32Twin(firmware=MODE_STM32_PID, gains=gains, imu_filter="kalman",
                  randomize=False, sample_difficulty=False,
                  episode_seconds=secs,
                  hold_station=bool(pol and pol.hold_station))
    if cfg is not None:
        c.set_disturbance(cfg)
    return c


def blank(pol):
    return np.zeros(pol.gain_space.dim) if pol else None


def act(pol, c, last, vr, yr):
    return pol.drive(c, last, vr, yr) if pol else last


# ---------------------------------------------------------------------------
# 目标 1 + 2：零指令站 60 秒
# ---------------------------------------------------------------------------
def still(gains, pol, cfg, seeds=range(6), secs=60.0):
    span, ratio, yaw = [], [], []
    for sd in seeds:
        c = make(gains, pol, cfg, secs)
        c.reset(seed=sd)
        last, n, xy, ps = blank(pol), 0, [], []
        while True:
            last = act(pol, c, last, 0.0, 0.0)
            c.set_command(0.0, 0.0)
            _, _, t, tr, _ = c.step()
            n += 1
            if n * DT > 5.0:            # 前 5 秒给它稳下来
                xy.append((float(c.state[IX]), float(c.state[IY])))
                ps.append(float(c.state[IPSI]))
            if t or tr:
                break
        if n < int(secs / DT) or len(xy) < 100:
            continue
        P = np.array(xy)
        span.append(np.linalg.norm(P - P.mean(0), axis=1).max() * 2)
        h = len(P) // 2
        a = np.linalg.norm(P[:h] - P[:h].mean(0), axis=1).max() * 2
        b = np.linalg.norm(P[h:] - P[h:].mean(0), axis=1).max() * 2
        ratio.append(b / max(a, 1e-6))
        u = np.unwrap(np.array(ps))
        yaw.append(np.degrees(abs(u[-1] - u[0])))
    if not span:
        return None
    return (float(np.mean(span)), float(np.mean(ratio)),
            float(np.mean(yaw)), len(span))


# ---------------------------------------------------------------------------
# 目标 3：行进中 / 转向中的振荡
# ---------------------------------------------------------------------------
def moving(gains, pol, v_ref, yaw_ref, cfg, seeds=range(6), secs=25.0):
    """匀速段的波动量。全部去掉均值——要的是抖动，不是数值本身。"""
    TH, VV, YY = [], [], []
    for sd in seeds:
        c = make(gains, pol, cfg, secs)
        c.reset(seed=sd)
        last, n, th, vs, yr_ = blank(pol), 0, [], [], []
        v = 0.0
        while True:
            v = min(v_ref, v + 0.9 * DT)
            last = act(pol, c, last, float(v), yaw_ref)
            c.set_command(float(v), yaw_ref)
            _, _, t, tr, _ = c.step()
            n += 1
            if n * DT > 8.0:            # 等斜坡结束、进入匀速
                th.append(float(c.state[ITH]))
                vs.append(float(c.state[IV]))
                yr_.append(float(c.state[IPSID]))
            if t or tr:
                break
        if n < int(secs / DT) or len(th) < 100:
            continue
        TH.append(np.degrees(np.sqrt(np.mean(np.square(
            np.array(th) - np.mean(th))))))
        VV.append(float(np.std(vs)))
        YY.append(float(np.std(yr_)))
    if not TH:
        return None
    return (float(np.mean(TH)), float(np.mean(VV)),
            float(np.mean(YY)), len(TH))


def evaluate(name, gains, pol):
    """跑完三条目标，返回 {指标: 值}。"""
    out = {}
    s = still(gains, pol, WIND)
    if s is None:
        return None
    out["1 静止位置(60s)"] = s[0]
    out["1 后半段/前半段"] = s[1]
    out["2 静止自转(60s)"] = s[2]
    m = moving(gains, pol, 0.20, 0.0, WIND)
    if m is None:
        return None
    out["3 直行俯仰RMS"] = m[0]
    out["3 直行速度波动"] = m[1]
    t = moving(gains, pol, 0.20, 1.0, WIND)
    if t is None:
        return None
    out["3 转向俯仰RMS"] = t[0]
    out["3 转向偏航率波动"] = t[2]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--policy", nargs="*", default=[],
                    help="要一起评的 npz，可给多个")
    args = ap.parse_args()

    cases = [("原厂 PID", dict(PID_GAINS), None)]
    for p in args.policy:
        pol = STM32InferencePolicy(p)
        cases.append((pol.label().split(" · ")[0] + " " +
                      os.path.basename(p), None, pol))

    print("三条目标验收   侧风 0.35N@60°   静止 60 秒 / 行进 25 秒   6 种子")
    print("（全部去均值，测的是波动不是数值本身）\n")
    keys = list(GOAL)
    w = max(len(k) for k in keys) + 2
    print(f"  {'指标':{w}s}{'目标':>10s}" +
          "".join(f"{n[:16]:>18s}" for n, _, _ in cases))
    results = {}
    for name, g, pol in cases:
        results[name] = evaluate(name, g, pol)
    for k in keys:
        thr, op, unit = GOAL[k]
        line = f"  {k:{w}s}{op + str(thr):>10s}"
        for name, _, _ in cases:
            r = results[name]
            if r is None:
                line += f"{'摔':>18s}"
                continue
            v = r[k]
            ok = (v <= thr) if op == "<=" else (v >= thr)
            line += f"{v:14.4g} {'OK' if ok else '✗ '}"
        print(line)

    print("\n通过情况:")
    for name, _, _ in cases:
        r = results[name]
        if r is None:
            print(f"  {name:24s} 摔了")
            continue
        n_ok = sum(1 for k, (thr, op, _) in GOAL.items()
                   if ((r[k] <= thr) if op == "<=" else (r[k] >= thr)))
        bad = [k for k, (thr, op, _) in GOAL.items()
               if not ((r[k] <= thr) if op == "<=" else (r[k] >= thr))]
        print(f"  {name:24s} {n_ok}/{len(GOAL)} 达标"
              + ("" if not bad else "   未过: " + ", ".join(bad)))


if __name__ == "__main__":
    main()
