"""给一个训练好的策略做体检，输出**下一版该改什么**。
Diagnose a trained policy and print what to change in the next training run.

为什么要有这个脚本：#3 到 #10 每一版的诊断都是临时想出来的，同一类错误犯了
不止一次——贴边发现过两次、奖励和验收不一致发现过两次、标定失败两次。下面
五项检查覆盖了到目前为止**所有**真正定过方向的判断，每一项都直接给出一条可
执行的训练改动，而不是一个还需要再解读的数字。

关键在于这五项能互相排除。同一个「位置守不住」的现象，可能是：
  奖励权重错（账本会说：策略赢了奖励却输了验收）
  搜索空间截断（贴边会说：某个增益一半时间贴在边界上）
  训练分布不覆盖（时间尺度会说：失效发生在训练回合之外）
这三者要改的东西完全不同，猜错一次就是 55 分钟。

Every direction change from #3 to #10 was diagnosed ad hoc and the same class
of bug was found more than once.  These five checks cover all of them, and each
emits a concrete training change rather than a number needing interpretation.
Crucially they discriminate: one symptom, three possible causes, three
different fixes, and guessing wrong costs a 55-minute training run.

    python scripts/diagnose_policy.py policy_stm32_pid_v9.npz
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import balance_bot.stm32_env as E                                  # noqa: E402
from balance_bot.stm32_env import STM32Env                          # noqa: E402
from balance_bot.stm32_policy import STM32InferencePolicy           # noqa: E402
from balance_bot.firmware.twin_baseline import (STM32Twin,          # noqa: E402
                                                MODE_STM32_PID)
from balance_bot.firmware.controllers import PID_GAINS              # noqa: E402
from balance_bot.params import DisturbanceConfig                    # noqa: E402
from balance_bot.dynamics import IX, IY, IPSI, IV                   # noqa: E402

DT = 0.04
WIND = DisturbanceConfig(wind_force=0.35, wind_dir=1.05)

# 验收线。前两条是追平原厂（0.0076 m / 1.00），后两条是保住 PPO 已经赢下的
# 优势（原厂 279 步 / 33.3 度）。改这里等于改「什么叫成功」，要慎重。
# The first two match stock; the last two hold the ground PPO already won.
TARGET = dict(pos_wind=(0.010, "<="), track=(0.95, ">="),
              surv=(400.0, ">="), heading=(10.0, "<="))
STOCK = dict(pos_wind=0.0076, track=1.00, surv=279.0, heading=33.3)


def _env(pol, cfg, secs=30.0, seed=0, difficulty=0.0):
    per_gain = bool(pol.gain_space.spans.max() != pol.gain_space.spans.min())
    e = STM32Env(firmware=MODE_STM32_PID, imu_filter="kalman", randomize=False,
                 episode_seconds=secs, command_prob=0.0,
                 hold_station=pol.hold_station,
                 per_gain_span=per_gain, seed=seed)
    e.set_difficulty(difficulty)
    e.reset(seed=seed)
    if cfg is not None:
        e.core.set_disturbance(cfg)
    return e


def rollout_reward(pol, cfg, use_policy, seeds=range(4), secs=30.0,
                   difficulty=0.0):
    """平均每步奖励；use_policy=False 就用 action=0，也就是原厂增益。"""
    tot, n = 0.0, 0
    for sd in seeds:
        e = _env(pol, cfg, secs, seed=sd, difficulty=difficulty)
        last = np.zeros(e.gain_space.dim)
        while True:
            if use_policy:
                a = pol.predict(pol.observe(e.core, last, 0.0, 0.0))
                last = 0.7 * last + 0.3 * a
            else:
                a = np.zeros(e.gain_space.dim, dtype=np.float32)
            _, r, t, tr, _ = e.step(a)
            tot += r
            n += 1
            if t or tr:
                break
    return tot / max(n, 1)


# ---------------------------------------------------------------------------
# 1. 标定：活着必须比摔倒划算
# ---------------------------------------------------------------------------
def check_calibration(pol, out):
    m = rollout_reward(pol, None, use_policy=False, difficulty=0.6)
    steps = 750
    ok = m * steps > -50.0
    out.append(("标定", ok,
                f"原厂增益下每步 {m:+.3f}，活满 {steps} 步 = {m * steps:+.0f}，"
                f"摔倒 = -50",
                None if ok else
                "**奖励权重整体偏负，策略会正确地算出主动摔倒更划算。** "
                "抬高 R_ALIVE，或按同一比例缩小所有惩罚权重，直到这一项转正。"
                "任何改权重的动作之后都必须重跑这一项。"))


# ---------------------------------------------------------------------------
# 2. 贴边：动作是不是被搜索空间截断了，而不是收敛了
# ---------------------------------------------------------------------------
def check_saturation(pol, out, seeds=range(4)):
    A = []
    for sd in seeds:
        e = _env(pol, WIND, seed=sd)
        last = np.zeros(e.gain_space.dim)
        while True:
            a = pol.predict(pol.observe(e.core, last, 0.0, 0.0))
            last = 0.7 * last + 0.3 * a
            _, _, t, tr, _ = e.step(a)
            A.append(last.copy())
            if t or tr:
                break
    A = np.array(A)
    frac = np.mean(np.abs(A) > 0.9, axis=0)
    pinned = [(nm, float(f), float(np.mean(A[:, i])))
              for i, (nm, f) in enumerate(zip(pol.gain_space.names, frac))
              if f > 0.5]
    detail = "  ".join(f"{nm}({f * 100:.0f}% 贴{'上' if m > 0 else '下'}限)"
                       for nm, f, m in pinned) or "无维度贴边"
    fix = None
    if pinned:
        fix = ("**这些增益超过一半时间贴在搜索空间边界上，说明是空间截断了"
               "策略，不是它收敛了——它还想往外走。** 在 stm32_env.py 的 "
               "贴上限就同时调大 span 和 high，贴下限就调大 span。\n     "
               "涉及: " + ", ".join(nm for nm, _, _ in pinned))
    out.append(("贴边", not pinned, detail, fix))


# ---------------------------------------------------------------------------
# 3. 验收：四条线，和原厂对比
# ---------------------------------------------------------------------------
def measure(pol, gains, seeds=range(4)):
    """(顶风位置, 跟踪@0.30, 存活@0.75, 航向)。pol=None 就跑固定增益。"""
    def core(cfg, secs, randomize=False, difficulty=0.0):
        c = STM32Twin(firmware=MODE_STM32_PID, gains=gains,
                      imu_filter="kalman", randomize=randomize,
                      sample_difficulty=False, episode_seconds=secs,
                      hold_station=bool(pol and pol.hold_station))
        c.difficulty = difficulty
        if cfg is not None:
            c.set_disturbance(cfg)
        return c

    def act(c, last, vr, yr):
        return pol.drive(c, last, vr, yr) if pol else last

    def blank():
        return np.zeros(pol.gain_space.dim) if pol else None

    W, H, V, S = [], [], [], []
    for sd in seeds:
        c = core(WIND, 30.0)
        c.reset(seed=sd)
        last, n, xy = blank(), 0, []
        while True:
            last = act(c, last, 0.0, 0.0)
            c.set_command(0.0, 0.0)
            _, _, t, tr, _ = c.step()
            n += 1
            if n * DT > 10.0:
                xy.append((float(c.state[IX]), float(c.state[IY])))
            if t or tr:
                break
        if n >= 750 and xy:
            P = np.array(xy)
            W.append(np.linalg.norm(P - P.mean(0), axis=1).max() * 2)

        c = core(WIND, 30.0)
        c.reset(seed=sd)
        last, n, ps = blank(), 0, []
        while True:
            vr, yr = (0.25, 1.0) if n * DT < 5.0 else (0.0, 0.0)
            last = act(c, last, vr, yr)
            c.set_command(vr, yr)
            _, _, t, tr, _ = c.step()
            n += 1
            if n * DT > 10.0:
                ps.append(float(c.state[IPSI]))
            if t or tr:
                break
        if n >= 750 and ps:
            u = np.unwrap(np.array(ps))
            H.append(np.degrees(abs(u[-1] - u[0])))

    for sd in (0, 1, 2):
        c = core(DisturbanceConfig(), 12.0)
        c.reset(seed=sd)
        last, v, n, vs = blank(), 0.0, 0, []
        while True:
            v = min(0.30, v + 0.9 * DT)
            last = act(c, last, float(v), 0.0)
            c.set_command(float(v), 0.0)
            _, _, t, tr, _ = c.step()
            n += 1
            if n * DT > 6.0:
                vs.append(float(c.state[IV]))
            if t or tr:
                break
        if n >= 300:
            V.append(np.mean(vs))

    for i in range(10):
        c = core(None, 20.0, randomize=True, difficulty=0.75)
        c.reset(seed=20000 + i)
        c.set_command(0.0, 0.0)
        last, n = blank(), 0
        while True:
            last = act(c, last, 0.0, 0.0)
            _, _, t, tr, _ = c.step()
            n += 1
            if t or tr:
                break
        S.append(n)

    return dict(pos_wind=float(np.mean(W)) if W else np.nan,
                track=float(np.mean(V)) / 0.30 if V else np.nan,
                surv=float(np.mean(S)),
                heading=float(np.mean(H)) if H else np.nan)


def check_acceptance(pol, out):
    got = measure(pol, None)
    fails, lines = [], []
    for k, (thr, op) in TARGET.items():
        v = got[k]
        ok = (v <= thr) if op == "<=" else (v >= thr)
        if not ok:
            fails.append(k)
        lines.append(f"{k}={v:.4g}({op}{thr}, 原厂{STOCK[k]:g}) "
                     f"{'OK' if ok else '未达标'}")
    out.append(("验收", not fails, "; ".join(lines),
                None if not fails else "未达标: " + ", ".join(fails)))
    return got, fails


# ---------------------------------------------------------------------------
# 4. 账本：奖励和验收是不是在打架
# ---------------------------------------------------------------------------
def check_ledger(pol, out, got, fails):
    if not fails:
        out.append(("账本", True, "验收全过，无需追查", None))
        return
    r_pol = rollout_reward(pol, WIND, use_policy=True)
    r_stock = rollout_reward(pol, WIND, use_policy=False)
    stock_m = measure(None, PID_GAINS)
    better = sum(1 for k, (_, op) in TARGET.items()
                 if ((stock_m[k] < got[k]) if op == "<="
                     else (stock_m[k] > got[k])))
    if r_pol >= r_stock and better >= 2:
        msg = (f"策略 {r_pol:+.3f}/步 >= 原厂 {r_stock:+.3f}/步，"
               f"但原厂在 {better}/4 项验收上更好")
        fix = ("**奖励函数和验收标准不一致：策略优化的是奖励，而奖励在奖励"
               "错误的行为。这是权重问题，不是训练不够。** 给未达标的那几项"
               "提价（W_POS / W_TRACK / W_YAW），或降低与之打架的项"
               "（W_CHATTER / W_JERK）。加训练步数只会让它更坚定地做错事。")
    else:
        msg = (f"策略 {r_pol:+.3f}/步 < 原厂 {r_stock:+.3f}/步——"
               f"策略连奖励本身都没优化赢原厂")
        fix = ("**奖励定义没问题，是优化没到位。** 加训练步数、放宽贴边"
               "维度、或检查观测里是否缺少做出判别所需的信息。这种情况下"
               "改权重会掩盖真正的问题。")
    out.append(("账本", False, msg, fix))


# ---------------------------------------------------------------------------
# 5. 时间尺度：失效比训练回合还慢，奖励根本看不见
# ---------------------------------------------------------------------------
def check_timescale(pol, out, seeds=range(4)):
    early, late = [], []
    for sd in seeds:
        c = STM32Twin(firmware=MODE_STM32_PID, gains=None, imu_filter="kalman",
                      randomize=False, sample_difficulty=False,
                      episode_seconds=35.0,
                      hold_station=pol.hold_station)
        c.set_disturbance(WIND)
        c.reset(seed=sd)
        last, n, stop, d = np.zeros(pol.gain_space.dim), 0, None, []
        while True:
            vr, yr = (0.25, 1.0) if n * DT < 5.0 else (0.0, 0.0)
            last = pol.drive(c, last, vr, yr)
            c.set_command(vr, yr)
            _, _, t, tr, _ = c.step()
            n += 1
            if abs(n * DT - 5.0) < DT / 2:
                stop = (float(c.state[IX]), float(c.state[IY]))
            if stop is not None:
                d.append(np.hypot(c.state[IX] - stop[0],
                                  c.state[IY] - stop[1]))
            if t or tr:
                break
        if len(d) < 700:
            continue
        early.append(max(d[100:200]))     # 停止后 4~8 秒
        late.append(max(d[300:700]))      # 停止后 12~28 秒
    if not early:
        out.append(("时间尺度", True, "样本不足，跳过", None))
        return
    e_, l_ = float(np.mean(early)), float(np.mean(late))
    ok = l_ <= e_ * 1.2
    out.append(("时间尺度", ok,
                f"停止后 4~8 秒最大偏移 {e_:.3f} m，12~28 秒 {l_:.3f} m",
                None if ok else
                "**失效发生在停止 12 秒之后，比训练里见过的最长站定段还长，"
                "奖励看不见它。** 拉长 stm32_env.py 里 STM32CommandScript 的 "
                "zero_hold_max，以及 --episode-seconds，让训练回合覆盖这个"
                "时间尺度。改权重无效，因为那段时间从没进过训练分布。"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("policy")
    args = ap.parse_args()

    pol = STM32InferencePolicy(args.policy)
    print(f"体检对象: {pol.label()}")
    print(f"  动作 {pol.gain_space.dim} 维   跨度 {pol.gain_space.spans}")
    print(f"  当前奖励权重: R_ALIVE={E.R_ALIVE} W_PITCH={E.W_PITCH} "
          f"W_PRATE={E.W_PRATE} W_TRACK={E.W_TRACK} W_POS={E.W_POS} "
          f"W_YAW={E.W_YAW} W_JERK={E.W_JERK} W_CHATTER={E.W_CHATTER}")
    print("  （权重是当前源码里的值，不是训练该模型时的值——改过就对不上）")

    out = []
    check_calibration(pol, out)
    check_saturation(pol, out)
    got, fails = check_acceptance(pol, out)
    check_ledger(pol, out, got, fails)
    check_timescale(pol, out)

    print("\n" + "=" * 72)
    for name, ok, detail, _ in out:
        print(f"  [{'通过' if ok else '问题'}] {name:6s} {detail}")

    fixes = [(n, f) for n, ok, _, f in out if not ok and f]
    print("=" * 72)
    if not fixes:
        print("  没有发现需要改训练的地方。")
        return
    print("  下一版建议（从上往下，通常一次只改一条，否则无法归因）:\n")
    for i, (n, f) in enumerate(fixes, 1):
        print(f"  {i}. [{n}] {f}\n")


if __name__ == "__main__":
    main()
