"""Baseline benchmark for the STM32 digital twin: how long does it stay up?

The metric is **agent steps survived** (25 Hz, so 500 steps = the full 20 s
episode).  Every controller and every parameter set is run on the *same*
seeded episodes -- same robot perturbation, same noise realisation, same
shove at the same instant -- so the numbers are comparable.

    # the two shipped firmwares, as they ship
    python scripts/bench_twin_baseline.py --episodes 40

    # score a parameter set an RL run or an LLM proposed
    python scripts/bench_twin_baseline.py --params my_gains.json --episodes 40

    # what do the two firmware bugs actually cost?
    python scripts/bench_twin_baseline.py --ablate

A parameter file is JSON, one of::

    {"firmware": "stm32_lqr",
     "gains": {"K1": -62.05, "K2": -73.32, "K3": -361.46,
               "K4": -35.90, "K5": 15.81, "K6": 15.81}}

    {"firmware": "stm32_pid",
     "gains": {"balance_kp": 96.0, "balance_kd": 0.48,
               "velocity_kp": 62.0, "velocity_ki": 0.31,
               "turn_kp": 14.0, "turn_kd": 0.20, "mid_angle_deg": 1.0}}

which is exactly the shape an optimiser or a language model can emit.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (STM32Twin, make_mujoco_twin,   # noqa: E402
                                       MODE_STM32_LQR, MODE_STM32_PID)
from balance_bot.firmware.controllers import LQR_GAINS, PID_GAINS     # noqa: E402
from balance_bot.dynamics import ITH, ITHD, IV, IPSID                 # noqa: E402

MAX_STEPS_HINT = 500


def build(backend, firmware, gains, difficulty, episode_seconds,
          imu_filter="kalman", **kw):
    factory = make_mujoco_twin if backend == "mujoco" else STM32Twin
    core = factory(firmware=firmware, gains=gains, imu_filter=imu_filter,
                   randomize=difficulty > 0.0, sample_difficulty=False,
                   episode_seconds=episode_seconds, **kw)
    core.difficulty = difficulty
    return core


def apply_policy(core, policy, last_action, v_ref=0.0, yaw_ref=0.0):
    """一个智能体拍的增益调度。观测和写回都在 STM32InferencePolicy 里，
    UI / bench / 训练环境共用同一份，不会再各飘各的。
    One agent step of gain scheduling -- the observation and the write-back
    live in STM32InferencePolicy so the UI, the bench and the training env
    cannot drift apart again."""
    return policy.drive(core, last_action, v_ref, yaw_ref)


def run(core, episodes, seed0, drive=False, policy=None):
    steps, fell, pitches = [], 0, []
    for i in range(episodes):
        core.reset(seed=seed0 + i)
        if not drive:
            core.set_command(0.0, 0.0)
        last_action = np.zeros(policy.gain_space.dim) if policy else None
        n, ps = 0, 0.0
        while True:
            if policy is not None:
                last_action = apply_policy(core, policy, last_action)
            _, _, term, trunc, info = core.step()
            n += 1
            ps += info["pitch"] ** 2
            if term or trunc:
                fell += int(info["fell"])
                break
        steps.append(n)
        pitches.append(np.sqrt(ps / n))
    return dict(steps=float(np.mean(steps)), steps_sd=float(np.std(steps)),
                worst=int(np.min(steps)), fall=fell / episodes,
                rms_pitch=float(np.mean(pitches)),
                full=float(np.mean([s >= core.sim.max_agent_steps
                                    for s in steps])))


def report(label, r, max_steps):
    print(f"  {label:<34} 步数 {r['steps']:6.1f} ± {r['steps_sd']:5.1f}"
          f"  (满 {max_steps})   最差 {r['worst']:4d}"
          f"   摔倒 {r['fall'] * 100:5.1f}%   跑满 {r['full'] * 100:5.1f}%"
          f"   RMS俯仰 {np.degrees(r['rms_pitch']):5.2f}°")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=("analytic", "mujoco"),
                    default="analytic")
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed0", type=int, default=20_000)
    ap.add_argument("--levels", type=float, nargs="+",
                    default=[0.0, 0.25, 0.5, 0.75, 1.0])
    ap.add_argument("--episode-seconds", type=float, default=20.0)
    ap.add_argument("--params", nargs="*", default=[],
                    help="JSON parameter files to score alongside the defaults")
    ap.add_argument("--drive", action="store_true",
                    help="let the scripted command source drive it around")
    ap.add_argument("--imu", choices=("kalman", "complementary", "dmp"), default="kalman",
                    help="kalman = KF.c as the board runs it (default); "
                         "Get_Angle(way): 2=kalman(出厂) 3=complementary 1=dmp")
    ap.add_argument("--policy", nargs="*", default=[],
                    help="policy_*.npz；每 40 ms 改写一次固件增益，"
                         "和原厂并排打分")
    ap.add_argument("--ablate", action="store_true",
                    help="also measure what the two firmware bugs cost")
    args = ap.parse_args()

    runs = [("固件 LQR (原样)", MODE_STM32_LQR, None, {}),
            ("固件 PID (原样)", MODE_STM32_PID, None, {})]

    if args.ablate:
        runs += [
            ("固件 LQR + 修偏航标度", MODE_STM32_LQR, None,
             dict(fix_yaw_scale=True)),
        ]

    for path in args.params:
        with open(path, encoding="utf-8") as f:
            spec = json.load(f)
        fw = spec.get("firmware", MODE_STM32_LQR)
        runs.append((spec.get("name", os.path.basename(path)), fw,
                     spec.get("gains"), spec.get("options", {})))

    policies = []
    for path in args.policy:
        from balance_bot.stm32_policy import STM32InferencePolicy
        pol = STM32InferencePolicy(path)
        if pol.imu_filter != args.imu:
            print(f"[警告] {os.path.basename(path)} 是在「{pol.imu_filter}」"
                  f"姿态模型上训的，本次评测用的是「{args.imu}」。观测分布不同，"
                  f"这个分数不作数——加 --imu {pol.imu_filter} 或者重训。\n")
        policies.append((f"PPO {os.path.basename(path)}", pol))

    print(f"后端 {args.backend}   每档 {args.episodes} 个相同种子的回合   "
          f"回合长度 {args.episode_seconds:.0f} s\n")

    results = {}
    for d in args.levels:
        print(f"--- 扰动难度 {d:.2f} ---")
        for label, fw, gains, opts in runs:
            core = build(args.backend, fw, gains, d, args.episode_seconds,
                         imu_filter=args.imu, **opts)
            r = run(core, args.episodes, args.seed0, drive=args.drive)
            report(label, r, core.sim.max_agent_steps)
            results.setdefault(label, {})[d] = r
        for label, pol in policies:
            # 站定版策略需要孪生打开出厂固件没有的那一环
            # a station-hold policy needs the outer loop the firmware lacks
            # 策略必须在它训练时的那台车上评测——固件基线仍然是原样，
            # 但策略跟着自己 npz 里记的设置走，和 imu_filter 一个道理。
            # A policy is scored on the plant it was trained on; the firmware
            # baselines stay stock.  Same rule as imu_filter.
            core = build(args.backend, pol.firmware, None, d,
                         args.episode_seconds, imu_filter=args.imu,
                         hold_station=pol.hold_station)
            r = run(core, args.episodes, args.seed0, drive=args.drive,
                    policy=pol)
            report(label, r, core.sim.max_agent_steps)
            results.setdefault(label, {})[d] = r
        print()

    # one-line summary, easiest thing to paste into a comparison
    print("汇总（平均存活步数）")
    header = "  " + " " * 34 + "".join(f"{d:>8.2f}" for d in args.levels)
    print(header)
    for label in results:
        row = "".join(f"{results[label][d]['steps']:>8.1f}" for d in args.levels)
        print(f"  {label:<34}{row}")


if __name__ == "__main__":
    main()
