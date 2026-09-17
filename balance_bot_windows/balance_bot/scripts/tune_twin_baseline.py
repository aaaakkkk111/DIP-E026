"""Search the firmware's own gain vector against the twin (CEM).

This is the "RL-tuned parameters" side of the comparison.  It optimises
*exactly the numbers that are flashed to the board* -- the six LQR gains or
the six PID gains -- so the result is a drop-in replacement for the constants
in ``app_control.c`` / ``pid_control.c``, not a neural network you cannot put
on an STM32.

    python scripts/tune_twin_baseline.py --firmware stm32_lqr --out tuned_lqr.json
    python scripts/bench_twin_baseline.py --params tuned_lqr.json --episodes 40

Two things make the comparison honest, and both are enforced here:

*   **The search starts at the firmware's own values.**  The parameter vector
    is a log-multiplier on each shipped gain, so the zero vector *is* the
    prototype.  Any improvement is therefore measured against a controller
    the search had to actually beat, not against a bad initialisation.

*   **Tuning seeds and scoring seeds are disjoint.**  ``tune_twin_baseline.py``
    searches on seeds starting at ``--seed0`` (default 70000);
    ``bench_twin_baseline.py`` scores on 20000.  Tuning and reporting on the same
    episodes measures memorisation, not control.

``mid_angle_deg`` is deliberately NOT searched.  It is the chassis's
mechanical zero, a property of how the IMU is bolted on -- letting an
optimiser move it while the twin's mounting error stays fixed would just be
fitting the simulator's own offset.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.controllers import LQR_GAINS, PID_GAINS   # noqa: E402
from balance_bot.firmware.twin_baseline import (STM32Twin, MODE_STM32_LQR,   # noqa: E402
                                       MODE_STM32_PID)

# which keys are searched, and in what order
LQR_KEYS = ("K1", "K2", "K3", "K4", "K5", "K6")
PID_KEYS = ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki",
            "turn_kp", "turn_kd")


def nominal_for(firmware):
    if firmware == MODE_STM32_LQR:
        return LQR_KEYS, np.array([LQR_GAINS[k] for k in LQR_KEYS])
    return PID_KEYS, np.array([PID_GAINS[k] for k in PID_KEYS])


def vector_to_gains(x, keys, nominal, span):
    """Log-multiplier vector -> a gains dict.  x = 0 is the firmware."""
    mult = np.exp(np.clip(x, -1.0, 1.0) * np.log(span))
    return {k: float(n * m) for k, n, m in zip(keys, nominal, mult)}


def score(firmware, gains, seeds, difficulties, episode_seconds,
          imu_filter="kalman"):
    """Mean survival steps, averaged over difficulties.  Higher is better."""
    total, n = 0.0, 0
    pitch_pen = 0.0
    for d in difficulties:
        core = STM32Twin(firmware=firmware, gains=gains, randomize=True,
                         sample_difficulty=False, imu_filter=imu_filter,
                         episode_seconds=episode_seconds)
        core.difficulty = d
        for s in seeds:
            core.reset(seed=s)
            core.set_command(0.0, 0.0)
            k, ps = 0, 0.0
            while True:
                _, _, term, trunc, info = core.step()
                k += 1
                ps += info["pitch"] ** 2
                if term or trunc:
                    break
            total += k
            pitch_pen += np.sqrt(ps / k)
            n += 1
    # survival dominates; RMS pitch only breaks ties between equal survivors
    return total / n - 40.0 * (pitch_pen / n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--firmware", choices=(MODE_STM32_LQR, MODE_STM32_PID),
                    default=MODE_STM32_LQR)
    ap.add_argument("--iters", type=int, default=12)
    ap.add_argument("--pop", type=int, default=14)
    ap.add_argument("--elite", type=float, default=0.25)
    ap.add_argument("--sigma", type=float, default=0.35)
    ap.add_argument("--span", type=float, default=4.0,
                    help="search range as a multiple of each shipped gain")
    ap.add_argument("--episodes", type=int, default=4,
                    help="episodes per candidate per difficulty")
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--difficulties", type=float, nargs="+",
                    default=[0.75, 1.0])
    ap.add_argument("--seed0", type=int, default=70_000,
                    help="TUNING seeds; keep disjoint from the scoring seeds")
    ap.add_argument("--rng", type=int, default=0)
    ap.add_argument("--imu", choices=("kalman", "lag"), default="kalman",
                    help="姿态滤波模型；必须和之后 bench_twin_baseline.py 用的一致")
    ap.add_argument("--out", default="tuned.json")
    args = ap.parse_args()

    keys, nominal = nominal_for(args.firmware)
    seeds = [args.seed0 + i for i in range(args.episodes)]
    rng = np.random.default_rng(args.rng)

    def fit(x):
        return score(args.firmware, vector_to_gains(x, keys, nominal, args.span),
                     seeds, args.difficulties, args.episode_seconds)

    base = fit(np.zeros(len(keys)))
    print(f"固件原型（搜索起点）得分 {base:8.2f}")
    print(f"搜索 {len(keys)} 个增益，范围 x1/{args.span:.0f} ~ x{args.span:.0f}，"
          f"调参种子 {args.seed0}..{args.seed0 + args.episodes - 1}，"
          f"难度 {args.difficulties}\n")

    mu = np.zeros(len(keys))
    sigma = np.full(len(keys), args.sigma)
    best_x, best_f = mu.copy(), base
    n_elite = max(2, int(args.pop * args.elite))
    t0 = time.time()

    for it in range(args.iters):
        pop = np.clip(mu + sigma * rng.normal(size=(args.pop, len(keys))),
                      -1.0, 1.0)
        fits = np.array([fit(x) for x in pop])
        order = np.argsort(-fits)
        elite = pop[order[:n_elite]]
        mu = elite.mean(axis=0)
        sigma = np.maximum(elite.std(axis=0), 0.05) * 0.92

        if fits[order[0]] > best_f:
            best_f, best_x = float(fits[order[0]]), pop[order[0]].copy()
        print(f"  iter {it + 1:2d}/{args.iters}  最好 {fits[order[0]]:8.2f}"
              f"   精英均值 {fits[order[:n_elite]].mean():8.2f}"
              f"   sigma {sigma.mean():.3f}"
              f"   [{time.time() - t0:5.0f}s]", flush=True)

    # the CEM mean is usually more robust than the single best sample
    mu_f = fit(mu)
    if mu_f > best_f:
        best_x, best_f = mu, mu_f

    gains = vector_to_gains(best_x, keys, nominal, args.span)
    print(f"\n调参集上：原型 {base:.2f}  ->  调参后 {best_f:.2f}"
          f"   ({best_f - base:+.2f})")
    print("增益（括号内为相对固件的倍数）:")
    for k, n in zip(keys, nominal):
        print(f"  {k:<14} {gains[k]:10.4f}   ({gains[k] / n:5.2f}x  固件 {n:.4f})")

    spec = {
        "name": f"CEM 调参 ({args.firmware})",
        "firmware": args.firmware,
        "gains": gains,
        "_provenance": {
            "method": "cross-entropy method over log-multipliers",
            "start": "firmware nominal (x = 0)",
            "tuning_seeds": [args.seed0, args.seed0 + args.episodes - 1],
            "tuning_difficulties": args.difficulties,
            "tuning_episode_seconds": args.episode_seconds,
            "imu_model": args.imu,
            "tuning_score_prototype": base,
            "tuning_score_tuned": best_f,
            "note": "调参集得分不能当作结论，必须用 bench_twin_baseline.py 在互不相交的"
                    "种子上复评；复评时 --imu 要和这里的 imu_model 一致，"
                    "否则测的是两个不同的孪生",
        },
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)
    print(f"\n写出 -> {args.out}")
    print(f"现在在**没见过的种子**上复评：\n"
          f"  python scripts/bench_twin_baseline.py --params {args.out} --episodes 40")


if __name__ == "__main__":
    main()
