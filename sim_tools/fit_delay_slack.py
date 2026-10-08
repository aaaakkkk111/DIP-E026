"""Estimate the car's sense-to-act delay and gearbox slack from its logs.

Two different policies ran on the same car (run 8 and run 9, 2026-10-07).
Delay and slack change their behaviour differently, so simulating BOTH over a
delay x slack grid and matching both to the car separates the two. Delay
alone cannot explain run 9: it is calm in simulation at any delay, yet
wobbled on the car.

    python sim_tools/fit_delay_slack.py
    python sim_tools/fit_delay_slack.py --log car_logs/<date>/rl_serial_log.txt \
        --policy run9=models/best_real_RUN9/best_model.zip=20:14:00-20:20:00 ...

Result on 2026-10-07: slack ~4-6 deg at the wheel with 10-20 ms of delay; the
simulation reproduces run 8 well but run 9's wobble at only ~half size.
"""
import argparse
import itertools
import math

import numpy as np

import simlib as S

DEFAULT_LOG = "car_logs/2026-10-07/rl_serial_log.txt"
DEFAULT_POLICIES = ["run8=models/best_real_RUN8/best_model.zip=15:39:00-15:41:00",
                    "run9=models/best_real_RUN9/best_model.zip=20:14:00-20:20:00"]


def err(s, c):
    if s is None:
        return 25.0
    return (math.log(max(s["rate_sd"], 0.5) / c["rate_sd"]) ** 2
            + ((s["freq"] - c["freq"]) / 2.0) ** 2
            + ((s["sat"] - c["sat"]) / 20.0) ** 2
            + ((s["pwm"] - c["pwm"]) / 300.0) ** 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default=DEFAULT_LOG)
    ap.add_argument("--policy", action="append", default=None, metavar="NAME=CHECKPOINT=HH:MM:SS-HH:MM:SS")
    ap.add_argument("--slack", type=float, nargs="+", default=[0, 1, 2, 3, 4, 5, 6, 8])
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()

    pols = []
    for spec in args.policy or DEFAULT_POLICIES:
        name, path, window = spec.split("=")
        car = S.car_stats(args.log, *window.split("-"))
        pols.append((name, S.load(path), car))
        print(f"car, {name}: {car['runs']} runs, {car['ticks']} ticks of balancing: "
              f"rate sd {car['rate_sd']:.1f} deg/s, {car['freq']:.1f} Hz, full PWM {car['sat']:.0f} %, "
              f"mean |PWM| {car['pwm']:.0f}")

    print(f"\n{'delay':>6s} {'slack':>6s} | " + " | ".join(f"{n:>24s}" for n, _, _ in pols) + " |  error")
    results = []
    for lat, g in itertools.product(args.delay, args.slack):
        cells, e = [], 0.0
        for name, pol, car in pols:
            ss = [S.run(pol, slack_deg=(g, g), latency_ticks=lat, seed=sd) for sd in range(args.seeds)]
            ok = [s for s in ss if s["fell"] is None]
            e += 25.0 * (len(ss) - len(ok)) / len(ss)
            if ok:
                mean = {k: float(np.mean([s[k] for s in ok])) for k in ("rate_sd", "freq", "sat", "pwm")}
                e += err(mean, car) * len(ok) / len(ss)
                cells.append(f"{mean['rate_sd']:6.1f} {mean['freq']:5.1f} {mean['sat']:4.0f}% {mean['pwm']:5.0f}"
                             + ("" if len(ok) == len(ss) else "F"))
            else:
                cells.append(f"{'FELL':>24s}")
        results.append((e, lat, g))
        print(f"{lat*5:4d}ms {g:5.1f}° | " + " | ".join(cells) + f" | {e:6.2f}", flush=True)

    print("\nbest fits (columns: rate sd, Hz, % full PWM, mean |PWM|):")
    for e, lat, g in sorted(results)[:6]:
        print(f"  delay {lat*5:2d} ms, slack {g:.1f} deg  -> error {e:.2f}")


if __name__ == "__main__":
    main()
