"""Compare checkpoints on the simulated car: wobble against delay and gear
slack while standing still, training-like cars, and command tracking.

    python sim_tools/compare_models.py
    python sim_tools/compare_models.py --model run9=models/best_real_RUN9/best_model.zip \
                                       --model run10=models/best_real/best_model.zip

The car itself (2026-10-07): run 8 wobbled with a pitch-rate sd of 95 deg/s,
run 9 with 68 deg/s, both at ~7 Hz. A policy that is calm here at 5 deg of
slack (sd of a few deg/s) is the one to flash next.
"""
import argparse
import os

import simlib as S

DEFAULT_MODELS = [("run9", "models/best_real_RUN9/best_model.zip"),
                  ("run10", "models/best_real_RUN10/best_model.zip"),
                  ("run11", "models/best_real/best_model.zip")]


def fmt(s):
    if s.get("fell") is not None:
        return f"{'FELL at ' + format(s['fell'], '.1f') + ' s':>22s}"
    return f"{s['rate_sd']:6.1f} {s['freq']:5.1f} {s['sat']:4.0f}%"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", action="append", default=None, metavar="NAME=PATH")
    ap.add_argument("--slack", type=float, nargs="+", default=[0.0, 5.0], help="gear slack, deg")
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3, 4, 6], help="delay, 5 ms ticks")
    args = ap.parse_args()
    if args.model:
        models = [tuple(m.split("=", 1)) for m in args.model]
    else:
        models = [(n, p) for n, p in DEFAULT_MODELS if os.path.exists(S.repo_path(p))]
    pols = {n: S.load(p) for n, p in models}
    names = list(pols)

    print("STANDING STILL: pitch-rate sd (deg/s), wobble Hz, % ticks at full PWM")
    print(f"{'slack':>6s} {'delay':>6s} | " + " | ".join(f"{n:>17s}" for n in names))
    for slack in args.slack:
        for lat in args.delay:
            cells = [fmt(S.run(pols[n], slack_deg=(slack, slack), latency_ticks=lat)) for n in names]
            print(f"{slack:5.1f}° {lat*5:4d}ms | " + " | ".join(cells), flush=True)

    last = names[-1]
    print(f"\nTRAINING-LIKE CARS ({last}): random motor, delay, slack, IMU errors; standing still")
    for seed in range(8):
        s = S.run(pols[last], training_like=True, seed=seed)
        tag = f"FELL at {s['fell']:.1f} s" if s["fell"] is not None else \
              f"held 10 s, rate sd {s['rate_sd']:5.1f}, drift {s['v']:+.3f} m/s"
        print(f"  seed {seed}: slack {s['slack'][0]:.1f}/{s['slack'][1]:.1f} deg, delay {s['latency_ms']:2d} ms: {tag}")

    print(f"\nCOMMAND TRACKING at slack 5 deg, delay 15 ms (steady state)")
    for n in names:
        for v, w in ((0.25, 0.0), (-0.25, 0.0), (0.0, 0.5)):
            s = S.run(pols[n], v_cmd=v, w_cmd=w, slack_deg=(5.0, 5.0), latency_ticks=3)
            if s["fell"] is not None:
                print(f"  {n:6s} v {v:+.2f} w {w:+.2f}: FELL at {s['fell']:.1f} s")
                continue
            got = f"v {s['v']:+.3f} ({100*s['v']/v:3.0f} %)" if v else f"w {s['w']:+.3f} ({100*s['w']/w:3.0f} %)"
            print(f"  {n:6s} v {v:+.2f} w {w:+.2f}: {got}, rate sd {s['rate_sd']:5.1f}")


if __name__ == "__main__":
    main()
