"""Policies and the stock PID in the simulator with the motor extended by
the reversal test (motor_reversal_fit.py): torque limit and viscous friction
added to fitted model A. Does the simulator now wobble like the car?

    python sim_tools/extended_motor_check.py [--limit 0.75] [--visc 0.00054]

The torque limit is motor_model.TAU_DRIVER_LIMIT and the actuators'
ctrlrange (both 0.4 N m in training); both are raised for this script only.
"""
import argparse

import numpy as np

import motor_fit as F
import motor_model as MM
import pid_sim as P
import simlib as S
from policy_filter_sim import CAR, FITTED_A

T = S.T


def with_limit(limit, fn, *a, **kw):
    """Run fn with the torque limit raised in motor_model and in every new env."""
    old_lim, old_make = MM.TAU_DRIVER_LIMIT, S.make_env

    def make(**k):
        env, obs = old_make(**k)
        env.unwrapped.model.actuator_ctrlrange[:] = [-limit, limit]
        return env, obs

    MM.TAU_DRIVER_LIMIT, S.make_env = limit, make
    try:
        return fn(*a, **kw)
    finally:
        MM.TAU_DRIVER_LIMIT, S.make_env = old_lim, old_make


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--limit", type=float, default=0.75)
    ap.add_argument("--visc", type=float, default=0.00054)
    ap.add_argument("--slack", type=float, nargs="+", default=[2.0, 3.0])
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()
    ext = (FITTED_A[0], FITTED_A[1], np.full(2, args.visc))
    models = (("fitted A (0.4 N m)", FITTED_A, 0.4), (f"extended ({args.limit:.2f} N m + viscous)", ext, args.limit))

    print("stock PID, standing 30 s (car: rate sd 7.6 deg/s, 1461 mean |PWM|)")
    for name, model, lim in models:
        for g in args.slack:
            for lat in args.delay:
                s = with_limit(lim, P.sim_pid, model, g, lat, 120.0, 5.0, 40.0, seconds=30.0)
                print(f"  {name:32s} {g:3.0f} deg {5 * lat:3d} ms: " +
                      (f"FELL at {s['fell']:.1f} s" if s["fell"] is not None else P.fmt(s)), flush=True)

    print("\npolicies, standing 10 s; cells: pitch-rate sd deg/s, Hz, mean |PWM|")
    for name, path in (("run8", "models/best_real_RUN8/best_model.zip"),
                       ("run9", "models/best_real_RUN9/best_model.zip"),
                       ("run11", "models/best_real_RUN11/best_model.zip")):
        pol = S.load(path)
        print(f"{name}  (car {CAR[name]} deg/s)")
        for g in args.slack:
            for lat in args.delay:
                cells = []
                for mname, model, lim in models:
                    ss = [with_limit(lim, F.run_with_motor, pol, model, slack_deg=(g, g), latency_ticks=lat, seed=sd)
                          for sd in range(args.seeds)]
                    ok = [s for s in ss if s["fell"] is None]
                    mn = lambda k: float(np.mean([s[k] for s in ok]))
                    cells.append(f"{mn('rate_sd'):6.1f} {mn('freq'):5.1f} {mn('pwm'):5.0f}" if ok else f"{'FELL':>18s}")
                print(f"  {g:3.0f} deg {5 * lat:3d} ms   A: {cells[0]}   extended: {cells[1]}", flush=True)


if __name__ == "__main__":
    main()
