"""Compare the car's two-level motor steps (motor_reversal_test.py) with the
simulator's motor model, and say what the model gets wrong after the PWM
changes while the wheel is turning.

    python sim_tools/motor_reversal_fit.py [--csv car_logs/<date>/motor_reversal.csv]

Each step: pwm1 for ticks 0-199, pwm2 for 200-399, 0 for 400-499; enc at
tick k counts the 5 ms ending at k, so pwm2 acts from tick 201.
Simulated with motor_fit.sim_rotor()'s rotor (motor_model.pwm_to_torque,
fitted motor A per wheel, unrounded counts) for any PWM sequence.

Per step, after the change to pwm2 (car | model):
  speed at the change     mean counts/tick over ticks 180-200
  time to half way        from the change until the speed is half way to
                          its final value (mean of ticks 350-400)
  time to zero            for reversals: until the wheel stops
  final speed             mean of ticks 350-400
"""
import argparse
import collections
import csv
import math

import numpy as np

import simlib as S
from motor_model import TAU_DRIVER_LIMIT, MotorParams, pwm_to_torque
from policy_filter_sim import FITTED_A

RAD_PER_COUNT = 2 * math.pi / 1320
J_WHEEL = 1.964e-5
CHANGE = 200


def load(path):
    rows = collections.defaultdict(list)
    with open(S.repo_path(path)) as fh:
        for r in csv.DictReader(fh):
            rows[(r["side"], int(r["pwm1"]), int(r["pwm2"]))].append((int(r["k"]), int(r["enc_l"]), int(r["enc_r"])))
    out = {}
    for (side, p1, p2), rr in rows.items():
        a = np.array(sorted(rr), dtype=float)
        out[(side, p1, p2)] = a[:, 1] if side == "L" else a[:, 2]
    return out


def sim(side, p1, p2, model=FITTED_A, scale=None, n=500):
    """Counts per tick (unrounded) of one wheel through the PWM sequence."""
    i = 0 if side == "L" else 1
    mp, arm = model[0], model[1]
    params = MotorParams(dead_zone=mp.dead_zone[i:i + 1], tau_scale=mp.tau_scale[i:i + 1],
                         kv_scale=mp.kv_scale[i:i + 1], coulomb=mp.coulomb[i:i + 1])
    j = arm[i] + J_WHEEL
    th = w = 0.0
    prev = 0.0
    out = np.zeros(n)
    for k in range(n):
        out[k] = th / RAD_PER_COUNT - prev
        prev = th / RAD_PER_COUNT
        p = p1 if k < CHANGE else (p2 if k < 2 * CHANGE else 0)
        tau = float(np.clip(pwm_to_torque(np.array([p]), np.array([w]), params), -TAU_DRIVER_LIMIT, TAU_DRIVER_LIMIT)[0])
        if scale is not None:
            tau = scale(p, w, tau)
        for _ in range(5):
            w += tau / j * 0.001
            th += w * 0.001
    return out


def smooth(e, n=3):
    return np.convolve(e, np.ones(n) / n, mode="same")


def features(e):
    s = smooth(e)
    v0 = float(np.mean(e[180:201]))
    v1 = float(np.mean(e[350:401]))
    after = s[201:400]
    half = 0.5 * (v0 + v1)
    cross = np.nonzero(np.sign(after - half) != np.sign(v0 - half))[0] if v0 != half else []
    zero = np.nonzero(np.sign(after) != np.sign(v0))[0] if v0 and np.sign(v1) != np.sign(v0) else []
    return dict(v0=v0, v1=v1, t_half=5 * (cross[0] + 1) if len(cross) else None,
                t_zero=5 * (zero[0] + 1) if len(zero) else None)


def ms(v):
    return f"{v:4d}" if v is not None else "   -"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=None, help="default car_logs/2026-10-08/motor_reversal.csv "
                    "(motor_lowpwm.csv with --lowpwm)")
    ap.add_argument("--lowpwm", action="store_true", help="the low-PWM set (motor_reversal_test.py --set lowpwm)")
    args = ap.parse_args()
    if args.lowpwm:
        lowpwm_report(load(args.csv or "car_logs/2026-10-08/motor_lowpwm.csv"))
        return
    data = load(args.csv or "car_logs/2026-10-08/motor_reversal.csv")
    print("counts/tick (x 0.952 = rad/s at the wheel); times in ms after the PWM change.  car | fitted model A")
    print(f"{'step':>22s}   {'speed before':>13s}   {'to half way':>11s}   {'to zero':>11s}   {'final speed':>13s}")
    sums = collections.defaultdict(list)
    for key in sorted(data, key=lambda k: (k[0], k[1] < 0, abs(k[1]), k[2] * np.sign(k[1]))):
        side, p1, p2 = key
        if p1 == 0:
            continue
        fc, fm = features(data[key]), features(sim(side, p1, p2))
        kind = "coast" if p2 == 0 else ("reverse" if np.sign(p2) != np.sign(p1) else "slow")
        print(f"{side} {p1:+5d} -> {p2:+5d} {kind:>7s}   {fc['v0']:+6.1f} {fm['v0']:+6.1f}   {ms(fc['t_half'])} {ms(fm['t_half'])}"
              f"     {ms(fc['t_zero'])} {ms(fm['t_zero'])}     {fc['v1']:+6.1f} {fm['v1']:+6.1f}")
        if fc["t_half"] and fm["t_half"]:
            sums[kind].append(fc["t_half"] / fm["t_half"])
    print("\ncar / model time to half way, median by kind: "
          + ", ".join(f"{k} {np.median(v):.2f} (n {len(v)})" for k, v in sums.items()))



# ---------------------------------------------------------------- extended model
def sim_ext(side, p1, p2, limit, visc, model=FITTED_A, n=500, two_level=True):
    """Like sim(), with the torque limit `limit` (motor_model has 0.4 N m) and a
    viscous friction -visc * omega added; counts per tick, unrounded."""
    i = 0 if side == "L" else 1
    mp, arm = model[0], model[1]
    params = MotorParams(dead_zone=mp.dead_zone[i:i + 1], tau_scale=mp.tau_scale[i:i + 1],
                         kv_scale=mp.kv_scale[i:i + 1], coulomb=mp.coulomb[i:i + 1])
    import motor_model as MM
    old = MM.TAU_DRIVER_LIMIT
    MM.TAU_DRIVER_LIMIT = limit
    try:
        j = arm[i] + J_WHEEL
        th = w = prev = 0.0
        out = np.zeros(n)
        for k in range(n):
            out[k] = th / RAD_PER_COUNT - prev
            prev = th / RAD_PER_COUNT
            p = p1 if k < CHANGE else ((p2 if k < 2 * CHANGE else 0) if two_level else 0)
            tau = float(np.clip(MM.pwm_to_torque(np.array([p]), np.array([w]), params), -limit, limit)[0]) - visc * w
            for _ in range(5):
                w += tau / j * 0.001
                th += w * 0.001
    finally:
        MM.TAU_DRIVER_LIMIT = old
    return out


def fit_extension(data):
    """Torque limit and viscous friction (both wheels) to the two-level steps;
    everything else stays fitted model A."""
    from scipy.optimize import least_squares
    keys = [k for k in data if k[1] != 0]

    def res(x):
        r = []
        for side, p1, p2 in keys:
            e = data[(side, p1, p2)]
            s = sim_ext(side, p1, p2, x[0], x[1] / 1000)
            r.append((smooth(e, 4) - smooth(s, 4))[150:500] / (abs(p1) / 1000))
        return np.concatenate(r)

    sol = least_squares(res, [0.4, 0.5], bounds=([0.2, 0.0], [1.5, 10.0]), diff_step=1e-3)
    base = 0.5 * float(np.sum(res([0.4, 0.0]) ** 2))
    return sol.x[0], sol.x[1] / 1000, float(sol.cost), base


# ---------------------------------------------------------------- low-PWM set
def lowpwm_report(data, limit=0.75, visc=0.00054):
    """motor_lowpwm.csv: thresholds from rest and from motion, small steps,
    low-PWM reversals; car against the corrected motor (limit + viscous)."""
    rad = RAD_PER_COUNT / 0.005
    print("speeds in rad/s at the wheel (car | model); times in ms after the change")
    print("\nthreshold from rest (2 s at the PWM): speed in the last 0.5 s, first count, time to 63 % of that speed")
    for side in "LR":
        for sgn in (1, -1):
            cells = []
            for p in (1420, 1440, 1460, 1480, 1500, 1520):
                key = (side, sgn * p, sgn * p)
                if key not in data:
                    continue
                e, m = data[key], sim_ext(*key, limit, visc)
                vc, vm = sgn * np.mean(e[300:400]) * rad, sgn * np.mean(m[300:400]) * rad
                first = np.nonzero(sgn * e[1:400] > 0)[0]
                sm = sgn * smooth(e, 5) * rad
                t63 = np.nonzero(sm[1:400] >= 0.63 * vc)[0] if vc > 0.3 else []
                cells.append(f"{p}: {vc:4.1f}|{vm:4.1f} {5 * (first[0] + 1) if len(first) else '-':>4} "
                             f"{5 * (t63[0] + 1) if len(t63) else '-':>4}")
            print(f"  {side} {'fwd' if sgn > 0 else 'rev'}  " + "   ".join(cells))
    print("\nfrom motion (1 s at 1800, then 1 s at the PWM): speed in the last 0.25 s")
    for side in "LR":
        for sgn in (1, -1):
            cells = []
            for p in (1420, 1440, 1460, 1480, 1500):
                key = (side, sgn * 1800, sgn * p)
                if key in data:
                    e, m = data[key], sim_ext(*key, limit, visc)
                    cells.append(f"{p}: {sgn * np.mean(e[350:400]) * rad:4.1f}|{sgn * np.mean(m[350:400]) * rad:4.1f}")
            print(f"  {side} {'fwd' if sgn > 0 else 'rev'}  " + "   ".join(cells))
    print("\nsmall steps and low-PWM reversals: speed before | after (car / model), time to half way (car / model)")
    for side in "LR":
        for sgn in (1, -1):
            for p1, p2 in ((1500, 1600), (1500, 1700), (1600, 1800), (1700, 1500), (1700, 1600), (2000, 1500),
                           (1600, -1600), (1700, -1500), (1500, -1500)):
                key = (side, sgn * p1, sgn * p2)
                if key not in data:
                    continue
                fc, fm = features(data[key]), features(sim_ext(*key, limit, visc))
                print(f"  {side} {sgn * p1:+5d} -> {sgn * p2:+5d}: {fc['v0'] * rad:+5.1f}/{fm['v0'] * rad:+5.1f} -> "
                      f"{fc['v1'] * rad:+5.1f}/{fm['v1'] * rad:+5.1f}   half way {ms(fc['t_half'])}/{ms(fm['t_half'])}")


if __name__ == "__main__":
    main()
