"""Stiction near the dead zone: fit a speed-dependent dead zone to the
low-PWM steps (motor_lowpwm.csv), then rerun the stock PID and the policies.

    python sim_tools/stiction_check.py [--fit-only]

The car (motor_reversal_test.py --set lowpwm): from rest the wheel does not
turn below ~1500, but a turning wheel keeps turning at 1420; small PWM
changes near the dead zone take 1.3-2x longer than the model. Modelled as

    dead_zone(omega) = dz_kin + (dz_stat - dz_kin) * max(0, 1 - |omega| / w0)

(per wheel and direction, the same shape), on top of the corrected motor
(fitted model A with torque limit 0.75 N m and viscous friction), with the
Coulomb friction refitted. Nothing in motor_model.py changes; the env's
torque call is replaced for this script only.
"""
import argparse
import math

import numpy as np
from scipy.optimize import least_squares

import motor_fit as F
import motor_model as MM
import motor_reversal_fit as R
import pid_sim as P
import simlib as S
from extended_motor_check import with_limit
from policy_filter_sim import CAR, FITTED_A

T = S.T
LIMIT, VISC = 0.75, 0.00054


def torque(pwm, omega, mp, dz_stat, dz_kin, w0, coulomb, i=None):
    """pwm_to_torque with the speed-dependent dead zone, limit LIMIT, viscous VISC.
    pwm, omega: arrays over wheels (i selects one wheel's parameters)."""
    pwm = np.asarray(pwm, dtype=np.float64)
    omega = np.asarray(omega, dtype=np.float64)
    sl = slice(None) if i is None else slice(i, i + 1)
    s = np.sign(pwm)
    dz = dz_kin + (dz_stat - dz_kin) * np.clip(1.0 - np.abs(omega) / w0, 0.0, 1.0)
    eff = np.clip((np.abs(pwm) - dz) / (MM.PWM_PERIOD - dz), 0.0, 1.0)
    drive = MM.TAU_STALL * mp.tau_scale[sl] * eff - MM.KV * mp.kv_scale[sl] * s * omega
    tau = np.clip(s * np.maximum(drive, 0.0), -LIMIT, LIMIT)
    return tau - coulomb * np.tanh(omega / MM.COULOMB_EPS) - VISC * omega


def sim_steps(side, p1, p2, x, n=500):
    """Rotor alone through a two-level step (as motor_reversal_fit.sim_ext)."""
    dz_stat, dz_kin, w0, c = x
    i = 0 if side == "L" else 1
    j = FITTED_A[1][i] + R.J_WHEEL
    th = w = prev = 0.0
    out = np.zeros(n)
    for k in range(n):
        out[k] = th / R.RAD_PER_COUNT - prev
        prev = th / R.RAD_PER_COUNT
        p = p1 if k < 200 else (p2 if k < 400 else 0)
        tau = float(torque([p], [w], FITTED_A[0], dz_stat, dz_kin, w0, c, i)[0])
        for _ in range(5):
            w += tau / j * 0.001
            th += w * 0.001
    return out


def fit(data):
    keys = [k for k in data if k[1] != 0]

    def res(x):
        return np.concatenate([(R.smooth(data[k], 4) - R.smooth(sim_steps(*k, x), 4))[5:500] for k in keys])

    x0 = [1500.0, 1350.0, 1.0, 0.01]
    sol = least_squares(res, x0, bounds=([1380, 1200, 0.1, 0.0], [1600, 1500, 10.0, 0.05]),
                        x_scale=[100, 100, 1, 0.01], diff_step=1e-3)
    base = 0.5 * float(np.sum(res([1395.0, 1395.0, 1.0, 0.0186]) ** 2))
    return sol.x, float(sol.cost), base


def install(x):
    """Replace the env's motor call (PWMCommandWrapper.step uses T.action_to_torque)."""
    dz_stat, dz_kin, w0, c = x
    orig = T.action_to_torque

    def act(action, omega, params):
        return torque(MM.action_to_pwm(action), omega, params, dz_stat, dz_kin, w0, c)

    T.action_to_torque = act
    return orig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="car_logs/2026-10-08/motor_lowpwm.csv")
    ap.add_argument("--fit-only", action="store_true")
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()

    data = R.load(args.csv)
    x, cost, base = fit(data)
    print(f"fit to {len(data)} low-PWM steps: dead zone {x[0]:.0f} at rest -> {x[1]:.0f} above {x[2]:.2f} rad/s, "
          f"Coulomb {x[3]:.4f} N m (cost {cost:.1f}; single dead zone 1395 with Coulomb 0.0186: {base:.1f})")
    rad = R.RAD_PER_COUNT / 0.005
    for key in (("L", 1480, 1480), ("L", 1520, 1520), ("L", 1800, 1420), ("L", 1700, 1600), ("L", 1600, -1600)):
        e, m = data[key], sim_steps(*key, x)
        print(f"  {key[0]} {key[1]:+5d} -> {key[2]:+5d}: final speed car {np.mean(e[350:400]) * rad:+.1f} / model "
              f"{np.mean(m[350:400]) * rad:+.1f} rad/s; half way {R.ms(R.features(e)['t_half'])} / "
              f"{R.ms(R.features(m)['t_half'])} ms")
    if args.fit_only:
        return

    ext = (FITTED_A[0], FITTED_A[1], np.zeros(2))            # viscous is inside torque()
    orig = install(x)
    try:
        print("\nstock PID, standing 30 s (car 7.6 deg/s, 1461 mean |PWM|):")
        for g, lat in ((2, 1), (2, 2), (2, 3), (3, 1)):
            s = with_limit(LIMIT, P.sim_pid, ext, g, lat, 120.0, 5.0, 40.0, seconds=30.0)
            print(f"  {g} deg {5 * lat:2d} ms: " + (f"FELL at {s['fell']:.1f} s" if s["fell"] is not None else P.fmt(s)),
                  flush=True)
        print("\npolicies, standing 10 s (pitch-rate sd deg/s, Hz, mean |PWM|):")
        for name, path in (("run8", "models/best_real_RUN8/best_model.zip"),
                           ("run9", "models/best_real_RUN9/best_model.zip"),
                           ("run11", "models/best_real_RUN11/best_model.zip")):
            pol = S.load(path)
            cells = []
            for g, lat in ((2, 1), (2, 2), (2, 3), (3, 2)):
                ss = [with_limit(LIMIT, F.run_with_motor, pol, ext, slack_deg=(g, g), latency_ticks=lat, seed=sd)
                      for sd in range(args.seeds)]
                ok = [s for s in ss if s["fell"] is None]
                mn = lambda k: float(np.mean([s[k] for s in ok]))
                cells.append(f"{g}°/{5 * lat}ms: " + (f"{mn('rate_sd'):5.1f} {mn('freq'):4.1f} {mn('pwm'):4.0f}" if ok else "FELL"))
            print(f"  {name:5s} (car {CAR[name]:4.1f}): " + "   ".join(cells), flush=True)
    finally:
        T.action_to_torque = orig


if __name__ == "__main__":
    main()
