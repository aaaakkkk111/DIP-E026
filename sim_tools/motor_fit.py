"""Fit motor_model.py to the car's motor step test, then check whether the
simulator wobbles like the car with the fitted motor.

    python sim_tools/motor_fit.py                       # measure + fit
    python sim_tools/motor_fit.py --wobble              # ... + runs 8/9/11 in simulation
    python sim_tools/motor_fit.py --csv car_logs/<date>/motor_steps.csv

Input: motor_step_test.py's CSV. Each step is raw PWM on one wheel for ticks
0-199 (written at the end of the 5 ms ISR) and 0 for 200-299; enc at tick k
counts the 5 ms that end at tick k, so the PWM acts on ticks 1-200.

1. Measured, per wheel and direction: dead zone (lowest PWM that turns the
   wheel steadily), steady speed against PWM (last 0.4 s of the step), delay
   to the first count, time to 63 % of the steady speed, coast-down (time to
   37 % of the steady speed, and to a stop, after the PWM goes to 0).
2. Fit, per wheel: dead_zone (forward, reverse), tau_scale, kv_scale, coulomb
   and the rotor armature, through motor_model.pwm_to_torque() exactly as
   training uses it. The fit runs on the rotor alone (one inertia, the
   wheel's 1.96e-5 kg m^2 added, torque held over each 5 ms tick from the
   speed at its start, as PWMCommandWrapper does), then the fitted motor is
   re-simulated in real_robot.xml with the chassis held in the air and the
   result reported too.
   Free spin alone cannot tell torque from inertia: scaling tau_scale,
   kv_scale, coulomb and armature together gives the same wheel speeds. Only
   the 0.4 N m driver limit (motor_model.TAU_DRIVER_LIMIT), active in the
   first ticks of the larger steps, ties them to an absolute size. The
   profile along that direction is printed.
3. --wobble: runs 8, 9 and 11 standing still in simulation with the fitted
   motor (simlib.make_env, then env._motor and the armature replaced), slack
   2-4 deg, delay 5-20 ms, against the car's balancing statistics.
"""
import argparse
import collections
import csv
import math
import os
import sys

import mujoco
import numpy as np
from scipy.optimize import least_squares

import simlib as S
from motor_model import (KV, NOMINAL_MOTOR, TAU_DRIVER_LIMIT, MotorParams,
                         pwm_to_torque)

T = S.T
RAD_PER_COUNT = 2 * math.pi / 1320
DT = 0.005
SUB = T.FRAME_SKIP                      # 1 ms physics steps per tick
ON = 200                                # ticks at the step PWM
J_WHEEL = 1.964e-5                      # real_robot.xml wheel inertia about its axle
STEADY = slice(121, 201)                # last 0.4 s of the PWM
W_PER_COUNT = RAD_PER_COUNT / DT        # rad/s per count/tick

TRAINING = dict(dead_zone="1460 (1350-1600)", tau_scale="1 (0.85-1.05)", kv_scale="1 (0.9-1.1)",
                coulomb="0.0025 (0.002-0.02)", armature="2.25e-3 (9e-4 - 4.5e-3)")

# the car's balancing statistics (simlib.car_stats, E lines)
CARS = [("run8", "models/best_real_RUN8/best_model.zip", "car_logs/2026-10-07/rl_serial_log.txt", "15:39:00", "15:41:00"),
        ("run9", "models/best_real_RUN9/best_model.zip", "car_logs/2026-10-07/rl_serial_log.txt", "20:14:00", "20:20:00"),
        ("run11", "models/best_real_RUN11/best_model.zip", "car_logs/2026-10-08/rl_serial_log.txt", "15:55:00", "15:57:00")]


# ---------------------------------------------------------------- measurement
def load_steps(path):
    """{(side, pwm): (enc of the driven wheel, enc of the other wheel)}, 300 ticks each."""
    rows = collections.defaultdict(list)
    with open(S.repo_path(path)) as fh:
        for r in csv.DictReader(fh):
            rows[(r["side"], int(r["pwm"]))].append((int(r["k"]), int(r["enc_l"]), int(r["enc_r"])))
    steps = {}
    for (side, pwm), rr in rows.items():
        rr.sort()
        a = np.array(rr, dtype=np.float64)
        drv, oth = (a[:, 1], a[:, 2]) if side == "L" else (a[:, 2], a[:, 1])
        steps[(side, pwm)] = (drv, oth)
    return steps


def smooth(e, n=4):
    """Counts/tick averaged over the last n ticks, as the firmware's speed."""
    c = np.concatenate([[0.0], np.cumsum(e)])
    out = np.empty_like(e)
    for k in range(len(e)):
        lo = max(0, k + 1 - n)
        out[k] = (c[k + 1] - c[lo]) / (k + 1 - lo)
    return out


def describe(e, pwm):
    """Steady speed (rad/s), first-count delay, t63 and coast-down (ms) of one step."""
    sgn = 1 if pwm > 0 else -1
    steady = float(np.mean(e[STEADY])) * W_PER_COUNT
    d = dict(steady=steady, first=None, t63=None, t37=None, stop=None)
    moved = np.nonzero(sgn * e[1:ON + 1] > 0)[0]
    if len(moved):
        d["first"] = 5 * (moved[0] + 1)
    if abs(steady) < 0.5:
        return d
    sm = sgn * smooth(e, 2) * W_PER_COUNT
    up = np.nonzero(sm[1:ON + 1] >= 0.63 * abs(steady))[0]
    if len(up):
        d["t63"] = 5 * (up[0] + 1)
    down = np.nonzero(sm[ON + 1:] <= 0.37 * abs(steady))[0]
    if len(down):
        d["t37"] = 5 * (down[0] + 1)
    still = [k for k in range(ON + 1, len(e) - 2) if not e[k] and not e[k + 1] and not e[k + 2]]
    if still:
        d["stop"] = 5 * (still[0] - ON)
    return d


def report_measured(steps):
    pwms = sorted({abs(p) for _, p in steps if p})
    print(f"{len(steps)} steps. Speeds in rad/s at the wheel (1320 counts/rev); times in ms after the PWM change.\n")
    print("steady speed       " + "".join(f"{p:>6d}" for p in pwms))
    rows = {}
    for side in "LR":
        for sgn, nm in ((1, "fwd"), (-1, "rev")):
            ds = {p: describe(steps[(side, sgn * p)][0], sgn * p) for p in pwms if (side, sgn * p) in steps}
            rows[(side, nm)] = ds
            print(f"  {side} {nm}           " + "".join(f"{abs(ds[p]['steady']):6.1f}" if p in ds else "     -" for p in pwms))
    print()
    for key, label in (("first", "first count"), ("t63", "time to 63 %"), ("t37", "coast to 37 %"), ("stop", "coast to stop")):
        print(f"{label:<19s}" + "".join(f"{p:>6d}" for p in pwms))
        for (side, nm), ds in rows.items():
            print(f"  {side} {nm}           " + "".join(
                f"{ds[p][key]:6d}" if p in ds and ds[p][key] is not None else "     -" for p in pwms))
        print()
    print("dead zone (lowest PWM with a steady speed above 0.5 rad/s; highest without):")
    for (side, nm), ds in rows.items():
        on = [p for p in pwms if p in ds and abs(ds[p]["steady"]) > 0.5]
        off = [p for p in pwms if p in ds and abs(ds[p]["steady"]) <= 0.5 and (not on or p < on[0])]
        print(f"  {side} {nm}: turns at {on[0] if on else '-'}, still at {off[-1] if off else '-'}")
    others = max(float(np.max(np.abs(o))) for _, o in steps.values())
    print(f"\nlargest count on the undriven wheel in any step: {others:.0f} per tick")
    return rows


# ---------------------------------------------------------------- simulation
def sim_rotor(pwms, dz_f, dz_r, ts, kv, c, armature, visc=0.0, n=300):
    """Counts per tick of one wheel for each PWM step (vectorised over steps).
    Not rounded to whole counts: the fit needs a smooth function of the
    parameters. visc (N m per rad/s) is NOT in motor_model.py; it is only
    for the extended fit that asks whether the model is missing it."""
    pwms = np.asarray(pwms, dtype=np.float64)
    m = len(pwms)
    params = MotorParams(dead_zone=np.tile([dz_f, dz_r], (m, 1)), tau_scale=np.full(m, ts),
                         kv_scale=np.full(m, kv), coulomb=np.full(m, c))
    j = armature + J_WHEEL
    th, w = np.zeros(m), np.zeros(m)
    prev = np.zeros(m)
    out = np.zeros((m, n))
    for k in range(n):
        cnt = th / RAD_PER_COUNT
        out[:, k] = cnt - prev
        prev = cnt
        p = pwms if k < ON else np.zeros(m)
        tau = np.clip(pwm_to_torque(p, w, params), -TAU_DRIVER_LIMIT, TAU_DRIVER_LIMIT) - visc * w
        for _ in range(SUB):
            w = w + tau / j * 0.001
            th = th + w * 0.001
    return out


class AirSim:
    """real_robot.xml with the chassis held 0.3 m up: both wheels free."""

    def __init__(self, slack_deg=3.0):
        self.m = mujoco.MjModel.from_xml_path(T.XML_FILE_PATH)
        self.d = mujoco.MjData(self.m)
        T.set_gear_slack(self.m, slack_deg, slack_deg)

    def step_test(self, side, pwm, motor, armature, visc=0.0, n=300):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        m.dof_armature[T.ROTOR_QVEL] = armature
        d.qpos[2] = 0.3
        root_q = d.qpos[:7].copy()
        i = 0 if side == "L" else 1
        prev = np.floor(d.qpos[T.ROTOR_QPOS] / RAD_PER_COUNT)
        out = np.zeros((n, 2))
        for k in range(n):
            cnt = np.floor(d.qpos[T.ROTOR_QPOS] / RAD_PER_COUNT)
            out[k] = cnt - prev
            prev = cnt
            p = np.zeros(2)
            if k < ON:
                p[i] = pwm
            w = d.qvel[T.ROTOR_QVEL].copy()
            d.ctrl[:] = pwm_to_torque(p, w, motor) - visc * w
            for _ in range(SUB):
                mujoco.mj_step(m, d)
                d.qpos[:7] = root_q
                d.qvel[:6] = 0.0
        return out[:, i]


def residuals(meas, pwms, x):
    sim = sim_rotor(pwms, *x)
    r = []
    for e, s in zip(meas, sim):
        w = 1.0 / (abs(np.mean(e[STEADY])) + 3.0)     # low-PWM steps count as much as fast ones
        r.append(w * (smooth(e) - smooth(s)))
    return np.concatenate(r)


# x = dead zone fwd, rev, tau_scale, kv_scale, coulomb, armature, viscous
X0 = np.array([1460, 1460, 1.0, 1.0, 0.0025, 2.25e-3, 0.0])
LO = np.array([1200, 1200, 0.3, 0.3, 0.0, 2e-4, 0.0])
HI = np.array([1800, 1800, 3.0, 3.0, 0.1, 2e-2, 0.02])
SCALE = np.array([1000, 1000, 1, 1, 0.01, 0.001, 0.001])     # x / SCALE is O(1)


def fit_wheel(steps, side, kv_fixed=None, visc=False, starts=((1420, 1420), (1480, 1480)), x_start=None):
    keys = sorted(k for k in steps if k[0] == side and k[1])
    pwms = [p for _, p in keys]
    meas = [steps[k][0] for k in keys]
    x0 = (X0 if x_start is None else x_start).copy()
    free = np.ones(7, bool)
    free[6] = visc
    if not visc:
        x0[6] = 0.0
    if kv_fixed is not None:
        x0[3] = kv_fixed
        free[3] = False

    def f(z):
        x = x0.copy()
        x[free] = z * SCALE[free]
        return residuals(meas, pwms, x)

    best = None
    for dz in (starts if x_start is None else [tuple(x0[:2])]):   # the dead zone is not smooth
        z0 = x0.copy()
        z0[:2] = dz
        z0 = np.clip(z0, LO, HI)
        sol = least_squares(f, z0[free] / SCALE[free], bounds=(LO[free] / SCALE[free], HI[free] / SCALE[free]),
                            diff_step=1e-3, x_scale="jac")
        if best is None or sol.cost < best.cost:
            best = sol
    x = x0.copy()
    x[free] = best.x * SCALE[free]
    return x, float(best.cost), keys


def motor_from(xl, xr):
    """MotorParams, armature and viscous coefficient (per wheel) from two fits."""
    return (MotorParams(dead_zone=np.array([xl[:2], xr[:2]]), tau_scale=np.array([xl[2], xr[2]]),
                        kv_scale=np.array([xl[3], xr[3]]), coulomb=np.array([xl[4], xr[4]])),
            np.array([xl[5], xr[5]]), np.array([xl[6], xr[6]]))


# ---------------------------------------------------------------- wobble
def run_with_motor(policy, model, **kw):
    """simlib.run() with env._motor and the rotor armature replaced after the
    reset; model = None (nominal) or (MotorParams, armature, viscous)."""
    orig_make, orig_tq = S.make_env, T.action_to_torque

    def make(**a):
        env, obs = orig_make(**a)
        env._motor = model[0]
        env.unwrapped.model.dof_armature[T.ROTOR_QVEL] = model[1]
        return env, obs

    def torque(action, omega, params):            # PWMCommandWrapper.step's motor call
        return orig_tq(action, omega, params) - model[2] * omega

    if model is not None:
        S.make_env = make
        if np.any(model[2]):
            T.action_to_torque = torque
    try:
        return S.run(policy, **kw)
    finally:
        S.make_env, T.action_to_torque = orig_make, orig_tq


def wobble(models, slacks, delays, seeds):
    print("\nStanding still, t > 2 s of 10 s; columns: pitch-rate sd deg/s, Hz, % full PWM, mean |PWM|")
    for name, path, log, t0, t1 in CARS:
        car = S.car_stats(log, t0, t1)
        pol = S.load(path)
        print(f"\n{name}  car: {car['rate_sd']:6.1f} {car['freq']:5.1f} {car['sat']:4.0f}% {car['pwm']:5.0f}")
        print(f"  {'':10s} " + " ".join(f"{n:>26s}   " for n in models))
        for g in slacks:
            for lat in delays:
                cells = []
                for mot in models.values():
                    ss = [run_with_motor(pol, mot, slack_deg=(g, g), latency_ticks=lat, seed=sd)
                          for sd in range(seeds)]
                    ok = [s for s in ss if s["fell"] is None]
                    if not ok:
                        cells.append(f"{'FELL':>26s}")
                        continue
                    mn = {k: float(np.mean([s[k] for s in ok])) for k in ("rate_sd", "freq", "sat", "pwm")}
                    cells.append(f"{mn['rate_sd']:6.1f} {mn['freq']:5.1f} {mn['sat']:4.0f}% {mn['pwm']:5.0f}"
                                 + ("F" if len(ok) < len(ss) else " ") + "   ")
                print(f"  {g:3.0f}° {5 * lat:3d}ms " + " ".join(cells), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="car_logs/2026-10-08/motor_steps.csv")
    ap.add_argument("--slack", type=float, default=3.0, help="gear slack for the in-air check, deg")
    ap.add_argument("--wobble", action="store_true")
    ap.add_argument("--skip-check", action="store_true", help="skip the real_robot.xml in-air check")
    ap.add_argument("--wobble-slack", type=float, nargs="+", default=[2, 3, 4])
    ap.add_argument("--wobble-delay", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()

    steps = load_steps(args.csv)
    print(f"== measured: {args.csv}")
    report_measured(steps)

    print("\n== fit (rotor model; residual = 4-tick speed, weighted 1 / (steady counts + 3))")
    fits, ext = {}, {}
    nominal_cost = {}
    for side in "LR":
        keys = sorted(k for k in steps if k[0] == side and k[1])
        nominal_cost[side] = 0.5 * float(np.sum(residuals([steps[k][0] for k in keys], [p for _, p in keys], X0) ** 2))
        x, cost, _ = fit_wheel(steps, side)
        fits[side] = x
        print(f"  {side}: dead zone fwd {x[0]:.0f} rev {x[1]:.0f}, tau_scale {x[2]:.3f}, kv_scale {x[3]:.3f}, "
              f"coulomb {x[4]:.4f} N m, armature {x[5]:.3e} kg m^2   (cost {cost:.2f}; nominal {nominal_cost[side]:.2f})")
        print(f"     ratios: tau_scale/kv_scale {x[2] / x[3]:.3f}, armature/kv_scale {x[5] / x[3]:.3e}, "
              f"coulomb/kv_scale {x[4] / x[3]:.4f}, time constant J/(KV kv) {1000 * (x[5] + J_WHEEL) / (KV * x[3]):.0f} ms")
        prof = []
        for kvf in (0.7, 0.85, 1.15, 1.3):
            xs = x.copy()
            xs[2], xs[4], xs[5] = x[2] * kvf / x[3], x[4] * kvf / x[3], x[5] * kvf / x[3]
            xp, cp, _ = fit_wheel(steps, side, kv_fixed=kvf, x_start=xs)
            prof.append(f"kv {kvf}: {cp:.2f} (tau {xp[2]:.2f}, J {xp[5]:.2e})")
        print("     scale profile (cost with kv_scale held): " + "; ".join(prof))
        xe, ce, _ = fit_wheel(steps, side, visc=True, x_start=np.append(x[:6], 0.002))
        ext[side] = xe
        print(f"     + viscous friction (not in motor_model.py): dead zone {xe[0]:.0f}/{xe[1]:.0f}, tau_scale {xe[2]:.3f}, "
              f"kv_scale {xe[3]:.3f}, coulomb {xe[4]:.4f}, armature {xe[5]:.3e}, viscous {xe[6]:.4f} N m s/rad "
              f"(cost {ce:.2f})")

    # The best fit sits somewhere along a nearly flat valley (scale profile
    # above); report it also with kv_scale held at 1, i.e. KV as motor_model.py
    # has it (stall torque / no-load speed), which fixes the absolute size.
    kv1 = {}
    for side in "LR":
        x = fits[side]
        xs = x.copy()
        xs[2], xs[4], xs[5] = x[2] / x[3], x[4] / x[3], x[5] / x[3]
        kv1[side], c1, _ = fit_wheel(steps, side, kv_fixed=1.0, x_start=xs)
        print(f"  {side} with kv_scale held at 1: dead zone {kv1[side][0]:.0f}/{kv1[side][1]:.0f}, "
              f"tau_scale {kv1[side][2]:.3f}, coulomb {kv1[side][4]:.4f}, armature {kv1[side][5]:.3e} (cost {c1:.2f})")

    motor = motor_from(kv1["L"], kv1["R"])
    motor_opt = motor_from(fits["L"], fits["R"])
    motor_ext = motor_from(ext["L"], ext["R"])
    print("\nfitted vs training (training value, range); A = kv_scale held at 1, B = best fit (weaker, lighter):")
    for name, ff in (("A", kv1), ("B", fits)):
        print(f"  {name} dead_zone  L fwd/rev {ff['L'][0]:.0f}/{ff['L'][1]:.0f}, R fwd/rev {ff['R'][0]:.0f}/{ff['R'][1]:.0f}"
              f"   training {TRAINING['dead_zone']}")
        for i, k in ((2, "tau_scale"), (3, "kv_scale"), (4, "coulomb"), (5, "armature")):
            print(f"  {name} {k:<10s} L {ff['L'][i]:.4g}, R {ff['R'][i]:.4g}   training {TRAINING[k]}")

    if not args.skip_check:
        print(f"\n== check in real_robot.xml (chassis held in the air, slack {args.slack:.0f} deg, encoder counts "
              "rounded as on the car)\n   steady rad/s, t63 ms, coast to 37 % ms: car | fitted A | fitted + viscous | nominal")
        air = AirSim(args.slack)
        f = lambda d: (f"{abs(d['steady']):5.1f} {d['t63'] if d['t63'] is not None else '-':>4} "
                       f"{d['t37'] if d['t37'] is not None else '-':>4}")
        for side in "LR":
            for (s, p) in sorted((k for k in steps if k[0] == side and abs(k[1]) >= 1450),
                                 key=lambda k: (k[1] < 0, abs(k[1]))):
                i = 0 if s == "L" else 1
                cells = [describe(steps[(s, p)][0], p)]
                for mot, arm, visc in (motor, motor_ext, (NOMINAL_MOTOR, T.ARMATURE_NOMINAL, np.zeros(2))):
                    cells.append(describe(air.step_test(s, p, mot, arm, visc[i]), p))
                print(f"  {s} {p:+5d}: " + " | ".join(f(d) for d in cells))

    if args.wobble:
        wobble({"nominal motor": None, "fitted A (kv 1)": motor, "fitted B (best fit)": motor_opt,
                "fitted + viscous": motor_ext}, args.wobble_slack, args.wobble_delay, args.seeds)


if __name__ == "__main__":
    sys.exit(main())
