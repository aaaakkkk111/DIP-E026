"""The stock Yahboom PID (mode 1) on the car and on the simulated car.

    python sim_tools/pid_sim.py --log car_logs/2026-10-08/pid_mode1.txt [--sim]

The car log is car_firmware_pidlog's 'P' stream recorded by stream_log.py
(see there for the fields). In mode 1 the stock code leaves the main loop
too little time to print every tick, so about every second tick arrives:
statistics use the ticks that did, with their real tick numbers.

1. Car: when it balanced, the standing part (before the first push) and
   the pushes, with pitch-rate sd, wobble frequency (spectral peak), mean
   |PWM| and the share of ticks inside the motor's dead zone.
2. Port check: the logged angle, gyro and encoders through this file's copy
   of Balance_PD + Velocity_PI + PWM_Ignore, against the logged PWM.
3. --sim: the same controller on real_robot.xml. The stock sensing is
   emulated: accelerometer (MuJoCo's body acceleration at the IMU, which
   includes gravity) and gyro in raw MPU6050 counts, with noise matched to
   the car, through a copy of the stock KF_X() Kalman filter. Nominal and
   fitted motor (motor_fit.py), slack 2-4 deg, delay 5-20 ms.

Stock code copied (car_firmware_* APP/PID/pid_control.c, APP/KF/KF.c,
APP/app_motor.c, Get_Angle in APP/app_control.c), mode 1 standing still:
  angle   = KF_X(accY, accZ, gyro) in deg; y = atan2(-accY, accZ)
  balance = int(96 * (angle - Mid_Angle) + 0.75 * Gyro_Balance), Mid_Angle 0
  bias    = 0.84 bias - 0.16 (encL + encR); integral += bias, +/-8000
  velocity= int(-60 bias - 0.3 integral)            (integral zeroed while off)
  PWM     = balance + velocity, +/-1300 (PWM_Ignore), clamp +/-2800
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np

import simlib as S

T = S.T
GYRO_LSB = 16.4 * 180 / math.pi        # raw counts per rad/s (939.8)
ACC_LSB_G = 16384.0
DT = 0.005


# ---------------------------------------------------------------- stock code
class StockKF:
    """KF_X() from APP/KF/KF.c: state (angle, gyro bias), Q 1e-10, R 1e-4."""

    def __init__(self):
        self.x = np.zeros(2)
        self.P = np.eye(2)
        self.A = np.array([[1.0, -DT], [0.0, 1.0]])

    def update(self, acc_y, acc_z, gyro_rads):
        y = math.atan2(-acc_y, acc_z)
        xm = self.A @ self.x + np.array([DT * gyro_rads, 0.0])
        Pm = self.A @ self.P @ self.A.T + 1e-10 * np.eye(2)
        k = Pm[:, 0] / (Pm[0, 0] + 1e-4)
        self.x = xm + k * (y - xm[0])
        self.P = (np.eye(2) - np.outer(k, [1.0, 0.0])) @ Pm
        return self.x[0]


def c_int(v):
    """C float -> int conversion: truncation toward zero."""
    return int(v)


class StockPID:
    def __init__(self):
        self.bias = 0.0
        self.integral = 0.0

    def balance(self, angle_deg, gyro_raw):
        return c_int(96.0 * angle_deg + 0.75 * gyro_raw)

    def velocity(self, el, er, motors_off=False):
        self.bias = self.bias * 0.84 + (-(el + er)) * 0.16
        self.integral = min(max(self.integral + self.bias, -8000.0), 8000.0)
        v = c_int(-self.bias * 60.0 - self.integral * 0.3)
        if motors_off:
            self.integral = 0.0
        return v

    def step(self, angle_deg, gyro_raw, el, er):
        off = abs(angle_deg) > 40.0
        m = self.balance(angle_deg, gyro_raw) + self.velocity(el, er, off)
        if m > 0:
            m += 1300
        elif m < 0:
            m -= 1300
        return max(-2800, min(2800, m))


# ---------------------------------------------------------------- statistics
def spectrum_peak(x, dt, fmin=1.0, fmax=20.0):
    x = np.asarray(x, dtype=np.float64) - np.mean(x)
    n = 1 << int(math.ceil(math.log2(max(len(x), 64))))
    f = np.fft.rfftfreq(n, dt)
    p = np.abs(np.fft.rfft(x * np.hanning(len(x)), n)) ** 2
    band = (f >= fmin) & (f <= fmax)
    return float(f[band][np.argmax(p[band])])


def seg_stats(rate_dps, pwm_l, pwm_r, dt):
    rate = np.asarray(rate_dps, dtype=np.float64)
    a = (np.abs(pwm_l) + np.abs(pwm_r)) / 2
    return dict(rate_sd=float(rate.std()), freq=spectrum_peak(rate, dt),
                pwm=float(np.mean(a)), dead=float(100 * np.mean(a < 1395)),
                sat=float(100 * np.mean(a >= 2700)), flips=float(100 * np.mean(np.sign(pwm_l[1:]) != np.sign(pwm_l[:-1]))))


def fmt(s):
    return (f"rate sd {s['rate_sd']:5.1f} deg/s, peak {s['freq']:4.1f} Hz, mean |PWM| {s['pwm']:5.0f}, "
            f"in dead zone {s['dead']:3.0f} %, full {s['sat']:2.0f} %, sign flips {s['flips']:3.0f} %")


# ---------------------------------------------------------------- the car
def load_car(path):
    rows = []
    for line in open(S.repo_path(path), encoding="utf-8", errors="replace"):
        f = line.split()
        if len(f) == 11 and f[1] == "P":
            try:
                rows.append([int(v) for v in f[2:]])
            except ValueError:
                pass
    a = np.array(rows, dtype=np.float64)
    k = np.concatenate([[0], np.cumsum((np.diff(a[:, 0]) % 65536))]) + a[0, 0]
    return dict(k=k, angle=a[:, 1] / 100, gyro=a[:, 2], L=a[:, 3], R=a[:, 4], el=a[:, 5], er=a[:, 6],
                accy=a[:, 7], accz=a[:, 8])


def car_segments(c, settle_s=3.0, push_dps=80.0):
    """Index ranges: standing (motors on, before the first push) and each push."""
    on = (c["L"] != 0) | (c["R"] != 0)
    i0 = int(np.argmax(on))
    t = (c["k"] - c["k"][i0]) * DT
    rate = c["gyro"] / 16.4
    big = np.nonzero((np.abs(rate) > push_dps) & (t > settle_s))[0]
    first_push = big[0] if len(big) else len(t)
    stand = np.nonzero((t > settle_s) & (np.arange(len(t)) < first_push - 100) & on)[0]
    pushes, last = [], -10 ** 9
    for i in big:
        if c["k"][i] - last > 400:            # a new push at least 2 s after the last
            pushes.append(i)
        last = c["k"][i]
    return stand, pushes


def report_car(c):
    stand, pushes = car_segments(c)
    span = (c["k"][-1] - c["k"][0]) * DT
    steps = np.diff(c["k"])
    print(f"{len(c['k'])} samples over {span:.0f} s of ticks; tick steps: "
          + ", ".join(f"{int(s)}: {np.mean(steps == s) * 100:.0f} %" for s in (1, 2, 3)))
    if len(stand):
        sub = {key: c[key][stand] for key in ("gyro", "L", "R", "angle")}
        dt = float(np.median(np.diff(c["k"][stand]))) * DT
        s = seg_stats(sub["gyro"] / 16.4, sub["L"], sub["R"], dt)
        print(f"standing, {len(stand)} samples ({(c['k'][stand[-1]] - c['k'][stand[0]]) * DT:.0f} s): {fmt(s)}")
        print(f"   angle mean {sub['angle'].mean():+.2f} sd {sub['angle'].std():.2f} deg; "
              f"L == R in {np.mean(sub['L'] == sub['R']) * 100:.0f} % of ticks")
    else:
        s = None
        print("no standing part found")
    print(f"{len(pushes)} pushes:")
    for i in pushes:
        w = slice(i, min(i + 400, len(c["k"])))
        a, g, pl = c["angle"][w], c["gyro"][w] / 16.4, c["L"][w]
        back = np.nonzero(np.abs(a[20:]) < 2.0)[0]
        tb = (c["k"][w][20 + back[0]] - c["k"][i]) * DT if len(back) else float("nan")
        print(f"   t {(c['k'][i] - c['k'][0]) * DT:6.1f} s: peak rate {g[np.argmax(np.abs(g))]:+6.0f} deg/s, "
              f"peak angle {a[np.argmax(np.abs(a))]:+5.1f} deg, max |PWM| {np.max(np.abs(pl)):4.0f}, "
              f"back within 2 deg after {tb:.2f} s, fell: {bool(np.any(np.abs(a) > 40))}")
    return s, stand


def port_check(c, stand):
    """Logged PWM minus the copied Balance_PD must be the velocity term,
    which only moves slowly (a low-passed encoder sum and its integral). If
    the copied gains were wrong, the angle and gyro would leak into it."""
    pid = StockPID()
    core = np.where(c["L"] > 0, c["L"] - 1300, np.where(c["L"] < 0, c["L"] + 1300, 0)).astype(float)
    bal = np.array([pid.balance(a, g) for a, g in zip(c["angle"], c["gyro"])], dtype=float)
    i = stand[1:][np.diff(c["k"][stand]) == 1]                     # consecutive ticks
    d_core, d_rest = core[i] - core[i - 1], (core - bal)[i] - (core - bal)[i - 1]
    print(f"port check ({len(i)} consecutive standing ticks): logged PWM changes by {np.std(d_core):.1f} per tick (sd); "
          f"minus the copied balance term, by {np.std(d_rest):.1f}")
    print(f"   implied velocity term while standing: mean {np.mean((core - bal)[stand]):+.0f}, "
          f"sd {np.std((core - bal)[stand]):.0f} (it also cancels the gyro offset: 0.75 x 40 = 30)")


# ---------------------------------------------------------------- simulation
def sim_pid(model, slack, lat, acc_sd, gyro_sd, gyro_bias, seconds=30.0, settle=3.0, seed=0, imu_h=0.04):
    env, _ = S.make_env(slack_deg=(slack, slack), latency_ticks=lat, noisy_imu=False, seed=seed)
    if model is not None:
        env._motor = model[0]
        env.unwrapped.model.dof_armature[T.ROTOR_QVEL] = model[1]
    orig_tq = T.action_to_torque
    if model is not None and np.any(model[2]):
        T.action_to_torque = lambda a, w, p: orig_tq(a, w, p) - model[2] * w
    m, d = env.unwrapped.model, env.unwrapped.data
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    rng = np.random.default_rng(seed)
    kf, pid = StockKF(), StockPID()
    for _ in range(2000):                 # 10 s at rest first, as on the car before KEY1
        kf.update(0.0, ACC_LSB_G + rng.normal(0, acc_sd), (gyro_bias + rng.normal(0, gyro_sd)) / GYRO_LSB)
    r = np.array([0.0, 0.0, imu_h])
    acc6 = np.zeros(6)
    prev = np.floor(d.qpos[T.ROTOR_QPOS] / T.ENC_RAD_PER_COUNT)
    G, L, R, fell = [], [], [], None
    try:
        for k in range(int((seconds + settle) / DT)):
            mujoco.mj_rnePostConstraint(m, d)
            mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, body, acc6, 1)
            w = d.qvel[3:6]
            f = acc6[3:] + np.cross(acc6[:3], r) + np.cross(w, np.cross(w, r))
            accy = f[0] / 9.81 * ACC_LSB_G + rng.normal(0, acc_sd)
            accz = f[2] / 9.81 * ACC_LSB_G + rng.normal(0, acc_sd)
            gyro = w[1] * GYRO_LSB + gyro_bias + rng.normal(0, gyro_sd)
            angle = math.degrees(kf.update(accy, accz, gyro / GYRO_LSB))
            cnt = np.floor(d.qpos[T.ROTOR_QPOS] / T.ENC_RAD_PER_COUNT)
            el, er = cnt - prev
            prev = cnt
            pwm = pid.step(angle, gyro, el, er)
            a = 0.0 if pwm == 0 else math.copysign((abs(pwm) - 1300) / 1500, pwm)
            _, _, term, _, _ = env.step(np.array([a, a]))
            if k * DT >= settle:
                G.append((gyro - gyro_bias) / 16.4); L.append(pwm); R.append(pwm)
            if term:
                fell = k * DT
                break
    finally:
        T.action_to_torque = orig_tq
        env.close()
    if fell is not None:
        return dict(fell=fell)
    s = seg_stats(np.array(G[::2]), np.array(L[::2]), np.array(R[::2]), 2 * DT)   # sampled like the car log
    s["fell"] = None
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default="car_logs/2026-10-08/pid_mode1.txt")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--slack", type=float, nargs="+", default=[2, 3, 4])
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--seconds", type=float, default=30.0)
    # car, 2026-10-08: tick-to-tick sd with the motors off (handled, so upper
    # bounds) accY 118, gyro 7.4 counts; gyro mean while standing +39.8
    ap.add_argument("--acc-noise", type=float, default=120.0, help="raw counts")
    ap.add_argument("--gyro-noise", type=float, default=5.0, help="raw counts")
    ap.add_argument("--gyro-bias", type=float, default=40.0, help="raw counts, Gyro_Balance")
    args = ap.parse_args()

    c = load_car(args.log)
    print(f"== car: {args.log}")
    car, stand = report_car(c)
    port_check(c, stand)
    acc_sd, gyro_sd, gyro_bias = args.acc_noise, args.gyro_noise, args.gyro_bias

    if args.sim:
        import motor_fit as F
        steps = F.load_steps("car_logs/2026-10-08/motor_steps.csv")
        fits = {}
        for side in "LR":
            x, _, _ = F.fit_wheel(steps, side)
            xs = x.copy()
            xs[2], xs[4], xs[5] = x[2] / x[3], x[4] / x[3], x[5] / x[3]
            fits[side], _, _ = F.fit_wheel(steps, side, kv_fixed=1.0, x_start=xs)
        fitted = F.motor_from(fits["L"], fits["R"])
        print(f"\n== simulated stock PID, standing {args.seconds:.0f} s after 3 s; car: {fmt(car)}")
        for name, model in (("nominal motor", None), ("fitted motor A", fitted)):
            print(f"  {name}:")
            for g in args.slack:
                for lat in args.delay:
                    s = sim_pid(model, g, lat, acc_sd, gyro_sd, gyro_bias, seconds=args.seconds)
                    print(f"    {g:3.0f} deg {5 * lat:3d} ms: " + (f"FELL at {s['fell']:.1f} s" if s["fell"] is not None
                                                                      else fmt(s)), flush=True)


if __name__ == "__main__":
    sys.exit(main())
