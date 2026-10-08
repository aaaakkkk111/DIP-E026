"""Do the policies wobble more in simulation when they see pitch and roll
through the car's own attitude filter instead of the true angles?

    python sim_tools/policy_filter_sim.py [--imu-h 0.04] [--slack 2] [--delay 1 2 3]

On the car (rl_mode.c att_update, the friend's imu.c): gyro integration plus
k = dt/(0.5 s + dt) of the accelerometer angle per tick, only while |a| is
within 15 % of 1 g. The accelerometer also feels the chassis' own
acceleration, which during a wobble is large. In training the policy sees
the true roll/pitch plus 0.002 rad of noise (PWMCommandWrapper._sense).

Here _sense is replaced, for this script only, by:
  accelerometer = MuJoCo's chassis acceleration (gravity included) at an IMU
                  imu_h above the axle, in g, + noise
  gyro          = body rates + noise (as training: GYRO_NOISE_RADS)
  roll, pitch   = rl_mode.c att_update() on those
  wheel speeds and v_forward as in training, with the filtered pitch.
Motor: fitted model A (motor_fit.py), slack and delay as given.
"""
import argparse
import math

import mujoco
import numpy as np

import motor_fit as F
import simlib as S
from motor_model import MotorParams

T = S.T
DT = 0.005
IMU_TAU_S = 0.5
FITTED_A = (MotorParams(dead_zone=np.array([[1397.0, 1398.0], [1395.0, 1394.0]]),
                        tau_scale=np.array([1.070, 1.111]), kv_scale=np.ones(2),
                        coulomb=np.array([0.01925, 0.01796])),
            np.array([1.448e-3, 1.509e-3]), np.zeros(2))
CAR = {"run8": 95.5, "run9": 68.2, "run11": 57.5}


def att_from_acc(a):
    return math.atan2(a[1], a[2]), math.atan2(-a[0], math.sqrt(a[1] ** 2 + a[2] ** 2))


class CarFilter:
    """rl_mode.c att_update(), same math."""

    def __init__(self):
        self.ready = False
        self.roll = self.pitch = 0.0

    def update(self, a, w):
        if not self.ready:
            self.roll, self.pitch = att_from_acc(a)
            self.ready = True
            return self.roll, self.pitch
        if abs(math.cos(self.pitch)) < 0.2:
            self.roll, self.pitch = att_from_acc(a)
            return self.roll, self.pitch
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        roll = self.roll + (w[0] + (w[1] * sr + w[2] * cr) * math.tan(self.pitch)) * DT
        pitch = self.pitch + (w[1] * cr - w[2] * sr) * DT
        n2 = a[0] ** 2 + a[1] ** 2 + a[2] ** 2
        if 0.7225 < n2 < 1.3225:
            k = DT / (IMU_TAU_S + DT)
            ra, pa = att_from_acc(a)
            roll += k * (ra - roll)
            pitch += k * (pa - pitch)
        self.roll = (roll + math.pi) % (2 * math.pi) - math.pi
        self.pitch = (pitch + math.pi) % (2 * math.pi) - math.pi
        return self.roll, self.pitch


def install_car_filter(env, imu_h, acc_noise_g, rng):
    """Replace env._sense (this instance only) by the car's sensing path."""
    m, d = env.unwrapped.model, env.unwrapped.data
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    r = np.array([0.0, 0.0, imu_h])
    acc6 = np.zeros(6)
    filt = CarFilter()
    orig = env._sense
    trace = []

    def sense(raw_obs):
        _, _, gyro, wheel_vel, _ = orig(raw_obs)          # gyro: body rates + training noise; encoders
        mujoco.mj_rnePostConstraint(m, d)
        mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, body, acc6, 1)
        w = d.qvel[3:6]
        a = (acc6[3:] + np.cross(acc6[:3], r) + np.cross(w, np.cross(w, r))) / 9.81
        a = a + rng.normal(0.0, acc_noise_g, 3)
        roll, pitch = filt.update(a, gyro)
        v_fwd = T.WHEEL_RADIUS * (0.5 * (wheel_vel[0] + wheel_vel[1]) + gyro[1]) * math.cos(pitch)
        v_fwd = float(np.clip(v_fwd, -T.V_FORWARD_CLIP, T.V_FORWARD_CLIP))
        qw, qx, qy, qz = raw_obs[3:7]
        trace.append((pitch, math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))))
        return roll, pitch, gyro, wheel_vel, v_fwd

    env._sense = sense
    return trace


def run(policy, slack, lat, filt, imu_h, acc_noise_g, seed):
    orig_make = S.make_env
    info = {}

    def make(**a):
        env, obs = orig_make(**a)
        if filt:
            rng = np.random.default_rng(seed + 100)
            info["trace"] = install_car_filter(env, imu_h, acc_noise_g, rng)
            env._hist = None
            env._enc_ring = None
            obs = env._get_conditioned_obs(env.unwrapped._get_obs())
        return env, obs

    S.make_env = make
    try:
        s = F.run_with_motor(policy, FITTED_A, slack_deg=(slack, slack), latency_ticks=lat, seed=seed)
    finally:
        S.make_env = orig_make
    if "trace" in info and s.get("fell") is None:
        tr = np.array(info["trace"][400:])
        s["filter_err"] = float(np.degrees(np.std(tr[:, 0] - tr[:, 1])))
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slack", type=float, nargs="+", default=[2.0])
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--imu-h", type=float, default=0.04, help="IMU height above the axle, m")
    ap.add_argument("--acc-noise", type=float, default=120 / 16384, help="accelerometer noise, g")
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()

    print(f"fitted motor A; IMU {100 * args.imu_h:.0f} cm above the axle; accelerometer noise "
          f"{args.acc_noise * 1000:.1f} mg. Cells: pitch-rate sd deg/s, Hz, mean |PWM| "
          "(and, with the car filter, the sd of filtered minus true pitch, deg)")
    for name, path in (("run8", "models/best_real_RUN8/best_model.zip"),
                       ("run9", "models/best_real_RUN9/best_model.zip"),
                       ("run11", "models/best_real_RUN11/best_model.zip")):
        pol = S.load(path)
        print(f"\n{name}  (car {CAR[name]} deg/s)")
        for g in args.slack:
            for lat in args.delay:
                cells = []
                for filt in (False, True):
                    ss = [run(pol, g, lat, filt, args.imu_h, args.acc_noise, sd) for sd in range(args.seeds)]
                    ok = [s for s in ss if s.get("fell") is None]
                    if not ok:
                        cells.append(f"{'FELL':>30s}")
                        continue
                    mn = lambda k: float(np.mean([s[k] for s in ok]))
                    cell = f"{mn('rate_sd'):6.1f} {mn('freq'):5.1f} {mn('pwm'):5.0f}"
                    if filt:
                        cell += f"  err {mn('filter_err'):4.2f}"
                    cells.append(cell + ("F" if len(ok) < len(ss) else " "))
                print(f"  {g:3.0f} deg {5 * lat:3d} ms   true angles: {cells[0]}   car filter: {cells[1]}", flush=True)


if __name__ == "__main__":
    main()
