"""Tick-by-tick check that firmware/policy.c builds the same 34 observations
as the training wrapper (train_real_robot.py PWMCommandWrapper).

Rolls the training wrapper forward, records the raw signals the firmware
would receive each tick (encoder count deltas, fused roll/pitch, gyro rates,
commands), feeds them through the C pipeline (policy_reset_state +
policy_odom_update + policy_build_obs_rp, via test_obs_builder.dll) and
compares every observation. Runs both the nominal eval car and a randomised,
noisy training car, so the IMU-error path is exercised too.

    cd firmware
    gcc -O2 -shared -o test_obs_builder.dll test_obs_builder.c policy.c -lm
    ..\\venv\\Scripts\\python.exe check_obs_builder.py
"""
import ctypes
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
os.chdir(os.path.dirname(HERE))
import train_real_robot as trr  # noqa: E402


def rollout(is_eval, seed, ticks=600):
    env = trr.PWMCommandWrapper(trr.RealRobotEnv(), is_eval=is_eval,
                                max_payload_kg=trr.MAX_PAYLOAD_KG,
                                max_v_forward=trr.MAX_V_FORWARD,
                                max_v_turn=trr.MAX_V_TURN,
                                fall_angle_limit=trr.FALL_ANGLE_LIMIT)
    if not is_eval:
        env.set_curriculum(0.5, trr.MAX_V_FORWARD, trr.MAX_V_TURN)
    rec = []
    orig = env._sense

    def sense(raw):
        out = orig(raw)
        roll, pitch, gyro, _, _ = out
        counts = np.floor(np.asarray(raw[7:9]) / trr.ENC_RAD_PER_COUNT)
        rec.append([counts[0], counts[1], roll, pitch, *gyro,
                    env.target_v_forward, env.target_v_turn])
        return out
    env._sense = sense

    rng = np.random.default_rng(seed)
    obs, _ = env.reset(seed=seed)
    observations = [obs]
    for _ in range(ticks):
        # Smooth random actions: enough to move the wheels both ways, keep it
        # upright long enough to cover the window and the history taps.
        a = np.clip(rng.normal(0.0, 0.4, 2), -1, 1)
        obs, _, term, _, _ = env.step(a)
        observations.append(obs)
        if term:
            break
    env.close()
    rec = np.array(rec, dtype=np.float64)
    counts = rec[:, :2]
    rec[1:, :2] = counts[1:] - counts[:-1]     # firmware gets per-tick deltas
    rec[0, :2] = 0.0
    return rec.astype(np.float32), np.array(observations, dtype=np.float32)


def main():
    lib = ctypes.CDLL(os.path.join(HERE, "test_obs_builder.dll"))
    lib.obs_builder_run.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_float),
                                    ctypes.POINTER(ctypes.c_float)]
    worst_all = 0.0
    for is_eval, seed in ((True, 1), (True, 2), (False, 3), (False, 4)):
        inp, ref = rollout(is_eval, seed)
        n = len(inp)
        out = np.zeros((n, 34), dtype=np.float32)
        lib.obs_builder_run(n, inp.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                            out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        err = np.abs(out[1:] - ref[1:])
        worst = err.max()
        worst_all = max(worst_all, worst)
        i = np.unravel_index(err.argmax(), err.shape)
        print(f"{'eval ' if is_eval else 'train'} seed {seed}: {n - 1} ticks, worst |firmware - training| "
              f"{worst:.2e} (obs[{i[1]}] at tick {i[0] + 1})")
    print("PASS" if worst_all < 1e-4 else "FAIL - firmware and training disagree")
    return 0 if worst_all < 1e-4 else 1


if __name__ == "__main__":
    sys.exit(main())
