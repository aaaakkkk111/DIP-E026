"""Shared helpers for the simulation checks.

run()       one run of a checkpoint on the simulated car, with a chosen gear
            slack and sense-to-act delay, measured the way the car's flight
            recorder measures it.
car_stats() the same statistics from the car's 200 Hz flight recorder ('E'
            lines) in a log written by rl_trim_helper.py.

Statistics (t > 2 s in simulation; on the car, the balancing part of each
recording, i.e. before |pitch| first passes 8 deg on the final push):
  rate_sd   standard deviation of the pitch rate, deg/s   (the wobble size)
  freq      dominant wobble frequency, Hz, from pitch-rate sign changes
  sat       % of ticks with either wheel at |PWM| >= 2700
  pwm       mean |PWM|
  flips     % of ticks where a wheel's PWM changes sign
  turn_sd   standard deviation of (L - R) / 2: left-right wagging
"""
import collections
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from motor_model import action_to_pwm  # noqa: E402
import train_real_robot as T  # noqa: E402

DT = 0.005


def repo_path(p):
    return p if os.path.isabs(p) else os.path.join(ROOT, p)


def stats(rate_dps, L, R, pitch_deg=None):
    rate, L, R = (np.asarray(x, dtype=np.float64) for x in (rate_dps, L, R))
    zc = np.sum(np.sign(rate[1:] - rate.mean()) != np.sign(rate[:-1] - rate.mean()))
    s = dict(rate_sd=float(rate.std()), freq=float(zc / 2 / (len(rate) * DT)),
             sat=float(100 * np.mean(np.maximum(abs(L), abs(R)) >= 2700)),
             pwm=float(np.mean((abs(L) + abs(R)) / 2)),
             flips=float(50 * (np.mean(L[1:] * L[:-1] < 0) + np.mean(R[1:] * R[:-1] < 0))),
             turn_sd=float(np.std((L - R) / 2)))
    if pitch_deg is not None:
        s["pitch_sd"] = float(np.std(pitch_deg))
    return s


def make_env(slack_deg=(0.0, 0.0), latency_ticks=1, noisy_imu=True, seed=0, training_like=False):
    """Nominal car with the given slack (deg, per wheel) and delay (5 ms ticks).
    training_like=True instead draws everything the way training does."""
    env = T.PWMCommandWrapper(T.RealRobotEnv(), is_eval=not training_like,
                              max_payload_kg=0.0, fall_angle_limit=T.FALL_ANGLE_LIMIT)
    obs, _ = env.reset(seed=seed)
    if not training_like:
        T.set_gear_slack(env.unwrapped.model, *slack_deg)
        env._act_queue = collections.deque([np.zeros(2)] * latency_ticks)
        env._noisy = noisy_imu            # IMU noise only: eval mode has no offsets or bias
    return env, obs


def run(policy, v_cmd=0.0, w_cmd=0.0, seconds=10.0, skip_s=2.0, **env_kw):
    env, obs = make_env(**env_kw)
    m = env.unwrapped.model
    info = dict(slack=tuple(np.degrees(2 * m.tendon_range[:, 1]).round(1)),
                latency_ms=5 * len(env._act_queue), fell=None)
    Q, P, L, R, V, W = [], [], [], [], [], []
    for k in range(int(seconds / DT)):
        env.target_v_forward, env.target_v_turn = v_cmd, w_cmd
        a, _ = policy.predict(obs, deterministic=True)
        pwm = action_to_pwm(a)
        obs, _, term, _, _ = env.step(a)
        _, pitch, local_vel, v_turn = env._decode_state(env.unwrapped._get_obs())
        Q.append(math.degrees(obs[11])); P.append(math.degrees(pitch))
        L.append(pwm[0]); R.append(pwm[1]); V.append(local_vel[0]); W.append(v_turn)
        if term:
            info["fell"] = k * DT
            break
    env.close()
    n0 = int(skip_s / DT)
    if info["fell"] is not None or len(Q) <= n0 + 50:
        return info
    info.update(stats(Q[n0:], L[n0:], R[n0:], P[n0:]))
    info.update(v=float(np.mean(V[n0:])), w=float(np.mean(W[n0:])))
    return info


def car_stats(log_path, t_from, t_to):
    """Balancing statistics from every run ending between t_from and t_to
    ('HH:MM:SS', the logger's PC time)."""
    lines = open(repo_path(log_path), encoding="utf-8", errors="replace").read().splitlines()
    Q, L, R, P, runs = [], [], [], [], 0
    for i, x in enumerate(lines):
        f = x.split()
        if not (len(f) == 5 and f[1] == "R" and t_from <= f[0] <= t_to):
            continue
        runs += 1
        j = i + 1
        while j < len(lines) and " E " in lines[j]:
            p, q, _, l, r = (int(v) for v in lines[j].split()[3:8])
            if abs(p) > 800:          # the push that ended the run
                break
            P.append(p / 100); Q.append(q / 100); L.append(l); R.append(r)
            j += 1
    s = stats(Q, L, R, P)
    s.update(runs=runs, ticks=len(Q))
    return s


def load(path):
    from stable_baselines3 import PPO
    return PPO.load(repo_path(path), device="cpu")
