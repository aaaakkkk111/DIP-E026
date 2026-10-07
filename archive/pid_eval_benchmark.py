"""Score the cascade PID on exactly the metric EvalCallback reports.

Runs the same 20 held commands (EVAL_COMMANDS) through the same wrapped
environment PPO is evaluated in, so the mean episode length and mean return
printed here are directly comparable to the ep_len_mean / mean_reward that
EvalCallback logs during training. That gives the RL run a concrete target:
a fixed-gain classical controller's score on the identical task.

Gains are retuned for 80Hz (the RL control rate); the 200Hz gains in
pid_baseline.py do not transfer - see the rate sweep in train_yahboom_3d.py.
"""
import numpy as np

from train_yahboom_3d import make_wrapped_env, FRAME_SKIP, EVAL_COMMANDS, MAX_EPISODE_STEPS
from pid_baseline import CascadePID

GAINS_80HZ = (19.96, 1.604, 1.947, 2.250)

# kp_turn was left at CascadePID's 0.5 default in the first version of this
# benchmark, which reached only 0.348 rad/s against a 0.5 command and made the
# turn axis look like a plant limitation. It is not: sweeping the gain against
# commanded yaw (scratchpad/turn_ceiling.py) shows 0.485 rad/s at kp_turn=4.0,
# nothing falls anywhere in the sweep, and 1.93 rad/s is reachable at a 2.0
# command - including while driving forward at 0.3 m/s. The low gain was an
# arbitrary default of mine, so the baseline it produced understated the PID.
KP_TURN = 4.0


def main():
    env = make_wrapped_env(is_eval=True)()
    w = env.env
    dt = FRAME_SKIP * w.unwrapped.model.opt.timestep

    print(f"PID benchmark on the EvalCallback task "
          f"({len(EVAL_COMMANDS)} episodes, {1/dt:.0f}Hz, cap {MAX_EPISODE_STEPS} steps)\n")
    print(f"{'cmd_v':>7} {'cmd_turn':>9} {'steps':>6} {'return':>9} "
          f"{'v_act':>8} {'turn_act':>9}")

    lengths, returns = [], []
    for ep in range(len(EVAL_COMMANDS)):
        env.reset(seed=1000 + ep)
        pid = CascadePID(*GAINS_80HZ, kp_turn=KP_TURN)
        tv, tt = w.target_v_forward, w.target_v_turn
        n, total = 0, 0.0
        vs, turns = [], []
        while True:
            d = w.unwrapped.data
            raw = np.concatenate([d.qpos, d.qvel])
            _, pitch, local_vel, v_turn = w._decode_state(raw)
            tl, tr = pid(pitch, d.qvel[4], local_vel[0], v_turn, dt,
                         target_v=tv, target_yaw=tt)
            _, rew, term, trunc, _ = env.step(np.array([tl, tr], dtype=np.float32))
            n += 1
            total += rew
            vs.append(local_vel[0])
            turns.append(v_turn)
            if term or trunc:
                break
        half = slice(n // 2, None)
        lengths.append(n)
        returns.append(total)
        print(f"{tv:7.2f} {tt:9.2f} {n:6d} {total:9.1f} "
              f"{np.mean(vs[half]):8.3f} {np.mean(turns[half]):9.3f}")

    print(f"\n  mean episode length : {np.mean(lengths):7.1f} +/- {np.std(lengths):.1f}"
          f"   (cap {MAX_EPISODE_STEPS})")
    print(f"  mean return         : {np.mean(returns):7.1f} +/- {np.std(returns):.1f}")
    env.close()


if __name__ == "__main__":
    main()
