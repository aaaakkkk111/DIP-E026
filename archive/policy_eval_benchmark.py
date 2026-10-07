"""Score a trained PPO policy on the same task as pid_eval_benchmark.py.

Same 20 held commands, same wrapped environment, same seeds, same printed
columns - so the two tables can be read side by side and the question "does
the learned controller beat a fixed-gain classical one" has a direct answer
rather than an impression.

Usage:
    python policy_eval_benchmark.py [path/to/model.zip]
"""
import os
import sys

import numpy as np
from stable_baselines3 import PPO

from train_yahboom_3d import make_wrapped_env, EVAL_COMMANDS, MAX_EPISODE_STEPS

DEFAULT_MODEL = "models/best_their/best_model.zip"


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MODEL
    if not os.path.exists(model_path):
        raise SystemExit(f"{model_path} not found - has training produced a checkpoint yet?")
    model = PPO.load(model_path)

    env = make_wrapped_env(is_eval=True)()
    w = env.env

    # A my_robot.xml checkpoint has the same 17-dim observation shape and would
    # load here without complaint, then saturate this plant's +/-0.6 Nm motors
    # with +/-4.0 Nm commands - scoring badly for the wrong reason. Catch it.
    trained_hi = float(np.max(model.action_space.high))
    if not np.isclose(trained_hi, w.tau_max, atol=1e-3):
        raise SystemExit(
            f"Action-space mismatch: checkpoint was trained with limit "
            f"+/-{trained_hi:.2f} Nm, this plant's limit is +/-{w.tau_max:.2f} Nm.\n"
            f"{model_path} almost certainly belongs to a different robot model."
        )

    print(f"Policy benchmark: {model_path}")
    print(f"({len(EVAL_COMMANDS)} episodes, cap {MAX_EPISODE_STEPS} steps, "
          f"deterministic actions)\n")
    print(f"{'cmd_v':>7} {'cmd_turn':>9} {'steps':>6} {'return':>9} "
          f"{'v_act':>8} {'turn_act':>9}")

    lengths, returns = [], []
    for ep in range(len(EVAL_COMMANDS)):
        obs, _ = env.reset(seed=1000 + ep)   # same seeds as the PID benchmark
        tv, tt = w.target_v_forward, w.target_v_turn
        n, total = 0, 0.0
        vs, turns = [], []
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, rew, term, trunc, _ = env.step(action)
            n += 1
            total += rew
            d = w.unwrapped.data
            _, _, local_vel, v_turn = w._decode_state(np.concatenate([d.qpos, d.qvel]))
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
    print("\n  PID baseline for comparison (pid_eval_benchmark.py):")
    print("    mean episode length : 1000.0 +/- 0.0")
    print("    mean return         :  360.2 +/- 163.3")
    env.close()


if __name__ == "__main__":
    main()
