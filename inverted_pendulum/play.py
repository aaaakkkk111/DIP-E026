"""Load a trained PPO model and use it to control InvertedPendulum."""

import argparse
from pathlib import Path

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO


ENV_ID = "InvertedPendulum-v5"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("models/ppo_inverted_pendulum.zip"),
        help="The .zip model saved by train_ppo.py",
    )
    parser.add_argument("--episodes", type=int, default=5, help="Number of episodes")
    parser.add_argument("--seed", type=int, default=100, help="Demonstration seed")
    parser.add_argument(
        "--reset-noise",
        type=float,
        default=0.01,
        help="Initial position and velocity noise; the standard environment uses 0.01",
    )
    parser.add_argument(
        "--demo-push",
        action="store_true",
        help="Briefly push the cart near steps 200 and 500 to make recovery visible",
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Disable the animation window and print scores only",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = args.model
    if model_path.suffix != ".zip":
        model_path = model_path.with_suffix(".zip")
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model not found: {model_path}. Run python train_ppo.py first."
        )

    model = PPO.load(model_path)
    render_mode = None if args.no_render else "human"
    env = gym.make(
        ENV_ID,
        render_mode=render_mode,
        reset_noise_scale=args.reset_noise,
    )

    try:
        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            total_reward = 0.0
            steps = 0

            while True:
                # deterministic=True selects the policy's best action during playback.
                action, _state = model.predict(observation, deterministic=True)
                next_step = steps + 1

                # Optional short disturbances make the policy's recovery behavior visible.
                if args.demo_push and 200 <= next_step < 225:
                    action = np.clip(
                        action + 2.0, env.action_space.low, env.action_space.high
                    )
                    if next_step == 200:
                        print("  Step 200: applying a brief rightward disturbance...")
                elif args.demo_push and 500 <= next_step < 525:
                    action = np.clip(
                        action - 2.0, env.action_space.low, env.action_space.high
                    )
                    if next_step == 500:
                        print("  Step 500: applying a brief leftward disturbance...")

                observation, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                steps += 1

                if terminated or truncated:
                    reason = "pole angle exceeded the limit" if terminated else "survived all 1,000 steps"
                    print(
                        f"Episode {episode + 1}: {steps} steps, "
                        f"reward {total_reward:.0f}, reason: {reason}"
                    )
                    break
    finally:
        env.close()


if __name__ == "__main__":
    main()
