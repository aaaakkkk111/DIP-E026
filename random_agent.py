"""Run a non-learning random controller as the pre-training baseline."""

import argparse

import gymnasium as gym


ENV_ID = "InvertedPendulum-v5"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=5, help="Number of episodes")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Disable the animation window and print results only",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    render_mode = None if args.no_render else "human"
    env = gym.make(ENV_ID, render_mode=render_mode)
    env.action_space.seed(args.seed)

    try:
        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            total_reward = 0.0
            steps = 0

            while True:
                # sample() draws a force from [-3, 3] without using the observation.
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                steps += 1

                if terminated or truncated:
                    reason = "pole angle exceeded the limit" if terminated else "1,000-step limit reached"
                    print(
                        f"Episode {episode + 1}: {steps} steps, "
                        f"reward {total_reward:.0f}, reason: {reason}"
                    )
                    break
    finally:
        env.close()


if __name__ == "__main__":
    main()
