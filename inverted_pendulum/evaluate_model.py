"""Evaluate a trained PPO model and write machine-readable replication results."""

import argparse
import json
from importlib.metadata import version
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import PPO


ENV_ID = "InvertedPendulum-v5"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("models/ppo_inverted_pendulum.zip"),
        help="Trained PPO model",
    )
    parser.add_argument("--episodes", type=int, default=20, help="Evaluation episodes")
    parser.add_argument("--seed", type=int, default=100, help="First episode seed")
    parser.add_argument(
        "--expected-reward",
        type=float,
        default=1000.0,
        help="Require every episode to receive this reward; use a negative value to disable",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/replication_results.json"),
        help="JSON output path",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.episodes <= 0:
        raise ValueError("--episodes must be greater than zero")
    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    model = PPO.load(args.model)
    env = gym.make(ENV_ID)
    rewards: list[float] = []
    lengths: list[int] = []

    try:
        for episode in range(args.episodes):
            observation, _info = env.reset(seed=args.seed + episode)
            episode_reward = 0.0
            steps = 0

            while True:
                action, _state = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, _info = env.step(action)
                episode_reward += float(reward)
                steps += 1
                if terminated or truncated:
                    break

            rewards.append(episode_reward)
            lengths.append(steps)
            print(
                f"Episode {episode + 1:02d}: reward={episode_reward:.0f}, "
                f"length={steps}"
            )
    finally:
        env.close()

    mean_reward = sum(rewards) / len(rewards)
    result = {
        "environment": ENV_ID,
        "model": str(args.model),
        "episodes": args.episodes,
        "first_seed": args.seed,
        "deterministic": True,
        "episode_rewards": rewards,
        "episode_lengths": lengths,
        "mean_reward": mean_reward,
        "minimum_reward": min(rewards),
        "maximum_reward": max(rewards),
        "all_expected_reward": (
            all(reward == args.expected_reward for reward in rewards)
            if args.expected_reward >= 0
            else None
        ),
        "versions": {
            "gymnasium": version("gymnasium"),
            "mujoco": version("mujoco"),
            "stable-baselines3": version("stable-baselines3"),
            "torch": version("torch"),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=True), encoding="utf-8"
    )
    print(f"\nMean reward: {mean_reward:.1f}")
    print(f"Results written to: {args.output}")

    if args.expected_reward >= 0 and not result["all_expected_reward"]:
        raise RuntimeError(
            f"Replication failed: not every episode received {args.expected_reward:.0f}"
        )

    if args.expected_reward >= 0:
        print(
            f"Replication passed: all {args.episodes} episodes received "
            f"{args.expected_reward:.0f}."
        )


if __name__ == "__main__":
    main()

