"""Save a trained InvertedPendulum rollout as a GIF for headless systems."""

import argparse
import os
import sys
from pathlib import Path

import gymnasium as gym
from PIL import Image
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
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/inverted_pendulum_trained.gif"),
        help="GIF output path",
    )
    parser.add_argument("--seed", type=int, default=2026, help="Demonstration seed")
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=4,
        help="Capture one frame every N physics steps; larger values create smaller files",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Do not open the GIF automatically after generation",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.frame_stride <= 0:
        raise ValueError("--frame-stride must be greater than zero")
    if not args.model.exists():
        raise FileNotFoundError(f"Model not found: {args.model}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = PPO.load(args.model)
    env = gym.make(ENV_ID, render_mode="rgb_array", width=480, height=480)
    frames: list[Image.Image] = []
    total_reward = 0.0

    try:
        observation, info = env.reset(seed=args.seed)
        frames.append(Image.fromarray(env.render()))

        for step in range(1, 1001):
            action, _state = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += float(reward)

            if step % args.frame_stride == 0 or terminated or truncated:
                frames.append(Image.fromarray(env.render()))
            if terminated or truncated:
                break
    finally:
        env.close()

    # MuJoCo runs at 50 FPS; the duration preserves real-time speed after subsampling.
    frame_duration_ms = 20 * args.frame_stride
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
    )
    print(f"Animation: {args.output}")
    print(f"Episode steps: {step}")
    print(f"Episode reward: {total_reward:.0f}")
    print(f"Saved frames: {len(frames)}")

    # On Windows, open the result with the default image viewer after generation.
    if not args.no_open:
        if sys.platform == "win32":
            os.startfile(args.output.resolve())
        else:
            print(f"Open this file manually: {args.output.resolve()}")


if __name__ == "__main__":
    main()
