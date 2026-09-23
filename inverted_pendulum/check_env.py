"""Verify that Gymnasium, MuJoCo, and InvertedPendulum are installed correctly."""

from importlib.metadata import version

import gymnasium as gym
import numpy as np


ENV_ID = "InvertedPendulum-v5"


def main() -> None:
    print("=== Package versions ===")
    for package_name in ("gymnasium", "mujoco", "stable-baselines3", "torch"):
        print(f"{package_name:18s}: {version(package_name)}")

    # This numerical smoke test does not open a window, so it is safe to run first.
    env = gym.make(ENV_ID)
    try:
        observation, info = env.reset(seed=42)
        zero_action = np.zeros(env.action_space.shape, dtype=env.action_space.dtype)
        next_observation, reward, terminated, truncated, step_info = env.step(
            zero_action
        )

        print("\n=== Environment information ===")
        print(f"Environment ID       : {ENV_ID}")
        print(f"Action space         : {env.action_space}")
        print(f"Observation space    : {env.observation_space}")
        print(f"Maximum episode steps: {env.spec.max_episode_steps}")
        print(f"Seeded observation   : {np.array2string(observation, precision=6)}")
        print(f"After zero action    : {np.array2string(next_observation, precision=6)}")
        print(f"Step reward          : {reward}")
        print(f"Terminated           : {terminated}")
        print(f"Truncated            : {truncated}")
        print(f"Reset info           : {info}")
        print(f"Step info            : {step_info}")

        assert env.action_space.shape == (1,), "The action dimension must be 1"
        assert env.observation_space.shape == (4,), "The observation dimension must be 4"
        assert env.action_space.contains(zero_action), "Zero force must be a valid action"
    finally:
        env.close()

    print("\nEnvironment check passed: reset and one simulation step succeeded.")


if __name__ == "__main__":
    main()
