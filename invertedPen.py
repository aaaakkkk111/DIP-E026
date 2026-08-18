import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
import os

class PIDTuningEnv(gym.Env):
    """
    Custom Environment that wraps InvertedPendulum-v4.
    The RL agent interacts with this environment by choosing PID coefficients,
    and this environment uses those coefficients to drive the actual MuJoCo simulation.
    """
    def __init__(self, render_mode=None):
        super(PIDTuningEnv, self).__init__()

        # Load the base environment with optional rendering
        self.env = gym.make('InvertedPendulum-v4', render_mode=render_mode)

        # RL algorithms perform best with symmetric, normalized action spaces [-1, 1].
        # The agent will output 3 values representing Kp, Ki, Kd in the range [-1, 1].
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space remains identical to the base environment
        self.observation_space = self.env.observation_space

        # Initialize PID memory
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_obs = None

    def reset(self, seed=None, options=None):
        """Resets the environment and PID state at the start of each episode."""
        super().reset(seed=seed)
        self.integral = 0.0
        self.prev_error = 0.0
        # Gymnasium reset returns (observation, info)
        self.current_obs, info = self.env.reset(seed=seed, options=options)
        return self.current_obs, info

    def step(self, action):
        """
        Takes the RL agent's action (PID bounds), translates it to a control force, 
        and steps the base environment.
        """
        # 1. Map actions [-1, 1] to STRICTLY POSITIVE, SCALED ranges.
        # Adjusted to respect MuJoCo's strict [-3.0, 3.0] force limits.
        kp = (action[0] + 1.0) * 15.0   # Maps to [0, 30]
        ki = (action[1] + 1.0) * 1.0    # Maps to [0, 2]
        kd = (action[2] + 1.0) * 2.5    # Maps to [0, 5]

        # 2. Extract current state
        angle = self.current_obs[1]
        error = 0.0 - angle 

        # 3. Compute Integral and Derivative
        self.integral += error
        self.integral = np.clip(self.integral, -10.0, 10.0)  # Anti-windup safeguard

        derivative = error - self.prev_error

        # 4. Calculate Control Force (PID Equation)
        force = (kp * error) + (ki * self.integral) + (kd * derivative)
        self.prev_error = error

        # 5. Apply the force to the simulation
        clipped_force = np.clip([force], self.env.action_space.low, self.env.action_space.high)

        # Gymnasium step returns 5 values
        self.current_obs, base_reward, terminated, truncated, info = self.env.step(clipped_force)

        # 6. Reward Shaping (The "Suicide Bug" Fix)
        angle_penalty = (angle ** 2) * 10.0 
        wobble_penalty = (derivative ** 2) * 0.1

        # max(0.1, ...) guarantees the agent always gets a tiny positive reward for surviving,
        # but gets a MUCH bigger reward for surviving while perfectly still.
        custom_reward = max(0.1, base_reward - angle_penalty - wobble_penalty)

        return self.current_obs, custom_reward, terminated, truncated, info

    def render(self):
        return self.env.render()

    def close(self):
        self.env.close()


if __name__ == "__main__":
    # ---------------------------------------------------------
    # SET THIS TO TRUE TO TRAIN, OR FALSE TO JUST WATCH THE VISUALS
    TRAIN_MODE = True  
    # ---------------------------------------------------------

    MODEL_PATH = "ppo_pid_pendulum_model"

    if TRAIN_MODE:
        print("Setting up training environment (Visuals disabled for max speed)...")
        env = PIDTuningEnv(render_mode=None)

        print("Initializing PPO Agent...")
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.0003)

        print("Training the agent to find optimal PID coefficients...")
        # Increased timesteps to 1,000,000 to ensure consistent learning convergence
        model.learn(total_timesteps=1000000)

        print("Training completed. Evaluating the tuned controller...")
        mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=5)
        print(f"Mean reward: {mean_reward} +/- {std_reward}")

        print(f"Saving the trained model to {MODEL_PATH}.zip...")
        model.save(MODEL_PATH)
        env.close()

    # Test the trained agent WITH rendering so you can see it!
    print("Running a visual test...")
    test_env = PIDTuningEnv(render_mode="human")

    if not TRAIN_MODE:
        if os.path.exists(MODEL_PATH + ".zip"):
            print(f"Loading existing model from {MODEL_PATH}.zip...")
            model = PPO.load(MODEL_PATH, env=test_env)
        else:
            print(f"Error: Could not find {MODEL_PATH}.zip! Please set TRAIN_MODE = True to train and save the model first.")
            test_env.close()
            exit()

    obs, info = test_env.reset()

    for _ in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = test_env.step(action)

        if terminated or truncated:
            obs, info = test_env.reset()

    test_env.close()
