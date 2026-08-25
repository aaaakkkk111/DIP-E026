import os
import math
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy


class PIDTuningEnv(gym.Env):
    """
    Custom Environment wrapping InvertedPendulum-v5.
    Synchronized with interactive_control.py for seamless RL model evaluation and deployment.
    """
    def __init__(self, render_mode=None, target_angle_deg=0.0):
        super(PIDTuningEnv, self).__init__()

        # Load base MuJoCo environment
        self.env = gym.make('InvertedPendulum-v5', render_mode=render_mode)
        
        # Match actuator force limits with interactive_control.py
        self.max_force = 20.0
        self.env.unwrapped.model.jnt_limited[0] = False
        self.env.unwrapped.model.actuator_ctrlrange[0] = [-self.max_force, self.max_force]

        # Action space: normalized [-1, 1] for Kp, Ki, Kd
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space matches base environment
        self.observation_space = self.env.observation_space

        # PID memory and parameters
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_obs = None
        self.dt = 0.02  # Time step

        # Setpoint state
        self.target_angle_deg = target_angle_deg
        self.target_angle_rad = math.radians(self.target_angle_deg)

    def reset(self, seed=None, options=None):
        """Resets the environment and PID state."""
        super().reset(seed=seed)
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_obs, info = self.env.reset(seed=seed, options=options)
        return self.current_obs, info

    def step(self, action):
        """
        Translates RL action into Kp, Ki, Kd gains and applies control force
        matching the exact algorithm used in interactive_control.py.
        """
        # 1. Map actions [-1, 1] to Gain Ranges: Kp in [0, 100], Ki in [0, 10], Kd in [0, 20]
        kp = (action[0] + 1.0) * 50.0
        ki = (action[1] + 1.0) * 5.0
        kd = (action[2] + 1.0) * 10.0

        # 2. Extract state variables
        cart_pos = self.current_obs[0]
        pole_angle = self.current_obs[1]
        cart_vel = self.current_obs[2]
        pole_ang_vel = self.current_obs[3]

        # 3. Error and Integral calculation matching interactive_control.py
        error = pole_angle - self.target_angle_rad
        self.integral += error * self.dt
        self.integral = float(np.clip(self.integral, -6.0, 6.0))

        derivative = pole_ang_vel

        # 4. Target angle gain scheduling (/ cos_t)
        cos_t = max(math.cos(self.target_angle_rad), 0.2)
        kp_eff = kp / cos_t
        ki_eff = ki / cos_t
        kd_eff = kd / cos_t

        # 5. Feedforward equilibrium force
        if abs(self.target_angle_rad) > 1e-4:
            feedforward = 0.8807 * (
                math.tan(self.target_angle_rad) / math.tan(math.radians(30.0))
            )
        else:
            feedforward = 0.0

        # 6. PID force calculation
        pid_force = feedforward + (kp_eff * error) + (kd_eff * derivative) + (ki_eff * self.integral)

        # 7. Soft return-to-origin cart correction when upright
        if abs(self.target_angle_rad) < 1e-4:
            k_cart = 0.8
            k_vel = 1.2
            cart_correction = (k_cart * cart_pos + k_vel * cart_vel)
            pid_force += cart_correction

        # 8. Force clipping
        clipped_force = np.clip([pid_force], -self.max_force, self.max_force)

        # 9. Step physical simulation
        self.current_obs, base_reward, terminated, truncated, info = self.env.step(clipped_force)

        # 10. Reward Shaping relative to setpoint
        angle_err = pole_angle - self.target_angle_rad
        angle_penalty = (angle_err ** 2) * 20.0
        wobble_penalty = (derivative ** 2) * 0.01

        custom_reward = max(0.1, base_reward - angle_penalty - wobble_penalty)

        return self.current_obs, custom_reward, terminated, truncated, info

    def render(self):
        return self.env.render()

    def close(self):
        self.env.close()


if __name__ == "__main__":
    # SET THIS TO TRUE TO TRAIN, OR FALSE TO JUST WATCH THE VISUALS
    TRAIN_MODE = True

    MODEL_PATH = "ppo_pid_pendulum_model"

    if TRAIN_MODE:
        print("Setting up training environment (Visuals disabled for max speed)...")
        env = PIDTuningEnv(render_mode=None)

        print("Initializing PPO Agent...")
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.0003, n_steps=4096)

        print("Training the agent to find optimal PID coefficients matching interactive controller...")
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
