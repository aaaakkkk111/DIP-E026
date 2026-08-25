import os
import math
import random
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv


class PIDTuningEnv(gym.Env):
    """
    Robust Environment wrapping InvertedPendulum-v5 with Domain Randomization.
    Includes randomized manual force push disturbances and setpoint tracking during training.
    """
    def __init__(self, render_mode=None):
        super(PIDTuningEnv, self).__init__()

        # Load base MuJoCo environment
        self.env = gym.make('InvertedPendulum-v5', render_mode=render_mode)

        # Actuator force limit
        self.max_force = 20.0
        self.env.unwrapped.model.jnt_limited[0] = False
        self.env.unwrapped.model.actuator_ctrlrange[0] = [-self.max_force, self.max_force]

        # Action space: normalized [-1, 1] for Kp, Ki, Kd
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space: identical to base environment
        self.observation_space = self.env.observation_space

        # PID memory and parameters
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_obs = None
        self.dt = 0.02  # Time step

        # Setpoint state
        self.target_angle_rad = 0.0

    def reset(self, seed=None, options=None):
        """Resets environment with randomized target angles within safe Gym limits."""
        super().reset(seed=seed)
        self.integral = 0.0
        self.prev_error = 0.0

        # Randomize target angle within safe window (-5° to +5°) to prevent Gym 0.2 rad auto-termination
        self.target_angle_rad = math.radians(random.uniform(-5.0, 5.0))

        self.current_obs, info = self.env.reset(seed=seed, options=options)
        return self.current_obs, info

    def step(self, action):
        """
        Translates RL action into Kp, Ki, Kd gains and applies control force,
        including randomized push disturbances to simulate manual force inputs.
        """
        # 1. Map actions [-1, 1] to Gain Ranges: Kp in [0, 60], Ki in [0, 10], Kd in [0, 15]
        kp = (action[0] + 1.0) * 30.0
        ki = (action[1] + 1.0) * 5.0
        kd = (action[2] + 1.0) * 7.5

        # 2. Extract state variables
        cart_pos = self.current_obs[0]
        pole_angle = self.current_obs[1]
        cart_vel = self.current_obs[2]
        pole_ang_vel = self.current_obs[3]

        # 3. Error and Integral calculation
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

        # 8. Inject Randomized Push Force Disturbance
        # 5% chance per step to apply a random push anywhere between -2.5 N and +2.5 N
        random_disturbance = random.uniform(-2.5, 2.5) if random.random() < 0.05 else 0.0
        total_force = pid_force + random_disturbance

        # 9. Force clipping
        clipped_force = np.clip([total_force], -self.max_force, self.max_force)

        # 10. Step physical simulation
        self.current_obs, base_reward, terminated, truncated, info = self.env.step(clipped_force)

        # 11. Optimized Reward Shaping relative to setpoint
        angle_err = pole_angle - self.target_angle_rad
        angle_penalty = (angle_err ** 2) * 10.0  # Reduced penalty scale to avoid heavily punishing necessary tilt recovery
        wobble_penalty = (derivative ** 2) * 0.005

        custom_reward = max(0.1, base_reward - angle_penalty - wobble_penalty)

        # Precision Bonus: Reward the agent when angle error is within 0.5 degrees
        if abs(angle_err) < math.radians(0.5):
            custom_reward += 0.5

        return self.current_obs, custom_reward, terminated, truncated, info

    def render(self):
        return self.env.render()

    def close(self):
        self.env.close()


if __name__ == "__main__":
    # Set to True to train the model, or False to skip training and run visual test directly
    TRAIN_MODE = True
    MODEL_PATH = "ppo_pid_pendulum_model"

    if TRAIN_MODE:
        num_cpu = max(1, (os.cpu_count() or 2) - 1)
        print(f"Setting up training environment with {num_cpu} parallel CPU workers...")

        env = make_vec_env(
            PIDTuningEnv,
            n_envs=num_cpu,
            vec_env_cls=SubprocVecEnv,
        )

        # Custom Neural Network Architecture: Larger capacity for complex recovery policies
        policy_kwargs = dict(net_arch=[256, 256])

        print("Initializing PPO Agent on CPU with expanded neural network [256, 256]...")
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=0.0003,
            n_steps=2048,
            batch_size=128,
            n_epochs=10,
            ent_coef=0.005,  # Encourages exploration to find sharper control gains
            policy_kwargs=policy_kwargs,
            device="cpu",
        )

        print("Training the agent with parallel workers, expanded network, and precision bonus...")
        model.learn(total_timesteps=1000000)

        print("Training completed. Evaluating tuned controller...")
        eval_env = PIDTuningEnv(render_mode=None)
        mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)
        print(f"Mean reward: {mean_reward} +/- {std_reward}")

        print(f"Saving trained model to {MODEL_PATH}.zip...")
        model.save(MODEL_PATH)
        env.close()
        eval_env.close()

    # ---------------------------------------------------------
    # VISUAL TEST & LIVE KP, KI, KD PRINTOUT
    # ---------------------------------------------------------
    print("\nRunning visual test and printing live PID gains...")
    test_env = PIDTuningEnv(render_mode="human")

    if os.path.exists(MODEL_PATH + ".zip"):
        print(f"Loading model from {MODEL_PATH}.zip...")
        model = PPO.load(MODEL_PATH, env=test_env, device="cpu")
    else:
        print(f"Error: Could not find {MODEL_PATH}.zip! Please set TRAIN_MODE = True first.")
        test_env.close()
        exit()

    obs, info = test_env.reset()

    for step_idx in range(1000):
        action, _states = model.predict(obs, deterministic=True)

        # Convert raw [-1, 1] actions into Kp, Ki, and Kd gain values
        kp = (action[0] + 1.0) * 30.0
        ki = (action[1] + 1.0) * 5.0
        kd = (action[2] + 1.0) * 7.5

        # Print the dynamically predicted gains every 50 steps
        if step_idx % 50 == 0:
            print(f"Step {step_idx:04d} -> Kp: {kp:5.2f} | Ki: {ki:5.2f} | Kd: {kd:5.2f}")

        obs, reward, terminated, truncated, info = test_env.step(action)

        if terminated or truncated:
            obs, info = test_env.reset()

    test_env.close()
