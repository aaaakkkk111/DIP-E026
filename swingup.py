"""
Hybrid Swing-Up & Balance Simulation for Inverted Pendulum.
Tuned for Early Handover (Step 30-60).
Phase 1: Fast Energy Controller swings pole from 180° (downward) to 0° (upright).
Phase 2: Trained PPO model catches and balances the pole at 0°.
"""

import os
import math
import time
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO


def normalize_angle(angle_rad: float) -> float:
    """Wraps angle into range [-pi, pi]. 0 is upright vertical, +/-pi is hanging down."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def compute_energy_swing_up_force(
    angle_rad: float,
    angular_velocity: float,
    cart_pos: float = 0.0,
    cart_vel: float = 0.0,
    k_energy: float = 25.0,
    max_swing_force: float = 12.0,
    track_limit: float = 1.5,
    max_force: float = 20.0
) -> float:
    """Fast energy pumping controller for early swing-up."""
    m, g, L = 0.1, 9.81, 0.6
    J = (1.0 / 3.0) * m * (2.0 * L)**2

    E_current = 0.5 * J * (angular_velocity**2) + m * g * L * (math.cos(angle_rad) - 1.0)
    E_deficit = 0.0 - E_current

    pump_term = k_energy * E_deficit * angular_velocity * math.cos(angle_rad)
    swing_force = -max_swing_force * math.tanh(pump_term)

    centering_force = -1.2 * cart_pos - 0.4 * cart_vel

    boundary_force = 0.0
    warn_threshold = track_limit * 0.70
    if abs(cart_pos) > warn_threshold:
        overstep = abs(cart_pos) - warn_threshold
        push_direction = -1.0 if cart_pos > 0 else 1.0
        boundary_force = push_direction * (30.0 * (overstep**2) + 2.0 * abs(cart_vel))

    total_force = swing_force + centering_force + boundary_force
    return float(np.clip(total_force, -max_force, max_force))


class HybridSwingUpApp:
    def __init__(self, model_path: str = "models/ppo_inverted_pendulum.zip", track_limit: float = 1.5):
        # 1. Initialize environment with human rendering
        self.env = gym.make("InvertedPendulum-v5", render_mode="human")
        self.max_force = 20.0
        self.track_limit = track_limit

        # Physical track boundaries and unlocked pole hinge
        unwrapped = self.env.unwrapped
        unwrapped.model.jnt_limited[0] = True
        unwrapped.model.jnt_range[0] = [-self.track_limit, self.track_limit]
        unwrapped.model.jnt_limited[1] = False
        unwrapped.model.actuator_ctrlrange[0] = [-self.max_force, self.max_force]

        # 2. Load trained PPO balancing model
        self.model = None
        if os.path.exists(model_path):
            print(f"Loading PPO balance model from {model_path}...")
            self.model = PPO.load(model_path)
        elif os.path.exists("ppo_pid_pendulum_model.zip"):
            print("Loading PPO balance model from ppo_pid_pendulum_model.zip...")
            self.model = PPO.load("ppo_pid_pendulum_model.zip")
        else:
            print("Warning: No PPO model found. PPO catch phase will apply zero force.")

        # 3. Early Handover Trigger Settings
        self.catch_angle_deg = 12.0   # Expanded catch window (|theta| <= 12.0°)
        self.catch_angle_rad = math.radians(self.catch_angle_deg)
        self.max_catch_omega = 4.5    # Expanded velocity window (|omega| <= 4.5 rad/s)

    def reset_downward(self):
        """Resets environment with pole hanging straight down (180 degrees) and returns updated obs."""
        self.env.reset()
        unwrapped = self.env.unwrapped
        qpos = np.array([0.0, np.pi], dtype=np.float64)
        qvel = np.array([0.0, 0.0], dtype=np.float64)
        unwrapped.set_state(qpos, qvel)
        obs = unwrapped._get_obs()
        return obs

    def update_camera(self, cart_pos: float):
        """Updates camera lookat target so viewport follows cart horizontally."""
        try:
            unwrapped = self.env.unwrapped
            renderer = getattr(unwrapped, "mujoco_renderer", None)
            if renderer is not None:
                viewer = getattr(renderer, "viewer", None)
                if viewer is not None and hasattr(viewer, "cam"):
                    viewer.cam.lookat[0] = float(cart_pos)
                    viewer.cam.lookat[1] = 0.0
                    viewer.cam.lookat[2] = 0.3
                    viewer.cam.distance = 2.8
                elif hasattr(renderer, "default_cam"):
                    renderer.default_cam.lookat[0] = float(cart_pos)
        except Exception:
            pass

    def run_simulation(self, total_steps: int = 1000):
        obs = self.reset_downward()
        current_phase = "SWING_UP"

        print("\n--- Starting Fast Swing-Up & Early Handover Simulation ---")
        print(f"Initial state: Pendulum hanging downward at 180° | Track Limits: [-{self.track_limit}m, +{self.track_limit}m]\n")

        for step in range(total_steps):
            cart_pos, pole_angle, cart_vel, pole_ang_vel = obs

            norm_angle = normalize_angle(pole_angle)
            angle_deg = math.degrees(norm_angle)

            in_angle_window = abs(norm_angle) <= self.catch_angle_rad
            in_velocity_window = abs(pole_ang_vel) <= self.max_catch_omega

            if current_phase == "SWING_UP":
                if in_angle_window and in_velocity_window:
                    current_phase = "PPO_BALANCE"
                    print(f"⚡ Early Handover Triggered at Step {step}: Angle={angle_deg:.1f}°, Omega={pole_ang_vel:.2f} rad/s -> Handing over to PPO!")
            elif current_phase == "PPO_BALANCE":
                if abs(norm_angle) > math.radians(15.0):
                    current_phase = "SWING_UP"

            if current_phase == "PPO_BALANCE" and self.model is not None:
                obs_ppo = np.array([cart_pos, norm_angle, cart_vel, pole_ang_vel], dtype=np.float32)
                action, _ = self.model.predict(obs_ppo, deterministic=True)
                force = float(action[0])

                if abs(cart_pos) > self.track_limit * 0.8:
                    push_dir = -1.0 if cart_pos > 0 else 1.0
                    force += push_dir * 10.0
            else:
                force = compute_energy_swing_up_force(
                    angle_rad=norm_angle,
                    angular_velocity=pole_ang_vel,
                    cart_pos=cart_pos,
                    cart_vel=cart_vel,
                    k_energy=25.0,
                    max_swing_force=12.0,
                    track_limit=self.track_limit,
                    max_force=self.max_force
                )

            clipped_force = np.array([force], dtype=np.float32)
            obs, reward, terminated, truncated, info = self.env.step(clipped_force)

            self.update_camera(cart_pos)

            if step % 25 == 0:
                print(
                    f"Step {step:04d} | Phase: {current_phase:11s} | "
                    f"Pos: {cart_pos:+5.2f}m | Angle: {angle_deg:+6.1f}° | Omega: {pole_ang_vel:+5.2f} rad/s | Force: {clipped_force[0]:+5.2f} N"
                )

            time.sleep(0.02)

        self.env.close()


if __name__ == "__main__":
    app = HybridSwingUpApp(track_limit=1.5)
    app.run_simulation(total_steps=1000)