"""
enjoy_yahboom.py - Watch your trained PPO agent balance the robot!
"""
import time
import numpy as np
from stable_baselines3 import PPO
from train_yahboom_3d import YahboomEnv, WideCatchDisturbanceWrapper3D
import mujoco.viewer

# 1. Load the best saved model
model_path = "models/ppo_yahboom_3d.zip"
print(f"Loading model from {model_path}...")
model = PPO.load(model_path)

# 2. Create a fresh environment for testing
base_env = YahboomEnv()
env = WideCatchDisturbanceWrapper3D(base_env, is_eval=True)

# 3. Start the visualizer and the control loop
obs, _ = env.reset()

print("Starting simulation... (Close the MuJoCo window to stop)")
with mujoco.viewer.launch_passive(env.unwrapped.model, env.unwrapped.data) as viewer:
    while viewer.is_running():
        # The AI looks at the state (obs) and decides how to spin the wheels (action)
        action, _states = model.predict(obs, deterministic=True)
        
        # Step the physics simulation
        obs, reward, terminated, truncated, info = env.step(action)
        
        # Sync the visual viewer with the physics
        viewer.sync()
        
        # If the robot falls over, reset it
        if terminated or truncated:
            obs, _ = env.reset()
            
        # Slow down the loop so it plays in real-time (MuJoCo timestep is 0.005s)
        time.sleep(0.005)
