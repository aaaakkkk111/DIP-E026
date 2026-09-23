import numpy as np
import pandas as pd
import torch
from stable_baselines3 import PPO

# Load trained model
model = PPO.load("models/ppo_inverted_pendulum.zip")

# Allow NumPy to print ENTIRE arrays without truncation (...)
np.set_printoptions(threshold=np.inf, linewidth=200, precision=6, suppress=True)

print("==================================================================")
print("1. FEATURE IMPORTANCE WEIGHTS OF INPUT LAYER (FC1: 4 Inputs -> 128 Neurons)")
print("==================================================================")

# Extract first layer weights [128, 4]
# Column 0: Cart Pos (x), Col 1: Pole Angle (θ), Col 2: Cart Vel (v), Col 3: Pole Ang Vel (ω)
fc1_weights = model.policy.mlp_extractor.policy_net[0].weight.data.cpu().numpy()
fc1_biases = model.policy.mlp_extractor.policy_net[0].bias.data.cpu().numpy()

# Calculate average absolute influence of each physical observation feature
feature_names = ["Cart Pos (x)", "Pole Angle (θ)", "Cart Vel (v)", "Pole Ang Vel (ω)"]
mean_abs_weights = np.abs(fc1_weights).mean(axis=0)

for name, weight_val in zip(feature_names, mean_abs_weights):
    print(f"Feature: {name:20s} | Mean Absolute Weight Influence: {weight_val:.6f}")

print("\n--- FC1 Weight Matrix [128 Neurons x 4 Inputs] (First 10 Neurons) ---")
print(" Neuron |   Cart Pos (x) | Pole Angle (θ) |   Cart Vel (v) | Pole Ang Vel (ω) |       Bias")
print("-" * 80)
for neuron_idx in range(10):
    w = fc1_weights[neuron_idx]
    b = fc1_biases[neuron_idx]
    print(f" #{neuron_idx:03d}   |  {w[0]:+12.6f} |  {w[1]:+12.6f} |  {w[2]:+12.6f} |  {w[3]:+12.6f} | {b:+10.6f}")

print("\n==================================================================")
print("2. EXPORTING ALL 34,563 NUMERICAL WEIGHTS TO CSV & TXT")
print("==================================================================")

records = []
for tensor_name, param_tensor in model.policy.named_parameters():
    array_data = param_tensor.data.cpu().numpy().flatten()
    for idx, val in enumerate(array_data):
        records.append({
            "tensor_name": tensor_name,
            "flattened_index": idx,
            "weight_value": float(val)
        })

df_weights = pd.DataFrame(records)
csv_output_path = "models/all_ppo_weights.csv"
df_weights.to_csv(csv_output_path, index=False)
print(f"✅ Successfully exported all {len(df_weights):,} numeric parameter values to '{csv_output_path}'")
