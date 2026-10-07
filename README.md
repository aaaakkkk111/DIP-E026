**Aim**

To test whether **LLM-designed reward + PPO residual** improved PID convergence compared with PID-only, LLM-tuned PID, and fixed-reward PPO baselines in the same tzejun measured-plant MuJoCo environment (**`experiment_comparison/protocol.json`**).

**Methodology**

Four controllers were compared: **PID-only**, **LLM-PID**, **Fixed-Reward-PPO + PID**, and **LLM-Reward-PPO + PID**. Both residual PPO policies used a **9→64→64→2 actor** and **9→64→64→1 critic**; Ollama `llama3.2:3b` completed five reward-design iterations (**`experiment_comparison/fixed_reward_training.json`**, **`experiment_comparison/reward_design_log.json`**). Evaluation used labels **0–14**, alternating fixed **±10° pitch**, zero pitch rate, zero wheel position, and identical ten-second episodes on **`real_robot.xml`**. The original **tzejun run-7 standalone PPO** was evaluated as an additional reference and was not mislabeled as a PID residual (**`experiment_comparison/protocol.json`**).

**Libraries and Frameworks**

- **MuJoCo 3.2.3** — measured-plant physics simulation.
- **Gymnasium 1.3.0** — residual training environment API.
- **NumPy 2.5.3** — state processing and metric analysis.
- **Matplotlib 3.11.2** — comparison plot.
- **Ollama 0.35.1 / llama3.2:3b** — local PID and reward-weight proposals.
- **PyTorch 2.14.0+cpu** — neural-policy execution.
- **Stable-Baselines3 2.9.0** — PPO training and deterministic inference.

**Procedure**

1. **`compare_tzejun.py`** calibrated the PID on two pilot trajectories outside the reported labels, selecting pitch gains `6.0/0.16`, speed gains `0.7/0.08`, and yaw gain `0.35` (**`experiment_comparison/pid_calibration.json`**).
2. Ollama proposed bounded PID changes; the accepted gains were pitch `4.8/0.192`, speed `0.84/0.096`, and yaw `0.32` (**`experiment_comparison/llm_pid_decision.json`**).
3. Fixed-Reward-PPO trained for **200,000 timesteps**, seed `0`, using `-theta² - 0.1*theta_dot² - 0.001*torque²`; training took **2173.15 s** (**`experiment_comparison/fixed_reward_training.json`**).
4. LLM-Reward-PPO ran **5 iterations × 200,000 timesteps**, seeds `0–4`; PPO training took **3037.91 s** total. Invalid weight sums were repaired and explicitly normalized, with every raw response and accepted weight vector retained (**`experiment_comparison/reward_design_log.json`**).
5. **`compare_tzejun.py`** evaluated the four requested controllers plus the original tzejun checkpoint over 15 matched labels and saved the raw episodes, summary, log, and plot (**`experiment_comparison/comparison_raw.json`**, **`experiment_comparison/comparison_summary.json`**, **`experiment_comparison/run.log`**, **`experiment_comparison/comparison.png`**).

**Observations**

Values are **mean ± 95% CI** across the 15 reported labels (**`experiment_comparison/comparison_summary.json`**).

| Condition | Settling (s) | Overshoot (°) | RMS theta (°) | Mean \|torque\| (N·m) | Torque var | Success | Survival |
|---|---:|---:|---:|---:|---:|---:|---:|
| **PID-only** | **0.0750±0.0000** | **0.0000±0.0000** | 0.0512±0.0000 | 0.26642±0.00000 | 0.071211±0.000000 | **1.000±0.000** | **1.000±0.000** |
| LLM-PID | 0.0800±0.0000 | **0.0000±0.0000** | 0.0882±0.0000 | 0.26687±0.00000 | 0.071428±0.000000 | **1.000±0.000** | **1.000±0.000** |
| Fixed-Reward-PPO + PID | 0.3440±0.0862 | **0.0000±0.0000** | **0.0398±0.0026** | **0.18627±0.00524** | **0.049360±0.001782** | **1.000±0.000** | **1.000±0.000** |
| LLM-Reward-PPO + PID | 0.1097±0.0026 | **0.0000±0.0000** | 0.0507±0.0006 | 0.26325±0.00006 | 0.070735±0.000024 | **1.000±0.000** | **1.000±0.000** |
| tzejun standalone PPO reference | 0.4253±0.0170 | **0.0000±0.0000** | 0.3017±0.0006 | 0.28302±0.00003 | 0.080259±0.000012 | **1.000±0.000** | **1.000±0.000** |

**PID-only was the fastest-settling controller.** LLM-Reward-PPO settled **46.2% slower** than PID-only and reduced RMS pitch by only **1.0%**. Fixed-Reward-PPO achieved the lowest RMS pitch and torque but drifted **−2.904±0.187 m** with a zero speed command; LLM-Reward-PPO developed a tail yaw rate of **−0.822±0.000 rad/s** with zero yaw command (**`experiment_comparison/comparison_summary.json`**). Those behaviours expose a weakness in the stated success criterion, which does not reject position or yaw drift.

**Conclusions**

The corrected result does **not** support the hypothesis that LLM-designed reward produced faster PID convergence: **PID-only settled fastest at 0.075 s**, versus **0.110 s** for LLM-Reward-PPO. LLM-Reward-PPO beat Fixed-Reward-PPO on settling time but not RMS error or torque, and its uncommanded yaw means the LLM contribution was not practically beneficial. Fixed-Reward-PPO improved pitch RMS and torque, but its large backward drift makes it unsuitable for station keeping. For this simulation and protocol, **PID-only is the practical recommendation** until position and yaw penalties are made acceptance-critical.

**Limitations**

- The 15 labels repeat only two deterministic initial states, `+10°` and `−10°`; they are not 15 independent randomized physical trials, so near-zero confidence intervals must not be interpreted as population uncertainty (**`experiment_comparison/comparison_raw.json`**).
- The success definition checks settling, pitch overshoot, pitch RMS, and survival but omits horizontal drift and yaw rate; therefore all conditions report 100% success despite the PPO failure modes (**`compare_tzejun.py`**, **`experiment_comparison/comparison_summary.json`**).
- Ollama's repaired proposals still missed the exact sum constraint and were explicitly normalized; the original and repaired text is retained for audit (**`experiment_comparison/reward_design_log.json`**).
- The original tzejun checkpoint is a **34→64→64→2 standalone PPO PWM controller**, not an LLM-reward or PID-residual policy, so it is included only as a reference (**`experiment_tzejun/summary.json`**).
- No sim-to-real hardware validation was performed.

**Running Instructions**

Open PowerShell. Everything stays inside this one Desktop folder:

```powershell
$projectDir = "$env:USERPROFILE\Desktop\sridevi-DIP-recess-week"
powershell -ExecutionPolicy Bypass -File "$projectDir\run_tzejun_experiment.ps1" -Check
powershell -ExecutionPolicy Bypass -File "$projectDir\run_tzejun_experiment.ps1" -Compare
```

The first command creates/checks the local **`.venv`**, loads the checkpoint, and executes one MuJoCo control step. The second command retrains Fixed-Reward-PPO for 200,000 steps, runs five 200,000-step LLM reward iterations through local Ollama, evaluates all comparison arms, and writes **`experiment_comparison/`**. Allow roughly **1–2 hours** on this PC. Start Ollama first if needed:

```powershell
ollama serve
ollama pull llama3.2:3b
```

Run only the original tzejun checkpoint evaluation:

```powershell
powershell -ExecutionPolicy Bypass -File "$projectDir\run_tzejun_experiment.ps1"
```

Open the original tzejun trained model in MuJoCo:

```powershell
powershell -ExecutionPolicy Bypass -File "$projectDir\run_tzejun_experiment.ps1" -View
```

Click the MuJoCo window. Use **Up/Down** for forward/reverse, **Left/Right** for yaw, **Space** to stop, **R** to reset, and **Esc** to quit.
