# MuJoCo Inverted Pendulum PPO Experiment

This package reproduces the MuJoCo **InvertedPendulum-v5** experiment with a
trained Proximal Policy Optimization (PPO) controller. A cart moves horizontally
and must keep a hinged pole upright by applying a continuous force.

The package includes the source code, pinned dependencies, the trained model,
reference results, a rendered GIF, and one-click Windows launchers.

## Quick start on Windows

### Prerequisite

Install 64-bit Python 3.12 from the
[official Python website](https://www.python.org/downloads/windows/). During
installation, enable **Add python.exe to PATH**.

### Reproduce the included result

Double-click:

```text
RUN_EXPERIMENT.bat
```

The launcher will:

1. create an isolated `.venv` environment;
2. install the complete dependency lock in `requirements-lock.txt`;
3. verify Gymnasium and MuJoCo;
4. run a ten-episode random-controller baseline;
5. evaluate the included PPO model for twenty episodes;
6. require all twenty episodes to score 1,000;
7. write `results/replication_results.json`;
8. generate `artifacts/inverted_pendulum_trained.gif`.

The first run downloads PyTorch and may take several minutes. Later runs reuse
the local `.venv` and finish much faster.

### Watch the live controller

After `RUN_EXPERIMENT.bat` has completed, double-click:

```text
PLAY_LIVE_DEMO.bat
```

The live demonstration starts with a larger initial tilt and applies brief
disturbances near steps 200 and 500 so the recovery motion is easy to see.

### Retrain from scratch

Double-click:

```text
RETRAIN_FROM_SCRATCH.bat
```

This repeats the 200,000-step PPO training run with seed 42, saves a new model,
and evaluates it. Training took about five minutes on the original Windows CPU.

The included pretrained model is the reliable reproduction path. A fresh
reinforcement-learning run can differ slightly across CPU architectures and
library runtimes even when the random seed and package versions are fixed.

## Reference result

The original full run used:

- environment: `InvertedPendulum-v5`;
- algorithm: PPO;
- training timesteps: 200,000;
- random seed: 42;
- completed training episodes: 1,087;
- final ten-episode evaluation: `1000.0 ± 0.0`;
- additional twenty-episode evaluation: twenty scores of `1000`.

The original optimization run recorded MuJoCo 3.11.0 in
`models/training_summary.json`. The transferable package pins MuJoCo 3.3.7 to
retain Gymnasium 1.3.0 live-viewer compatibility. The included policy was
re-evaluated under 3.3.7 and again scored 1,000 in all twenty episodes.

The random-action baseline survived only 4–10 steps in its ten seeded episodes.
During training, the first scheduled evaluation at 20,000 steps averaged 121.2.
At 40,000 steps, all five evaluation episodes reached the maximum score of
1,000. Every later scheduled evaluation through 200,000 steps also scored 1,000.

## Environment definition

The observation contains four numerical values:

| Index | Meaning | Unit |
| ---: | --- | --- |
| 0 | cart position along the rail | metres |
| 1 | pole angle relative to upright | radians |
| 2 | cart linear velocity | metres/second |
| 3 | pole angular velocity | radians/second |

The action contains one continuous force in `[-3, 3]` newtons. The pole is
perfectly upright at `0` radians. An episode terminates when the absolute pole
angle reaches approximately `0.2` radians (`11.46°`) or the state becomes
non-finite. It is truncated after 1,000 steps.

The environment awards `+1` for every healthy step. Therefore, an episode that
keeps the pole upright for all 1,000 steps receives the maximum reward of 1,000.

## What PPO learns

PPO does not tune explicit PID gains such as `Kp`, `Ki`, and `Kd`. It trains an
actor-critic neural network through backpropagation. The saved policy contains
9,091 trainable parameters. Both the policy and value networks use two hidden
layers of 64 `tanh` units.

The main training settings are:

```python
learning_rate = 3e-4
n_steps = 2048
batch_size = 64
gamma = 0.99
```

At each step, the policy receives the four observations and produces one force.
MuJoCo advances the physics simulation and returns the next observation and
reward. PPO collects rollouts, estimates which actions performed better than
expected, computes its clipped policy and value losses, and PyTorch updates the
network parameters with backpropagation and the Adam optimizer.

## Package contents

| Path | Purpose |
| --- | --- |
| `RUN_EXPERIMENT.bat` | one-click dependency setup and result reproduction |
| `PLAY_LIVE_DEMO.bat` | one-click live MuJoCo demonstration |
| `RETRAIN_FROM_SCRATCH.bat` | one-click 200,000-step retraining |
| `check_env.py` | package and environment smoke test |
| `random_agent.py` | random-action baseline |
| `evaluate_model.py` | deterministic evaluation and JSON result writer |
| `train_ppo.py` | PPO training program |
| `play.py` | live or headless model playback |
| `record_gif.py` | headless GIF renderer |
| `models/ppo_inverted_pendulum.zip` | included trained PPO model |
| `models/training_summary.json` | original training metadata |
| `results/reference_results.json` | expected evaluation result |
| `artifacts/inverted_pendulum_trained.gif` | reference animation |
| `requirements.txt` | pinned direct dependencies |
| `requirements-lock.txt` | complete verified dependency graph |

## Manual commands

Create and activate a virtual environment:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-lock.txt
```

Verify the installation:

```powershell
python check_env.py
```

Run the random baseline:

```powershell
python random_agent.py --no-render --episodes 10 --seed 42
```

Evaluate the included model and require twenty perfect episodes:

```powershell
python evaluate_model.py --episodes 20 --expected-reward 1000
```

Play the visibly disturbed live demonstration:

```powershell
python play.py --episodes 5 --seed 25 --reset-noise 0.08 --demo-push
```

Render a GIF without opening a window:

```powershell
python record_gif.py --no-open
```

Retrain:

```powershell
python train_ppo.py --timesteps 200000 --seed 42
```

## Software versions

- Python 3.12
- Gymnasium 1.3.0
- MuJoCo 3.3.7
- Stable-Baselines3 2.9.0
- PyTorch 2.13.0

MuJoCo 3.3.7 is intentionally pinned because Gymnasium 1.3.0's live viewer
still uses the 3.3-series camera function signature. Newer MuJoCo releases
changed that signature and can raise a `mjv_moveCamera` error during mouse
camera events.

## Troubleshooting

### Python is not found

Install Python 3.12 and enable **Add python.exe to PATH**, then reopen the
terminal or run the batch file again.

### PowerShell script execution is blocked

The provided `.bat` launchers invoke PowerShell with a process-scoped bypass.
No permanent execution-policy change is made.

### The live window does not open

Run the headless evaluator first:

```powershell
.\.venv\Scripts\python.exe evaluate_model.py --episodes 20
```

If headless evaluation succeeds, the model and physics environment are working;
the remaining issue is normally the OpenGL driver, Remote Desktop, or a system
without a graphical desktop. Use the included GIF on such systems.

### The pole appears almost motionless

That is expected for a successful balancing policy. Use `PLAY_LIVE_DEMO.bat`,
which starts from a larger tilt and adds two short disturbances.

## References

- [Original 2022 Gym Inverted Pendulum page](https://mgoulao.github.io/gym-docs/environments/mujoco/inverted_pendulum/)
- [Current Gymnasium Inverted Pendulum documentation](https://gymnasium.farama.org/environments/mujoco/inverted_pendulum/)
- [Gymnasium MuJoCo documentation](https://gymnasium.farama.org/environments/mujoco/)
- [Stable-Baselines3 documentation](https://stable-baselines3.readthedocs.io/en/master/)
