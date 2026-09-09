# Balance Bot Lab

A local desktop laboratory for a Yahboom-style STM32 two-wheel self-balancing robot. It models a coupled two-wheel inverted pendulum (not CartPole), isolates unverified hardware protocol details, and runs every command through a safety layer.

## Setup (one virtual environment)

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e .[dev]
# Optional: PPO ZIP models made by Stable-Baselines3
python -m pip install -e .[ppo]
```

## Run

```powershell
python -m app.main
# or: balance-bot-lab
```

The dashboard is Tkinter (bundled with normal Python on Windows). Click **Start**, select PID/LQR/PPO, and use the disturbance controls while running. Hardware is deliberately disarmed at startup. Configure `controller.model_path` in `configs/default.json` to load an SB3 PPO `.zip`; a missing optional dependency/model becomes a visible safe error, not a motor command.

The dashboard includes an in-process live pitch line analyzer; saved `results/*.jsonl` files are also consumable through `LineAnalyzer.load_jsonl`. Experiment modes (`development`, `validation`, and `research`) are controlled by `ExperimentConfig`; its guards permit up to 1,000 trials × 10 iterations but defaults stay deliberately small.

The live analyzer draws PID P/I/D traces and reward together. The dashboard’s reset panel accepts simulation position, pitch, velocity, and pitch-rate. Its Pipeline B panel loads any Stable-Baselines3 PPO `.zip`, accepts a natural-language prompt, sends it to local Ollama only, displays the proposal, validates it as a bounded structured reward, then previews its reward trace using the selected policy. It never runs generated code or permits Ollama to send commands.

The orbitable 3D canvas depicts the manufacturer-described metal chassis, paired driven wheels, upright STM32/6-axis-IMU electronics stack, and expansion deck. Its proportions are visual placeholders: product pages do not publish authoritative CAD/dimension data, so they are not represented as measured facts.

## Test

```powershell
python -m pytest -q
python -m app.main --headless-smoke
```

## Scope and safety

`HardwareBackend` intentionally has no invented STM32 serial protocol. It is a configuration-gated adapter with an explicit `send_verified` hook; use `MockHardwareBackend` for end-to-end tests until the exact vendor protocol is independently verified. Ollama is local-only (`http://127.0.0.1:11434`) and can only return validated candidate configurations. It never receives a motor interface.

All persisted runs, SQLite metadata, JSONL telemetry, and checkpoints reside under `results/`.

## Physical validation protocol

The configuration-driven, non-actuating physical-validation experiment lives in `configs/physical_validation_development.json`. It schedules five randomized development trials per controller/condition, freezes a config fingerprint, requires raw telemetry and battery range checks, and checkpoints after each recorded trial. It refuses to arm or validate hardware until `configs/hardware_unverified.json` is replaced from verified vendor documentation and bench checks. Plan it with `python -m app.experiments.physical_validation --plan`; generate a report only after records exist with `python -m app.experiments.physical_validation --report`.

Copy `models/ppo_artifact.manifest.example.json` to each selected PPO manifest and populate it with the exact model hash and validated observation/action compatibility before any physical protocol is allowed to progress.

Create a simulation-only baseline PPO model with `python -m app.training.ppo_training --output models/ppo_baseline.zip --timesteps 10000`. This model is not automatically validated for hardware use.

For checkpointed PPO training/resume and large simulation evaluation, see `python -m app.training.ppo_training --help` and `python -m app.experiments.batch_simulation --help`.
