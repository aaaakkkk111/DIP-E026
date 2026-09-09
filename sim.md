# Balance Bot Lab: run guide

## Prerequisites

- Windows with Python 3.10+ installed and available as `python` (or use `py`).
- Optional for PPO model loading: a Stable-Baselines3-compatible PPO `.zip` model.
- Optional for reward design: [Ollama](https://ollama.com/) installed locally. No cloud API is used.

## First-time setup

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
# Install this only when using a PPO .zip model:
python -m pip install -e ".[ppo]"
```

If `python` is not found, install Python 3.10+ and reopen PowerShell, or substitute `py -3` for `python` in the commands above.

## Validate and launch

```powershell
.\.venv\Scripts\Activate.ps1
python -m pytest -q
python -m app.main --headless-smoke
python -m app.main
```

To launch a selected local PPO model path without editing source or JSON:

```powershell
python -m app.main --controller ppo --model-path "C:\path\to\your_policy.zip"
```

The GUI shows the rotatable robot view, telemetry, PID P/I/D/reward traces, custom simulation reset state, disturbance controls, PPO model picker, and local Ollama reward-design panel.

## Local Ollama reward design

Install and start Ollama, then in a new PowerShell window run:

```powershell
ollama pull llama3.2
ollama run llama3.2 ""
```

In a second PowerShell window, verify the API with this PowerShell-safe request. `-UseBasicParsing` prevents the legacy web-content parsing warning, and the here-string avoids JSON quoting mistakes:

```powershell
$body = @'
{"model":"llama3.2","prompt":"Return a JSON object with survival equal to 1.","format":"json","stream":false}
'@
Invoke-WebRequest -UseBasicParsing -Method POST -Uri "http://localhost:11434/api/generate" -ContentType "application/json" -Body $body
```

Expected result: HTTP 200 and JSON containing a `response` field. In the app, retain `http://localhost:11434/api/generate` in the Local API endpoint field and select `llama3.2`.

If the request returns HTTP 404, another program is serving port 11434 or Ollama is not running. Diagnose it with:

```powershell
Get-NetTCPConnection -LocalPort 11434 | Select-Object OwningProcess
Get-Process -Id <PID>
```

## Create a PPO model in simulation

Install optional PPO dependencies, then train only in the local simulation:

```powershell
python -m pip install -e ".[ppo]"
python -m app.training.ppo_training --output "models/ppo_baseline.zip" --timesteps 10000 --seed 7
```

Select the resulting `.zip` using **Browse .zip PPO model** in the dashboard. A simulation-trained model is not automatically a physically validated artifact.

For a checkpointable long training job, use a larger timestep budget and retain checkpoints:

```powershell
python -m app.training.ppo_training --output "models/ppo_baseline.zip" --reward "configs/reward_baseline.json" --timesteps 1000000 --checkpoint-steps 50000 --seed 7
```

Resume from a checkpoint when interrupted:

```powershell
python -m app.training.ppo_training --output "models/ppo_baseline_resumed.zip" --resume "models/checkpoints/ppo_baseline_500000_steps.zip" --reward "configs/reward_baseline.json" --timesteps 500000 --checkpoint-steps 50000 --seed 7
```

Run a checkpointed large simulation evaluation (it never uses hardware):

```powershell
python -m app.experiments.batch_simulation --controller ppo --model-path "models/ppo_baseline.zip" --trials 1000 --iterations 10 --timesteps 1000 --randomized --allow-large --results "results/ppo_baseline_large_eval.json"
```

## Safety and hardware

The simulation starts normally. Hardware mode is intentionally disarmed and blocks motor commands until a separately verified Yahboom STM32 transport is supplied. The app has no invented serial protocol. Custom pose reset is only supported for simulation/mock backends.

## Results

The application writes telemetry, experiment checkpoints, CSV/JSONL logs, and SQLite metadata below `results/`.

## Physical-validation planning (no motor actuation)

```powershell
python -m app.experiments.physical_validation --plan
```

This produces the frozen randomized order for PID, baseline-reward PPO, and LLM-reward PPO across nominal, mild, and moderate conditions. The runner intentionally refuses physical execution until the hardware protocol/configuration and explicit PPO artifact manifests are independently verified. See `results/<experiment-id>/report.md` after using the post-trial recorder and running `--report`.

After a physically supervised trial creates raw JSONL telemetry, record the next frozen trial without changing configuration:

```powershell
python -m app.experiments.physical_validation --record "results/physical-validation-development/raw/TRIAL.jsonl" --battery 7.6 --operator "OPERATOR_ID"
python -m app.experiments.physical_validation --report
```

Follow `PHYSICAL_VALIDATION_CHECKLIST.md` before any hardware attempt.
