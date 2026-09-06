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

## Safety and hardware

The simulation starts normally. Hardware mode is intentionally disarmed and blocks motor commands until a separately verified Yahboom STM32 transport is supplied. The app has no invented serial protocol. Custom pose reset is only supported for simulation/mock backends.

## Results

The application writes telemetry, experiment checkpoints, CSV/JSONL logs, and SQLite metadata below `results/`.
