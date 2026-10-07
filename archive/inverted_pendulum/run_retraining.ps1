param([switch]$NoPause)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

& (Join-Path $PSScriptRoot "run_setup_only.ps1")
$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

Write-Host "Starting the full 200,000-step PPO training run..."
& $venvPython train_ppo.py --timesteps 200000 --seed 42
& $venvPython evaluate_model.py --episodes 20 --seed 100 --expected-reward 1000
& $venvPython record_gif.py --no-open

Write-Host ""
Write-Host "Retraining and evaluation completed."
if (-not $NoPause) {
    Read-Host "Press Enter to close"
}
