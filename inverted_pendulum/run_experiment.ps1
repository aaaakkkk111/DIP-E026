param([switch]$NoPause)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

Write-Host "=== MuJoCo Inverted Pendulum PPO Experiment ==="

$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $venvPython)) {
    $pythonCommand = $null
    $pythonArguments = @()
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $pythonCommand = (Get-Command py).Source
        $pythonArguments = @("-3.12")
    } elseif (Get-Command python -ErrorAction SilentlyContinue) {
        $pythonCommand = (Get-Command python).Source
    } else {
        throw "Python was not found. Install 64-bit Python 3.12 and enable Add python.exe to PATH."
    }
    Write-Host "[1/5] Creating the isolated Python environment..."
    & $pythonCommand @pythonArguments -m venv .venv
} else {
    Write-Host "[1/5] Reusing the existing isolated Python environment."
}

Write-Host "[2/5] Installing pinned dependencies..."
& $venvPython -m pip install --disable-pip-version-check -r requirements-lock.txt

Write-Host "[3/5] Checking Gymnasium and MuJoCo..."
& $venvPython check_env.py

Write-Host "[4/5] Running the random-controller baseline..."
& $venvPython random_agent.py --no-render --episodes 10 --seed 42

Write-Host "[5/5] Evaluating the included trained model..."
& $venvPython evaluate_model.py --episodes 20 --seed 100 --expected-reward 1000
& $venvPython record_gif.py --no-open

Write-Host ""
Write-Host "Experiment completed successfully."
Write-Host "Results: results\replication_results.json"
Write-Host "Animation: artifacts\inverted_pendulum_trained.gif"
if (-not $NoPause) {
    Read-Host "Press Enter to close"
}
