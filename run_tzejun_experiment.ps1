param(
    [switch]$View,
    [switch]$Check,
    [switch]$Compare,
    [ValidateRange(1, 1000)][int]$Seeds = 15
)

# Resolve every path from this script, so calling it from any directory works.
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvDir = Join-Path $projectDir '.venv'
$pythonExe = Join-Path $venvDir 'Scripts\python.exe'
$experimentDir = Join-Path $projectDir 'experiment_tzejun'
$entrypoint = Join-Path $projectDir 'tzejun_experiment.py'
$comparisonEntrypoint = Join-Path $projectDir 'compare_tzejun.py'
$requirements = Join-Path $projectDir 'requirements.txt'
New-Item -ItemType Directory -Force -Path $experimentDir | Out-Null

# Remove a PYTHONPATH left over from earlier manual instructions. In
# particular, the old .deps directory has unreadable files on this PC.
Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue

if (-not (Test-Path $pythonExe)) {
    if (-not (Get-Command py -ErrorAction SilentlyContinue)) {
        throw "Python launcher 'py' is required to create the project-local .venv. Install Python 3.12."
    }
    Write-Host "Creating $venvDir with Python 3.12..."
    & py -3.12 -m venv $venvDir
    if ($LASTEXITCODE -ne 0) { throw "Could not create .venv (exit code $LASTEXITCODE)." }
}

$requirementsHash = (Get-FileHash -LiteralPath $requirements -Algorithm SHA256).Hash
$installedHashFile = Join-Path $venvDir 'requirements.sha256'
$installedHash = if (Test-Path $installedHashFile) { (Get-Content $installedHashFile -Raw).Trim() } else { '' }
if ($installedHash -ne $requirementsHash) {
    Write-Host 'Installing pinned dependencies into the project-local .venv (first run may take several minutes)...'
    & $pythonExe -m pip install --disable-pip-version-check -r $requirements
    if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed (exit code $LASTEXITCODE)." }
    Set-Content -LiteralPath $installedHashFile -Value $requirementsHash -NoNewline
}

$probe = 'import torch, stable_baselines3, mujoco; print(torch.__version__, stable_baselines3.__version__, mujoco.__version__)'
Write-Host "Using Python: $pythonExe"
Write-Host "Project: $projectDir"
$previousPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
& $pythonExe -c $probe
$probeExit = $LASTEXITCODE
$ErrorActionPreference = $previousPreference
if ($probeExit -ne 0) {
    throw "The project-local Python cannot import its dependencies (exit code $probeExit). The complete traceback is above."
}

if ($Check) {
    $ErrorActionPreference = 'Continue'
    & $pythonExe $entrypoint --check
    $runExit = $LASTEXITCODE
    $ErrorActionPreference = $previousPreference
    if ($runExit -ne 0) { throw "Checkpoint and MuJoCo check exited with code $runExit." }
} elseif ($View) {
    $ErrorActionPreference = 'Continue'
    & $pythonExe $entrypoint --view
    $runExit = $LASTEXITCODE
    $ErrorActionPreference = $previousPreference
    if ($runExit -ne 0) { throw "Viewer exited with code $runExit." }
} elseif ($Compare) {
    $comparisonDir = Join-Path $projectDir 'experiment_comparison'
    New-Item -ItemType Directory -Force -Path $comparisonDir | Out-Null
    $logPath = Join-Path $comparisonDir 'run.log'
    $ErrorActionPreference = 'Continue'
    & $pythonExe $comparisonEntrypoint --timesteps 200000 --iterations 5 --seeds $Seeds 2>&1 | Tee-Object -FilePath $logPath
    $runExit = $LASTEXITCODE
    $ErrorActionPreference = $previousPreference
    if ($runExit -ne 0) {
        Write-Host "Full Python output: $logPath"
        throw "Comparison exited with code $runExit. See the full traceback above and in the log."
    }
    Write-Host "Results: $(Join-Path $comparisonDir 'comparison_summary.json')"
} else {
    $logPath = Join-Path $experimentDir 'run.log'
    $ErrorActionPreference = 'Continue'
    & $pythonExe $entrypoint --seeds $Seeds 2>&1 | Tee-Object -FilePath $logPath
    $runExit = $LASTEXITCODE
    $ErrorActionPreference = $previousPreference
    if ($runExit -ne 0) {
        Write-Host "Full Python output: $logPath"
        throw "Experiment exited with code $runExit. See the full traceback above and in the log."
    }
    Write-Host "Results: $(Join-Path $experimentDir 'summary.json')"
}
