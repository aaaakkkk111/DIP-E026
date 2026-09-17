# Windows setup for the STM32 balance-car twin.  No WSL, no ROS.
#   Right-click -> Run with PowerShell, or:
#     powershell -ExecutionPolicy Bypass -File setup_windows.ps1
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$py = "py"
if (-not (Get-Command $py -ErrorAction SilentlyContinue)) { $py = "python" }
& $py --version
if ($LASTEXITCODE -ne 0) {
  Write-Host "找不到 Python。先从 python.org 装 3.10-3.12，安装时勾选 Add to PATH。" -ForegroundColor Red
  exit 1
}

if (-not (Test-Path ".venv")) {
  Write-Host ">> 建虚拟环境 .venv"
  & $py -m venv .venv
}
& .\.venv\Scripts\python.exe -m pip install --upgrade pip wheel | Out-Null

Write-Host ">> 装依赖（numpy / mujoco / PyQt6）"
& .\.venv\Scripts\python.exe -m pip install "numpy>=1.24" "mujoco>=3.1" "PyQt6>=6.4" "scipy>=1.10"

if ($args -contains "--train") {
  Write-Host ">> 装训练依赖（torch / gymnasium / stable-baselines3，约 800 MB）"
  & .\.venv\Scripts\python.exe -m pip install "gymnasium>=0.29" "stable-baselines3>=2.3" torch tensorboard
}

Write-Host "`n>> 自检"
& .\.venv\Scripts\python.exe tests\test_core.py
& .\.venv\Scripts\python.exe tests\test_twin_baseline.py

Write-Host @"

装好了。下次开 PowerShell 先激活：
    .\.venv\Scripts\Activate.ps1

然后（或者直接双击上一层目录里的 run_gui.bat）：
    python scripts\sim_gui.py                                  # 交互控制台
    python scripts\bench_twin_baseline.py --backend mujoco --episodes 40 # baseline
    python scripts\tune_gui.py                                 # 滑条调参
"@ -ForegroundColor Green
