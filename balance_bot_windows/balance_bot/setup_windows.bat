@echo off
REM Windows setup, cmd.exe version.  Double-click or run: setup_windows.bat
cd /d "%~dp0"
where py >nul 2>nul && (set PY=py) || (set PY=python)
%PY% --version || (echo 找不到 Python，先从 python.org 装 3.10-3.12 并勾选 Add to PATH & pause & exit /b 1)
if not exist .venv %PY% -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip wheel
.venv\Scripts\python.exe -m pip install "numpy>=1.24" "mujoco>=3.1" "PyQt6>=6.4" "scipy>=1.10"

rem 训练依赖是可选的（约 800 MB），要练 PPO 就跑：setup_windows.bat --train
if /i "%~1"=="--train" (
  echo.
  echo === 装训练依赖 torch / gymnasium / stable-baselines3 ===
  .venv\Scripts\python.exe -m pip install "gymnasium>=0.29" "stable-baselines3>=2.3" torch tensorboard
)
echo.
echo === 自检 ===
.venv\Scripts\python.exe tests\test_core.py
.venv\Scripts\python.exe tests\test_twin_baseline.py
echo.
echo 装好了。下次用： .venv\Scripts\activate.bat
pause
