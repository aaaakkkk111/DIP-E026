@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_setup_only.ps1"
if errorlevel 1 (
  pause
  exit /b 1
)
"%~dp0.venv\Scripts\python.exe" "%~dp0play.py" --episodes 5 --seed 25 --reset-noise 0.08 --demo-push
if errorlevel 1 pause
