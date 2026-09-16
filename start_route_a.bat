@echo off
setlocal
set "ROUTE_A_PYTHON=C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe"
if not exist "%ROUTE_A_PYTHON%" (
  echo Python not found: %ROUTE_A_PYTHON%
  exit /b 1
)
cd /d "%~dp0"
"%ROUTE_A_PYTHON%" -m route_a.main %*
