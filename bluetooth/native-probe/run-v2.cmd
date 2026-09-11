@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist BleScan.exe call build.cmd
if not exist BleScan.exe exit /b 1
del /q last-address.txt 2>nul
BleScan.exe
if errorlevel 1 goto done
set /p BLE_ADDRESS=<last-address.txt
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0BleProbeV2.ps1" -Address "%BLE_ADDRESS%"
:done
echo.
echo Exit code: %ERRORLEVEL%
pause
