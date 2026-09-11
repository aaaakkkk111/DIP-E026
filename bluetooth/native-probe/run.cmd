@echo off
chcp 65001 >nul
cd /d "%~dp0"
if not exist BleProbe.exe call build.cmd
if not exist BleProbe.exe exit /b 1
BleProbe.exe
