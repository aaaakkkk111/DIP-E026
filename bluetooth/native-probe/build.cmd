@echo off
setlocal
chcp 65001 >nul
set "CSC=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
set "FX=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319"
set "WINMD=%WINDIR%\Lenovo\ImController\PluginHost\Windows.winmd"

"%CSC%" /nologo /target:exe /out:BleProbe.exe ^
  /reference:"%FX%\System.Runtime.dll" ^
  /reference:"%FX%\System.Runtime.WindowsRuntime.dll" ^
  /reference:"%FX%\System.Runtime.InteropServices.WindowsRuntime.dll" ^
  /reference:"%WINMD%" ^
  BleProbe.cs

if errorlevel 1 (
  echo Build failed.
  pause
  exit /b 1
)
"%CSC%" /nologo /target:exe /out:BleScan.exe ^
  /reference:"%FX%\System.Runtime.dll" ^
  /reference:"%FX%\System.Runtime.WindowsRuntime.dll" ^
  /reference:"%FX%\System.Runtime.InteropServices.WindowsRuntime.dll" ^
  /reference:"%WINMD%" ^
  BleScan.cs

if errorlevel 1 (
  echo Scanner build failed.
  pause
  exit /b 1
)
echo Build succeeded: %CD%\BleProbe.exe and BleScan.exe
pause
