@echo off
for %%I in ("%~dp0.") do set "ROOT=%%~fI"
if not defined JAVA_HOME set "JAVA_HOME=D:\DIP\Simulation\.jdk17\jdk-17.0.20.1+1"
"D:\DIP\Simulation\.gradle-portable\gradle-8.10.2\bin\gradle.bat" -p "%ROOT%" %*
