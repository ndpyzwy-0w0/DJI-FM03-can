@echo off
cd /d "%~dp0"
if exist "dist\FM03_MotorControl_StrictPosition.exe" (
  start "" "dist\FM03_MotorControl_StrictPosition.exe"
  exit /b 0
)
if exist "dist\FM03_MotorControl_PositionOptions.exe" (
  start "" "dist\FM03_MotorControl_PositionOptions.exe"
  exit /b 0
)
if exist "dist\FM03_MotorControl_Position.exe" (
  start "" "dist\FM03_MotorControl_Position.exe"
  exit /b 0
)
if exist "dist\FM03_MotorControl_Calibration.exe" (
  start "" "dist\FM03_MotorControl_Calibration.exe"
  exit /b 0
)
if exist "dist\FM03_MotorControl_Status.exe" (
  start "" "dist\FM03_MotorControl_Status.exe"
  exit /b 0
)
powershell -NoProfile -Command "Start-Process -FilePath '.\.venv\Scripts\pythonw.exe' -ArgumentList 'tools\motor_control.py' -WorkingDirectory (Get-Location).Path -WindowStyle Hidden"
