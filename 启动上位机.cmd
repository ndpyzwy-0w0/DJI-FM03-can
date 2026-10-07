@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" (
  echo Python environment missing. Run setup from README first.
  pause
  exit /b 1
)
powershell -NoProfile -Command "Start-Process -FilePath '.\.venv\Scripts\pythonw.exe' -ArgumentList 'tools\gui_server.py','--open' -WorkingDirectory (Get-Location).Path -WindowStyle Hidden"
