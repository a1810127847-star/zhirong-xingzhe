@echo off
where pyw.exe >nul 2>&1
if %errorlevel% equ 0 (
  start "" pyw.exe -3 "%~dp0ros2_ws\tools\acceptance_panel.py"
  exit /b 0
)

where pythonw.exe >nul 2>&1
if %errorlevel% equ 0 (
  start "" pythonw.exe "%~dp0ros2_ws\tools\acceptance_panel.py"
  exit /b 0
)

echo [ERROR] Windows Python 3 was not found. Install Python 3 with tkinter first.
pause
exit /b 1
