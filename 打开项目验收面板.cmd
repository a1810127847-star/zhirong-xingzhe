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

echo [ERROR] Windows Python 3 was not found.
echo 首次部署请双击“一键部署并打开验收.cmd”，它会自动安装所需环境。
pause
exit /b 1
