@echo off
setlocal
chcp 65001 >nul

set "BOOTSTRAP=%~dp0windows_bootstrap.ps1"
if not exist "%BOOTSTRAP%" set "BOOTSTRAP=%~dp0ros2_ws\tools\windows_bootstrap.ps1"

if not exist "%BOOTSTRAP%" (
  echo [ERROR] windows_bootstrap.ps1 was not found.
  echo Extract the complete deployment bundle before running this launcher.
  pause
  exit /b 1
)

echo Zhirong Xingzhe one-click deployment is starting.
echo Administrator approval is required. A Windows restart may be required once.
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%BOOTSTRAP%" %*
set "RESULT=%ERRORLEVEL%"

if not "%RESULT%"=="0" (
  echo.
  echo [ERROR] Deployment did not complete. Exit code: %RESULT%
  echo Keep this window or the bootstrap log and contact the project owner.
  pause
)

exit /b %RESULT%
