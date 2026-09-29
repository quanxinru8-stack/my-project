@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_and_log.ps1" %*
exit /b %ERRORLEVEL%
