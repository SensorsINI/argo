@echo off
REM WSL Port Forwarding Batch Script (bypasses execution policy)
REM This script wraps the PowerShell script to avoid execution policy issues
REM Run this from Windows (not WSL)

echo Setting up WSL port forwarding for Foxglove Bridge (port 8765)...
echo.
echo NOTE: This script requires Administrator privileges.
echo Please right-click and select "Run as Administrator" if not already elevated.
echo.

REM Check if running as administrator
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo ERROR: This script must be run as Administrator!
    echo Right-click this file and select "Run as Administrator"
    pause
    exit /b 1
)

REM Get the directory where this batch file is located
set SCRIPT_DIR=%~dp0

REM Run PowerShell script with execution policy bypass
powershell.exe -ExecutionPolicy Bypass -File "%SCRIPT_DIR%wsl_port_forward.ps1"

if %errorLevel% equ 0 (
    echo.
    echo Port forwarding configured successfully!
) else (
    echo.
    echo ERROR: Port forwarding configuration failed!
)

pause

