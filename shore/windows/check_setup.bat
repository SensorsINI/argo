@echo off
REM ============================================================================
REM Argo Shore Station Setup Checker
REM ============================================================================
REM 
REM This script verifies that your Windows system is ready to run the
REM Argo shore station. Run this after completing the setup steps.
REM ============================================================================

echo.
echo ========================================
echo Argo Shore Station Setup Checker
echo ========================================
echo.

set ALL_OK=1

REM Check 1: WSL2 Installation
echo [1/6] Checking WSL2 installation...
wsl --status >nul 2>&1
if %errorlevel% neq 0 (
    echo   ✗ WSL2 is NOT installed
    echo     Install with: wsl --install -d Ubuntu-22.04
    set ALL_OK=0
) else (
    echo   ✓ WSL2 is installed
    wsl --status
)
echo.

REM Check 2: Ubuntu Distribution
echo [2/6] Checking Ubuntu distribution...
wsl --list --verbose | findstr "Ubuntu" >nul 2>&1
if %errorlevel% neq 0 (
    echo   ✗ Ubuntu distribution not found
    echo     Install with: wsl --install -d Ubuntu-22.04
    set ALL_OK=0
) else (
    echo   ✓ Ubuntu distribution found
    wsl --list --verbose
)
echo.

REM Check 3: ROS2 Installation in WSL2
echo [3/6] Checking ROS2 installation in WSL2...
wsl bash -c "source /opt/ros/humble/setup.bash 2>/dev/null && echo OK" | findstr "OK" >nul 2>&1
if %errorlevel% neq 0 (
    echo   ✗ ROS2 Humble not found in WSL2
    echo     Follow ROS2 installation steps in README_WINDOWS_SETUP.md
    set ALL_OK=0
) else (
    echo   ✓ ROS2 Humble is installed
)
echo.

REM Check 4: Python Dependencies
echo [4/6] Checking Python dependencies in WSL2...
wsl bash -c "python3 -c 'import serial, flask, flask_cors' 2>/dev/null && echo OK" | findstr "OK" >nul 2>&1
if %errorlevel% neq 0 (
    echo   ✗ Python dependencies not installed
    echo     Install with: pip3 install pyserial flask flask-cors
    set ALL_OK=0
) else (
    echo   ✓ Python dependencies installed
)
echo.

REM Check 5: Argo Files in WSL2
echo [5/6] Checking Argo files in WSL2...
wsl bash -c "test -f ~/argo/shore/lora_shore.py && echo OK" | findstr "OK" >nul 2>&1
if %errorlevel% neq 0 (
    echo   ✗ Argo files not found in WSL2 ~/argo/ directory
    echo     Copy files to WSL2 as described in README_WINDOWS_SETUP.md
    set ALL_OK=0
) else (
    echo   ✓ Argo files found in WSL2
)
echo.

REM Check 6: USB Device Tools (usbipd)
echo [6/6] Checking USB device passthrough tools...
where usbipd >nul 2>&1
if %errorlevel% neq 0 (
    echo   ⚠ usbipd-win not found (needed for USB LoRa device)
    echo     Install with: winget install --interactive --exact dorssel.usbipd-win
    echo     This is required for LoRa communication.
    set ALL_OK=0
) else (
    echo   ✓ usbipd-win is installed
    echo.
    echo   USB devices attached to WSL2:
    usbipd list
)
echo.

REM Summary
echo ========================================
echo Summary
echo ========================================
echo.

if %ALL_OK% equ 1 (
    echo ✓✓✓ All checks passed! ✓✓✓
    echo.
    echo Your system is ready to run the Argo shore station.
    echo.
    echo Next steps:
    echo   1. Attach your LoRa USB device to WSL2:
    echo      usbipd attach --wsl --busid YOUR_BUSID
    echo.
    echo   2. Launch the shore station:
    echo      Double-click launch_argo_shore.bat
    echo.
    echo   3. Access the web dashboard:
    echo      http://localhost:8081
) else (
    echo ✗✗✗ Setup incomplete ✗✗✗
    echo.
    echo Please complete the missing steps above.
    echo See README_WINDOWS_SETUP.md for detailed instructions.
)

echo.
echo ========================================
echo.
pause


