@echo off
REM ============================================================================
REM Argo Shore Station Launcher for Windows
REM ============================================================================
REM 
REM This batch file launches the Argo shore station components via WSL2:
REM   1. LoRa shore receiver (lora_shore.py)
REM   2. Web dashboard (argo_web_dashboard.py)
REM 
REM REQUIREMENTS:
REM   - WSL2 with Ubuntu 22.04 installed
REM   - ROS2 Humble installed in WSL2
REM   - Argo files copied to ~/argo/ in WSL2
REM   - USB LoRa device attached to WSL2 (via usbipd)
REM 
REM See README_WINDOWS_SETUP.md for complete setup instructions.
REM ============================================================================

echo.
echo ========================================
echo Argo Shore Station Launcher
echo ========================================
echo.

REM Check if WSL2 is installed
wsl --status >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: WSL2 is not installed!
    echo.
    echo Please install WSL2 first:
    echo   wsl --install -d Ubuntu-22.04
    echo.
    echo See README_WINDOWS_SETUP.md for instructions.
    pause
    exit /b 1
)

echo [1/4] Checking WSL2 status...
wsl --list --verbose
echo.

echo [2/4] Checking USB LoRa device attachment...
echo.
echo If your LoRa device is not attached to WSL2, run this in PowerShell (Administrator):
echo   usbipd list
echo   usbipd attach --wsl --busid YOUR_BUSID
echo.
echo Press any key to continue (Ctrl+C to cancel)...
pause >nul

echo.
echo [3/4] Starting ROS2 environment in WSL2...
echo.

REM Launch WSL2 with a new Windows Terminal window (if available)
REM This keeps the terminal open so you can see logs

REM Try to use Windows Terminal if available
where wt >nul 2>&1
if %errorlevel% equ 0 (
    REM Windows Terminal is available - use split panes
    echo Using Windows Terminal with split panes...
    echo.
    echo TIP: You can resize the panes by dragging the divider
    echo.
    
    REM Launch with split panes:
    REM - Left pane: LoRa shore receiver
    REM - Right pane: Web dashboard
    wt --title "Argo Shore Station" ^
       wsl bash -c "source /opt/ros/humble/setup.bash && echo '=== LoRa Shore Receiver ===' && echo 'Port: /dev/ttyUSB0 (or /dev/ttyACM0)' && echo '' && cd ~/argo && python3 shore/lora_shore.py --port /dev/ttyUSB0 ; read -p 'Press Enter to close...'" ^
       ; split-pane --horizontal ^
       wsl bash -c "source /opt/ros/humble/setup.bash && echo '=== Web Dashboard ===' && echo 'Starting in 5 seconds...' && sleep 5 && cd ~/argo && python3 nodes/argo_web_dashboard.py ; read -p 'Press Enter to close...'"
) else (
    REM Windows Terminal not available - use separate windows
    echo Windows Terminal not found, using separate windows...
    echo.
    echo You will see two separate terminal windows:
    echo   1. LoRa Shore Receiver
    echo   2. Web Dashboard
    echo.
    
    REM Launch LoRa receiver in new window
    start "Argo LoRa Shore Receiver" wsl bash -c "source /opt/ros/humble/setup.bash && echo '=== LoRa Shore Receiver ===' && echo 'Port: /dev/ttyUSB0 (or /dev/ttyACM0)' && echo '' && cd ~/argo && python3 shore/lora_shore.py --port /dev/ttyUSB0 ; read -p 'Press Enter to close...'"
    
    REM Wait a moment for ROS2 to initialize
    timeout /t 3 /nobreak >nul
    
    REM Launch web dashboard in new window
    start "Argo Web Dashboard" wsl bash -c "source /opt/ros/humble/setup.bash && echo '=== Web Dashboard ===' && echo 'Starting in 5 seconds...' && sleep 5 && cd ~/argo && python3 nodes/argo_web_dashboard.py ; read -p 'Press Enter to close...'"
)

echo.
echo [4/4] Shore station is starting...
echo.
echo ========================================
echo Shore Station Access
echo ========================================
echo.
echo Web Dashboard URL:
echo   http://localhost:8081
echo.
echo To find your computer's IP for remote access:
echo   1. Open WSL2: wsl
echo   2. Run: hostname -I
echo   3. Access from phone: http://YOUR_IP:8081
echo.
echo ========================================
echo Controls
echo ========================================
echo.
echo To stop the shore station:
echo   - Close the terminal windows, or
echo   - Press Ctrl+C in each window
echo.
echo ========================================
echo.

REM Keep this window open for reference
echo This window can be closed. The shore station will continue running.
echo.
pause


