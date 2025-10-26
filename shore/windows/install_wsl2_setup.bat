@echo off
REM ============================================================================
REM Argo Shore Station - WSL2 Setup Installer (Interactive)
REM ============================================================================
REM 
REM This script guides you through the complete WSL2 setup process.
REM Run this once to set up your Windows computer for Argo shore station.
REM 
REM Administrator privileges required for WSL2 installation.
REM ============================================================================

echo.
echo ========================================
echo Argo Shore Station Setup Wizard
echo ========================================
echo.
echo This wizard will help you set up your Windows computer to run
echo the Argo shore station (LoRa receiver + Web Dashboard).
echo.
echo Total setup time: ~15-20 minutes
echo.
echo ========================================
echo.

REM Check for Administrator privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: This script requires Administrator privileges!
    echo.
    echo Please right-click this file and select "Run as administrator"
    echo.
    pause
    exit /b 1
)

echo [✓] Running with Administrator privileges
echo.

REM ============================================================================
REM STEP 1: Check and install WSL2
REM ============================================================================

echo ========================================
echo Step 1: WSL2 Installation
echo ========================================
echo.

wsl --status >nul 2>&1
if %errorlevel% neq 0 (
    echo WSL2 is not currently installed.
    echo.
    echo Installing WSL2 with Ubuntu 22.04...
    echo This will take 5-10 minutes and require a restart.
    echo.
    
    wsl --install -d Ubuntu-22.04
    
    echo.
    echo ========================================
    echo RESTART REQUIRED
    echo ========================================
    echo.
    echo Windows will restart to complete WSL2 installation.
    echo.
    echo After restart:
    echo   1. Ubuntu will open automatically - set your username/password
    echo   2. Run this script again (as Administrator) to continue setup
    echo.
    echo Press any key to restart now...
    pause >nul
    shutdown /r /t 0
    exit /b 0
) else (
    echo [✓] WSL2 is already installed
    wsl --status
    echo.
)

REM ============================================================================
REM STEP 2: Check Ubuntu installation
REM ============================================================================

echo ========================================
echo Step 2: Ubuntu Distribution
echo ========================================
echo.

wsl --list --verbose | findstr "Ubuntu" >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing Ubuntu 22.04...
    wsl --install -d Ubuntu-22.04
    echo.
    echo Please complete Ubuntu setup (username/password) in the window that opens.
    echo After setup, press any key here to continue...
    pause >nul
) else (
    echo [✓] Ubuntu distribution is installed
    wsl --list --verbose
    echo.
)

REM ============================================================================
REM STEP 3: Install ROS2 and dependencies in WSL2
REM ============================================================================

echo ========================================
echo Step 3: ROS2 Humble Installation
echo ========================================
echo.
echo This step will:
echo   - Install ROS2 Humble in WSL2 Ubuntu
echo   - Install Python dependencies (pyserial, flask, flask-cors)
echo   - Configure ROS2 environment
echo.
echo This will take 5-10 minutes...
echo.
echo Press any key to start ROS2 installation...
pause >nul

echo.
echo Installing ROS2 Humble and dependencies...
echo (This may take several minutes, please be patient)
echo.

REM Create installation script in WSL2
wsl bash -c "cat > /tmp/install_ros2.sh << 'EOF'
#!/bin/bash
set -e

echo '[1/7] Updating package lists...'
sudo apt update -y
sudo apt upgrade -y

echo '[2/7] Installing prerequisites...'
sudo apt install -y software-properties-common curl gnupg lsb-release

echo '[3/7] Adding ROS2 repository...'
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo \"deb http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'

echo '[4/7] Updating package lists with ROS2 repository...'
sudo apt update

echo '[5/8] Installing ROS2 Humble (this may take several minutes)...'
sudo apt install -y ros-humble-ros-base python3-pip

echo '[6/8] Installing git...'
sudo apt install -y git

echo '[7/8] Installing Python dependencies...'
pip3 install pyserial flask flask-cors

echo '[8/8] Configuring ROS2 environment...'
if ! grep -q 'source /opt/ros/humble/setup.bash' ~/.bashrc; then
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
fi

echo ''
echo '=========================================='
echo 'ROS2 Installation Complete!'
echo '=========================================='

EOF
chmod +x /tmp/install_ros2.sh
/tmp/install_ros2.sh
"

if %errorlevel% equ 0 (
    echo.
    echo [✓] ROS2 Humble installed successfully!
    echo.
) else (
    echo.
    echo [✗] ROS2 installation failed!
    echo Please check the error messages above and try again.
    echo.
    pause
    exit /b 1
)

REM ============================================================================
REM STEP 4: Clone Argo repository in WSL2
REM ============================================================================

echo ========================================
echo Step 4: Clone Argo Repository
echo ========================================
echo.
echo Cloning Argo repository from GitHub into WSL2...
echo This ensures you have the latest version and can easily update.
echo.

wsl bash -c "cd ~ && if [ -d argo ]; then echo 'Argo directory already exists. Updating...'; cd argo && git pull; else echo 'Cloning Argo repository...'; git clone https://github.com/SensorsINI/argo.git; fi"

if %errorlevel% equ 0 (
    echo.
    echo [✓] Argo repository cloned/updated successfully!
    echo Location: ~/argo/ in WSL2
    echo.
    echo To update later: wsl bash -c "cd ~/argo && git pull"
    echo.
) else (
    echo.
    echo [✗] Failed to clone repository
    echo.
    echo Please check your internet connection and try manually:
    echo   wsl bash -c "cd ~ && git clone https://github.com/SensorsINI/argo.git"
    echo.
    echo If git is not installed:
    echo   wsl bash -c "sudo apt install -y git"
    echo.
)

REM ============================================================================
REM STEP 5: Install USB passthrough tools
REM ============================================================================

echo ========================================
echo Step 5: USB Device Passthrough (usbipd)
echo ========================================
echo.
echo Installing usbipd-win for USB LoRa device access...
echo.

where usbipd >nul 2>&1
if %errorlevel% equ 0 (
    echo [✓] usbipd-win is already installed
    echo.
) else (
    echo Installing usbipd-win...
    winget install --interactive --exact dorssel.usbipd-win
    
    if %errorlevel% equ 0 (
        echo [✓] usbipd-win installed successfully
    ) else (
        echo [⚠] usbipd-win installation may have failed
        echo You can install manually from: https://github.com/dorssel/usbipd-win/releases
    )
    echo.
)

REM ============================================================================
REM STEP 6: Create desktop shortcut
REM ============================================================================

echo ========================================
echo Step 6: Create Desktop Shortcut
echo ========================================
echo.

set DESKTOP=%USERPROFILE%\Desktop
set LAUNCHER_PATH=%~dp0launch_argo_shore.bat

echo Creating desktop shortcut...
powershell -Command "$WS = New-Object -ComObject WScript.Shell; $SC = $WS.CreateShortcut('%DESKTOP%\Argo Shore Station.lnk'); $SC.TargetPath = '%LAUNCHER_PATH%'; $SC.WorkingDirectory = '%~dp0'; $SC.Description = 'Launch Argo Shore Station (LoRa + Web Dashboard)'; $SC.Save()"

if %errorlevel% equ 0 (
    echo [✓] Desktop shortcut created
    echo.
) else (
    echo [⚠] Failed to create desktop shortcut
    echo You can manually create a shortcut to: %LAUNCHER_PATH%
    echo.
)

REM ============================================================================
REM Setup Complete!
REM ============================================================================

echo.
echo ========================================
echo Setup Complete! 🎉
echo ========================================
echo.
echo Your Windows computer is now ready to run the Argo shore station.
echo.
echo Next steps:
echo.
echo 1. Plug in your Waveshare LoRa USB device
echo.
echo 2. Attach it to WSL2 (run in regular PowerShell, not as Admin):
echo    usbipd list
echo    usbipd attach --wsl --busid YOUR_BUSID
echo.
echo 3. Launch the shore station:
echo    Double-click "Argo Shore Station" on your Desktop
echo.
echo 4. Access the web dashboard in your browser:
echo    http://localhost:8081
echo.
echo ========================================
echo.
echo Tip: Run check_setup.bat to verify everything is working.
echo.
pause

exit /b 0

