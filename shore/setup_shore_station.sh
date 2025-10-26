#!/bin/bash
################################################################################
# Argo Shore Station Setup Script
################################################################################
#
# This script checks for and installs all dependencies needed to run the
# Argo shore station (LoRa receiver + Web Dashboard) on Ubuntu/WSL2.
#
# Usage:
#   ./shore/setup_shore_station.sh
#
# What it does:
#   1. Checks for ROS2 Humble installation
#   2. Installs ROS2 Humble if not found
#   3. Checks for Python dependencies (pyserial, flask, flask-cors)
#   4. Installs missing Python packages
#   5. Configures ROS2 environment in ~/.bashrc
#   6. Verifies USB serial device access (dialout group)
#   7. Creates launcher script
#
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory (works even if called via symlink)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo ""
echo "========================================"
echo "Argo Shore Station Setup"
echo "========================================"
echo ""
echo "This script will set up your Ubuntu/WSL2 environment for running"
echo "the Argo shore station (LoRa receiver + Web Dashboard)."
echo ""

# Check if running on Ubuntu/Debian
if ! command -v apt &> /dev/null; then
    echo -e "${RED}ERROR: This script requires Ubuntu/Debian (apt package manager)${NC}"
    echo "For other Linux distributions, manually install:"
    echo "  - ROS2 Humble"
    echo "  - Python packages: pyserial flask flask-cors"
    exit 1
fi

echo -e "${BLUE}[1/7] Checking system...${NC}"
echo "  OS: $(lsb_release -d | cut -f2)"
echo "  Argo root: $ARGO_ROOT"

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
UBUNTU_CODENAME=$(lsb_release -cs)
echo "  Ubuntu version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"

# Determine appropriate ROS2 version
if [[ "$UBUNTU_VERSION" == "22.04" ]] || [[ "$UBUNTU_CODENAME" == "jammy" ]]; then
    ROS_DISTRO="humble"
    echo "  ROS2 version: Humble (recommended for Ubuntu 22.04)"
elif [[ "$UBUNTU_VERSION" == "20.04" ]] || [[ "$UBUNTU_CODENAME" == "focal" ]]; then
    ROS_DISTRO="foxy"
    echo -e "  ${YELLOW}⚠${NC} Ubuntu 20.04 detected - will install ROS2 Foxy instead of Humble"
    echo "    (For Humble, upgrade to Ubuntu 22.04)"
else
    echo -e "  ${RED}✗${NC} Unsupported Ubuntu version: $UBUNTU_VERSION"
    echo "  Supported versions: 20.04 (ROS2 Foxy) or 22.04 (ROS2 Humble)"
    exit 1
fi

# Check for conda environment
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo -e "  ${YELLOW}⚠${NC} Running in conda environment: $CONDA_DEFAULT_ENV"
    echo "    This may cause Python package conflicts with ROS2"
    echo "    Consider running: conda deactivate"
fi
echo ""

# Function to check if ROS2 is installed
check_ros2() {
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
        if command -v ros2 &> /dev/null; then
            return 0
        fi
    fi
    return 1
}

# Check ROS2 installation
echo -e "${BLUE}[2/7] Checking ROS2 $ROS_DISTRO installation...${NC}"
if check_ros2; then
    echo -e "  ${GREEN}✓${NC} ROS2 $ROS_DISTRO is already installed"
    ROS2_VERSION=$(ros2 --version 2>/dev/null || echo "unknown")
    echo "    Version: $ROS2_VERSION"
else
    echo -e "  ${YELLOW}⚠${NC} ROS2 $ROS_DISTRO not found"
    echo ""
    echo "  Installing ROS2 $ROS_DISTRO (this will take 5-10 minutes)..."
    echo "  You may be prompted for your password (sudo access required)"
    echo ""
    
    # Update package list
    echo "  [2.1/2.6] Updating package lists..."
    sudo apt update -qq
    
    # Install prerequisites
    echo "  [2.2/2.6] Installing prerequisites..."
    sudo apt install -y software-properties-common curl gnupg lsb-release
    
    # Add ROS2 repository with proper GPG key handling
    echo "  [2.3/2.6] Adding ROS2 repository..."
    sudo add-apt-repository universe -y
    
    # Use proper GPG key installation for newer systems
    if command -v gpg &> /dev/null; then
        echo "    Installing ROS2 GPG key..."
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    else
        # Fallback to apt-key for older systems
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    fi
    
    # Update with ROS2 repo
    echo "  [2.4/2.6] Updating package lists with ROS2 repository..."
    sudo apt update
    
    # Install git if not present
    echo "  [2.5/2.6] Ensuring git is installed..."
    sudo apt install -y git
    
    # Install ROS2
    echo "  [2.6/2.6] Installing ROS2 $ROS_DISTRO (this may take several minutes)..."
    echo "    Package: ros-${ROS_DISTRO}-ros-base"
    
    if ! sudo apt install -y ros-${ROS_DISTRO}-ros-base python3-pip; then
        echo -e "  ${RED}✗${NC} ROS2 installation failed!"
        echo ""
        echo "  Possible issues:"
        echo "    - No internet connection"
        echo "    - Disk space full"
        echo "    - Package repository issues"
        echo ""
        echo "  Try manually:"
        echo "    sudo apt update"
        echo "    sudo apt install ros-${ROS_DISTRO}-ros-base"
        echo ""
        exit 1
    fi
    
    # Verify installation
    if check_ros2; then
        echo -e "  ${GREEN}✓${NC} ROS2 $ROS_DISTRO installed successfully!"
    else
        echo -e "  ${RED}✗${NC} ROS2 installation failed!"
        echo "  The package was installed but cannot be found."
        echo "  Please check the error messages above."
        exit 1
    fi
fi
echo ""

# Ensure ROS2 is sourced for rest of script
source /opt/ros/${ROS_DISTRO}/setup.bash

# Check Python dependencies
echo -e "${BLUE}[3/7] Checking Python dependencies...${NC}"

MISSING_PACKAGES=()

# Check pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    MISSING_PACKAGES+=("pyserial")
fi

# Check flask
if ! python3 -c "import flask" 2>/dev/null; then
    MISSING_PACKAGES+=("flask")
fi

# Check flask-cors
if ! python3 -c "import flask_cors" 2>/dev/null; then
    MISSING_PACKAGES+=("flask-cors")
fi

if [ ${#MISSING_PACKAGES[@]} -eq 0 ]; then
    echo -e "  ${GREEN}✓${NC} All Python dependencies are installed"
    python3 -c "import serial; print('    pyserial:', serial.__version__)"
    python3 -c "import flask; print('    flask:', flask.__version__)"
    python3 -c "import flask_cors; print('    flask-cors: installed')"
else
    echo -e "  ${YELLOW}⚠${NC} Missing Python packages: ${MISSING_PACKAGES[*]}"
    echo "  Installing missing packages..."
    pip3 install "${MISSING_PACKAGES[@]}" > /dev/null 2>&1
    
    # Verify installation
    ALL_OK=true
    for package in "${MISSING_PACKAGES[@]}"; do
        case $package in
            pyserial)
                if ! python3 -c "import serial" 2>/dev/null; then
                    ALL_OK=false
                fi
                ;;
            flask)
                if ! python3 -c "import flask" 2>/dev/null; then
                    ALL_OK=false
                fi
                ;;
            flask-cors)
                if ! python3 -c "import flask_cors" 2>/dev/null; then
                    ALL_OK=false
                fi
                ;;
        esac
    done
    
    if $ALL_OK; then
        echo -e "  ${GREEN}✓${NC} Python dependencies installed successfully!"
    else
        echo -e "  ${RED}✗${NC} Some Python packages failed to install"
        echo "  Try manually: pip3 install pyserial flask flask-cors"
        exit 1
    fi
fi
echo ""

# Check ROS2 environment in bashrc
echo -e "${BLUE}[4/7] Configuring ROS2 environment...${NC}"
if grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
    echo -e "  ${GREEN}✓${NC} ROS2 environment already configured in ~/.bashrc"
else
    echo -e "  ${YELLOW}⚠${NC} Adding ROS2 environment to ~/.bashrc"
    echo "" >> ~/.bashrc
    echo "# ROS2 $ROS_DISTRO environment (added by Argo setup)" >> ~/.bashrc
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
    echo -e "  ${GREEN}✓${NC} ROS2 environment added to ~/.bashrc"
    echo "    (will be active in new terminal sessions)"
fi
echo ""

# Check USB serial device access (dialout group)
echo -e "${BLUE}[5/7] Checking USB serial device access...${NC}"
if groups | grep -q dialout; then
    echo -e "  ${GREEN}✓${NC} User is in 'dialout' group (can access USB serial devices)"
else
    echo -e "  ${YELLOW}⚠${NC} User is not in 'dialout' group"
    echo "  Adding user to 'dialout' group for USB serial access..."
    sudo usermod -a -G dialout "$USER"
    echo -e "  ${GREEN}✓${NC} User added to 'dialout' group"
    echo -e "  ${YELLOW}⚠${NC} You must log out and back in for this to take effect!"
    echo "    Run: 'exit' then reopen terminal"
fi
echo ""

# Create launcher script
echo -e "${BLUE}[6/7] Creating launcher script...${NC}"
LAUNCHER="$ARGO_ROOT/shore/launch_shore_station.sh"

cat > "$LAUNCHER" << 'EOF'
#!/bin/bash
################################################################################
# Argo Shore Station Launcher
################################################################################
# Launches LoRa shore receiver and web dashboard

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS2 environment (try both Humble and Foxy)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "ERROR: ROS2 not found! Run setup_shore_station.sh first."
    exit 1
fi

# Detect serial port
LORA_PORT=""
for port in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
    if [ -e "$port" ]; then
        LORA_PORT="$port"
        break
    fi
done

if [ -z "$LORA_PORT" ]; then
    echo "WARNING: No USB serial device found (/dev/ttyUSB* or /dev/ttyACM*)"
    echo "Please connect your LoRa USB device and try again."
    echo ""
    echo "If using WSL2, attach device with:"
    echo "  usbipd list"
    echo "  usbipd attach --wsl --busid YOUR_BUSID"
    exit 1
fi

echo "========================================"
echo "Argo Shore Station"
echo "========================================"
echo ""
echo "Starting services..."
echo "  LoRa port: $LORA_PORT"
echo "  Web dashboard: http://localhost:8081"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch LoRa receiver in background
cd "$ARGO_ROOT"
python3 shore/lora_shore.py --port "$LORA_PORT" &
LORA_PID=$!

# Wait for LoRa to initialize
sleep 2

# Launch web dashboard
python3 nodes/argo_web_dashboard.py &
DASHBOARD_PID=$!

# Wait a moment for dashboard to start
sleep 3

echo ""
echo "========================================"
echo "Shore Station Running!"
echo "========================================"
echo ""
echo "Web Dashboard: http://localhost:8081"
echo ""
echo "To find your IP for remote access:"
echo "  hostname -I"
echo "  Then access from phone: http://YOUR_IP:8081"
echo ""
echo "Press Ctrl+C to stop both services"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping shore station..."
    kill $LORA_PID 2>/dev/null
    kill $DASHBOARD_PID 2>/dev/null
    wait $LORA_PID 2>/dev/null
    wait $DASHBOARD_PID 2>/dev/null
    echo "Stopped."
    exit 0
}

# Set trap for Ctrl+C
trap cleanup SIGINT SIGTERM

# Wait for both processes
wait
EOF

chmod +x "$LAUNCHER"
echo -e "  ${GREEN}✓${NC} Launcher script created: $LAUNCHER"
echo ""

# Verify installation
echo -e "${BLUE}[7/7] Verifying installation...${NC}"
ALL_OK=true

# Check ROS2
if ! check_ros2; then
    echo -e "  ${RED}✗${NC} ROS2 not properly installed"
    ALL_OK=false
else
    echo -e "  ${GREEN}✓${NC} ROS2 Humble: OK"
fi

# Check Python packages
if ! python3 -c "import serial, flask, flask_cors" 2>/dev/null; then
    echo -e "  ${RED}✗${NC} Python dependencies missing"
    ALL_OK=false
else
    echo -e "  ${GREEN}✓${NC} Python dependencies: OK"
fi

# Check Argo files
if [ ! -f "$ARGO_ROOT/shore/lora_shore.py" ]; then
    echo -e "  ${RED}✗${NC} lora_shore.py not found"
    ALL_OK=false
else
    echo -e "  ${GREEN}✓${NC} lora_shore.py: OK"
fi

if [ ! -f "$ARGO_ROOT/nodes/argo_web_dashboard.py" ]; then
    echo -e "  ${RED}✗${NC} argo_web_dashboard.py not found"
    ALL_OK=false
else
    echo -e "  ${GREEN}✓${NC} argo_web_dashboard.py: OK"
fi

echo ""

if $ALL_OK; then
    echo "========================================"
    echo -e "${GREEN}✓ Setup Complete!${NC}"
    echo "========================================"
    echo ""
    echo "ROS2 $ROS_DISTRO installed and configured"
    echo ""
    
    if [ -n "$CONDA_DEFAULT_ENV" ]; then
        echo -e "${YELLOW}⚠ IMPORTANT:${NC} You're in a conda environment ($CONDA_DEFAULT_ENV)"
        echo "   Before running the shore station, deactivate conda:"
        echo "     conda deactivate"
        echo ""
    fi
    
    # Check if running in WSL and provide USB setup instructions
    if [[ $(uname -r) == *microsoft* ]] || [[ $(uname -r) == *WSL* ]]; then
        echo "========================================"
        echo "USB Device Setup for WSL2"
        echo "========================================"
        echo ""
        echo "IMPORTANT: You're running in WSL2. USB devices need special setup."
        echo ""
        echo "To use your LoRa USB device with WSL2:"
        echo ""
        echo "1. Install usbipd-win on WINDOWS (not Linux):"
        echo ""
        echo "   Open Windows PowerShell (not this Ubuntu terminal)"
        echo "   and run:"
        echo ""
        echo "     winget install --interactive --exact dorssel.usbipd-win"
        echo ""
        echo "2. Plug in your Waveshare LoRa USB device"
        echo ""
        echo "3. Attach device to WSL2 (in Windows PowerShell):"
        echo ""
        echo "     usbipd list"
        echo "     usbipd bind --busid X-X        (replace X-X with your device's BUSID)"
        echo "     usbipd attach --wsl --busid X-X"
        echo ""
        echo "4. Verify device is visible (in this Ubuntu terminal):"
        echo ""
        echo "     ls /dev/ttyUSB* /dev/ttyACM*"
        echo ""
        echo "   Should show: /dev/ttyUSB0 or /dev/ttyACM0"
        echo ""
        echo "Note: Run 'usbipd attach' in Windows PowerShell after each Windows restart"
        echo ""
        echo "========================================"
        echo ""
    fi
    
    echo "Next steps:"
    echo ""
    echo "1. Connect your Waveshare LoRa USB device"
    if [[ $(uname -r) == *microsoft* ]] || [[ $(uname -r) == *WSL* ]]; then
        echo "   (See USB setup instructions above)"
    fi
    echo ""
    echo "2. Launch the shore station:"
    echo "     $LAUNCHER"
    echo ""
    echo "   Or create a shorter alias in ~/.bashrc:"
    echo "     alias argo_shore='$LAUNCHER'"
    echo ""
    echo "3. Access web dashboard:"
    echo "     http://localhost:8081"
    echo ""
    echo "========================================"
else
    echo "========================================"
    echo -e "${RED}✗ Setup Incomplete${NC}"
    echo "========================================"
    echo ""
    echo "Please fix the errors above and run this script again."
    exit 1
fi

