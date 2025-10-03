#!/bin/bash
# Setup script for remote simulator on sensors-tobidh87.lan.ini.uzh.ch

# Load centralized configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "$SCRIPT_DIR/load_config.py" --export-shell)"

# Use config variable name
ARGO_DIR="$REMOTE_ARGO_DIR"

echo "🚢 Argo Remote Simulator Setup"
echo "=============================="
echo "Remote host: $REMOTE_HOST"
echo "Remote user: $REMOTE_USER"
echo "Argo directory: $ARGO_DIR"
echo ""

# Check SSH connection
echo "🔍 Testing SSH connection..."
if ! ssh -o ConnectTimeout=10 -o BatchMode=yes $REMOTE_USER@$REMOTE_HOST "echo 'SSH connection successful'" 2>/dev/null; then
    echo "❌ SSH connection failed!"
    echo "   Make sure you can SSH to the remote machine:"
    echo "   ssh $REMOTE_USER@$REMOTE_HOST"
    echo ""
    echo "   If you need to set up SSH keys:"
    echo "   ssh-copy-id $REMOTE_USER@$REMOTE_HOST"
    exit 1
fi

echo "✅ SSH connection successful"
echo ""

# Check if ROS2 is installed on remote
echo "🔍 Checking ROS2 installation on remote machine..."
if ! ssh $REMOTE_USER@$REMOTE_HOST "source /opt/ros/humble/setup.bash && python3 -c 'import rclpy; print(\"ROS2 available\")'" 2>/dev/null; then
    echo "❌ ROS2 not found on remote machine"
    echo "   Installing ROS2 Humble on remote machine..."
    
    ssh $REMOTE_USER@$REMOTE_HOST "
        # Clean up any existing ROS2 installation
        sudo rm -f /etc/apt/sources.list.d/ros2.list
        sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
        
        # Update package lists
        sudo apt update
        
        # Install prerequisites
        sudo apt install -y software-properties-common curl gnupg lsb-release
        
        # Add universe repository
        sudo add-apt-repository universe
        
        # Add ROS2 GPG key (updated method)
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        
        # Add ROS2 repository
        echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list
        
        # Update package lists again
        sudo apt update
        
        # Install ROS2 Humble
        sudo apt install -y ros-humble-desktop python3-argcomplete python3-colcon-common-extensions
        
        # Add ROS2 setup to bashrc
        if ! grep -q 'source /opt/ros/humble/setup.bash' ~/.bashrc; then
            echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
        fi
    "
    
    if [ $? -eq 0 ]; then
        echo "✅ ROS2 installed successfully on remote machine"
    else
        echo "❌ Failed to install ROS2 on remote machine"
        echo "   Manual installation required. Run on remote machine:"
        echo "   sudo apt update"
        echo "   sudo apt install -y ros-humble-desktop"
        exit 1
    fi
else
    echo "✅ ROS2 already installed on remote machine"
fi

echo ""

# Check if remote argo directory exists
echo "🔍 Checking remote argo directory..."
if ssh $REMOTE_USER@$REMOTE_HOST "test -d $ARGO_DIR"; then
    echo "✅ Remote argo directory exists: $ARGO_DIR"
    echo "   Skipping sync (directory already synchronized)"
else
    echo "❌ Remote argo directory not found: $ARGO_DIR"
    echo "   Please ensure the argo project is cloned and synchronized on the remote machine"
    exit 1
fi

echo ""

# Install Python dependencies on remote
echo "📦 Installing Python dependencies on remote machine..."
ssh $REMOTE_USER@$REMOTE_HOST "
    cd $ARGO_DIR
    # Source ROS2 environment first
    source /opt/ros/humble/setup.bash
    # Install only non-ROS2 Python dependencies
    pip3 install --user smbus2 pyserial pynmea2 numpy PyYAML matplotlib
"

if [ $? -eq 0 ]; then
    echo "✅ Python dependencies installed on remote machine"
else
    echo "❌ Failed to install Python dependencies"
    echo "   Note: rclpy and other ROS2 packages are installed via apt, not pip"
    exit 1
fi

echo ""

# Test remote simulator
echo "🧪 Testing remote simulator..."
ssh $REMOTE_USER@$REMOTE_HOST "
    cd $ARGO_DIR
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=42
    timeout 5 python3 nodes/argo_unified_simulator_bridge.py --help
"

if [ $? -eq 0 ]; then
    echo "✅ Remote simulator test successful"
else
    echo "❌ Remote simulator test failed"
    exit 1
fi

echo ""
echo "🎉 Remote simulator setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Start SSH tunnel (in terminal 1):"
echo "   ./scripts/remote_simulator_tunnel.sh"
echo ""
echo "2. Start remote simulator (in terminal 2):"
echo "   python3 scripts/remote_simulator_launch.py"
echo ""
echo "3. Start local bridge (in terminal 3):"
echo "   python3 nodes/argo_remote_simulator_bridge.py"
echo ""
echo "4. Start local Argo nodes:"
echo "   python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo.yaml"
echo "   python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml"
echo ""
echo "💡 The simulator will now run on the remote machine, saving CPU on your Orange Pi!"
