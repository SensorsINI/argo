#!/bin/bash
# Debug script for remote ROS2 installation issues

# Load centralized configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "$SCRIPT_DIR/load_config.py" --export-shell)"

echo "🔍 Remote ROS2 Debug Information"
echo "================================"
echo "Remote host: $REMOTE_HOST"
echo "Remote user: $REMOTE_USER"
echo ""

# Check SSH connection
echo "1. Testing SSH connection..."
if ssh -o ConnectTimeout=10 $REMOTE_USER@$REMOTE_HOST "echo 'SSH OK'"; then
    echo "✅ SSH connection successful"
else
    echo "❌ SSH connection failed"
    exit 1
fi
echo ""

# Check ROS2 installation
echo "2. Checking ROS2 installation..."
ssh $REMOTE_USER@$REMOTE_HOST "
    echo 'ROS2 packages installed:'
    dpkg -l | grep ros-humble | head -10
    echo ''
    echo 'ROS2 directory exists:'
    ls -la /opt/ros/ 2>/dev/null || echo 'No /opt/ros/ directory'
    echo ''
    echo 'ROS2 setup file exists:'
    ls -la /opt/ros/humble/setup.bash 2>/dev/null || echo 'No setup.bash file'
"
echo ""

# Check ROS2 environment
echo "3. Testing ROS2 environment..."
ssh $REMOTE_USER@$REMOTE_HOST "
    echo 'Sourcing ROS2 environment...'
    source /opt/ros/humble/setup.bash 2>&1
    echo 'ROS2 environment sourced'
    echo ''
    echo 'Testing rclpy import:'
    python3 -c 'import rclpy; print(\"rclpy import successful\")' 2>&1
    echo ''
    echo 'Testing ros2 command:'
    ros2 --help 2>&1 | head -5
"
echo ""

# Check Python packages
echo "4. Checking Python packages..."
ssh $REMOTE_USER@$REMOTE_HOST "
    echo 'Python version:'
    python3 --version
    echo ''
    echo 'Installed Python packages:'
    pip3 list | grep -E '(rclpy|smbus2|pyserial|numpy)' || echo 'No relevant packages found'
    echo ''
    echo 'Python path:'
    python3 -c 'import sys; print(\"\\n\".join(sys.path))'
"
echo ""

# Check ROS2 repository
echo "5. Checking ROS2 repository configuration..."
ssh $REMOTE_USER@$REMOTE_HOST "
    echo 'ROS2 sources list:'
    cat /etc/apt/sources.list.d/ros2.list 2>/dev/null || echo 'No ros2.list file'
    echo ''
    echo 'ROS2 GPG key:'
    ls -la /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || echo 'No GPG key file'
    echo ''
    echo 'APT update test:'
    sudo apt update 2>&1 | grep -E '(ros2|ROS2)' || echo 'No ROS2-related apt issues'
"
echo ""

# Check argo directory
echo "6. Checking argo directory..."
ssh $REMOTE_USER@$REMOTE_HOST "
    echo 'Argo directory:'
    ls -la $REMOTE_ARGO_DIR/ 2>/dev/null || echo 'Argo directory not found'
    echo ''
    echo 'Requirements file:'
    cat $REMOTE_ARGO_DIR/requirements.txt 2>/dev/null || echo 'No requirements file'
"
echo ""

echo "🔧 Suggested fixes:"
echo "1. If ROS2 is not installed:"
echo "   ssh $REMOTE_USER@$REMOTE_HOST"
echo "   sudo apt update"
echo "   sudo apt install -y ros-humble-desktop"
echo ""
echo "2. If rclpy import fails:"
echo "   ssh $REMOTE_USER@$REMOTE_HOST"
echo "   source /opt/ros/humble/setup.bash"
echo "   python3 -c 'import rclpy'"
echo ""
echo "3. If Python packages are missing:"
echo "   ssh $REMOTE_USER@$REMOTE_HOST"
echo "   cd $REMOTE_ARGO_DIR"
echo "   pip3 install --user smbus2 pyserial pynmea2 numpy PyYAML matplotlib"





