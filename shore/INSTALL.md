# Shore-Side LoRa Node Installation Guide

This guide covers installing dependencies for the shore-side LoRa communication node on a desktop/laptop computer (not the Argo sailboat).

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy) or compatible Linux
- **Python**: 3.10 or later
- **Hardware**: Waveshare USB-TO-LoRa-LF-B module (or compatible)
- **Disk Space**: ~500 MB for ROS2 base installation

## Installation Overview

1. Install ROS2 Humble (base)
2. Install Python dependencies
3. Configure serial port permissions
4. Test installation

---

## 1. Install ROS2 Humble (Minimal)

### Quick Install (Recommended)

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Base (minimal, no GUI tools)
sudo apt update
sudo apt install -y ros-humble-ros-base

# Install Python dependencies for ROS2
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

**Installation size**: ~300-500 MB (much smaller than desktop-full)

### Verify ROS2 Installation

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Test ROS2 command
ros2 --help

# Should show ROS2 command help without errors
```

### Alternative: Docker Installation

If you prefer Docker (useful for non-Ubuntu systems):

```bash
# Pull ROS2 Humble base image
docker pull ros:humble-ros-base

# Run shore node in container
docker run -it --rm \
  --device=/dev/ttyACM0 \
  --network host \
  -v $(pwd):/workspace \
  ros:humble-ros-base \
  bash -c "source /opt/ros/humble/setup.bash && cd /workspace/shore && python3 lora_shore.py"
```

---

## 2. Install Python Dependencies

```bash
# Navigate to shore directory
cd /path/to/argo/shore

# Install Python packages
pip3 install -r requirements.txt

# Or install manually
pip3 install pyserial
```

---

## 3. Configure Serial Port Permissions

### Add User to dialout Group

```bash
# Add current user to dialout group (for serial port access)
sudo usermod -a -G dialout $USER

# Verify membership
groups $USER | grep dialout

# Log out and back in (or reboot) for changes to take effect
```

### Verify Serial Port Access

```bash
# Check if Waveshare module is detected
ls -l /dev/ttyACM0

# Should show: crw-rw---- 1 root dialout ...
# The "rw" for group means you can read/write

# Test access (should not show permission denied)
cat /dev/ttyACM0
# Press Ctrl+C to stop
```

---

## 4. Set Up Shell Environment

### Add to ~/.bashrc (Automatic)

```bash
# Add ROS2 sourcing to your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Reload shell
source ~/.bashrc
```

### Manual Sourcing (Each Terminal)

```bash
# Source ROS2 environment in each terminal
source /opt/ros/humble/setup.bash
```

---

## 5. Test Installation

### Test 1: Check Dependencies

```bash
# Navigate to shore directory
cd /path/to/argo/shore

# Run dependency check
python3 -c "
import sys
try:
    import rclpy
    print('✅ ROS2 (rclpy) installed')
except ImportError:
    print('❌ ROS2 not installed or not sourced')
    print('   Run: source /opt/ros/humble/setup.bash')
    sys.exit(1)

try:
    import serial
    print('✅ pyserial installed')
except ImportError:
    print('❌ pyserial not installed')
    print('   Run: pip3 install pyserial')
    sys.exit(1)

print('✅ All dependencies OK!')
"
```

### Test 2: Run Shore Node (Dry Run)

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Try to run the node
python3 lora_shore.py --ros-args -p serial_port:=/dev/ttyACM0

# Should show:
# [INFO] [timestamp] [lora_shore_node]: Connected to Waveshare LoRa...
# Or error if device not connected (that's OK for testing)
```

---

## Troubleshooting

### Error: "ModuleNotFoundError: No module named 'rclpy'"

**Cause**: ROS2 not installed or environment not sourced.

**Solution**:
```bash
# Check if ROS2 is installed
dpkg -l | grep ros-humble

# If not installed, follow Step 1 above

# If installed, source the environment
source /opt/ros/humble/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Error: "ModuleNotFoundError: No module named 'serial'"

**Cause**: pyserial not installed.

**Solution**:
```bash
pip3 install pyserial
```

### Error: "[Errno 13] Permission denied: '/dev/ttyACM0'"

**Cause**: User not in dialout group.

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in (required!)
# Then verify:
groups | grep dialout
```

### Error: "No such file or directory: '/dev/ttyACM0'"

**Cause**: Waveshare module not connected or not detected.

**Solution**:
```bash
# Check if device is connected
lsusb | grep -i 'serial\|lora\|waveshare'

# Check dmesg for USB events
dmesg | tail -20

# Try different USB port
# Module should appear as /dev/ttyACM0 or /dev/ttyUSB0
```

### Error: "ros2: command not found"

**Cause**: ROS2 not installed or not in PATH.

**Solution**:
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Or add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Minimal vs Full ROS2 Installation

### What We Install (ros-humble-ros-base)

**Includes:**
- Core ROS2 libraries (rclpy, rclcpp)
- Communication infrastructure (DDS)
- Command-line tools (ros2 cli)
- Python and C++ APIs

**Excludes:**
- RViz (3D visualization) - not needed for shore side
- Gazebo (simulation) - not needed
- Desktop tools - not needed
- Large demos and tutorials

**Why minimal?**
- Faster installation (~5-10 minutes vs 30+ minutes)
- Less disk space (~500 MB vs 2+ GB)
- Shore computer only needs to communicate, not visualize

### If You Need Visualization (Optional)

For RViz or other GUI tools:

```bash
# Install desktop version instead
sudo apt install ros-humble-desktop

# Or add just RViz
sudo apt install ros-humble-rviz2
```

---

## Platform-Specific Notes

### Ubuntu 22.04 (Jammy) - RECOMMENDED
- Fully supported
- Follow instructions above

### Ubuntu 20.04 (Focal)
- Use ROS2 Foxy instead of Humble
- Change all "humble" to "foxy" in commands

### Other Linux Distributions
- ROS2 Humble supports: Debian, RHEL, Fedora
- See: https://docs.ros.org/en/humble/Installation.html

### macOS / Windows
- Use Docker method (see Step 1, Alternative)
- Or use Windows Subsystem for Linux (WSL2) on Windows

---

## Verifying Everything Works

### Complete Verification Script

```bash
#!/bin/bash
# Save as: check_shore_install.sh
# Run: bash check_shore_install.sh

echo "=== Shore-Side LoRa Node Dependency Check ==="
echo ""

# Check ROS2
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 command found"
    ros2 --version
else
    echo "❌ ROS2 not found. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check Python
python3 --version
echo "✅ Python 3 found"

# Check rclpy
if python3 -c "import rclpy" 2>/dev/null; then
    echo "✅ rclpy (ROS2 Python) found"
else
    echo "❌ rclpy not found. Install: sudo apt install ros-humble-ros-base"
    exit 1
fi

# Check pyserial
if python3 -c "import serial" 2>/dev/null; then
    echo "✅ pyserial found"
else
    echo "❌ pyserial not found. Install: pip3 install pyserial"
    exit 1
fi

# Check dialout group
if groups | grep -q dialout; then
    echo "✅ User in dialout group"
else
    echo "⚠️  User NOT in dialout group. Run: sudo usermod -a -G dialout $USER"
    echo "   Then log out and back in"
fi

# Check serial device
if [ -e /dev/ttyACM0 ]; then
    echo "✅ /dev/ttyACM0 found"
    ls -l /dev/ttyACM0
elif [ -e /dev/ttyUSB0 ]; then
    echo "✅ /dev/ttyUSB0 found (use this port instead)"
    ls -l /dev/ttyUSB0
else
    echo "⚠️  No serial device found. Connect Waveshare module"
fi

echo ""
echo "=== All checks passed! ==="
echo "Ready to run: python3 lora_shore.py"
```

---

## Quick Start Summary

```bash
# 1. Install ROS2 (one time)
sudo apt update
sudo apt install -y ros-humble-ros-base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 2. Install Python dependencies (one time)
pip3 install pyserial

# 3. Configure permissions (one time)
sudo usermod -a -G dialout $USER
# Log out and back in

# 4. Run shore node (every time)
source /opt/ros/humble/setup.bash  # Or automatic from ~/.bashrc
python3 shore/lora_shore.py
```

---

## Support

- **ROS2 Installation Issues**: https://docs.ros.org/en/humble/Installation.html
- **Argo LoRa Documentation**: See `README.md` and `../nodes/README-LORA-TO-WAVESHARE.md`
- **Serial Port Issues**: Check `dmesg` output after plugging in device

---

## Docker Alternative (Complete Example)

If you prefer containerized deployment:

### Dockerfile

```dockerfile
FROM ros:humble-ros-base

# Install Python dependencies
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install pyserial

# Copy shore node
COPY shore/ /workspace/shore/
WORKDIR /workspace/shore

# Run node
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 lora_shore.py"]
```

### Build and Run

```bash
# Build image
docker build -t argo-shore .

# Run with device access
docker run -it --rm \
  --device=/dev/ttyACM0 \
  --network host \
  argo-shore
```

---

**Installation Complete!** You're ready to communicate with Argo via LoRa! 🚀

