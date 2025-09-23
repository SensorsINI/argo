# Argo Autonomous Sailboat

An autonomous sailboat system based on Dragonforce 65 hull, running on Orange Pi Zero 2W with ROS2. The system includes comprehensive sensor integration, autonomous navigation, and safety monitoring.

## System Overview

The Argo system consists of multiple ROS2 nodes that work together to provide autonomous sailing capabilities:

- **Sensor Nodes**: GPS (u-blox NEO-M9N), IMU (ICM-20948), Wind sensors (3x Sensirion SDP3x), Battery/Water monitoring
- **Control Interface**: PWM capture for radio control and servo output  
- **Autonomous Control**: Navigation and sail trimming algorithms
- **Safety Systems**: Manual override, battery monitoring, water intrusion detection

### Demo Video
See the Argo autonomous sailboat in action at the 2024 CCNW (before current waterproofing and PCB developements): [Argo Sailboat Demo](https://youtu.be/tjC1262BsCY?si=1GFPk1QcOqpzw8h2)

![Argo Autonomous Sailboat](https://img.youtube.com/vi/tjC1262BsCY/maxresdefault.jpg)

## Hardware Platform

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **GPS**: u-blox NEO-M9N via UART5 (/dev/ttyS5)
- **IMU**: ICM-20948 9-DOF via I2C0 (0x69)
- **Wind Sensor**: 3x Sensirion SDP3x differential pressure sensors via I2C0 (0x21, 0x22, 0x23)
- **ADC**: MAX11612 for battery/water sensing via I2C0 (0x34)
- **Environment**: SHT45 temperature/humidity via I2C0 (0x44)
- **PWM I/O**: Custom kernel module for radio control and servo interfaces

## ROS2 Software Architecture

### Project Organization

The Argo system follows a modular ROS2 architecture with clear separation of concerns:

**Directory Structure:**
- **`nodes/`** - Individual ROS2 nodes (Python-based sensor interfaces)
- **`launch/`** - Lifecycle management, systemd services, and launch configurations
- **`power_control/`** - Power management system (separate ROS2 package)
- **`foxglove/`** - Visualization layouts for Foxglove Studio

## Directory Overview

### `launch/` - System Management & Services
Central control hub for the entire Argo system:
- **`argo_lifecycle_manager.py`** - Core lifecycle management with intelligent monitoring
- **`argo_*.sh`** - Shell scripts for start/stop/restart/status operations
- **`argo-launch.service`** - Systemd service configuration
- **`argo_gui.py`** - GTK-based status monitoring GUI
- **`argo_storage_monitor.py`** - Storage space monitoring and notifications
- **`Makefile`** - Service installation and management automation

### `nodes/` - ROS2 Node Implementations
Hardware interface and control nodes:
- **Sensor Nodes**: `gps.py`, `imu.py`, `anem.py`, `battery_water.py`, `temp_monitor.py`
- **Control Nodes**: `pwm.py`, `controller.py`, `record.py`
- **`pwm_capture_module/`** - Custom kernel module for radio control and servo interfaces
- **`RTIMULib2/`** - IMU sensor fusion library
- **Configuration files**: `argo.yaml`, calibration data, and support utilities

### `power_control/` - Hardware Power Management
Standalone ROS2 package for power button and LED control:
- **`argo_power_control.py`** - Main power control node with GPIO management
- **Hardware Functions**: Power button monitoring, LED patterns (heartbeat/SOS), graceful shutdown
- **ROS2 Services**: LED control, system health monitoring
- **`Makefile`** - Independent installation and service management

### `pcb/` - Hardware Design Files
Custom PCB development for production-ready integration:
- **`argo-v9-stable/`** - Current stable PCB design (KiCad project files)
- **`datasheets/`** - Component specifications and reference materials
- **`orange-pi/`** - Orange Pi Zero 2W integration documentation and pin definitions
- **Bill of Materials**: Component sourcing and assembly documentation

### `foxglove/` - Real-time Visualization
Foxglove Studio integration for live system monitoring:
- **`argo_ros2.json`** - Pre-configured dashboard layout for Argo sailboat
- **`custom-argo-panel/`** - TypeScript custom panel for specialized boat visualization
- **`setup_foxglove.sh`** - Automated Foxglove Bridge configuration and startup

**Core ROS2 Nodes:**
- **`gps.py`** - GPS receiver interface (UART5, u-blox NEO-M9N)
- **`imu.py`** - 9-DOF IMU sensor fusion (I2C, ICM-20948)
- **`anem.py`** - Wind sensor array (3x SDP3x pressure sensors)
- **`battery_water.py`** - Power monitoring and safety systems
- **`pwm.py`** - Radio control input and servo output interface
- **`controller.py`** - Autonomous navigation and sail control algorithms
- **`record.py`** - Data recording management (ROS2 bag files)
- **`temp_monitor.py`** - System temperature monitoring

### Node Lifecycle Management

The Argo system uses a sophisticated lifecycle management approach centered around **`argo_lifecycle_manager.py`**:

**Key Features:**
- **Real-time Monitoring**: Active process detection during startup stabilization (not static sleeps)
- **Failure Detection**: Immediate FATAL error detection and reporting from systemd journal
- **Auto-restart**: Configurable restart policies with exponential backoff
- **Graceful Shutdown**: Proper cleanup and process termination
- **Status Reporting**: Comprehensive system health monitoring

**Lifecycle Management Modes:**
```bash
python3 launch/argo_lifecycle_manager.py start      # Launch all nodes
python3 launch/argo_lifecycle_manager.py stop       # Graceful shutdown
python3 launch/argo_lifecycle_manager.py restart    # Restart all nodes
python3 launch/argo_lifecycle_manager.py status     # Show system status
python3 launch/argo_lifecycle_manager.py monitor    # Continuous monitoring
python3 launch/argo_lifecycle_manager.py continuous # Production mode (systemd)
```

**Critical Node Management:**
- **Critical Nodes**: `pwm.py`, `controller.py` (essential for boat operation)
- **Success Criteria**: All critical nodes + minimum 3 total nodes running
- **Failure Handling**: Intelligent restart with failure analysis and error reporting

**Startup Monitoring Pattern:**
1. **Launch Phase** - Start all node processes
2. **Detection Phase** - Wait for nodes to register (30s timeout)
3. **Stabilization Phase** - Active monitoring for failures (15s with 1s intervals)
4. **Validation Phase** - Final status check and success determination

**Systemd Integration:**
- **Service**: `argo-launch.service` runs lifecycle manager in continuous mode
- **Dependencies**: Waits for network and hardware module initialization
- **Restart Policy**: Automatic restart on failure with 5-second delay
- **Environment**: ROS2 Humble sourcing and logging configuration

## Installation on New SD Card

### 1. Flash Orange Pi OS
```bash
# Download Orange Pi OS from official website
# Use Orange Pi Imager or similar tool to flash the OS
```

### 2. Initial System Setup
```bash
# Boot the system and connect via SSH
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install python3-pip python3-dev build-essential
sudo apt install i2c-tools device-tree-compiler git
sudo apt install ros-humble-desktop  # ROS2 Humble
```

### 3. Clone Repository
```bash
cd /home/orangepi
git clone https://github.com/SensorsINI/argo.git
cd argo
```

### 4. Install Python Dependencies
```bash
# Install system dependencies
pip3 install smbus2 pyserial numpy tqdm matplotlib

# Install ROS2 dependencies (if not already installed)
sudo apt install python3-rclpy python3-std-msgs python3-geometry-msgs
```

### 5. Hardware Configuration

#### Enable I2C and UART Overlays
Edit `/boot/orangepiEnv.txt`:
```bash
sudo nano /boot/orangepiEnv.txt
```

Add these overlays:
```
overlays=pi-i2c0 disable-uart0 ph-uart5 pi-pwm2 pi-pwm4
user_overlays=argo_radio_servo_overlay
```

#### Install PWM Capture Module
```bash
cd /home/orangepi/argo/scripts/pwm_capture_module
sudo make install
sudo depmod -a
sudo modprobe argo_radio_servo_module
```

#### Set User Permissions
```bash
# Add user to required groups
sudo usermod -a -G i2c,dialout $USER
# Logout and login again for group changes to take effect
```

### 6. Verify Hardware Setup
```bash
# Check I2C devices (should show: 21 22 23 34 44 69)
sudo i2cdetect -y 0

# Check PWM kernel module
lsmod | grep argo
ls -la /sys/kernel/argo_radio_servo/

# Check UART GPS (should show NMEA data)
sudo cat /dev/ttyS5
```

### 7. Install Argo CLI and Configure Services
```bash
# Install Argo CLI (shell aliases and functions)
make install-argo-cli

# Activate CLI in current terminal
source ~/.bashrc

# Install and configure system services (optional)
make -C launch install
sudo systemctl daemon-reload
sudo systemctl enable argo-launch.service
```

#### Makefile and Shell Aliases

The repository includes a comprehensive Makefile system for easy management:

**Top-level Makefile targets:**
- `make install-deps` - Install ROS2 dependencies (foxglove-bridge)
- `make install-python-deps` - Install Python runtime dependencies
- `make install-hardware` - Install PWM capture module
- `make install-all` - Complete hardware and dependency setup
- `make install-argo-cli` - Install shell aliases and functions
- `make -C launch start` - Start Argo system with monitoring
- `make -C launch stop` - Stop Argo system
- `make -C power_control install` - Install power control system

**Shell aliases (available after `make install-argo-cli`):**
- `al` - Launch Argo service with monitoring
- `aq` - Quit/stop Argo service
- `ars` - Restart Argo service
- `as` - Show Argo status
- `ar` - Start data recording (via ROS2 service)
- `ac` - Stop data recording (via ROS2 service)
- `am` - Monitor mode for lifecycle management
- `ag` - Launch Argo GUI
- `argo_status` - Detailed system status check
- `argo_help` - Show detailed help information

## Running the System

### Manual Launch (Recommended for Testing)
```bash
cd /home/orangepi/argo
source /opt/ros/humble/setup.bash
ros2 launch launch/argo_launch.py
```

### Individual Node Testing
```bash
# Test individual sensors with debug output
ros2 run argo gps.py --debug
ros2 run argo imu.py --debug
ros2 run argo anem.py --debug
ros2 run argo battery_water.py --debug
ros2 run argo pwm.py
ros2 run argo control.py
```

### System Monitoring
```bash
# Monitor ROS2 topics
ros2 topic list
ros2 topic echo /battery_voltage
ros2 topic echo /anem_speed_angle_temp
ros2 topic echo /rudder_sail_radio
```

## Key ROS2 Nodes

- **`gps.py`**: GPS interface via UART5, publishes `/gps_data`
- **`imu.py`**: 9-DOF IMU data, publishes `/accel`, `/gyro`, `/compass`
- **`anem.py`**: Wind speed/direction from 3 pressure sensors, publishes `/anem_speed_angle_temp`
- **`battery_water.py`**: Power and safety monitoring, publishes battery/water alerts
- **`pwm.py`**: Radio control interface and servo output
- **`control.py`**: Autonomous navigation controller

## Configuration Files

- **`argo.yaml`**: Main control parameters (mode, gains, etc.)
- **`argo.env`**: ROS2 environment variables for systemd services
- **`RTIMULib.ini`**: IMU calibration and sensor fusion settings

## Safety Features

- **Manual Override**: Human can take control via radio at any time
- **Battery Monitoring**: Automatic low battery alerts (7.2V threshold)
- **Water Intrusion Detection**: Immediate alerts on water sensor activation
- **Sensor Fault Detection**: Automatic reconnection and error handling
- **Timeout Protection**: Safe defaults if communication is lost

## Data Analysis

To plot recorded bag file data, see [argo-plots.py](develop/analysis/argo-plots.py)

## Troubleshooting

### Common Issues
1. **I2C Permission Errors**: Ensure user is in `i2c` group
2. **UART Permission Errors**: Ensure user is in `dialout` group  
3. **PWM Module Not Loading**: Check device tree overlay installation
4. **Sensor Not Found**: Verify I2C connections and addresses
5. **GPS No Data**: Check UART5 overlay and baud rate settings

### Debug Commands
```bash
# Check system health
sudo i2cdetect -y 0
ros2 topic list
ros2 node list
systemctl status argo-launch.service
```

## 2025 Development Updates

**Exciting developments are underway for 2025:**

- **Custom PCB Design**: A dedicated PCB is being designed to integrate all sensors and control systems into a single, robust board
- **Enhanced Seaworthiness**: Significant improvements to waterproofing, structural integrity, and marine-grade components for extended autonomous operation
- **Production Readiness**: Moving from prototype to production-ready autonomous sailboat system

## Documentation

For detailed technical documentation, see the [Google Doc README](https://docs.google.com/document/d/1k4FYVaFQ-n34UVE_fHvFsXmN1hixLR1Vu4WXfNnHY-0/edit?usp=sharing)

## License

BSD License - See package.xml for details