# Argo Autonomous Sailboat

An autonomous sailboat system based on Dragonforce 65 hull, running on Orange Pi Zero 2W with ROS2. The system includes comprehensive sensor integration, autonomous navigation, and safety monitoring.

## News
- **Feb 2026:** Updated [wind sensor design PCB and housing](Windsensor-rev1/) complete. 3D parts for wind sensor I2C cable and mast step up completed and printed.
- **Jan 2026:** [Argo main PCB version 3](pcb/argo-v9-stable/argo-v9-stable-rev3/) completed and seems fully functional with no surgery needed.
- **Aug-Dec 2025:** Intense development of new [Argo PCB](pcb/) and [ROS2 software framework](nodes/). System services developed (see [`power_control/argo_power_control.py`](power_control/argo_power_control.py) and [`launch/argo_lifecycle_manager.py`](launch/argo_lifecycle_manager.py)) along with robust [lifecycle management](launch/argo_lifecycle_manager.py) and [battery/power control](power_control/argo_power_control.py).
- **Oct-Nov 2025:** Developed first version of [Argo simulator](simulator/).

## System Overview

The Argo system consists of multiple ROS2 nodes that work together to provide autonomous sailing capabilities:

- **Sensor Nodes**: GPS (u-blox NEO-M9N), IMU ([Adafruit BNO085](https://www.adafruit.com/product/4754)), Wind sensors (3x Sensirion SDP3x), Battery/Water monitoring
- **Control Interface**: PWM capture for radio control and servo output  
- **Autonomous Control**: Navigation and sail trimming algorithms
- **Safety Systems**: Manual override, battery monitoring, water intrusion detection

### Demo Video
See the Argo autonomous sailboat in action at the 2024 CCNW (before current waterproofing and PCB developements): [Argo Sailboat Demo](https://youtu.be/tjC1262BsCY?si=1GFPk1QcOqpzw8h2)

![Argo Autonomous Sailboat](https://img.youtube.com/vi/tjC1262BsCY/maxresdefault.jpg)

## Hardware Platform
The main PCB design for Argo is available in the [pcb/](pcb/) folder. The latest board version is [rev3](pcb/argo-v9-stable/argo-v9-stable-rev3/), which contains the current sailboat control and sensor interface design.

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **GPS**: u-blox NEO-M9N via UART5 (/dev/ttyS5)
- **IMU**: [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU via I2C0 (0x4a)
- **Wind Sensor**: 3x Sensirion SDP3x differential pressure sensors via I2C0 (0x21, 0x22, 0x23). This PCB includes an RGBW LED and its I2C controller chip LP5814DLR at I2C address 0x2C.  
  - **Design files:** See [`Windsensor-rev1/`](Windsensor-rev1/) directory for full hardware schematics, layout, and BOM.
- **ADC**: MAX11612 for battery/water sensing via I2C0 (0x34)
- **Environment**: SHT45 temperature/humidity via I2C0 (0x44)
- **PWM I/O**: Custom kernel module for radio control and servo interfaces with high impedance safety mode
- **LORA**: Long range radio based on RA-01 radio module from ai-thinker.com using SX1276 LORA chip

### Hardware Replacement and MAC Address Cloning

The Argo system is designed to handle frequent SBC hardware replacements (due to damage) while maintaining network identity. The WiFi MAC address is frozen to a consistent value (`c8:26:e2:6c:58:ba`) stored in `network/ARGO_MAC_ID.txt` and committed to git. This ensures:

- **Network Stability**: Same MAC address across hardware changes maintains network access (uzh-iot, ZeroTier, tobi-wlan)
- **SD Card Portability**: Same SD card works seamlessly with different SBC hardware
- **Automatic Configuration**: Run `make freeze-mac-address` after hardware replacement to restore network identity

See [Network Improvements Documentation](network/README.md#mac-address-cloning-hardware-replacement) for details.

### Servo Control Hardware

The Argo system uses a shared control circuit design that allows both radio control and autonomous software control of servos through a fail-safe high impedance switching mechanism:

#### Stock DF65 V6 (2018) Servo Configuration
- **Sail Winch Servo**: Joysway #880545
  - Type: Standard sail winch servo with high-precision plastic gears
  - Operating voltage: 4.8V to 6.0V
  - PWM frequency: 1520μs/50Hz
  - **Input impedance: 12kΩ** (measured)
  - **Circuit resistor: 3.3kΩ** (optimized for proper signal level)

- **Rudder Servo**: Joysway #881504  
  - Type: Digital metal gear servo
  - Operating voltage: 4.8V to 6.0V
  - PWM frequency: 1520μs/330Hz
  - **Input impedance: 400kΩ** (measured)
  - **Circuit resistor: 10kΩ** (standard value works well)

#### Shared Control Circuit Design
The hardware implements a fail-safe shared control circuit where:

1. **High Impedance Safety Mode**: When PWM outputs are disabled (high impedance), radio control signals pass through resistors directly to servo inputs
2. **Software Control Mode**: When PWM outputs are enabled, software generates servo control signals
3. **Voltage Divider Considerations**: Resistor values are optimized based on servo input impedance:
   - Sail servo: 3.3V × (12kΩ / (3.3kΩ + 12kΩ)) = 2.6V (adequate signal level)
   - Rudder servo: 3.3V × (400kΩ / (10kΩ + 400kΩ)) = 3.2V (excellent signal level)

#### GPIO Pin Assignments (Orange Pi Zero 2W)
- **PI11 (Pin 7)**: Radio Rudder Input - GPIO input with interrupt capability
- **PI13 (Pin 26)**: Radio Sail Input - GPIO input with interrupt capability  
- **PI12 (Pin 33)**: Servo Rudder Output - PWM2 channel, high impedance when disabled
- **PI14 (Pin 16)**: Servo Sail Output - PWM4 channel, high impedance when disabled

This design ensures **radio control always works** when software is not running, providing maximum safety for autonomous sailboat operation.

## ROS2 Software Architecture

### Project Organization

The Argo system follows a modular ROS2 architecture with clear separation of concerns:

**Directory Structure:**
- **`nodes/`** - Individual ROS2 nodes (Python-based sensor interfaces)
- **`launch/`** - Lifecycle management, systemd services, and launch configurations
- **`power_control/`** - Power management system (separate ROS2 package)
- **`foxglove/`** - Visualization layouts for Foxglove Studio
- **`simulator/`** - Sailboat simulation submodule (sailboat-playground)

## Other READMEs

### Core System Documentation
- [**Systemd Services Architecture**](docs/README-services.md) - Service dependencies, boot sequence, and troubleshooting
- [**Network Improvements**](network/README.md) - WiFi reconnection, MAC address cloning, and NetworkManager configuration
- [**ZeroTier VPN Setup**](docs/README_zerotier.md) - ZeroTier VPN configuration, SSH access, and network management
- [**Power Control System**](power_control/README.md) - Power button, LED control, and graceful shutdown
- [**Systemd Integration**](launch/README.md) - Launch system and lifecycle management
- [**Shutdown Fixes**](docs/SHUTDOWN_FIXES.md) - Shutdown procedure fixes and improvements
- [**LED Status Guide**](docs/ARGO_LED_STATUS_GUIDE.md) - LED status display and hatch cover indicators

### Development & Simulation
- [**Simulation System Overview**](simulator/README.md) - Sailboat simulation framework
- [**Simulation Documentation**](docs/SIMULATION.md) - Simulation setup and usage
- [**Debug Simulation**](docs/README-debug-simulation.md) - Debugging simulation issues

### Hardware & Configuration
- [**I2C Configuration**](docs/README-i2c.md) - I2C bus setup and troubleshooting
- [**Watchdog Configuration**](docs/README-watchdog.md) - Watchdog setup for Orange Pi Zero 2W
- [**CPU Frequency Tuning**](docs/README-cpufreq-tuning-service.md) - CPU governor configuration
- [**Battery & Power Monitoring**](docs/README-battery-power.md) - Battery monitoring system
- [**Battery Panel Setup**](docs/BATTERY_PANEL_SETUP.md) - Battery panel configuration
- [**SD Card Backup**](docs/README-sd-card-backup.md) - SD card backup procedures
- [**DVFS Userspace Access**](docs/dvfs_userspace_access.md) - Dynamic voltage and frequency scaling

### Visualization & Analysis
- [**Foxglove Visualization**](foxglove/README.md) - Real-time visualization setup
- [**Foxglove Debugging**](docs/FOXGLOVE_DEBUGGING.md) - Debugging Foxglove issues
- [**Foxglove Joystick Setup**](docs/FOXGLOVE_JOYSTICK_SETUP.md) - Joystick control configuration
- [**3D Visualization**](docs/README-3d-visualization.md) - 3D visualization configuration
- [**Visualization Marker Persistence**](docs/VISUALIZATION_MARKER_PERSISTENCE.md) - Marker persistence in visualization
- [**Adding Sailing Maps**](docs/README-adding-sailing-maps.md) - Map integration guide
- [**Maps Documentation**](maps/README-maps.md) - Sailing area maps

### Remote Access & Interface
- [**Remote Desktop VNC**](docs/README_REMOTE_DESKTOP_VNC.md) - VNC remote desktop setup
- [**Web Dashboard**](docs/WEB_DASHBOARD_README.md) - Mobile-friendly web interface

### CLI & User Interface
- [**CLI Completion**](docs/CLI_COMPLETION.md) - Command-line completion setup
- [**Completion Quickstart**](docs/COMPLETION_QUICKSTART.md) - Quick start guide for CLI completion
- [**MOTD Customization**](docs/MOTD_CUSTOMIZATION.md) - Message of the day customization

### Additional Resources
- [**ROS2 Nodes**](nodes/README.md) - Node documentation and configuration
- [**Scripts**](scripts/README.md) - Utility scripts documentation
- [**Dotfiles Setup**](dotfiles/README.md) - CLI aliases and shell configuration
- [**Persistent Logging**](docs/README-persistent-logging.md) - Logging configuration


### Simulation Support

Argo includes comprehensive simulation support for development and testing:

**Local Simulation:**
- Runs simulator directly on Orange Pi
- Uses sailboat-playground or mock simulator
- Excludes conflicting hardware nodes (GPS, IMU, anemometer, radio control)
- Perfect for algorithm development and testing

**Remote Simulation:**
- Offloads CPU-intensive simulation to remote machine
- SSH tunnel for ROS2 communication
- Centralized configuration management
- Ideal for resource-constrained environments

**Quick Start:**
```bash
# Local simulation
python3 launch/argo_lifecycle_manager.py simulate_local

# Remote simulation (requires setup)
python3 launch/argo_lifecycle_manager.py simulate_remote

# Or use helper scripts (equivalent flow):
./scripts/remote_simulator_tunnel.sh &
python3 scripts/remote_simulator_launch.py &
./scripts/launch_simulator_remote.sh
```

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
- **Sensor Nodes**: `gps.py`, `imu.py`, `anem.py`, `argo_battery_water.py`, `temp_monitor.py`
- **Control Nodes**: `rudder_sail_radio.py`, `controller.py`, `record.py`
- **`pwm_capture_module/`** - Custom kernel module for radio control and servo interfaces
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

### `system-monitoring/` - Optional System Monitoring Services
Optional system-level monitoring services for debugging and development:
- **`Makefile`** - Installation and management of system monitoring services
- **`services/`** - Systemd service files for system monitoring
- **`scripts/`** - Monitoring scripts for boot history, memory, processes, etc.
- **Not installed by default** - Use `make install-system-monitoring` to install

**Core ROS2 Nodes:**
- **`gps.py`** - GPS receiver interface (UART5, u-blox NEO-M9N)
- **`bno085.py`** - 9-DOF Orientation IMU with sensor fusion (I2C, [Adafruit BNO085](https://www.adafruit.com/product/4754))
- **`anem.py`** - Wind sensor array (3x SDP3x pressure sensors)
- **`argo_battery_water.py`** - Power monitoring and safety systems
- **`rudder_sail_radio.py`** - Radio control input and servo output interface
- **`controller.py`** - Autonomous navigation and sail control algorithms
- **`record.py`** - Data recording management (ROS2 bag files)
- **`temp_monitor.py`** - System temperature monitoring
- **`argo_unified_simulator_bridge.py`** - Unified simulator bridge for local and remote simulation

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
python3 launch/argo_lifecycle_manager.py run             # Launch all nodes
python3 launch/argo_lifecycle_manager.py stop            # Graceful shutdown
python3 launch/argo_lifecycle_manager.py restart         # Restart all nodes
python3 launch/argo_lifecycle_manager.py status          # Show system status
python3 launch/argo_lifecycle_manager.py monitor         # Continuous monitoring
python3 launch/argo_lifecycle_manager.py simulate_local  # Local simulation mode
python3 launch/argo_lifecycle_manager.py simulate_remote # Remote simulation mode
```

**Critical Node Management:**
- **Critical Nodes**: `rudder_sail_radio.py`, `controller.py` (essential for boat operation)
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
# Install Python runtime dependencies
make install-python-deps  # installs from requirements.txt

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
cd ~/argo/nodes/pwm_capture_module
make all
# Reboot or load the module manually after install
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
# Check I2C devices (should show: 21 22 23 34 44 4a)
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
- `as` - Show Status
Shows a comprehensive status of the Argo system, including:
- Service status (`argo_launch_standard.service`, `argo_power_control.service`, etc.)
- Running ROS2 nodes and their health
- System diagnostics (CPU, memory, temperature, battery)

### `al` / `aq` / `ars` - Service Control
- `al`: Start the `argo_launch_standard.service`
- `aq`: Stop the `argo_launch_standard.service`
- `ars`: Restart the `argo_launch_standard.service`

### `alog` - View Logs
Tails the logs for all major Argo services with color-coding.

### `asim` - Local Simulation
Starts the Argo system in local simulation mode using `ros2 launch`.

- `ar` - Start data recording (via ROS2 service)
- `ac` - Stop data recording (via ROS2 service)
- `am` - Monitor mode for lifecycle management
- `ag` - Launch Argo GUI
- `argo_status` - Detailed system status check
- `argo_help` - Show detailed help information

## Running the System

### Manual Launch (Recommended for Testing)
```bash
cd ~/argo
source /opt/ros/humble/setup.bash
python3 launch/argo_lifecycle_manager.py run
```

### Individual Node Testing
```bash
# Test individual sensors with debug output
python3 nodes/gps.py --debug
python3 nodes/bno085.py bridge  # IMU bridge mode
python3 nodes/bno085.py status  # Check IMU health
python3 nodes/anem.py --debug
python3 nodes/argo_battery_water.py --debug
python3 nodes/rudder_sail_radio.py
python3 nodes/controller.py
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
- **`bno085.py`**: [Adafruit BNO085](https://www.adafruit.com/product/4754) IMU with sensor fusion, publishes `/compass`, `/pose`, `/accel`, `/gyro`, `/imu_health`
- **`anem.py`**: Wind speed/direction from 3 pressure sensors, publishes `/anem_speed_angle_temp`
- **`argo_battery_water.py`**: Power and safety monitoring, publishes battery/water alerts
- **`rudder_sail_radio.py`**: Radio control interface and servo output
- **`controller.py`**: Autonomous navigation controller
- **`argo_unified_simulator_bridge.py`**: Unified simulator bridge for local and remote simulation
- **`lora.py`**: Long range radio node for LORA module

## Configuration Files

- **`argo.yaml`**: Main control parameters (mode, gains, etc.)
- **`argo.env`**: ROS2 environment variables for systemd services
- **`RTIMULib.ini`**: IMU calibration and sensor fusion settings

## Safety Features

- **High Impedance Safety Mode**: Servo outputs default to high impedance (PWM disabled), allowing radio control to pass through directly to servos via resistor network
- **Manual Override**: Human can take control via radio at any time, with immediate priority over autonomous control
- **Shared Control Circuit**: Fail-safe design where radio control always works when software is not running
- **Battery Monitoring**: Automatic low battery alerts (7.2V threshold)
- **Water Intrusion Detection**: Immediate alerts on water sensor activation
- **Sensor Fault Detection**: Automatic reconnection and error handling
- **Timeout Protection**: Safe defaults if communication is lost

## Data Analysis

To plot recorded bag file data, see [argo-plots-2025.py](develop/analysis/argo-plots-2025.py) (or [argo-plots-2023.py](develop/analysis/argo-plots-2023.py)).

## Troubleshooting

### Common Issues
1. **I2C Permission Errors**: Ensure user is in `i2c` group
2. **UART Permission Errors**: Ensure user is in `dialout` group  
3. **PWM Module Not Loading**: Check device tree overlay installation
4. **Sensor Not Found**: Verify I2C connections and addresses
5. **GPS No Data**: Check UART5 overlay and baud rate settings
6. **Servo Low Signal Levels**: Check servo input impedance and adjust resistor values accordingly:
   - High-power servos (e.g., winch servos) may have low input impedance (12kΩ) requiring smaller resistors (3.3kΩ)
   - Standard digital servos typically have high input impedance (400kΩ) working well with 10kΩ resistors

### Debug Commands
```bash
# Check system health
ah # show aliases
adevcheck # check for argo devices present and communicating
alog # follow argo logs - see help options
sudo i2cdetect -y 0 # check i2c bus
ros2 topic list
ros2 node list # check nodes
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
