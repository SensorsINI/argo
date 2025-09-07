# Argo Autonomous Sailboat

An autonomous sailboat system based on Dragonforce 65 hull, running on Orange Pi Zero 2W with ROS2. The system includes comprehensive sensor integration, autonomous navigation, and safety monitoring.

## System Overview

The Argo system consists of multiple ROS2 nodes that work together to provide autonomous sailing capabilities:

- **Sensor Nodes**: GPS (u-blox NEO-M9N), IMU (ICM-20948), Wind sensors (3x Sensirion SDP3x), Battery/Water monitoring
- **Control Interface**: PWM capture for radio control and servo output  
- **Autonomous Control**: Navigation and sail trimming algorithms
- **Safety Systems**: Manual override, battery monitoring, water intrusion detection

## Hardware Platform

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **GPS**: u-blox NEO-M9N via UART5 (/dev/ttyS5)
- **IMU**: ICM-20948 9-DOF via I2C0 (0x69)
- **Wind Sensor**: 3x Sensirion SDP3x differential pressure sensors via I2C0 (0x21, 0x22, 0x23)
- **ADC**: MAX11612 for battery/water sensing via I2C0 (0x34)
- **Environment**: SHT45 temperature/humidity via I2C0 (0x44)
- **PWM I/O**: Custom kernel module for radio control and servo interfaces

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

### 7. Configure System Services (Optional)
```bash
# Copy systemd service files
sudo cp /home/orangepi/argo/launch/argo-launch.service /etc/systemd/system/
sudo cp /home/orangepi/argo/launch/argo-record.service /etc/systemd/system/

# Enable services (uncomment WantedBy lines in service files first)
sudo systemctl daemon-reload
sudo systemctl enable argo-launch.service
sudo systemctl enable argo-record.service
```

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

## Documentation

For detailed technical documentation, see the [Google Doc README](https://docs.google.com/document/d/1k4FYVaFQ-n34UVE_fHvFsXmN1hixLR1Vu4WXfNnHY-0/edit?usp=sharing)

## License

BSD License - See package.xml for details