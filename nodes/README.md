# Argo Autonomous Sailboat - ROS2 Scripts

This directory contains the ROS2 nodes and scripts for the Argo autonomous sailboat system running on Orange Pi Zero 2W.

## System Overview

The Argo system consists of multiple sensor and control nodes that work together to provide autonomous sailing capabilities:

- **Sensor Nodes**: GPS, IMU, Wind (Anemometer), Battery/Water monitoring
- **Control Interface**: PWM capture for radio control and servo output  
- **Autonomous Control**: Navigation and sail trimming algorithms
- **Safety Systems**: Manual override, battery monitoring, water intrusion detection

## Hardware Platform

- **Orange Pi Zero 2W** (Allwinner H618 SoC)
- **GPS**: u-blox NEO-M9N via UART5 (/dev/ttyS5)
- **IMU**: [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU via I2C0 (0x4a)
- **Wind Sensor**: 3x Sensirion SDP3x differential pressure sensors via I2C0 (0x21, 0x22, 0x23)
- **ADC**: MAX11612 for battery/water sensing via I2C0 (0x34)
- **Environment**: SHT45 temperature/humidity via I2C0 (0x44)
- **PWM I/O**: Custom kernel module for radio control and servo interfaces

### I2C Bus Configuration
**Exclusive I2C Bus 0 Usage**: All sensor nodes exclusively use I2C bus 0 (Orange Pi Zero 2W default I2C interface)
- **I2C Bus 0 Pins**: SDA=PI6 (twi0-sda), SCL=PI5 (twi0-sck)
- **Configuration**: Enabled via `pi-i2c0` overlay in `/boot/orangepiEnv.txt`
- **Address Map**:
  - 0x21, 0x22, 0x23: Wind sensors (SDP3x differential pressure)
  - 0x34: MAX11612 ADC (battery/water monitoring)
  - 0x44: SHT45 temperature/humidity sensor
  - 0x4a: BNO085 IMU (9-DOF Orientation)
- **Bus Speed**: Standard 100kHz I2C operation
- **Power**: 3.3V logic levels, pull-up resistors on PCB

## ROS2 Nodes

### Core Navigation Nodes

#### 1. `gps.py` - GPS Interface Node
**Purpose**: Interfaces with u-blox NEO-M9N GPS receiver over UART5

**Hardware**: 
- UART5 pins: TX=11 (PH2), RX=13 (PH3)
- Enabled via `ph-uart5` overlay in orangepiEnv.txt
- Default baud rate: 38400

**Topics Published**:
- `/gps_data` (String) - Raw NMEA sentences

**Parameters**:
- `serial_port`: /dev/ttyS5 (default)
- `baud_rate`: 38400 (default)
- `gps_frame_id`: argo_gps (default)

**Usage**:
```bash
ros2 run argo gps.py
# With debug output:
ros2 run argo gps.py --debug
```

#### 2. `bno085.py` - Inertial Measurement Unit Node
**Purpose**: Interfaces with [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU for sensor fusion and motion data

**⚠️ IMPORTANT**: The BNO085 uses a **unique two-process architecture** (C++ driver + Python bridge) that requires systemd service management, unlike other sensor nodes. See **[BNO085_README.md](BNO085_README.md)** for complete documentation.

**Hardware**: I2C0 address 0x4a (via C++ `bno08x_driver` with Python bridge)

**Architecture**: 
- **C++ Driver** (`bno08x_driver`): Direct I2C hardware access
- **Python Bridge** (`bno085.py`): Converts to Argo topics
- **Systemd Service** (`argo_bno085.service`): **Required** for production use
- **I2C Recovery**: Automatic service restart on sensor failures

**Topics Published**:
- `/compass` (Vector3) - Magnetic field heading in µT (x, y, z=heading_deg)
- `/pose` (Vector3) - IMU orientation (x=roll, y=pitch, z=yaw in degrees)
- `/accel` (Vector3) - Linear acceleration in m/s² (x, y, z)
- `/gyro` (Vector3) - Angular velocity in rad/s (x, y, z)
- `/imu_health` (Bool) - Health status (true=healthy, false=unhealthy)

**Features**:
- **Sensor Fusion**: CEVA Hillcrest Labs SH-2 firmware for accurate orientation
- **Rotation Vector Output**: Quaternion-based absolute orientation (magnetic north reference)
- **Automatic Calibration**: On-board magnetometer, accelerometer, and gyroscope calibration
- **Health Monitoring**: Auto-recovery with 3-second data timeout detection
- **I2C Error Recovery**: Automatic systemd service restart on failures
- **Multiple Modes**: Bridge, calibration, verification, and status checking

**Quick Start** (see [BNO085_README.md](BNO085_README.md) for details):
```bash
# Install systemd service (first time)
cd /home/orangepi/argo/nodes
make bno085-service-install

# Service management
make bno085-service-status
make bno085-service-restart
make bno085-service-logs

# Calibration and verification
python3 bno085.py calibrate --duration 60
python3 bno085.py verify

# Check system status
python3 bno085.py status
```

**Documentation**: See **[BNO085_README.md](BNO085_README.md)** for:
- Systemd service architecture and setup
- I2C error recovery mechanisms
- Calibration procedures
- Troubleshooting guide
- Comparison with direct I2C sensor nodes

#### 3. `anem.py` - Wind Sensor Node
**Purpose**: Calculates wind speed and direction using 3 differential pressure sensors

**Hardware**: 
- I2C0 addresses 0x21 (CCW), 0x22 (Center), 0x23 (CW)
- Based on Sensirion SDP3x directional wind meter design

**Algorithm**: Uses pressure differentials across directional ports to compute wind vector

**Topics Published**:
- `/anem_speed_angle_temp` (Vector3):
  - x: wind speed (m/s)
  - y: wind angle (degrees CW from boat front)
  - z: average temperature (°C)
- `/anem_diffpressure` (Vector3):
  - x: CCW sensor pressure (Pa)
  - y: Center sensor pressure (Pa)
  - z: CW sensor pressure (Pa)

**Usage**:
```bash
ros2 run argo anem.py
# With debug output:
ros2 run argo anem.py --debug
# With visual wind vector display:
ros2 run argo anem.py --debug_visually
```

### System Monitoring Nodes

#### 4. `battery_water.py` - Power and Safety Monitoring Node
**Purpose**: Monitors battery voltage, water intrusion, and environmental conditions

**Hardware**:
- MAX11612 ADC (I2C0 0x34): AIN0=battery, AIN1=water probe, AIN2=sail current
- SHT45 sensor (I2C0 0x44): temperature and humidity

**Topics Published**:
- `battery_voltage` (Float32) - Battery voltage (V)
- `saltwater_voltage` (Float32) - Water probe voltage (V)  
- `sail_current` (Float32) - Sail winch current (A)
- `temperature` (Float32) - Temperature (°C)
- `relative_humidity` (Float32) - Relative humidity (%)
- `battery_remaining_pct` (Float32) - Battery state of charge (%)
- `battery_low_alert` (Bool) - Low battery warning
- `saltwater_alert` (Bool) - Water intrusion detection
- `humidity_alert` (Bool) - High humidity warning

**Key Parameters**:
- `battery_low_threshold_v`: 7.2V (default)
- `saltwater_alert_threshold_v`: 1.0V (default)
- `humidity_alert_threshold_pct`: 75.0% (default)
- `battery_series_cells`: 2 (for 2S LiPo, default)

**Usage**:
```bash
ros2 run argo battery_water.py
# With debug terminal bars:
ros2 run argo battery_water.py --debug
# ADC testing mode:
ros2 run argo battery_water.py --test-adc
```

### Control and Interface Nodes

#### 5. `pwm.py` - Radio Control and Servo Interface Node
**Purpose**: Interfaces with custom kernel module for PWM capture and servo control

**Hardware Dependencies**: 
- `argo_radio_servo_module` kernel module (see pwm_capture_module/)
- Sysfs interface at `/sys/kernel/argo_radio_servo/`

**Topics Published**:
- `/rudder_sail_radio` (Vector3) - Radio control inputs (-1 to +1)
  - x: rudder position (-1=left, +1=right)
  - y: sail position (-1=in, +1=out)
- `/rudder_sail_servo` (Vector3) - Actual servo commands sent
- `/human_controlled` (Bool) - True when human has control override

**Topics Subscribed**:
- `/rudder_sail_cmd` (Vector3) - Autonomous control commands

**Features**:
- Automatic human/computer control switching
- Pulse width validation (1000-2000µs)
- Real-time servo command conversion
- Safety timeouts and neutral positioning

**Configuration**:
- `HUMAN_CONTROL_TIMEOUT_S`: 2.0 seconds
- `HUMAN_CONTROL_THRESHOLD`: 0.2 (normalized rudder deviation)

**Usage**:
```bash
ros2 run argo pwm.py
```

#### 6. `control.py` - Autonomous Navigation Controller
**Purpose**: High-level autonomous sailing control logic

**Topics Subscribed**:
- `/rudder_sail_radio` - Radio control inputs
- `/human_controlled` - Manual override status
- `/pose` - IMU orientation data (when available)

**Topics Published**:
- `/rudder_sail_cmd` (Vector3) - Autonomous control commands

**Features**:
- Parameter file hot-reloading (`argo.yaml`)
- Multiple control modes (autonomous, manual override)
- Compass-based heading control
- Configurable PID gains and limits

**Parameters** (loaded from `argo.yaml`):
- `/argo/mode`: autonomous/manual
- `/argo/rudder_gain`: Control gain multiplier

**Usage**:
```bash
ros2 run argo control.py
```

## Utility Scripts

### `load_or_reload_params.py`
Utility for dynamically loading ROS2 parameters from YAML files

### `humidity.py` 
Standalone SHT45 humidity sensor testing script

### `imu-cal.py`
IMU calibration utility for magnetometer compass calibration

### `gpio_control_example.py`
Example GPIO control script for testing hardware interfaces

### `logskip.py`
Log filtering utility for reducing verbose ROS2 output

## Configuration Files

### `argo.yaml`
Main configuration file for autonomous control parameters:
```yaml
/argo/mode: autonomous
/argo/rudder_gain: 2
```

### `RTIMULib.ini`
IMU library configuration for sensor fusion and calibration

### `invensense-20948-compass-calibration.json`
Magnetometer calibration data for compass accuracy

## Launch and Deployment

### ROS2 Launch (Recommended)
The system is designed for ROS2 but launch files are being migrated from ROS1. Current manual startup:

```bash
# Start individual nodes
ros2 run argo gps.py &
ros2 run argo imu.py &
ros2 run argo anem.py &
ros2 run argo battery_water.py &
ros2 run argo pwm.py &
ros2 run argo control.py &
```

### Legacy ROS1 Launch Files
The `../launch/` directory contains ROS1 launch files that are being migrated:
- `argo.launch` - Main system launch file (ROS1 format)
- `argo-launch.service` - Systemd service for automatic startup

## Dependencies

### System Packages
```bash
sudo apt update
sudo apt install python3-pip python3-dev build-essential
sudo apt install i2c-tools device-tree-compiler
sudo apt install ros-humble-desktop  # ROS2 Humble
```

### Python Dependencies
```bash
pip3 install -r requirements.txt
```

Key packages:
- `rclpy` - ROS2 Python client library
- `smbus2` - I2C communication
- `pyserial` - Serial communication
- `numpy` - Numerical computations
- `tqdm` - Progress bars (optional)
- `matplotlib` - Plotting (optional)

### Hardware Setup
1. **Enable I2C and UART overlays** in `/boot/orangepiEnv.txt`:
   ```
   overlays=pi-i2c0 disable-uart0 ph-uart5 pi-pwm2 pi-pwm4
   user_overlays=argo_radio_servo_overlay
   ```
   Note: `pi-i2c0` overlay configures I2C bus 0 on pins SDA=PI6 (twi0-sda), SCL=PI5 (twi0-sck)

2. **Install PWM capture module** (see `pwm_capture_module/README.md`)

3. **I2C device verification**:
   ```bash
   sudo i2cdetect -y 0
   # Should show: 21 22 23 (wind), 34 (ADC), 44 (humidity), 4a (BNO085 IMU)
   # All sensors are on I2C bus 0 exclusively
   ```

## Debugging and Testing

### Individual Node Testing
Each node can be run standalone with debug output:
```bash
ros2 run argo <node_name>.py --debug
```

### System Health Check
```bash
# Check I2C devices on bus 0 (all sensors use this bus exclusively)
sudo i2cdetect -y 0
# Expected devices: 21 22 23 (wind), 34 (ADC), 44 (humidity), 4a (BNO085 IMU)

# Check PWM kernel module
lsmod | grep argo
ls -la /sys/kernel/argo_radio_servo/

# Check UART GPS
sudo cat /dev/ttyS5

# Monitor ROS2 topics
ros2 topic list
ros2 topic echo /battery_voltage
```

### Common Issues

1. **I2C Permission Errors**: Add user to i2c group:
   ```bash
   sudo usermod -a -G i2c $USER
   ```

2. **UART Permission Errors**: Add user to dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```

3. **PWM Module Not Loading**: Check device tree overlay installation
4. **Sensor Not Found**: Verify I2C connections and addresses
5. **GPS No Data**: Check UART5 overlay and baud rate settings

## Development Workflow

1. **Test individual sensors** using standalone debug modes
2. **Verify ROS2 topics** are publishing correctly
3. **Check parameter loading** from argo.yaml
4. **Monitor system health** via battery and environmental sensors
5. **Test manual override** functionality via radio control
6. **Validate autonomous control** in safe test environment

## Safety Features

- **Manual override**: Human can take control via radio at any time
- **Battery monitoring**: Automatic low battery alerts
- **Water intrusion detection**: Immediate alerts on water sensor activation
- **Sensor fault detection**: Automatic reconnection and error handling
- **Timeout protection**: Safe defaults if communication is lost
- **Pulse width validation**: Servo commands limited to safe ranges

## Future Enhancements

- **ROS2 launch files** to replace ROS1 legacy files
- **Web dashboard** for remote monitoring and control
- **Data logging** with automatic bag file recording
- **Advanced path planning** with GPS waypoint navigation
- **Machine learning** for optimal sail trimming
- **Telemetry system** for remote fleet management

This system provides a robust foundation for autonomous sailboat operation with comprehensive sensor integration, safety systems, and manual override capabilities.

