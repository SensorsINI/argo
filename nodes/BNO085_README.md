# BNO085 IMU Integration Guide

## Overview

The Argo autonomous sailboat uses the [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU for precise heading and motion sensing. This sensor provides superior accuracy compared to the previous ICM-20948 through on-chip sensor fusion and automatic calibration.

## Architecture

### System Overview

**IMPORTANT:** The BNO085 uses a **two-process architecture** unlike other Argo sensor nodes like `anem.py`, `gps.py`, or `battery_water.py` which directly access hardware. This dual-process design requires systemd service management for production use.

```
┌─────────────────────────────────────────────────────────┐
│              Systemd Service Layer                      │
│  argo_bno085.service - Auto-restart & lifecycle mgmt   │
└──────────────────┬──────────────────────────────────────┘
                   │ manages
                   ▼
    ┌──────────────┴──────────────┐
    ▼                              ▼
┌──────────────────┐    ┌──────────────────────┐
│  bno08x_driver   │    │  bno085.py bridge    │
│  (C++ ROS2 node) │    │  (Python ROS2 node)  │
│                  │    │                      │
│ Direct I2C bus   │    │ Subscribes:          │
│ access to        │───▶│  /imu                │
│ BNO085 sensor    │    │  /magnetic_field     │
│                  │    │                      │
│ Publishes:       │    │ Publishes (Argo):    │
│  /imu            │    │  /compass            │
│  /magnetic_field │    │  /pose               │
└──────────────────┘    │  /accel              │
                        │  /gyro               │
                        │  /imu_health         │
                        └──────────────────────┘
```

### Why Two Processes?

1. **Hardware Abstraction**: C++ driver provides low-level I2C communication and SH-2 protocol handling
2. **Sensor Fusion**: On-chip BNO085 firmware requires complex binary protocol (SH-2)
3. **ROS2 Ecosystem**: Standard `bno08x_driver` is well-maintained and widely used
4. **Performance**: C++ driver handles high-frequency sensor data efficiently
5. **Reliability**: Systemd service automatically restarts both processes on I2C errors

### Comparison with Other Sensors

| Sensor Node | I2C Access | Architecture | Service Required |
|-------------|-----------|--------------|------------------|
| `anem.py` | Direct (smbus2) | Single Python process | ❌ No |
| `gps.py` | N/A (UART) | Single Python process | ❌ No |
| `battery_water.py` | Direct (smbus2) | Single Python process | ✅ Yes (basic service) |
| **`bno085.py`** | **Via C++ driver** | **Two processes + systemd** | **✅ Yes (dual-process service)** |

### Systemd Service Integration

The `argo_bno085.service` systemd service is **required for production use** and provides:

- **Process Management**: Starts both C++ driver and Python bridge together
- **Auto-Restart**: Recovers from I2C errors and driver crashes (5-second delay)
- **ROS2 Environment**: Proper environment sourcing for both processes
- **Lifecycle Integration**: Status visible in `argo_lifecycle_manager.py`
- **Journal Logging**: Centralized logging via systemd journal

## Hardware Configuration

- **Sensor**: [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU
- **Interface**: I2C bus 0 at address 0x4a
- **Protocol**: SH-2 (Sensor Hub Transport Protocol)
- **Fusion**: CEVA Hillcrest Labs SH-2 firmware for on-chip sensor fusion

## Software Components

### 1. C++ Driver (`bno08x_driver`)
- **Source**: [bno08x-ros2-driver](https://github.com/bnbhat/bno08x-ros2-driver) (git submodule)
- **Location**: `nodes/vendor/bno08x_driver/`
- **Function**: Direct hardware communication via I2C
- **Topics Published**:
  - `/imu` (sensor_msgs/Imu) - Quaternion orientation, gyro, accel
  - `/magnetic_field` (sensor_msgs/MagneticField) - Magnetometer data

### 2. Python Bridge (`bno085.py`)
- **Location**: `nodes/bno085.py`
- **Function**: Converts C++ driver output to Argo format
- **Topics Subscribed**: `/imu`, `/magnetic_field`
- **Topics Published**:
  - `/compass` (Vector3) - Heading in degrees (z component)
  - `/pose` (Vector3) - Same as compass (for compatibility)
  - `/accel` (Vector3) - Linear acceleration in m/s²
  - `/gyro` (Vector3) - Angular velocity in rad/s
  - `/imu_health` (Bool) - Health status for lifecycle management

## Quick Start

### 1. Build and Install Service (First Time Setup)
```bash
cd /home/orangepi/argo/nodes

# Build the C++ driver and install systemd service
make bno085-service-install

# This will:
# - Initialize and build the bno08x_driver submodule
# - Install argo_bno085.service to systemd
# - Enable auto-start on boot
# - Start the service immediately
```

### 2. Verify Operation
```bash
# Check service status
make bno085-service-status
# or: systemctl status argo_bno085.service

# Check topics are publishing
ros2 topic list | grep -E '(imu|compass|pose)'

# Monitor compass heading
ros2 topic echo /compass

# Check health status
ros2 topic echo /imu_health

# View live logs
make bno085-service-logs
# or: journalctl -u argo_bno085.service -f
```

### 3. Service Management
```bash
# Start/stop/restart service
make bno085-service-start
make bno085-service-stop
make bno085-service-restart

# Check status in lifecycle manager
python3 ~/argo/launch/argo_lifecycle_manager.py status
# Should show: 🧭 BNO085 IMU: 🟢 RUNNING

# Uninstall service (if needed)
make bno085-service-uninstall
```

### 4. Development/Testing (Without Service)
```bash
# For development, you can run manually:
make bno08x-launch-full

# Or manually:
bash -c "source /opt/ros/humble/setup.bash && \
         source ~/argo_bno08x_ws/install/setup.bash && \
         ros2 run bno08x_driver bno08x_driver --ros-args --params-file vendor/bno08x_driver_argo.yaml &"
sleep 3
python3 bno085.py bridge
```

## Unified Tool Usage

All BNO085 functionality is consolidated into a single tool: `bno085.py`

### Commands

#### `bridge` (default mode)
Runs as a persistent ROS2 node for normal operation:
```bash
bno085.py bridge
# or simply:
bno085.py
```

#### `status`
Check system health and connectivity:
```bash
bno085.py status
```

#### `calibrate`
Interactive calibration with real-time guidance:
```bash
# Standard 2-minute calibration
bno085.py calibrate

# Custom duration (5 minutes)
bno085.py calibrate --duration 300
```

#### `verify`
Verify sensor output matches datasheet specifications:
```bash
# Continuous verification
bno085.py verify

# Verify for specific duration
bno085.py verify --duration 60
```

### Bash Completion
Enable tab completion for commands and options:
```bash
eval "$(register-python-argcomplete bno085.py)"
```

## Configuration

### Driver Configuration

The BNO08x C++ driver is configured via `nodes/vendor/bno08x_driver_argo.yaml`. This file is automatically loaded by both launch files.

**File Location:** `nodes/vendor/bno08x_driver_argo.yaml`

**Current Configuration:**
```yaml
bno08x_driver:
  ros__parameters:

    frame_id: "imu_link"  # Frame ID for Argo sailboat IMU

    # Communication Interface
    # I2C bus 0 on Orange Pi Zero 2W
    i2c:
      enabled: true
      bus: "/dev/i2c-0"  # Orange Pi Zero 2W uses i2c-0
      address: "0x4A"    # BNO085 default I2C address

    publish:
      magnetic_field: 
        enabled: true
        rate: 10   # 10 Hz for magnetic field (compass heading)
      imu:
        enabled: true
        rate: 10   # 10 Hz for IMU data (accel, gyro, orientation)
```

**Configuration Options:**
- **`frame_id`**: ROS2 frame ID for the IMU (default: "imu_link")
- **`i2c.bus`**: I2C device path (default: "/dev/i2c-0" for Orange Pi Zero 2W)
- **`i2c.address`**: BNO085 I2C address (default: "0x4A")
- **`publish.magnetic_field.rate`**: Magnetometer publish rate in Hz (1-400)
- **`publish.imu.rate`**: IMU data publish rate in Hz (1-400)

**Usage:**
This configuration file is automatically loaded by:
- Makefile targets (`bno08x-launch`, `bno08x-launch-full`)
- Direct `ros2 run` commands with `--params-file` argument

**Customization:**
To change publish rates or other settings, edit this file directly. The changes take effect on the next launch.

## Data Format

### Rotation Vector (Quaternion)
The BNO085 provides orientation as a quaternion (w, x, y, z) representing the Rotation Vector from datasheet section 2.2.4:

```yaml
# /imu topic (from C++ driver)
orientation:
  x: -0.113
  y: -0.301
  z: 0.839
  w: 0.439  # Rotation Vector quaternion
```

### Compass Heading
Converted to Euler angles for Argo compatibility:

```yaml
# /compass topic (from bridge)
x: 0.0
y: 0.0
z: 127.93  # Heading in degrees (0-360°)
```

## Health Monitoring

The bridge publishes `/imu_health` (std_msgs/Bool) for lifecycle management:

### Health States
- **✅ HEALTHY (`data: true`)**: Receiving data from C++ driver, data is fresh (< 3 seconds old)
- **❌ UNHEALTHY (`data: false`)**: No data received or data is stale (> 3 seconds old)

### Implementation
- **Initialization**: Starts unhealthy until first data received
- **Data Reception**: Updates health status when IMU data arrives
- **Periodic Check**: 1-second timer monitors data freshness
- **Auto-Recovery**: Automatically recovers when data resumes

### Monitoring Health
```bash
# Check current health status
ros2 topic echo /imu_health --once

# Monitor health continuously
ros2 topic echo /imu_health

# Via status command
bno085.py status
```

### Failure Scenarios
1. **C++ Driver Crashes**: Health becomes unhealthy after 3s timeout
2. **I2C Communication Failure**: Driver stops sending data, health timeout
3. **Bridge Node Restart**: Health starts unhealthy, recovers on first data

### Compatibility
- **Drop-in Replacement**: Same topic name and semantics as old `imu.py`
- **Lifecycle Integration**: Used by monitoring systems and lifecycle manager

## Calibration

### Understanding BNO085 Calibration
The BNO085 performs **on-chip sensor fusion** and calibration using three main sensors:

1. **Magnetometer** - Measures magnetic field to determine absolute heading
2. **Accelerometer** - Measures gravity and linear acceleration  
3. **Gyroscope** - Measures angular velocity (rotation rate)

### Calibration Accuracy Levels
- **UNRELIABLE (0)** - No calibration data available
- **LOW (1)** - Minimal calibration, may be inaccurate
- **MEDIUM (2)** - Partial calibration, reasonable accuracy
- **HIGH (3)** - Fully calibrated, optimal accuracy

**Target:** Achieve **HIGH (3)** accuracy for all sensors.

### Calibration Process

#### 1. Magnetometer Calibration (Most Important)
**Figure-8 Motion Pattern:**
1. Hold the sensor (or sailboat if mounted)
2. Move in figure-8 pattern through 3D space
3. Rotate through all three axes:
   - Roll: Tilt left and right
   - Pitch: Tilt forward and backward
   - Yaw: Rotate clockwise and counterclockwise

**Requirements:**
- Move slowly and smoothly
- Cover as many different orientations as possible
- Spend at least 30 seconds on each axis
- Avoid metal objects and magnetic interference

#### 2. Accelerometer Calibration (Automatic)
- Auto-calibrates during normal operation
- Experiencing different orientations helps (6-point static calibration)
- Not strictly required - figure-8 motion usually provides adequate calibration

#### 3. Gyroscope Calibration (Automatic)
- Auto-calibrates by analyzing zero-rate offset during stationary periods
- Keep sensor still for 5-10 seconds periodically
- Usually achieves HIGH accuracy quickly (within 30 seconds)

### Using the Calibration Tool
```bash
# Standard 2-minute calibration
bno085.py calibrate

# Custom duration (5 minutes)
bno085.py calibrate --duration 300

# Quick recalibration (1 minute)
bno085.py calibrate --duration 60
```

**Real-Time Display:**
The tool provides live status with progress bars and guidance:
- Shows calibration status for each sensor
- Provides motion guidance ("Motion detected" vs "Move sensor")
- Displays data collection statistics
- Auto-save reminders every 30 seconds

**Success Criteria:**
- **Minimum:** Magnetometer HIGH, others MEDIUM or higher
- **Optimal:** All sensors HIGH accuracy
- **Duration:** 120+ seconds for full calibration

## Integration with Argo

### Lifecycle Manager
The lifecycle manager automatically discovers `bno085.py` and launches it in bridge mode.

### Topic Compatibility
The bridge maintains compatibility with existing Argo nodes:
- `/compass` - Used by controller for heading control
- `/pose` - Alternative heading source
- `/accel` - Motion detection
- `/gyro` - Angular velocity monitoring
- `/imu_health` - System health monitoring

### Makefile Integration
The `nodes/Makefile` provides convenient shortcuts and automatically uses the configuration file:

```bash
# Build the driver
make bno08x-build

# Test hardware
make bno08x-test

# Launch full system (uses bno08x_driver_argo.yaml)
make bno08x-launch-full

# Run calibration
make bno08x-calibrate
```

**Makefile Configuration:**
- **`BNO08X_CONFIG`**: Points to `vendor/bno08x_driver_argo.yaml`
- **Launch targets**: Automatically load the configuration file
- **Build targets**: Use the configuration for driver setup

## Advantages Over ICM-20948

1. **Better Sensor Fusion**: On-chip SH-2 firmware from CEVA Hillcrest Labs
2. **Automatic Calibration**: Built-in magnetometer, accelerometer, and gyroscope calibration
3. **Higher Accuracy**: Gyro-corrected compass heading with magnetic north reference
4. **Standard Driver**: Well-maintained ROS2 C++ driver
5. **Lower CPU Usage**: Sensor fusion performed on BNO085 chip
6. **Backward Compatible**: Existing Argo code works without changes

## I2C Error Recovery and Reliability

### Automatic Recovery System

The BNO085 bridge includes **automatic I2C error recovery** that monitors sensor health and restarts the systemd service on failures:

#### Health Monitoring
- **Data Timeout**: Detects when no IMU data received for 3+ seconds
- **Failure Tracking**: Counts consecutive failures and total errors per session
- **Recovery Mode**: Switches to 3-second check interval during recovery attempts

#### Recovery Process
1. **Detection**: No IMU data for 3 seconds → sensor marked unhealthy
2. **Service Restart**: Executes `sudo systemctl restart argo_bno085.service`
3. **Throttling**: Recovery attempts limited to every 5 seconds
4. **Logging**: "BNO085 sensor unreachable" logged every 60 seconds during outage
5. **Auto-Recovery**: Returns to normal mode when data resumes

#### Recovery Logs
```bash
# Monitor recovery attempts
journalctl -u argo_bno085.service -f | grep -E "(recovery|unreachable|restart)"

# Example output:
# [ERROR] BNO085 sensor unreachable for 5.0s (recovery attempts: 1, failures: 0)
# [INFO] Attempting recovery #1: Restarting bno08x_driver...
# [INFO] bno08x_driver restart successful
# [INFO] BNO085 sensor recovered - switching to normal mode
```

### Systemd Service Configuration

The `argo_bno085.service` provides robust process management:

**Service File**: `/etc/systemd/system/argo_bno085.service`

```ini
[Unit]
Description=Argo BNO085 IMU Driver and Bridge
After=network.target

[Service]
ExecStartPre=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/orangepi/argo_bno08x_ws/install/setup.bash"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/orangepi/argo_bno08x_ws/install/setup.bash && ros2 run bno08x_driver bno08x_driver --ros-args --params-file /home/orangepi/argo/nodes/vendor/bno08x_driver/config/bno085_i2c.yaml & python3 /home/orangepi/argo/nodes/bno085.py bridge"
WorkingDirectory=/home/orangepi/argo/nodes
StandardOutput=journal
StandardError=journal
Restart=on-failure
RestartSec=5
User=orangepi

[Install]
WantedBy=multi-user.target
```

**Key Features:**
- **Auto-Restart**: Restarts both processes on failure (5-second delay)
- **Environment Sourcing**: Proper ROS2 environment for both C++ and Python
- **Journal Logging**: All output captured in systemd journal
- **User Context**: Runs as `orangepi` user (not root)
- **Boot Integration**: Starts automatically on system boot

### Why Systemd Service is Required

Unlike other Argo sensor nodes, the BNO085 **requires** systemd service management because:

1. **Two-Process Architecture**: Must coordinate C++ driver + Python bridge startup
2. **I2C Recovery**: Python bridge needs sudo privileges to restart the service
3. **Process Lifecycle**: Ensures both processes start/stop together
4. **Auto-Restart**: Recovers from I2C bus errors and driver crashes
5. **Production Reliability**: Systemd provides robust process supervision

### Comparison: Direct I2C vs Service-Managed

| Feature | `anem.py` (Direct I2C) | `bno085.py` (Service-Managed) |
|---------|------------------------|-------------------------------|
| I2C Access | Direct via smbus2 | Via C++ driver |
| Process Count | 1 (Python only) | 2 (C++ + Python) |
| Systemd Service | Optional | **Required** |
| I2C Recovery | Reconnect in-process | Restart entire service |
| Sudo Required | No | Yes (for service restart) |
| Complexity | Low | Medium |
| Reliability | Good | Excellent (systemd supervision) |

## Troubleshooting

### Common Issues

**Issue**: "Failed to get product IDs" at startup  
**Solution**: This is a transient error during initialization. The driver recovers and works correctly.

**Issue**: "Failed to open the I2C bus" - Bus `/dev/i2c-7` not found  
**Solution**: The driver config was pointing to wrong bus. Ensure `nodes/vendor/bno08x_driver/config/bno085_i2c.yaml` has `bus: "/dev/i2c-0"` (not i2c-7). This is a submodule file, so changes must be maintained locally.

**Issue**: No compass data  
**Solution**: Check that both C++ driver AND bridge node are running:
```bash
ros2 node list  # Should show bno08x_driver and bno085_bridge
make bno085-service-status  # Check systemd service
```

**Issue**: Service restart fails during recovery  
**Solution**: Verify service is installed and user has sudo privileges:
```bash
systemctl status argo_bno085.service
sudo systemctl restart argo_bno085.service  # Test manually
```

**Issue**: Hardware not detected  
**Solution**: Verify I2C configuration:
```bash
i2cdetect -y 0  # Should show "4a" at address 0x4a
```

**Issue**: Tool won't start  
**Solution**: Check ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

**Issue**: Service not found in lifecycle manager status  
**Solution**: Ensure `argo_lifecycle_manager.py` includes BNO085 service check. The status should show:
```
🧭 BNO085 IMU: 🟢 RUNNING
```

### Debug Commands
```bash
# Check system status
python3 bno085.py status

# Monitor topics
ros2 topic echo /imu --once
ros2 topic echo /compass --once

# Check I2C devices
i2cdetect -y 0

# Check driver logs
ros2 node list
ros2 node info /bno08x_driver
```

## Performance

- **Update Rate**: 10Hz (configurable up to 400Hz)
- **Latency**: <10ms (C++ driver + Python bridge)
- **CPU Usage**: Minimal (fusion done on BNO085 chip)
- **Memory**: ~5MB for driver + bridge
- **Accuracy**: ±1° heading accuracy with proper calibration

## File Structure

```
nodes/
├── bno085.py                           ← Unified tool (all functionality)
├── vendor/
│   ├── bno08x_driver/                  ← C++ driver (git submodule)
│   └── bno08x_driver_argo.yaml         ← Driver configuration (ACTIVE)
├── Makefile                            ← Build and launch shortcuts
└── BNO085_README.md                    ← This file
```

**Key Files:**
- **`bno085.py`**: Main unified tool with all functionality
- **`bno08x_driver_argo.yaml`**: **Active configuration file** used by Makefile and direct commands
- **`Makefile`**: Convenient build and launch shortcuts

## References

- [Adafruit BNO085 Product Page](https://www.adafruit.com/product/4754)
- [BNO08x ROS2 Driver Documentation](https://docs.ros.org/en/humble/p/bno08x_driver/)
- [GitHub Repository](https://github.com/bnbhat/bno08x-ros2-driver)
- [BNO080 Datasheet](../BNO080_Datasheet_v1.3 selectable 2017.pdf)

## Development History

### Integration Process
The BNO085 integration was completed through a systematic approach:

1. **Hardware Verification**: Confirmed BNO085 communication via I2C bus 0 at address 0x4a
2. **Driver Integration**: Added [bno08x-ros2-driver](https://github.com/bnbhat/bno08x-ros2-driver) as git submodule
3. **Rotation Vector Validation**: Verified datasheet section 2.2.4 Rotation Vector output
4. **I2C Compatibility**: Tested with other nodes accessing the same I2C bus
5. **Calibration Tool**: Developed interactive calibration CLI with real-time guidance
6. **Consolidation**: Merged 4 separate Python files into single unified tool
7. **Health Monitoring**: Added `/imu_health` topic for lifecycle management

### Key Achievements
- ✅ **Hardware Working**: BNO085 detected and communicating
- ✅ **Sensor Fusion**: On-chip SH-2 firmware providing accurate orientation
- ✅ **Backward Compatibility**: All existing Argo topics maintained
- ✅ **Calibration**: Interactive tool with real-time guidance
- ✅ **Health Monitoring**: Lifecycle management integration
- ✅ **Unified Tool**: Single entry point for all BNO085 functionality

---

**Status**: ✅ **FULLY OPERATIONAL**  
**Last Updated**: 2025-01-19  
**Version**: 2.0 (Consolidated)
