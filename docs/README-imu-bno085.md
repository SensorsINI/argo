# BNO085 IMU Integration Guide

## Overview

The Argo autonomous sailboat uses the [Adafruit BNO085](https://www.adafruit.com/product/4754) 9-DOF Orientation IMU for precise heading and motion sensing. This sensor provides superior accuracy compared to the previous ICM-20948 through on-chip sensor fusion and automatic calibration.

## Architecture

### System Overview

**IMPORTANT:** The BNO085 uses a **two-process architecture** unlike other Argo sensor nodes like `anem.py`, `gps.py`, or `argo_battery_water.py` which directly access hardware. This dual-process design requires systemd service management for production use.

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
| `argo_battery_water.py` | Direct (smbus2) | Single Python process | ✅ Yes (basic service) |
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
- **Protocol**: SHTP (Sensor Hub Transport Protocol) over I2C, carrying SH-2 sensor-hub commands
- **Fusion**: CEVA Hillcrest Labs SH-2 firmware for on-chip sensor fusion

## Software Components

### 1. C++ Driver (`bno08x_driver`)
- **Upstream**: [bnbhat/bno08x-ros2-driver](https://github.com/bnbhat/bno08x-ros2-driver)
- **Argo fork**: [tobidelbruck/bno08x_ros2_driver](https://github.com/tobidelbruck/bno08x_ros2_driver) (git submodule at `nodes/vendor/bno08x_driver/`)
- **Function**: Direct hardware communication via I2C and SHTP/SH-2 stack (`include/sh2/`)
- **Topics Published**:
  - `/imu` (sensor_msgs/Imu) - Quaternion orientation, gyro, accel
  - `/magnetic_field` (sensor_msgs/MagneticField) - Magnetometer data
- Launched by `bno085_driver_launcher.sh` (used by `argo_bno085.service`)

### 2. Python Bridge (`bno085.py`)
- **Location**: `nodes/bno085.py`
- **Function**: Converts C++ driver output to Argo format
- **Topics Subscribed**: `/imu`, `/magnetic_field`
- **Topics Published**: See [Bridge published topics (Argo)](#bridge-published-topics-argo) for units, frames, and conventions.

## Quick Start

### 1. Build and Install Service (First Time Setup)
```bash
cd ~/argo/nodes

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
         source ~/argo/nodes/argo_bno08x_driver_workspace/install/setup.bash && \
         ros2 run bno08x_driver bno08x_driver --ros-args --params-file vendor/bno085_i2c_argo.yaml &"
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

### Automatic I2C Bus Configuration

**IMPORTANT**: The BNO08x driver submodule C++ source may default to `/dev/i2c-7`, but Orange Pi Zero 2W uses `/dev/i2c-0`. Argo sets the bus in **`nodes/vendor/bno085_i2c_argo.yaml`** (`i2c.bus: "/dev/i2c-0"`). The Makefile **patches the C++ default** to i2c-0 during `make bno08x-build` if upstream still ships i2c-7.

**Manual verification:**
```bash
grep 'bus:' vendor/bno085_i2c_argo.yaml
# Should show: bus: "/dev/i2c-0"
```

**Why C++ patching?**
- **Submodule is upstream**: Can't commit hardware-specific changes without forking
- **YAML is authoritative for Argo**: Single file `bno085_i2c_argo.yaml` used by systemd, Makefile, and launch
- **Fresh install support**: Ensures new clones work after `make bno085-service-install`

### Driver Configuration

The BNO08x C++ driver is configured via **`nodes/vendor/bno085_i2c_argo.yaml`** (single source of truth).

**File Location:** `nodes/vendor/bno085_i2c_argo.yaml`

**Current Configuration:**
```yaml
bno08x_ros:  # ⚠️ IMPORTANT: Namespace must match node name (not bno08x_driver)
  ros__parameters:

    frame_id: "bno085"

    i2c:
      enabled: true
      bus: "/dev/i2c-0"
      address: "0x4A"

    publish:
      magnetic_field:
        enabled: true
        rate: 1    # 1 Hz
      imu:
        enabled: true
        rate: 5    # 5 Hz — stable on shared bus 0; increase only if needed
```

**Configuration Options:**
- **`frame_id`**: ROS2 frame ID for the IMU (Argo default: `"bno085"`)
- **`i2c.bus`**: I2C device path (default: `/dev/i2c-0` for Orange Pi Zero 2W)
- **`i2c.address`**: BNO085 I2C address (default: `"0x4A"`)
- **`publish.magnetic_field.rate`**: Magnetometer publish rate in Hz (1–100, default: 1)
- **`publish.imu.rate`**: IMU data publish rate in Hz (1–400, default: 5 for shared bus 0)
- **`watchdog.timeout_ms`**: C++ driver reset if no sensor reports (optional; see yaml comments)

**⚠️ CRITICAL: YAML Namespace Configuration**

The YAML file **must** use `bno08x_ros:` as the top-level namespace, **not** `bno08x_driver:`. This is because the ROS2 node name is `bno08x_ros` (defined in the C++ source).

```yaml
# ✅ CORRECT
bno08x_ros:
  ros__parameters:
    ...

# ❌ WRONG - Parameters will be ignored!
bno08x_driver:
  ros__parameters:
    ...
```

If the wrong namespace is used, the driver will ignore your configuration and use default values (100Hz for both rates).

**Usage:**
This configuration file is automatically loaded by:
- Makefile targets (`bno08x-launch`, `bno08x-launch-full`)
- Systemd service (`argo_bno085.service`)
- Direct `ros2 run` commands with `--params-file` argument

**Customization:**
To change publish rates or other settings:
1. Edit `vendor/bno085_i2c_argo.yaml` (ensure `bno08x_ros:` namespace)
2. Restart the service: `make bno085-service-restart`
3. Verify rates in logs: Look for "IMU Rate: 5" and "Magnetic Field Rate: 1"

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
The bridge derives a yaw angle from the C++ driver’s `sensor_msgs/Imu` quaternion (Euler Z after the configured mount offset). That value is published as **compass heading** on `/compass` and, after a convention conversion, as **ENU math yaw** on `/pose` (see below).

```yaml
# /compass topic (from bridge) — compass convention on z
x: 0.0
y: 0.0
z: 127.93  # degrees, clockwise from north, range [0, 360)
```

## Bridge published topics (Argo)

All Argo-facing topics below are published by `bno085.py` in **bridge** mode. They originate from the C++ driver’s `/imu` stream (rotation vector fusion on the BNO085), except `/imu_health`, which is generated in the bridge.

| Topic | Message type | Source | `x` / `y` / `z` meaning | Range / convention |
|-------|----------------|--------|-------------------------|--------------------|
| `/compass` | `geometry_msgs/Vector3` | Quaternion → Euler yaw + `yaw_offset_deg` | **Heading (compass):** `z` only; `x`,`y` set to `0`. | `z ∈ [0, 360)` **degrees**, **clockwise from north**: 0° = north, 90° = east, 180° = south, 270° = west. |
| `/pose` | `geometry_msgs/Vector3` | Same fused yaw as `/compass`, then converted for Argo TF/controller | **ENU yaw (mathematical):** `z` only; `x`,`y` set to `0`. | `z ∈ [0, 360)` **degrees**, **counter-clockwise from east** when viewed from above: 0° = east, 90° = north, 180° = west, 270° = south. Conversion: `pose.z = (450 - compass_z) % 360` (same as `argo_unified_simulator_bridge`). |
| `/accel` | `geometry_msgs/Vector3` | `Imu.linear_acceleration` (m/s²) scaled by `1/9.81` | Approx. **g** in driver axes; `x`,`y` optionally rotated with `apply_axis_rotation` and mount offset. | Roughly -1 to 1 g per axis at rest (gravity on one axis); not SI m/s² on the wire. |
| `/gyro` | `geometry_msgs/Vector3` | `Imu.angular_velocity` (rad/s) → degrees/s | **deg/s** about IMU X, Y, Z. | Magnitude depends on motion; signs follow right-hand rule around each axis. |
| `/imu_health` | `std_msgs/Bool` | Bridge timer + IMU freshness | `data`: sensor path healthy. | `true` / `false`. |

**Why `/compass` and `/pose` differ:** Controllers and dashboards consume **compass** heading (wind and navigation are stated in compass degrees). `argo_transform_publisher`, Foxglove ENU views, and the simulator bridge expect **`/pose.z` as ROS-style yaw in the horizontal plane** (east-referenced, CCW positive), so the bridge publishes the conversion on `/pose` only.

**Legacy rosbag2 recordings:** Bags recorded before `/pose` carried math yaw may store compass values on `/pose.z`. That skews TF and 3D playback. Rewrite bags with:

`python3 scripts/fix_rosbag_pose_legacy_compass.py INPUT_BAG_DIR OUTPUT_BAG_DIR`  
(use `--dry-run` first; `--force` overwrites an existing output folder). That script **only** rewrites `/pose`; it does **not** change recorded `/tf` or recorded visualization markers (see script docstring).

To **re-record** playback with live visualization and transforms into a new bag, the launch file lives under **`scripts/argo_bag_rerecord.py`** (not under `launch/`). Typical invocation from the Argo repo root with ROS 2 sourced:  
`ros2 launch ./scripts/argo_bag_rerecord.py input_bag:=bags/your_bag output_bag:=your_bag_with_viz`  
or run **`scripts/argo_rerecord_bag.sh`**, which calls that launch file for you.

See **[README-bagfiles.md](README-bagfiles.md)** for all bag tools (narration merge, backup, Foxglove playback, etc.).

## Health Monitoring

The bridge publishes `/imu_health` (std_msgs/Bool) for lifecycle management:

### Health States
- **✅ HEALTHY (`data: true`)**: Sustained `/imu` stream — not stale, past post-gap cooldown, stable for ~5 s (see tuning constants in `bno085.py`)
- **❌ UNHEALTHY (`data: false`)**: No data, stale stream, post-gap cooldown, or stream not yet stable

### Implementation
- **Initialization**: Starts unhealthy until stream meets stability criteria
- **Data Reception**: Updates on each `/imu` message from C++ driver
- **Periodic Check**: 1 Hz timer monitors freshness, gaps, and stability
- **Auto-Recovery**: After startup grace, may run `sudo systemctl restart argo_bno085.service` if `/imu` stays absent (~15 s, throttled)

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
The bridge feeds the same topic names the rest of Argo expects (see [Bridge published topics (Argo)](#bridge-published-topics-argo)):
- `/compass` — compass heading (degrees, CW from north) for control, wind, and plots
- `/pose` — ENU math yaw on `z` for `argo_transform_publisher`, simulator-aligned consumers, and any node that already treated `/pose` as mathematical yaw
- `/accel` — gravity-scaled acceleration (approx. g) for roll/pitch-style consumers
- `/gyro` — angular rates in deg/s
- `/imu_health` — lifecycle and operator health

### Makefile Integration
The `nodes/Makefile` provides convenient shortcuts and automatically uses the configuration file:

```bash
# Build the driver
make bno08x-build

# Quick I2C address scan (not a full SHTP test)
make bno08x-i2c-test

# Full hardware bench test (recommended)
python3 ~/argo/tests/test_bno085.py --skip-ros

# Launch full system (uses bno085_i2c_argo.yaml)
make bno08x-launch-full

# Run calibration
make bno08x-calibrate
```

**Makefile Configuration:**
- **`BNO08X_CONFIG`**: Points to `vendor/bno085_i2c_argo.yaml`
- **Launch targets**: Automatically load the configuration file
- **Build targets**: Use the configuration for driver setup

### Boot ordering on shared I2C bus 0

The IMU shares **i2c-0** with `argo_battery_water.service` (ADC 0x34, SHT45 0x44). BNO085 SHTP init is sensitive to concurrent bus traffic.

- `argo_bno085.service` starts **before** battery/water (`Before=argo_battery_water.service`)
- Battery/water **ExecStartPre** waits until `bno08x_driver` has been stable for 3 s (`scripts/wait_for_bno085_ready.sh`)

See **[README-i2c.md](README-i2c.md)** for bus map, lock recovery, and service install targets (`make -C nodes battery-water-install`, `make -C nodes bno085-service-install`).

## SHTP and SH-2 protocol handling

The BNO085 does **not** use SMBus register reads. It speaks **SHTP** (Sensor Hub Transport Protocol) on plain I2C `read()`/`write()` via `/dev/i2c-0`.

### Protocol layers

```
┌─────────────────────────────────────────────────────────┐
│  ROS2 bno08x_driver  →  sh2.c / shtp.c (vendor C++)     │  SH-2 commands, adverts, sensor reports
├─────────────────────────────────────────────────────────┤
│  I2CInterface::read/write  (i2c_interface.hpp)          │  SHTP framing, 32-byte chunks
├─────────────────────────────────────────────────────────┤
│  Linux i2c-dev  (/dev/i2c-0, address 0x4A)              │  Raw bus access
└─────────────────────────────────────────────────────────┘
```

**SHTP packet layout** (each logical transfer):

| Bytes | Field |
|-------|--------|
| 0–1 | Length (LSB, MSB with continuation bit in bit 15) |
| 2 | Channel (0=command, 1=executable, 2=SH-2 control, 3=reports, …) |
| 3 | Sequence |
| 4+ | Payload (SH-2 commands/responses or sensor reports) |

**I2C read algorithm** (Argo fork, `i2c_interface.hpp::read()`):

1. Read 4-byte header (length only; idle FIFO returns `[0,0,0,0]` → no data).
2. Read **`packet_size`** bytes into the HAL buffer in ≤32-byte chunks; continuation reads pull **+4** extra wire bytes per chunk and discard the first 4 (Hillcrest SHTP rule).
3. Pass the full buffer to `shtp.c` `rxAssemble()` for multi-fragment assembly.

**SH-2 bring-up** (`sh2_open()`): after soft reset `[5,0,1,0,1]` and 1 s wait, the stack polls reads until executable reset-complete and sensor-hub **advertisements** arrive (Argo fork waits for adverts before treating channel numbers as valid). Failure here surfaces as `SH2_ERR_TIMEOUT` in the journal.

### Argo fork vs stock upstream driver

Submodule path: `nodes/vendor/bno08x_driver/`  
**Upstream reference**: [bnbhat/bno08x-ros2-driver](https://github.com/bnbhat/bno08x-ros2-driver)  
**Argo fork** (pushed to `tobidelbruck/bno08x_ros2_driver`): Orange Pi / shared-bus fixes not in upstream.

| Area | Stock upstream (typical) | Argo fork |
|------|-------------------------|-----------|
| I2C bus default | Often `/dev/i2c-7` in C++ | Patched to `/dev/i2c-0`; **authoritative** bus in `nodes/vendor/bno085_i2c_argo.yaml` |
| `I2CInterface::read()` | Header peek + `memcpy` to buffer + read `packet_size − 4` cargo | **Peek header for length only**, then read **`packet_size`** bytes into buffer (required on Orange Pi `i2c-dev`; upstream-style read desynchronizes the FIFO → `SH2_ERR_TIMEOUT`) |
| Partial reads | Single `read()`; fail on short count | **`readExact_()`** retries `EAGAIN`/`EINTR` (matches bench test behavior) |
| Idle FIFO | May log invalid length | **`packet_size == 0` returns silently** (poll loop during `sh2_open`) |
| Soft reset | 5-byte reset packet | Same packet + **1 s** post-reset delay; **5** write retries |
| `sh2_open()` | Earlier races on adverts | **Wait for reset + sensor-hub adverts** before control traffic |
| I2C errors | Verbose per-read logs | Quiet by default; one-line summary on SH-2 failure (`BNO08X_I2C_VERBOSE=1` for traces) |
| Driver params | Packaged `config/bno085_i2c.yaml` | **Removed from submodule**; Argo uses `nodes/vendor/bno085_i2c_argo.yaml` only |
| Watchdog | Default 5 s | Configurable via yaml (`watchdog.timeout_ms`) |

After changing the submodule, rebuild: `make -C nodes bno08x-build`, then `make -C nodes bno085-service-restart`.

### Standalone hardware test (no ROS)

**`tests/test_bno085.py`** exercises the **same I2C/SHTP path** as the C++ driver but does **not** import or run `bno08x_driver`. It reimplements the HAL read and `shtp.c`-style `rxAssemble` in Python (used to debug bus issues when ROS is irrelevant).

**When to use:**

- Bench check after wiring, resolder, or I2C recovery
- Confirm chip responds before debugging systemd/ROS
- Compare against failing `ros2 run bno08x_driver …` (passing test + failing driver → rebuild driver submodule)

**Stop other i2c-0 users first:**

```bash
sudo systemctl stop argo_bno085.service argo_battery_water.service
```

**Commands** (from repo root):

```bash
# Full hardware path: soft reset, SHTP drain, Product ID vote
python3 tests/test_bno085.py --skip-ros

# Verbose HAL read / rxAssemble traces
python3 tests/test_bno085.py --skip-ros --debug

# Lock part number to Argo BOM (optional)
python3 tests/test_bno085.py --skip-ros --expect-part 0x0098A6B4

# Live accel reports only (skip flaky Product ID path)
python3 tests/test_bno085.py --skip-ros --skip-product-id

# Also require /imu streaming when ROS stack is up
python3 tests/test_bno085.py
```

**Pass criteria:** soft reset + first SHTP packet + Product ID (or accelerometer reports with `--skip-product-id`); optionally non-zero `/imu` rate without `--skip-ros`.

**Note:** `make -C nodes bno08x-i2c-test` only runs `i2cdetect` (address visible). It does **not** validate SHTP; use `test_bno085.py` for that.

**Important:** `smbus2` can leave the fd non-blocking; the test clears `O_NONBLOCK` before `os.read()`, same requirement as reliable blocking reads on this kernel.

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
Description=Argo BNO085 IMU Driver Service
After=network.target
Before=argo_battery_water.service argo_launch_standard.service

[Service]
ExecStartPre=/bin/mkdir -p /var/log.hdd/persistent
ExecStart=/home/orangepi/argo/nodes/bno085_driver_launcher.sh
WorkingDirectory=/home/orangepi/argo/nodes
StandardOutput=append:/var/log.hdd/persistent/argo-bno085.log
StandardError=append:/var/log.hdd/persistent/argo-bno085.log
Restart=always
RestartSec=5
User=orangepi
```

**Key Features:**
- **Auto-Restart**: Restarts driver + bridge on failure (5-second delay)
- **Launcher script**: Starts C++ driver, then Python bridge; exits non-zero if either child dies (systemd restarts whole unit)
- **Persistent log**: `/var/log.hdd/persistent/argo-bno085.log` (no `tee` pipeline — allows clean SIGTERM on stop)
- **Journal**: Use `journalctl -u argo_bno085.service` for systemd metadata

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

## Debugging

### Enabling Driver Debug Output

The BNO08x C++ driver includes comprehensive debug logging that can be enabled for troubleshooting I2C communication and sensor data issues.

#### How to Enable Debug Mode

1. **Edit the logger configuration:**
   ```bash
   nano nodes/vendor/bno08x_driver/include/bno08x_driver/logger.h
   ```

2. **Uncomment the debug enable line:**
   ```cpp
   // Change this line:
   // #define DEBUG_LOG_ENABLED
   
   // To this:
   #define DEBUG_LOG_ENABLED
   ```

3. **Rebuild the driver:**
   ```bash
   cd nodes
   make bno08x-build
   ```

4. **Restart the service:**
   ```bash
   make bno085-service-restart
   ```

#### Debug Output Examples

When debug mode is enabled, you'll see detailed output like:
```
DEBUG: BNO08x - I2C Interface Created
Bus: /dev/i2c-0
Address: 0x4a
DEBUG: BNO08x - Sending soft reset packet to the sensor
DEBUG: BNO08x - I2C Comm Opened and Soft Reset Sent
DEBUG: BNO08x - Packet size: 276
DEBUG BUFFER: 14 1 0 0 
DEBUG: BNO08x - Packet size: 5
DEBUG BUFFER: 5 0 1 0 
DEBUG: BNO08x - Packet size: 20
DEBUG BUFFER: 14 0 2 0 
```

#### Debug Information Provided

- **I2C Communication**: Bus initialization, address configuration
- **Packet Processing**: SH-2 protocol packet sizes and content
- **Sensor Callbacks**: When sensor data is received
- **Error Details**: Detailed I2C read/write operations
- **Buffer Contents**: Raw data being transmitted

#### When to Use Debug Mode

- **I2C Communication Issues**: When sensor is not detected or data is corrupted
- **Protocol Problems**: SH-2 protocol communication failures
- **Performance Analysis**: Understanding data flow and timing
- **Development**: Adding new features or troubleshooting custom modifications

#### Disabling Debug Mode

To disable debug output and improve performance:

1. **Comment out the debug enable line:**
   ```cpp
   // #define DEBUG_LOG_ENABLED
   ```

2. **Rebuild and restart:**
   ```bash
   make bno08x-build
   make bno085-service-restart
   ```

#### Performance Impact

- **Debug Enabled**: ~10-20% CPU overhead, verbose output
- **Debug Disabled**: Minimal overhead, clean logs
- **Production Use**: Always disable debug mode for production

## Troubleshooting

### Common Issues

**Issue**: `Failed to get product IDs` or `SH2_ERR_TIMEOUT` at startup  
**Solution**:
1. Run bench test: `python3 tests/test_bno085.py --skip-ros` (see [Standalone hardware test](#standalone-hardware-test-no-ros))
2. Stop bus contention: `sudo systemctl stop argo_battery_water.service` during IMU-only debug
3. Rebuild driver after submodule updates: `make -C nodes bno08x-build && make -C nodes bno085-service-restart`
4. If `i2cdetect` sees `0x4a` but both fail: check wiring and [README-i2c.md](README-i2c.md) bus lock recovery

**Issue**: "Failed to open the I2C bus" - Bus `/dev/i2c-7` not found  
**Solution**: The `make bno085-service-install` and `make bno08x-build` targets automatically patch the I2C bus from i2c-7 to i2c-0 for Orange Pi Zero 2W. This happens automatically on fresh installs.

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
# Standalone I2C/SHTP test (no ROS)
python3 ~/argo/tests/test_bno085.py --skip-ros --debug

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

- **Update Rate**: 5Hz for IMU, 1Hz for magnetometer (configurable 1-400Hz)
- **Latency**: <10ms (C++ driver + Python bridge)
- **CPU Usage**: ~10-15% combined (C++ driver + Python bridge at 5Hz)
- **Memory**: ~60MB for driver + bridge
- **Accuracy**: ±1° heading accuracy with proper calibration
- **Sample Rate Optimization**: 5Hz is sufficient for sailboat dynamics (slow compared to drones/cars)

## File Structure

```
nodes/
├── bno085.py                           ← Unified tool (bridge, calibrate, verify)
├── bno085_driver_launcher.sh           ← Starts C++ driver + bridge (systemd)
├── vendor/
│   ├── bno08x_driver/                  ← C++ driver submodule (Argo fork)
│   └── bno085_i2c_argo.yaml            ← Driver configuration (ACTIVE)
└── Makefile                            ← Build and launch shortcuts

tests/
└── test_bno085.py                      ← Standalone I2C/SHTP hardware test (no ROS)

docs/
├── README-imu-bno085.md                ← This guide
└── README-i2c.md                       ← Bus 0 map, boot order, lock recovery
```

**Key Files:**
- **`nodes/bno085.py`**: Main unified tool with all functionality
- **`nodes/vendor/bno085_i2c_argo.yaml`**: **Active configuration file** (systemd, Makefile, launch)
- **`tests/test_bno085.py`**: Bench validation without ROS
- **`nodes/Makefile`**: Convenient build and launch shortcuts

## References

- [Adafruit BNO085 Product Page](https://www.adafruit.com/product/4754)
- [BNO08x ROS2 Driver (upstream)](https://github.com/bnbhat/bno08x-ros2-driver)
- [Argo BNO08x fork](https://github.com/tobidelbruck/bno08x_ros2_driver)
- [BNO080 Datasheet](../BNO080_Datasheet_v1.3 selectable 2017.pdf)
- [I2C bus configuration and recovery](README-i2c.md)

## Development History

### Integration Process
The BNO085 integration was completed through a systematic approach:

1. **Hardware Verification**: Confirmed BNO085 communication via I2C bus 0 at address 0x4a
2. **Driver Integration**: Added [bno08x-ros2-driver](https://github.com/bnbhat/bno08x-ros2-driver) as git submodule; Argo-specific I2C/SHTP fixes maintained in [fork](https://github.com/tobidelbruck/bno08x_ros2_driver)
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
**Last Updated**: 2026-06-20  
**Version**: 2.1 (SHTP/I2C fork documentation, standalone test)
