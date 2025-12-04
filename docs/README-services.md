# Argo Systemd Services Documentation

This document describes the systemd services that manage the Argo autonomous sailboat system, their dependencies, and boot sequence.

## Table of Contents
- [Service Overview](#service-overview)
- [Dependency Architecture](#dependency-architecture)
- [Boot Sequence](#boot-sequence)
- [Shutdown Sequence](#shutdown-sequence)
- [Service Details](#service-details)
- [Installation](#installation)
- [Troubleshooting](#troubleshooting)

## Service Overview

The Argo system consists of 7 main systemd services that coordinate hardware, sensors, and ROS2 nodes:

| Service | Purpose | Critical | Dependencies |
|---------|---------|----------|--------------|
| `argo_power_control.service` | Power button, LED heartbeat, system control | Yes | network.target |
| `argo_battery_water.service` | Battery and water monitoring | Yes | network.target |
| `argo_bno085.service` | IMU driver (BNO085) | Yes | network.target |
| `argo_health_monitor.service` | Node health monitoring and aggregation | Yes | network.target |
| `argo_radio_servo_module.service` | Kernel module loader for radio/servo | Yes | network.target |
| `argo_launch_standard.service` | Main ROS2 node launcher | Yes | bno085, radio_servo_module, health_monitor |
| `argo_thermal_monitor.service` | Temperature monitoring | No | multi-user.target |

## Dependency Architecture

### Dependency Graph

```
network.target (system)
    ↓
multi-user.target starts
    ↓
    ├─────────────────────────────────────────┐
    ↓                                         ↓
argo_power_control.service            argo_battery_water.service
  - Wants: battery_water                - Independent startup
  - Provides: LED heartbeat             - Monitors battery/water
  - Handles: Power button               - Late shutdown
    ↓
    ↓ (continues independently)
    ↓
    ├─────────────────────────────────────────────────────────────┐
    ↓                    ↓                    ↓                   ↓
argo_bno085.service  argo_health_monitor.service  argo_radio_servo_module.service
  - IMU driver          - Health monitoring          - Kernel module loader
  - Blocks launch       - Aggregates node status    - Blocks launch until ready
  - Late shutdown       - Provides health service    - Provides radio/servo interface
    ↓                    ↓                           ↓
    └─────────────┬──────┴───────────────────────────┘
                  ↓
      argo_launch_standard.service
        - Main ROS2 launcher
        - Requires: BNO085 + health_monitor + radio/servo
        - Launches all ROS2 nodes

    ↓ (independent)
    ↓
argo_thermal_monitor.service
  - Temperature monitoring
  - Optional service
```

### Key Dependency Relationships

1. **power_control → battery_water** (`Wants`)
   - Power control prefers battery monitoring but can start without it
   - Graceful dependency for LED status patterns

2. **health_monitor → launch_standard** (`Before` + `Wants`)
   - Launch service MUST wait for health monitor
   - Provides aggregated health status for power control LED flashes
   - Essential for system monitoring

3. **bno085 → launch_standard** (`Before` + `Wants`)
   - Launch service MUST wait for IMU driver
   - Essential for autonomous navigation

4. **radio_servo_module → launch_standard** (`Before` + `Wants`)
   - Launch service MUST wait for radio/servo hardware
   - Essential for control interface

5. **battery_water → shutdown.target** (`Before`)
   - Battery monitoring stays active late into shutdown
   - Critical for safe power-off

6. **bno085 → shutdown.target** (`Before`)
   - IMU driver stays active late into shutdown
   - Proper hardware cleanup

## Boot Sequence

### Timeline and Critical Points

```
[T+0s] System Boot
    ↓
[T+2s] network.target completes
    ↓
[T+3s] multi-user.target starts all WantedBy services
    ↓
    ├─ argo_power_control.service starts
    │  - Initializes GPIO for power button
    │  - Starts LED heartbeat (slow = booting)
    │  - Requests battery_water to start (Wants)
    │
    ├─ argo_battery_water.service starts
    │  - Monitors battery voltage
    │  - Detects water intrusion
    │  - Publishes status for power_control
    │
    ├─ argo_bno085.service starts
    │  - Initializes BNO085 IMU via I2C
    │  - Blocks launch_standard from starting
    │
    ├─ argo_health_monitor.service starts
    │  - Initializes health monitoring ROS2 node
    │  - Provides aggregated health status service
    │  - Blocks launch_standard from starting
    │
    └─ argo_radio_servo_module.service starts
       - Loads kernel module
       - Initializes radio/servo hardware
       - Blocks launch_standard from starting
    ↓
[T+5s] Essential services ready
    ↓
    └─ argo_launch_standard.service starts
       - Waits for BNO085 + health_monitor + radio_servo_module
       - Launches all ROS2 nodes
       - GPS, anemometer, controller, etc.
    ↓
[T+8s] All ROS2 nodes running
    ↓
    └─ Power control LED: Fast heartbeat (system running)
    ↓
[T+10s] System fully operational
```

### Service Start Order

1. **network.target** - System network initialization
2. **multi-user.target** - Multi-user system mode
3. **Power Control** - First Argo service (LED, power button)
4. **Battery/Water** - Monitoring services (independent start)
5. **BNO085** - IMU driver (blocks launch)
6. **Health Monitor** - Node health monitoring (blocks launch)
7. **Radio/Servo Module** - Kernel module (blocks launch)
8. **Launch Standard** - Main ROS2 launcher (waits for 5-7)
9. **Thermal Monitor** - Optional monitoring (independent)

## Shutdown Sequence

### Timeline and Critical Points

```
[T+0s] Shutdown initiated (poweroff/halt command)
    ↓
[T+0.5s] shutdown.target starts
    ↓
[T+1s] Most services stop (reverse dependency order)
    ↓
    ├─ argo_launch_standard.service stops first
    │  - ROS2 nodes shutdown gracefully
    │  - Controller stops sending commands
    │  - Servos go to high impedance mode
    │
    ├─ argo_radio_servo_module.service stops
    │  - Kernel module remains loaded
    │  - Hardware stays in safe state
    │
    └─ argo_thermal_monitor.service stops
       - Temperature monitoring ends
    ↓
[T+3s] Late shutdown services continue
    ↓
    ├─ argo_bno085.service still running
    │  - IMU driver active (Before=shutdown.target)
    │  - Continues publishing data
    │
    └─ argo_battery_water.service still running
       - Battery monitoring active (Before=shutdown.target)
       - Critical battery status available
    ↓
[T+5s] Power control prepares for shutdown
    ↓
    └─ argo_power_control.service final actions
       - LED patterns indicate shutdown
       - Monitors shutdown progress
       - Last service to stop
    ↓
[T+6s] Late shutdown services stop
    ↓
    ├─ argo_bno085.service stops
    │  - IMU driver cleanup
    │
    └─ argo_battery_water.service stops
       - Battery monitoring cleanup
    ↓
[T+8s] shutdown.target completes
    ↓
[T+10s] System halts/powers off
```

### Important Shutdown Notes

- **Late Shutdown**: Battery and BNO085 services use `Before=shutdown.target` to stay active longer
- **Power Control**: Handles shutdown LED patterns and hardware cleanup
- **Graceful Stop**: Services now shut down cleanly in 1-2 seconds (previously required SIGKILL)
- **Safe State**: Servos automatically go to high impedance (radio control) on shutdown

### Graceful Shutdown Implementation (Dec 2025)

**Problem Identified**: Services were being force-killed with SIGKILL during shutdown, preventing clean resource cleanup.

**Root Cause**: `KillMode=mixed` in systemd service configuration sends SIGTERM only to the main process (bash shell) and **immediately** sends SIGKILL to all child processes (Python, tee), preventing Python signal handlers from executing.

**Solution Implemented**:
1. **Changed `KillMode` from `mixed` to `control-group`**
   - All processes in control group now receive SIGTERM
   - Systemd waits `TimeoutStopSec` before escalating to SIGKILL
   - Python signal handlers have time to execute cleanup code

2. **Added explicit signal configuration**:
   ```ini
   KillMode=control-group
   KillSignal=SIGTERM
   FinalKillSignal=SIGKILL
   ```

3. **Added `exec` to ExecStart** (limited benefit with pipes):
   ```ini
   ExecStart=/bin/bash -c 'set -eo pipefail; source /opt/ros/humble/setup.bash && exec /usr/bin/python3 ...'
   ```
   Note: `exec` has limited effect when piping to `tee` for persistent logging, but included for clarity.

4. **Battery service: Replaced `rclpy.spin()` with `spin_once()` loop**:
   - `rclpy.spin()` blocks in C code and doesn't check signals frequently
   - Custom `spin_once()` loop with 100ms timeout returns to Python regularly
   - Allows Python to check `shutdown_requested` flag and handle signals responsively
   - Power control still uses `rclpy.spin()` but shuts down cleanly with `KillMode=control-group`

**Results**:
- Battery service: Clean shutdown in ~1-2 seconds (was 10s timeout → SIGKILL)
- Power control: Clean shutdown in ~1-2 seconds (was immediate SIGKILL)
- No force-kill messages in logs
- Proper resource cleanup (battery slopes saved, timers cancelled, etc.)

**Key Learning**: `KillMode=mixed` is inappropriate for services with child processes that need graceful shutdown. Use `KillMode=control-group` to allow all processes to handle SIGTERM.

## Service Details

### 1. argo_power_control.service

**Purpose**: Hardware power management and user interface

**Configuration**:
```ini
[Unit]
Description=Argo Power Control System
After=network.target
Wants=argo_battery_water.service

[Service]
Type=simple
User=orangepi
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /home/orangepi/argo/power_control/argo_power_control.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Functions**:
- Power button monitoring (GPIO-based)
- LED heartbeat patterns:
  - Slow (1Hz) = System booting
  - Fast (2Hz) = System running
  - SOS pattern = Critical battery
- ROS2 services for recording control
- Shutdown coordination
- Battery status LED indicators

**Dependencies**: 
- Requires: network.target
- Wants: argo_battery_water.service (soft dependency)
- Started by: multi-user.target

**Installation**: `make install` in `power_control/` directory

---

### 2. argo_battery_water.service

**Purpose**: Battery voltage and water intrusion monitoring

**Configuration**:
```ini
[Unit]
Description=Argo Battery and Water Monitoring Service
After=network.target
Before=shutdown.target
DefaultDependencies=no

[Service]
Type=simple
User=orangepi
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /home/orangepi/argo/nodes/argo_battery_water.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Functions**:
- Battery voltage monitoring (via MAX11612 ADC)
- Water detection sensor
- Critical battery alerts (<7.2V)
- ROS2 topic publishing for status
- Health service for system monitoring

**Dependencies**:
- Requires: network.target
- Before: shutdown.target (late shutdown)
- Started by: multi-user.target
- Wanted by: argo_power_control.service

**Installation**: `make battery-water-install` in `nodes/` directory

---

### 3. argo_bno085.service

**Purpose**: IMU driver for orientation and navigation

**Configuration**:
```ini
[Unit]
Description=Argo BNO085 IMU Driver Service
After=network.target
Before=argo_launch_standard.service shutdown.target
DefaultDependencies=no

[Service]
Type=simple
User=orangepi
ExecStart=/home/orangepi/argo/nodes/bno085_driver_launcher.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Functions**:
- BNO085 IMU initialization (I2C)
- Sensor fusion (quaternion, Euler angles)
- Compass heading
- Accelerometer and gyroscope data
- ROS2 topic publishing

**Dependencies**:
- Requires: network.target
- Before: argo_launch_standard.service (blocks launch)
- Before: shutdown.target (late shutdown)
- Started by: multi-user.target
- Required by: argo_launch_standard.service (via Wants)

**Installation**: `make bno085-service-install` in `nodes/` directory

---

### 4. argo_health_monitor.service

**Purpose**: Node health monitoring and aggregation service

**Configuration**:
```ini
[Unit]
Description=Argo Health Monitor Service
After=network.target
Before=argo_launch_standard.service
Wants=network.target

[Service]
Type=simple
User=orangepi
Group=orangepi
WorkingDirectory=/home/orangepi/argo/launch
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && /usr/bin/python3 /home/orangepi/argo/launch/argo_health_monitor.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Functions**:
- Monitors health status of all ROS2 nodes
- Aggregates health data from individual node health topics
- Provides `/argo/health/status` service for system-wide health queries
- Tracks node running status via `ros2 node list`
- Distinguishes simulation-only nodes (not expected in physical robot mode)
- Used by power control for LED flash count indication

**Dependencies**:
- Requires: network.target
- Before: argo_launch_standard.service (blocks launch)
- Started by: multi-user.target
- Required by: argo_launch_standard.service (via Wants)

**Installation**: `make install_health_monitor` in `launch/` directory

---

### 5. argo_radio_servo_module.service

**Purpose**: Kernel module loader for radio/servo hardware

**Configuration**:
```ini
[Unit]
Description=Load Argo Radio Servo Kernel Module
After=network.target
Before=argo_launch_standard.service

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'if ! lsmod | grep -q argo_radio_servo_module; then insmod /lib/modules/$(uname -r)/kernel/drivers/misc/argo/argo_radio_servo_module.ko; fi'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

**Functions**:
- Loads kernel module for PWM control
- Radio control input (PI11, PI13)
- Servo output (PI12, PI14)
- High impedance safety mode
- Sysfs interface for user space

**Dependencies**:
- Requires: network.target
- Before: argo_launch_standard.service (blocks launch)
- Started by: multi-user.target
- Required by: argo_launch_standard.service (via Wants)

**Installation**: `make install_module_service` in `nodes/pwm_capture_module/` directory

---

### 6. argo_launch_standard.service

**Purpose**: Main ROS2 node launcher and coordinator

**Configuration**:
```ini
[Unit]
Description=Argo Robot ROS2 Launch Service
After=network.target argo_bno085.service argo_radio_servo_module.service argo_health_monitor.service
Wants=argo_bno085.service argo_radio_servo_module.service argo_health_monitor.service

[Service]
Type=simple
User=orangepi
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 launch launch/argo_launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Functions**:
- Launches all ROS2 nodes
- GPS, anemometer, controller, recorder
- Lifecycle management
- Node health monitoring
- Automatic restart on failure

**Dependencies**:
- Requires: network.target, argo_bno085.service, argo_radio_servo_module.service, argo_health_monitor.service
- Wants: argo_bno085.service, argo_radio_servo_module.service, argo_health_monitor.service (must start)
- Started by: multi-user.target
- Blocked by: argo_bno085.service, argo_health_monitor.service, argo_radio_servo_module.service (until ready)

**Installation**: `make install-standard` in `launch/` directory

---

### 7. argo_thermal_monitor.service

**Purpose**: System temperature monitoring

**Configuration**:
```ini
[Unit]
Description=Thermal monitoring and logging
After=multi-user.target

[Service]
Type=simple
User=root
ExecStart=/usr/local/bin/argo_thermal_monitor.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Functions**:
- CPU temperature monitoring
- Periodic logging
- Optional service

**Dependencies**:
- Requires: multi-user.target
- Started by: multi-user.target
- Independent (no other services depend on it)

**Installation**: `make install_thermal_monitor` in `launch/` directory

## Installation

### Install All Services

```bash
# 1. Install power control
cd /home/orangepi/argo/power_control
sudo make install

# 2. Install battery/water monitoring
cd /home/orangepi/argo/nodes
sudo make battery-water-install

# 3. Install BNO085 driver
cd /home/orangepi/argo/nodes
sudo make bno085-service-install

# 4. Install health monitor
cd /home/orangepi/argo/launch
sudo make install_health_monitor

# 5. Install radio/servo module
cd /home/orangepi/argo/nodes/pwm_capture_module
sudo make install_module_service

# 6. Install launch service
cd /home/orangepi/argo/launch
sudo make install-standard

# 7. (Optional) Install thermal monitor
cd /home/orangepi/argo/launch
sudo make install_thermal_monitor

# 8. Reload and enable all services
sudo systemctl daemon-reload
sudo systemctl enable argo_power_control.service
sudo systemctl enable argo_battery_water.service
sudo systemctl enable argo_bno085.service
sudo systemctl enable argo_health_monitor.service
sudo systemctl enable argo_radio_servo_module.service
sudo systemctl enable argo_launch_standard.service
```

### Verify Installation

```bash
# Check service status
systemctl status argo_power_control.service
systemctl status argo_battery_water.service
systemctl status argo_bno085.service
systemctl status argo_health_monitor.service
systemctl status argo_radio_servo_module.service
systemctl status argo_launch_standard.service

# Check for circular dependencies
systemd-analyze verify /etc/systemd/system/argo_*.service

# View dependency tree
systemctl list-dependencies argo_launch_standard.service
```

## Troubleshooting

### Common Issues

#### Service Not Starting at Boot

**Symptom**: Service shows as "inactive (dead)" after boot

**Diagnosis**:
```bash
# Check service status
sudo systemctl status argo_power_control.service

# Check boot logs
journalctl -b 0 -u argo_power_control.service

# Verify service is enabled
sudo systemctl is-enabled argo_power_control.service
```

**Solutions**:
1. Enable service: `sudo systemctl enable argo_power_control.service`
2. Check for dependency issues: `systemd-analyze verify /etc/systemd/system/argo_power_control.service`
3. Check for circular dependencies (see below)

---

#### Circular Dependency Errors

**Symptom**: Systemd log shows "ordering cycle" or services don't start

**Diagnosis**:
```bash
# Check for circular dependencies
systemd-analyze verify /etc/systemd/system/argo_*.service

# Check boot logs for "ordering cycle"
journalctl -b 0 | grep -i "ordering cycle"
```

**Common Causes**:
- Service has `After=X.target` but also `WantedBy=X.target`
- Two services have `After` dependencies on each other

**Solution**: Review service `After=` and `WantedBy=` directives, ensure no circular references

---

#### Service Fails to Start

**Symptom**: Service shows as "failed" status

**Diagnosis**:
```bash
# Check service logs
journalctl -u argo_launch_standard.service -n 50

# Check for recent errors
journalctl -p err -b 0 | grep argo

# Manual test
cd /home/orangepi/argo
source /opt/ros/humble/setup.bash
python3 nodes/argo_battery_water.py
```

**Common Causes**:
- Missing ROS2 environment
- Hardware not available (I2C, GPIO, UART)
- Permission issues (user not in i2c/gpio groups)
- Missing dependencies

---

#### Services Start in Wrong Order

**Symptom**: Launch service starts before BNO085 is ready

**Diagnosis**:
```bash
# Check actual boot order
systemd-analyze plot > boot.svg
# View boot.svg in browser to see timeline

# Check dependencies
systemctl list-dependencies argo_launch_standard.service
```

**Solution**: Verify `After=` and `Before=` directives are correct

---

#### Late Shutdown Not Working

**Symptom**: Battery/BNO085 services stop too early during shutdown

**Diagnosis**:
```bash
# Check service configuration
grep "Before=shutdown.target" /etc/systemd/system/argo_battery_water.service
```

**Solution**: Ensure service has `Before=shutdown.target` directive

---

#### Service Force-Killed with SIGKILL

**Symptom**: Systemd logs show "Killing process ... with signal SIGKILL" immediately during service stop

**Diagnosis**:
```bash
# Check service shutdown logs
sudo journalctl -u argo_power_control.service --no-pager --since "5 minutes ago" | grep -i "killing\|sigkill"

# Check KillMode setting
grep "KillMode" /etc/systemd/system/argo_power_control.service

# Test shutdown timing
sudo systemctl start argo_power_control.service
sleep 5
sudo systemctl stop argo_power_control.service
# Check if it stops cleanly in 1-2 seconds or requires force-kill
```

**Common Causes**:
- `KillMode=mixed` sends SIGKILL immediately to child processes
- Python using `rclpy.spin()` which blocks signals in C code
- No signal handlers installed in Python code
- Timeout too short for cleanup operations

**Solutions**:

1. **Change KillMode to control-group** (primary fix):
   ```ini
   [Service]
   KillMode=control-group
   KillSignal=SIGTERM
   FinalKillSignal=SIGKILL
   TimeoutStopSec=10
   ```

2. **Add exec to ExecStart** (helps signal forwarding):
   ```ini
   ExecStart=/bin/bash -c 'set -eo pipefail; source /opt/ros/humble/setup.bash && exec /usr/bin/python3 /path/to/script.py'
   ```

3. **For ROS2 nodes: Replace rclpy.spin() with spin_once() loop** (for responsive shutdown):
   ```python
   # Instead of:
   rclpy.spin(node)
   
   # Use:
   while rclpy.ok() and not node.shutdown_requested:
       rclpy.spin_once(node, timeout_sec=0.1)
   ```

4. **Ensure Python signal handlers are installed**:
   ```python
   import signal
   import rclpy.executors
   
   def _signal_handler(self, signum, frame):
       self.get_logger().info(f"Received signal {signum}, shutting down...")
       self.shutdown_requested = True
       if rclpy.ok():
           rclpy.shutdown()
   
   signal.signal(signal.SIGTERM, self._signal_handler)
   signal.signal(signal.SIGINT, self._signal_handler)
   ```

**Verification**:
```bash
# Should show clean shutdown in 1-2 seconds
sudo systemctl restart argo_power_control.service
sudo systemctl stop argo_power_control.service

# Check logs - should NOT see "Killing process" messages
sudo journalctl -u argo_power_control.service --no-pager --since "1 minute ago" | tail -20
```

**Reference**: See "Graceful Shutdown Implementation" in the Shutdown Sequence section above.

---

### Debug Commands

```bash
# View all Argo services
systemctl list-units "argo_*"

# Check service dependencies
systemctl show argo_launch_standard.service -p After -p Wants -p Before

# Monitor service logs in real-time
journalctl -u argo_launch_standard.service -f

# View boot timeline
systemd-analyze plot > boot.svg

# Check critical chains
systemd-analyze critical-chain argo_launch_standard.service

# Verify all Argo services
for svc in argo_power_control argo_battery_water argo_bno085 argo_health_monitor argo_radio_servo_module argo_launch_standard; do
    echo "=== $svc ==="
    systemd-analyze verify /etc/systemd/system/$svc.service 2>&1 | head -5
done
```

### Getting Help

For more detailed documentation:
- [Systemd Integration](../launch/README.md)
- [Power Control System](../power_control/README.md)
- [Node Documentation](../nodes/README.md)
- [Argo Main README](../README.md)

## Service Management Commands

### Start/Stop/Restart Individual Services

```bash
# Power control
sudo systemctl start argo_power_control.service
sudo systemctl stop argo_power_control.service
sudo systemctl restart argo_power_control.service

# Battery/Water
sudo systemctl start argo_battery_water.service
sudo systemctl stop argo_battery_water.service

# BNO085
sudo systemctl start argo_bno085.service
sudo systemctl stop argo_bno085.service

# Health Monitor
sudo systemctl start argo_health_monitor.service
sudo systemctl stop argo_health_monitor.service

# Launch (main ROS2 system)
sudo systemctl start argo_launch_standard.service
sudo systemctl stop argo_launch_standard.service
sudo systemctl restart argo_launch_standard.service
```

### Using Argo CLI Aliases

After installing the Argo CLI (`make install-argo-cli`):

```bash
# Launch/quit/restart the main system
al     # Start argo_launch_standard.service
aq     # Stop argo_launch_standard.service
ars    # Restart argo_launch_standard.service

# Status checks
as     # Show comprehensive system status
alog   # View logs with color-coding

# See all available commands
ah     # Show help for Argo aliases
```

## Systemd Directive Reference

### Common Directives Used in Argo Services

**After=X**
- Start this service AFTER X has started
- Ordering only (not a dependency)
- Example: `After=network.target` waits for network

**Before=X**
- Start this service BEFORE X starts
- Forces X to wait for this service
- Example: `Before=shutdown.target` for late shutdown

**Wants=X**
- Try to start X when starting this service
- Weak dependency (if X fails, this service still starts)
- Example: `Wants=argo_battery_water.service`

**WantedBy=X** (in [Install] section)
- X wants this service to start
- Creates reverse Wants relationship
- Example: `WantedBy=multi-user.target`

**Requires=X**
- X MUST start successfully for this service to start
- Strong dependency (not used in Argo services)

## Known Issues and Work In Progress

### WIP: Web Dashboard I2C Failure Display (Dec 2025)

**Issue**: The web dashboard may not correctly reflect I2C failure status from the battery service in all scenarios.

**Background**: 
- Battery service publishes I2C failure status on `/argo/critical/i2c_failure` topic
- Dashboard subscribes to this topic for real-time I2C failure detection
- Service call to battery service for status queries does NOT include I2C failure state
- Dashboard should rely ONLY on topic subscription for I2C failure status

**Current Implementation**:
- Dashboard correctly subscribes to `/argo/critical/i2c_failure` topic
- Comments added to clarify that service call timeout is different from I2C failure
- I2C failure state managed exclusively by topic subscription callback

**Potential Issues**:
- Dashboard may not show I2C failure if it starts before battery service publishes first status
- Topic subscription timing may cause brief delay in displaying I2C failures
- Battery service periodically republishes I2C status (every 5 seconds) to help late subscribers

**Workaround**: 
- Battery service now republishes I2C failure state periodically
- Dashboard will catch up within 5 seconds of connecting

**Future Work**:
- Verify dashboard correctly displays I2C failures in all scenarios
- Add explicit I2C failure status to battery service query response (if needed)
- Consider using ROS2 QoS settings (TRANSIENT_LOCAL) for late-joining subscribers

## Summary

The Argo systemd service architecture provides:

✅ **Clean dependency graph** - No circular dependencies
✅ **Safe boot sequence** - Essential services start in correct order
✅ **Late shutdown** - Critical services stay active longer
✅ **Graceful degradation** - Soft dependencies where appropriate
✅ **Easy troubleshooting** - Clear logs and status information
✅ **Graceful shutdown** - Services stop cleanly in 1-2 seconds with proper signal handling

All services work together to provide a reliable, safe autonomous sailboat control system.









