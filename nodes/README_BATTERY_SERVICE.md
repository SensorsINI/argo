# Battery Water Independent Service

## Overview

The `battery_water.service` runs as an **independent systemd service** to provide critical battery monitoring that operates continuously, even when the main Argo system (`argo-launch.service`) is stopped.

This architecture ensures **fail-safe operation**: critical battery monitoring never stops, allowing the power control system to detect critically low battery and initiate emergency halt to preserve power for manual sailing via radio control.

## Architecture

### Service Dependencies
```
argo_power_control.service  (Independent)
    ↓ (monitors battery via ROS2 service)
battery_water.service       (Independent)
    ↓ (excluded from lifecycle management)
argo-launch.service         (Managed by lifecycle)
```

### Key Design Points
1. **Independent Operation**: battery_water runs as its own systemd service
2. **Not Managed**: Excluded from argo_lifecycle_manager control
3. **Always Running**: Continues operating when argo-launch is stopped
4. **Critical Safety**: Enables critical battery monitoring at all times
5. **Auto-Restart**: Service restarts automatically if it fails

## Installation

### Quick Install
```bash
cd /home/orangepi/argo/nodes
sudo make install enable start
```

### Step-by-Step
```bash
# Check dependencies
make check

# Install service
sudo make install

# Enable auto-start on boot
sudo make enable

# Start service
sudo make start

# Check status
make status
```

## Service Management

### Status Commands
```bash
make status        # Show service status
make logs          # Show last 50 log lines
make logs-follow   # Follow logs in real-time
```

### Control Commands
```bash
sudo make start    # Start service
sudo make stop     # Stop service
sudo make restart  # Restart service
sudo make enable   # Enable auto-start
sudo make disable  # Disable auto-start
```

### Uninstall
```bash
sudo make uninstall
```

## Verification

### Check Service is Running
```bash
systemctl is-active battery_water.service
# Should output: active
```

### Check Battery Data is Available
```bash
bash -c 'source /opt/ros/humble/setup.bash && ros2 service call /battery_status std_srvs/srv/Trigger'
# Should return battery voltage, remaining %, and sensor data
```

### Verify Independent Operation
```bash
# Stop main Argo system
aq

# Check battery service is still running
systemctl is-active battery_water.service
# Should output: active

# Battery data should still be available
bash -c 'source /opt/ros/humble/setup.bash && ros2 service call /battery_status std_srvs/srv/Trigger'
# Should still work!
```

## Integration with Argo Status

The `argo_lifecycle_manager.py` status command now shows battery service status separately:

```bash
python3 /home/orangepi/argo/launch/argo_lifecycle_manager.py status
```

Output includes:
```
⚡ POWER CONTROL: 🟢 RUNNING
🔋 BATTERY MONITOR: 🟢 RUNNING    ← Independent service status
📋 LAUNCH SERVICE: 🟢 RUNNING
```

## Critical Battery Monitoring

With this architecture, the power control service can **always** monitor battery voltage:

1. **Power control** calls `/battery_status` service every 30 seconds
2. **Battery service** is always running (independent of argo-launch)
3. **Critical threshold**: If voltage < 6.5V, system initiates emergency halt
4. **Power preservation**: Shutdown hook preserves power relay for manual sailing
5. **Radio control**: Boat remains manually controllable via radio

## Service Configuration

The service file is located at `/etc/systemd/system/battery_water.service`:

```ini
[Unit]
Description=Argo Battery and Water Monitoring Service
After=basic.target sysinit.target
Before=argo-launch.service shutdown.target

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi/argo/nodes
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /home/orangepi/argo/nodes/battery_water.py'
Restart=always
RestartSec=5

[Install]
WantedBy=basic.target
```

Key points:
- **Restart=always**: Automatically restarts if it fails
- **Before=argo-launch.service**: Starts before main Argo system
- **User=orangepi**: Runs as orangepi user (not root)

## Troubleshooting

### Service Won't Start
```bash
# Check service logs
sudo journalctl -u battery_water.service -n 50

# Check for I2C hardware
i2cdetect -y 0
# Should show devices at 0x34 (ADC) and 0x44 (SHT45)
```

### Service Keeps Restarting
```bash
# Check for fatal errors
sudo journalctl -u battery_water.service | grep FATAL

# Check I2C bus
i2cdetect -y 0

# Check Python dependencies
bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import rclpy, smbus2, gpiod"'
```

### Battery Data Not Available
```bash
# Check if service is running
systemctl is-active battery_water.service

# Check ROS2 service is available
bash -c 'source /opt/ros/humble/setup.bash && ros2 service list | grep battery'

# Try calling service
bash -c 'source /opt/ros/humble/setup.bash && ros2 service call /battery_status std_srvs/srv/Trigger'
```

## Benefits

### Before (battery_water managed by lifecycle)
- ❌ Battery monitoring stops when argo-launch stops
- ❌ Critical battery detection fails when system is offline  
- ❌ No failsafe if main system is shut down

### After (battery_water as independent service)
- ✅ Battery monitoring always running
- ✅ Critical battery detection always works
- ✅ True failsafe operation
- ✅ Emergency halt works even when robot offline
- ✅ Power preserved for manual sailing in critical scenarios

## Files

- `battery_water.py` - Main battery monitoring ROS2 node
- `battery_water.service` - Systemd service configuration
- `Makefile` - Installation and management commands
- `README_BATTERY_SERVICE.md` - This documentation

## See Also

- [argo-power-management.mdc](../.cursor/rules/argo-power-management.mdc) - Power management patterns and critical battery monitoring
- [power_control/argo_power_control.py](../power_control/argo_power_control.py) - Power control service that monitors battery
- [power_control/argo_poweroff.shutdown](../power_control/argo_poweroff.shutdown) - Shutdown hook that preserves power on critical battery


