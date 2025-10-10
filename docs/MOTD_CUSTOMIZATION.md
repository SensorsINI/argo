# Argo MOTD Customization

## Overview

The Argo system includes a custom Message of the Day (MOTD) script that displays critical battery shutdown information and system shutdown events when logging in via SSH. This provides immediate visibility into why the system may have shut down or halted.

## Features

The Argo MOTD script (`15-argo-shutdown-status`) checks for and displays:

1. **Critical Battery Shutdown Alerts**
   - Detects if system was halted due to critical battery voltage
   - Shows battery voltage at time of shutdown
   - Displays timestamp of critical event
   - Shows critical battery flag file if present

2. **Low Battery Warnings**
   - Detects low battery warnings from previous session
   - Shows when SOS LED pattern was activated
   - Displays battery voltage that triggered warning

3. **Normal Shutdown Events**
   - Shows normal power button initiated shutdowns
   - Displays shutdown timestamp

4. **Current Battery Status**
   - Queries current battery voltage (if Argo system is running)
   - Color-coded warnings:
     - **RED**: Critical (< 7.2V) - "Charge battery immediately!"
     - **YELLOW**: Low (< 7.6V) - "Consider charging soon"
     - **CYAN**: Moderate (< 8.0V)
     - Good voltage (≥ 8.0V) - not displayed to keep MOTD clean

## Installation

### Quick Install
```bash
cd ~/argo
make install-motd
```

This will:
- Copy the MOTD script to `/etc/update-motd.d/15-argo-shutdown-status`
- Make it executable
- Enable it for all SSH logins

### Test Before Installing
```bash
cd ~/argo
make test-motd
```

This shows what the MOTD will display without actually installing it.

### Uninstall
```bash
cd ~/argo
make uninstall-motd
```

## How It Works

### Integration with Power Control System

The MOTD script integrates with `argo_power_control.py` to detect critical battery events:

1. **Critical Battery Flag**: When `argo_power_control.py` detects critical battery voltage (< 7.2V), it:
   - Creates flag file: `/tmp/argo_critical_battery`
   - Writes timestamp and critical event details
   - Initiates system halt

2. **Journal Parsing**: The MOTD script searches `journalctl` for:
   - `argo-power-control.service` logs
   - `systemd-shutdown` logs
   - Power button shutdown messages
   - Battery warning events

3. **ROS2 Service Integration**: If Argo system is running, MOTD:
   - Calls `/battery_status` ROS2 service
   - Extracts current battery voltage
   - Displays color-coded status if concerning

### MOTD Script Location

- **Source**: `/home/orangepi/argo/scripts/15-argo-shutdown-status`
- **Installed**: `/etc/update-motd.d/15-argo-shutdown-status`
- **Execution Order**: Runs after system info (30-orangepi-sysinfo) but before tips (35-orangepi-tips)

### Search Strategy

The script checks journalctl logs for the past 7 days:

```bash
journalctl -u argo-power-control.service -u systemd-shutdown --since "7 days ago" --no-pager -r
```

Priority order:
1. Critical battery shutdown (highest priority)
2. Low battery warning
3. Normal power button shutdown
4. Generic systemd shutdown

## Example Output

### Critical Battery Shutdown
```
⚠ CRITICAL BATTERY SHUTDOWN DETECTED
  Battery voltage: 6.8V
  Event time: Oct 10 08:23:15
  Action required: Check battery health and charging system
```

### Low Battery Warning
```
⚠ LOW BATTERY WARNING (previous session)
  Battery voltage: 7.4V
  Event time: Oct 10 07:45:22
```

### Normal Shutdown
```
✓ Normal shutdown via power button
  Event time: Oct 10 09:15:30
```

### Current Battery Status (Critical)
```
⚠ Current battery: 6.9V (CRITICAL - below 7.2V)
  Charge battery immediately!
```

### Critical Battery Flag File
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
⚠ CRITICAL BATTERY FLAG DETECTED
  System was halted due to critical battery voltage
  Flag file: /tmp/argo_critical_battery
  CRITICAL_BATTERY_DETECTED
  Timestamp: 2025-10-10T08:23:15.123456
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Color Coding

The script uses ANSI color codes for visual clarity:

- **Red** (`\e[0;91m`): Critical alerts, critical battery
- **Yellow** (`\e[0;93m`): Warnings, low battery
- **Green** (`\e[0;92m`): Normal operations
- **Cyan** (`\e[0;96m`): Informational messages

## Customization

### Disabling the MOTD Script

To temporarily disable without uninstalling:

```bash
# Add to /etc/default/orangepi-motd
MOTD_DISABLE="argo-shutdown-status"
```

### Adjusting Time Window

Edit the script to change the search window (default: 7 days):

```bash
sudo vim /etc/update-motd.d/15-argo-shutdown-status

# Change line:
journalctl -u argo-power-control.service -u systemd-shutdown --since "7 days ago" ...
# To (for example, 30 days):
journalctl -u argo-power-control.service -u systemd-shutdown --since "30 days ago" ...
```

### Adding Custom Checks

The MOTD script can be extended to check for other events:

```bash
# Add to check_shutdown_event() function
if echo "$shutdown_info" | grep -q "YOUR_CUSTOM_EVENT"; then
    # Custom handling
    echo -e "${YELLOW}Custom event detected${RESET}"
    return 0
fi
```

## Troubleshooting

### MOTD Not Showing

1. **Check if installed**:
   ```bash
   ls -la /etc/update-motd.d/15-argo-shutdown-status
   ```

2. **Verify it's executable**:
   ```bash
   sudo chmod +x /etc/update-motd.d/15-argo-shutdown-status
   ```

3. **Test manually**:
   ```bash
   sudo /etc/update-motd.d/15-argo-shutdown-status
   ```

### Journal Truncation Errors

If you see "Journal file is truncated" errors:
```bash
# This is usually harmless and can be ignored
# To fix persistently:
sudo journalctl --vacuum-time=7d
```

### ROS2 Service Timeout

If battery status check times out:
- Ensure `argo-launch.service` is running
- Check ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Verify battery service exists: `ros2 service list | grep battery_status`

### Permission Issues

MOTD scripts run during login and need proper permissions:
```bash
# Fix permissions
sudo chown root:root /etc/update-motd.d/15-argo-shutdown-status
sudo chmod 755 /etc/update-motd.d/15-argo-shutdown-status
```

## Integration with Argo System

### Power Control Integration

The MOTD script works seamlessly with `argo_power_control.py`:

- **Critical Battery Detection**: Power control sets flag file before halt
- **Journal Logging**: All events logged to systemd journal
- **Status Persistence**: Critical battery flag survives reboot

### Battery Monitoring Integration

Current battery status is retrieved via:
```bash
ros2 service call /battery_status std_srvs/srv/Trigger
```

Response parsing:
- Extracts JSON from service message
- Parses `battery_voltage` field
- Applies threshold checks (7.2V critical, 7.6V low)

## Best Practices

1. **Keep Installed**: Install MOTD on all production systems
2. **Monitor Logs**: Check journalctl regularly for patterns
3. **Battery Maintenance**: Act on low/critical battery warnings immediately
4. **Test Periodically**: Run `make test-motd` to verify functionality

## Related Documentation

- [Power Control System](../power_control/README.md)
- [Battery Monitoring](../nodes/battery_water.py)
- [Argo Lifecycle Management](../launch/README.md)

## Version History

- **v1.0** (2025-10-10): Initial release with critical battery detection
  - Critical battery shutdown alerts
  - Low battery warnings
  - Normal shutdown detection
  - Current battery status integration

