# Persistent Monitoring Services Documentation

This document provides a comprehensive overview of all services that generate log files in `/var/log.hdd/persistent/` on the Argo autonomous sailboat system.

## Overview

The Argo system uses a sophisticated logging architecture that combines Argo-specific monitoring services with system-level monitoring services. All logs are stored in `/var/log.hdd/persistent/` to ensure they survive system reboots and provide persistent diagnostic information.

## Log Directory Structure

All monitoring services write to `/var/log.hdd/persistent/` with the following naming conventions:
- **Daily logs**: `service-name-YYYYMMDD.log`
- **Timestamped logs**: `service-name-YYYYMMDD-HHMMSS.log`
- **CSV data**: `service-name-YYYYMMDD.csv`

## Argo-Specific Monitoring Services

### 1. Thermal Monitor Service

**Service**: `argo_thermal_monitor.service`  
**Script**: `/usr/local/bin/argo_thermal_monitor.sh`  
**Installation**: `make -C launch install_thermal_monitor`  
**Makefile**: `launch/Makefile` (lines 134-177)

**Log Files**:
- `thermal-YYYYMMDD.log`

**Purpose**: Monitors system temperatures every 30 seconds  
**Log Format**:
```
2025-10-01 06:30:23: GPU:60°C VE:57°C CPU:58°C DDR:58°C
```

**Features**:
- Monitors GPU, VE, CPU, and DDR thermal zones
- Logs warnings when temperatures exceed 80°C
- Sends critical alerts to syslog
- Used by Argo lifecycle manager for status checks

### 2. Battery Water Monitoring Service

**Node**: `nodes/battery_water.py` (ROS2 node)  
**Installation**: `make -C scripts install-battery-monitor`  
**Makefile**: `scripts/Makefile` (lines 28-35)

**Log Files**:
- `battery-monitor-YYYYMMDD.csv`

**Purpose**: Monitors battery voltage, current, temperature, humidity, and saltwater intrusion  
**Logging Interval**: Every 30 seconds  
**Data Includes**:
- Battery voltage and state-of-charge
- Saltwater probe voltage
- Sail winch current
- PCB temperature and humidity
- Alert status (battery low, saltwater, humidity)

### 3. WiFi Reconnection Service

**Service**: `argo_wifi_reconnect.service` + `argo_wifi_reconnect.timer`  
**Installation**: `make install-network-improvements`  
**Script**: `network/install/install_network_improvements.sh`

**Log Files**:
- `wifi-reconnect.log`

**Purpose**: Automatically switches to preferred WiFi networks  
**Features**:
- Connection stability checks
- Lock file protection
- Comprehensive logging
- Runs every 5 minutes via systemd timer

### 4. Power Control Service

**Service**: `argo_power_control.service`  
**Installation**: `make -C power_control install`  
**Makefile**: `power_control/Makefile` (lines 27-40)

**Log Files**:
- `argo-power-control.log` (referenced in testing scripts)

**Purpose**: Power button and LED control, system health monitoring  
**Features**:
- GPIO management for power button and LEDs
- Graceful shutdown procedures
- System health monitoring

## System-Level Monitoring Services

### 1. Boot History Logger Service

**Service**: `boot-history-logger.service`  
**Script**: `/usr/local/bin/boot-logger.sh`

**Log Files**:
- `boot-history.log`
- `dmesg-YYYYMMDD.log` (boot-time dmesg)
- `journalctl-YYYYMMDD-HHMMSS.log` (boot-time journal)

**Purpose**: Logs boot events and captures boot-time system messages  
**Features**:
- Records boot timestamps and uptime
- Captures dmesg output at boot time
- Saves systemd journal for each boot
- Runs once per boot via systemd service

### 2. Cursor Process Monitor Service

**Service**: `cursor-monitor.service`  
**Script**: `/usr/local/bin/cursor-monitor.sh`

**Log Files**:
- `cursor-processes-YYYYMMDD.log`

**Purpose**: Monitors Cursor IDE processes and memory usage  
**Monitoring Interval**: Every 60 seconds  
**Features**:
- Tracks Cursor agent and extension processes
- Monitors memory usage
- Alerts when memory usage exceeds 1GB
- Logs process details and memory consumption

### 3. Memory Monitor Service

**Service**: `memory-monitor.service`  
**Script**: `/usr/local/bin/memory-monitor.sh`

**Log Files**:
- `memory-YYYYMMDD.log`
- `processes-YYYYMMDD.log`

**Purpose**: Monitors system memory usage and process activity  
**Monitoring Interval**: Every 30 seconds  
**Features**:
- Tracks memory and swap usage percentages
- Logs detailed memory statistics
- Records top memory-consuming processes
- Alerts on high memory usage (>85%) and high swap usage (>60%)

### 4. Orange Pi Monitor Service

**Service**: `orangepi-monitor.service`

**Log Files**:
- `orangepi-monitor-YYYYMMDD.log`

**Purpose**: Orange Pi hardware monitoring using `orangepimonitor` tool  
**Features**:
- Hardware-specific monitoring for Orange Pi Zero 2W
- System resource tracking
- Hardware health monitoring

### 5. Persistent dmesg Service

**Service**: `persistent-dmesg.service`

**Log Files**:
- `dmesg-YYYYMMDD-HHMMSS.log`

**Purpose**: Captures kernel messages with timestamps  
**Features**:
- Runs once per boot
- Creates timestamped dmesg logs
- Preserves kernel messages for debugging

### 6. Crash Dump Service

**Service**: `crash-dump.service`

**Purpose**: Copies crash dumps to persistent storage  
**Features**:
- Automatically copies crash dumps from `/var/crash/`
- Preserves crash information across reboots
- Runs once per boot

## Temperature Logging Services

### Temperature Logger Service

**Service**: `temp-logger.service` + `temp-logger.timer`  
**Script**: `/usr/local/bin/temp-logger.sh`

**Log Files**: `/var/log/temperature.log` (not in persistent directory)

**Purpose**: Logs thermal zone temperatures (separate from Argo thermal monitor)  
**Features**:
- Runs via systemd timer
- Logs to standard system log directory
- Monitors all thermal zones
- Alerts on high temperatures

## System Integration Services

### Orange Pi Ramlog Service

**Service**: `orangepi-ramlog.service`  
**Fix Script**: `scripts/fix-orangepi-ramlog.sh`  
**Installation**: `make fix-orangepi-ramlog`

**Purpose**: Manages log rotation and preservation in `/var/log.hdd/persistent/`  
**Features**:
- Prevents deletion of rotated log files
- Ensures persistent storage of all logs
- Critical for maintaining log history across reboots

## Service Installation and Management

### Argo Services Installation

```bash
# Install all Argo monitoring services
make -C launch install_thermal_monitor    # Thermal monitoring
make -C scripts install-battery-monitor   # Battery monitoring  
make install-network-improvements         # WiFi reconnection
make -C power_control install             # Power control
make fix-orangepi-ramlog                  # Preserve persistent logs
```

### System Services Management

```bash
# Check service status
systemctl status argo_thermal_monitor.service
systemctl status boot-history-logger.service
systemctl status cursor-monitor.service
systemctl status memory-monitor.service
systemctl status orangepi-monitor.service

# View service logs
journalctl -u argo_thermal_monitor.service -f
journalctl -u boot-history-logger.service -f
journalctl -u cursor-monitor.service -f
journalctl -u memory-monitor.service -f
```

### Log File Monitoring

```bash
# Monitor all persistent logs
tail -f /var/log.hdd/persistent/*.log

# Monitor specific services
tail -f /var/log.hdd/persistent/thermal-$(date +%Y%m%d).log
tail -f /var/log.hdd/persistent/memory-$(date +%Y%m%d).log
tail -f /var/log.hdd/persistent/cursor-processes-$(date +%Y%m%d).log

# View boot history
cat /var/log.hdd/persistent/boot-history.log
```

## Log File Summary

| Log File Pattern | Generating Service | Purpose | Frequency |
|------------------|-------------------|---------|-----------|
| `thermal-YYYYMMDD.log` | `argo_thermal_monitor.service` | Argo thermal monitoring | 30s |
| `battery-monitor-YYYYMMDD.csv` | `battery_water.py` (ROS2) | Battery/sensor data | 30s |
| `wifi-reconnect.log` | `argo_wifi_reconnect.service` | WiFi management | 5min |
| `argo-power-control.log` | `argo_power_control.service` | Power control | Event-based |
| `boot-history.log` | `boot-history-logger.service` | Boot events | Per boot |
| `dmesg-YYYYMMDD.log` | `boot-history-logger.service` | Boot-time kernel messages | Per boot |
| `dmesg-YYYYMMDD-HHMMSS.log` | `persistent-dmesg.service` | Timestamped kernel messages | Per boot |
| `journalctl-YYYYMMDD-HHMMSS.log` | `boot-history-logger.service` | Boot-time systemd journal | Per boot |
| `cursor-processes-YYYYMMDD.log` | `cursor-monitor.service` | Cursor IDE monitoring | 60s |
| `memory-YYYYMMDD.log` | `memory-monitor.service` | Memory usage | 30s |
| `processes-YYYYMMDD.log` | `memory-monitor.service` | Process monitoring | 30s |
| `orangepi-monitor-YYYYMMDD.log` | `orangepi-monitor.service` | Orange Pi hardware monitoring | Continuous |

## Troubleshooting

### Service Not Running

```bash
# Check service status
systemctl status <service-name>

# Restart service
sudo systemctl restart <service-name>

# Enable service for auto-start
sudo systemctl enable <service-name>
```

### Log Files Not Created

```bash
# Check directory permissions
ls -la /var/log.hdd/persistent/

# Check service logs
journalctl -u <service-name> -f

# Verify service is active
systemctl is-active <service-name>
```

### High Log Volume

```bash
# Check log file sizes
du -sh /var/log.hdd/persistent/*

# Clean old logs (be careful!)
find /var/log.hdd/persistent/ -name "*.log" -mtime +30 -delete
```

## Integration with Argo Lifecycle Manager

The Argo lifecycle manager (`launch/argo_lifecycle_manager.py`) integrates with several monitoring services:

- **Thermal logs**: Used for CPU temperature display in status checks
- **Battery logs**: Referenced for power management decisions
- **System logs**: Used for diagnostic information during failures

## Best Practices

1. **Regular Monitoring**: Check log files regularly for system health
2. **Log Rotation**: Monitor disk space usage in persistent storage
3. **Service Health**: Ensure all monitoring services are running
4. **Alert Thresholds**: Configure appropriate alert levels for each service
5. **Backup**: Consider backing up critical log files for long-term analysis

## Related Documentation

- [Argo Lifecycle Management](README.md#node-lifecycle-management)
- [Thermal Monitor Installation](launch/README-thermal-monitor.md)
- [Network Improvements](network/README.md)
- [Power Control System](power_control/README.md)
- [Battery Monitoring](scripts/README-plot-battery-water.md)
