# Persistent Monitoring Services Documentation

This document provides a comprehensive overview of all services that generate log files in `/var/log.hdd/persistent/` on the Argo autonomous sailboat system.

## Overview

The Argo system uses a sophisticated logging architecture that combines Argo-specific monitoring services with system-level monitoring services. All logs are stored in `/var/log.hdd/persistent/` to ensure they survive system reboots and provide persistent diagnostic information.

## Log Directory Structure

The Argo system uses a dual logging architecture:

### Persistent Logging (`/var/log.hdd/persistent/`)
**Purpose**: Logs that survive system reboots and provide long-term diagnostic information
- **Daily logs**: `service-name-YYYYMMDD.log`
- **Timestamped logs**: `service-name-YYYYMMDD-HHMMSS.log`
- **CSV data**: `service-name-YYYYMMDD.csv`

### System Logging (`/var/log.hdd/` and `/var/log/`)
**Purpose**: Standard system logs managed by rsyslog and systemd journal
- **System logs**: `/var/log.hdd/syslog`, `/var/log.hdd/kern.log`, etc.
- **Journal logs**: Managed by systemd-journald with rotation
- **Service logs**: Some services log to both persistent and system logs

### Logging Configuration Decision Matrix

| Service Type | Persistent Logging | System Logging | Reason |
|--------------|-------------------|----------------|---------|
| **Argo Monitoring Services** | ✅ Required | ✅ Optional | Critical for diagnostics |
| **System Services** | ❌ Not needed | ✅ Standard | Managed by systemd/journald |
| **Development Services** | ✅ Recommended | ✅ Optional | Debugging and analysis |
| **Boot-time Services** | ✅ Required | ✅ Optional | Boot diagnostics |

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

**Node**: `nodes/argo_battery_water.py` (ROS2 node)  
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
- `argo-power-control.log` - Persistent logging to `/var/log.hdd/persistent/`
- **Systemd Journal**: Also logs to systemd journal (rotated)

**Purpose**: Power button and LED control, system health monitoring  
**Features**:
- GPIO management for power button and LEDs
- Graceful shutdown procedures
- System health monitoring
- **Dual Logging**: Both persistent files and systemd journal

## System-Level Monitoring Services (Optional)

**Note**: These services are **NOT installed by default** and are available as optional debugging tools in the `system-monitoring/` directory.

### Installation
```bash
# Install all system monitoring services
make install-system-monitoring

# Or install individually
make -C system-monitoring install-all
```

### 1. Boot History Logger Service

**Service**: `boot-history-logger.service`  
**Script**: `system-monitoring/scripts/boot-logger.sh`  
**Installation**: `make -C system-monitoring install-boot-logger`

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
**Script**: `system-monitoring/scripts/cursor-monitor.sh`  
**Installation**: `make -C system-monitoring install-cursor-monitor`

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
**Script**: `system-monitoring/scripts/memory-monitor.sh`  
**Installation**: `make -C system-monitoring install-memory-monitor`

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
**Installation**: `make -C system-monitoring install-orangepi-monitor`

**Log Files**:
- `orangepi-monitor-YYYYMMDD.log`

**Purpose**: Orange Pi hardware monitoring using `orangepimonitor` tool  
**Features**:
- Hardware-specific monitoring for Orange Pi Zero 2W
- System resource tracking
- Hardware health monitoring

### 5. Persistent dmesg Service

**Service**: `persistent-dmesg.service`  
**Installation**: `make -C system-monitoring install-persistent-dmesg`

**Log Files**:
- `dmesg-YYYYMMDD-HHMMSS.log`

**Purpose**: Captures kernel messages with timestamps  
**Features**:
- Runs once per boot
- Creates timestamped dmesg logs
- Preserves kernel messages for debugging

### 6. Crash Dump Service

**Service**: `crash-dump.service`  
**Installation**: `make -C system-monitoring install-crash-dump`

**Purpose**: Copies crash dumps to persistent storage  
**Features**:
- Automatically copies crash dumps from `/var/crash/`
- Preserves crash information across reboots
- Runs once per boot

### 7. Temperature Logger Service

**Service**: `temp-logger.service` + `temp-logger.timer`  
**Script**: `system-monitoring/scripts/temp-logger.sh`  
**Installation**: `make -C system-monitoring install-temp-logger`

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

## Log Rotation Configuration

### Systemd Journal Rotation
The systemd journal is configured in `/etc/systemd/journald.conf`:
```ini
[Journal]
Storage=persistent
Compress=yes
RateLimitIntervalSec=30s
RateLimitBurst=10000
SystemMaxUse=500M
SystemKeepFree=1G
MaxRetentionSec=2week
ForwardToSyslog=no
```

**Key Settings**:
- **Storage**: `persistent` - Logs stored in `/var/log/journal/`
- **SystemMaxUse**: `500M` - Maximum disk usage for journal
- **MaxRetentionSec**: `2week` - Maximum retention time
- **Compress**: `yes` - Compress old journal files

### Logrotate Configuration
Log rotation is managed by `/etc/logrotate.conf` and `/etc/logrotate.d/`:

#### Global Settings (`/etc/logrotate.conf`)
```bash
# Rotate log files weekly
weekly

# Use the adm group by default
su root adm

# Keep 4 weeks worth of backlogs
rotate 4

# Create new (empty) log files after rotating old ones
create

# Compress rotated files
compress
```

#### Persistent Logs (`/etc/logrotate.d/persistent-logs`)
```bash
/var/log.hdd/persistent/*.log {
    daily
    missingok
    rotate 2
    compress
    delaycompress
    create 644 root root
    su root root
}
```

#### System Logs (`/etc/logrotate.d/rsyslog`)
```bash
/var/log.hdd/syslog
/var/log.hdd/mail.info
/var/log.hdd/mail.warn
/var/log.hdd/mail.err
/var/log.hdd/mail.log
/var/log.hdd/daemon.log
/var/log.hdd/kern.log
/var/log.hdd/auth.log
/var/log.hdd/user.log
/var/log.hdd/lpr.log
/var/log.hdd/cron.log
/var/log.hdd/debug
/var/log.hdd/messages
{
    rotate 4
    weekly
    missingok
    notifempty
    compress
    delaycompress
    sharedscripts
    postrotate
        /usr/lib/rsyslog/rsyslog-rotate
    endscript
}
```

### Log Rotation Schedule

| Log Type | Rotation Frequency | Retention | Compression | Location |
|----------|-------------------|-----------|-------------|----------|
| **Persistent Logs** | Daily | 2 days | Yes (delayed) | `/var/log.hdd/persistent/` |
| **System Logs** | Weekly | 4 weeks | Yes (delayed) | `/var/log.hdd/` |
| **Systemd Journal** | Automatic | 2 weeks | Yes | `/var/log/journal/` |
| **Boot History** | Daily | 2 days | Yes (delayed) | `/var/log.hdd/persistent/` |

### Log Rotation Troubleshooting

#### Check Logrotate Status
```bash
# Test logrotate configuration
sudo logrotate -d /etc/logrotate.conf

# Force log rotation
sudo logrotate -f /etc/logrotate.conf

# Check logrotate logs
sudo cat /var/log/logrotate.log
```

#### Check Journal Status
```bash
# Check journal disk usage
journalctl --disk-usage

# List available boots
journalctl --list-boots

# Check journal configuration
systemctl status systemd-journald
```

#### Manual Log Rotation
```bash
# Rotate specific log files
sudo logrotate -f /etc/logrotate.d/persistent-logs

# Check rotated files
ls -la /var/log.hdd/persistent/*.log*
```

### Service-Specific Logging Configuration

#### Services with Persistent Logging
These services are configured to log to `/var/log.hdd/persistent/`:

1. **argo_power_control.service**:
   ```ini
   StandardOutput=append:/var/log.hdd/persistent/argo-power-control.log
   StandardError=append:/var/log.hdd/persistent/argo-power-control.log
   ```

2. **argo_thermal_monitor.service**: Uses shell script with `>>` redirection

3. **Boot History Logger**: Uses shell script with `>>` redirection

#### Services with System Logging Only
These services log only to systemd journal:
- Most system services
- Services without explicit logging configuration
- Services using `StandardOutput=journal`

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
# Check Argo service status
systemctl status argo_thermal_monitor.service

# Check system monitoring services (if installed)
make -C system-monitoring status

# View Argo service logs
journalctl -u argo_thermal_monitor.service -f

# View system monitoring service logs (if installed)
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

| Log File Pattern | Generating Service | Purpose | Frequency | Logging Type |
|------------------|-------------------|---------|-----------|--------------|
| `thermal-YYYYMMDD.log` | `argo_thermal_monitor.service` | Argo thermal monitoring | 30s | Persistent |
| `battery-monitor-YYYYMMDD.csv` | `argo_battery_water.py` (ROS2) | Battery/sensor data | 30s | Persistent |
| `wifi-reconnect.log` | `argo_wifi_reconnect.service` | WiFi management | 5min | Persistent |
| `argo-power-control.log` | `argo_power_control.service` | Power control | Event-based | **Dual** (Persistent + Journal) |
| `boot-history.log` | `boot-history-logger.service` | Boot events | Per boot | Persistent |
| `dmesg-YYYYMMDD.log` | `boot-history-logger.service` | Boot-time kernel messages | Per boot | Persistent |
| `dmesg-YYYYMMDD-HHMMSS.log` | `persistent-dmesg.service` | Timestamped kernel messages | Per boot | Persistent |
| `journalctl-YYYYMMDD-HHMMSS.log` | `boot-history-logger.service` | Boot-time systemd journal | Per boot | Persistent |
| `cursor-processes-YYYYMMDD.log` | `cursor-monitor.service` | Cursor IDE monitoring | 60s | Persistent |
| `memory-YYYYMMDD.log` | `memory-monitor.service` | Memory usage | 30s | Persistent |
| `processes-YYYYMMDD.log` | `memory-monitor.service` | Process monitoring | 30s | Persistent |
| `orangepi-monitor-YYYYMMDD.log` | `orangepi-monitor.service` | Orange Pi hardware monitoring | Continuous | Persistent |

### Logging Type Legend
- **Persistent**: Logs to `/var/log.hdd/persistent/` - survives reboots
- **Dual**: Logs to both persistent files and systemd journal
- **Journal**: Logs only to systemd journal (rotated, may be lost)

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
