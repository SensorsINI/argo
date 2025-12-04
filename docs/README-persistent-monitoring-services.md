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

### 5. Argo Persistent Log Manager

**Service**: `argo-persistent-logs.service` + `argo-persistent-logs-prune.timer`  
**Script**: `/usr/local/bin/argo_persistent_log_manager.sh`  
**Installation**: `make -C system-monitoring install-persistent-log-manager`

**Log Files**:
- `argo-*.log` (live service logs)
- `argo-*-YYYYMMDD-HHMMSS-<BOOT_ID>.boot.log` (per-boot snapshots)
- `journalctl-<BOOT_ID>.log` (per-boot journal export)

**Purpose**: Guarantees that Argo service logs survive reboots and are only rotated when either a reboot occurs or the persistent log partition exceeds a configurable usage threshold (85% by default).

**Features**:
- Rotates each Argo service log at boot before dependent services start
- Maintains boot-specific log snapshots for retrospective analysis
- Prunes oldest log artifacts only when persistent storage usage crosses the threshold
- Disables the legacy `/etc/logrotate.d/persistent-logs` daily rotation policy
- Hourly pruning timer that does nothing unless the threshold is exceeded

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
**Script**: `/usr/local/bin/boot-logger.sh`  
**Installation**: `make -C system-monitoring install-boot-logger`

**Log Files**:
- `boot-history.log` - Boot event log (appends all boots)
- `dmesg-YYYYMMDD.log` - Daily dmesg (appends all boots from that day)
- `journalctl-YYYYMMDD-HHMMSS.log` - Per-boot journal backup (timestamped)

**Purpose**: Logs boot events and captures boot-time system messages  
**Features**:
- Records boot timestamps and uptime
- Captures dmesg output at boot time
- Saves systemd journal for each boot
- Runs once per boot via systemd service
- **Installed**: 2025-10-08 to enable persistent boot logging across reboots

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

### 8. Shutdown Logger Service

**Service**: `shutdown-logger.service`  
**Script**: `system-monitoring/scripts/shutdown-logger.sh`  
**Installation**: `make -C system-monitoring install-shutdown-logger`

**Log Files**:
- `shutdown-YYYYMMDD-HHMMSS.log` - Shutdown event log with service stop times and timing information

**Purpose**: Captures shutdown events, service stop times, and timing information to diagnose slow shutdowns  
**Features**:
- Runs early during shutdown (before services start stopping)
- Logs active systemd jobs and services being stopped
- Records service stop timeouts (identifies services with long TimeoutStopSec)
- Captures shutdown target information
- Detects critical battery shutdown flags
- Performs filesystem sync timing
- Logs to persistent storage, console, and kernel message buffer
- **Critical for diagnosing 2+ minute shutdown delays**

**Note**: This service is essential for diagnosing slow shutdowns. The systemd journal stops writing early in shutdown, so shutdown events are not captured in journalctl logs. This service fills that gap.

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
- **Storage**: `persistent` - Logs stored in `/var/log/journal/` (changed from `volatile` on 2025-10-08 to preserve boot logs across reboots)
- **SystemMaxUse**: `500M` - Maximum disk usage for journal
- **MaxRetentionSec**: `2week` - Maximum retention time
- **Compress**: `yes` - Compress old journal files

**Configuration History**:
- **2025-10-08**: Changed `Storage` from `volatile` to `persistent` to ensure boot logs survive reboots. Journal location: `/var/log/journal/ce4def9bd6a04cd594e94f8205bbe671/`

### Persistent Log Manager Configuration

- The `install-persistent-log-manager` target **disables** the legacy `/etc/logrotate.d/persistent-logs` policy.
- `/usr/local/bin/argo_persistent_log_manager.sh` rotates Argo service logs at boot and creates timestamped `*.boot.log` snapshots.
- The companion timer (`argo-persistent-logs-prune.timer`) triggers an hourly check that only removes files when the persistent log partition exceeds the configurable threshold (default: 85%).
- System logs outside the persistent directory continue to be rotated by the stock logrotate configuration in `/etc/logrotate.d/rsyslog`.

### Log Rotation Schedule

| Log Type | Rotation Frequency | Retention | Compression | Location |
|----------|-------------------|-----------|-------------|----------|
| **Persistent Logs** | On boot + threshold | Oldest snapshots pruned when usage ≥ threshold | Optional (manual) | `/var/log.hdd/persistent/` |
| **System Logs** | Weekly (logrotate) | 4 weeks | Yes (delayed) | `/var/log.hdd/` |
| **Systemd Journal** | Automatic | 2 weeks | Yes | `/var/log/journal/` |
| **Boot History** | Per boot | Unlimited (timestamped files) | No (plain text) | `/var/log.hdd/persistent/` |

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

#### Manual Log Maintenance
```bash
# Force a prune check (uses current threshold)
sudo /usr/local/bin/argo_persistent_log_manager.sh --prune

# Rotate service logs immediately (e.g. before manual testing)
sudo /usr/local/bin/argo_persistent_log_manager.sh --rotate-on-boot
```

### Service-Specific Logging Configuration

#### Services with Persistent Logging
These services are configured to log to `/var/log.hdd/persistent/`:

1. **argo_power_control.service**:
   ```ini
   Environment=PYTHONUNBUFFERED=1
   ExecStart=/bin/bash -c 'set -eo pipefail; source /opt/ros/humble/setup.bash && \
     /usr/bin/python3 /home/orangepi/argo/power_control/argo_power_control.py |& \
     tee -a /var/log.hdd/persistent/argo-power-control.log'
   StandardOutput=journal
   StandardError=journal
   ```

2. **argo_battery_water.service** and **argo_health_monitor.service** follow the same pattern using `tee -a` to append to their respective persistent log files while keeping journald output.

3. **argo_bno085.service** uses the launcher script and pipes combined stdout/stderr through `tee -a /var/log.hdd/persistent/argo-bno085.log`.

4. **argo_launch_standard.service** logs to `argo-launch-standard.log` using the same `tee` approach and retains journald output for `journalctl` consumers.

5. **argo_thermal_monitor.service** and **boot-history-logger.service** use shell redirection (`>>`) to append directly to persistent files.

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
| `argo-battery-water.log` | `argo_battery_water.service` | Battery + water monitoring | Event-based | **Dual** |
| `argo-health-monitor.log` | `argo_health_monitor.service` | Node health aggregation | Event-based | **Dual** |
| `argo-launch-standard.log` | `argo_launch_standard.service` | ROS2 launch orchestration | Event-based | **Dual** |
| `argo-bno085.log` | `argo_bno085.service` | IMU driver + bridge | Event-based | **Dual** |
| `argo-radio-servo-module.log` | `argo_radio_servo_module.service` | Kernel module loader | Oneshot | **Dual** |
| `argo-*-YYYYMMDD-HHMMSS-<BOOT_ID>.boot.log` | `argo-persistent-logs.service` | Per-boot service snapshots | Per boot | Persistent |
| `boot-history.log` | `boot-history-logger.service` | Boot events (aggregate) | Per boot | Persistent |
| `boot-history-YYYYMMDD-HHMMSS-<BOOT_ID>.log` | `boot-history-logger.service` | Boot event snapshot | Per boot | Persistent |
| `dmesg-YYYYMMDD.log` | `boot-history-logger.service` | Boot-time kernel messages | Per boot | Persistent |
| `dmesg-YYYYMMDD-HHMMSS.log` | `persistent-dmesg.service` | Timestamped kernel messages | Per boot | Persistent |
| `journalctl-YYYYMMDD-HHMMSS-<BOOT_ID>.log` | `boot-history-logger.service` | Boot-time systemd journal | Per boot | Persistent |
| `journalctl-<BOOT_ID>.log` | `boot-history-logger.service` | Previous boot journal snapshots | Per boot | Persistent |
| `shutdown-YYYYMMDD-HHMMSS.log` | `shutdown-logger.service` | Shutdown events and service stop times | Per shutdown | Persistent |
| `shutdown-hook-YYYYMMDD-HHMMSS.log` | `argo_poweroff.shutdown` | Shutdown hook execution (late phase) | Per shutdown | Persistent |
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

### System Hang Diagnosis

#### Case Study: Disk I/O Storm (October 7, 2025 ~08:59 AM)

**Root Cause**: Disk I/O storm causing system hang

**Timeline**:
- **08:57:57**: Normal - Load 3.32, CPU 85%, I/O 3%
- **08:58:39**: I/O storm begins - Load 6.62, CPU 97%, I/O 67%
- **08:59:45**: Critical - Load 23.08, CPU 97%, I/O 67%
- **Result**: System hung due to I/O wait

**Contributing Factors**:
- 67% I/O wait indicates all processes blocked on disk
- Load jumped 7x (3.32 → 23.08) in 2 minutes
- Likely causes:
  - Heavy file indexing/search
  - Log rotation or sync operations
  - Database/cache updates
- Low memory conditions (35MB free) may have triggered swapping

#### Investigating System Hangs

**Use These Commands**:
```bash
# Check boot history across reboots
cat /var/log.hdd/persistent/boot-history.log

# View journal from previous boots
sudo journalctl --list-boots
sudo journalctl -b -1  # Previous boot
sudo journalctl -b -2  # Two boots ago

# Check persistent dmesg logs
cat /var/log.hdd/persistent/dmesg-$(date +%Y%m%d).log

# View monitoring data before hang
cat /var/log.hdd/persistent/orangepi-monitor-$(date +%Y%m%d).log.1
```

**What to Look For**:
1. High I/O wait percentages (>50%)
2. Load average spikes (>10)
3. Memory exhaustion (free <50MB)
4. OOM killer events
5. Disk errors or timeouts
6. Heavy cursor/indexing processes

### Verification

Test that logging works across reboots:
```bash
# Before reboot - note current boot ID
sudo journalctl --list-boots

# After reboot - verify previous boot is preserved
sudo journalctl --list-boots
sudo journalctl -b -1 | head -50
cat /var/log.hdd/persistent/boot-history.log
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
