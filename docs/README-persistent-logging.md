# Persistent Logging Documentation

This document explains how Argo’s persistent logging works on the robot (Armbian / Orange Pi), how log rotation + pruning are installed, and why **Argo writes persistent logs to `/var/log.hdd/persistent/`** (not `/var/log/persistent/`).

## How persistent logging is set up (Armbian/Orange Pi + Argo)

Argo runs on an Armbian-derived Orange Pi image that uses **ramlog / log2ram-style logging** to reduce SD/eMMC wear:

### What is `systemd-journald`?

`systemd-journald` (often shortened to “journald”) is systemd’s built-in logging daemon. It collects log messages from:

- the kernel (via `kmsg`)
- systemd services (stdout/stderr)
- syslog clients (optionally)

It stores logs in a **binary journal format** and you query it with `journalctl`.

Common storage locations:
- **Volatile (RAM)**: `/run/log/journal/` (lost on reboot)
- **Persistent (disk)**: `/var/log/journal/` (survives reboot)

Key point for Armbian/ramlog systems: even if journald is configured with `Storage=persistent`, its “persistent” path is still **under `/var/log/journal/`** — and if `/var/log` is RAM-backed (zram/tmpfs), journald can still consume zram unless journald is redirected/symlinked to an on-disk location.

### What is “ramlog” / “log2ram”?

“ramlog” (Armbian/Orange Pi naming) is a mechanism that makes `/var/log` **live in RAM** to reduce flash wear:

- `/var/log/` is mounted on **zram** (compressed RAM) or `tmpfs`
- `/var/log.hdd/` points at the **on-disk** backing store
- a sync step (often `rsync`) copies logs between RAM and disk at boot/shutdown/intervals

The upside is fewer writes to SD/eMMC. The downside is that `/var/log` is **small**, so copying “persistent” logs into `/var/log` (or letting logs grow unbounded in RAM) can fill zram and cause `ENOSPC` issues.

- **`/var/log/`**: a **RAM-backed filesystem** (often zram, e.g. `/dev/zram1`) where “normal” system logs are written during runtime.
  - Fast, reduces flash writes, but **small** and can fill (causing `No space left on device`).
- **`/var/log.hdd/`**: a **bind-mount view** of the *on-disk* `/var/log` directory used as the persistence backing store.
  - The ramlog service syncs logs between `/var/log` (RAM) and `/var/log.hdd` (disk) on boot/shutdown/intervals (implementation varies).

Argo adds an explicit, durable layer on top:

- **`/var/log.hdd/persistent/`**: **Argo’s canonical persistent log directory** (survives reboots, not constrained by zram size).
- **`/var/log/persistent/`**: may exist as a **RAM mirror** (created/filled by ramlog syncing `/var/log.hdd/persistent` → `/var/log/persistent` at boot).
  - This directory is **not** the authoritative store and should not be relied on for long-term retention.

### Why there are both `/var/log` and `/var/log.hdd` (ramlog/log2ram background)

On Armbian, the “ramlog” mechanism commonly:

1. Ensures an on-disk log directory exists.
2. Creates a bind-mount so the on-disk logs are accessible at **`/var/log.hdd/`**.
3. Mounts a RAM-backed filesystem (zram/tmpfs) on **`/var/log/`**.
4. Uses `rsync`/copy on boot and shutdown to keep the disk copy updated.

In practical terms:
- **`/var/log.hdd/persistent`** is a *directory on disk*.
- When ramlog “restores logs” at boot, it may copy that directory into the RAM-backed `/var/log` as **`/var/log/persistent`**.
  - That is why you can end up with **both** paths existing at the same time.

This is a widely used pattern in Armbian/Orange Pi ecosystems; see Armbian discussions like:
- `https://forum.armbian.com/topic/3728-varlog-varloghdd/`
- `https://forum.armbian.com/topic/13483-help-me-to-understand-armbian-ramlog-where-he-log/`

### Why Argo logs to `/var/log.hdd/persistent` (not `/var/log/persistent`)

**`/var/log` is RAM and intentionally small.** If large “persistent” logs are copied into `/var/log/persistent`, zram can fill and break:
- `rsyslog` file outputs (`/var/log/syslog`, `/var/log/kern.log`, etc.)
- systemd-journald storage in `/var/log/journal`
- any service writing into `/var/log/*`

Therefore Argo uses **`/var/log.hdd/persistent/`** as the **single source of truth** for persistent logs and configures Argo services to write there directly (typically via `tee -a /var/log.hdd/persistent/<service>.log`).

**Note on systemd-journald**: `Storage=persistent` normally stores journals under `/var/log/journal/`. On ramlog systems, `/var/log` is RAM-backed, so journald “persistent” journals can still consume zram unless journald is explicitly redirected/symlinked to an on-disk location (varies by distro/image). Argo’s persistent log files are intentionally separate from journald for predictable retention.

If your `/var/log` zram fills, it’s usually because something is writing large files into `/var/log` or because ramlog is copying too much data from `/var/log.hdd` back into RAM. Argo includes a fix for the Orange Pi ramlog script to **exclude `persistent/` in BOTH directions** (disk→RAM and RAM→disk) so persistent logs stay on disk and don’t consume zram.

### Installation: top-level Makefile targets

Argo’s persistent logging and pruning is installed via these Makefile targets:

- **Install everything in `system-monitoring/` (includes persistent log manager)**:
  - `make install-system-monitoring`
- **Install only the Argo persistent log manager (boot rotation + hourly prune timer + logrotate policy)**:
  - `make -C system-monitoring install-persistent-log-manager`
- **Patch Orange Pi ramlog to avoid zram filling / accidental deletion**:
  - `make fix-orangepi-ramlog`

The persistent log manager installs and enables:
- `argo-persistent-logs.service` (oneshot at boot: rotate + prune)
- `argo-persistent-logs-prune.timer` (hourly prune checks)
- `/etc/logrotate.d/persistent-logs` (size-based rotation for live `argo-*.log` files)

## Overview

The Argo system uses a logging architecture that combines Argo-specific monitoring services with system-level logs. Argo service logs that must survive reboots are written to `/var/log.hdd/persistent/`, while system logs are managed separately by rsyslog/systemd-journald (often with ramlog/log2ram underneath).

## Log Directory Structure

The Argo system uses a dual logging architecture:

### Persistent Logging (`/var/log.hdd/persistent/`)
**Purpose**: Logs that survive system reboots and provide long-term diagnostic information
- **Daily logs**: `service-name-YYYYMMDD.log`
- **Timestamped logs**: `service-name-YYYYMMDD-HHMMSS.log`
- **CSV data**: `service-name-YYYYMMDD.csv`

**Important**: `/var/log/persistent/` may appear on some Armbian/Orange Pi installs as a **RAM mirror** populated by ramlog. For Argo, treat `/var/log.hdd/persistent/` as authoritative.

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

## Log Viewing Tools

### Argo Multi-Service Log Viewer

`argo_logs.sh` is a helper script that tails logs from multiple Argo system services with color-coded output for easy identification.

#### Services Monitored

- **argo_launch_standard.service** (cyan) - Main ROS2 node launcher
- **argo_battery_water.service** (yellow) - Battery and water monitoring
- **argo_power_control.service** (green) - Power control and button handling
- **argo_bno085.service** (magenta) - IMU driver and bridge
- **argo_health_monitor.service** (cyan) - Node health aggregation

#### Usage

**Via Alias (Recommended)**
```bash
alog              # Follow all service logs with color coding
```

**Direct Script Usage**
```bash
# Follow mode (default) - shows last 20 lines then follows
./scripts/argo_logs.sh

# Show last 50 lines per service then follow
./scripts/argo_logs.sh -n 50

# Error check mode: search all logs for ERROR/WARN/FATAL
./scripts/argo_logs.sh -e

# Show recent errors then follow
./scripts/argo_logs.sh -ef

# Show all logs from last 2 boots
./scripts/argo_logs.sh -a

# Filter logs by pattern
./scripts/argo_logs.sh "controller"
./scripts/argo_logs.sh "(controller|dashboard)"

# Show help
./scripts/argo_logs.sh -h
```

#### Features

- **Color-coded output**: Each service has a distinct color for easy visual scanning
- **Priority highlighting**: ERROR lines (bold bright red), WARN lines (bold dark yellow)
- **Multi-service support**: Uses `journalctl` with multiple `-u` flags for efficient log following
- **Real-time following**: Continuously tails all services simultaneously
- **Error detection**: Can filter for ERROR/WARN/FATAL messages across all services
- **Pattern filtering**: Supports extended regex patterns for log filtering
- **No sudo required**: Works with user permissions

#### Technical Details

**Implementation**
- Uses `journalctl -f -u <service1> -u <service2> ...` for efficient multi-service following
- Colors applied via real-time processing as logs stream
- Timestamps shown in ISO-precise format for accurate correlation
- Filters out foxglove bridge spam (Advertising/Removing channel messages)

**Color Codes**
```bash
COLOR_ARGO_LAUNCH='\e[0;96m'   # Cyan
COLOR_BATTERY='\e[0;93m'        # Yellow
COLOR_POWER='\e[0;92m'          # Green
COLOR_IMU='\e[0;95m'            # Magenta
COLOR_ERROR='\e[1;91m'          # Bold bright red
COLOR_WARN='\e[1;33m'           # Bold dark yellow
```

#### Examples

**Normal Usage**
```bash
$ alog
📋 Argo Multi-Service Logs
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
● argo_launch_standard.service (cyan)
● argo_battery_water.service (yellow)
● argo_power_control.service (green)
● argo_bno085.service (magenta)
● argo_health_monitor.service (cyan)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

2025-10-13T07:35:05.123456+00:00 orangepizero2w systemd[1]: Started Argo Robot ROS2 Launch Service.
2025-10-13T07:35:06.234567+00:00 orangepizero2w python3[1234]: [INFO] Battery voltage: 7.698V
2025-10-13T07:35:07.345678+00:00 orangepizero2w python3[1235]: [INFO] Power controller initialized
```

**Show More History**
```bash
# Show last 100 lines per service
alog -n 100
```

**Error Detection**
```bash
# Find all errors across all services
alog -e

# Find errors for specific node
alog -e "controller"

# Follow errors in real-time
alog -ef
```

#### Integration with Dotfiles

The `alog` alias is defined in `dotfiles/bash_aliases`:
```bash
alias alog='~/argo/scripts/argo_logs.sh'
```

After updating dotfiles, reload with:
```bash
source ~/.bashrc
# or
relogin
```

### Alternative Log Viewing Methods

**Direct journalctl**
```bash
# View specific service logs
journalctl -u argo_launch_standard.service -f

# View multiple services
journalctl -u argo_launch_standard.service -u argo_power_control.service -f

# View logs from previous boot
journalctl -u argo_launch_standard.service -b -1
```

**Persistent Log Files**
```bash
# Monitor all persistent logs
tail -f /var/log.hdd/persistent/*.log

# Monitor specific service
tail -f /var/log.hdd/persistent/argo-power-control.log
tail -f /var/log.hdd/persistent/thermal-$(date +%Y%m%d).log
```

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
- Maintains boot-specific log snapshots for retrospective analysis (automatically compressed to save ~90% space)
- Prunes oldest log artifacts only when persistent storage usage crosses the threshold
- Installs size-based logrotate configuration (rotates at 10MB, keeps 2 copies) to prevent unbounded growth between reboots
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
SystemMaxUse=100M
SystemKeepFree=1G
MaxRetentionSec=2day
ForwardToSyslog=no
```

**Key Settings**:
- **Storage**: `persistent` - Logs stored in `/var/log/journal/` (changed from `volatile` on 2025-10-08 to preserve boot logs across reboots)
- **SystemMaxUse**: `100M` - Maximum disk usage for journal (reduced from 500M on 2025-12-16)
- **MaxRetentionSec**: `2day` - Maximum retention time (reduced from 2week on 2025-12-16)
- **Compress**: `yes` - Compress old journal files

**Configuration History**:
- **2025-10-08**: Changed `Storage` from `volatile` to `persistent` to ensure boot logs survive reboots. Journal location: `/var/log/journal/ce4def9bd6a04cd594e94f8205bbe671/`
- **2025-12-16**: Reduced `SystemMaxUse` from 500M to 100M and `MaxRetentionSec` from 2week to 2day to reduce log storage requirements

### Persistent Log Manager Configuration

- The `install-persistent-log-manager` target installs a new size-based logrotate configuration at `/etc/logrotate.d/persistent-logs`.
- `/usr/local/bin/argo_persistent_log_manager.sh` rotates Argo service logs at boot and creates timestamped `*.boot.log.gz` snapshots (automatically compressed).
- **Size-based rotation**: Logrotate rotates Argo service logs when they exceed 10MB, keeping 2 rotated copies (compressed). This prevents unbounded growth between reboots.
- The companion timer (`argo-persistent-logs-prune.timer`) triggers an hourly check that only removes files when the persistent log partition exceeds the configurable threshold (default: 85%).
- System logs outside the persistent directory continue to be rotated by the stock logrotate configuration in `/etc/logrotate.d/rsyslog`.

### Log Rotation Schedule

| Log Type | Rotation Frequency | Retention | Compression | Location |
|----------|-------------------|-----------|-------------|----------|
| **Argo Service Logs** | Size-based (10MB) + boot | 2 rotated copies + boot snapshots | Yes (automatic) | `/var/log.hdd/persistent/` |
| **Boot Log Snapshots** | Per boot | Pruned when usage ≥ 85% | Yes (automatic) | `/var/log.hdd/persistent/` |
| **System Logs** | Daily (logrotate) | 2 days | Yes (delayed) | `/var/log.hdd/` |
| **Systemd Journal** | Automatic | 2 days, 100MB max | Yes | `/var/log/journal/` |
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

**Log Rotation**: All Argo service logs (items 1-4) are automatically rotated by logrotate when they exceed 10MB, preventing unbounded growth. Boot snapshots are automatically compressed to save space.

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

# Use argo_logs.sh for error detection
alog -e                    # Find all errors
alog -ef                   # Follow errors in real-time
alog -a "ERROR\|FATAL"     # Search all boots for critical errors
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
6. **Use Log Viewer**: Use `alog` for efficient multi-service log monitoring
7. **Error Detection**: Regularly run `alog -e` to catch errors early

## Related Documentation

- [Argo Lifecycle Management](README.md#node-lifecycle-management)
- [Thermal Monitor Installation](launch/README-thermal-monitor.md)
- [Network Improvements](network/README.md)
- [Power Control System](power_control/README.md)
- [Battery Monitoring](scripts/README-plot-battery-water.md)
