# System Monitoring Services

This directory contains optional system-level monitoring services that are useful for debugging and development but are **NOT installed by default** on fresh Argo installations.

## Overview

These services provide comprehensive system monitoring and logging capabilities that complement the core Argo monitoring services. They are designed to help with:

- **Development Debugging**: Process monitoring, memory usage tracking
- **System Diagnostics**: Boot history, kernel messages, crash dumps
- **Hardware Monitoring**: Orange Pi specific hardware monitoring
- **Performance Analysis**: Temperature logging, resource usage tracking

## Services Included

### 1. Boot History Logger Service
- **Service**: `boot-history-logger.service`
- **Script**: `scripts/boot-logger.sh`
- **Log Files**: 
  - `boot-history.log`
  - `dmesg-YYYYMMDD.log` (boot-time dmesg)
  - `journalctl-YYYYMMDD-HHMMSS.log` (boot-time journal)
- **Purpose**: Logs boot events and captures boot-time system messages

### 2. Cursor Process Monitor Service
- **Service**: `cursor-monitor.service`
- **Script**: `scripts/cursor-monitor.sh`
- **Log Files**: `cursor-processes-YYYYMMDD.log`
- **Purpose**: Monitors Cursor IDE processes and memory usage every 60 seconds

### 3. Memory Monitor Service
- **Service**: `memory-monitor.service`
- **Script**: `scripts/memory-monitor.sh`
- **Log Files**: 
  - `memory-YYYYMMDD.log`
  - `processes-YYYYMMDD.log`
- **Purpose**: Monitors system memory usage and process activity every 30 seconds

### 4. Orange Pi Monitor Service
- **Service**: `orangepi-monitor.service`
- **Log Files**: `orangepi-monitor-YYYYMMDD.log`
- **Purpose**: Orange Pi hardware monitoring using `orangepimonitor` tool

### 5. Persistent dmesg Service
- **Service**: `persistent-dmesg.service`
- **Log Files**: `dmesg-YYYYMMDD-HHMMSS.log`
- **Purpose**: Captures kernel messages with timestamps

### 6. Crash Dump Service
- **Service**: `crash-dump.service`
- **Purpose**: Copies crash dumps to persistent storage

### 7. Temperature Logger Service
- **Service**: `temp-logger.service` + `temp-logger.timer`
- **Script**: `scripts/temp-logger.sh`
- **Log Files**: `/var/log/temperature.log` (not in persistent directory)
- **Purpose**: Logs thermal zone temperatures (separate from Argo thermal monitor)

## Installation

### Install All Services
```bash
# Install all system monitoring services
make install-all
```

### Install Individual Services
```bash
# Install specific services
make install-boot-logger
make install-cursor-monitor
make install-memory-monitor
make install-orangepi-monitor
make install-persistent-dmesg
make install-crash-dump
make install-temp-logger
```

## Service Management

### Check Service Status
```bash
# Check status of all services
make status

# Check individual service status
sudo systemctl status boot-history-logger.service
sudo systemctl status cursor-monitor.service
sudo systemctl status memory-monitor.service
```

### Start/Stop Services
```bash
# Start all services
make start

# Stop all services
make stop

# Restart all services
make restart
```

### Uninstall Services
```bash
# Uninstall all services
make uninstall-all

# Uninstall individual services
make uninstall-boot-logger
make uninstall-cursor-monitor
make uninstall-memory-monitor
```

## Log File Monitoring

### View Log Files
```bash
# Monitor all persistent logs
tail -f /var/log.hdd/persistent/*.log

# Monitor specific services
tail -f /var/log.hdd/persistent/boot-history.log
tail -f /var/log.hdd/persistent/memory-$(date +%Y%m%d).log
tail -f /var/log.hdd/persistent/cursor-processes-$(date +%Y%m%d).log
```

### Clean Old Logs
```bash
# Clean log files older than 30 days
make clean-logs
```

## Testing

### Test All Services
```bash
# Test all service scripts
make test-services
```

### Test Individual Scripts
```bash
# Test boot logger
sudo /usr/local/bin/boot-logger.sh

# Test cursor monitor (will run for 10 seconds)
timeout 10 sudo /usr/local/bin/cursor-monitor.sh

# Test memory monitor (will run for 10 seconds)
timeout 10 sudo /usr/local/bin/memory-monitor.sh

# Test temperature logger
sudo /usr/local/bin/temp-logger.sh
```

## Integration with Argo

These services work alongside the core Argo monitoring services:

- **Argo Thermal Monitor**: `argo_thermal_monitor.service` (in `launch/`)
- **Argo Battery Monitor**: `battery_water.py` (in `nodes/`)
- **Argo WiFi Monitor**: `argo_wifi_reconnect.service` (in `network/`)
- **Argo Power Control**: `argo_power_control.service` (in `power_control/`)

All services log to `/var/log.hdd/persistent/` for consistent log management.

## Dependencies

### Required Packages
```bash
# Install required packages
sudo apt update
sudo apt install -y bc  # For temperature calculations
```

### Orange Pi Monitor Tool
The Orange Pi monitor service requires the `orangepimonitor` tool:
```bash
# Install orangepimonitor (if not already installed)
sudo apt install -y orangepimonitor
```

## Troubleshooting

### Service Not Starting
```bash
# Check service status
sudo systemctl status <service-name>

# Check service logs
sudo journalctl -u <service-name> -f

# Restart service
sudo systemctl restart <service-name>
```

### Permission Issues
```bash
# Check script permissions
ls -la /usr/local/bin/*monitor*.sh
ls -la /usr/local/bin/*logger*.sh

# Fix permissions if needed
sudo chmod +x /usr/local/bin/*monitor*.sh
sudo chmod +x /usr/local/bin/*logger*.sh
```

### Log Directory Issues
```bash
# Check persistent log directory
ls -la /var/log.hdd/persistent/

# Create directory if missing
sudo mkdir -p /var/log.hdd/persistent
sudo chown orangepi:orangepi /var/log.hdd/persistent
```

## File Structure

```
system-monitoring/
├── README.md                    # This documentation
├── Makefile                     # Main installation and management Makefile
├── services/                    # Systemd service files
│   ├── boot-history-logger.service
│   ├── cursor-monitor.service
│   ├── memory-monitor.service
│   ├── orangepi-monitor.service
│   ├── persistent-dmesg.service
│   ├── crash-dump.service
│   ├── temp-logger.service
│   └── temp-logger.timer
└── scripts/                     # Monitoring scripts
    ├── boot-logger.sh
    ├── cursor-monitor.sh
    ├── memory-monitor.sh
    └── temp-logger.sh
```

## Best Practices

1. **Selective Installation**: Only install services you need for debugging
2. **Regular Monitoring**: Check log files regularly for system health
3. **Log Rotation**: Use `make clean-logs` to prevent disk space issues
4. **Service Health**: Monitor service status with `make status`
5. **Resource Usage**: These services consume minimal resources but monitor them

## When to Use

### Development Environment
- Install all services for comprehensive debugging
- Monitor Cursor IDE processes and memory usage
- Track system performance during development

### Production Environment
- Install only essential services (boot-logger, memory-monitor)
- Skip development-specific services (cursor-monitor)
- Focus on system health and crash diagnostics

### Troubleshooting
- Install services as needed for specific debugging
- Use boot-logger and persistent-dmesg for startup issues
- Use memory-monitor for performance problems

## Related Documentation

- [Persistent Monitoring Services](../docs/README-persistent-monitoring-services.md)
- [Argo Lifecycle Management](../README.md#node-lifecycle-management)
- [Thermal Monitor Installation](../launch/README-thermal-monitor.md)
- [Network Improvements](../network/README.md)
- [Power Control System](../power_control/README.md)
