# Thermal Monitor Service Installation

This service monitors system temperatures and logs them for diagnostics and status reporting.

## Files

- `thermal-monitor.sh` - Monitoring script that logs temperatures every 30 seconds
- `thermal-monitor.service` - Systemd service configuration

## Installation

### 1. Copy Script to System
```bash
sudo cp thermal-monitor.sh /usr/local/bin/thermal-monitor.sh
sudo chmod +x /usr/local/bin/thermal-monitor.sh
```

### 2. Install Systemd Service
```bash
sudo cp thermal-monitor.service /etc/systemd/system/thermal-monitor.service
sudo systemctl daemon-reload
sudo systemctl enable thermal-monitor.service
sudo systemctl start thermal-monitor.service
```

### 3. Verify Installation
```bash
# Check service status
sudo systemctl status thermal-monitor.service

# View recent logs
tail -f /var/log.hdd/persistent/thermal-$(date +%Y%m%d).log
```

## Log Format

The service logs to `/var/log.hdd/persistent/thermal-YYYYMMDD.log` with the format:
```
2025-10-01 06:30:23: GPU:60°C VE:57°C CPU:58°C DDR:58°C
```

## Temperature Zones

- `thermal_zone0` - GPU temperature
- `thermal_zone1` - VE (Video Engine) temperature
- `thermal_zone2` - CPU temperature
- `thermal_zone3` - DDR (Memory) temperature

## Alert Threshold

The service logs warnings when temperatures exceed 80°C and also sends them to syslog.

## Integration with Argo

The Argo lifecycle manager reads the thermal log file to display CPU temperature in status checks, avoiding the expensive `sensors` command call. This significantly improves status check performance.

See `argo_lifecycle_manager.py` lines 824-847 for the implementation.

