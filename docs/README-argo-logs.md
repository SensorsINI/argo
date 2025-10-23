# Argo Multi-Service Log Viewer

## Overview

`argo_logs.sh` is a helper script that tails logs from multiple Argo system services with color-coded output for easy identification.

## Services Monitored

- **argo-launch.service** (cyan) - Main ROS2 node launcher
- **battery_water.service** (yellow) - Battery and water monitoring
- **argo_power_control.service** (green) - Power control and button handling

## Usage

### Via Alias (Recommended)
```bash
alog              # Follow all three service logs with color coding
```

### Direct Script Usage
```bash
# Follow mode (default) - shows last 20 lines then follows
./scripts/argo_logs.sh

# Show last 50 lines per service then follow
./scripts/argo_logs.sh -n 50

# Show help
./scripts/argo_logs.sh -h
```

## Features

- **Color-coded output**: Each service has a distinct color for easy visual scanning
- **Multi-service support**: Uses `journalctl` with multiple `-u` flags for efficient log following
- **Real-time following**: Continuously tails all services simultaneously
- **No sudo required**: Works with user permissions

## Technical Details

### Implementation
- Uses `journalctl -f -u <service1> -u <service2> -u <service3>` for efficient multi-service following
- Colors applied via `sed` in real-time as logs stream
- Timestamps shown in ISO-precise format for accurate correlation

### Color Codes
```bash
COLOR_ARGO_LAUNCH='\e[0;96m'   # Cyan
COLOR_BATTERY='\e[0;93m'        # Yellow
COLOR_POWER='\e[0;92m'          # Green
```

## Examples

### Normal Usage
```bash
$ alog
📋 Argo Multi-Service Logs
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
● argo-launch.service (cyan)
● battery_water.service (yellow)
● argo_power_control.service (green)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

2025-10-13T07:35:05.123456+00:00 orangepizero2w systemd[1]: Started Argo Robot ROS2 Launch Service.
2025-10-13T07:35:06.234567+00:00 orangepizero2w python3[1234]: [INFO] Battery voltage: 7.698V
2025-10-13T07:35:07.345678+00:00 orangepizero2w python3[1235]: [INFO] Power controller initialized
```

### Show More History
```bash
# Show last 100 lines per service
alog -n 100
```

## Integration with Dotfiles

The `alog` alias is defined in `dotfiles/.bash_aliases`:
```bash
alias alog='/home/orangepi/argo/scripts/argo_logs.sh'
```

After updating dotfiles, reload with:
```bash
source ~/.bashrc
# or
relogin
```

## See Also

- [argo-systemd-integration](../docs/argo-systemd-integration.md) - Systemd service patterns
- [15-argo-shutdown-status](./15-argo-shutdown-status) - MOTD script with color examples





