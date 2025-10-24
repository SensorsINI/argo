# Argo Network Improvements

This directory contains system-level network improvements for the Argo sailboat control system, specifically addressing WiFi reconnection issues and NetworkManager optimizations.

## Problem Solved

NetworkManager 1.36.6 has a known limitation: it doesn't automatically switch to higher-priority networks when they become available while already connected to a lower-priority network. This caused issues where Argo would:

1. Connect to `uzh-iot` (priority 5) when preferred networks weren't available
2. Stay connected to `uzh-iot` even when `tobi-wlan` (priority 10) or `tobi-s24` (priority 15) became available
3. Require manual intervention to switch to preferred networks

## Critical Bug Fix (v2.0)

**IMPORTANT**: The initial implementation had a critical bug where aggressive NetworkManager settings (`wifi.scan-interval=30` and `wifi.roam-threshold=70`) caused frequent reconnection attempts that prevented SSH access. This has been fixed in v2.0 with:

- Conservative NetworkManager settings (default scan intervals)
- Connection stability checks (30-second minimum before switching)
- Lock file protection against multiple instances
- Reduced timer frequency (every 5 minutes instead of 2)
- Comprehensive logging and error handling

## Solution Components

### 1. WiFi Reconnection Script (`scripts/wifi_reconnect.sh`)

- **Purpose**: Automatically switches to preferred networks when they become available
- **Frequency**: Runs every 5 minutes via systemd timer (safer than 2 minutes)
- **Priority Order**: 
  - `tobi-s24` (priority 15) - highest
  - `tobi-wlan` (priority 10) - medium
  - `uzh-iot` (priority 5) - lowest (fallback)
- **Safety Features**:
  - Connection stability checks (30-second minimum before switching)
  - Lock file protection against multiple instances
  - Comprehensive logging and error handling
- **Logging**: All activity logged to `/var/log.hdd/persistent/wifi-reconnect.log`
- **Smart Detection**: Ignores ZeroTier connections, only manages WiFi

### 2. NetworkManager Configuration (`config/argo_wifi_scan.conf`)

- **Conservative Settings**: Uses default scan intervals (120s) to prevent aggressive scanning
- **Improved Recognition**: Disabled random MAC addresses for better network recognition
- **No Aggressive Roaming**: Disabled to prevent frequent connection attempts
- **Location**: Installed to `/etc/NetworkManager/conf.d/argo_wifi_scan.conf`

### 3. Systemd Service and Timer (`config/argo_wifi_reconnect.service` & `config/argo_wifi_reconnect.timer`)

- **Service File**: Defines what to run (the WiFi reconnection script)
- **Timer File**: Defines when to run (every 5 minutes)
- **Benefits over cron**:
  - Better logging integration with systemd journal
  - Dependency management (waits for network to be ready)
  - More reliable error handling and restart policies
  - Better integration with systemd service management

### 4. Installation Script (`install/install_network_improvements.sh`)

- **Automated Setup**: Installs all components with proper permissions
- **Systemd Integration**: Sets up service and timer instead of cron jobs
- **Service Restart**: Restarts NetworkManager to apply configuration
- **Testing**: Validates installation and shows status
- **Error Handling**: Comprehensive error checking and logging

## Installation

### From Repository Root

```bash
# Install network improvements
make install-network-improvements

# Or install as part of complete system
make install-all
```

### Manual Installation

```bash
# Run installation script directly
sudo ./network/install/install_network_improvements.sh
```

## File Structure

```
network/
├── README.md                           # This documentation
├── scripts/
│   └── wifi_reconnect.sh              # WiFi reconnection script
├── config/
│   └── argo_wifi_scan.conf            # NetworkManager configuration
└── install/
    └── install_network_improvements.sh # Installation script
```

## System Integration

### Makefile Integration

The network improvements are integrated into the main Argo Makefile:

- **Target**: `install-network-improvements`
- **Included in**: `install-all` (complete system installation)
- **Help**: Listed in `make help` output

### Service Dependencies

- **NetworkManager**: Must be running for WiFi management
- **Systemd**: Used for timer-based reconnection checks and NetworkManager service management
- **No Cron**: Uses systemd timers instead of cron for better reliability

## Monitoring and Troubleshooting

### Log Files

- **Reconnection Log**: `/var/log.hdd/persistent/wifi-reconnect.log`
- **Systemd Service Log**: `journalctl -u argo_wifi_reconnect.service`
- **Systemd Timer Log**: `journalctl -u argo_wifi_reconnect.timer`
- **NetworkManager Log**: `journalctl -u NetworkManager`

### Manual Testing

```bash
# Test reconnection script manually
/usr/local/bin/wifi_reconnect.sh

# Check current WiFi connections
nmcli connection show --active

# View available networks
nmcli device wifi list

# Check cron job
crontab -l
```

### Common Issues

1. **Script not running**: Check cron service and job configuration
2. **Networks not switching**: Verify network priorities and availability
3. **Permission errors**: Ensure script has execute permissions
4. **NetworkManager issues**: Check service status and configuration

## Configuration Customization

### Adding New Preferred Networks

Edit `/usr/local/bin/wifi_reconnect.sh`:

```bash
# Add to PREFERRED_NETWORKS array
PREFERRED_NETWORKS=("tobi-s24" "tobi-wlan" "new-network")
```

### Changing Scan Frequency

Edit `/etc/NetworkManager/conf.d/argo_wifi_scan.conf`:

```ini
[connection]
wifi.scan-interval=15  # Scan every 15 seconds
```

### Modifying Reconnection Frequency

Edit cron job:

```bash
# Change from every 2 minutes to every minute
crontab -e
# Change */2 to */1
```

## Uninstallation

To remove the network improvements:

```bash
# Remove cron job
crontab -e  # Delete the wifi_reconnect.sh line

# Remove files
sudo rm -f /usr/local/bin/wifi_reconnect.sh
sudo rm -f /etc/NetworkManager/conf.d/argo_wifi_scan.conf

# Restart NetworkManager
sudo systemctl restart NetworkManager
```

## Technical Details

### Network Priority System

NetworkManager uses numeric priorities where higher numbers = higher priority:

- **15**: `tobi-s24` (mobile hotspot)
- **10**: `tobi-wlan` (home network)
- **5**: `uzh-iot` (university network)

### Reconnection Logic

1. **Check Current Connection**: Get active WiFi connection name
2. **Compare Against Preferred**: Check if current connection is preferred
3. **Scan for Better Options**: Look for higher-priority networks
4. **Switch if Available**: Connect to highest-priority available network
5. **Log Activity**: Record all decisions and actions

### Error Handling

- **Network Unavailable**: Gracefully handle when preferred networks aren't found
- **Connection Failures**: Log failed connection attempts
- **Permission Issues**: Handle sudo requirements properly
- **Log Rotation**: Prevent log files from growing too large

## Future Enhancements

Potential improvements for future versions:

1. **Signal Strength Monitoring**: Switch based on signal quality
2. **Time-based Priorities**: Different priorities at different times
3. **Network Health Checks**: Ping tests before switching
4. **User Notifications**: Alert when switching networks
5. **Configuration GUI**: Web interface for network management

## Support

For issues or questions:

1. Check log files for error messages
2. Verify network configuration and priorities
3. Test manual reconnection script
4. Review NetworkManager service status
5. Check cron job execution

