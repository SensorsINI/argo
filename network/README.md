# Argo Network Improvements

This directory contains system-level network improvements for the Argo sailboat control system, specifically addressing WiFi reconnection issues and NetworkManager optimizations.

## Problem Solved

NetworkManager 1.36.6 has a known limitation: it doesn't automatically switch to higher-priority networks when they become available while already connected to a lower-priority network. This caused issues where Argo would:

1. Connect to `uzh-iot` (priority 5) when preferred networks weren't available
2. Stay connected to `uzh-iot` even when `tobi-wlan` (priority 10) or `tobi-s24` (priority 15) became available
3. Require manual intervention to switch to preferred networks

## Solution Components

### 1. WiFi Reconnection Script (`scripts/wifi_reconnect.sh`)

- **Purpose**: Automatically switches to preferred networks when they become available
- **Frequency**: Runs every 2 minutes via cron job
- **Priority Order**: 
  - `tobi-s24` (priority 15) - highest
  - `tobi-wlan` (priority 10) - medium
  - `uzh-iot` (priority 5) - lowest (fallback)
- **Logging**: All activity logged to `/var/log.hdd/persistent/wifi-reconnect.log`
- **Smart Detection**: Ignores ZeroTier connections, only manages WiFi

### 2. NetworkManager Configuration (`config/argo_wifi_scan.conf`)

- **Faster Scanning**: Reduced scan interval from 120s to 30s
- **Better Roaming**: Enabled with 70% signal threshold
- **Improved Recognition**: Disabled random MAC addresses
- **Location**: Installed to `/etc/NetworkManager/conf.d/argo_wifi_scan.conf`

### 3. Installation Script (`install/install_network_improvements.sh`)

- **Automated Setup**: Installs all components with proper permissions
- **Cron Job Management**: Sets up automatic reconnection monitoring
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
- **Cron**: Required for scheduled reconnection checks
- **Systemd**: Used for NetworkManager service management

## Monitoring and Troubleshooting

### Log Files

- **Reconnection Log**: `/var/log.hdd/persistent/wifi-reconnect.log`
- **NetworkManager Log**: `journalctl -u NetworkManager`
- **Cron Log**: `journalctl -u cron`

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

