# Argo Network Improvements

**Source of truth for dev** is this file in your working tree. The `Documentation` line in `config/argo_wifi_reconnect.service` and `config/argo_wifi_reconnect.timer` points at GitHub so `systemctl` and editor links work on every OS; that page can lag uncommitted or unpushed work. To open the local file in the editor, use **Go to File** (Ctrl+P) and type `network/README.md`.

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

## MAC Address Cloning (Hardware Replacement)

**Problem**: When replacing Orange Pi SBC hardware (due to damage), the new hardware has a different MAC address, which can break network configurations that depend on a specific MAC address (e.g., `uzh-iot`, ZeroTier, `tobi-wlan`).

**Solution**: The Argo system uses NetworkManager's cloned MAC address feature to freeze the WiFi MAC address to a consistent value stored in `ARGO_MAC_ID.txt`. This allows the same SD card to work across different hardware while maintaining the same network identity.

### MAC Address Management

- **Frozen MAC Address**: `c8:26:e2:6c:58:ba` (stored in `network/ARGO_MAC_ID.txt`)
- **Configuration**: All WiFi connections use NetworkManager's `cloned-mac-address` setting
- **Persistence**: MAC address persists across hardware changes when using the same SD card
- **Script**: `scripts/freeze_mac_address.sh` - Automatically configures MAC address from git repo or logs

### Usage

```bash
# Freeze MAC address to value from ARGO_MAC_ID.txt (recommended)
make freeze-mac-address

# Or use script directly
./scripts/freeze_mac_address.sh --from-logs

# Or specify MAC address directly
./scripts/freeze_mac_address.sh c8:26:e2:6c:58:ba
```

### How It Works

1. **MAC Address Source**: The script first checks `network/ARGO_MAC_ID.txt` (committed to git)
2. **NetworkManager Configuration**: Sets `802-11-wireless.cloned-mac-address` for all WiFi connections
3. **Storage**: Connection profiles are stored in `/etc/NetworkManager/system-connections/*.nmconnection` (on SD card)
4. **Automatic Application**: MAC address is applied automatically during boot when NetworkManager activates WiFi connections
5. **Verification**: Check current MAC with `ip link show wlan0`

### Boot Process

When you swap Orange Pi hardware and boot with the same SD card:

1. **System Boots** → Linux kernel initializes hardware (new SBC has different hardware MAC)
2. **NetworkManager Starts** → Reads connection profiles from `/etc/NetworkManager/system-connections/`
3. **Connection Activation** → NetworkManager activates WiFi connection (e.g., `tobi-wlan`)
4. **MAC Address Applied** → NetworkManager reads `cloned-mac-address=C8:26:E2:6C:58:BA` from connection profile
5. **Interface Configured** → WiFi interface `wlan0` is set to use cloned MAC address instead of hardware MAC
6. **Network Ready** → WiFi connects using frozen MAC address, maintaining network identity

**Result**: The new hardware automatically uses the frozen MAC address (`c8:26:e2:6c:58:ba`) without any manual intervention, because the connection profiles are stored on the SD card and applied automatically by NetworkManager during boot.

**No Manual Steps Required**: If the connection profiles already have the cloned MAC address configured (which they do after running `make freeze-mac-address`), the MAC address will be applied automatically on the new hardware during boot.

### Files

- `network/ARGO_MAC_ID.txt` - Frozen MAC address (committed to git)
- `scripts/freeze_mac_address.sh` - MAC address configuration script
- NetworkManager connections - Each WiFi connection has `cloned-mac-address` set

### When to Use

- **After hardware replacement**: Run `make freeze-mac-address` after replacing SBC
- **Initial setup**: MAC address is automatically configured from git repo
- **Network issues**: If network access fails after hardware change, verify MAC address

### ZeroTier Note

ZeroTier uses its own virtual MAC address and doesn't depend on the WiFi MAC address. No ZeroTier configuration changes are needed when freezing the WiFi MAC address.

## Solution Components

### 1. WiFi Reconnection Script (`scripts/wifi_reconnect.sh`)

- **Purpose**: Automatically switches to preferred networks when they become available
- **Frequency**: Runs every 5 minutes via systemd timer (safer than 2 minutes)
- **Priority Order**: Defined in wifi_reconnect.sh in PREFERRED_NETWORKS
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
├── ARGO_MAC_ID.txt                     # Frozen MAC address for hardware replacement
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

## Adding New WiFi Networks

When adding a new WiFi network (e.g., "Hotel dei Pini" at a new location), follow these steps to ensure proper configuration with MAC address cloning and network priority.

### Step-by-Step Guide

#### 1. Add the WiFi Network

**Option A: Using NetworkManager CLI (Recommended)**
```bash
# For password-protected network
nmcli connection add type wifi con-name "Hotel dei Pini" ifname wlan0 ssid "Hotel dei Pini" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "password-here" \
    connection.autoconnect yes connection.autoconnect-priority 25

# For open network (no password)
nmcli connection add type wifi con-name "Hotel dei Pini" ifname wlan0 ssid "Hotel dei Pini" \
    connection.autoconnect yes connection.autoconnect-priority 25
```

**Option B: Using NetworkManager GUI**
- Open NetworkManager settings
- Add new WiFi network
- Set connection name, SSID, and password
- Configure autoconnect and priority (see priority guidelines below)

#### 2. Set Network Priority

Choose an appropriate priority based on network type:
- **25-30**: Temporary high-priority networks (hotels, cafes, temporary hotspots)
- **20**: Personal mobile hotspots (tobi-matebook, tobi-s24)
- **15**: Secondary home/office networks
- **5-10**: Primary home/office networks (tobi-wlan)
- **0-5**: Fallback networks (uzh-iot)

**Example for Hotel dei Pini:**
```bash
# Set high priority for temporary hotel network
nmcli connection modify "Hotel dei Pini" connection.autoconnect-priority 25
```

#### 3. **CRITICAL: Set Cloned MAC Address**

**This step is essential** - without it, the new network won't use the frozen MAC address and may fail to connect or break network identity.

```bash
# Get the frozen MAC address from ARGO_MAC_ID.txt
FROZEN_MAC=$(grep -E "^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$" network/ARGO_MAC_ID.txt | tail -1)

# Set cloned MAC address for the new network
nmcli connection modify "Hotel dei Pini" 802-11-wireless.cloned-mac-address "$FROZEN_MAC"

# Or specify directly
nmcli connection modify "Hotel dei Pini" 802-11-wireless.cloned-mac-address "c8:26:e2:6c:58:ba"
```

#### 4. Verify Configuration

```bash
# Check connection details
nmcli connection show "Hotel dei Pini"

# Verify cloned MAC address is set
nmcli connection show "Hotel dei Pini" | grep cloned-mac-address

# Test connection
nmcli connection up "Hotel dei Pini"

# Verify actual MAC address used
ip link show wlan0 | grep -oP '(?<=link/ether )[^ ]+'
# Should show: c8:26:e2:6c:58:ba
```

#### 5. Update WiFi Reconnection Script (Optional)

If you want the new network to be considered in automatic network switching, edit `/usr/local/bin/wifi_reconnect.sh`:

```bash
# Add to PREFERRED_NETWORKS array (in priority order)
PREFERRED_NETWORKS=("Hotel dei Pini" "tobi-s24" "tobi-wlan" "uzh-iot")
```

**Note**: The WiFi reconnection script only switches to higher-priority networks. If "Hotel dei Pini" has priority 25, it will be preferred over lower-priority networks.

### Complete Example: Adding "Hotel dei Pini"

```bash
# 1. Add network with password
nmcli connection add type wifi con-name "Hotel dei Pini" ifname wlan0 ssid "Hotel dei Pini" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "hotel-password" \
    connection.autoconnect yes connection.autoconnect-priority 25

# 2. Set cloned MAC address (CRITICAL!)
nmcli connection modify "Hotel dei Pini" 802-11-wireless.cloned-mac-address "c8:26:e2:6c:58:ba"

# 3. Verify configuration
nmcli connection show "Hotel dei Pini" | grep -E "cloned-mac-address|autoconnect-priority"

# 4. Test connection
nmcli connection up "Hotel dei Pini"

# 5. Verify MAC address
ip link show wlan0 | grep -oP '(?<=link/ether )[^ ]+'
```

### Quick Reference: Network Priority Guidelines

| Priority | Network Type | Examples |
|----------|-------------|----------|
| 25-30 | Temporary high-priority | Hotels, cafes, temporary hotspots |
| 20 | Personal mobile hotspots | tobi-matebook, tobi-s24 |
| 15 | Secondary networks | Backup home/office networks |
| 5-10 | Primary networks | tobi-wlan (home router) |
| 0-5 | Fallback networks | uzh-iot (university network) |

### Troubleshooting New Networks

**Problem**: Network connects but uses wrong MAC address
- **Solution**: Verify cloned MAC address is set: `nmcli connection show "Network-Name" | grep cloned-mac-address`

**Problem**: Network doesn't connect automatically
- **Solution**: Check autoconnect is enabled: `nmcli connection modify "Network-Name" connection.autoconnect yes`

**Problem**: Network connects but has low priority
- **Solution**: Increase priority: `nmcli connection modify "Network-Name" connection.autoconnect-priority 25`

**Problem**: Network requires MAC address whitelisting
- **Solution**: Ensure cloned MAC address (`c8:26:e2:6c:58:ba`) is whitelisted on the network's access point/router

## Configuration Customization

### Adding New Preferred Networks to Reconnection Script

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
