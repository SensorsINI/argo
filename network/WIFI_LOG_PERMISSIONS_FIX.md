# WiFi Reconnection Log Permission Fix

## Problem

When the systemd service rotates log files in `/var/log.hdd/persistent`, the new `wifi-reconnect.log` file was created with root ownership. Since the service was running as user `orangepi`, it couldn't append to the log file, resulting in "Permission denied" errors.

### Root Cause

1. **Log Rotation**: When journald or other system processes rotate logs, they create new files as root
2. **Service User**: The service was running as `User=orangepi`, which couldn't write to root-owned files
3. **Permission Denied**: The `tee -a` command in the script failed when trying to append to a root-owned file

## Solution

The fix involves two changes:

### 1. Service Configuration (`network/config/argo_wifi_reconnect.service`)

Changed the service to run as root instead of orangepi:

```ini
[Service]
Type=oneshot
User=root
Group=root
ExecStart=~/argo/network/scripts/wifi_reconnect.sh
Environment="HOME=/home/orangepi"
Environment="USER=orangepi"
```

**Why root?**
- Root can always write to log files, even after rotation
- Root can fix file permissions if needed
- NetworkManager operations (nmcli) work regardless of user when run by root
- Log files are owned by root with 644 permissions (readable by all)

### 2. Script Permissions Handler (`network/scripts/wifi_reconnect.sh`)

Added `ensure_log_permissions()` function that:

1. Creates the log directory if needed
2. Creates the log file with correct permissions (644 - readable by all)
3. Ensures the file is readable after rotation

```bash
ensure_log_permissions() {
    mkdir -p "$(dirname "$LOG_FILE")"
    
    if [ ! -f "$LOG_FILE" ]; then
        touch "$LOG_FILE"
        chmod 644 "$LOG_FILE"
        # Service runs as root, log file is owned by root
    else
        # Ensure file is readable by all
        chmod 644 "$LOG_FILE" 2>/dev/null || true
    fi
}
```

The `log_message()` function now calls `ensure_log_permissions()` before every write, ensuring the file always has correct permissions.

## Benefits

1. **Automatic Recovery**: Handles log rotation gracefully without manual intervention
2. **Consistent Permissions**: Log files are always readable by all users (644 permissions)
3. **No Permission Errors**: Root can always write to log files, even after rotation
4. **User-Friendly**: Log files are readable by any user for monitoring

## Testing

To test the fix:

```bash
# Manually rotate the log to simulate the issue
sudo mv /var/log.hdd/persistent/wifi-reconnect.log /var/log.hdd/persistent/wifi-reconnect.log.old
sudo touch /var/log.hdd/persistent/wifi-reconnect.log

# Verify it's owned by root
ls -la /var/log.hdd/persistent/wifi-reconnect.log
# Should show root:root with 644 permissions

# Run the script (or wait for timer to trigger)
systemctl start argo_wifi_reconnect.service

# Verify permissions
ls -la /var/log.hdd/persistent/wifi-reconnect.log
# Should show root:root with 644 permissions (readable by all)

# Check logs for successful writes
tail -f /var/log.hdd/persistent/wifi-reconnect.log
```

## Installation

After making these changes, the service must be reinstalled using the top-level Makefile:

```bash
# Reinstall the WiFi reconnection system (this will install the updated service and script)
make install-network-improvements

# Or if the service is already running, just reload and restart
sudo systemctl daemon-reload
sudo systemctl restart argo_wifi_reconnect.timer

# Verify it's running
systemctl status argo_wifi_reconnect.service
```

## Alternative Solutions Considered

1. **Run as orangepi with sudo in script**: Would require password or sudoers configuration
2. **ACL permissions**: More complex, harder to maintain
3. **Separation of concerns**: Separate log rotation from the service
4. **Change rotation tool**: Would require changing system-wide configuration

The chosen solution is simple, maintainable, and robust.
