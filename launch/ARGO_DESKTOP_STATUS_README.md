# Argo Desktop Status Monitor

A desktop notification-based status monitor for Argo ROS2 services and nodes.

## Features

- **Smart Notification Management**: Clears previous notifications before sending new ones to minimize spam
- **Event-Driven Updates**: Monitors ROS2 events for efficient, real-time updates (no constant polling)
- **Auto-Hide on Close**: Stops monitoring when notification is closed by user
- **Comprehensive Status**: Shows service status, node status, recording status, and system info
- **Resource Efficient**: Uses change detection instead of constant polling (15-60 second intervals)
- **Minimal Notification Spam**: Clears old notifications before showing new ones

## Usage

### Basic Usage
```bash
# Start the desktop status monitor
python3 /home/orangepi/argo/launch/argo_desktop_status.py

# Or use the launcher script (includes ROS2 environment setup)
/home/orangepi/argo/launch/argo_desktop_status.sh
```

### Desktop Integration

#### Add to Desktop Autostart
1. Copy the desktop entry to your applications directory:
   ```bash
   cp /home/orangepi/argo/launch/argo-desktop-status.desktop ~/.local/share/applications/
   ```

2. Add to autostart (if using GNOME):
   ```bash
   # Add to GNOME autostart
   mkdir -p ~/.config/autostart
   cp /home/orangepi/argo/launch/argo-desktop-status.desktop ~/.config/autostart/
   ```

#### Manual Desktop Entry
- Double-click `argo-desktop-status.desktop` to start the monitor
- Or add it to your desktop environment's application menu

## Status Information

The notification displays:

### Service Status
- **Launch Service**: Systemd service status (active/inactive)
- **Recording Service**: ROS2 service status (Recording/Stopped)

### Node Status
- **Running Nodes**: Count of active nodes (e.g., "8/9")
- **Stopped Nodes**: List of non-running nodes
- **Node CPU Usage**: Total CPU usage by all nodes

### System Information
- **CPU Temperature**: Current CPU temperature
- **Memory Usage**: System memory usage percentage
- **Node Resource Usage**: CPU usage by Argo nodes

## Behavior

### Notification Lifecycle
1. **Startup**: Shows initial status notification
2. **Updates**: Automatically updates when status changes
3. **User Close**: Stops monitoring when notification is closed
4. **Shutdown**: Clears notification on script exit

### Event Monitoring
- **Node Changes**: Monitors for nodes starting/stopping
- **Recording Changes**: Monitors recording status changes
- **Service Changes**: Monitors systemd service status
- **Efficient**: Only updates when changes are detected

## Requirements

### System Requirements
- Desktop environment (GNOME, KDE, XFCE, etc.)
- `notify-send` command available
- ROS2 Humble environment
- Python 3.6+

### Installation
```bash
# Install notification support (if not already installed)
sudo apt install libnotify-bin

# Make scripts executable
chmod +x /home/orangepi/argo/launch/argo_desktop_status.py
chmod +x /home/orangepi/argo/launch/argo_desktop_status.sh
```

## Troubleshooting

### Common Issues

#### "No desktop environment detected"
- **Cause**: Running in terminal/SSH without desktop environment
- **Solution**: Run from desktop session or use `DISPLAY` environment variable

#### "notify-send not found"
- **Cause**: Notification system not installed
- **Solution**: Install with `sudo apt install libnotify-bin`

#### Notification not updating
- **Cause**: ROS2 environment not sourced
- **Solution**: Use the launcher script or source ROS2 environment manually

#### Notification appears but no updates
- **Cause**: Argo services not running
- **Solution**: Start Argo services with `al` command

#### Multiple notifications appearing
- **Cause**: Notification system doesn't support replacement (common on Xfce4)
- **Solution**: The latest version clears previous notifications before sending new ones
- **Test**: Run `python3 /home/orangepi/argo/launch/test_notification_clearing.py` to verify clearing works
- **Note**: Some notification systems don't support replacement, so you may still see multiple notifications

### Debug Mode
To see debug output, run with verbose logging:
```bash
python3 /home/orangepi/argo/launch/argo_desktop_status.py 2>&1 | tee argo_desktop_status.log
```

## Integration with Argo System

The desktop status monitor integrates seamlessly with the Argo system:

- **Uses Centralized Node Detection**: Leverages `argo_node_utils.py` for consistent node monitoring
- **ROS2 Service Integration**: Monitors the new ROS2-based recording system
- **Systemd Service Monitoring**: Tracks the main launch service status
- **Event-Driven Updates**: Efficiently updates only when changes occur

## Customization

### Notification Appearance
The notification uses standard desktop notification settings. Customize through your desktop environment's notification settings.

### Update Frequency
Default update frequency is 15 seconds for ROS2 monitoring and 60 seconds for notification status checks. The script only updates when changes are detected, making it very efficient. Modify in the script if needed.

### Status Information
Add or remove status information by modifying the `_format_status_message()` method in the script.
