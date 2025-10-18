# Argo Battery Status Panel for XFCE4

This document describes how to add battery status from the Argo sailboat system to the XFCE4 desktop panel.

## Overview

The battery status panel displays real-time battery information from the `battery_water.py` ROS2 node, including:
- Battery voltage and percentage
- Charging status
- AC power presence
- Critical alerts (low battery, saltwater intrusion, high humidity)

## Files Created

- `scripts/battery_status_panel.py` - Main battery status panel with continuous updates
- `scripts/battery_status_simple.py` - Simple one-shot battery status display
- `scripts/battery_status_wrapper.sh` - Wrapper script with ROS2 environment
- `scripts/argo_battery_panel.sh` - Alternative launcher script
- `scripts/setup_battery_panel.sh` - Setup and configuration script
- `.local/share/applications/argo-battery-panel.desktop` - Desktop entry

## Quick Setup

1. Run the setup script:
   ```bash
   cd /home/orangepi/argo
   ./scripts/setup_battery_panel.sh
   ```

2. Follow the on-screen instructions to add the Generic Monitor plugin to your XFCE4 panel

## Manual Setup

### Option 1: Generic Monitor Plugin (Recommended)

1. Right-click on the XFCE4 panel
2. Select "Panel" → "Add New Items..."
3. Choose "Generic Monitor" and click "Add"
4. Right-click the new Generic Monitor and select "Properties"
5. Configure the plugin:
   - Command: `/home/orangepi/argo/scripts/battery_status_genmon_wrapper.sh`
   - Update interval: 10 seconds (or your preference)
   - Label: Leave empty or set to "Battery"
6. Click "Close" to save

### Option 2: Terminal Command

You can always check battery status manually:
```bash
/home/orangepi/argo/scripts/battery_status_wrapper.sh
```

## Generic Monitor Plugin Features

The Generic Monitor plugin provides rich display capabilities:

- **Dynamic Icons**: Changes based on battery status (charging, low, normal)
- **Color-coded Text**: Green for normal, red for alerts
- **Detailed Tooltips**: Hover to see comprehensive battery information
- **Click Actions**: Click to open detailed status dialog (if zenity is available)
- **Real-time Updates**: Configurable update interval (default: 10 seconds)

## Display Format

The battery status displays with icons and text:

- `🔋 7.2V (45%) 🔌⚡` - Normal battery with charging and AC power
- `🔴 6.8V (15%) 🔋` - Low battery warning
- `💧 7.1V (42%) 🔌` - Saltwater intrusion detected
- `💦 7.3V (48%) ⚡` - High humidity detected
- `🔋 Service unavailable` - Battery service not available
- `🔋 Error` - Service error or timeout

## Icons Legend

- `🔋` - Normal battery
- `🔴` - Low battery alert
- `💧` - Saltwater intrusion
- `💦` - High humidity
- `🔌` - Charging active
- `⚡` - AC power present

## Troubleshooting

### Battery Status Not Appearing

1. Check if battery_water node is running:
   ```bash
   ros2 service list | grep battery_status
   ```

2. Test the display manually:
   ```bash
   /home/orangepi/argo/scripts/battery_status_wrapper.sh
   ```

3. Check ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 service call /battery_status std_srvs/srv/Trigger
   ```

### Service Not Available

If the battery service is not available:
1. Start the battery_water node:
   ```bash
   python3 /home/orangepi/argo/nodes/battery_water.py
   ```

2. Or start the full Argo system:
   ```bash
   al  # Using Argo CLI alias
   ```

### Error Handling

The battery panel includes robust error handling:
- **Throttled logging**: Errors are logged max once every 30 seconds
- **Graceful degradation**: Shows "Service unavailable" when service is down
- **Automatic recovery**: Resumes normal operation when service returns
- **Timeout handling**: 2-second timeout for service calls

## Configuration

### Update Intervals

- **Panel script**: 5 seconds (configurable with `--update-interval`)
- **Simple script**: One-shot display
- **XFCE4 Generic Monitor**: 10 seconds (recommended)

### Customization

You can modify the display format by editing the `_format_battery_display()` method in the Python scripts.

## Integration with Argo System

The battery panel integrates seamlessly with the Argo sailboat system:
- Uses the existing `/battery_status` service from `battery_water.py`
- Respects the same error handling and logging patterns
- Works with the Argo lifecycle management system
- Compatible with simulation mode (shows mock data)

## Dependencies

- ROS2 Humble
- Python 3
- XFCE4 desktop environment
- `battery_water.py` node running
- `std_srvs` ROS2 package

## Notes

- The panel will show "Service unavailable" when the Argo system is not running
- Battery status updates automatically when the Argo system starts
- No manual intervention required after initial setup
- Works in both real hardware and simulation modes

