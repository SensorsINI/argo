# Argo Launch Directory

This directory contains all the launch scripts, systemd services, and management tools for the Argo autonomous sailboat system.

## 📁 Directory Contents

### 🚀 **Core Launch Files**

#### **ROS2 Launch System**
- **`argo_lifecycle_manager.py`** - Main ROS2 lifecycle manager that directly launches all sensor nodes and control systems
  - Dynamic node discovery from the `nodes/` directory
  - Fault-tolerant operation with partial node failure support
  - Direct node launching without intermediate launch files
  - Usage: `python3 launch/argo_lifecycle_manager.py run|stop|restart|status|monitor`

#### **Interactive Control**
- **`argo_gui.py`** - Interactive CLI GUI for real-time monitoring and control
  - Real-time ROS2 node status monitoring
  - Service control (start/stop/restart)
  - Recording control with color-coded status
  - Keystroke commands for quick control
  - Usage: `sudo python3 launch/argo_gui.py`

- **`argo_gui.sh`** - Shell wrapper for the GUI with proper environment setup
  - Ensures sudo privileges and ROS2 environment
  - Usage: `sudo ./launch/argo_gui.sh`

### 🔧 **System Management**

#### **Status and Monitoring**
- **`argo_lifecycle_manager.py`** - Main lifecycle management and status system
  - Systemd services status
  - ROS2 nodes status
  - System resources and storage
  - Usage: `python3 argo_lifecycle_manager.py status`

- **`storage_monitor.py`** - Storage space monitoring for recording
  - Calculates remaining recording hours
  - Displays storage warnings
  - Usage: `python3 storage_monitor.py`

- **`storage_notifications.py`** - GUI storage notifications
  - Desktop notifications for storage warnings
  - Used by systemd service for automated monitoring

#### **Power Control System**
- **`argo_power_control.py`** - Power control daemon for Orange Pi Zero 2W
  - GPIO-based power button monitoring
  - LED status indicators
  - Safe shutdown procedures
  - See `ARGO_POWER_CONTROL_README.md` for detailed documentation

- **`argo_power_control.service`** - Systemd service for power control
  - Starts early in boot process
  - Handles graceful shutdown with power relay control

### 📦 **Recording System**

#### **Data Recording**
- **`argo_record.sh`** - ROS2 bag recording script
  - Handles complete system recording
  - Automatic timestamped bag naming
  - Logging and cleanup on exit
  - Usage: `./launch/argo_record.sh`

### 🔔 **Notification System**

#### **Startup and Warnings**

- **`argo_terminal_warning.sh`** - Terminal warning system
  - Displays important system warnings
  - Storage and service alerts

#### **Setup Scripts**
- **`setup_gui_notifications.sh`** - GUI notification setup
  - Configures desktop notification system
  - Sets up storage monitoring notifications

- **`setup_terminal_warning.sh`** - Terminal warning setup
  - Configures terminal-based warning system

### ⚙️ **Systemd Services**

#### **Core Services**
- **`argo-launch.service`** - Main ROS2 launch service
  - Starts the complete Argo system
  - Uses `argo_lifecycle_manager.py` as entry point
  - Configured for user service (not auto-start by default)

- **Recording** - ROS2 service-based recording (no separate systemd service)
  - Records system data using `record.py` ROS2 node
  - Controlled via ROS2 services: `/argo/recording/start` and `/argo/recording/stop`
  - Depends on `argo-launch.service`
  - Manual start only


- **`argo-storage-monitor.service`** - Storage monitoring service
  - Monitors storage space and sends GUI notifications
  - Runs in GUI environment only
  - Automatic restart on failure

#### **Power Control Service**
- **`argo-power-control.service`** - Power control system service
  - Manages GPIO-based power control
  - Handles power button and LED indicators
  - Critical for safe shutdown procedures

### 🔧 **Configuration**

#### **Environment Files**
- **`argo.env`** - ROS2 environment configuration
  - Sets ROS2 version and distribution
  - Configures logging and network settings
  - Used by all systemd services

#### **Help System**
- **`argo_help.sh`** - Quick help reference
  - Lists available commands and aliases
  - Usage: `./argo_help.sh`

## 🚀 **Quick Start**

### **Manual Launch**
```bash
# Start the complete Argo system
python3 launch/argo_lifecycle_manager.py run

# Launch interactive GUI
sudo ./launch/argo_gui.sh

# Check system status
python3 launch/argo_lifecycle_manager.py status
```

### **Service Management**
```bash
# Install and start services
sudo systemctl --user link ~/argo/launch/argo-launch.service
sudo systemctl --user start argo-launch

# Check service status
sudo systemctl --user status argo-launch

# View logs
sudo journalctl --user -u argo-launch -f
```

### **Recording Control**
```bash
# Start recording
sudo systemctl --user start argo-record

# Stop recording
sudo systemctl --user stop argo-record

# Manual recording
./launch/argo_record.sh
```

## 📋 **Service Dependencies**

```
argo-launch.service
├── argo.env (environment)
└── argo_lifecycle_manager.py (main lifecycle manager)

recording (ROS2 service)
├── argo-launch.service (dependency)
├── argo.env (environment)
└── argo_record.sh (recording script)

argo-storage-monitor.service
├── storage_notifications.py (notification daemon)
└── DISPLAY environment (GUI required)

argo-power-control.service
├── argo_power_control.py (power daemon)
└── GPIO devices (hardware access)
```

## 🔧 **Installation**

### **From Project Root**
```bash
# Install all services
make install

# Install specific components
make install-argo-cli
make install_argo_power_control
```

### **Manual Installation**
```bash
# Install services
sudo systemctl --user link ~/argo/launch/argo-launch.service
sudo systemctl --user link ~/argo/launch/recording (ROS2 service)
sudo systemctl --user link ~/argo/launch/argo-storage-monitor.service

# Install power control (requires sudo)
sudo cp argo-power-control.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable argo-power-control
```

## 🛠️ **Troubleshooting**

### **Common Issues**
1. **Service won't start**: Check logs with `journalctl --user -u <service-name> -f`
2. **Permission errors**: Ensure proper user/group permissions
3. **ROS2 not found**: Source ROS2 environment: `source /opt/ros/humble/setup.bash`
4. **GUI issues**: Ensure DISPLAY environment variable is set

### **Log Locations**
- **Service logs**: `journalctl --user -u <service-name>`
- **Recording logs**: `/tmp/argo_record.log`
- **Power control logs**: `journalctl -u argo-power-control`

### **Status Checking**
```bash
# Comprehensive status check
python3 launch/argo_lifecycle_manager.py status

# Storage monitoring
python3 launch/storage_monitor.py

# Service status
sudo systemctl --user status argo-launch argo-record argo-storage-monitor
sudo systemctl status argo-power-control
```

## 📚 **Documentation**

- **Power Control**: See `ARGO_POWER_CONTROL_README.md` for detailed power control documentation
- **Main Project**: See `~/argo/README.md` for overall project documentation
- **Makefile Help**: Run `make help` from project root for available commands

## 🔄 **Maintenance**

### **Regular Tasks**
- Monitor storage space before long recordings
- Check service logs for errors
- Update bag file cleanup (older than 7 days)
- Verify GPIO permissions for power control

### **Updates**
- Services are automatically reloaded when files change
- Use `sudo systemctl daemon-reload` after service file modifications
- Restart services after configuration changes


