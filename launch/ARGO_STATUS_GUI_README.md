# Argo Status Desktop GUI

A lightweight desktop application for monitoring the Argo sailboat system status in real-time.

## 🚀 Features

- **Real-time Monitoring**: Auto-refreshing status display (every 5 seconds)
- **Service Status**: Monitor systemd services (argo-launch, argo-record)
- **ROS2 Nodes**: Track all ROS2 nodes with CPU/memory usage
- **System Resources**: Monitor system load, memory, and storage
- **Color-coded Status**: Green (good), yellow (warning), red (error)
- **Scrollable Interface**: Handle large amounts of status information
- **Lightweight**: Uses tkinter (built into Python) - no extra dependencies

## 📋 Requirements

- Python 3.x with tkinter support
- Access to Argo system files (argo_status_check.py, argo_node_utils.py)
- ROS2 environment (for full functionality)

## 🎮 Usage

### Quick Start
```bash
# Navigate to launch directory
cd /home/orangepi/argo/launch

# Run the GUI (recommended method)
./argo_status_gui.sh

# Or run directly with Python
python3 argo_status_gui.py
```

### GUI Controls

- **🔄 Refresh Button**: Manually refresh status immediately
- **Auto Refresh Checkbox**: Toggle automatic refresh on/off
- **Mouse Wheel**: Scroll through status panels
- **Window Close**: Close the application

## 📊 Status Panels

### 1. **📋 SYSTEMD SERVICES**
- Shows status of argo-launch service with PID, CPU, and memory usage
- Displays argo-record service status (ROS2-based)

### 2. **🤖 ROS NODES**
- Lists all discovered ROS2 nodes
- Shows running status with PID, CPU%, and memory% for each
- Highlights stopped nodes in red

### 3. **💻 SYSTEM RESOURCES**
- System load average
- Memory usage percentage
- Storage usage (free space and percentage used)

### 4. **📊 SUMMARY**
- Overall running nodes count
- Total CPU and memory usage
- Overall system status indicator

## 🎨 Color Coding

- **🟢 Green**: Normal operation, good status
- **🟡 Yellow**: Warning levels, partial operation
- **🔴 Red**: Error conditions, stopped services
- **🔵 Blue**: Informational status

## 🔧 Troubleshooting

### GUI Won't Start
```bash
# Check if tkinter is installed
python3 -c "import tkinter"

# If not installed (Ubuntu/Debian):
sudo apt-get install python3-tk

# Check required modules
cd /home/orangepi/argo/launch
python3 -c "from argo_status_check import OptimizedArgoStatusChecker"
```

### No Status Data
- Ensure you're running from the correct directory (`/home/orangepi/argo/launch`)
- Check that Argo system files are present
- Verify ROS2 environment is properly sourced

### Permission Issues
- Some status checks may require sudo privileges
- Run the launcher script which handles environment setup

## 🔗 Integration

This GUI leverages the existing Argo infrastructure:
- **argo_status_check.py**: Core status checking functionality
- **argo_node_utils.py**: ROS2 node discovery and management
- **ROS2 ecosystem**: For real-time node monitoring

## 📝 Notes

- The application auto-refreshes every 5 seconds by default
- All status data is collected in real-time from the actual system
- The GUI is designed to be lightweight and responsive
- Scrolling is supported for systems with many nodes
- Window can be resized as needed

## 🚢 Part of Argo Launch System

This GUI is part of the comprehensive Argo launch and monitoring system. For full system control, also see:
- `argo_gui.py` - Interactive CLI interface
- `argo_status_check.py` - Command-line status checker
- `argo_lifecycle_manager.py` - Main system launcher
