# Argo Remote Desktop VNC Setup

This guide explains how to set up and use VNC (Virtual Network Computing) for remote desktop access to the Argo sailboat control system.

## Overview

The Argo system uses TightVNC server to provide remote desktop access with Xfce4 desktop environment. This allows you to:
- Access the full desktop environment remotely
- Run GUI applications for development and monitoring
- Manage the Argo system without physical access
- Develop headlessly while having desktop access when needed

## Prerequisites

- Orange Pi Zero 2W running Argo system
- Network connectivity between your local machine and Argo
- Router configured for port forwarding (if accessing from outside local network)

## VNC Server Setup (on Argo)

### Installation
TightVNC server is already installed on the Argo system:
```bash
sudo apt install tightvncserver
```

### Configuration
The VNC server is configured to run on display `:2` with Xfce4 desktop environment.

**Configuration files:**
- **Startup script**: `/home/orangepi/argo/scripts/start_vnc_xfce.sh`
- **VNC startup**: `/home/orangepi/.vnc/xstartup`
- **Password file**: `/home/orangepi/.vnc/passwd`

### Starting VNC Server
```bash
# Start VNC server with Xfce4
./scripts/start_vnc_xfce.sh

# Or manually
vncserver :2 -geometry 1920x1080 -depth 24
```

### Stopping VNC Server
```bash
# Stop VNC server
vncserver -kill :2
```

## VNC Client Setup (on your local machine)

### Ubuntu/Debian
```bash
# Install TigerVNC Viewer (recommended)
sudo apt update
sudo apt install tigervnc-viewer

# Or install TightVNC Viewer
sudo apt install xtightvncviewer

# Or install RealVNC Viewer
sudo apt install realvnc-vnc-viewer
```

### Other Linux Distributions
```bash
# Fedora/CentOS/RHEL
sudo dnf install tigervnc

# Arch Linux
sudo pacman -S tigervnc

# openSUSE
sudo zypper install tigervnc
```

### Windows
- Download RealVNC Viewer from https://www.realvnc.com/download/viewer/
- Or TightVNC Viewer from https://www.tightvnc.com/download.php

### macOS
```bash
# Using Homebrew
brew install --cask vnc-viewer

# Or download RealVNC Viewer from https://www.realvnc.com/download/viewer/
```

## Connection Details

### Connection Information
- **Server**: Argo Orange Pi IP address
- **Port**: `5902`
- **Display**: `:2`
- **Password**: `argodev` (8 characters)
- **Desktop Environment**: Xfce4
- **Resolution**: 1920x1080

### Connection Methods

#### Direct Connection
```bash
# From command line
vncviewer <argo-ip>:5902

# Example
vncviewer 192.168.1.100:5902
```

#### SSH Tunnel (More Secure)
```bash
# Create SSH tunnel
ssh -L 5902:localhost:5902 orangepi@<argo-ip>

# In another terminal, connect locally
vncviewer localhost:5902
```

#### GUI Connection
Most VNC clients provide a GUI where you can enter:
- **Server**: `<argo-ip>:5902`
- **Password**: `argodev`

## Network Configuration

### Local Network Access
If you're on the same local network as Argo:
```bash
vncviewer <local-argo-ip>:5902
```

### Remote Access (Internet)
For access from outside the local network, configure router port forwarding:

1. **Router Port Forwarding**:
   - External Port: `5902` (or custom port like `15902`)
   - Internal IP: Argo's local IP address
   - Internal Port: `5902`
   - Protocol: TCP

2. **Connection**:
   ```bash
   vncviewer <router-external-ip>:5902
   ```

### Firewall Configuration
Ensure the firewall allows VNC connections:
```bash
# On Argo system
sudo ufw allow 5902/tcp
```

## Troubleshooting

### Common Issues

#### Connection Refused
- Check if VNC server is running: `ps aux | grep Xtightvnc`
- Verify port is open: `netstat -tlnp | grep 5902`
- Check firewall settings

#### Authentication Failed
- Verify password is correct: `argodev`
- Check password file permissions: `ls -la ~/.vnc/passwd`
- Reset password if needed: `vncpasswd`

#### Desktop Not Loading
- Check VNC log: `tail -f ~/.vnc/orangepizero2w:2.log`
- Verify Xfce4 is installed: `dpkg -l | grep xfce4`
- Restart VNC server: `vncserver -kill :2 && ./scripts/start_vnc_xfce.sh`

#### Performance Issues
- Reduce color depth: `vncserver :2 -depth 16`
- Lower resolution: `vncserver :2 -geometry 1280x720`
- Use compression: Add `-compresslevel 6` to vncserver command

### Log Files
- **VNC Server Log**: `~/.vnc/orangepizero2w:2.log`
- **System Log**: `journalctl -u vncserver@:2.service` (if using systemd)
- **Xfce4 Log**: Check for errors in VNC log file

### Debug Commands
```bash
# Check VNC processes
ps aux | grep vnc

# Check VNC server status
vncserver -list

# Check network connectivity
netstat -tlnp | grep 5902

# Check VNC log
tail -f ~/.vnc/orangepizero2w:2.log

# Test local connection
vncviewer localhost:5902
```

## Security Considerations

### Password Security
- Change default password: `vncpasswd`
- Use strong passwords (8+ characters)
- Consider using SSH tunneling for additional security

### Network Security
- Use SSH tunnels when possible
- Configure firewall to restrict access
- Consider using non-standard ports
- Use VPN for remote access when available

### Access Control
- Limit VNC access to trusted networks
- Monitor connection logs
- Disable VNC when not needed

## Usage Examples

### Development Workflow
1. Start VNC server on Argo: `./scripts/start_vnc_xfce.sh`
2. Connect from local machine: `vncviewer <argo-ip>:5902`
3. Open terminal in VNC session
4. Run Argo commands: `al`, `as`, `ar`, etc.
5. Use file manager for file operations
6. Run GUI applications as needed

### Monitoring and Control
1. Connect to VNC desktop
2. Open system monitor to check resources
3. Use Argo GUI: `sudo python3 /home/orangepi/argo/launch/argo_gui.py`
4. Monitor logs: `journalctl -u argo-launch.service -f`
5. Check system status: `argo_status`

### File Management
1. Connect to VNC desktop
2. Open Thunar file manager
3. Navigate to `/home/orangepi/argo/`
4. Edit configuration files
5. Manage data logs and recordings

## Integration with Argo System

### Argo CLI Integration
The VNC desktop provides access to all Argo CLI commands:
```bash
# Start Argo system
al

# Check status
as

# Start recording
ar

# Stop recording
ac

# Monitor mode
am
```

### ROS2 Development
Use VNC desktop for ROS2 development:
```bash
# Start ROS2 nodes
ros2 run argo_nodes gps

# Monitor topics
ros2 topic list
ros2 topic echo /gps_cog

# Use Foxglove
# Open browser and navigate to Foxglove
```

### System Administration
Perform system administration tasks:
```bash
# Update system
sudo apt update && sudo apt upgrade

# Check system resources
htop

# Monitor network
iftop

# Check disk usage
df -h
```

## Maintenance

### Regular Tasks
- Monitor VNC server status
- Check log files for errors
- Update VNC server if needed
- Backup VNC configuration

### Updates
```bash
# Update VNC server
sudo apt update
sudo apt upgrade tightvncserver

# Update Xfce4
sudo apt upgrade xfce4
```

## Support

For issues with VNC setup:
1. Check this documentation
2. Review log files
3. Test network connectivity
4. Verify configuration files
5. Contact system administrator

## Related Documentation

- [Argo System Overview](../README.md)
- [Argo CLI Commands](README_CLI.md)
- [Network Configuration](README_NETWORK.md)
- [Troubleshooting Guide](README_TROUBLESHOOTING.md)
