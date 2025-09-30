# Argo Dotfiles Configuration

This directory contains shell configuration files that provide convenient aliases and automatic status checking for the Argo sailboat control system.

## Files Overview

### `.bash_aliases`
Contains all Argo-related shell aliases for quick command execution.

### `.bashrc`
Main bash configuration that sources the aliases and sets up automatic status checking.

### `.tmux.conf`
Tmux configuration with useful plugins for terminal session management.

## Argo Shell Commands

### Basic Control Commands
- `al` - Start Argo nodes (`argo_start.sh`)
- `aq` - Stop Argo nodes (`argo_stop.sh`)
- `ars` - Restart Argo nodes (`argo_restart.sh`)
- `as` - Show Argo status (`argo_status.sh`)
- `am` - Monitor mode (continuous status monitoring)

### Recording Commands
- `ar` - Start data recording (requires service to be running)
- `ac` - Stop data recording (requires service to be running)

### Simulation Commands
- `asim` - Start Argo in LOCAL simulation mode
- `asimr` - Start Argo in REMOTE simulation mode

### Utility Commands
- `ah` - Show Argo help
- `ag` - Start Argo GUI (requires sudo)
- `alog` - Show Argo launch logs (follow mode)
- `argo_status` - Manual status check (always shows full details)

## Automatic Status Checking

The system includes an intelligent automatic status checking mechanism that runs on every shell startup:

### How It Works

1. **Quick Timer Function**: `argo_quick_timer()` checks if 5+ minutes have passed since the last status check
2. **Timestamp Tracking**: Uses `~/.argo_last_check` file to track the last check time
3. **Condensed Output**: Only shows system metrics (CPU, memory, disk, temperature) to avoid spam
4. **Manual Override**: Use `argo_status` for full detailed status anytime

### Status Check Behavior

- **First Login**: Always shows status (no previous timestamp)
- **Frequent Logins**: Shows condensed status only if 5+ minutes since last check
- **Manual Check**: `argo_status` always shows full details and updates timestamp
- **Service Integration**: Status checks update the timestamp file to prevent redundant checks

### Status Display Format

**Condensed (Automatic)**:
```
🚢 ARGO: CPU 15.2% | Mem. 45.3% | Free Disk 2.1GB (78.5% used) | CPU Temp. 42°C | Batt. 12.4V (85%)
```

**Full (Manual)**:
```
🚢 ARGO STATUS - 2024-01-15 14:30:25
============================================================
📋 LAUNCH SERVICE: 🟢 RUNNING
🤖 ROS NODES:
  pwm.py: 🟢 RUNNING
  control.py: 🟢 RUNNING
  gps.py: 🟢 RUNNING
  ...
📊 SYSTEM: CPU 15.2% | Mem. 45.3% | Free Disk 2.1GB (78.5% used) | CPU Temp. 42°C | Batt. 12.4V (85%)
============================================================
```

## Configuration Details

### PATH Setup
The dotfiles add Argo directories to the PATH:
- `~/argo/launch` - Launch scripts
- `~/argo/nodes` - Node executables

### Deprecated Functions
- `argo_status_check()` - Deprecated, use `argo_status` instead

### Tmux Integration
The tmux configuration includes:
- Mouse support
- Session restoration
- Plugin management
- Useful plugins for development

## Usage Examples

```bash
# Start Argo system
al

# Check status (full details)
as
# or
argo_status

# Monitor continuously
am

# Start simulation
asim

# Check logs
alog

# Quick system check (automatic on login)
# No command needed - runs automatically
```

## Troubleshooting

### Status Not Updating
- Check if `~/.argo_last_check` file exists and has valid timestamp
- Manual status check: `argo_status`

### Aliases Not Working
- Reload configuration: `relogin` or `source ~/.bashrc`
- Check if `.bashrc` sources `.bash_aliases`

### Service Issues
- Check service status: `systemctl status argo-launch.service`
- View logs: `alog` or `journalctl -u argo-launch.service -f`

## File Locations

- **Aliases**: `~/argo/dotfiles/.bash_aliases`
- **Bash Config**: `~/argo/dotfiles/.bashrc`
- **Tmux Config**: `~/argo/dotfiles/.tmux.conf`
- **Timestamp File**: `~/.argo_last_check`
- **Argo Directory**: `~/argo/`
