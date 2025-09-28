# Remote Simulator Setup for Argo

This guide explains how to run the Argo simulator on a remote machine (`sensors-tobidh87.lan.ini.uzh.ch`) to offload CPU usage from your Orange Pi.

## Architecture

```
Orange Pi (Local)                    Remote Machine
┌─────────────────┐                 ┌─────────────────┐
│ Argo Control    │                 │ Simulator       │
│ - GPS, IMU, etc │                 │ - Physics       │
│ - Control logic │                 │ - Wind, etc     │
└─────────────────┘                 └─────────────────┘
         │                                   │
         │ SSH Tunnel (ROS2 Topics)          │
         │                                   │
         └───────────────────────────────────┘
```

## Quick Start

### 1. Initial Setup (One-time)

```bash
# Set up remote machine with Argo and dependencies
./scripts/setup_remote_simulator.sh
```

**Note**: The remote machine already has the argo project at `/home/tobi/Dropbox/GitHub/SensorsINI/argo`. The setup script will verify this and install dependencies.

### 2. Daily Usage

```bash
# Terminal 1: Start SSH tunnel
./scripts/remote_simulator_tunnel.sh

# Terminal 2: Start remote simulator
python3 scripts/remote_simulator_launch.py

# Terminal 3: Start local bridge
python3 nodes/argo_remote_simulator_bridge.py

# Terminal 4: Start local Argo nodes
python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo.yaml
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml
```

## Files Created

### Configuration
- `scripts/remote_simulator_config.json` - Single JSON configuration file
- `scripts/load_config.py` - Configuration loader for Python and shell scripts

### Scripts
- `scripts/setup_remote_simulator.sh` - One-time setup script
- `scripts/remote_simulator_tunnel.sh` - SSH tunnel for ROS2 communication
- `scripts/remote_simulator_launch.py` - Launches simulator on remote machine

### Nodes
- `nodes/argo_remote_simulator_bridge.py` - Local bridge to remote simulator

## Configuration

All remote simulator scripts use a single JSON configuration file:

### Configuration File (`scripts/remote_simulator_config.json`)
```json
{
  "remote": {
    "host": "sensors-tobidh87.lan.ini.uzh.ch",
    "user": "tobi",
    "argo_dir": "/home/tobi/Dropbox/GitHub/SensorsINI/argo"
  },
  "ros2": {
    "domain_id": 42
  },
  "network": {
    "local_port": 11311,
    "remote_port": 11311
  },
  "ssh": {
    "timeout": 10,
    "key_path": "~/.ssh/id_rsa"
  }
}
```

### Configuration Loader (`scripts/load_config.py`)
The configuration loader provides access from both Python and shell scripts:

```bash
# View full configuration
python3 scripts/load_config.py

# Export as shell environment variables
python3 scripts/load_config.py --export-shell

# Get specific value
python3 scripts/load_config.py --get remote.host
```

### Viewing Configuration
```bash
# View full configuration
python3 scripts/load_config.py

# Load into shell environment
eval "$(python3 scripts/load_config.py --export-shell)"
echo "Remote host: $REMOTE_HOST"
```

## How It Works

### 1. SSH Tunnel
The `remote_simulator_tunnel.sh` script creates an SSH tunnel that forwards local port 11311 to the remote machine's port 11311, enabling ROS2 communication.

### 2. Remote Simulator
The `remote_simulator_launch.py` script:
- Connects to the remote machine via SSH
- Starts the `argo_simulator_bridge.py` on the remote machine
- Monitors the remote process and forwards output

### 3. Local Bridge
The `argo_remote_simulator_bridge.py` node:
- Subscribes to sensor topics from the remote simulator
- Publishes control commands to the remote simulator
- Monitors connection status and reports issues

## Network Configuration

### ROS2 Domain ID
Both local and remote machines use `ROS_DOMAIN_ID=42` to ensure they can communicate.

### Ports
- **11311**: ROS2 communication (tunneled via SSH)
- **9090**: Foxglove bridge (if used)

### Firewall
Ensure the following ports are open:
- **22**: SSH (for tunnel and remote execution)
- **11311**: ROS2 (tunneled, not directly exposed)

## Troubleshooting

### Connection Issues

```bash
# Test SSH connection
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch

# Test ROS2 on remote
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "source /opt/ros/humble/setup.bash && ros2 node list"

# Check tunnel status
netstat -ln | grep 11311
```

### Performance Issues

```bash
# Monitor remote CPU usage
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "top -p \$(pgrep -f argo_simulator_bridge)"

# Monitor network usage
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "iftop -i eth0"
```

### Logs

```bash
# Remote simulator logs
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "journalctl -u argo-simulator -f"

# Local bridge logs
ros2 topic echo /rosout --filter "name=='argo_remote_simulator_bridge'"
```

## Benefits

1. **CPU Offloading**: Simulator physics runs on remote machine
2. **Better Performance**: Remote machine likely has more CPU/RAM
3. **Scalability**: Can run multiple simulators on different remote machines
4. **Isolation**: Simulator crashes don't affect local Argo system

## Security Considerations

- SSH keys are used for authentication (no passwords)
- ROS2 communication is tunneled through SSH
- Remote machine should be on trusted network
- Consider VPN for production use

## Alternative Configurations

### Multiple Remote Machines
You can run different components on different remote machines:

```bash
# Simulator on machine 1
python3 scripts/remote_simulator_launch.py --remote-host machine1.example.com

# Control logic on machine 2  
python3 scripts/remote_control_launch.py --remote-host machine2.example.com
```

### Local Fallback
The system automatically falls back to local mock simulator if remote connection fails.

## Maintenance

### Updating Configuration
To change remote machine settings, edit the JSON configuration file:

```bash
# Edit configuration
nano scripts/remote_simulator_config.json

# View updated configuration
python3 scripts/load_config.py
```

### Updating Remote Code
Since the remote machine uses Dropbox synchronization, code changes are automatically synced. You may need to restart the remote simulator to pick up changes:

```bash
# Stop remote simulator (Ctrl+C in terminal 2)
# Restart remote simulator
python3 scripts/remote_simulator_launch.py
```

### Monitoring Remote Resources
```bash
# Check disk space
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "df -h"

# Check memory usage
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "free -h"

# Check process status
ssh tobi@sensors-tobidh87.lan.ini.uzh.ch "ps aux | grep argo"
```

## Support

For issues with the remote simulator setup:
1. Check SSH connectivity
2. Verify ROS2 installation on remote machine
3. Check network connectivity and firewall rules
4. Review logs for specific error messages
