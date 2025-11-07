# Standard ROS2 Launch System for Argo

## Overview

This directory now contains a **standard ROS2 launch system** that replaces the custom lifecycle manager while maintaining equivalent functionality.

## Key Components

### 1. `argo_nodes.yaml` - Node Configuration
Human-readable YAML file that defines all Argo ROS2 nodes:
- Node names, executables, and descriptions
- Required vs optional nodes
- Critical vs non-critical nodes
- Simulation mode configuration

**No compilation needed** - just edit YAML and restart service.

### 2. `argo_launch.py` - ROS2 Launch File
Standard ROS2 launch file using Python launch API:
- Loads `argo_nodes.yaml` configuration
- Launches nodes based on mode (normal/simulation)
- Handles respawn for non-critical nodes
- Outputs to systemd journal

**Usage:**
```bash
ros2 launch launch/argo_launch.py
ros2 launch launch/argo_launch.py mode:=simulation
```

### 3. `argo_health_monitor.py` - Health Monitor Node
ROS2 node that continuously monitors node health:
- Checks node status every 2 seconds using `ros2 node list`
- Publishes health status on `/argo/health/status` topic
- Provides `/argo/health/status` service for queries
- Tracks which nodes are running/stopped

**This provides the continuous health monitoring that was missing.**

### 4. `argo_status.py` - Status Reporter
Command-line tool for status reporting:
- Uses standard ROS2 commands (`ros2 node list`, `ros2 service call`)
- Queries health monitor service
- Provides full and quick status modes

**Usage:**
```bash
python3 launch/argo_status.py           # Full status
python3 launch/argo_status.py --quick  # Quick one-line status
```

### 5. `argo_launch_standard.service` - Systemd Service
Updated systemd service file:
- Uses `ros2 launch` instead of custom Python script
- Starts health monitor automatically
- Restarts on failure (standard systemd behavior)

## Quick Start

### Installation

1. Install new systemd service:
   ```bash
   sudo cp launch/argo_launch_standard.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable argo_launch_standard.service
   ```

2. Start the service:
   ```bash
   sudo systemctl start argo_launch_standard.service
   ```

3. Check status:
   ```bash
   python3 launch/argo_status.py
   ```

### Checking Node Status

**Standard ROS2 commands work:**
```bash
ros2 node list                    # List all running nodes
ros2 topic list                   # List all topics
ros2 service list                 # List all services
ros2 service call /argo/health/status std_srvs/srv/Trigger  # Get health status
```

**Custom status reporter:**
```bash
python3 launch/argo_status.py           # Full status
python3 launch/argo_status.py --quick  # Quick status
```

## Configuration

### Adding a New Node

Edit `launch/argo_nodes.yaml`:

```yaml
nodes:
  - name: my_new_node
    executable: nodes/my_new_node.py
    required: false
    critical: false
    description: "My new node description"
```

Then restart the service:
```bash
sudo systemctl restart argo_launch_standard.service
```

### Node Configuration Options

- `name`: ROS2 node name (as it appears in `ros2 node list`)
- `executable`: Path to Python script (relative to ARGO_DIR)
- `required`: If false, system continues if node fails
- `critical`: If true, missing node is considered system failure
- `description`: Human-readable description
- `special`: If true, requires custom launch handling (e.g., `ros2 run`)
- `args`: Optional command-line arguments

## Comparison with Custom Lifecycle Manager

| Feature | Custom Manager | Standard ROS2 |
|---------|---------------|---------------|
| Systemd service | ✅ | ✅ |
| Status reporting | ✅ | ✅ |
| Shell aliases | ✅ | ✅ |
| Health monitoring | ❌ (only on status check) | ✅ (continuous) |
| Standard ROS2 tools | ❌ | ✅ |
| Human-readable config | ❌ (Python code) | ✅ (YAML) |
| Easy node addition | ❌ (code changes) | ✅ (YAML edit) |

## Migration

See `MIGRATION_GUIDE.md` for detailed migration steps from the custom lifecycle manager.

## Troubleshooting

### Health Monitor Not Running

Check if it's in the process list:
```bash
ps aux | grep argo_health_monitor
```

Start manually if needed:
```bash
source /opt/ros/humble/setup.bash
python3 launch/argo_health_monitor.py
```

### Nodes Not Launching

Check launch file:
```bash
ros2 launch launch/argo_launch.py --show-args
```

Check systemd logs:
```bash
sudo journalctl -u argo_launch_standard.service -f
```

### Status Command Fails

Ensure ROS2 is sourced:
```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

If health monitor isn't running, start it manually (should be automatic via systemd).

## Benefits of Standard ROS2 Approach

1. **Standard Tooling**: All standard ROS2 commands work (`ros2 node list`, `ros2 topic list`, etc.)
2. **Better Integration**: Works with ROS2 tooling ecosystem
3. **Continuous Monitoring**: Health monitor provides real-time status
4. **Easier Configuration**: Human-readable YAML (no compilation)
5. **Easier Maintenance**: Standard ROS2 patterns, easier for new developers
6. **Better Debugging**: Standard ROS2 tools and patterns

