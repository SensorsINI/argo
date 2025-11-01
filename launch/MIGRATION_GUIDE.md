# Migration Guide: Custom Lifecycle Manager → Standard ROS2 Launch

## Overview

This guide describes migrating from the custom `argo_lifecycle_manager.py` to standard ROS2 launch system while maintaining equivalent functionality.

## Key Benefits

1. **Standard ROS2**: Uses `ros2 launch` - the standard ROS2 way to start nodes
2. **Continuous Health Monitoring**: Health monitor node provides real-time status
3. **Systemd Integration**: Still works reliably at boot
4. **Human-Readable Config**: YAML configuration file (no compilation)
5. **Better Tooling**: Standard ROS2 commands work (`ros2 node list`, `ros2 topic list`, etc.)

## Architecture Changes

### Before (Custom)
```
systemd service → argo_lifecycle_manager.py → subprocess.Popen() → nodes
                                 ↓
                          Status reporting via Python
```

### After (Standard)
```
systemd service → ros2 launch → argo_launch.py → nodes
              ↓
         argo_health_monitor.py (monitors all nodes)
              ↓
         /argo/health/status service
              ↓
         argo_status.py (reports status)
```

## Components

### 1. `argo_nodes.yaml` - Node Configuration
Human-readable YAML file defining all nodes:
- Node names and executables
- Required vs optional nodes
- Critical vs non-critical nodes
- Simulation mode nodes

### 2. `argo_launch.py` - ROS2 Launch File
Standard ROS2 launch file using Python launch API:
- Loads `argo_nodes.yaml`
- Launches nodes based on mode (normal/simulation)
- Handles respawn for non-critical nodes
- Outputs to systemd journal

### 3. `argo_health_monitor.py` - Health Monitor Node
ROS2 node that continuously monitors node health:
- Checks node status every 2 seconds
- Publishes health status on `/argo/health/status` topic
- Provides `/argo/health/status` service for queries
- Tracks which nodes are running/stopped

### 4. `argo_status.py` - Status Reporter
Command-line tool for status reporting:
- Uses standard ROS2 commands (`ros2 node list`, `ros2 service call`)
- Queries health monitor service
- Provides full and quick status modes

### 5. `argo_launch_standard.service` - Systemd Service
Updated systemd service file:
- Uses `ros2 launch` instead of custom Python script
- Starts health monitor as background process
- Restarts on failure (standard systemd behavior)

## Migration Steps

### Step 1: Install New Components

The new files are already created:
- `launch/argo_nodes.yaml`
- `launch/argo_launch.py`
- `launch/argo_health_monitor.py`
- `launch/argo_status.py`
- `launch/argo_launch_standard.service`

### Step 2: Update Shell Aliases

Update `dotfiles/.bash_aliases` to use new status command:

```bash
# Old:
alias as='$ARGO_DIR/launch/argo_status.sh'

# New:
alias as='python3 $ARGO_DIR/launch/argo_status.py'
alias asq='python3 $ARGO_DIR/launch/argo_status.py --quick'  # Quick status
```

### Step 3: Update Shell Functions

Update `dotfiles/.bashrc` to use new status function:

```bash
# Old:
argo_status() {
    python3 "$ARGO_DIR/launch/argo_lifecycle_manager.py" status
}

# New:
argo_status() {
    python3 "$ARGO_DIR/launch/argo_status.py"
}

argo_quick_status() {
    python3 "$ARGO_DIR/launch/argo_status.py" --quick
}
```

### Step 4: Switch Systemd Service

1. Stop old service:
   ```bash
   sudo systemctl stop argo_launch.service
   ```

2. Install new service:
   ```bash
   sudo cp launch/argo_launch_standard.service /etc/systemd/system/
   sudo systemctl daemon-reload
   ```

3. Start new service:
   ```bash
   sudo systemctl start argo_launch_standard.service
   sudo systemctl enable argo_launch_standard.service
   ```

4. Optional: Remove old service:
   ```bash
   sudo systemctl disable argo_launch.service
   ```

### Step 5: Test New System

1. Check service status:
   ```bash
   sudo systemctl status argo_launch_standard.service
   ```

2. Check ROS2 nodes:
   ```bash
   ros2 node list
   ```

3. Check health status:
   ```bash
   python3 launch/argo_status.py
   ros2 service call /argo/health/status std_srvs/srv/Trigger
   ```

4. Test quick status:
   ```bash
   python3 launch/argo_status.py --quick
   ```

## Configuration

### Adding New Nodes

Edit `launch/argo_nodes.yaml`:

```yaml
nodes:
  - name: my_new_node
    executable: nodes/my_new_node.py
    required: false
    critical: false
    description: "My new node description"
```

No code changes needed - just edit YAML and restart service.

### Node Configuration Options

- `name`: ROS2 node name (as it appears in `ros2 node list`)
- `executable`: Path to Python script (relative to ARGO_DIR)
- `required`: If false, system continues if node fails
- `critical`: If true, missing node is considered system failure
- `description`: Human-readable description
- `special`: If true, requires custom launch handling (e.g., `ros2 run`)

## Compatibility

### Preserved Functionality

✅ Systemd service at boot  
✅ Status reporting (full and quick)  
✅ Shell aliases and functions  
✅ Node health monitoring  
✅ Simulation mode support  
✅ Excluded nodes (battery, BNO085)  

### Improvements

✅ Standard ROS2 commands work  
✅ Better integration with ROS2 tooling  
✅ Health monitoring is continuous (not just on status check)  
✅ Human-readable configuration (no compilation)  
✅ Easier to add/remove nodes  

### Removed (No Longer Needed)

❌ Custom lifecycle manager Python code  
❌ Direct subprocess management  
❌ Custom process tracking  

## Troubleshooting

### Health Monitor Not Starting

Check if ROS2 is available:
```bash
ros2 node list
```

If health monitor isn't running, start manually:
```bash
source /opt/ros/humble/setup.bash
python3 launch/argo_health_monitor.py
```

### Nodes Not Launching

Check launch file syntax:
```bash
ros2 launch launch/argo_launch.py --show-args
```

Check systemd logs:
```bash
sudo journalctl -u argo_launch_standard.service -f
```

### Status Command Fails

Ensure health monitor is running:
```bash
ros2 service list | grep health
```

If missing, health monitor needs to be started (should be automatic via systemd).

## Rollback Plan

If issues occur, rollback to old system:

1. Stop new service:
   ```bash
   sudo systemctl stop argo_launch_standard.service
   ```

2. Start old service:
   ```bash
   sudo systemctl start argo_launch.service
   ```

3. Restore old aliases in `dotfiles/.bash_aliases` and `dotfiles/.bashrc`

## Next Steps

1. Test thoroughly in development environment
2. Update documentation references
3. Update any scripts that call lifecycle manager directly
4. Consider adding more health metrics to health monitor
5. Consider adding ROS2 lifecycle node support for graceful shutdown

