# Standard ROS2 Launch Solution Summary

## Problem Statement

The custom `argo_lifecycle_manager.py` provides:
1. ✅ Systemd service that runs reliably at boot
2. ✅ Extensive status reporting
3. ❌ **Missing**: Continuous health monitoring (only checks on status command)

We want to migrate to standard ROS2 lifecycle management while maintaining equivalent functionality.

## Solution Overview

**Standard ROS2 Launch System** that:
- Uses `ros2 launch` (standard ROS2 way)
- Provides continuous health monitoring via dedicated node
- Maintains systemd integration
- Uses human-readable YAML configuration (no compilation)
- Preserves all existing functionality

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                 systemd service                         │
│         (argo_launch_standard.service)                  │
└───────────────────┬─────────────────────────────────────┘
                    │
                    ▼
         ┌──────────────────────┐
         │   ros2 launch         │
         │   argo_launch.py      │
         └──────────┬────────────┘
                    │
        ┌───────────┴────────────┐
        │                        │
        ▼                        ▼
┌───────────────┐      ┌──────────────────────┐
│  argo_nodes   │      │  argo_health_        │
│  .yaml        │      │  monitor.py          │
│  (config)     │      │  (monitors nodes)    │
└───────────────┘      └──────────┬───────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
                    ▼                             ▼
        ┌──────────────────────┐      ┌──────────────────┐
        │  All ROS2 nodes       │      │  /argo/health/   │
        │  (gps, controller,    │      │  status service  │
        │   anem, etc.)         │      │  (for queries)   │
        └───────────────────────┘      └──────────────────┘
```

## Key Components

### 1. `argo_nodes.yaml` - Configuration
**Human-readable YAML** defining all nodes:
```yaml
nodes:
  - name: controller_node
    executable: nodes/controller.py
    required: true
    critical: true
    description: "Autonomous navigation controller"
```

**Benefits:**
- No compilation needed
- Easy to add/remove nodes
- Clear documentation of node requirements

### 2. `argo_launch.py` - Launch File
**Standard ROS2 launch file** using Python launch API:
- Loads YAML configuration
- Launches nodes based on mode
- Handles respawn for non-critical nodes
- Starts health monitor automatically

**Usage:**
```bash
ros2 launch launch/argo_launch.py
ros2 launch launch/argo_launch.py mode:=simulation
```

### 3. `argo_health_monitor.py` - Health Monitor
**ROS2 node** that continuously monitors health:
- Checks `ros2 node list` every 2 seconds
- Publishes on `/argo/health/status` topic
- Provides `/argo/health/status` service
- Tracks node running/stopped status

**This solves the missing continuous monitoring!**

### 4. `argo_status.py` - Status Reporter
**Command-line tool** using standard ROS2 commands:
- `ros2 node list` - get running nodes
- `ros2 service call /argo/health/status` - get health
- Provides full and quick status modes

### 5. `argo_launch_standard.service` - Systemd Service
**Updated service file**:
- Uses `ros2 launch` instead of custom Python
- Restarts on failure (standard systemd)
- Outputs to journal

## Migration Path

### Step 1: Install New Components
Files are already created - no installation needed for files.

### Step 2: Update Systemd Service
```bash
sudo cp launch/argo_launch_standard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable argo_launch_standard.service
sudo systemctl start argo_launch_standard.service
```

### Step 3: Update Shell Aliases (Optional)
Update `dotfiles/.bash_aliases` to use new status command:
```bash
alias as='python3 $ARGO_DIR/launch/argo_status.py'
alias asq='python3 $ARGO_DIR/launch/argo_status.py --quick'
```

### Step 4: Test
```bash
# Check service
sudo systemctl status argo_launch_standard.service

# Check nodes
ros2 node list

# Check health
python3 launch/argo_status.py
ros2 service call /argo/health/status std_srvs/srv/Trigger
```

## Benefits Over Custom Solution

| Feature | Custom Manager | Standard ROS2 |
|---------|---------------|---------------|
| Systemd service | ✅ | ✅ |
| Status reporting | ✅ | ✅ |
| **Continuous health monitoring** | ❌ | ✅ **SOLVED** |
| Standard ROS2 tools | ❌ | ✅ |
| Human-readable config | ❌ | ✅ |
| Easy node addition | ❌ | ✅ |

## Preserved Functionality

✅ Systemd service at boot  
✅ Status reporting (full and quick)  
✅ Shell aliases and functions  
✅ Node health monitoring (now continuous!)  
✅ Simulation mode support  
✅ Excluded nodes (battery, BNO085)  

## Improvements

✅ **Continuous health monitoring** (was missing)  
✅ Standard ROS2 commands work (`ros2 node list`, etc.)  
✅ Better integration with ROS2 tooling  
✅ Human-readable configuration (YAML, no compilation)  
✅ Easier to add/remove nodes (just edit YAML)  
✅ Standard ROS2 patterns (easier for new developers)  

## Configuration Example

Adding a new node is now trivial:

1. Edit `launch/argo_nodes.yaml`:
```yaml
nodes:
  - name: my_new_node
    executable: nodes/my_new_node.py
    required: false
    critical: false
    description: "My new node"
```

2. Restart service:
```bash
sudo systemctl restart argo_launch_standard.service
```

**No code changes needed!**

## Next Steps

1. Test in development environment
2. Update shell aliases/functions (optional)
3. Switch systemd service (when ready)
4. Consider adding more health metrics to health monitor
5. Consider ROS2 lifecycle node support for graceful shutdown

## Files Created

- `launch/argo_nodes.yaml` - Node configuration
- `launch/argo_launch.py` - ROS2 launch file
- `launch/argo_health_monitor.py` - Health monitor node
- `launch/argo_status.py` - Status reporter
- `launch/argo_launch_standard.service` - Systemd service
- `launch/argo_start_standard.sh` - Start script
- `launch/argo_stop_standard.sh` - Stop script
- `launch/MIGRATION_GUIDE.md` - Detailed migration steps
- `launch/README_STANDARD_ROS2.md` - Usage documentation

## Testing Checklist

- [ ] Service starts at boot
- [ ] Nodes launch correctly
- [ ] Health monitor runs continuously
- [ ] Status command works
- [ ] Quick status works
- [ ] Shell aliases work (if updated)
- [ ] Adding a node works (edit YAML, restart)
- [ ] Simulation mode works
- [ ] Standard ROS2 commands work (`ros2 node list`, etc.)

