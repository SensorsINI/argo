# Argo Pause/Unpause System

The Argo system now includes a comprehensive pause/unpause mechanism that allows you to temporarily suspend processing in all managed nodes while keeping critical safety nodes running.

## Overview

The pause system consists of:

1. **Shared TogglePause Service** (`nodes/support/toggle_pause_service.py`) - A reusable service class for all nodes
2. **Lifecycle Manager Integration** - Centralized control through the lifecycle manager
3. **Node Integration** - All managed nodes check pause state before processing

## How It Works

### Node Behavior When Paused

- **Paused nodes**: Skip their main processing loops (timers, callbacks)
- **Critical nodes**: Continue running normally (battery_water.py, temp_monitor.py)
- **Safety features**: Always remain active (control authority, safety alerts)
- **Service responsiveness**: Pause service remains responsive for unpause requests

### Critical Nodes (Never Paused)

The following nodes are considered critical for safety and monitoring and will **never** be paused:

- `battery_water.py` - Battery monitoring and alerts
- `temp_monitor.py` - System temperature monitoring

### Pausable Nodes

All other nodes can be paused:

- `controller.py` - Autonomous navigation control
- `gps.py` - GPS data processing
- `imu.py` - IMU sensor processing
- `anem.py` - Wind sensor processing
- `rudder_sail_radio.py` - Control arbitration (but safety features remain active)

## Usage

### Via Lifecycle Manager Service

The lifecycle manager provides a `toggle_pause` service that automatically determines whether to pause or unpause based on current node states.

```bash
# Pause all pausable nodes
ros2 service call /toggle_pause std_srvs/srv/Trigger

# Unpause all nodes (call the same service again)
ros2 service call /toggle_pause std_srvs/srv/Trigger
```

### Via Individual Node Services

Each node also provides its own `toggle_pause` service:

```bash
# Pause/unpause specific nodes
ros2 service call /controller/toggle_pause std_srvs/srv/Trigger
ros2 service call /gps/toggle_pause std_srvs/srv/Trigger
ros2 service call /imu/toggle_pause std_srvs/srv/Trigger
```

### Using the Test Script

A test script is provided to demonstrate the pause functionality:

```bash
python3 scripts/test_pause_system.py
```

## Health Status

When nodes are paused, their health status topics will show `false` to indicate they are not actively processing:

- `/controller_health`
- `/gps_health`
- `/imu_health`
- `/anem_health`
- `/rudder_sail_radio_health`

Critical nodes will continue to show `true` when healthy.

## Safety Considerations

### What Remains Active When Paused

- **Control authority detection** - Human/robot control switching
- **Safety alerts** - Battery, temperature, and other critical alerts
- **Hardware safety** - High impedance mode and fail-safe features
- **Service responsiveness** - Pause/unpause services remain available

### What Gets Suspended When Paused

- **Sensor data processing** - GPS, IMU, wind sensor data processing
- **Autonomous control** - Controller algorithm execution
- **Data publishing** - Most sensor data topics (except safety alerts)
- **CPU-intensive operations** - Reduces system load

## Implementation Details

### Adding Pause Support to New Nodes

To add pause support to a new node:

1. Import the shared service:
```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from toggle_pause_service import TogglePauseService
```

2. Initialize in the constructor:
```python
def __init__(self):
    super().__init__('my_node')
    self.pause_service = TogglePauseService(self)
```

3. Add pause checks to processing methods:
```python
def timer_callback(self):
    if self.pause_service.is_paused():
        return  # Skip processing when paused
    # Normal processing here
```

### Service Interface

The `TogglePauseService` provides:

- `is_paused()` - Check current pause state
- `force_pause()` - Force pause (for external control)
- `force_unpause()` - Force unpause (for external control)
- `get_pause_status()` - Get detailed status information

## Troubleshooting

### Service Not Available

If the `toggle_pause` service is not available:

1. Ensure the lifecycle manager is running
2. Check that nodes have been started with pause support
3. Verify ROS2 communication is working

### Nodes Not Pausing

If nodes don't respond to pause requests:

1. Check node health status topics
2. Verify nodes are running and responsive
3. Check systemd journal for errors

### Critical Nodes Being Paused

Critical nodes should never be paused. If they are:

1. Check the `no_pause_nodes` list in the lifecycle manager
2. Verify node names match exactly (including .py extension)
3. Check that the pause service is properly initialized

## Examples

### Pause All Nodes for Maintenance

```bash
# Pause all pausable nodes
ros2 service call /toggle_pause std_srvs/srv/Trigger

# Perform maintenance tasks...

# Unpause all nodes
ros2 service call /toggle_pause std_srvs/srv/Trigger
```

### Pause Specific Node for Debugging

```bash
# Pause just the controller for debugging
ros2 service call /controller/toggle_pause std_srvs/srv/Trigger

# Debug other nodes while controller is paused...

# Unpause controller
ros2 service call /controller/toggle_pause std_srvs/srv/Trigger
```

### Check Pause Status

```bash
# Check health status of all nodes
ros2 topic echo /controller_health --once
ros2 topic echo /gps_health --once
ros2 topic echo /imu_health --once
```

When paused, health topics will show `data: false`.
