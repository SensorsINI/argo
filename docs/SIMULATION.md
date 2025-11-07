# Argo Simulation Guide

This guide explains how to run the Argo in local simulation mode using the robust ROS2 launch system.

## Overview

When running simulation, several hardware nodes conflict with the simulator topics and are automatically excluded.

**Conflicting Hardware Nodes (excluded in simulation):**
- `gps.py` → publishes `/gps_cog`, `/gps_sog`, `/gps_velocity`
- `bno085.py` → publishes `/compass`
- `anem.py` → publishes `/anem_speed_angle_temp`
- `rudder_sail_radio.py` → subscribes to `/rudder_sail_servo`

**Nodes launched in simulation:**
- `argo_unified_simulator_bridge.py` → provides simulated sensor data
- `controller.py` → handles autonomous navigation
- `sailing_area_publisher.py` → provides map boundaries for visualization
- `foxglove_bridge` → provides the websocket for Foxglove Studio

## Local Simulation

The local simulation runs all necessary components directly on the local machine. It uses the `sailboat-playground` simulator for realistic physics or falls back to a simpler mock simulator if needed.

### Usage

The entire simulation environment is managed by a ROS2 launch file. To start it, simply use the `asim` alias in your terminal:

```bash
# Start the complete local simulation environment
asim
```

This command will:
1.  Launch the simulator bridge, controller, and sailing area publisher.
2.  Start the `foxglove_bridge` automatically.
3.  Ensure all processes are managed correctly and shut down cleanly with `Ctrl+C`.

This is now the only supported method for starting the local simulation. The previous methods using `argo_lifecycle_manager.py` or running nodes manually are deprecated due to stability issues.

### Visualization

Once the simulation is running, you can connect to it using Foxglove Studio.
- **Connection URL:** `ws://localhost:8765`

### External Control

The simulator can be controlled via external tools:

#### Foxglove Joystick Panel (Recommended)

1. **Install Joystick Panel**: In Foxglove Studio, add the "Joystick" community panel extension from the Extension Marketplace

2. **Critical Configuration Steps**:
   - **Publish Section**:
     - **Publish Mode**: Set to **"On"** (this is critical - must be enabled!)
     - **Pub Joy Topic**: Set to `/joy`
   - **Data Source Section**:
     - **Data Source**: Select one of:
       - **"Interactive Display Mode"** - Use mouse/touchscreen to control (recommended)
       - **"Keyboard Mode"** - Use keyboard keys
       - **"Gamepad Mode"** - Use a physical gamepad connected to your computer
     - **DO NOT** select "Subscribed Joy To..." - that's for monitoring, not controlling
   - **Display Section**:
     - **Display Mode**: "Auto-Generated" is fine
     - Other display settings can be left as default

3. **Control Mapping**:
   - **Axis 0** (default): Rudder control (-1=full left, +1=full right)
   - **Axis 1** (default): Sail control (-1=pulled in, +1=let out)
   - Sail axis is inverted by default (pull up = sail out)

4. **Operation**: 
   - Once configured, the panel should show joystick controls that you can interact with
   - Moving the joystick indicators will publish Joy messages to `/joy`
   - The simulator bridge automatically converts these to control commands

5. **Troubleshooting**:
   - If panel shows "waiting for first data": **Enable Publish Mode to "On"**
   - If nothing happens when you interact: Check that Data Source is set to Interactive/Keyboard/Gamepad mode (not Subscribe mode)
   - Verify topic is correct: `/joy`

6. **Customization**: You can change axis mapping via ROS2 parameters:
   ```bash
   --ros-args -p joy_rudder_axis:=0 -p joy_sail_axis:=1 -p joy_sail_invert:=true
   ```

#### Foxglove Teleop Panel (Alternative)

1. **Install Teleop Panel**: In Foxglove Studio, add the "Teleop" community panel extension
2. **Configure Topic**: Set the Teleop panel to publish to `/rudder_sail_radio` topic
3. **Control Mapping**: 
   - **Left/Right arrows**: Control rudder (x-axis, -1=full left, +1=full right)
   - **Up/Down arrows**: Control sail (y-axis, -1=pulled in, +1=let out)
4. **Operation**: Unlike real RC radio control, Teleop panel controls are position-based rather than rate-based. The position is held until you change it (no automatic return to neutral).
5. **Note**: Teleop only supports binary values. For smooth scalar control, use the Joystick panel instead.

#### Terminal Control

You can also control the simulator from the terminal:

```bash
# Set rudder to 50% right, sail to 30% out
ros2 topic pub --once /rudder_sail_radio geometry_msgs/msg/Vector3 "{x: 0.5, y: 0.3, z: 0.0}"

# Center controls
ros2 topic pub --once /rudder_sail_radio geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

# Continuous control (press Ctrl+C to stop)
ros2 topic pub -r 10 /rudder_sail_radio geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"
```

#### Keyboard Control (Curses Mode)

When running the simulator bridge directly (not via launch file), keyboard control is available:
- **Arrow Keys**: ←→ for rudder, ↑↓ for sail
- **'c'**: Center controls
- **'q'**: Quit

Note: Keyboard control publishes to `/rudder_sail_radio` topic, so it works with the same control system as Teleop.

## Remote Simulation

Connects to a remote simulator running on another machine.

**Features:**
- Offloads CPU-intensive simulation to remote machine
- Forwards sensor data from remote simulator
- Sends control commands to remote simulator
- Monitors connection status

**Prerequisites:**
1. Remote machine with Argo project cloned
2. SSH access to remote machine
3. ROS2 Humble installed on remote machine

**Setup:**

```bash
# 1. Setup remote machine (one-time)
./scripts/setup_remote_simulator.sh

# 2. Start SSH tunnel (terminal 1)
./scripts/remote_simulator_tunnel.sh

# 3. Start remote simulator (terminal 2)
python3 scripts/remote_simulator_launch.py

# 4. Start local simulation (terminal 3)
python3 launch/argo_lifecycle_manager.py simulate_remote
```

## Unified Simulator Bridge

The `argo_unified_simulator_bridge.py` replaces both `argo_simulator_bridge.py` and `argo_remote_simulator_bridge.py`:

**Local Mode:**
- Runs sailboat-playground or mock simulator
- Publishes simulated sensor data
- Handles control commands locally

**Remote Mode:**
- Connects to remote simulator via ROS2 topics
- Forwards sensor data from remote
- Sends control commands to remote
- Monitors connection health

## Topic Mapping

The simulator bridge provides these topics to the Argo system:

**Published Topics (Simulator → Argo):**
- `/pose` - IMU compass heading (Vector3, z=heading degrees)
- `/compass` - Raw compass data (Vector3, z=heading degrees)
- `/gps_cog` - Course over ground (Float64, degrees)
- `/gps_sog` - Speed over ground (Float64, knots)
- `/gps_velocity` - GPS velocity vector (Vector3, x=north, y=east, z=speed knots)
- `/anem_speed_angle_temp` - Wind data (Vector3, x=speed m/s, y=angle degrees, z=temp °C)
- `/rudder_sail_radio` - Radio control input status (Vector3, x=rudder, y=sail, z=0)

**Subscribed Topics (Argo → Simulator):**
- `/rudder_sail_radio` - **External control input** (Vector3, x=rudder -1 to +1, y=sail -1 to +1, z=0)
  - Can be controlled via Foxglove Teleop panel, terminal commands, or keyboard (when curses enabled)
  - Takes priority over autonomous control when active
  - Automatically enables human control mode
- `/rudder_sail_servo` - Final servo commands from Argo (Vector3, x=rudder, y=sail)
- `/human_controlled` - Control mode status (Bool, true=human, false=robot)

## Configuration

### Centralized Configuration

All remote simulation settings are centralized in `scripts/remote_simulator_config.json`:

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

### Loading Configuration

Both Python and shell scripts load configuration using `scripts/load_config.py`:

```bash
# Shell scripts
eval "$(python3 scripts/load_config.py --export-shell)"

# Python scripts
from load_config import load_config
config = load_config()
```

## Troubleshooting

### Local Simulation Issues

1. **`package 'argo_launch' not found` error:**
   - This error occurred with the old launch system. The new `simulation_launch.py` uses `ExecuteProcess` and should not produce this error. Ensure you have the latest code.

2. **`FileNotFoundError` for `naca0015.csv`:**
   - This was an issue with relative paths. The bridge now calculates and passes an absolute path to the simulator's data files, which has resolved this.

3. **`foxglove_bridge` doesn't start or `Bind Error`:**
   - The `foxglove_bridge` is now included in the main launch file and starts automatically. If you see a `Bind Error`, it means a previous simulation did not shut down cleanly. Use `pkill -f foxglove_bridge` to terminate the old process. The new launch system should prevent this from happening in the future.

4. **sailboat-playground not available:**
   - Check if simulator submodule is initialized: `make submodule-status`
   - Initialize if needed: `make submodule-init`
   - Verify sailboat-playground is in `simulator/sailboat-playground/` directory

### Remote Simulation Issues

1. **Connection timeout:**
   - Check SSH tunnel: `./scripts/remote_simulator_tunnel.sh`
   - Verify remote simulator is running
   - Check network connectivity

2. **ROS2 communication issues:**
   - Ensure both machines use same ROS_DOMAIN_ID
   - Check firewall settings
   - Verify SSH tunnel is active

3. **Remote setup issues:**
   - Run setup script: `./scripts/setup_remote_simulator.sh`
   - Check ROS2 installation on remote machine
   - Verify Python dependencies

### Debug Commands

```bash
# Check the status of running ROS2 nodes
ros2 node list

# Echo a topic to see if data is being published
ros2 topic echo /pose

# Check the definition of the asim alias
alias asim
```

## Benefits

### Local Simulation
- No network dependencies
- Fast startup
- Good for development and testing
- Uses local CPU resources

### Remote Simulation
- Offloads CPU-intensive simulation
- Better performance on resource-constrained Orange Pi
- Scalable (multiple simulators on different machines)
- Isolation (simulator crashes don't affect local system)

## Migration from Old Bridges

The unified simulator bridge replaces the old separate bridges:

**Old approach:**
- `argo_simulator_bridge.py` (local only)
- `argo_remote_simulator_bridge.py` (remote only)

**New approach:**
- `argo_unified_simulator_bridge.py` (both local and remote)

**Migration:**
1. Use `--mode local` instead of `argo_simulator_bridge.py`
2. Use `--mode remote` instead of `argo_remote_simulator_bridge.py`
3. Update launch scripts to use unified bridge
4. Remove old bridge files (optional)

This unified approach reduces code duplication and provides a consistent interface for both simulation modes.

## Bag File Playback with Visualization

When playing back recorded bag files in Foxglove, you need additional nodes running to properly visualize the data:

### Required for Visualization

The visualization markers (`/visualization_marker_array`) are recorded in the bag, but they need:
1. **Transform publisher** (`argo_transform_publisher.py`) - Provides `/tf` transforms for base_link → map coordinate frames
2. **Visualization node** (`argo_boat_visualization.py`) - Recreates markers from source topics (optional, if markers weren't recorded)
3. **Sailing area publisher** (`sailing_area_publisher.py`) - Provides boundaries/waypoints/hazards (optional)

### Usage

Use the dedicated bag playback launch file:

```bash
# Play back a bag file with full visualization support
ros2 launch launch/argo_bag_playback.py bag_file:=~/argo/bags/argo_20251105_141014\ first\ dry\ sail

# With custom options
ros2 launch launch/argo_bag_playback.py \
    bag_file:=~/argo/bags/your_bag_file \
    use_foxglove:=true \
    use_sailing_area:=true \
    use_visualization:=true \
    use_transform:=true
```

### What Gets Launched

The launch file automatically starts:
1. **Bag playback** (`ros2 bag play`) - Republishes all recorded topics
2. **Transform publisher** - Publishes `/tf` transforms from GPS/pose data in the bag
3. **Visualization node** - Subscribes to replayed topics and republishes visualization markers
4. **Sailing area publisher** - Provides map boundaries/waypoints (if available)
5. **Foxglove bridge** - Connects to Foxglove Studio at `ws://localhost:8765`

### Connecting Foxglove

1. Start the playback launch file
2. Open Foxglove Studio
3. Connect to `ws://localhost:8765`
4. The visualization markers should appear in the 3D panel

### Troubleshooting

**Visualization markers not showing:**
- Check that `/tf` transforms are being published: `ros2 topic echo /tf`
- Verify visualization node is running: `ros2 node list | grep visualization`
- Check that source topics are in the bag: `ros2 bag info /path/to/bag`

**Transforms missing:**
- The transform publisher needs `/fix` (GPS) and `/pose` (heading) topics from the bag
- If these topics weren't recorded, transforms won't be available
- Check bag contents: `ros2 bag info /path/to/bag`

**Sailing area not showing:**
- The sailing_area_publisher needs map configuration
- Check if map data is available: `ros2 topic echo /sailing_boundaries`
- If not needed, disable with `use_sailing_area:=false`
