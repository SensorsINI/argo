# Argo Simulation Guide

This guide explains how to run the Argo in local simulation mode.

## Overview

When running simulation, several hardware nodes conflict with the simulator topics and are automatically excluded.

**Conflicting Hardware Nodes (excluded in simulation):**
- `gps.py` → publishes `/gps_cog`, `/gps_sog`, `/gps_velocity`
- `bno085.py` → publishes `/compass`
- `anem.py` → publishes `/anem_speed_angle_temp`
- `rudder_sail_radio.py` → subscribes to `/rudder_sail_servo`

**Nodes launched in simulation:**
- `argo_unified_simulator_bridge.py` → provides simulated sensor data (uses mock simulator by default)
- `controller.py` → handles autonomous navigation
- `sailing_area_publisher.py` → provides map boundaries for visualization
- `foxglove_bridge` → provides the websocket for Foxglove Studio

## Quick Start

### Step 1: Start Simulation

In your first terminal, launch the simulation with the mock simulator (the physical simulator is not yet stable):

```bash
asim --mock
```

Or simply:

```bash
asim
```

(The `--mock` flag is the default, so it can be omitted. Use `--real` if you want to try the physical simulator, but it's not recommended yet.)

This command will:
1. Launch the simulator bridge, controller, and sailing area publisher
2. Start the `foxglove_bridge` automatically
3. Ensure all processes are managed correctly and shut down cleanly with `Ctrl+C`

### Step 2: Start Keyboard Control (Optional)

In a second terminal, launch the keyboard control interface:

```bash
asimkb
```

Or directly:

```bash
python3 nodes/argo_keyboard_control.py
```

**Keyboard Controls:**
- **← →** : Rudder control (left/right)
- **↑ ↓** : Sail control (out/in)
- **c** : Center both controls
- **w** : Rotate wind +10°
- **e** : Rotate wind -10°
- **SPACE** : Toggle simulation pause (freezes simulation, keeps markers visible)
- **r** : Reset simulation
- **h** : Toggle Return-to-Home controller
- **m** : Toggle controller pause (manual/autonomous)
- **q** : Quit simulation
- **x** : Quit keyboard control (keep simulation running)
- **ENTER** : Refresh display

The keyboard control node publishes to `/rudder_sail_radio` topic, which takes priority over autonomous control when active.

### Step 3: Visualize in Foxglove Studio

1. Open Foxglove Studio
2. Connect to: `ws://localhost:8765`
3. The visualization should show:
   - Boat position and heading
   - Sailing area boundaries
   - Wind direction
   - Control commands
   - Sensor data

## Configuration

All simulation parameters are centralized in `nodes/argo.yaml`. This file contains:

- **Simulation parameters**: Rates, grounding behavior, sensor noise models
- **Mock simulator physics**: Turn rates, speeds, tack parameters
- **Wind characteristics**: Speed, direction, variability
- **Controller parameters**: Heading gains, boundary thresholds
- **Map configuration**: Geofence and sailing area settings

### Key Parameters

The most commonly adjusted parameters are in the `simulation` section:

```yaml
simulation:
  publish_rate: 2.0         # Sensor publishing rate (Hz)
  simulation_rate: 10.0     # Physics simulation rate (Hz)
  grounding_behavior: reset # Behavior on grounding: reset|terminate|continue
  wind:
    wind_direction: 300.0   # Wind direction in degrees (compass, 0°=North)
    wind_min_speed: 5.0     # Minimum wind speed (m/s)
    wind_max_speed: 10.0    # Maximum wind speed (m/s)
```

To modify parameters:
1. Edit `nodes/argo.yaml`
2. Restart the simulation (the nodes will reload parameters automatically)

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
- `/rudder_sail_radio` - External control input (Vector3, x=rudder -1 to +1, y=sail -1 to +1, z=0)
  - Published by keyboard control node (`asimkb`)
  - Takes priority over autonomous control when active
  - Automatically enables human control mode
- `/rudder_sail_servo` - Final servo commands from Argo (Vector3, x=rudder, y=sail)
- `/human_controlled` - Control mode status (Bool, true=human, false=robot)

## Troubleshooting

### Simulation won't start

1. **Check ROS2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Verify the alias is set:**
   ```bash
   alias asim
   ```
   If not set, install Argo CLI: `make install-argo-cli`

3. **Check for running processes:**
   ```bash
   ros2 node list
   ```
   If old nodes are running, stop them first

### Foxglove connection issues

1. **`foxglove_bridge` doesn't start or `Bind Error`:**
   - A previous simulation may not have shut down cleanly
   - Kill the old process: `pkill -f foxglove_bridge`
   - Restart the simulation

2. **Can't connect to `ws://localhost:8765`:**
   - Verify `foxglove_bridge` is running: `ros2 node list | grep foxglove`
   - Check if port is in use: `lsof -i :8765`
   - Restart the simulation

### Keyboard control not working

1. **Node not found:**
   ```bash
   ros2 node list | grep keyboard
   ```
   If not present, check that `asimkb` alias is set or run directly:
   ```bash
   python3 nodes/argo_keyboard_control.py
   ```

2. **No response to key presses:**
   - Check that keyboard control is publishing: `ros2 topic echo /rudder_sail_radio`
   - Verify the simulation is running and subscribed to the topic
   - Try refreshing the display with ENTER key

### Mock simulator issues

1. **Simulation seems unrealistic:**
   - The mock simulator is simplified compared to the physical simulator
   - Adjust parameters in `nodes/argo.yaml` under `simulation.mock_simulator`
   - Consider using `--real` flag (experimental, may be unstable)

2. **Boat doesn't respond to controls:**
   - Check control mode: `ros2 topic echo /human_controlled`
   - Verify keyboard control is publishing: `ros2 topic echo /rudder_sail_radio`
   - Check controller status: `ros2 topic echo /controller_pause_state`

### Debug Commands

```bash
# Check running ROS2 nodes
ros2 node list

# Monitor a specific topic
ros2 topic echo /pose
ros2 topic echo /rudder_sail_radio
ros2 topic echo /human_controlled

# Check topic rates
ros2 topic hz /pose
ros2 topic hz /gps_velocity

# View simulation parameters
ros2 param list /argo_unified_simulator_bridge
ros2 param get /argo_unified_simulator_bridge simulation.simulation_rate
```

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
