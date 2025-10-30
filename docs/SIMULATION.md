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
- `/rudder_sail_radio` - Keyboard control input (Vector3, x=rudder, y=sail, z=0)

**Subscribed Topics (Argo → Simulator):**
- `/rudder_sail_servo` - Control commands from Argo (Vector3, x=rudder, y=sail)
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
