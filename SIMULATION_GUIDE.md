# Argo Simulation Guide

This guide explains how to run Argo in simulation mode, both locally and remotely, avoiding conflicts with hardware nodes.

## Overview

When running simulation, several hardware nodes conflict with the simulator topics:

**Conflicting Hardware Nodes (excluded in simulation):**
- `gps.py` → publishes `/gps_cog`, `/gps_sog`, `/gps_velocity` (conflicts with simulator)
- `imu.py` → publishes `/compass` (conflicts with simulator) 
- `anem.py` → publishes `/anem_speed_angle_temp` (conflicts with simulator)
- `rudder_sail_radio.py` → subscribes to `/rudder_sail_servo` (conflicts with simulator control)

**Nodes that run in simulation:**
- `argo_unified_simulator_bridge.py` → provides simulated sensor data
- `controller.py` → autonomous navigation
- `battery_water.py` → hardware monitoring (no conflicts)
- `temp_monitor.py` → hardware monitoring (no conflicts)

## Simulation Modes

### 1. Local Simulation

Runs the simulator directly on the Orange Pi.

**Features:**
- Uses sailboat-playground or mock simulator
- Provides simulated sensor data
- Handles control commands locally
- No network dependencies

**Usage:**

```bash
# Method 1: Using lifecycle manager
python3 launch/argo_lifecycle_manager.py simulate

# Method 2: Using launch script
./scripts/launch_simulator_local.sh

# Method 3: Manual launch
python3 nodes/argo_unified_simulator_bridge.py --mode local &
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml &
python3 nodes/battery_water.py &
python3 nodes/temp_monitor.py &
```

### 2. Remote Simulation

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
./scripts/launch_simulator_remote.sh
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
- `/rudder_sail_radio` - Mock human input (Vector3, x=rudder, y=sail, z=0)

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

1. **sailboat-playground not available:**
   - Check if simulator submodule is initialized: `git submodule update --init`
   - Verify sailboat-playground is in `simulator/` directory

2. **Mock simulator fallback:**
   - This is normal if sailboat-playground fails to load
   - Mock simulator provides basic physics simulation

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
# Check node status
python3 launch/argo_lifecycle_manager.py status

# Monitor system
python3 launch/argo_lifecycle_manager.py monitor

# Debug remote connection
./scripts/debug_remote_ros2.sh

# Test simulator bridge
python3 nodes/argo_unified_simulator_bridge.py --mode local --help
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

