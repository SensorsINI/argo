# Scripts for Argo Simulation

This directory contains scripts for setting up, launching, and debugging the Argo simulation environment in both local and remote modes.

## Bag recording and playback

### `argo_rerecord_bag.sh`
Interactive wrapper to **re-record** an existing rosbag2 with visualization nodes (markers, transforms, sailing area). It invokes `ros2 launch` on **`argo_bag_rerecord.py`** in this directory.

### `argo_bag_rerecord.py`
ROS 2 **launch** description (Python): play an input bag with sim time, run `argo_boat_visualization`, `argo_transform_publisher`, and related nodes, and record a new bag. Invoke with `ros2 launch ./scripts/argo_bag_rerecord.py ...` from the repo root (or pass an absolute path to this file).

By default (`exclude_recorded_markers:=true`), marker topics that those live nodes re-publish (`/visualization_marker*`, `/sailing_*` marker arrays, etc.) are **not** replayed from the input bag, so the output recording does not contain duplicate marker streams. Set `exclude_recorded_markers:=false` to replay the bag verbatim (previous behavior).

**Time trim:** pass both `trim_start` and `trim_end` to copy only messages whose **bag log timestamps** fall in that window into a temporary bag before playback. Use ISO-8601 (e.g. `2026-05-08T16:32:33.466+02:00`), or wall clock with `trim_timezone` (default `Europe/Zurich`), e.g. `trim_start:="2026-05-08 4:32:33.466 PM"` (suffix `CEST` is stripped). Alternatively pass two **numbers** as seconds from the bag start (per `metadata.yaml`). The temp bag is deleted on shutdown.

### `fix_rosbag_pose_legacy_compass.py`
Offline bag copy that rewrites legacy **`/pose.z`** (compass duplicated on pose) to ENU math yaw. Does not regenerate markers or `/tf`.

## Simulation Launch Scripts

### `launch_simulator_local.sh`
Starts the complete Argo simulation system on the local machine (Orange Pi). It launches the `argo_unified_simulator_bridge.py` in `local` mode, along with other necessary nodes like the controller.

### `launch_simulator_remote.sh`
Starts the local components required for a remote simulation. It runs the `argo_unified_simulator_bridge.py` in `remote` mode, which acts as a bridge to forward data between the local control system and the remote simulator.

## Remote Simulation Management

These scripts are used for managing the remote simulation environment.

### `remote_simulator_launch.py`
Connects to the configured remote machine via SSH and starts the `argo_unified_simulator_bridge.py` in `local` mode on that machine. This script is responsible for running the actual simulation process on the remote hardware.

### `remote_simulator_tunnel.sh`
Establishes a secure SSH tunnel between the local machine and the remote simulator. This tunnel forwards the necessary ROS2 communication ports, allowing the local and remote nodes to discover and communicate with each other.

### `setup_remote_simulator.sh`
A one-time setup script to configure a new remote machine for running the Argo simulator. It installs ROS2, Python dependencies, and verifies that the Argo project directory exists.

## Configuration

### `remote_simulator_config.json`
A JSON file containing all the configuration parameters for remote simulation, such as the remote host's address, user, and SSH settings.

### `load_config.py`
A helper script that loads the settings from `remote_simulator_config.json` and makes them accessible to both Python and shell scripts. It can export the configuration as environment variables for shell scripts.

## Debugging

### `debug_remote_ros2.sh`
A utility script for troubleshooting ROS2 issues on the remote machine. It checks the ROS2 installation, verifies Python dependencies, and tests the connection to ensure the remote environment is set up correctly.

