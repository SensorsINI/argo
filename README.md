# Argo Power Control System - ROS2 Node

This package provides a ROS2 node for the Argo power control system, which manages power button functionality, LED indicators, and system health monitoring.

## Features

- **Power Button Control**: Monitors power button presses and initiates graceful shutdown on long press
- **LED Status Indicators**: 
  - Green LED: Heartbeat at 1Hz (normal) or 2Hz (during bagfile recording)
  - Blue LED: SOS pattern when argo nodes are down
- **Node Health Monitoring**: Monitors other argo nodes and triggers SOS pattern if any critical nodes die
- **Bagfile Recording Detection**: Increases green LED frequency to 2Hz during recording
- **ROS2 Integration**: Full ROS2 node with topics, services, and diagnostics

## ROS2 Topics

### Subscribed Topics
- `/argo/recording/bagfile_status` (std_msgs/Bool): Bagfile recording status

### Published Topics
- `/argo/power_control/led_status` (std_msgs/String): LED status information
- `/argo/power_control/node_health` (diagnostic_msgs/DiagnosticArray): Node health diagnostics

### Services
- `/argo/power_control/set_led` (std_srvs/SetBool): Manual LED control

## Usage

### Running the Node

```bash
# Normal operation
ros2 run argo_power_control argo_power_control

# Test mode (safe for testing)
ros2 run argo_power_control argo_power_control --test-mode

# With custom threshold
ros2 run argo_power_control argo_power_control --threshold 3.0
```

### Using the Launch File

```bash
# Normal launch
ros2 launch argo_power_control argo_power_control_launch.py

# Test mode launch
ros2 launch argo_power_control argo_power_control_launch.py test_mode:=true

# Custom threshold launch
ros2 launch argo_power_control argo_power_control_launch.py threshold:=3.0
```

### Testing LED Control

```bash
# Turn blue LED on
ros2 service call /argo/power_control/set_led std_srvs/srv/SetBool "{data: true}"

# Turn blue LED off
ros2 service call /argo/power_control/set_led std_srvs/srv/SetBool "{data: false}"
```

### Testing Bagfile Recording Detection

```bash
# Simulate bagfile recording start
ros2 topic pub /argo/recording/bagfile_status std_msgs/msg/Bool "{data: true}"

# Simulate bagfile recording stop
ros2 topic pub /argo/recording/bagfile_status std_msgs/msg/Bool "{data: false}"
```

## Hardware Configuration

- **PI3 (Pin 40)**: !POW - Open drain output to control power relay
- **PI9 (Pin 28)**: !POW_BUT - Input from power button (external pullup required)
- **PH4 (Pin 18)**: Green LED in power button (system running indicator)
- **PI1 (Pin 12)**: Blue LED in power button (status/warning indicator)
- **Red LED**: Directly connected to power button (not GPIO controlled)

## LED Patterns

### Green LED (Heartbeat)
- **Normal Operation**: 1Hz heartbeat
- **Bagfile Recording**: 2Hz heartbeat
- **Button Press**: Gradual frequency increase from 2Hz to 20Hz
- **Shutdown**: 1Hz with 5% duty cycle

### Blue LED (Status/Warning)
- **Normal Operation**: Off
- **Button Press**: Same pattern as green LED
- **SOS Pattern**: Morse code SOS (··· --- ···) when argo nodes are down
- **Shutdown**: Same pattern as green LED

### Red LED
- **Note**: Red LED is directly connected to power button and not GPIO controlled
- **Behavior**: Controlled by hardware power button circuit

## Node Health Monitoring

The node monitors the following argo nodes:
- `argo_navigation`
- `argo_sensors`
- `argo_actuators`
- `argo_communication`

If any of these nodes are detected as missing, the blue LED will flash an SOS pattern until all nodes are healthy again.

## Building the Package

```bash
cd /home/orangepi/argo
colcon build --packages-select argo_power_control
# Note: Pure Python ROS2 packages can be run directly without sourcing install/
```

## Requirements

- Python 3
- ROS2 (Humble or later)
- python3-gpiod library
- User must be member of 'gpio' group
- External pullup resistor on power button input
- Proper hardware connections as described above

## Safety Features

- Open drain configuration prevents damage from multiple control sources
- Graceful shutdown ensures proper system shutdown before cutting power
- GPIO pins automatically revert to input state on halt, de-energizing relay
- External pullup resistor requirement prevents floating inputs
- GPIO group membership requirement for controlled GPIO access