# Argo Two-Node Control Architecture

This document describes the two-node control system that separates high-level autonomous control from low-level hardware control and human/robot arbitration.

## Architecture Overview

```
┌─────────────────────┐    /rudder_sail_cmd     ┌─────────────────────────┐
│   controller.py     │ ────────────────────► │  rudder_sail_radio.py   │
│ (High-level control)│                       │ (Low-level arbitration) │
│                     │ ◄──────────────────── │                         │
│ - Sensor processing │   /human_controlled    │ - Human priority        │
│ - Path planning     │   /control_authority   │ - Hardware interface    │
│ - Control algorithms│                       │ - Safety limits         │
└─────────────────────┘                       └─────────────────────────┘
         ▲                                              │
         │                                              ▼
    ┌────────────────┐                           ┌─────────────────┐
    │ Sensor Topics  │                           │ Hardware Topics │
    │ /pose, /gps_*, │                           │ /rudder_sail_   │
    │ /compass, /wind│                           │ _radio, _servo  │
    └────────────────┘                           └─────────────────┘
```

## Node Responsibilities

### controller.py (High-Level Control)
- **Purpose**: Autonomous navigation and control logic
- **Inputs**: All sensor data (GPS, IMU, wind, compass)
- **Outputs**: Autonomous control commands 
- **Features**:
  - Modular controller architecture (proportional, wind-aware, etc.)
  - Data collection during human control for training
  - Hot-swappable algorithms via parameter changes
  - Target heading management

### rudder_sail_radio.py (Low-Level Control)
- **Purpose**: Hardware interface and human/robot arbitration
- **Inputs**: Radio commands from hardware, autonomous commands from controller.py
- **Outputs**: Final servo commands to hardware, control status
- **Features**:
  - **Human priority**: Any radio activity immediately grants human control
  - Timeout-based handover to robot control
  - Safety limits on all commands
  - High-frequency control loop (20Hz) for responsive arbitration

## Topic Interface

### From controller.py to rudder_sail_radio.py
```yaml
/rudder_sail_cmd:
  type: geometry_msgs/Vector3
  description: Autonomous control commands
  format:
    x: rudder command (-1:+1, left:right)
    y: sail command (-1:+1, in:out) 
    z: reserved (0.0)
```

### From rudder_sail_radio.py to controller.py
```yaml
/human_controlled:
  type: std_msgs/Bool
  description: Current control authority status
  data: true if human has control, false if robot

/control_authority:
  type: geometry_msgs/Vector3  
  description: Detailed control status
  format:
    x: current authority (1=human, 0=robot)
    y: time since last human activity (seconds)
    z: time since last auto command (seconds)
```

### Hardware Interface (rudder_sail_radio.py only)
```yaml
/rudder_sail_radio:    # Input from PWM hardware
  type: geometry_msgs/Vector3
  description: Radio control input
  
/rudder_sail_servo:    # Output to PWM hardware  
  type: geometry_msgs/Vector3
  description: Final servo commands
```

### Sensor Topics (controller.py subscribers)
- `/pose` - IMU data with compass heading in z-component
- `/gps_cog`, `/gps_sog`, `/gps_velocity` - GPS navigation data
- `/accel`, `/gyro`, `/compass` - IMU sensor data
- `/anem_speed_angle_temp` - Wind sensor data

## Control Flow

### 1. Normal Operation
1. **Sensor data** flows to `controller.py`
2. **controller.py** processes sensors and generates autonomous commands
3. **Autonomous commands** sent to `rudder_sail_radio.py` via `/rudder_sail_cmd`
4. **rudder_sail_radio.py** arbitrates between human and robot control
5. **Final commands** sent to hardware via `/rudder_sail_servo`

### 2. Human Override (Key Robustness Feature)
1. **Human moves radio controls** (detected by change > deadband_threshold)
2. **rudder_sail_radio.py** immediately switches to human control
3. **Human commands** pass through directly to hardware
4. **controller.py** receives human control status and stops publishing commands
5. **Target heading** continuously updated from current heading during human control

### 3. Return to Autonomous Control
1. **No human activity** for `human_override_timeout` seconds
2. **rudder_sail_radio.py** switches to robot control authority
3. **controller.py** resumes publishing autonomous commands
4. **Target heading** maintained from when human last had control

## Robustness Features

### Human Priority (in rudder_sail_radio.py)
```python
# Any radio input change above deadband immediately grants human control
if rudder_change > self.deadband_threshold or sail_change > self.deadband_threshold:
    self.last_human_activity = time.time()
    # Human control activated immediately
```

### Safety Limits
```python
# All commands clamped to safe ranges before hardware output
cmd_rudder = np.clip(cmd_rudder, -self.safety_max_rudder, self.safety_max_rudder)
cmd_sail = np.clip(cmd_sail, -self.safety_max_sail, self.safety_max_sail)
```

### Timeout-Based Handover
```python
# Smooth transition based on activity timing
time_since_human_activity = current_time - self.last_human_activity
human_control = time_since_human_activity < self.human_override_timeout
```

## Configuration

### Basic Setup
```yaml
# argo_two_node.yaml
controller_node:
  ros__parameters:
    controller_type: "proportional"
    rudder_gain: 2.0
    rudder_full_scale_deg: 60.0

rudder_sail_radio_node:
  ros__parameters:
    human_override_timeout: 2.0
    deadband_threshold: 0.05
    safety_max_rudder: 1.0
    safety_max_sail: 1.0
```

### Conservative Human Priority
```yaml
rudder_sail_radio_node:
  ros__parameters:
    human_override_timeout: 5.0      # Longer timeout
    deadband_threshold: 0.02         # More sensitive
    safety_max_rudder: 0.8          # Conservative limits
```

## Launching the System

### Start Both Nodes
```bash
# Terminal 1: Low-level control
python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo_two_node.yaml

# Terminal 2: High-level control  
python3 nodes/controller.py --ros-args --params-file nodes/argo_two_node.yaml
```

### Monitor Control Status
```bash
# Watch control authority
ros2 topic echo /human_controlled

# Watch detailed status
ros2 topic echo /control_authority

# Monitor commands
ros2 topic echo /rudder_sail_cmd    # Autonomous commands
ros2 topic echo /rudder_sail_servo  # Final hardware commands
```

## Benefits of Two-Node Architecture

1. **Separation of Concerns**: High-level planning separate from low-level control
2. **Human Priority**: Robust human override with immediate response
3. **Safety**: Multiple layers of safety limits and arbitration
4. **Modularity**: Easy to modify either node independently
5. **Testability**: Can test controller.py without hardware
6. **Fault Tolerance**: If controller.py fails, human control still works
7. **Real-time**: Low-level arbitration runs at 20Hz for responsiveness

## Migration from Single Node

The two-node system maintains the same sensor inputs and control logic as the original refactored control, but adds:

- **Robust human priority** with immediate override capability
- **Hardware abstraction** in the low-level node
- **Safety limits** applied at the hardware interface
- **Detailed control status** for monitoring and debugging

This architecture ensures that human control always takes priority while maintaining the sophisticated autonomous control capabilities of the original system.



