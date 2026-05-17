# Argo Two-Node Control System Usage Guide

This guide shows how to use the new two-node control system that separates high-level autonomous control from low-level hardware control and human/robot arbitration.

## Quick Start

### 1. Launch the System
```bash
# Terminal 1: Low-level control and arbitration
cd ~/argo
python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo.yaml

# Terminal 2: High-level autonomous control
cd ~/argo  
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml

# Terminal 3: Monitor system status (optional)
cd ~/argo
ros2 topic echo /human_controlled
```

### 2. Monitor System Status
```bash
# Watch control authority (who has control)
ros2 topic echo /human_controlled

# Watch autonomous commands
ros2 topic echo /rudder_sail_cmd

# Watch final hardware commands  
ros2 topic echo /rudder_sail_servo

# Watch detailed control status
ros2 topic echo /control_authority
```

## Key Features

### 🚨 Human Priority Control
- **Any radio input immediately grants human control**
- No button presses or mode switches required
- Move rudder or sail stick > 0.05 units and you have control
- Robot waits for 2 seconds of inactivity before taking back control

### 🛡️ Safety Features
- All commands clamped to safe limits (±1.0 by default)
- Human override works even if autonomous system fails
- 20Hz control loop for responsive arbitration
- Separate nodes mean hardware control continues if high-level control fails

### 🔄 Seamless Transitions
- Target heading automatically updated during human control
- Smooth handover between human and robot control
- No jerky transitions or mode switches

## Control Flow Examples

### Scenario 1: Normal Autonomous Operation
```
1. Both nodes start up
2. Human moves radio → Human gets immediate control
3. Human stops moving radio for 2+ seconds
4. Robot takes control with last human heading as target
5. Robot maintains heading using autonomous control
```

### Scenario 2: Human Override
```
1. Robot is controlling boat autonomously
2. Human moves radio stick (any amount > 0.05)
3. IMMEDIATELY: Human has control, robot commands ignored
4. Human controls boat directly
5. Human stops → Robot takes back control after 2 seconds
```

### Scenario 3: Emergency Takeover
```
1. Robot control active
2. Emergency situation detected by human
3. Human grabs radio and moves any control
4. INSTANT human override - no delays, no mode switches
5. Human has full control immediately
```

## Configuration Options

### Basic Configuration (argo.yaml)
```yaml
# High-level controller
controller_node:
  ros__parameters:
    controller_type: "proportional"     # or "wind_aware"
    rudder_gain: 2.0                   # Heading control sensitivity
    rudder_full_scale_deg: 60.0        # Full rudder at 60° heading error

# Low-level arbitration
rudder_sail_radio_node:
  ros__parameters:
    human_override_timeout: 2.0        # Seconds before robot can take control
    deadband_threshold: 0.40           # Minimum radio change to detect human (see argo.yaml)
    safety_max_rudder: 1.0            # Maximum rudder command
    safety_max_sail: 1.0              # Maximum sail command
```

### Conservative Human Priority
```yaml
rudder_sail_radio_node:
  ros__parameters:
    human_override_timeout: 5.0        # Longer timeout (5 seconds)
    deadband_threshold: 0.30           # More sensitive (lower = easier false triggers from RC noise)
    safety_max_rudder: 0.8            # More conservative limits
    safety_max_sail: 0.8
```

### Wind-Aware Autonomous Control
```yaml
controller_node:
  ros__parameters:
    controller_type: "wind_aware"      # Use wind-based sail control
    rudder_gain: 1.5                   # Slightly less aggressive
    sail_wind_gain: 0.3               # 30% wind-based, 70% radio-based sail
```

## Monitoring and Debugging

### Check System Health
```bash
# Are both nodes running?
ros2 node list | grep -E "(controller|rudder_sail_radio)"

# Is sensor data flowing?
ros2 topic hz /pose /compass /anem_speed_angle_temp

# Is radio input working?
ros2 topic echo /rudder_sail_radio

# Are autonomous commands being generated?
ros2 topic echo /rudder_sail_cmd
```

### Debug Control Authority Issues
```bash
# Check detailed control status
ros2 topic echo /control_authority
# x: 1=human control, 0=robot control  
# y: seconds since last human activity
# z: seconds since last autonomous command

# Check for radio input changes
ros2 topic echo /rudder_sail_radio
# Look for changes > 0.05 in x (rudder) or y (sail)
```

### Monitor Control Performance
```bash
# Compare autonomous vs final commands
ros2 topic echo /rudder_sail_cmd     # What controller.py wants
ros2 topic echo /rudder_sail_servo   # What actually goes to hardware

# Should be identical during robot control
# Should be different during human control
```

## Troubleshooting

### Problem: Robot never takes control
```bash
# Check if radio input is constantly changing (noise)
ros2 topic echo /rudder_sail_radio

# Solution: Increase deadband_threshold or check radio calibration
```

### Problem: Human can't override robot
```bash
# Check if radio input is reaching the system
ros2 topic echo /rudder_sail_radio

# Check if changes are above deadband threshold
# Solution: Lower deadband_threshold or fix radio hardware
```

### Problem: Jerky transitions
```bash
# Check human_override_timeout setting
# Solution: Tune timeout value (try 1.0-5.0 seconds)
```

### Problem: Autonomous control not working
```bash
# Check if controller.py is receiving sensor data
ros2 topic echo /pose

# Check if autonomous commands are being generated
ros2 topic echo /rudder_sail_cmd

# Check controller configuration
ros2 param get /controller_node controller_type
```

## Data Collection

### Enable Training Data Collection
```yaml
controller_node:
  ros__parameters:
    data_collection_enabled: true
```

### Collect Training Data
1. Enable data collection in config
2. Restart controller.py
3. Operate boat in human control mode
4. Data automatically saved to `training_data/` directory
5. Use existing analysis tools from control_refactored.py

## Migration from Original Control

### From control.py
- Replace `python3 nodes/control.py` with the two-node system
- Same sensor inputs and control logic
- Enhanced robustness and human priority

### From control_refactored.py  
- All controller types work the same way
- Same data collection features
- Same parameter hot-reloading
- Added robust human override and hardware arbitration

## System Architecture Summary

```
Sensors → controller.py → /rudder_sail_cmd → rudder_sail_radio.py → Hardware
    ↓                                              ↑
    └─ Target heading management              Radio input
                                              Human override
                                              Safety limits
```

The key innovation is the **human priority arbitration** in `rudder_sail_radio.py` that ensures humans can always immediately take control without any mode switches or button presses - just move the radio controls and you have instant authority over the boat.
