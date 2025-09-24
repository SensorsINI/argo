# Argo Sailing Simulator Setup

## ✅ **Installation Complete**

The sailing simulator and bridge have been successfully installed and tested:

- **✅ sailboat-playground** installed via pip
- **✅ argo_simulator_bridge.py** created and tested
- **✅ Mock simulator** working reliably for headless operation
- **✅ ROS2 topics** publishing at 10Hz

## 🚀 **Quick Start Guide**

### 1. **Start the Complete System**

```bash
# Terminal 1: Simulator (provides sensor data)
cd /home/orangepi/argo
python3 nodes/argo_simulator_bridge.py

# Terminal 2: Low-level control (human/robot arbitration)
python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo.yaml

# Terminal 3: High-level control (autonomous algorithms)
python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml
```

### 2. **Monitor the System**

```bash
# Watch sensor data flowing from simulator
ros2 topic echo /pose          # Compass heading
ros2 topic echo /gps_sog       # Speed over ground
ros2 topic echo /anem_speed_angle_temp  # Wind data

# Watch control commands
ros2 topic echo /rudder_sail_cmd    # Autonomous commands
ros2 topic echo /rudder_sail_servo  # Final hardware commands

# Watch control authority
ros2 topic echo /human_controlled   # Who has control
```

## 📊 **What the Simulator Provides**

### **Realistic Sailing Physics**
- **Wind-based speed**: Boat speed depends on wind angle and sail position
- **Rudder steering**: Boat turns based on rudder position and speed
- **Apparent wind**: Wind direction relative to boat heading
- **Sailing efficiency**: Better performance when sail is properly trimmed

### **Complete Sensor Data**
- **GPS**: Course over ground, speed over ground, velocity vector
- **Compass/IMU**: Heading in degrees (0-360°)
- **Wind**: Speed (m/s), direction relative to boat, temperature
- **Mock radio**: Simulated human input for testing

### **Test Scenarios**
The simulator automatically generates:
- **Variable wind conditions**: 6-10 m/s wind speed
- **Changing wind direction**: Simulates realistic conditions
- **Mock human input**: For testing control arbitration

## 🎮 **Testing Control Algorithms**

### **Test Human Priority**
1. Watch the simulator generate mock radio input
2. See `rudder_sail_radio.py` grant human control authority
3. Observe human commands pass through to final output
4. After 2 seconds of no input, robot takes control

### **Test Autonomous Control**
1. Let the system settle into robot control mode
2. Watch `controller.py` generate autonomous commands
3. See the boat respond to heading control
4. Modify target heading by editing parameters

### **Test Controller Switching**
```bash
# Switch to wind-aware controller
# Edit nodes/argo.yaml and change:
# controller_type: "wind_aware"

# The system will automatically reload and switch controllers
```

## 📈 **Monitoring Performance**

### **Simulator Status**
The bridge logs show:
```
Boat: heading=45.2°, speed=1.7m/s, wind=45°, mode=HUMAN, last_cmd=2.1s ago
```

- **heading**: Current boat heading (0-360°)
- **speed**: Boat speed in m/s
- **wind**: Wind direction relative to boat (degrees)
- **mode**: HUMAN or ROBOT control authority
- **last_cmd**: Time since last control command received

### **Topic Data Rates**
All topics publish at **10Hz** for smooth control:
- `/pose` - Compass heading
- `/gps_*` - GPS navigation data  
- `/anem_speed_angle_temp` - Wind sensor data
- `/rudder_sail_radio` - Mock human input

## 🔧 **Advanced Testing**

### **Data Collection Mode**
```yaml
# Enable in nodes/argo.yaml
controller_node:
  ros__parameters:
    data_collection_enabled: true
```

This will collect training data during human control sessions.

### **Custom Wind Conditions**
Edit the simulator parameters in `argo_simulator_bridge.py`:
```python
self.wind_speed = 12.0     # Stronger wind
self.wind_direction = 90.0  # Wind from starboard
```

### **Sailing Performance Analysis**
```bash
# Monitor boat performance
ros2 topic echo /gps_velocity  # North/East velocity components
ros2 topic echo /gps_sog       # Speed over ground in knots

# Analyze control effectiveness
ros2 topic echo /rudder_sail_cmd    # What controller wants
ros2 topic echo /rudder_sail_servo  # What actually happens
```

## 🛟 **Troubleshooting**

### **"Cannot connect to display"**
- ✅ **Fixed**: Simulator uses headless mode automatically
- Mock simulator works reliably without graphics

### **No sensor data flowing**
```bash
# Check if bridge is running
ros2 node list | grep simulator

# Check topic data
ros2 topic list | grep -E "(pose|gps|anem)"
```

### **Control not working**
```bash
# Check if all three nodes are running
ros2 node list | grep -E "(controller|rudder_sail|simulator)"

# Verify control flow
ros2 topic echo /rudder_sail_cmd     # Should show autonomous commands
ros2 topic echo /rudder_sail_servo   # Should show final commands
```

## 🎯 **Next Steps**

1. **Test your control algorithms** with the simulator
2. **Collect training data** during simulated human control
3. **Tune control parameters** using the safe simulation environment
4. **Validate before hardware testing** on the real boat

The simulator provides a safe, repeatable environment for developing and testing your autonomous sailing algorithms before deploying to the real Argo sailboat!





