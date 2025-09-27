# Argo Sailing Simulator Setup

## 📦 **Installation Requirements**

Before running the simulator, install the required dependencies:

### **Install sailboat-playground**

The `sailboat-playground` is a simple framework for sailboat simulation and autonomous navigation algorithms development created by Gabriel Gazola Milan at UFRJ (Federal University of Rio de Janeiro).

```bash
# Install the sailing physics simulation library
pip install sailboat-playground

# Verify installation
pip list | grep sailboat-playground

# Test import (should show available module contents)
python3 -c "import sailboat_playground; print('✅ sailboat-playground imported successfully')"
```

#### **About sailboat-playground**
- **Purpose**: Framework for sailboat simulation and autonomous navigation algorithm development
- **Author**: Gabriel Gazola Milan (UFRJ - Federal University of Rio de Janeiro) 
- **License**: GPL-3.0
- **Dependencies**: cython, numpy, pandas, pyglet
- **GitHub**: https://github.com/gabriel-milan/sailboat-playground

#### **Troubleshooting sailboat-playground Import Issues**

If you see warnings about sailboat-playground not being available:

1. **Check Python Environment Consistency**:
   ```bash
   # Ensure pip and python are from the same environment
   which python3
   which pip3
   python3 -m pip list | grep sailboat
   ```

2. **Virtual Environment Issues**:
   ```bash
   # If using virtual environment, ensure it's activated
   source /path/to/venv/bin/activate  # Linux/Mac
   # or on Windows: \path\to\venv\Scripts\activate
   ```

3. **Multiple Python Installations**:
   ```bash
   # Use python3 -m pip to ensure correct pip version
   python3 -m pip install sailboat-playground
   ```

4. **Reinstall if needed**:
   ```bash
   pip3 uninstall sailboat-playground
   pip3 install sailboat-playground
   ```

#### **sailboat-playground API Structure**

The sailboat-playground package provides three main modules:
- **`constants`**: Physical constants and configuration parameters
- **`engine`**: Core simulation engine for sailboat physics (Manager, Boat, Environment)
- **`visualization`**: Rendering and display components (requires graphics)

```python
# Correct usage pattern for Argo integration
import os
os.environ['PYGLET_HEADLESS'] = '1'  # Headless mode

from sailboat_playground.engine import Manager, Boat, Environment

# Initialize simulation components
sim_manager = Manager()
boat = Boat()
environment = Environment()
```

**API Classes Found:**
- **`Manager`**: Main simulation controller
- **`Boat`**: Sailboat physics model
- **`Environment`**: Wind and water conditions
- **`utils`**: Utility functions

#### **Current Integration Status**

✅ **sailboat-playground Detection**: Successfully detected and imported  
✅ **Mock Simulator**: Fully functional with realistic sailing physics  
✅ **Real Simulator Integration**: Configured with proper JSON configuration files  
✅ **Configuration Files**: Sample boat and environment configs available

**Integration Behavior:**
1. **First Attempt**: Try to initialize real sailboat-playground simulator with configuration files
2. **Fallback**: Use reliable mock simulator if real simulator fails or configs missing
3. **Identical Output**: Both simulators provide same ROS2 topic structure

## 📁 **Configuration Files Setup**

The Argo repository now includes a `sailboat-playground/` directory with required configuration files:

```
sailboat-playground/
├── boats/sample_boat.json           # Boat physical properties
├── environments/playground.json     # Wind and water conditions  
├── foils/naca0015.json             # Airfoil characteristics
├── examples/sailing_upwind.py      # Usage example
└── README.md                       # Configuration documentation
```

### **Boat Configuration** (`sample_boat.json`)
- Physical properties: length (1.1m), mass (30kg), sail area (1.0m²)
- Hydrodynamics: hull friction, rotation resistance
- Control surfaces: rudder area (0.02m²), sail/rudder foils

### **Environment Configuration** (`playground.json`)  
- Wind: constant 10 m/s from west (270°)
- Water: no current for simplicity
- Gusts: disabled for stable testing

### **Testing the Integration**

To test the sailboat-playground integration:

```bash
# Test that configuration files are detected
cd /home/orangepi/argo
python3 nodes/argo_simulator_bridge.py --help

# The bridge will attempt to use real sailboat-playground first,
# then fall back to the reliable mock simulator
```

**Current Status:**
- ✅ Configuration files created and detected
- ✅ sailboat-playground imports successfully  
- ✅ Graceful fallback to mock simulator
- 🔧 Advanced physics requires further API tuning

**Note**: The system automatically attempts to use real sailboat-playground with these configurations. Whether using the real or mock simulator, you get identical ROS2 topic output for Argo compatibility. The mock simulator provides reliable operation with realistic sailing physics.

### **Verify ROS2 Environment**
```bash
# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Verify ROS2 is working
ros2 node list
```

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





