# Argo Control System Refactoring

This document describes the refactored control system architecture that addresses three key requirements:

1. **Single `generate_control(state) -> control` function**
2. **Easy controller swapping mechanism**
3. **End-to-end data collection for training**

## Architecture Overview

### Core Components

- **`BoatState`** - Dataclass encapsulating all sensor inputs (compass, GPS, wind, IMU, radio)
- **`ControlCommand`** - Dataclass for rudder/sail output commands
- **`BaseController`** - Abstract base class defining the `generate_control()` interface
- **`DataCollector`** - Automatic training data collection during human control
- **`ControlNode`** - Main ROS2 node with modular controller architecture

### Key Design Principles

```python
# THE CORE ARCHITECTURE: Every controller implements this interface
class BaseController(ABC):
    @abstractmethod
    def generate_control(self, state: BoatState) -> ControlCommand:
        """Single function: state -> control"""
        pass
```

## Available Controllers

### 1. ProportionalHeadingController
Simple heading maintenance using proportional control:
- Uses compass heading error for rudder control  
- Passes through radio sail commands
- Equivalent to original control.py logic

### 2. WindAwareController  
Enhanced controller considering wind conditions:
- Same heading control as proportional
- Wind-based sail control using wind angle
- Blends radio and wind-based sail commands

## Easy Controller Swapping

### Method 1: Parameter File
```yaml
# argo_refactored.yaml
control_node:
  ros__parameters:
    controller_type: "wind_aware"  # Switch to wind-aware controller
    rudder_gain: 1.5
    sail_wind_gain: 0.3
```

### Method 2: Runtime Parameter Change
```bash
ros2 param set /control_node controller_type "proportional"
```

### Method 3: Hot-swapping via Config File
The system monitors the parameter file and automatically switches controllers when `controller_type` changes.

## Data Collection for Training

### Automatic Collection
```yaml
# Enable data collection
control_node:
  ros__parameters:
    data_collection_enabled: true
```

### Data Collection Process
1. **Start**: When human takes control, a new training session begins
2. **Record**: At 10Hz, captures `(BoatState, HumanAction)` pairs
3. **Stop**: When switching to computer control, saves session to JSON file

### Output Format
```json
{
  "start_time": 1694123456.789,
  "end_time": 1694123556.789, 
  "sample_count": 1000,
  "data": [
    {
      "state": {
        "compass_heading": 45.2,
        "target_heading": 50.0,
        "wind_speed": 5.2,
        "wind_angle": 120.0,
        "gps_sog": 4.5,
        // ... all sensor data
      },
      "action": {
        "rudder": 0.15,
        "sail": -0.3,
        "timestamp": 1694123457.123
      },
      "relative_time": 0.334
    }
    // ... more samples
  ]
}
```

## Usage Examples

### 1. Run with Proportional Controller
```bash
# Use existing config (defaulted to proportional)
python3 control_refactored.py

# Or specify explicitly
ros2 run argo control_refactored.py --ros-args -p controller_type:=proportional
```

### 2. Run with Data Collection
```bash
# Enable data collection in config file first
python3 control_refactored.py
# Training data will be saved to training_data/ directory
```

### 3. Analyze Collected Data
```bash
# Analyze human control patterns and train neural network
python3 training_data_example.py --data_dir training_data --output_dir analysis_results

# Skip neural training if not enough data
python3 training_data_example.py --skip_training
```

### 4. Switch Controllers During Operation
```bash
# Edit argo_refactored.yaml and change controller_type
# System will automatically detect file change and switch controllers
```

## Adding New Controllers

### Step 1: Create Controller Class
```python
class MyNewController(BaseController):
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        # Initialize your controller parameters
    
    def generate_control(self, state: BoatState) -> ControlCommand:
        # Implement your control logic
        # Input: complete boat state
        # Output: rudder and sail commands
        return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)
```

### Step 2: Register in ControlNode
```python
# In ControlNode._initialize_controller()
elif controller_type == 'my_new_controller':
    self.controller = MyNewController(config)
```

### Step 3: Use New Controller
```yaml
# argo_refactored.yaml
control_node:
  ros__parameters:
    controller_type: "my_new_controller"
```

## Benefits of This Architecture

1. **Clean Separation**: ROS2 communication separated from control logic
2. **Testability**: Controllers can be unit tested with mock BoatState objects
3. **Modularity**: Easy to add new controllers without touching ROS2 code
4. **Data-Driven**: Automatic collection enables machine learning approaches
5. **Hot-Swapping**: Change controllers without restarting the system
6. **Comprehensive State**: All sensor data available to every controller

## Files

- `control_refactored.py` - Main refactored control node
- `argo_refactored.yaml` - Example configuration with different controllers
- `training_data_example.py` - Analysis and neural network training example
- `README_control_refactor.md` - This documentation

## Migration Path

1. **Test**: Run `control_refactored.py` alongside original `control.py`
2. **Validate**: Ensure equivalent behavior with proportional controller
3. **Collect Data**: Enable data collection during manual operation
4. **Experiment**: Try wind-aware controller and custom controllers
5. **Deploy**: Replace original control.py when satisfied

The refactored architecture maintains full backward compatibility while enabling advanced control algorithms and data-driven development.
