# Sailboat Playground Configuration Files

This directory contains configuration files required for using the `sailboat-playground` package with the Argo simulator.

## Directory Structure

```
sailboat-playground/
├── boats/           # Boat configuration files
├── environments/    # Environment/weather configuration files  
├── foils/           # Airfoil/hydrofoil configuration files
├── examples/        # Example usage scripts
└── README.md        # This file
```

## Configuration Files

### Boat Configuration (`boats/sample_boat.json`)
Defines the physical properties of the sailboat:
- **length**: Boat length in meters (1.1m)
- **mass**: Boat mass in kg (30kg) 
- **com_length**: Center of mass position (0.5m)
- **sail_area**: Sail area in m² (1.0m²)
- **sail_foil**: Airfoil profile for sail ("naca0015")
- **rudder_area**: Rudder area in m² (0.02m²)
- **rudder_foil**: Airfoil profile for rudder ("naca0015")
- **hull_area**: Hull underwater area (0.03m²)
- **hull_friction_coefficient**: Drag coefficient (0.2)
- **hull_rotation_resistance**: Rotation resistance (0.4)
- **moment_of_inertia**: Rotational inertia (100 kg⋅m²)

### Environment Configuration (`environments/playground.json`)
Defines wind and water conditions:
- **wind_min_speed/max_speed**: Wind speed range (10 m/s)
- **wind_direction**: Wind direction in degrees (270° = from west)
- **wind_gust_***: Gust parameters (disabled in this config)
- **current_speed**: Water current speed (0 m/s)
- **current_direction**: Current direction (225°)

### Foil Configuration (`foils/naca0015.json`)
Defines airfoil aerodynamic properties:
- **max_lift_coefficient**: Maximum lift coefficient (1.2)
- **min_drag_coefficient**: Minimum drag coefficient (0.02)
- **stall_angle**: Angle where stall occurs (15°)

## Usage with Argo Simulator

The sailboat-playground `Manager` class requires these configuration files:

```python
from sailboat_playground.engine import Manager
import numpy as np

# Initialize with configuration files
manager = Manager(
    "sailboat-playground/boats/sample_boat.json",
    "sailboat-playground/environments/playground.json",
    boat_heading=140,                    # Initial heading in degrees
    boat_position=np.array([0, -300]),   # Initial position [x, y]
    debug=False
)

# Simulation step with control inputs
manager.step([sail_angle, rudder_angle])

# Get current state
state = manager.agent_state
print(f"Position: {state['position']}")
print(f"Heading: {state['heading']}")
print(f"Wind direction: {state['wind_direction']}")
```

## Example Usage

See `examples/sailing_upwind.py` for a complete example of how to use these configuration files with the sailboat-playground engine.

## Integration with Argo

To use this with the Argo simulator bridge, the configuration files should be passed to the sailboat-playground Manager class during initialization in `nodes/argo_simulator_bridge.py`.

## Configuration Customization

You can create custom boat and environment configurations by copying and modifying these JSON files. Adjust parameters to match your specific sailing conditions or boat characteristics.

