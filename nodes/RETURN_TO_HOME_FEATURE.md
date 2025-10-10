# Return-to-Home Feature Implementation

## Overview

The controller now supports **automatic return-to-home (RTH)** functionality that activates when WiFi/LoRa connection to shore is lost. This ensures the Argo sailboat can autonomously navigate back to its starting position if remote communication is interrupted.

## How It Works

### Home Position Tracking
- **Automatic**: Home position is automatically set on the **first valid GPS fix** after startup
- **Stored**: Latitude and longitude of starting position are stored in `BoatState`
- **Current Position**: Continuously tracks current GPS position via `/fix` topic

### Activation Triggers

The return-to-home mode activates under two conditions:

1. **Manual Command via LoRa**:
   ```bash
   # From shore side
   ros2 topic pub --once /lora_remote_command std_msgs/String "data: 'return_home'"
   ```

2. **Automatic on Connection Loss**:
   - If no LoRa contact for **120 seconds** (configurable)
   - Controller automatically switches to RTH mode
   - Logs warning: "🏠 Switching to RETURN TO HOME mode due to connectivity loss"

### Navigation Behavior

When RTH is active:
- Calculates **bearing to home** using GPS coordinates (Haversine formula)
- Calculates **distance to home** in nautical miles
- Updates `target_heading` to point toward home
- Uses wind-aware sail control for efficient navigation
- **Arrival detection**: Stops within 0.05nm (~90 meters) of home position

## Architecture Changes

### 1. Extended BoatState Dataclass

```python
# Connectivity monitoring
shore_connected: bool = False                # LoRa connection status
last_shore_contact: Optional[float] = None   # Timestamp of last contact
remote_command: Optional[str] = None         # Latest remote command

# Home position tracking
home_latitude: Optional[float] = None        # Starting position
home_longitude: Optional[float] = None
current_latitude: Optional[float] = None     # Current GPS position
current_longitude: Optional[float] = None
return_to_home_active: bool = False          # RTH mode flag
```

### 2. New Navigation Methods

```python
# Calculate bearing to home (0-360 degrees)
bearing = boat_state.get_bearing_to_home()

# Calculate distance to home (nautical miles)
distance = boat_state.get_distance_to_home()
```

### 3. New Controller Class: `ReturnToHomeController`

Dedicated controller for autonomous return-to-home navigation:
- **GPS-based navigation**: Uses bearing to home as target heading
- **Wind-aware sail control**: Optimizes sail position based on wind angle
- **Arrival detection**: Switches to neutral drift mode when home reached
- **Fallback behavior**: Maintains heading if GPS unavailable

## ROS2 Topic Integration

### Subscribed Topics (New)

| Topic | Type | Purpose |
|-------|------|---------|
| `/lora_connection_status` | `Bool` | LoRa connectivity status |
| `/lora_remote_command` | `String` | Remote commands from shore |
| `/fix` | `NavSatFix` | GPS position (lat/lon) |

### Remote Commands

| Command | Action |
|---------|--------|
| `return_home` | Activate RTH mode |
| `autonomous` | Resume normal autonomous operation |
| `stop` | Safe stop mode (logs warning) |

## Configuration Parameters

Add to `argo.yaml`:

```yaml
controller_node:
  ros__parameters:
    controller_type: 'proportional'  # or 'return_to_home' for manual RTH
    rudder_gain: 1.0
    rudder_full_scale_deg: 60.0
    # RTH-specific parameters (automatically used when RTH activates)
    shore_connection_timeout: 120.0    # seconds before auto-RTH
    arrival_distance_nm: 0.05          # distance to consider "arrived" (~90m)
```

## Usage Examples

### Manual Return-to-Home Activation

From shore side (via LoRa):
```bash
# Send RTH command
ros2 topic pub --once /lora_remote_command std_msgs/String "data: 'return_home'"

# Check RTH status
ros2 topic echo /rudder_sail_cmd
```

### Resume Normal Operation

```bash
# Cancel RTH and resume normal autonomous control
ros2 topic pub --once /lora_remote_command std_msgs/String "data: 'autonomous'"
```

### Monitor RTH Status

```bash
# Watch controller logs
journalctl -u argo-launch.service -f | grep "RETURN TO HOME"

# Check GPS position
ros2 topic echo /fix

# Monitor LoRa connectivity
ros2 topic echo /lora_connection_status
```

## Testing Without LoRa Hardware

If LoRa hardware is not yet installed, you can simulate the functionality:

```bash
# Simulate connection loss
ros2 topic pub /lora_connection_status std_msgs/Bool "data: false"

# Simulate RTH command
ros2 topic pub --once /lora_remote_command std_msgs/String "data: 'return_home'"

# Provide mock GPS position
ros2 topic pub /fix sensor_msgs/NavSatFix "..."
```

## Logging and Monitoring

The controller provides detailed logging:

```
📡 Shore connection ESTABLISHED via LoRa
📡 Shore connection LOST - Return-to-home may activate
📡 Received remote command: 'return_home'
🏠 Home position set: 37.123456°, -122.654321°
🏠 RETURN TO HOME activated - Bearing: 270.5°, Distance: 1.23nm
🏠 RETURN TO HOME - Distance: 0.85nm, Bearing: 268.3°, Current heading: 265.1°
🏠 Switching to RETURN TO HOME mode due to connectivity loss
```

## Safety Features

1. **Graceful Degradation**: If GPS unavailable, maintains current heading
2. **Arrival Detection**: Automatically stops when within 90m of home
3. **Human Override**: Human control always takes priority
4. **Battery Integration**: Works alongside existing battery/saltwater alerts
5. **Persistent State**: Home position persists during session

## Integration with Existing System

### Compatible with Existing Controllers
- RTH can be used alongside `proportional` and `wind_aware` controllers
- Automatic switching to RTH on connection loss
- Manual switching via LoRa commands

### No Breaking Changes
- All existing functionality remains unchanged
- RTH is opt-in via configuration or remote command
- Existing topics and parameters fully compatible

## Future Enhancements

Potential improvements:
1. **Waypoint Navigation**: Multiple waypoints instead of single home position
2. **Obstacle Avoidance**: Integration with collision detection
3. **Wind-Optimized Routes**: Calculate upwind/downwind optimal paths
4. **Multi-Home Positions**: Support for multiple safe harbor locations
5. **RTH Persistence**: Save home position to file for crash recovery

## Technical Details

### Bearing Calculation
Uses **Haversine formula** for great circle navigation:
```python
bearing = atan2(sin(Δlon)·cos(lat2), cos(lat1)·sin(lat2) - sin(lat1)·cos(lat2)·cos(Δlon))
```

### Distance Calculation
Great circle distance in nautical miles (Earth radius = 3440.065nm):
```python
distance = 2·R·asin(√(sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)))
```

### Coordinate System
- **GPS**: WGS84 datum (latitude/longitude in decimal degrees)
- **Bearing**: True north (0-360°, 0° = North, 90° = East)
- **Distance**: Nautical miles (1nm = 1852 meters)

## Files Modified

- `nodes/controller.py`: Main implementation
  - Extended `BoatState` with connectivity and GPS tracking
  - Added `ReturnToHomeController` class
  - New callbacks for LoRa and GPS position
  - RTH activation logic in control loop

## Dependencies

All required dependencies are already in `requirements.txt`:
- `rclpy` - ROS2 Python client
- `sensor_msgs` - For NavSatFix message
- `numpy` - For mathematical operations
- `math` - Standard library (Haversine calculations)

## Testing Checklist

- [x] GPS position tracking and home position setting
- [x] LoRa connectivity monitoring
- [x] Remote command parsing
- [x] Bearing and distance calculations
- [x] RTH controller activation
- [x] Arrival detection
- [x] Controller switching
- [x] Logging and status reporting
- [ ] Hardware testing with actual LoRa radio
- [ ] GPS navigation accuracy validation
- [ ] Wind-aware sail optimization testing

## Questions?

For implementation details, see:
- `nodes/controller.py` - Main implementation
- `nodes/lora.py` - LoRa communication node
- `.cursor/rules/argo-lora-communication.mdc` - LoRa system documentation

