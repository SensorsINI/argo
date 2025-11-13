# How to Test Patrol Controller

## Quick Setup

### Method 1: Edit Configuration File (Recommended)

1. **Edit `nodes/argo.yaml`**:
   ```yaml
   controller_node:
     ros__parameters:
       controller_type: "patrol"  # Change from "proportional" to "patrol"
   ```

2. **Start simulation**:
   ```bash
   asim --force-mock
   ```

3. **In another terminal, start keyboard control**:
   ```bash
   asimkb
   ```

4. **Switch to robot control**: 
   - In the keyboard control terminal, stop providing input
   - The controller will automatically switch to robot control after the timeout
   - Or use the pause toggle: `ap` (pauses/unpauses controller)

### Method 2: Override via ROS2 Parameter (After Starting)

If you want to switch controllers without editing the file:

1. **Start simulation with default controller**:
   ```bash
   asim --force-mock
   ```

2. **Switch to patrol controller**:
   ```bash
   ros2 param set /controller_node controller_type "patrol"
   ```

3. **Verify controller switched**:
   ```bash
   ros2 topic echo /controller_state
   ```
   Should show: `data: "PatrolController"`

## Patrol Controller Parameters

The patrol controller uses these parameters (already configured in `argo.yaml`):

- `patrol_lookahead_time: 15.0` - Seconds to predict ahead for boundary crossing
- `boundary_turn_threshold: 15.0` - Distance to boundary to trigger turn (meters)
- `tack_angle: 90.0` - Degrees to turn during tack maneuver
- `broad_reach_angle: 110.0` - Preferred angle to wind for patrol (degrees)
- `geofence_map_name: "Argo Irchel pond sailing area"` - GeoJSON map name

## Testing Workflow

1. **Start simulation**: `asim --force-mock`
2. **Start keyboard control**: `asimkb` (in another terminal)
3. **Manually position boat**: Use arrow keys to move boat to center of sailing area
4. **Switch to robot control**: Stop keyboard input (wait for timeout) or press `ap` to unpause
5. **Observe patrol behavior**: 
   - Boat should sail on broad reaches
   - When approaching boundary, should execute tack/jibe
   - Should stay within geofence

## Monitoring

- **Controller state**: `ros2 topic echo /controller_state`
- **Distance to boundary**: Check controller logs for boundary distance
- **Boundary crossing prediction**: Check controller logs for crossing time predictions
- **Sailing mode**: Check controller logs for mode changes (broad_reach, tacking, jibing)

## Troubleshooting

- **Controller not switching**: Check that `controller_type: "patrol"` is set correctly
- **Geofence not loading**: Verify map file exists at `foxglove/maps/Argo Irchel pond sailing area.geojson`
- **No boundary detection**: Ensure GPS position is being published on `/fix` topic
- **Boat not responding**: Check that controller is not paused (`ap` to toggle)

