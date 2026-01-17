# Argo 3D Visualization Guide

This guide explains how to use the new 3D visualization system for the Argo autonomous sailboat in Foxglove Studio.

## Overview

The 3D visualization system provides a comprehensive view of the Argo sailboat's state, including:
- **3D Boat Model**: Hull, mast, rudder, and sail indicators
- **Wind Visualization**: Wind vector arrow showing speed and direction
- **GPS Data**: Position, velocity vector, and course over ground
- **Control State**: Rudder and sail positions from autonomous commands
- **Sailing Areas**: Geofence boundaries, waypoints, and hazards from GeoJSON files
- **IMU Data**: Roll and pitch indicators from accelerometer data

## Components

### 1. Transform Publisher (`argo_transform_publisher.py`)
- Publishes coordinate frame transforms (`/tf`, `/tf_static`)
- Establishes relationships between map, boat, and sensor frames
- Converts GPS position to map coordinates
- Estimates roll/pitch from IMU accelerometer data

### 2. Boat Visualization Node (`argo_boat_visualization.py`)
- Creates 3D markers for boat state visualization
- Publishes to `/visualization_marker` and `/visualization_marker_array`
- Shows hull, mast, rudder, sail, wind vector, velocity vector, and heading arrow

### 3. Sailing Area Publisher (`sailing_area_publisher.py`)
- Publishes sailing area data from GeoJSON files
- Topics: `/sailing_waypoints`, `/sailing_boundaries`, `/sailing_hazards`
- Includes home location, sailing areas, and hazard zones

### 4. Foxglove Layout (`argo_3d_map.json`)
- Combined map and 3D panel layout
- Shows GPS data on map alongside 3D boat visualization
- Includes data plots for GPS, wind, control, and IMU data

## Quick Start

### 1. Start the Visualization System
```bash
# Start 3D visualization nodes
./scripts/launch_3d_visualization.sh

# In another terminal, start main Argo system
./launch/argo_start.sh
```

### 2. Configure Foxglove Studio
1. Open Foxglove Studio
2. Connect to rosbridge on port 9090
3. Import layout: `foxglove/argo_3d_map.json`

### 3. View the 3D Visualization
- **Map Panel**: Shows GPS track and sailing areas
- **3D Panel**: Shows boat model with real-time state
- **Data Panels**: Show sensor data and control commands

## 3D Visualization Elements

### Boat Model
- **Hull**: Blue cube representing the Dragonforce 65 hull
- **Mast**: Brown cylinder at boat center
- **Rudder**: Red arrow showing rudder position (-30° to +30°)
- **Sail**: White arrow showing sail trim angle (-45° to +45°)

### Environmental Indicators
- **Wind Vector**: Green arrow showing wind speed and direction
- **Velocity Vector**: Yellow arrow showing GPS velocity
- **Heading Arrow**: Cyan arrow showing boat heading direction

### Sailing Area Markers
- **Waypoints**: Green spheres (home location, etc.)
- **Boundaries**: Blue lines (sailing area perimeters)
- **Hazards**: Red lines (rocks, obstacles)

## Coordinate Frames and Transforms

The system uses the following coordinate frame hierarchy with specific transform relationships:

```
map (fixed world frame at GPS origin)
 └── odom (odometry frame, same as map for now)
     └── base_link (boat center)
         ├── gps_link (GPS antenna position)
         ├── compass_link (IMU/magnetometer position)
         ├── wind_sensor_link (anemometer position)
         └── rudder_link (rudder position)
```

### Frame Descriptions

- **`map`**: Fixed world coordinate frame at the GPS origin point. This is the reference frame for all global positioning.
- **`odom`**: Odometry frame, currently identical to map frame. In a full system, this would represent the robot's estimated position from wheel encoders or other odometry sources.
- **`base_link`**: The center of the boat, typically at the waterline. This is the main reference frame for all boat-mounted sensors and actuators.
- **`gps_link`**: GPS antenna position relative to base_link. Accounts for the physical offset of the GPS antenna from the boat center.
- **`compass_link`**: IMU/magnetometer position relative to base_link. Critical for heading and orientation calculations.
- **`wind_sensor_link`**: Anemometer position relative to base_link. Accounts for wind sensor mounting location.
- **`rudder_link`**: Rudder position relative to base_link. Shows the actual rudder angle and position.

### Transform Types

- **`/tf`**: Dynamic transforms that change over time (boat position, rudder angle, etc.)
- **`/tf_static`**: Static transforms that remain constant (sensor mounting positions, etc.)

### ENU Frame (East-North-Up)

The system uses the **ENU (East-North-Up)** coordinate convention:
- **X-axis**: Points East (positive X = East)
- **Y-axis**: Points North (positive Y = North)  
- **Z-axis**: Points Up (positive Z = Up)

This is the standard convention for ROS2 and many robotics applications. The "Root frame" in Foxglove refers to the top-level coordinate frame in the transform tree, typically `map`.

### Common Transform Issues

1. **Rotation Problems**: If the wire mesh or boat model appears rotated 90°, check:
   - IMU calibration and mounting orientation
   - Coordinate frame conventions (ENU vs NED)
   - Transform publishing order and timing

2. **Scale Issues**: If objects appear too large/small:
   - Verify GPS coordinate conversion to meters
   - Check marker scale values in visualization nodes
   - Ensure consistent units across all transforms

3. **Position Drift**: If the boat position doesn't match GPS:
   - Check GPS-to-map coordinate conversion
   - Verify transform publisher is running
   - Ensure GPS fix quality is sufficient

## Data Sources

### Required Topics
- `/fix` (sensor_msgs/NavSatFix): GPS position
- `/pose` (geometry_msgs/Vector3): Boat heading (z-component)
- `/accel` (geometry_msgs/Vector3): IMU accelerometer data
- `/rudder_sail_cmd` (geometry_msgs/Vector3): Control commands
- `/anem_speed_angle_temp` (geometry_msgs/Vector3): Wind sensor data
- `/gps_velocity` (geometry_msgs/Vector3): GPS velocity vector

### Published Topics
- `/tf` (geometry_msgs/TransformStamped): Dynamic transforms
- `/tf_static` (geometry_msgs/TransformStamped): Static transforms
- `/visualization_marker` (visualization_msgs/Marker): Individual markers
- `/visualization_marker_array` (visualization_msgs/MarkerArray): All markers
- `/sailing_waypoints` (visualization_msgs/MarkerArray): Waypoint markers
- `/sailing_boundaries` (visualization_msgs/MarkerArray): Boundary markers
- `/sailing_hazards` (visualization_msgs/MarkerArray): Hazard markers

## Troubleshooting

### Common Issues

1. **3D Panel Shows No Data**
   - Check that transform publisher is running: `ros2 node list | grep transform`
   - Verify GPS fix is available: `ros2 topic echo /fix`
   - Check marker topics: `ros2 topic list | grep visualization`

2. **Boat Model Not Visible**
   - Ensure boat visualization node is running: `ros2 node list | grep visualization`
   - Check marker data: `ros2 topic echo /visualization_marker_array`
   - Verify coordinate frames: `ros2 run tf2_tools view_frames`

3. **Sailing Areas Not Showing**
   - Check sailing area publisher: `ros2 node list | grep sailing`
   - Verify GeoJSON files exist: `ls foxglove/maps/*.geojson`
   - Check sailing area topics: `ros2 topic echo /sailing_waypoints`

4. **Transform Errors**
   - Check transform topics: `ros2 topic echo /tf`
   - Verify GPS data quality: `ros2 topic hz /fix`
   - Check for coordinate frame issues: `ros2 run tf2_ros tf2_echo map base_link`

### Debug Commands

```bash
# Check all visualization nodes
ros2 node list | grep -E "(transform|visualization|sailing)"

# Check visualization topics
ros2 topic list | grep -E "(visualization|sailing|tf)"

# Monitor marker data
ros2 topic echo /visualization_marker_array

# Check transform tree
ros2 run tf2_tools view_frames

# Monitor GPS data
ros2 topic echo /fix

# Check boat state
ros2 topic echo /pose
ros2 topic echo /rudder_sail_cmd
```

## Customization

### Adding New Markers
Edit `argo_boat_visualization.py` to add new visualization elements:
1. Create a new marker method
2. Add it to the `publish_markers()` method
3. Subscribe to required data topics

### Modifying Boat Model
Update the marker creation methods in `argo_boat_visualization.py`:
- Change colors, sizes, or shapes
- Adjust positioning relative to boat center
- Add new boat components

### Adding Sailing Areas
Add new GeoJSON files to `foxglove/maps/`:
1. Create GeoJSON with proper feature structure
2. Restart sailing area publisher
3. Areas will appear automatically in visualization

### Custom Foxglove Layout
Modify `argo_3d_map.json` to:
- Change panel arrangements
- Add new data plots
- Adjust 3D camera settings
- Modify marker colors and visibility

## Performance Notes

- Visualization nodes run at 10 Hz for smooth updates
- Markers are optimized for real-time performance
- Transform publishing is throttled to prevent overload
- Large sailing areas may impact performance

## Integration with Existing System

The 3D visualization system is designed to work alongside the existing Argo system:
- No modifications to core Argo nodes required
- Uses existing sensor data topics
- Compatible with both simulation and real hardware
- Can be started/stopped independently

## Learning Resources

### Foxglove Studio Documentation
- **Official Foxglove Docs**: https://docs.foxglove.dev/
- **3D Panel Guide**: https://docs.foxglove.dev/docs/panels/3d
- **Layouts and Configurations**: https://docs.foxglove.dev/docs/panels/layouts
- **Data Sources**: https://docs.foxglove.dev/docs/connecting-to-data

### ROS2 Transform System
- **TF2 Documentation**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- **Coordinate Frames**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html#coordinate-frames
- **Transform Trees**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html#transform-trees

### 3D Visualization Best Practices
- **ROS2 Visualization Markers**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rviz/Rviz-Interactive-Markers.html
- **Coordinate Frame Conventions**: https://www.ros.org/reps/rep-0103.html
- **ENU vs NED Conventions**: https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates

### Foxglove Community
- **GitHub Repository**: https://github.com/foxglove/studio
- **Discord Community**: https://discord.gg/foxglove
- **Example Layouts**: https://github.com/foxglove/studio/tree/main/packages/studio-base/src/panels/ThreeDimensionalViz

## Future Enhancements

Potential improvements to the 3D visualization system:
- GPS position conversion to map coordinates
- Historical path visualization
- Collision detection visualization
- Weather overlay integration
- Multi-boat racing visualization
- VR/AR support
