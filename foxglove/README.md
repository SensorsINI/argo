# Foxglove Visualization Setup for Argo ROS2

This directory contains everything you need to visualize your Argo sailboat's ROS2 data in Foxglove Studio.

## Quick Start

### 1. Install rosbridge_server
```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-suite
```

### 2. Start Argo with Foxglove support
```bash
# Option A: Use the combined launch file (recommended)
ros2 launch ~/argo/foxglove/argo_with_foxglove_launch.py

# Option B: Start argo normally, then rosbridge separately
make start  # or your usual argo startup
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Connect Foxglove Studio
1. Open [Foxglove Studio](https://studio.foxglove.dev/) in your browser
2. Click "Open connection"
3. Select "Rosbridge (WebSocket)"
4. Enter: `ws://YOUR_ROBOT_IP:9090`
5. Find your robot's IP: `ip addr show | grep 'inet ' | grep -v '127.0.0.1'`

### 4. Load a layout
1. Click "Layout" in the top menu
2. Select "Import layout"
3. Choose one of these files:
   - `argo_layout.json` - Basic 4-panel layout
   - `argo_comprehensive_layout.json` - Full 6-panel layout

## Available Layouts

### Basic Layout (`argo_layout.json`)
- **3D Panel**: Shows GPS velocity vectors
- **GPS Panel**: Speed and course over ground
- **IMU Panel**: Accelerometer and gyroscope data
- **Wind Panel**: Wind speed, angle, and temperature

### Comprehensive Layout (`argo_comprehensive_layout.json`)
- **3D Panel**: GPS velocity visualization
- **GPS Panel**: Speed and course tracking
- **IMU Panel**: Full 6-axis IMU data
- **Wind Panel**: Wind sensor data
- **Power Panel**: Battery, current, and water monitoring
- **Control Panel**: Rudder/sail commands and radio input

## ROS2 Topics Visualized

### GPS Data
- `/gps_data` - Raw NMEA sentences
- `/gps_sog` - Speed over ground (knots)
- `/gps_cog` - Course over ground (degrees)
- `/gps_velocity` - Velocity vector (x=north, y=east, z=speed)

### IMU Data
- `/accel` - Accelerometer (x, y, z in g)
- `/gyro` - Gyroscope (x, y, z in deg/s)
- `/compass` - Magnetometer (x, y, z in µT)

### Wind Data
- `/anem_diffpressure` - Raw pressure differences
- `/anem_speed_angle_temp` - Wind speed (m/s), angle (deg), temp (C)

### Power & Environment
- `/battery_voltage` - Battery voltage
- `/battery_remaining_pct` - Battery percentage
- `/sail_current` - Sail servo current
- `/saltwater_voltage` - Water intrusion sensor
- `/temperature` - Environmental temperature
- `/relative_humidity` - Humidity percentage

### Control Data
- `/rudder_sail_radio` - Radio control input
- `/rudder_sail_servo` - Servo output commands
- `/rudder_sail_cmd` - Autonomous control commands
- `/human_controlled` - Manual override status

### Alerts
- `/battery_low_alert` - Low battery warning
- `/saltwater_alert` - Water intrusion alert
- `/humidity_alert` - High humidity warning

## Testing the Connection

Run the test publisher to verify Foxglove can receive data:
```bash
ros2 run argo ~/argo/foxglove/test_connection.py
```

You should see a "foxglove_test" topic in Foxglove with incrementing messages.

## Troubleshooting

### Connection Issues
1. **Can't connect to robot**: Check IP address and ensure rosbridge is running
2. **No topics visible**: Verify argo nodes are running (`ros2 topic list`)
3. **Layout not loading**: Check JSON file path and format

### Performance Issues
1. **Slow updates**: Reduce plot history length in Foxglove settings
2. **High CPU usage**: Disable unused panels or reduce update rates

### Common Commands
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check available topics
ros2 topic list

# Monitor a specific topic
ros2 topic echo /gps_sog

# Check rosbridge logs
ros2 launch rosbridge_server rosbridge_websocket_launch.xml --ros-args --log-level debug
```

## Customization

### Adding New Panels
1. Edit the layout JSON file
2. Add new panel configurations
3. Update the layout structure
4. Import the updated layout in Foxglove

### Modifying Existing Panels
1. Open the layout in Foxglove
2. Right-click on a panel → "Settings"
3. Modify colors, scales, or data sources
4. Save the layout for future use

## Integration with Makefile

You can add Foxglove support to your existing Makefile aliases:

```bash
# Add to your ~/.bash_aliases after running 'make aliases'
alias af='ros2 launch ~/argo/foxglove/argo_with_foxglove_launch.py'
alias afb='ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
```

Then use:
- `af` - Launch argo with Foxglove support
- `afb` - Launch just rosbridge for existing argo session
