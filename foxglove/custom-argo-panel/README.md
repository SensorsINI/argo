# Argo Sailboat Custom Foxglove Panel

This is a custom Foxglove panel extension specifically designed for visualizing the Argo sailboat's state in real-time. The panel extends Foxglove's capabilities to show compass heading, wind data, control positions, and GPS velocity in an intuitive interface.

## Features

### Real-time Data Visualization
- **Compass Heading**: Shows boat orientation using magnetometer data from `/compass` topic
- **Wind Data**: Displays wind speed, direction (relative and absolute), and temperature from `/anem_speed_angle_temp`
- **Control Status**: Shows rudder and sail positions from `/rudder_sail_radio` topic
- **GPS Velocity**: Displays course over ground and speed from `/gps_velocity` topic
- **Control Mode**: Indicates whether the boat is in manual or autonomous mode

### Visual Indicators
- **Compass Rose**: Interactive compass showing current heading with rotating needle
- **Wind Direction Arrow**: Visual arrow showing wind direction and strength
- **Control Bars**: Real-time indicators for rudder and sail positions
- **Status Colors**: Color-coded indicators for different states

## Installation

### Prerequisites
- Node.js (v16 or later)
- npm or yarn package manager
- Foxglove Studio

### Setup
1. Navigate to the panel directory:
   ```bash
   cd ~/argo/foxglove/custom-argo-panel
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Build the extension:
   ```bash
   npm run build
   ```

4. Package the extension:
   ```bash
   npm run package
   ```

## Usage

### Loading the Panel in Foxglove
1. Open Foxglove Studio
2. Connect to your Argo ROS2 system via rosbridge
3. Add the custom panel by:
   - Going to the panel menu
   - Selecting "Add panel"
   - Choosing "argo-sailboat-panel"

### Required ROS2 Topics
The panel subscribes to these topics (all should be available when Argo nodes are running):

- `/compass` (geometry_msgs/Vector3) - Magnetometer data
- `/anem_speed_angle_temp` (geometry_msgs/Vector3) - Wind speed, direction, temperature
- `/rudder_sail_radio` (geometry_msgs/Vector3) - Rudder and sail positions
- `/gps_velocity` (geometry_msgs/Vector3) - GPS velocity vector
- `/human_controlled` (std_msgs/Bool) - Control mode status

### Data Interpretation

#### Compass Heading
- **Source**: `/compass` topic z-component
- **Units**: Degrees (0-360°)
- **Reference**: Magnetic north

#### Wind Data
- **Speed**: `/anem_speed_angle_temp.x` in m/s
- **Direction**: `/anem_speed_angle_temp.y` in degrees CW from boat front
- **Temperature**: `/anem_speed_angle_temp.z` in Celsius
- **Absolute Direction**: Calculated as compass heading + wind angle

#### Control Data
- **Rudder**: `/rudder_sail_radio.x` (-1 = full left, +1 = full right)
- **Sail**: `/rudder_sail_radio.y` (-1 = pulled in, +1 = let out)
- **Mode**: `/human_controlled` (true = manual, false = autonomous)

#### GPS Velocity
- **Speed**: `/gps_velocity.z` in knots
- **North Component**: `/gps_velocity.x` in knots
- **East Component**: `/gps_velocity.y` in knots

## Development

### File Structure
```
custom-argo-panel/
├── src/
│   ├── index.ts              # Extension entry point
│   └── ArgoSailboatPanel.tsx # Main panel component
├── package.json              # Dependencies and scripts
├── tsconfig.json             # TypeScript configuration
└── README.md                 # This file
```

### Key Components

#### `index.ts`
- Entry point for the Foxglove extension
- Registers the panel with the extension context

#### `ArgoSailboatPanel.tsx`
- Main React component for the panel UI
- Handles message subscriptions and state management
- Implements visual indicators and data display

### Customization

To modify the panel:

1. **Add New Topics**: Update the message handlers in `ArgoSailboatPanel.tsx`
2. **Change Visual Style**: Modify the CSS styles in the component
3. **Add New Indicators**: Extend the visual indicators section
4. **Modify Data Processing**: Update the calculation functions

### Building and Testing

```bash
# Development mode with watch
npm run dev

# Production build
npm run build

# Package for distribution
npm run package
```

## Integration with Argo System

This panel is designed to work seamlessly with the Argo ROS2 system:

1. **Automatic Topic Detection**: The panel automatically subscribes to Argo topics
2. **Real-time Updates**: Updates at 10Hz to match Argo node publishing rates
3. **Error Handling**: Gracefully handles missing or invalid data
4. **Performance Optimized**: Efficient rendering to prevent memory leaks

## Troubleshooting

### Common Issues

1. **No Data Displayed**
   - Check that Argo nodes are running: `ros2 node list`
   - Verify topics are publishing: `ros2 topic list`
   - Ensure rosbridge is connected: Check Foxglove connection status

2. **Panel Not Loading**
   - Verify extension is built: `npm run build`
   - Check browser console for errors
   - Ensure all dependencies are installed

3. **Incorrect Data Values**
   - Verify topic message types match expected format
   - Check coordinate system definitions in the code
   - Validate data ranges and units

### Debug Mode
Enable debug logging by opening browser developer tools and checking the console for detailed information about message reception and processing.

## Contributing

To contribute improvements to this panel:

1. Make changes to the source code
2. Test thoroughly with live Argo data
3. Update documentation as needed
4. Submit pull request with detailed description

## License

MIT License - See package.json for details.

