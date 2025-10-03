# Argo Sailing Area Map Visualization in Foxglove

This guide explains how to visualize your Argo robot boat's sailing areas in Foxglove Studio using the KMZ files from your maps folder.

## Overview

Your KMZ files contain sailing area definitions for two locations:

1. **Hotel dei Pini** (Sardinia, Italy) - Coastal sailing area
2. **Argo Irchel pond** (Zurich, Switzerland) - Pond sailing area with hazards

## Quick Start

### 1. Convert KMZ to GeoJSON
```bash
cd /home/orangepi/argo
python3 scripts/kmz_to_geojson.py
```

This creates GeoJSON files in `/home/orangepi/argo/foxglove/maps/` that Foxglove can use.

### 2. Start Argo with Map Visualization
```bash
# Start the sailing area publisher
python3 nodes/sailing_area_publisher.py &

# Start Argo normally
sudo python3 launch/argo_lifecycle_manager.py start
```

### 3. Connect Foxglove
1. Open [Foxglove Studio](https://studio.foxglove.dev/)
2. Connect to your robot: `ws://YOUR_ROBOT_IP:9090`
3. Import the layout: `argo_maps_final.json`

## Available Map Topics

The sailing area publisher creates these ROS2 topics:

### `/sailing_waypoints` (MarkerArray)
- **Green spheres** marking home positions
- One for each sailing area

### `/sailing_boundaries` (MarkerArray)  
- **Blue lines** showing sailing area boundaries
- **Green lines** for sailing area perimeters

### `/sailing_hazards` (MarkerArray)
- **Red lines** marking dangerous areas (rocks, etc.)

## Map Panel Configuration

The map panel in Foxglove shows:
- **GPS position** of your boat (green dot)
- **Sailing boundaries** (blue lines)
- **Hazards** (red lines) 
- **Waypoints** (green spheres)

### Map Center Coordinates
- **Hotel dei Pini**: 40.584°N, 8.255°E
- **Irchel Pond**: 47.398°N, 8.545°E

## Customizing the Visualization

### Adding New Sailing Areas
1. Create a KMZ file with your sailing area
2. Place it in `/home/orangepi/argo/maps/`
3. Run the conversion script: `python3 scripts/kmz_to_geojson.py`
4. Restart the sailing area publisher

### Modifying Colors
Edit `nodes/sailing_area_publisher.py` and change the color values:
```python
# Waypoints (green)
marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

# Sailing areas (blue)  
marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)

# Hazards (red)
marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
```

### Adjusting Map Zoom
In the Foxglove layout, modify the map panel zoom level:
```json
"zoom": 18  // Higher = more zoomed in
```

## Troubleshooting

### No Map Data Visible
1. Check if sailing area publisher is running:
   ```bash
   ros2 topic list | grep sailing
   ```

2. Verify GeoJSON files exist:
   ```bash
   ls -la /home/orangepi/argo/foxglove/maps/
   ```

3. Check for errors:
   ```bash
   ros2 topic echo /sailing_waypoints
   ```

### Map Not Centered
The map automatically centers on the first sailing area. To change this:
1. Edit the layout JSON file
2. Modify the `center` coordinates in the map panel config

### Performance Issues
- Reduce marker update frequency in `sailing_area_publisher.py`
- Disable unused marker types in Foxglove
- Use lower zoom levels for better performance

## Integration with Navigation

Your sailing areas can be used for:
- **Geofencing**: Stay within safe boundaries
- **Path planning**: Avoid hazards and obstacles  
- **Mission planning**: Navigate between waypoints
- **Safety monitoring**: Alert when approaching hazards

## File Structure

```
/home/orangepi/argo/
├── maps/                                    # Original KMZ files
│   ├── Hotel dei Pini.kmz
│   └── Argo Irchel pond sailing area.kmz
├── foxglove/
│   ├── maps/                               # Converted GeoJSON files
│   │   ├── Hotel dei Pini.geojson
│   │   └── Argo Irchel pond sailing area.geojson
│   ├── argo_with_maps_layout.json         # Foxglove layout
│   └── MAP_VISUALIZATION_GUIDE.md         # This guide
├── scripts/
│   └── kmz_to_geojson.py                  # Conversion script
└── nodes/
    └── sailing_area_publisher.py          # ROS2 publisher
```

## Advanced Usage

### Creating Custom Sailing Areas
1. Use Google Earth to create your sailing area
2. Draw polygons for safe areas and hazards
3. Add waypoints for important locations
4. Export as KMZ file
5. Convert and use with Argo

### Real-time Updates
The sailing area publisher republishes markers every 30 seconds to handle reconnections. You can modify this interval in the code.

### Multiple Sailing Areas
The system supports multiple sailing areas simultaneously. All areas will be displayed on the same map with different colors for different types of features.
