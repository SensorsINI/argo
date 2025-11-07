# Argo Maps - GeoJSON Maps Directory

This directory contains **converted GeoJSON map files** used by the Argo simulation system for visualization and boat positioning.

## Contents

- **`.geojson` files**: GeoJSON format map data containing:
  - **Home waypoint**: Boat start location (GPS coordinates)
  - **Sailing boundaries**: LineString and Polygon features
  - **Hazards**: Polygon features marked as hazards

## Current Maps

- `Hotel dei Pini.geojson` - Sailing area for Hotel dei Pini location
- `Argo Irchel pond sailing area.geojson` - Sailing area for Argo Irchel pond

## How Maps Are Generated

GeoJSON files in this directory are **generated from KMZ source files** located in `maps/`:

1. **Source**: KMZ files exported from Google Maps (`maps/*.kmz`)
2. **Conversion**: Run `python3 scripts/kmz_to_geojson.py`
3. **Output**: GeoJSON files in this directory (`foxglove/maps/*.geojson`)

## Using Maps in Simulation

### Step 1: Ensure Map is Converted

If you've added a new KMZ file to `maps/`, convert it:

```bash
cd /home/tobi/argo
python3 scripts/kmz_to_geojson.py
```

### Step 2: Configure Map Selection

Edit `launch/argo_nodes.yaml`:

```yaml
simulation_config:
  map_name: "Hotel dei Pini"  # Use the filename without .geojson extension
```

Available maps:
- `"Hotel dei Pini"`
- `"Argo Irchel pond sailing area"`

### Step 3: Start Simulation

```bash
python3 launch/argo_lifecycle_manager.py simulate_local
```

## Map Structure

Each GeoJSON file contains:

### Home Waypoint (Required)
```json
{
  "type": "Feature",
  "properties": {
    "name": "home",
    "type": "waypoint"
  },
  "geometry": {
    "type": "Point",
    "coordinates": [longitude, latitude, altitude]
  }
}
```

- **Purpose**: Defines the boat's starting GPS location
- **Coordinates**: `[longitude, latitude, altitude]` format
- **Used by**: Simulator bridge to set `base_latitude` and `base_longitude`

### Sailing Boundaries (Optional)
- LineString features with `type: "sailing_boundary"`
- Polygon features with `type: "sailing_area"`
- Displayed as boundary markers in Foxglove Map panel

### Hazards (Optional)
- Polygon features with `type: "hazard"` or containing "rock" in name
- Displayed as hazard markers in Foxglove Map panel

## How Maps Are Used

1. **Simulator Bridge** (`argo_unified_simulator_bridge.py`):
   - Loads the specified map's GeoJSON file
   - Extracts the "home" waypoint coordinates
   - Uses them as the base GPS location for the simulation
   - Converts simulator XY coordinates to lat/lon using this base

2. **Sailing Area Publisher** (`sailing_area_publisher.py`):
   - Loads all GeoJSON files from this directory
   - Publishes waypoints, boundaries, and hazards as ROS2 markers
   - Displays them in Foxglove visualization

3. **Transform Publisher** (`argo_transform_publisher.py`):
   - Uses GPS fix data to establish coordinate frames
   - Creates TF transforms for 3D visualization

## Notes

- **Do not edit GeoJSON files manually** - they are auto-generated from KMZ sources
- **Update source KMZ files** in `maps/` and re-run the conversion script
- The "home" waypoint **must exist** in the GeoJSON for proper boat positioning
- If no "home" waypoint exists, the simulator calculates the map center as fallback

