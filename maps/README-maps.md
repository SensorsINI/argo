# Argo Maps - Source Maps Directory

This directory contains the **source map files** drawn in Google Maps and exported as KMZ files.

## Contents

- **`.kmz` files**: Google Maps exports containing sailing areas, waypoints, boundaries, and hazards
- **`.pdf` files**: Map visualizations and documentation
- **`doc.kml`**: KML source files (extracted from KMZ archives)

## Map Files

- `Hotel dei Pini.kmz` - Sailing area map for Hotel dei Pini location
- `Argo Irchel pond sailing area.kmz` - Sailing area map for Argo Irchel pond

## Converting Maps for Simulation

### Step 1: Export Map from Google Maps

1. Draw your sailing area in Google Maps (My Maps)
2. Include a **"home" waypoint** - this will be the boat's start location
3. Export as KMZ file
4. Save the `.kmz` file to this directory (`maps/`)

### Step 2: Convert KMZ to GeoJSON

Run the conversion script to generate GeoJSON files for Foxglove visualization:

```bash
cd /home/tobi/argo
python3 scripts/kmz_to_geojson.py
```

This will:
- Read all `.kmz` files from `maps/`
- Extract KML data
- Convert to GeoJSON format
- Save to `foxglove/maps/*.geojson`

### Step 3: Configure Simulation

Edit `launch/argo_nodes.yaml` to specify which map to use:

```yaml
simulation_config:
  map_name: "Hotel dei Pini"  # Map name without .geojson extension
```

### Step 4: Start Simulation

```bash
python3 launch/argo_lifecycle_manager.py simulate_local
```

The simulator will:
- Load the specified map's GeoJSON file
- Find the "home" waypoint
- Start the boat at that GPS location
- Display sailing areas, boundaries, and hazards in Foxglove

## Map Requirements

For proper simulation setup, your KMZ file should include:

1. **Home Waypoint** (required):
   - Name: `home`
   - Type: Point/Placemark
   - This becomes the boat's starting GPS location

2. **Sailing Boundaries** (optional):
   - LineString or Polygon features
   - Will be displayed as boundary markers in Foxglove

3. **Hazards** (optional):
   - Polygon features with "rock" or "hazard" in the name
   - Will be displayed as hazard markers

## Notes

- The conversion from KMZ to GeoJSON is a **one-time process** - you only need to run it when maps change
- GeoJSON files are stored in `foxglove/maps/` and used by the simulation system
- The "home" waypoint coordinates determine where the boat starts in the simulation

