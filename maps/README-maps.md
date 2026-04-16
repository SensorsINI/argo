# Argo Maps - Source Maps Directory

This directory contains the **source map files** drawn in [Google Maps MyMaps](https://www.google.com/maps/d/) and exported as KMZ files.

Maps are originally drawn in Google Maps My Maps. Here are samples

<img src="images/google%20mymaps.png" alt="MyMaps Example" width="500"/>


## Contents

- **`.kmz` files**: Google Maps exports containing sailing areas, waypoints, boundaries, and hazards
- **`.pdf` files**: Map visualizations and documentation
- **`doc.kml`**: KML source files (extracted from KMZ archives)

## Map Files

- `Hotel dei Pini.kmz` - [Hotel dei Pini](https://www.google.com/maps/d/viewer?mid=1YiW7Jl0Miv5geXh6tLaaxXCPSLpgqD0&usp=sharing) - Sailing area map for Hotel dei Pini location; swimming area in front of hotel excluding the boat lane and including the estimated swimming line that must be avoided.
- `Argo Irchel pond sailing area.kmz` - [Argo Irchel pond sailing area](https://www.google.com/maps/d/viewer?mid=1579Z0YR2ZneAvUGbFDLzg9fJPD2w91Q&usp=sharing) - Sailing area map for Argo Irchel pond; includes rock and complex shape.
- `Argo Irchel North pond sailing area.kmz` - [Argo Irchel North pond sailing area](https://www.google.com/maps/d/viewer?mid=14-I2RigSZ3UgvEm4Odftd1UoRcAeuW8&usp=sharing) - Sailing area map for Argo Irchel North pond, a simpler map with 'middle' waypoint for crossing controllers.


## Converting Maps for Simulation

 1. [Hotel dei Pini](https://www.google.com/maps/d/viewer?mid=1YiW7Jl0Miv5geXh6tLaaxXCPSLpgqD0&usp=sharing)
2. [Argo Irchel North pond sailing area](https://www.google.com/maps/d/viewer?mid=14-I2RigSZ3UgvEm4Odftd1UoRcAeuW8&usp=sharing)

### Step 1: Export Map from Google Maps

1. Draw your sailing area in [Google Maps (My Maps)](https://www.google.com/maps/d/)
2. Include a **"home" waypoint** - this will be the boat's start location
3. Include a **"middle"** waypoint if you want to customize the point for crosser controller.
4. Optionally add **`approach_1`**, **`approach_2`**, … placemarks for a launch path into the sailing area (see [Approach waypoints](#approach-waypoints-entering-the-sailing-area) below).
5. Include hazard markers/lines for Argo to avoid (rocks, swimming lines, buoys)
6. Export as KMZ file
7. Save the `.kmz` file to this directory (`maps/`)

### Step 2: Convert KMZ to GeoJSON

Run the conversion script to generate GeoJSON files for Foxglove visualization:

```bash
cd /path/to/argo
python3 scripts/kmz_to_geojson.py
```

This will:
- Read all `.kmz` files from `maps/`
- Extract KML data
- Convert to GeoJSON format
- Save to `foxglove/maps/*.geojson`

### Step 3: Configure Argo

Set the map name in `nodes/argo.yaml` (parameter `geofence_map_name`). Use the base name of the GeoJSON file under `foxglove/maps/` (no `.geojson` extension), for example `Hotel dei Pini`.

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

4. **Middle waypoint** (optional but recommended for crosser / sim):
   - Name: `middle`
   - Same placemark style as `home` (a Point)
   - Used as the main navigation target for the crosser controller and as the preferred initial heading target in local simulation

5. **Approach waypoints** (optional):
   - Names `approach_1`, `approach_2`, … as Points; see the section below for launch and RTH behavior.

## Approach waypoints (entering the sailing area)

Use these when the boat should follow a **fixed path from `home` into the course** before normal crosser behavior (sailing toward `middle` and across the pond). They are optional; if none are present, the crosser goes straight to `middle`.

### Naming and order

- Add points named **`approach_1`**, **`approach_2`**, **`approach_3`**, … with consecutive integers (gaps are fine as long as names sort correctly by number: `approach_10` after `approach_9`).
- In My Maps, set the placemark **name** exactly to `approach_1`, etc. After KMZ → GeoJSON conversion, each feature must have:
  - `properties.name`: `approach_<number>`
  - `properties.type`: `waypoint`
- **Execution order** is by the numeric suffix: `approach_1` first, then `approach_2`, and so on.

### Behavior on the boat

- **Crosser (launch):** While autonomous, the controller enters a **launching** phase and steers toward each approach point in order. After the last one is reached (within the controller arrival distance), it continues in **toward_middle** toward the `middle` waypoint.
- **Return to home (RTH):** When returning, the controller steers **`approach_N` → … → `approach_2` → `approach_1`** (reverse of the outbound path), then navigates to the GPS **`home`** position and holds there when close enough.

### Workflow

1. In My Maps, place the approach points along the intended channel from beach/dock toward the sailing area, in the order the boat should follow.
2. Export KMZ into `maps/` and run `python3 scripts/kmz_to_geojson.py` so `foxglove/maps/<Your Map>.geojson` is updated.
3. Confirm in GeoJSON that each approach point is a `Point` with `type: waypoint` (the conversion script maps KML names into these properties).
4. Set `geofence_map_name` in `nodes/argo.yaml` to match your map file name.

## Notes

- The conversion from KMZ to GeoJSON is a **one-time process** - you only need to run it when maps change
- GeoJSON files are stored in `foxglove/maps/` and used by the simulation system
- The "home" waypoint coordinates determine where the boat starts in the simulation

