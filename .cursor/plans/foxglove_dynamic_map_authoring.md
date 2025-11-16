# Foxglove-authored Dynamic Map & Waypoints – Engineering Plan

## Goal
Enable operators to add waypoints, geofences, and obstacle polygons from Foxglove during simulation or while the physical boat is running. These authored elements become live parts of the “local map,” persist to GeoJSON, and are visualized with durable markers. Controllers consume canonical state topics (not MarkerArray) for behavior.

## Principles
- Separation of concerns: Foxglove authoring → manager nodes own canonical state → visualization publishes durable markers → controllers subscribe to canonical state.
- Persistent storage: Append/overwrite to `~/argo/foxglove/maps/user_overrides.geojson`.
- QoS: Use TRANSIENT_LOCAL on visualization publishers so Foxglove gets the last known map immediately.
- Safety: Controller behavior only depends on canonical state topics; visualization markers are for display only.

## ROS Interfaces

### Canonical topics (manager publishes)
- `/argo/waypoints` – `nav_msgs/Path` (frame_id=`map`)
- `/argo/geofence` – array via repeated `geometry_msgs/PolygonStamped` (frame_id=`map`) or custom `GeofenceArray` if needed later

### Authoring topics (Foxglove publishes)
- `/argo/waypoints/add_local` – `geometry_msgs/PoseStamped` (frame_id=`map`)
- `/argo/waypoints/add_gps` – `sensor_msgs/NavSatFix` (hardware convenience)
- `/argo/geofence/add` – `geometry_msgs/PolygonStamped` (frame_id=`map`)

### Visualization topics (manager publishes, TRANSIENT_LOCAL)
- `/sailing_waypoints` – `visualization_msgs/MarkerArray` (green spheres)
- `/sailing_boundaries` – `visualization_msgs/MarkerArray` (blue line strips for polygons/boundaries)
- `/sailing_hazards` – `visualization_msgs/MarkerArray` (optional/red for obstacles; future)

### Services (manager provides)
- `/argo/waypoints/clear` – `std_srvs/Trigger`
- `/argo/waypoints/save` – `std_srvs/Trigger`
- `/argo/geofence/clear` – `std_srvs/Trigger`
- `/argo/geofence/save` – `std_srvs/Trigger`

## Nodes

### 1) `waypoint_manager.py`
- Subscriptions:
  - `/argo/waypoints/add_local` (`PoseStamped`)
  - `/argo/waypoints/add_gps` (`NavSatFix`) → convert to `PoseStamped` using map frame converter (below)
- Publications:
  - `/argo/waypoints` (`nav_msgs/Path`) – canonical state
  - `/sailing_waypoints` (`MarkerArray`, TRANSIENT_LOCAL) – visualization
- Services:
  - `/argo/waypoints/clear`, `/argo/waypoints/save`
- Persistence:
  - Save to `~/argo/foxglove/maps/user_overrides.geojson` (feature type `waypoint` for points; future extension to sequences/mission metadata)

### 2) `geofence_manager.py`
- Subscriptions:
  - `/argo/geofence/add` (`PolygonStamped`) – polygons in `map` frame
- Publications:
  - `/argo/geofence` (repeated `PolygonStamped`) – canonical state
  - `/sailing_boundaries` (`MarkerArray`, TRANSIENT_LOCAL) – visualization
- Services:
  - `/argo/geofence/clear`, `/argo/geofence/save`
- Persistence:
  - Save to `~/argo/foxglove/maps/user_overrides.geojson` (feature type `sailing_area` or `hazard` depending on UI selection; initial scope: `sailing_area`)

### 3) `map_frame_converter.py` (SIM/HW parity)
- Purpose: Convert `sensor_msgs/NavSatFix` to `geometry_msgs/PoseStamped` in `map` frame and back if needed.
- Inputs: `NavSatFix`, base lat/lon from current selected map (your simulator bridge already loads “home”)
- Output: `PoseStamped` in `map` frame to unify downstream consumption
- Notes:
  - Reuse the base lat/lon logic from `nodes/argo_unified_simulator_bridge.py` for consistent geodesy.
  - For hardware, source ROS2 env for any sudo runs (see Sudo ROS2 Command Environment rule).

### 4) (Optional) `foxglove_annotations_bridge.py`
- Translate `foxglove_msgs/Annotations` clicks/polylines into the authoring topics above.
- This enables drop-to-add directly in the Foxglove Annotations panel.

## Controller Integration
- Subscribe to `/argo/waypoints` (`nav_msgs/Path`). If non-empty, controller steers to first pose; on arrival within a radius threshold, pop and continue.
- Subscribe to `/argo/geofence` to apply soft/hard constraints in planning or heading selection (phase 2).
- Keep existing controller algorithms intact; add a light “mission follower” wrapper using the canonical topics.

## Persistence Format (user_overrides.geojson)
```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": { "type": "waypoint", "name": "wp1" },
      "geometry": { "type": "Point", "coordinates": [x, y, 0] }
    },
    {
      "type": "Feature",
      "properties": { "type": "sailing_area", "name": "geofence_1" },
      "geometry": { "type": "Polygon", "coordinates": [[[x,y,0], ...]] }
    }
  ]
}
```
Notes:
- Initial scope stores local-map-frame coordinates (x,y meters). If needed, include CRS metadata or convert to WGS84 on save.
- Managers re-load this file on startup and republish canonical + visualization topics.

## QoS
- Visualization publishers (`MarkerArray`): `TRANSIENT_LOCAL`, `RELIABLE`, `KEEP_LAST depth=10` – ensures Foxglove late subscribers see state.
- Canonical state topics: default reliable QoS is fine; consider latched state via TRANSIENT_LOCAL if consumers may come/go (optional).

## Operations & Commands

### Start managers (simulation/dev)
```bash
# Clean pycache before testing (Python Testing Environment Rules)
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true && find . -name "*.pyc" -delete 2>/dev/null || true

# Start managers
python3 nodes/waypoint_manager.py &
python3 nodes/geofence_manager.py &

# Optional: Annotations bridge
python3 nodes/foxglove_annotations_bridge.py &
```

### Add from Foxglove Publish panel
- Waypoint (local frame): publish `geometry_msgs/PoseStamped` to `/argo/waypoints/add_local` (header.frame_id=`map`)
- Geofence: publish `geometry_msgs/PolygonStamped` to `/argo/geofence/add` (header.frame_id=`map`)

### Save & clear
```bash
timeout 5 ros2 service call /argo/waypoints/save std_srvs/srv/Trigger
timeout 5 ros2 service call /argo/waypoints/clear std_srvs/srv/Trigger
timeout 5 ros2 service call /argo/geofence/save std_srvs/srv/Trigger
timeout 5 ros2 service call /argo/geofence/clear std_srvs/srv/Trigger
```

### Hardware (sudo + ROS2 env)
```bash
sudo bash -c 'source /opt/ros/humble/setup.bash && python3 /home/tobi/argo/nodes/waypoint_manager.py'  # if needed
```

## Incremental Roadmap
1) Interfaces finalized (topics/services/QoS) and doc updates
2) `waypoint_manager.py` (local-frame authoring) + durable visualization + save/load
3) `geofence_manager.py` (polygons) + durable visualization + save/load
4) `map_frame_converter.py` (NavSatFix→PoseStamped) for hardware parity
5) Controller minimal integration (consume `/argo/waypoints`)
6) Optional Annotations bridge for point/polyline authoring
7) Testing + Foxglove layout update + README updates

## Test Plan
- Unit: Managers save/load JSON round-trip, topic emissions after reload.
- Integration (SIM):
  - Start sim and managers; add waypoints; verify `/argo/waypoints` and markers appear; save; restart; verify persistence.
  - Add geofence; verify markers and canonical topic; save/load.
- Integration (HW):
  - Publish `NavSatFix` authoring; confirm conversion and path emission.
  - Verify controller reacts to first waypoint and advances on arrival.
- Foxglove:
  - Ensure durable markers are visible on reconnect.
  - Validate Publish panel message schemas; optional Annotations bridge flow.

## Risks & Mitigations
- Frame consistency: Standardize on `map` for local-frame authoring; add clear conversion utilities for GPS.
- Controller coupling: Keep controller subscribed to canonical topics only; avoid consuming MarkerArray.
- Persistence ambiguity: Start with local-frame GeoJSON; add WGS84 conversion later if required.


