# Patrol Controller Development Plan

## Overview

Create a new `PatrolController` that autonomously patrols a geofence sailing area using broad/beam reaches, executing tacks or jibes before reaching boundaries. The controller will read geofence data from GeoJSON files and implement intelligent boundary detection and sailing strategies.

## Architecture

### 1. New Controller Class: `PatrolController`

- Inherits from `BaseController` (following existing pattern)
- Implements `generate_control(state: BoatState) -> ControlCommand`
- Selectable via `controller_type: "patrol"` parameter

### 2. Geofence Management Module

- **File**: `nodes/support/geofence_manager.py` (new utility module)
- **Responsibilities**:
  - Load GeoJSON files from `foxglove/maps/` directory
  - Extract sailing area polygons (type: "sailing_area")
  - Convert lon/lat coordinates to local x/y meters (reuse existing conversion logic)
  - Implement point-in-polygon algorithm (ray casting)
  - Calculate distance to nearest boundary edge
  - Predict future position and check if it will cross boundary

### 3. Patrol Strategy Logic

- **Sailing Modes**:
  - **Broad Reach**: Sail at ~90-135° to wind (beam to broad reach)
  - **Boundary Approach**: Detect when approaching edge and prepare for turn
  - **Tack/Jibe Execution**: Turn before reaching boundary
  - **Upwind Tack**: When on downwind side, tack upwind to cross area

### 4. Edge Prediction Algorithm

- Predict boat position N seconds ahead using:
  - Current position (GPS lat/lon)
  - Current heading (compass)
  - Current speed (GPS SOG converted to m/s)
- Check predicted position against geofence boundary
- Trigger turn maneuver when predicted position is within threshold of boundary

## Implementation Steps

### Step 1: Create Geofence Manager Utility ✅

**File**: `nodes/support/geofence_manager.py`

**Features**:

- `GeofenceManager` class
- `load_geofence(map_name: str) -> Polygon` - Load and parse GeoJSON
- `lonlat_to_xy(lon, lat, origin_lon, origin_lat) -> (x, y)` - Coordinate conversion
- `is_point_inside_polygon(x, y, polygon) -> bool` - Point-in-polygon check
- `distance_to_boundary(x, y, polygon) -> float` - Distance to nearest edge
- `predict_future_position(lat, lon, heading_deg, speed_ms, time_sec) -> (lat, lon)` - Position prediction

**Coordinate System**:

- Reuse conversion logic from `sailing_area_publisher.py` (equirectangular projection)
- Use "home" waypoint as origin (same as simulator bridge)

### Step 2: Extend BoatState ✅

**File**: `nodes/controller.py`

**Additions to `BoatState` dataclass**:

- `geofence_polygon: Optional[List[Tuple[float, float]]] = None` - Cached polygon in local coordinates
- `distance_to_boundary: Optional[float] = None` - Current distance to nearest edge
- `predicted_boundary_crossing_time: Optional[float] = None` - Seconds until boundary crossing

### Step 3: Implement PatrolController ✅

**File**: `nodes/controller.py`

**Class Structure**:

```python
class PatrolController(BaseController):
    def __init__(self, config):
        # Initialize geofence manager
        # Load geofence polygon
        # Set patrol parameters (tack threshold, lookahead time, etc.)
    
    def generate_control(self, state: BoatState) -> ControlCommand:
        # 1. Check if inside geofence
        # 2. Predict future position
        # 3. Determine sailing strategy (broad reach, tack, jibe)
        # 4. Generate rudder and sail commands
```

**Patrol Logic**:

- **Normal Patrol**: Sail broad/beam reach (90-135° to wind)
- **Approaching Boundary**: When predicted position within threshold (e.g., 10-20m), prepare turn
- **Boundary Turn**: Execute tack or jibe based on:
  - Wind direction relative to boundary
  - Which side of area (upwind vs downwind)
  - Current heading
- **Upwind Recovery**: When on downwind edge, tack upwind to cross back

**Parameters**:

- `patrol_lookahead_time`: Seconds to predict ahead (default: 15.0)
- `boundary_turn_threshold`: Distance to boundary to trigger turn (default: 15.0 meters)
- `tack_angle`: Degrees to turn during tack (default: 90.0)
- `broad_reach_angle`: Preferred angle to wind for patrol (default: 110.0)

### Step 4: Integrate into Controller Node ✅

**File**: `nodes/controller.py`

**Modifications**:

- Add `'patrol'` case in `_initialize_controller()`
- Add patrol parameters to parameter declarations
- Load geofence at controller initialization
- Update `BoatState` with geofence data in GPS callback

**Geofence Loading**:

- Read map name from parameter or config file
- Load geofence polygon at startup
- Convert to local coordinates and cache in controller

### Step 5: Update Configuration ✅

**File**: `nodes/argo.yaml`

**Additions**:

```yaml
controller_node:
  ros__parameters:
    controller_type: "patrol"  # New option
    patrol_lookahead_time: 15.0
    boundary_turn_threshold: 15.0
    tack_angle: 90.0
    broad_reach_angle: 110.0
    geofence_map_name: "Argo Irchel pond sailing area"  # Or from config
```

### Step 6: Testing Strategy

**Phase 1: Unit Tests**

- Test point-in-polygon algorithm with known polygons
- Test coordinate conversion accuracy
- Test edge distance calculations
- Test position prediction

**Phase 2: Mock Simulator Testing** ✅

- ✅ Test with `mock_sailboat_simulator.py`
- ✅ Verify geofence loading
- ✅ Test boundary detection
- ✅ Test tack/jibe execution logic
- ✅ Integration tests with actual map files

**Phase 3: Sailboat-Playground Testing**

- Test with realistic physics
- Verify sailing behavior (broad reaches)
- Test boundary avoidance
- Test upwind tacking

**Phase 4: Water Testing**

- Deploy to Argo hardware
- Test in real sailing conditions
- Validate geofence accuracy
- Tune parameters

## Key Algorithms

### Point-in-Polygon (Ray Casting)

```python
def is_point_inside_polygon(x, y, polygon):
    # Ray casting algorithm
    # Count intersections of horizontal ray from point
    # Odd = inside, Even = outside
```

### Boundary Distance

```python
def distance_to_boundary(x, y, polygon):
    # Find minimum distance to any polygon edge
    # Use perpendicular distance to line segments
```

### Position Prediction

```python
def predict_position(lat, lon, heading_deg, speed_ms, time_sec):
    # Convert heading to radians
    # Calculate distance = speed * time
    # Project lat/lon using haversine or simple approximation
    # Return predicted (lat, lon)
```

## Files Created/Modified

**New Files**:

- `nodes/support/geofence_manager.py` - Geofence utilities ✅

**Modified Files**:

- `nodes/controller.py` - Add PatrolController class, integration, and captain's log system ✅
- `nodes/argo.yaml` - Add patrol controller parameters ✅

## Implementation Status

### Completed ✅

1. ✅ Geofence Manager Module (`nodes/support/geofence_manager.py`)

   - Point-in-polygon algorithm (ray casting)
   - Distance to boundary calculation
   - Future position prediction
   - Coordinate conversion (lon/lat ↔ x/y meters)
   - GeoJSON loading and parsing

2. ✅ BoatState Extensions (`nodes/controller.py`)

   - Added `geofence_polygon` field
   - Added `distance_to_boundary` field
   - Added `predicted_boundary_crossing_time` field

3. ✅ PatrolController Class (`nodes/controller.py`)

   - Inherits from `BaseController`
   - Broad reach sailing mode (90-135° to wind)
   - Boundary detection and prediction
   - Tack/jibe execution logic
   - Wind-aware sail control
   - Configurable parameters

4. ✅ Controller Integration (`nodes/controller.py`)

   - Added 'patrol' case in `_initialize_controller()`
   - Added patrol parameters to parameter declarations
   - Updated parameter validation to include 'patrol'
   - Integrated geofence loading

5. ✅ Configuration (`nodes/argo.yaml`)

   - Added patrol controller parameters
   - Added example configuration
   - Updated controller_type options documentation

6. ✅ Captain's Log System (`nodes/controller.py`)

   - **BaseController Publishing Infrastructure**:
     - Added `parent_node` reference to all controllers for publishing through ControllerNode
     - Lazy publisher creation (only when needed)
     - No overhead when not used
   
   - **Logging Methods in BaseController**:
     - `log_entry(message, level="INFO")` - Publishes formatted log entries to `/controller/captains_log`
     - `publish_state(state_name)` - Publishes controller state to `/controller/state` for Foxglove visualization
     - `log_periodic_state(state, interval_sec=30.0)` - Logs periodic boat/environmental state
   
   - **Controller State Publishing**:
     - All controllers publish their state (e.g., "proportional", "wind_aware", "return_to_home", "broad_reach", "tacking")
     - State changes published immediately for real-time visualization
   
   - **PatrolController Logging**:
     - Geofence violations logged as WARN
     - Boundary approach warnings logged as INFO
     - Mode changes (broad_reach → tacking) logged with context
     - Periodic state logging every 30 seconds (heading, speed, wind, position)
     - Geofence loading confirmation
   
   - **ReturnToHomeController Logging**:
     - Periodic updates every 60 seconds with distance, bearing, and heading
     - Arrival notifications
   
   - **Topics Published**:
     - `/controller/captains_log` (String) - Formatted log entries: `"[LEVEL] message"`
     - `/controller/state` (String) - Controller state for visualization: `"broad_reach"`, `"tacking"`, etc.
   
   - **Usage**:
     - Can be visualized in Foxglove (Text panels)
     - Recorded in bag files for post-analysis
     - Example log entries:
       - `[INFO] Geofence loaded: Argo Irchel pond sailing area`
       - `[INFO] Executing tack (15.2m from boundary, starboard tack)`
       - `[WARN] Geofence violation: 2.3m outside boundary`
       - `[INFO] State: heading=145.2°, speed=2.1kt, wind=3.5m/s @ 45.0°`

### Pending

- Mock simulator testing
- Sailboat-playground simulator testing
- Water testing

## Usage

To activate the patrol controller, set in `nodes/argo.yaml`:

```yaml
controller_node:
  ros__parameters:
    controller_type: "patrol"
    patrol_lookahead_time: 15.0
    boundary_turn_threshold: 15.0
    tack_angle: 90.0
    broad_reach_angle: 110.0
    geofence_map_name: "Argo Irchel pond sailing area"
```

## Key Features

- **Geofence Loading**: Reads GeoJSON files from `foxglove/maps/` directory
- **Boundary Detection**: Calculates distance to boundary and predicts crossing time
- **Sailing Strategy**: Uses broad reaches for efficient patrol, tacks when approaching boundaries
- **Wind Awareness**: Considers wind direction for sail control and tack/jibe decisions
- **Configurable**: All patrol parameters are configurable via argo.yaml

## Dependencies

- Existing: `sailing_area_publisher.py` coordinate conversion logic
- Existing: GeoJSON file structure in `foxglove/maps/`
- New: Point-in-polygon algorithm implementation
- New: Boundary distance calculation

## Future Enhancements (Out of Scope)

- Station-keeping mode (separate controller)
- Waypoint navigation
- Dynamic geofence updates from topics
- Multi-polygon support (hazards, multiple areas)