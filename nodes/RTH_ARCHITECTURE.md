# Return-to-Home System Architecture

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           SHORE SIDE                                    │
│  ┌──────────────┐                                                       │
│  │  Operator    │  Command: "return_home"                               │
│  │  (Foxglove)  │────────────────────┐                                  │
│  └──────────────┘                    │                                  │
│                                      ▼                                   │
│                              ┌──────────────┐                            │
│                              │ lora_shore.py│                            │
│                              │ (USB LoRa)   │                            │
│                              └──────┬───────┘                            │
└─────────────────────────────────────┼──────────────────────────────────┘
                                      │
                          LoRa Radio (433 MHz)
                                      │
┌─────────────────────────────────────┼──────────────────────────────────┐
│                          ARGO BOAT  │                                   │
│                              ┌──────▼───────┐                            │
│                              │   lora.py    │                            │
│                              │  (SPI/GPIO)  │                            │
│                              └──────┬───────┘                            │
│                                     │                                    │
│              ┌──────────────────────┼──────────────────────┐             │
│              │                      │                      │             │
│              ▼                      ▼                      ▼             │
│  ┌────────────────────┐  ┌──────────────────┐  ┌──────────────────┐    │
│  │/lora_connection_   │  │/lora_remote_     │  │/lora_signal_     │    │
│  │ status (Bool)      │  │ command (String) │  │ strength (Int32) │    │
│  └─────────┬──────────┘  └─────────┬────────┘  └──────────────────┘    │
│            │                       │                                    │
│            │         ┌─────────────┘                                    │
│            │         │                                                  │
│            │         │        ┌────────────┐                            │
│            │         │        │  gps.py    │                            │
│            │         │        │  (UART5)   │                            │
│            │         │        └─────┬──────┘                            │
│            │         │              │                                   │
│            │         │              ▼                                   │
│            │         │    ┌──────────────────┐                          │
│            │         │    │ /fix (NavSatFix) │                          │
│            │         │    │ - latitude       │                          │
│            │         │    │ - longitude      │                          │
│            │         │    └─────┬────────────┘                          │
│            │         │          │                                       │
│            ▼         ▼          ▼                                       │
│  ┌───────────────────────────────────────┐                              │
│  │         controller.py                 │                              │
│  │  ┌─────────────────────────────────┐  │                              │
│  │  │      BoatState                  │  │                              │
│  │  │  - shore_connected              │  │                              │
│  │  │  - last_shore_contact           │  │                              │
│  │  │  - remote_command               │  │                              │
│  │  │  - home_latitude/longitude      │  │                              │
│  │  │  - current_latitude/longitude   │  │                              │
│  │  │  - return_to_home_active        │  │                              │
│  │  └─────────────────────────────────┘  │                              │
│  │              │                         │                              │
│  │              ▼                         │                              │
│  │  ┌─────────────────────────────────┐  │                              │
│  │  │  RTH Activation Logic           │  │                              │
│  │  │  if shore_disconnected > 120s   │  │                              │
│  │  │  OR remote_command == 'rth'     │  │                              │
│  │  └──────────┬──────────────────────┘  │                              │
│  │             │                          │                              │
│  │             ▼                          │                              │
│  │  ┌─────────────────────────────────┐  │                              │
│  │  │ ReturnToHomeController          │  │                              │
│  │  │  1. bearing = get_bearing()     │  │                              │
│  │  │  2. distance = get_distance()   │  │                              │
│  │  │  3. target_heading = bearing    │  │                              │
│  │  │  4. generate_control()          │  │                              │
│  │  └──────────┬──────────────────────┘  │                              │
│  └─────────────┼──────────────────────────┘                              │
│                │                                                         │
│                ▼                                                         │
│  ┌────────────────────────────┐                                         │
│  │ /rudder_sail_cmd (Vector3) │                                         │
│  └─────────────┬──────────────┘                                         │
│                │                                                         │
│                ▼                                                         │
│  ┌──────────────────────────────┐                                       │
│  │  rudder_sail_radio.py        │                                       │
│  │  (Control Arbitration)       │                                       │
│  └─────────────┬────────────────┘                                       │
│                │                                                         │
│                ▼                                                         │
│  ┌──────────────────────────────┐                                       │
│  │  Servos (Rudder + Sail)      │                                       │
│  └──────────────────────────────┘                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

## State Machine

```
┌─────────────┐
│   STARTUP   │
│  (No GPS)   │
└──────┬──────┘
       │ First GPS fix
       ▼
┌─────────────┐
│  NORMAL     │◄───────────────────┐
│  (Human)    │                    │
└──────┬──────┘                    │
       │ Human releases control   │
       ▼                          │
┌─────────────┐                   │
│  AUTONOMOUS │                   │
│  (Standard) │                   │
└──────┬──────┘                   │
       │                          │
       │ Connection lost > 120s   │
       │ OR 'return_home' cmd     │
       ▼                          │
┌─────────────┐                   │
│   RETURN    │                   │
│   TO HOME   │                   │
│   (RTH)     │                   │
└──────┬──────┘                   │
       │                          │
       │ Arrived at home          │
       │ OR 'autonomous' cmd      │
       └──────────────────────────┘
```

## Controller Selection Logic

```python
def timer_callback():
    if human_controlled:
        # Human has control - update target heading
        target_heading = current_heading
        
    else:
        # Autonomous mode
        
        # Check if RTH should activate
        if isinstance(controller, ReturnToHomeController):
            if controller.should_activate(boat_state):
                # Already in RTH mode, continue
                pass
        else:
            # Check if we need to switch to RTH
            temp_rth = ReturnToHomeController(config)
            if temp_rth.should_activate(boat_state):
                # Connection lost! Switch to RTH
                controller = temp_rth
        
        # Generate control commands
        control_cmd = controller.generate_control(boat_state)
        publish(control_cmd)
```

## RTH Navigation Algorithm

```python
class ReturnToHomeController:
    def generate_control(self, state):
        # 1. Calculate navigation parameters
        bearing_to_home = state.get_bearing_to_home()  # 0-360 degrees
        distance_to_home = state.get_distance_to_home()  # nautical miles
        
        # 2. Check arrival
        if distance_to_home < 0.05:  # 90 meters
            return ControlCommand(rudder=0.0, sail=0.0)  # Arrived!
        
        # 3. Set target heading toward home
        state.target_heading = bearing_to_home
        
        # 4. Calculate rudder command (proportional control)
        heading_error = bearing_to_home - state.compass_heading
        rudder = rudder_gain * (heading_error / rudder_full_scale)
        
        # 5. Calculate sail command (wind-aware)
        if wind_angle is not None:
            sail = wind_based_sail_position()
        
        return ControlCommand(rudder, sail)
```

## GPS Home Position Logic

```
GPS Fix Received (/fix topic)
       │
       ▼
   Is home position set?
       │
       ├─ NO ──► Set home_latitude = current_latitude
       │         Set home_longitude = current_longitude
       │         Log: "🏠 Home position set: lat, lon"
       │
       └─ YES ─► Update current_latitude = GPS latitude
                 Update current_longitude = GPS longitude
                 
Every control loop:
    - Calculate bearing_to_home using Haversine
    - Calculate distance_to_home using great circle
    - Use bearing as target_heading if RTH active
```

## LoRa Command Processing

```
Shore sends command ────► LoRa Radio ────► lora.py
                                              │
                                              ▼
                                    /lora_remote_command
                                              │
                                              ▼
                                    controller.py
                                              │
                                              ▼
                          ┌───────────────────┴───────────────────┐
                          │                                       │
                    "return_home"                          "autonomous"
                          │                                       │
                          ▼                                       ▼
            Set return_to_home_active = True      Set return_to_home_active = False
            Log bearing and distance to home      Resume normal autonomous operation
```

## Typical Operation Sequence

1. **System Start**
   - Controller starts in human control mode
   - GPS acquires fix
   - Home position set automatically
   - LoRa connection established

2. **Autonomous Operation**
   - Human releases control via radio
   - Controller uses `proportional` or `wind_aware` mode
   - LoRa sends periodic status to shore

3. **Connection Loss**
   - LoRa connection times out (120 seconds)
   - Controller detects: `shore_connected = False`
   - Automatic switch to `ReturnToHomeController`
   - Boat begins navigating toward home

4. **RTH Navigation**
   - Calculate bearing to home: 270° (West)
   - Calculate distance: 2.5 nm
   - Update heading to 270°
   - Adjust sail based on wind
   - Log progress every 10 seconds

5. **Arrival**
   - Distance < 0.05 nm (90 meters)
   - Set rudder and sail to neutral (0.0)
   - Boat drifts near starting position

6. **Connection Restored**
   - LoRa reconnects
   - Shore sends `"autonomous"` command
   - Controller switches back to normal mode
   - Resume mission

## Key Design Decisions

### Why GPS-based navigation?
- **Precise**: GPS provides accurate position data
- **Independent**: Works without shore communication
- **Standard**: Uses existing `/fix` topic from gps.py

### Why automatic home position?
- **Safe**: Always has a return point
- **Simple**: No manual configuration required
- **Persistent**: Set once at startup, used throughout mission

### Why 120-second timeout?
- **Tolerant**: Allows temporary connection loss
- **Safe**: Long enough to avoid false triggers
- **Configurable**: Can be adjusted in parameters

### Why 90-meter arrival radius?
- **GPS Accuracy**: Typical GPS error ~5-10 meters
- **Safe**: Large enough to reliably detect arrival
- **Small**: Close enough to original position

## Monitoring and Debugging

### Check RTH Status
```bash
# Monitor controller logs
ros2 topic echo /rudder_sail_cmd

# Check GPS position
ros2 topic echo /fix --field latitude,longitude

# Monitor LoRa connectivity
ros2 topic echo /lora_connection_status

# Watch distance to home (requires custom subscriber)
ros2 topic echo /control_authority
```

### Simulate RTH
```bash
# Force connection loss
ros2 topic pub /lora_connection_status std_msgs/Bool "data: false"

# Manual RTH trigger
ros2 topic pub --once /lora_remote_command std_msgs/String "data: 'return_home'"

# Check controller type
ros2 param get /controller_node controller_type
```


