# Visualization Marker Persistence

## Problem

When a simulation terminates, all visualization markers disappear from Foxglove/RViz because the publishing node shuts down. This makes post-simulation analysis difficult.

## Solutions

### ✅ Recommended: Use Bag Recording and Playback

The **best solution** is to record your simulation and play it back:

```bash
# During simulation
ar    # Start recording

# After simulation ends
apb   # Playback latest recording with visualization
```

**How it works:**
1. Recording captures all sensor data (GPS, heading, IMU, etc.)
2. Playback launches the visualization node which recreates markers from sensor data
3. You get full visualization even after the original simulation ended
4. Bonus: You can adjust visualization settings during playback!

**Why this is better than recording markers:**
- Sensor data is much smaller than full marker messages (~100x less bandwidth)
- Markers can be recreated with different settings during playback
- You can toggle visualization on/off during playback

### ⚡ Quick Method: Keyboard Control Pause

If you're using keyboard control, just press **SPACE** to freeze the simulation:

```bash
# Start keyboard control
asimkb

# While simulation is running
SPACE     # Freeze simulation (SIGSTOP) - markers stay visible!

# Inspect markers in Foxglove at your leisure
# All simulation processes are frozen, so markers persist

SPACE     # Resume simulation (SIGCONT)
```

**How it works:**
- Sends SIGSTOP to all simulation processes (simulator, visualization, controller, etc.)
- Processes are frozen but alive, so ROS nodes keep markers published
- Markers remain visible in Foxglove for inspection
- Press SPACE again to resume with SIGCONT

**Pros:** Instant, built-in, easy to remember  
**Cons:** Can't replay or share, only works during active simulation

### ⚡ Manual Workaround: Ctrl+Z Pause

If you're not using keyboard control:

```bash
# While simulation is running
Ctrl+Z    # Pause (SIGSTOP) the entire terminal session

# Inspect markers in Foxglove at your leisure
# They stay visible because the node is paused, not terminated

fg        # Resume when done (or kill with Ctrl+C)
```

**Pros:** Instant, no extra tools needed  
**Cons:** Pauses the entire terminal session, awkward to remember

### 📊 Foxglove Recording Feature

Foxglove has built-in recording that captures everything visible:

1. Click the record button in Foxglove
2. Run your simulation
3. Stop recording
4. Playback the Foxglove recording

**Pros:** Easy to use, captures exactly what you see  
**Cons:** Larger files, requires Foxglove running during sim

## Marker Trail Efficiency

The heading trail uses **incremental publishing** - only new markers are sent, not the entire trail every frame. This reduces network bandwidth by ~500x:

- **Before optimization**: ~200 KB/s for trail markers
- **After optimization**: ~0.4 KB/s for trail markers

This is especially important for the boat's limited WiFi bandwidth.

## Color-Coded Trail States

The heading trail shows different colors based on control mode and controller state:

| Color | State |
|-------|-------|
| 🔴 Red | Human control |
| 🟢 Green | Autonomous tacking |
| 🟠 Orange | Autonomous jibing |
| 🟣 Purple | Return to home |
| 🔵 Cyan | Normal sailing (broad reach, proportional, wind aware) |

Text labels appear at state transitions to identify each segment.

## See Also

- `argo_help` - View all Argo commands including recording/playback
- `scripts/argo_playback_latest.sh` - Playback script
- `nodes/argo_boat_visualization.py` - Visualization node implementation

