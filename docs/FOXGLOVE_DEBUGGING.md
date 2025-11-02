# Foxglove Studio Debugging Guide

## Viewing Detailed Error Messages in Foxglove Studio (Windows 11)

**Note**: Foxglove Studio may not provide easily accessible detailed logs. The best approach is to diagnose issues from the ROS2 side using command-line tools.

### Recommended: Use ROS2 Diagnostic Tools

Instead of trying to find Foxglove logs, diagnose issues using ROS2 commands:

#### Quick Diagnosis Script
```bash
# Run the diagnostic script
./scripts/diagnose_topic_schema.sh
```

This will show:
- Current topic types and any conflicts
- All publishers and subscribers
- Specific schema mismatch details

#### Manual Diagnosis
```bash
# Check topic types
ros2 topic info /rudder_sail_radio --verbose

# Check what nodes are publishing
ros2 node info /foxglove_bridge

# Check topic type
ros2 topic type /rudder_sail_radio

# Echo topic to see actual messages
ros2 topic echo /rudder_sail_radio
```

### Alternative: Browser Developer Tools (Web Version)

If using Foxglove Studio Web (browser version):
1. Press `F12` to open Developer Tools
2. Check Console tab for errors
3. Check Network tab for websocket issues

### Desktop App: Limited Logging Options

The Foxglove Studio desktop app does not expose detailed logs easily. The schema conflict errors are usually visible in:
- The main error message in the UI (often limited details)
- ROS2 topic diagnostics (recommended approach above)

## Common Schema Conflicts

### Error: "multiple channels advertise the same topic but the schema, name, or encoding do not match"

This happens when multiple publishers use different message types for the same topic.

### Diagnosing Schema Conflicts

**When simulation is running**, use the diagnostic script:

```bash
./scripts/diagnose_topic_schema.sh
```

This will show exactly what's wrong:
- Which nodes are publishing
- What message types they're using
- Specific conflicts

### Solution for `/rudder_sail_radio` Topic

The `/rudder_sail_radio` topic expects `geometry_msgs/msg/Vector3`.

**The Problem:**
- `foxglove_bridge` may be advertising it will publish `Twist` messages (from a misconfigured panel)
- The simulator bridge subscribes to `Vector3` messages
- This creates a schema conflict

**Solution: Use Joystick Panel Instead**

1. **Don't use Teleop panel** for `/rudder_sail_radio` - it causes conflicts
2. **Use Joystick panel** - publish to `/joy` topic instead
3. The simulator bridge automatically converts `/joy` to control commands

**If you must use `/rudder_sail_radio`:**

1. In Foxglove Studio, check all panels that might publish:
   - Remove or disable any Teleop panels
   - Remove or disable any panels publishing to `/rudder_sail_radio`
   - Ensure only Vector3 type is used
2. Configure Teleop panel (if using):
   - Set **Topic** to: `/rudder_sail_radio`
   - Set **Message Type** to: `geometry_msgs/msg/Vector3` (NOT Twist)
   - Configure mappings:
     - **X-axis** (rudder): `x` field (-1 to +1)
     - **Y-axis** (sail): `y` field (-1 to +1)
     - **Z-axis**: Leave as `z` field (should be 0.0)

**Alternative: Check Current Topic Types**
```bash
# On the ROS2 system (Linux/WSL)
ros2 topic info /rudder_sail_radio --verbose
```

This will show all publishers and subscribers with their message types.

## Verifying Topic Schema

### Check Topic Type
```bash
ros2 topic type /rudder_sail_radio
# Should output: geometry_msgs/msg/Vector3
```

### Check All Publishers
```bash
ros2 topic info /rudder_sail_radio --verbose
# Look for Publisher entries and their Topic type
```

### Test Publishing Correct Type
```bash
ros2 topic pub /rudder_sail_radio geometry_msgs/msg/Vector3 "{x: 0.5, y: -0.3, z: 0.0}"
```

### Echo Topic to Verify
```bash
ros2 topic echo /rudder_sail_radio
# Should show Vector3 messages with x, y, z fields
```

## Debugging Steps

1. **Check active publishers:**
   ```bash
   ros2 topic info /rudder_sail_radio --verbose
   ```

2. **Verify simulator bridge is subscribing:**
   ```bash
   ros2 node info /argo_unified_simulator_bridge | grep -A 5 "Subscribers"
   ```

3. **Check if foxglove_bridge is publishing incorrectly:**
   - The foxglove_bridge shouldn't publish to topics
   - It only forwards messages between ROS2 and websocket
   - If it's publishing, there may be a configuration issue

4. **Restart nodes:**
   ```bash
   # Stop simulation
   aq
   
   # Start simulation
   asim
   ```

5. **Check Foxglove connection:**
   - Verify websocket URL: `ws://localhost:8765`
   - Check connection status in Foxglove
   - Look for connection errors in Developer Console

## Expected Topic Configuration

For `/rudder_sail_radio`:
- **Type**: `geometry_msgs/msg/Vector3`
- **Fields**: 
  - `x` (float64): Rudder position (-1.0 to +1.0)
  - `y` (float64): Sail position (-1.0 to +1.0)
  - `z` (float64): Reserved (should be 0.0)
- **Publisher**: Foxglove Teleop panel (or terminal command)
- **Subscriber**: `argo_unified_simulator_bridge`

## Troubleshooting

### Issue: Foxglove shows Twist instead of Vector3
**Solution**: Configure Teleop panel to use Vector3 message type, not Twist.

### Issue: Multiple publishers with different types
**Solution**: Ensure all tools publishing to the topic use the same message type.

### Issue: Topic not appearing in Foxglove
**Solution**: 
1. Verify topic exists: `ros2 topic list | grep rudder_sail_radio`
2. Check foxglove_bridge is running: `ros2 node list | grep foxglove`
3. Restart foxglove_bridge if needed

