# Foxglove Joystick Panel Setup Guide

## Quick Setup Steps

1. **Install the Panel**
   - Open Foxglove Studio
   - Go to Profile menu → Extensions → Extension Marketplace
   - Search for "Joystick" and install

2. **Configure Publishing** (CRITICAL)
   - Open the Joystick panel configuration
   - In the **"Publish" section**:
     - **Publish Mode**: Toggle to **"On"** ⚠️ This is required!
     - **Pub Joy Topic**: Enter `/joy`

3. **Configure Data Source**
   - In the **"Data Source" section**:
     - **Data Source**: Select one of:
       - **"Interactive Display Mode"** - Click/drag on visual joystick (recommended)
       - **"Keyboard Mode"** - Use keyboard keys
       - **"Gamepad Mode"** - Use physical gamepad
     - **DO NOT** use "Subscribed Joy To..." - that's for monitoring only

4. **Verify Configuration**
   - The panel should now show interactive controls (not "waiting for first data")
   - Interacting with the controls should publish to `/joy` topic
   - The simulator will automatically receive and process the commands

## Troubleshooting

### "Waiting for first data" Message

**Cause**: The panel is set to subscribe mode instead of publish mode.

**Fix**:
1. Check **Publish Mode** is set to **"On"** (not "Off")
2. Check **Data Source** is NOT "Subscribed Joy To..."
3. Verify **Pub Joy Topic** is set to `/joy`

### "Tried to publish on topic /joy that has not been advertised before"

**Cause**: ROS2 requires topics to be advertised AND foxglove_bridge needs to see message examples to learn the schema.

**Fix**:
1. Ensure the simulator is running (`asim` command)
2. Wait 3-5 seconds for the simulator bridge to publish 3 schema messages, then it stops
3. Verify the topic exists:
   ```bash
   ros2 topic list | grep joy
   ros2 topic info /joy
   ```
4. **Important**: The simulator publishes 3 initial messages then stops. After that, Foxglove should be able to publish.
5. If still can't publish:
   - Check Data Source is NOT "Subscribed Joy To..." (that disables publish mode)
   - Data Source must be "Interactive Display Mode", "Keyboard Mode", or "Gamepad Mode"
   - If buttons are grayed out, the panel is in subscribe-only mode - change Data Source
   - Try refreshing the Foxglove connection
   - Wait a few more seconds after simulation starts

### Can't See /joy Topic Rate in Foxglove

**Cause**: The simulator only publishes 3 initial messages then stops. After that, the topic only has data when YOU publish from Foxglove.

**This is expected behavior**:
- The simulator publishes 3 messages at startup to teach foxglove_bridge the schema
- Then it stops publishing
- You should see messages when you move the joystick controls in Foxglove
- You can verify by watching: `ros2 topic echo /joy` - should see YOUR messages when you interact with the panel

### Controls Not Responding / Publish Mode Buttons Grayed Out

**Possible causes**:
1. **Data Source is in Subscribe Mode** - This DISABLES publish mode
   - **Fix**: Change Data Source from "Subscribed Joy To..." to:
     - "Interactive Display Mode" (recommended)
     - "Keyboard Mode" 
     - "Gamepad Mode"
2. **Publish Mode is Off** - Enable it in Publish section (only available if Data Source is correct)
3. **Wrong Data Source** - Must be Interactive/Keyboard/Gamepad mode (NOT Subscribe mode)
4. **Topic mismatch** - Verify Pub Joy Topic is `/joy`
5. **Topic not fully advertised** - Wait a few seconds after simulation starts

### Verify It's Working

From your ROS2 terminal (where simulation is running):

```bash
# Check if /joy topic exists and has a publisher
ros2 topic info /joy

# Watch messages being published (should update when you move controls)
timeout 10 ros2 topic echo /joy

# Check simulator is receiving
ros2 topic echo /rudder_sail_radio
```

## Configuration Reference

### Publish Section
- **Publish Mode**: Must be **"On"** for the panel to send commands
- **Pub Joy Topic**: `/joy` (this is what the simulator subscribes to)
- **Joy Frame ID**: Can be left empty

### Data Source Section
- **Data Source**: Choose input method
  - **Interactive Display Mode**: Click/touch the visual joystick display
  - **Keyboard Mode**: Map keyboard keys to joystick axes
  - **Gamepad Mode**: Use physical gamepad/controller
- **Subsc. Joy Topic**: Only needed if using Subscribe mode (not recommended for control)

### Display Section
- **Display Mode**: "Auto-Generated" works well
- **Theme/Mapping**: Can be customized as needed

## How It Works

1. You interact with the Joystick panel (mouse, keyboard, or gamepad)
2. Panel publishes `sensor_msgs/msg/Joy` messages to `/joy` topic
3. Simulator bridge (`argo_unified_simulator_bridge.py`) subscribes to `/joy`
4. Bridge converts Joy messages to rudder/sail control (Vector3)
5. Control is applied to the simulator

## Axis Mapping

Default mapping in the simulator bridge:
- **Axis 0** → Rudder (-1 = left, +1 = right)
- **Axis 1** → Sail (-1 = in, +1 = out, inverted)

You can customize this with ROS2 parameters when starting the simulator.

