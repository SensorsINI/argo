# Testing Mast LED and Power Button Integration

This describes how to verify coordination between the **mast LED node** (`argo_mast_leds.py`) and the **power control node** (`argo_power_control.py`): the mast head LEDs mirror the power button LED state via ROS2 topics.

**Note:** The power button has **RGB only** (no white). The mast has **RGBW**. Mirroring updates only R, G, B on the mast; the white channel is not driven by the power button and stays off unless something else publishes to `/mastled_w` or `/mastled_rgbw`.

## Prerequisites

- **Power control** runs as a systemd service: `argo_power_control.service` (publishes `/argo/power_button/rgb` and `/argo/power_button/pattern`).
- **Mast LEDs** run as a regular ROS node (lifecycle manager or standalone): subscribes to those topics and drives the PCA9632 on the wind sensor PCB.

## Quick integration test (manual RGB)

1. Start Argo so both are running:
   - `al` (or start `argo_launch` service) so `mastleds_node` is running.
   - Ensure `argo_power_control.service` is running: `sudo systemctl status argo_power_control`.

2. Run the test script (publishes RGB to the same topic power_control uses; mast should mirror):
   ```bash
   source /opt/ros/humble/setup.bash
   bash /home/orangepi/argo/scripts/test_mast_led_power_button_integration.sh
   ```
   Watch the **mast head LEDs**: they should cycle red → green → blue → off (a few seconds each). White is not tested here because the power button has no white channel.

3. Or publish once manually:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic pub --once /argo/power_button/rgb geometry_msgs/Vector3 "{x: 1.0, y: 0.0, z: 0.0}"
   ```
   Mast should show **red**. Try `x: 0, y: 1, z: 0` for green, `x: 0, y: 0, z: 1` for blue.

## Testing pattern mirroring (heartbeat)

1. With Argo running (power_control service + mastleds node), **press and hold the power button** to start the heartbeat pattern on the power button LEDs.
2. Power control publishes `/argo/power_button/pattern` (JSON with category, pattern string, timing).
3. **Mast head LEDs** should show the same heartbeat pattern (e.g. green blinks) in sync.

## Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/argo/power_button/rgb` | `geometry_msgs/Vector3` | power_control | mastleds (RGB mirror) |
| `/argo/power_button/pattern` | `std_msgs/String` | power_control | mastleds (pattern mirror) |

## Troubleshooting

- **Mast LEDs don’t change when publishing RGB**
  - Check mastleds node is running: `ros2 node list` should show `mastleds_node`.
  - Check subscription: `ros2 topic info /argo/power_button/rgb`.
- **No pattern on mast when holding power button**
  - Ensure power_control service is running and publishing: `ros2 topic echo /argo/power_button/pattern`.
  - Mast node must be running and subscribed to `/argo/power_button/pattern`.
