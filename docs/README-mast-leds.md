# Mast LED Controller (RGBW)

The mast head has four LEDs (Red, Green, Blue, White) driven by an NXP **PCA9632** I2C LED driver on the wind sensor PCB. The **mast LED node** (`argo_mast_leds.py`) is a ROS2 node that subscribes to topics and drives the hardware; it also mirrors the power button RGB state so the mast shows the same status as the button.

## Hardware

- **Device:** PCA9632 (4-channel I2C LED driver, NXP)
- **Bus:** I2C bus 2 (same as wind sensor, pins 27/28)
- **Addresses:** Individual **0x62** (used by the node); **0x70** is LED All Call (broadcast, visible at power-up)
- **Channels:** LED0 = Red, LED1 = Green, LED2 = Blue, LED3 = White (RGBW)
- **Verify:** `i2cdetect -y 2` should show `62` and `70`

See [docs/README-i2c.md](README-i2c.md) for the I2C address table.

## Node

- **Executable:** `nodes/argo_mast_leds.py`
- **ROS2 node name:** `mastleds_node`
- **Lifecycle:** Started with the rest of Argo via the lifecycle manager (`al`); defined in `launch/argo_nodes.yaml` in the `physical_robot` group.

## Topics (subscribed)

Any node can control the mast LEDs by publishing to these topics.

| Topic | Type | Description |
|-------|------|-------------|
| `/mastled_r` | `std_msgs/Float32` | Red brightness 0.0–1.0 |
| `/mastled_g` | `std_msgs/Float32` | Green brightness 0.0–1.0 |
| `/mastled_b` | `std_msgs/Float32` | Blue brightness 0.0–1.0 |
| `/mastled_w` | `std_msgs/Float32` | **White** brightness 0.0–1.0 |
| `/mastled_rgbw` | `std_msgs/Float32MultiArray` | All four: `data: [R, G, B, W]` (each 0.0–1.0) |
| `/argo/power_button/rgb` | `geometry_msgs/Vector3` | Power button mirror: x=R, y=G, z=B (RGB only) |
| `/argo/power_button/pattern` | `std_msgs/String` | Power button pattern mirror (e.g. heartbeat) |

**Controlling white:** The power button has RGB only; it does not drive white. To set white, publish to `/mastled_w` or include the fourth value in `/mastled_rgbw` (e.g. `data: [0.0, 0.0, 0.0, 1.0]` for white only).

**Examples:**
```bash
# White full on
ros2 topic pub --once /mastled_w std_msgs/Float32 "{data: 1.0}"

# All channels: R=0, G=0, B=0, W=1
ros2 topic pub --once /mastled_rgbw std_msgs/Float32MultiArray "{data: [0.0, 0.0, 0.0, 1.0]}"
```

## Running

- **With Argo:** `al` (or start `argo_launch` service) — mastleds is started as a normal node.
- **Standalone:** `python3 nodes/argo_mast_leds.py` (from repo root, with ROS2 sourced).
- **Options:** `--debug` (I2C and LED logging), `--diagnostic` (print device diagnostics on startup).

## Testing

1. **Hardware only (no ROS2):** `python3 scripts/test_mast_leds.py` — interactive keys (r/g/b/w, 1/0, t for toggle test). Confirms PCA9632 and RGBW at 0x62.
2. **Power button integration:** See [TEST_MAST_LED_POWER_BUTTON_INTEGRATION.md](TEST_MAST_LED_POWER_BUTTON_INTEGRATION.md). Run `scripts/test_mast_led_power_button_integration.sh` to cycle RGB on the mast via `/argo/power_button/rgb`.

## Diagnostics

- **Startup:** Run with `--diagnostic` to print MODE1, LEDOUT, PWM registers and recommendations.
- **Health:** Node publishes health; lifecycle manager and health monitor use `/mastleds_node_health` and `/mastleds_node/health`.
- **I2C:** If the node reports device unavailable, run `i2cdetect -y 2` and check for 0x62/0x70; see [README-i2c.md](README-i2c.md).
