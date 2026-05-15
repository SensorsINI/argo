# Argo Orange Pi Header and Bus Reference

This document centralizes the current Argo hardware/software mapping for the Orange Pi Zero 2W 40-pin header:

- power rails and grounds
- I2C buses and device ownership
- SPI wiring
- PWM/radio-servo wiring
- serial/UART wiring
- per-pin software ownership, direction, and pull configuration

It reflects:

- current project code (`nodes/*`, `power_control/*`, `nodes/pwm_capture_module/*`)
- live runtime snapshot from `gpio readall`

---

## Critical Bus Numbering (TWI vs Linux)

On this platform, schematic names and Linux `i2c-*` numbering are not intuitive and
can vary by image/kernel:

- **TWI0 (PI5/PI6 on pins 15/22)** -> commonly Linux **`/dev/i2c-0`**
- **TWI1 (PI8/PI7 on pins 3/5)** -> commonly Linux **`/dev/i2c-3`**
- **TWI2 (PI10/PI9 on pins 27/28)** -> Linux **`/dev/i2c-3` or `/dev/i2c-4`** depending on kernel/device-tree ordering (wind/mast bus)
- Linux **`/dev/i2c-2`** is often **HDMI DDC**, not the wind sensor header bus

Current boot overlay line for Argo should include TWI0 + TWI2:

- `overlays=pi-i2c0 pi-i2c2 ...`

If `pi-i2c1` is enabled, pins 3/5 are claimed by I2C and can conflict with Argo GPIO use
(for example `!LORA_SEL` on pin 3).

---

## I2C Device Ownership

| Linux bus | Schematic bus | Header pins | Typical devices | Node(s) |
|---|---|---|---|---|
| `i2c-0` (typical) | TWI0 | pin 15 (SCL), pin 22 (SDA) | `0x34` MAX11612, `0x44` SHT45, `0x4a` BNO085 | `nodes/argo_battery_water.py`, `nodes/bno085.py` |
| `i2c-3` or `i2c-4` (runtime) | TWI2 | pin 27 (SDA.2), pin 28 (SCL.2) | `0x21/0x22/0x23` SDP3x, `0x62/0x70` PCA9632 | `nodes/anem.py`, `nodes/argo_mast_leds.py`, `scripts/ai2c.sh` |

Runtime note: Argo tools now resolve TWI2 dynamically from `/sys/devices/platform/soc*/5002800.i2c/i2c-*` to avoid hardcoding `i2c-*` numbers.

Pullup note: I2C pullups on both sides of the level translation path are provided by the two TXS0102 sections (3.3V OPi side and 5V sensor side).

---

## 40-Pin Header Usage Table

Legend:

- **Direction/mode** = software-intended direction or peripheral mode
- **Pull** = gpiod-requested bias where explicitly set in code
- **Live mode** = observed from current `gpio readall` snapshot

| Pin | OPI signal | GPIO line | Argo function | Used by node(s) | Direction/mode | Pull | Live mode |
|---|---|---:|---|---|---|---|---|
| 1 | 3.3V | - | 3.3V rail | power | power | - | 3.3V |
| 2 | 5V | - | 5V rail | power | power | - | 5V |
| 3 | SDA.1 (PI8/TWI1_SDA) | 264 | `!LORA_SEL` (LoRa CS) | `nodes/lora.py` | GPIO output (default high) | none explicit | `ALT5` |
| 4 | 5V | - | 5V rail | power | power | - | 5V |
| 5 | SCL.1 (PI7/TWI1_SCL) | 263 | reserved | - | unused/reserved | - | `ALT5` |
| 6 | GND | - | ground | power | ground | - | GND |
| 7 | PI13/PWM3/UART4_TX | 269 | `RADIO_SAIL` capture input | PWM kernel module | GPIO input (module-owned) | overlay-defined | `ALT6` |
| 8 | PH0/UART_TX | 224 | `POW_BUT` (power button) | `power_control/argo_power_control.py` | GPIO input (rising-edge events) | `BIAS_DISABLE` | `ALT6` |
| 9 | GND | - | ground | power | ground | - | GND |
| 10 | PH1/UART_RX | 225 | `!GPS_RESET` | `nodes/gps.py` | GPIO output (active-low reset) | none explicit | `OUT` |
| 11 | PH2/UART5_TX | 226 | GPS UART TX | `nodes/gps.py` | UART5 TX | peripheral | `ALT2` |
| 12 | PI1 | 257 | Testpoint only (blue LED no longer wired here) | reserved/testpoint | unused | - | `OFF` |
| 13 | PH3/UART5_RX | 227 | GPS UART RX | `nodes/gps.py` | UART5 RX | peripheral | `ALT2` |
| 14 | GND | - | ground | power | ground | - | GND |
| 15 | PI5/TWI0_SCL/UART2_TX | 261 | I2C0 SCL | battery/IMU nodes | I2C peripheral | pullups via TXS0102 (3.3V+5V sides) | `ALT5` |
| 16 | PI14/PWM4/UART4_RX | 270 | `SERVO_SAIL` PWM output | PWM kernel module | PWM output | overlay-defined | `OFF` |
| 17 | 3.3V | - | 3.3V rail | power | power | - | 3.3V |
| 18 | PH4 | 228 | Green heartbeat in power button and HB status LED | `power_control/argo_power_control.py`, `power_control/sys_heartbeat` | sysfs/GPIO output | kernel LED overlay behavior | `OFF` |
| 19 | PH7/SPI1_MOSI | 231 | LoRa SPI MOSI | `nodes/lora.py` | SPI1 MOSI | peripheral | `ALT4` |
| 20 | GND | - | ground | power | ground | - | GND |
| 21 | PH8/SPI1_MISO | 232 | LoRa SPI MISO | `nodes/lora.py` | SPI1 MISO | peripheral | `ALT4` |
| 22 | PI6/TWI0_SDA/UART2_RX | 262 | I2C0 SDA | battery/IMU nodes | I2C peripheral | pullups via TXS0102 (3.3V+5V sides) | `ALT5` |
| 23 | PH6/SPI1_SCLK | 230 | LoRa SPI SCLK | `nodes/lora.py` | SPI1 clock | peripheral | `ALT4` |
| 24 | PH5/SPI1_CS0 | 229 | `GPS_PPS` input (v4: non-functional, see note below) | `nodes/gps.py` | GPIO input | `BIAS_DISABLE` | `ALT4` |
| 25 | GND | - | ground | power | ground | - | GND |
| 26 | PH9/SPI1_CS1 | 233 | `!ACOK` (charger AC-present signal) | `power_control/argo_power_control.py` | GPIO input | board-driven | `IN` |
| 27 | PI10/TWI2_SDA/UART3_RX | 266 | I2C2 SDA (wind/mast bus) | `nodes/anem.py`, `nodes/argo_mast_leds.py` | I2C peripheral | pullups via TXS0102 (3.3V+5V sides) | `OFF` |
| 28 | PI9/TWI2_SCL/UART3_TX | 265 | I2C2 SCL (wind/mast bus) | `nodes/anem.py`, `nodes/argo_mast_leds.py` | I2C peripheral | pullups via TXS0102 (3.3V+5V sides) | `OFF` |
| 29 | PI0 | 256 | `!LORA_RST` | `nodes/lora.py` | GPIO output (default high) | none explicit | `OFF` |
| 30 | GND | - | ground | power | ground | - | GND |
| 31 | PI15 | 271 | `LORA_OUT` / DIO0 interrupt | `nodes/lora.py` | GPIO input | none explicit | `OFF` |
| 32 | PI11/PWM1 | 267 | `RADIO_RUDDER` capture input | PWM kernel module | GPIO input (module-owned) | overlay-defined | `ALT6` |
| 33 | PI12/PWM2 | 268 | `SERVO_RUDDER` PWM output | PWM kernel module | PWM output | overlay-defined | `OFF` |
| 34 | GND | - | ground | power | ground | - | GND |
| 35 | PI2 | 258 | `BUZZER` | `power_control/argo_power_control.py` | GPIO output | none explicit | `OUT` |
| 36 | PC12 | 76 | `!CHG` / charger `!STAT` line (blue LED control + charge-state sense) | `power_control/argo_power_control.py` | GPIO open-drain style control and readback | pulldown disabled in software flow | `IN` |
| 37 | PI16 | 272 | Red warning LED | `power_control/argo_power_control.py` | GPIO output | none explicit | `OUT` |
| 38 | PI4 | 260 | `PG` from CH221K USB-C chip (USB-C attached + valid power negotiation) | currently not claimed by active node | input/reserved | board-driven | `IN` |
| 39 | GND | - | ground | power | ground | - | GND |
| 40 | PI3 | 259 | `POW_OFF` relay reset control | shutdown hook / power-control system | GPIO output pulse (shutdown path) | none explicit | `OFF` |

---

## Power Button Pin Check (Requested)

Confirmed:

- **Power button is on PH0 (pin 8, GPIO line 224)** in current code and runtime mapping.
- In the provided `gpio readall`, pin 8 appears as `TXD.0` (`ALT6`) at sample time; this can occur depending on active overlays/services at that moment.
- The Argo power-control code uses GPIO line **224** and requests it as an event/input line with `BIAS_DISABLE`.

---

## Green LED Ownership and Sysfs Names (PH4)

`PH4` (pin 18, line 228) is shared between:

- kernel heartbeat trigger during boot/idle (`trigger=heartbeat`)
- `power_control/argo_power_control.py` during Argo LED pattern control (`trigger=none`)

Sysfs naming depends on active overlay/DT label:

- custom Argo label: `/sys/class/leds/argo:green:heartbeat`
- compatibility/generic label: `/sys/class/leds/green_led`

These are two labels for the same PH4 LED path, not different hardware pins.

---

## Blue LED and `!CHG` Update

Hardware update (v4):

- The power-button blue LED is **not driven by PI1** anymore.
- PI1 is effectively a **testpoint** path and not the active blue LED control path.
- Blue LED behavior is tied to the MPC2672 charger status node:
  - header pin 36 / PC12 / line 76, net `!CHG`
  - this net is tied to charger `!STAT` (pin 15 on MPC2672)

Behavior:

- If OPi releases/drives the line high (with pulldown disabled), OPi can read charger status on this node:
  - low: charging
  - high: charged/idle
  - blinking: charger fault indication
- If OPi drives the node low, it forces blue LED on.

---

## SPI, PWM, Serial Summary

### SPI (LoRa)

- Bus/device: SPI1 (`spidev1.0`)
- Header pins:
  - MOSI pin 19 (GPIO 231)
  - MISO pin 21 (GPIO 232)
  - SCLK pin 23 (GPIO 230)
  - CS via GPIO pin 3 (GPIO 264, `!LORA_SEL`)
  - Reset pin 29 (GPIO 256)
  - IRQ pin 31 (GPIO 271)

### PWM / Radio Servo

- Inputs (radio capture): pin 7 (`PI13`), pin 32 (`PI11`)
- Outputs (servo PWM): pin 33 (`PI12` -> PWM2), pin 16 (`PI14` -> PWM4)
- Managed by kernel module/overlay in `nodes/pwm_capture_module`.

### Serial

- GPS UART5:
  - TX pin 11 (`PH2`)
  - RX pin 13 (`PH3`)
- Additional GPS control/status lines:
  - `!GPS_RESET` pin 10 (GPIO 225)
  - `GPS_PPS` pin 24 (GPIO 229) - **v4 limitation:** Pin claimed by SPI1_CS0 for LoRa, PPS monitoring non-functional (see v4→v5 migration doc)

---

## Quick Verification Commands

```bash
# Header live state
gpio readall

# I2C bus scans (both active external buses)
i2cdetect -y 0
TWI2_NODE="$(basename "$(ls -d /sys/devices/platform/soc*/5002800.i2c/i2c-* | head -n1)")"
TWI2_BUS="${TWI2_NODE#i2c-}"
echo "TWI2 runtime bus: i2c-${TWI2_BUS}"
i2cdetect -y "$TWI2_BUS"

# Combined Argo check script (alias ai2c)
bash ~/argo/scripts/ai2c.sh

# Green LED control path (which label is active)
ls /sys/class/leds
for p in /sys/class/leds/argo:green:heartbeat /sys/class/leds/green_led; do [ -d "$p" ] && echo "$p"; done
```
