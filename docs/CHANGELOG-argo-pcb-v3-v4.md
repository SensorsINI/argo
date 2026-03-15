# Argo PCB v3 -> v4 Changelog

This changelog summarizes the important hardware/software updates required when moving from Argo PCB v3 to v4.

## Scope

- Orange Pi Zero 2W 40-pin header mappings
- I2C bus topology and runtime adapter mapping
- Power-control LED and button behavior
- Node-level pin and bus updates
- Boot overlay and installation flow

## Hardware Mapping Changes

- **Power button (`POW_BUT`) moved**:
  - from older mapping to `PH0` (pin 8, GPIO line 224).
- **LoRa chip select (`!LORA_SEL`) moved**:
  - now uses line `264` (header pin 3 / `SDA.1`).
- **Blue LED path changed**:
  - `PI1` is no longer the direct blue LED control path (testpoint role).
  - Blue LED state is tied to charger signaling (`!CHG` / `PC12`) behavior.
- **Green heartbeat LED remains on PH4**:
  - PH4 drives both power-button green LED and HB LED path.
- **`PG` (pin 38) clarified**:
  - from CH221K USB-C PD logic, currently unused by software.

## I2C Topology Changes (Critical)

- Wind/mast bus on v4 is on **TWI2** (pins 27/28).
- Linux adapter numbering for TWI2 is **not fixed** across images/DT ordering.
  - observed as `i2c-3` or `i2c-4`.
- `i2c-2` may be HDMI DDC and should not be assumed to be wind/mast bus.

### Practical rule

- Resolve TWI2 at runtime from sysfs:
  - `/sys/devices/platform/soc*/5002800.i2c/i2c-*`

## Software Updates Applied

- `nodes/anem.py`
  - switched from hardcoded I2C bus to runtime TWI2 resolver.
- `nodes/argo_mast_leds.py`
  - switched from hardcoded I2C bus to runtime TWI2 resolver.
- `scripts/ai2c.sh`
  - now resolves TWI2 dynamically (aligned with `anem.py`).
- `nodes/gps.py`
  - added `!GPS_RESET` handling for prolonged failures.
  - added `GPS_PPS` status topic publication.
- `nodes/lora.py`
  - updated `!LORA_SEL` line to v4 mapping.
- `power_control/argo_power_control.py`
  - updated power button line.
  - robust green LED sysfs handling for both labels:
    - `/sys/class/leds/argo:green:heartbeat`
    - `/sys/class/leds/green_led`

## Boot/Overlay Integration Changes

- `power_control/orangepiEnv.txt`
  - updated overlays for v4 I2C path:
    - includes `pi-i2c0` and `pi-i2c2`.
- `power_control/Makefile`
  - `install` now includes:
    - `install-orangpienv` (copies authoritative `power_control/orangepiEnv.txt` to `/boot`)
    - `install-sys-heartbeat` (installs PH4 heartbeat overlay + permissions service)
- `power_control/sys_heartbeat/Makefile`
  - `install` adds `argo-ph4-led-overlay` into `user_overlays` via centralized `manage_overlays.sh`.

## Documentation Added/Updated

- Added consolidated hardware reference:
  - `docs/README-pins-i2c.md`
- Updated:
  - `docs/README-i2c.md`
  - `docs/README-mast-leds.md`
  - `nodes/README.md`
  - `power_control/README.md`
  - `power_control/sys_heartbeat/README.md`

## Known Gotchas

- If `argo-ph4-led-overlay` is missing from `user_overlays`, LED label/behavior can differ from expected Argo path.
- If `argo-ph4-led-perms.service` fails, userspace LED control may fail with permission errors.
- If wind bus appears missing, first confirm resolved TWI2 Linux adapter before scanning.

## Quick Verification After Install/Reboot

```bash
# Boot overlay state
sudo sed -n '/^overlays=/p;/^user_overlays=/p' /boot/orangepiEnv.txt

# I2C adapters
i2cdetect -l
ls /sys/devices/platform/soc*/5002800.i2c/i2c-*

# Argo combined I2C check
ai2c

# Green LED label and control path
ls /sys/class/leds
for p in /sys/class/leds/argo:green:heartbeat /sys/class/leds/green_led; do [ -d "$p" ] && echo "$p"; done

# Power-control and permissions service status
sudo systemctl status argo_power_control.service --no-pager
sudo systemctl status argo-ph4-led-perms.service --no-pager
```

