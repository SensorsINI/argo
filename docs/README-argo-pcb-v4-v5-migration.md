# Argo PCB v4 -> v5 Migration Notes

This document captures planned hardware and software migration items from Argo PCB v4 to v5.

## Goals

- simplify SPI/diagnostic pin ownership
- reduce boot-time pin conflicts
- keep LoRa SPI reliable while re-enabling GPS PPS diagnostics

## Planned Pin/Bus Changes

- Move `!LORA_SEL` to `SPI1_CS0` (pin 24 / PH5).
- Move `GPS_PPS` off pin 24 to `PI8` (where `!LORA_SEL` is currently used in v4).

Rationale:

- Current v4 setup uses `spi1-cs0-spidev` for LoRa SPI access (`/dev/spidev1.0`), which claims CS0.
- That blocks GPIO request of pin 24 for PPS monitoring.
- v5 routing above removes this overlap and simplifies runtime configuration.

## Current v4 Limitation (Accepted)

**GPS PPS monitoring is non-functional on v4 due to pin conflict:**

- GPS module successfully outputs PPS pulses (configured via UBX-CFG-TP5)
- Pin 24 (GPIO 229 / PH5) is hardwired to GPS PPS output
- However, pin 24 is claimed by kernel as SPI1_CS0 for LoRa communication
- GPIO monitoring cannot co-exist with SPI peripheral mode
- Result: PPS pulses are generated but not monitored by software

**Impact:** Cosmetic only. GPS fix status is determined by NMEA sentence parsing (RMC/GGA), not PPS monitoring. PPS monitoring was intended for timing validation and diagnostics only.

**Software behavior:** `nodes/gps.py` detects the conflict at startup and logs:
```
GPS_PPS GPIO line 229 is in use by the kernel (SPI1_CS0 / LoRa on this board).
Skipping PPS pin monitoring; module PPS output via UBX is unchanged.
```

Status messages show `PPS=unavailable` while the system operates normally.

## Overlay/Software Implications (Planned)

- Update LoRa SPI/CS handling in `nodes/lora.py` to use hardware CS0 path.
- Update GPS PPS line in `nodes/gps.py` to new v5 GPIO line.
- Review `/boot/orangepiEnv.txt` overlays to ensure they match v5 wiring.
- Update docs:
  - `docs/README-pins-i2c.md`
  - `docs/CHANGELOG-argo-pcb-v3-v4.md` (or successor v5 changelog)
  - node-specific READMEs where pin numbers are mentioned

## Buzzer Hardware Note (v5 reference)

- `BUZZER` output is pulled low on PCB by a **10k ohm pulldown**.
- This keeps the buzzer OFF during boot while pins may float.
- Shutdown hook buzzer behavior is already verified via `power_control/argo_poweroff.shutdown`.

## Validation Checklist (when v5 board is available)

```bash
# SPI device presence
ls -l /dev/spidev*

# GPIO ownership checks
gpioinfo gpiochip0 | awk '/line 229:|line 264:|line 233:/'

# LoRa node startup
lora.py

# GPS node startup and PPS request diagnostics
gps.py
```

