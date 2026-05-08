# I2C0 400 kHz overlay (BNO085 / bus 0)

Sets `clock-frequency = <400000>` on `&i2c0` (PI5/PI6), on top of `pi-i2c0`.

## Build / install

From repository `nodes/` directory:
is 
```bash
make bno085-i2c0-fast-build      # local .dtbo only
sudo make bno085-i2c0-fast-install
sudo reboot
```

Uninstall: `sudo make bno085-i2c0-fast-uninstall` then reboot.

## Integration

- **Install path**: `/boot/overlay-user/argo-bno085-i2c0-fast-overlay.dtbo`
- **Boot config**: `user_overlays` in `/boot/orangepiEnv.txt` via `orangepiEnv/manage_overlays.sh`
- **Top-level**: `make -C nodes bno085-i2c0-fast-install` from repo root (with `cd` into `nodes` as appropriate)

## Notes

- Bus 0 is **shared** (IMU, battery/water, etc.): every device must support **Fast mode (400 kHz)**.
- See `docs/README-i2c.md` for the full I2C map and safety notes.
