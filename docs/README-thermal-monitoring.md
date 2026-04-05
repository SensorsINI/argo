# Orange Pi thermal monitoring (SBC)

This documents **onboard computer** temperature logging (GPU, VE, CPU, DDR). 

## Purpose

- Log thermal zones periodically for diagnostics and persistent storage on the SD card.
- Let `argo_lifecycle_manager.py` read the latest log line to show CPU temperature in status output without invoking `sensors` every time.

## Service and files

| Item | Location |
|------|----------|
| Systemd unit | `launch/argo_thermal_monitor.service` (installed as `/etc/systemd/system/argo_thermal_monitor.service`) |
| Script | `launch/argo_thermal_monitor.sh` (installed as `/usr/local/bin/argo_thermal_monitor.sh`) |
| Log directory | `/var/log.hdd/persistent/` |

## Installation

From the `launch/` directory:

```bash
make install_thermal_monitor
make enable_thermal_monitor
make start_thermal_monitor
```

Other targets: `status_thermal_monitor`, `stop_thermal_monitor`, `disable_thermal_monitor`, `uninstall_thermal_monitor` (see `launch/Makefile`).

## Verify

```bash
make -C launch status_thermal_monitor
sudo systemctl status argo_thermal_monitor.service
tail -f /var/log.hdd/persistent/thermal-$(date +%Y%m%d).log
```

## Log format

Example line:

```
2025-10-01 06:30:23: GPU:60°C VE:57°C CPU:58°C DDR:58°C
```

## Temperature zones (Orange Pi Zero 2W)

- `thermal_zone0` — GPU  
- `thermal_zone1` — VE (video engine)  
- `thermal_zone2` — CPU  
- `thermal_zone3` — DDR (memory)  

## Alert threshold

The script logs warnings when any zone exceeds **80°C** and also sends them to syslog.

## Integration with Argo

The lifecycle manager scans `/var/log.hdd/persistent/thermal-*.log` for a recent CPU temperature when building status output. If the log is missing or empty, it may fall back to reading `/sys/class/thermal/thermal_zone2/temp` directly.

See `launch/argo_lifecycle_manager.py` for the thermal log handling logic.

## Related documentation

- [Persistent logging](README-persistent-logging.md) — where thermal logs live on disk  
- [System monitoring](../system-monitoring/README.md) — overview of monitoring services  
