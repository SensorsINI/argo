# TODO: Persistent Systemd Logging (Simplify Argo Services)

This document captures analysis and recommendations for **simplifying all Argo systemd services** by making **all** systemd journal logging go to persistent SD storage globally, instead of per-service tee pipelines.

## Current Situation

Argo services are complicated by the need during development to have **persistent logging** so logs can be inspected across reboots. On Armbian/Orange Pi:

- **ramlog** often keeps `/var/log` (and thus `/var/log/journal`) in **zram/RAM**, so journal data is lost on reboot.
- Argo uses **`/var/log.hdd/persistent/`** as the canonical persistent log directory.

To get service output onto persistent storage, **each** Argo service currently:

1. Uses a **tee pipeline** in `ExecStart`, e.g.:
   ```bash
   exec /usr/bin/python3 /path/to/script.py |& tee -a /var/log.hdd/persistent/argo-*.log
   ```
2. Often adds `ExecStartPre=/bin/mkdir -p /var/log.hdd/persistent`.
3. Keeps `StandardOutput=journal` and `StandardError=journal` so output also goes to the journal (which may be volatile).

**Problems with this approach:**

- Every service has a **complicated** `ExecStart` (bash, pipe, tee, log path).
- The **pipe can block** if tee doesn’t read (e.g. full pipe buffer), making the process appear hung and producing no log output.
- Duplication: output is written both to the journal and to a file; maintenance and debugging are harder.

**Services that currently use this tee pattern** (as of this doc):

- `argo_power_control.service`
- `argo_battery_water.service`
- `argo_bno085.service`
- `argo_launch_standard.service`
- `argo_health_monitor.service`
- `argo_radio_servo_module.service`
- Plus some system-monitoring services (e.g. `orangepi-monitor.service`, `crash-dump.service`, `persistent-dmesg.service`) that write directly to `/var/log.hdd/persistent/` in other ways.

## Recommended Approach: Global Persistent Journal

**Idea:** Configure **systemd journald** so that its storage lives on **persistent SD** (e.g. under `/var/log.hdd/persistent` or equivalent). Then:

- All service stdout/stderr that goes to the journal (via `StandardOutput=journal` / `StandardError=journal`) is **automatically** on persistent storage and survives reboot.
- Argo services can **drop** the tee pipeline and `ExecStartPre=mkdir`; `ExecStart` becomes a simple command.
- One global configuration replaces per-service complexity.

### Why This Works

1. **Journal is the single sink:** Service output is captured by systemd and written by journald. If journald’s storage is on disk (SD), all that output is persistent.
2. **No per-service tee:** No pipe, no tee, no per-service log file path—just a normal `ExecStart=... /path/to/script.py`.
3. **Existing tooling still works:** `alog` (argo_logs.sh) already uses **journalctl** to tail and grep by unit name; it does **not** read the tee’d files. So `alog` and `alog <pattern>` continue to work unchanged once everything is in the journal.
4. **Inspect after reboot:** `journalctl -u argo_power_control.service -b -1` (previous boot) and similar commands show persistent history as long as the journal is on disk.

## How to Put the Journal on Persistent Storage

Journald does **not** support an arbitrary path in its config. It uses a fixed layout under **`/var/log/journal`** when `Storage=persistent` (or `auto` with that directory present). So we don’t change the path in journald; we make **`/var/log/journal`** point at our persistent location.

### Option: Bind-mount journal to SD

1. **Create directory on persistent mount**
   - e.g. `/var/log.hdd/persistent/journal` (reuse the same mount that already provides `/var/log.hdd/persistent`).

2. **Ensure the persistent mount is available early**
   - The unit that mounts `/var/log.hdd/persistent` (or the partition that contains it) must run before journald starts (or before any service that needs persistent journal). Ordering may require a mount unit and `Before=systemd-journald.service` (or equivalent) as needed.

3. **Bind-mount over `/var/log/journal`**
   - In a mount unit (e.g. `var-log-journal.mount`) or in `/etc/fstab`:
   - Bind-mount: **`/var/log.hdd/persistent/journal`** → **`/var/log/journal`**
   - So journald still “sees” `/var/log/journal`, but the real data lives on SD.

4. **Configure journald for persistent storage**
   - In `/etc/systemd/journald.conf` (or a drop-in under `/etc/systemd/journald.conf.d/`):
   - Set **`Storage=persistent`**.
   - Restart journald after the bind mount and this config are in place.

5. **Ordering**
   - Mount that provides `/var/log.hdd/persistent`  
   - → then mount unit that bind-mounts `.../persistent/journal` to `/var/log/journal`  
   - → then `systemd-journald.service` (and all Argo services).

After this, **all** systemd logging (including every Argo service) goes into the journal, and the journal lives on SD. No tee needed.

## Simplifying Argo Services

Once the journal is on persistent storage as above:

**Remove from each Argo service:**

- `ExecStartPre=/bin/mkdir -p /var/log.hdd/persistent`
- The entire `|& tee -a /var/log.hdd/persistent/argo-*.log` pipeline

**Keep (or rely on default):**

- `StandardOutput=journal`
- `StandardError=journal`

**Set `ExecStart` to the simple command**, e.g.:

```ini
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 /path/to/script.py'
```

(No pipe, no tee.)

## Trade-offs

- **Slower:** Writing to SD is slower than writing to zram; journal I/O may add a small delay under heavy logging.
- **SD wear:** More writes to the SD card. Acceptable for development; for long-term deployment, consider log size limits and rotation (journald has `SystemMaxUse`, `MaxFileSec`, etc.).
- **Single point of configuration:** One bind-mount and one journald config; if the mount fails, journal falls back to volatile storage unless additional safeguards are added.

## Summary

- **Problem:** Per-service tee to `/var/log.hdd/persistent` makes ExecStart complex and can cause pipe-blocking “hang” behavior.
- **Recommendation:** Put the **journal** on persistent SD (bind-mount `/var/log.hdd/persistent/journal` to `/var/log/journal`, set `Storage=persistent`), then **simplify all Argo services** by removing tee and mkdir from ExecStart.
- **Result:** One global configuration for persistent systemd logging; `alog` and journalctl continue to work; logs survive reboot; services stay simple and robust.

## Related

- **Current persistent logging:** `docs/README-persistent-logging.md` (ramlog, `/var/log.hdd/persistent`, Argo’s current setup).
- **Log viewer:** `scripts/argo_logs.sh` (alias `alog`) uses journalctl only; no change needed after moving to global persistent journal.
