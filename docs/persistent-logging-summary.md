# Persistent Boot Logging Configuration Summary

## Changes Made (2025-10-08)

### 1. Systemd Journal Persistent Storage
- **Location**: `/var/log/journal/ce4def9bd6a04cd594e94f8205bbe671/`
- **Configuration**: `/etc/systemd/journald.conf`
  - `Storage=persistent` (was: volatile)
  - `SystemMaxUse=500M` (500MB max journal size)
  - `MaxRetentionSec=2week` (2 week retention)
- **Result**: Boot logs now preserved across reboots in `/var/log/journal/`

### 2. Boot History Logger
- **Service**: `boot-history-logger.service`
- **Script**: `/usr/local/bin/boot-logger.sh`
- **Output Files**:
  - `/var/log.hdd/persistent/boot-history.log` - Boot event log (appends)
  - `/var/log.hdd/persistent/dmesg-YYYYMMDD.log` - Daily dmesg (appends all boots)
  - `/var/log.hdd/persistent/journalctl-YYYYMMDD-HHMMSS.log` - Per-boot journal backup

### 3. Existing Monitoring (Unchanged)
- `orangepi-monitor.service` - System resource monitoring
- `thermal-monitor.service` - Temperature monitoring
- `memory-monitor.service` - Memory usage monitoring
- `cursor-monitor.service` - Cursor process monitoring

## Hang Diagnosis (October 7, 2025 ~08:59 AM)

### Root Cause: Disk I/O Storm
- **08:57:57**: Normal - Load 3.32, CPU 85%, I/O 3%
- **08:58:39**: I/O storm begins - Load 6.62, CPU 97%, I/O 67%
- **08:59:45**: Critical - Load 23.08, CPU 97%, I/O 67%
- **Result**: System hung due to I/O wait

### Contributing Factors
- 67% I/O wait indicates all processes blocked on disk
- Load jumped 7x (3.32 → 23.08) in 2 minutes
- Likely causes:
  - Heavy file indexing/search
  - Log rotation or sync operations
  - Database/cache updates
- Low memory conditions (35MB free) may have triggered swapping

## Future Hang Investigation

### Use These Commands:
```bash
# Check boot history across reboots
cat /var/log.hdd/persistent/boot-history.log

# View journal from previous boots
sudo journalctl --list-boots
sudo journalctl -b -1  # Previous boot
sudo journalctl -b -2  # Two boots ago

# Check persistent dmesg logs
cat /var/log.hdd/persistent/dmesg-$(date +%Y%m%d).log

# View monitoring data before hang
cat /var/log.hdd/persistent/orangepi-monitor-$(date +%Y%m%d).log.1
```

### What to Look For:
1. High I/O wait percentages (>50%)
2. Load average spikes (>10)
3. Memory exhaustion (free <50MB)
4. OOM killer events
5. Disk errors or timeouts
6. Heavy cursor/indexing processes

## Verification

Test that logging works across reboots:
```bash
# Before reboot - note current boot ID
sudo journalctl --list-boots

# After reboot - verify previous boot is preserved
sudo journalctl --list-boots
sudo journalctl -b -1 | head -50
cat /var/log.hdd/persistent/boot-history.log
```
