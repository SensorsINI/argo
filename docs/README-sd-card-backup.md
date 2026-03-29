# SD Card Backup Solution for Argo

## ✅ What Was Created

I've created a complete SD card backup and restore system for your Orange Pi Zero 2W running the Argo sailboat project.

### New Files

1. **`scripts/argo_sd_backup.sh`** - Creates compressed SD card backups
2. **`scripts/argo_sd_restore.sh`** - Restores backups to SD cards  
3. **`scripts/SD_BACKUP_README.md`** - Complete documentation
4. **Updated `scripts/Makefile`** - Added backup targets

### Key Features

✅ **Yes, you can create SD card backups** from the Orange Pi to remote storage using scp/rsync concepts

✅ **Compressed backups** - 30GB SD card compresses to ~10-15GB

✅ **Remote storage support** - Transfers to remote machines via scp

✅ **Safety features** - Multiple confirmation prompts, prevents running as root during backup

✅ **Progress monitoring** - Uses `pv` (pipe viewer) to show progress

✅ **Automatic naming** - Backups named with hostname and timestamp

## Quick Start

### Create a Backup

**Can run on Argo itself or on another Linux computer with the SD card attached**

```bash
cd ~/argo/scripts

# Interactive mode (auto-detects source device from current root disk)
./argo_sd_backup.sh

# Or use Makefile
make backup-sd

# Backup to specific remote host
./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

# Local backup only (no remote transfer)
./argo_sd_backup.sh --local

# On Argo OPi: if no destination is given, script auto-detects host and uses default remote
./argo_sd_backup.sh

# Local backup from attached SD card partition (whole disk auto-resolved)
./argo_sd_backup.sh --local --source-device /dev/sde1

# If /dev/sdX changes between computers, select by mountpoint instead
./argo_sd_backup.sh --local --source-mount /media/$USER/<sd_mount_name>
```

### Restore from Backup

```bash
cd ~/argo/argo/scripts

# Restore from local file
sudo ./argo_sd_restore.sh ~/sd_backups/argo_img.gz

# Restore from remote
sudo ./argo_sd_restore.sh user@host:~/backup.img.gz
```

## How It Works

### Backup Process

1. **Creates image** using `dd` to read an SD card source disk (auto-detected or passed with `--source-device` / `--source-mount`)
2. **Compresses** with `gzip` to reduce 30GB → ~10-15GB
3. **Monitors progress** with `pv` pipe viewer
4. **Transfers to remote** via `scp` to your configured storage location
5. **Optional cleanup** - Can remove local copy after successful transfer

### Time Estimates

- **Backup creation**: 15-20 minutes (reading and compressing 30GB)
- **Remote transfer**: 20-40 minutes (depends on network speed)
- **Total**: 35-60 minutes for complete remote backup

### Remote Storage

The script uses the existing remote configuration:
- **Host**: `sensors-tobidh87.lan.ini.uzh.ch`
- **User**: `tobi`  
- **Location**: `~/argo/` or wherever you specify

## Usage Examples

### Example 1: Interactive Backup

```bash
cd ~/argo/scripts
./argo_sd_backup.sh

# Prompts for destination:
# Enter remote destination (user@host) or 'local' for local only: tobi@sensors-tobidh87.lan.ini.uzh.ch
```

### Example 2: Direct Remote Backup

```bash
./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch
```

### Example 3: Local Backup Only

```bash
./argo_sd_backup.sh --local

# Saves to ~/sd_backups/
```

### Example 4: Restore from Backup

```bash
sudo ./argo_sd_restore.sh ~/sd_backups/argo_orangepizero2w_20240115_143022.img.gz
```

## Safety Features

### Backup Script Safety

- ✅ **Not run as root** - Prevents accidental overwrites
- ✅ **Clear confirmations** - Warns about time and network usage
- ✅ **Shows progress** - Lets you monitor the backup process

### Restore Script Safety

- ✅ **Requires root** - Must run with `sudo`
- ✅ **Multiple confirmations** - Must type "YES" to proceed
- ✅ **Final check** - Extra confirmation before writing
- ✅ **Device verification** - Checks device exists before proceeding

## Configuration

Source selection options:
- **Default**: auto-detect from current system root disk (best when run on Argo itself)
- **Desktop/local attached SD**: use `--source-device /dev/sdX1` (script resolves to whole disk)
- **Portable across changing device names**: use `--source-mount /path/to/mountpoint`
- **Typical SD size**: ~30GB (compressed backup usually ~10-15GB)

Host auto-detection and destination behavior:
- If running on Argo Orange Pi (Armbian + Orange Pi kernel/model + Argo layout), script assumes self-backup mode
- In that case, if no destination is passed and `--local` is not set, it uses default remote destination
- Override default destination with env var: `ARGO_SD_BACKUP_REMOTE_DEST=user@host`
- On non-Argo computers, script will **not** auto-select the current root/system disk; it only auto-selects external/removable candidates or asks you to choose
- Root/system disk backup on non-Argo hosts is blocked by default; override only if intentional with `--allow-root-source`

## Documentation

Full documentation available in:
- **`scripts/SD_BACKUP_README.md`** - Complete guide with examples
- **`make backup-help`** - Quick reference in Makefile
- **`./argo_sd_backup.sh --help`** - Command-line help
- **`./argo_sd_restore.sh --help`** - Command-line help

## Alternative: Using dd Directly

If you prefer manual control:

```bash
# Create backup
sudo dd if=/dev/mmcblk0 bs=4M status=progress | gzip > backup.img.gz

# Transfer to remote
scp backup.img.gz user@host:~

# Restore
gunzip -c backup.img.gz | sudo dd of=/dev/mmcblk0 bs=4M status=progress
```

The scripts wrap these commands with safety features and progress monitoring.

## Next Steps

1. **Test the backup script** - Try a local backup first to verify
2. **Verify remote storage** - Ensure SSH access to remote host
3. **Schedule regular backups** - Consider automating with cron
4. **Test restore** - Verify you can restore on spare hardware

## Help

```bash
# Show help
./argo_sd_backup.sh --help
./argo_sd_restore.sh --help

# Or use Makefile
cd ~/argo/scripts
make backup-help
```

The backup system is ready to use!
