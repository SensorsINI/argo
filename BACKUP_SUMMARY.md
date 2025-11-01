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

```bash
cd ~/argo/scripts

# Interactive mode (prompts for destination)
./argo_sd_backup.sh

# Or use Makefile
make backup-sd

# Backup to specific remote host
./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

# Local backup only (no remote transfer)
./argo_sd_backup.sh --local
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

1. **Creates image** using `dd` to read entire SD card (`/dev/mmcblk0`)
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

Your SD card details:
- **Device**: `/dev/mmcblk0`
- **Boot partition**: `/dev/mmcblk0p1` (ext4)
- **Total size**: ~30GB (29.7GB)
- **Used space**: ~16GB (before compression)
- **Compressed**: ~10-15GB

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


