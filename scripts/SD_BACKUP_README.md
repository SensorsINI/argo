# SD Card Backup and Restore for Argo

Complete SD card image backup and restore procedures for the Orange Pi Zero 2W.

## Overview

The Argo backup system allows you to create full system backups of the SD card that can be stored remotely and restored to identical hardware. This is essential for:
- **System recovery** - Restore a working configuration after problems
- **Hardware replacement** - Clone to new SD card for hardware upgrades
- **Configuration replication** - Deploy the same setup to multiple boards

## SD Card Details

- **Device**: `/dev/mmcblk0`
- **Boot Partition**: `/dev/mmcblk0p1` (ext4, 8192-62333951 sectors)
- **Total Size**: ~30GB (29.7GB)
- **File System**: ext4
- **Used Space**: ~16GB (before compression)
- **Compressed Size**: ~10-15GB (after gzip)

## Quick Start

### Create a Backup

```bash
# Interactive mode (prompts for destination)
cd ~/argo/scripts
./argo_sd_backup.sh

# Specify remote destination explicitly
./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

# Local backup only
./argo_sd_backup.sh --local

# Unattended remote backup (immune to SSH hangups)
nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y &

# Unattended remote backup and remove local file after transfer
nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y --rm-local &
```

This documentation is now out of date.

## Backup Process

### Step-by-Step Process

1.  **Create compressed image** - Uses `dd` to read entire SD card
2.  **Compress with gzip** - Reduces 30GB to ~10-15GB
3.  **Transfer to remote** - Uses `scp` to copy to remote storage
4.  **Optional cleanup** - Can remove local copy after successful transfer

### Time Estimates

- **Local backup creation**: 15-20 minutes (30GB read + compression)
- **Remote transfer**: 20-40 minutes (10-15GB over network, depends on speed)
- **Total time**: 35-60 minutes for complete remote backup

### Resource Usage

- **Disk I/O**: High read load during backup
- **CPU**: Moderate (gzip compression)
- **Network**: High bandwidth during transfer
- **Storage**: ~10-15GB for compressed backup

## Backup Scripts

### `argo_sd_backup.sh`

**Features:**
- Creates complete SD card image using `dd`
- Compresses with `gzip` for storage efficiency
- Local disk space check to prevent failures
- Shows progress with `pv` (pipe viewer)
- Interactive and non-interactive modes
- SSH connection pre-flight check for remote backups
- Automatic SSH transfer to remote storage
- Safety checks to prevent running as root
- **Automatic cleanup** of partial files on interruption

**Usage:**
```bash
# Interactive mode
./argo_sd_backup.sh

# Explicit destination
./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

# Local only
./argo_sd_backup.sh --local

# Unattended remote backup
nohup ./argo_sd_backup.sh user@host -y &

# Help
./argo_sd_backup.sh --help
```

**Arguments:**
- `remote_user@host` - SSH destination for backup
- `--local` - Save locally only, no remote transfer
- `-d, --destination PATH` - Local backup directory (default: `~/sd_backups`)
- `-y, --yes` - Non-interactive mode (no prompts)
- `--rm-local` - Remove local file after successful remote transfer
- `--no-space-check` - Skip local disk space check
- `-h, --help` - Show usage information

### `argo_sd_restore.sh`

**Features:**
- Restores compressed images to SD card
- Downloads from remote SSH locations automatically
- Safety confirmations to prevent accidental overwrites
- Progress monitoring during restore
- Device verification before proceeding

**Usage:**
```bash
# Restore from local
sudo ./argo_sd_restore.sh ~/sd_backups/argo_img.gz

# Restore from remote
sudo ./argo_sd_restore.sh user@host:~/backup.img.gz

# Custom device
sudo ./argo_sd_restore.sh --device /dev/sda backup.img.gz
```

**Safety Features:**
- Requires typing "YES" to proceed
- Final confirmation prompt
- Shows device details before writing
- Must run as root (can't accidentally run without sudo)

## Unattended Backups (Immune to SSH Hangups)

To run a backup that continues even if your SSH connection drops, use `nohup` and run the script in the background with `&`.

### How It Works
- **`nohup`**: Prevents the command from being terminated when you log out.
- **`&`**: Runs the command in the background.
- **`-y`**: Enables non-interactive mode.
- **`[destination]`**: You must specify a remote host or `--local`.

### Command Examples

**Remote Backup (Unattended):**
```bash
nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y &
```
- The output will be saved to a file named `nohup.out`.
- The local backup will be kept by default.

**Remote Backup and Remove Local Copy:**
```bash
nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y --rm-local &
```
- This is useful for saving space on the Orange Pi.

**Local Backup (Unattended):**
```bash
nohup ./argo_sd_backup.sh --local -y &
```

### Monitoring Progress
You can monitor the progress by checking the `nohup.out` file:
```bash
tail -f nohup.out
```

## Robustness and Safety

### SSH Connection Check
**New Feature**: Before starting the long backup process, the script now performs a quick, non-interactive SSH connection test to the remote host. If the connection fails for any reason (e.g., wrong hostname, authentication error), the script will exit immediately. This "fail-fast" approach prevents you from waiting for the local backup to complete only to find out the remote transfer will fail.

### Automatic Cleanup on Interruption
**New Feature**: If the backup script is interrupted (e.g., with `Ctrl+C`), it will now automatically clean up the partial `.img.7z` file it was creating. This prevents large, useless files from consuming your limited disk space.

A `trap` is set at the beginning of the script to catch the `INT` and `TERM` signals. If caught, a cleanup function runs to remove the partial backup before exiting. The trap is automatically disabled upon successful completion.

### Local Disk Space Check

**Important**: Even for remote backups, the script creates a full compressed backup file on the local SD card first. This requires significant free space.

- **Required space**: ~15GB (conservative estimate for a 30GB card)
- **Available space**: 13GB (on your system)

The script now automatically performs a disk space check before starting. If you have less than 15GB of free space, it will exit with an error to prevent the backup from failing midway.

### Bypassing the Check
If you are confident you have enough space, or if you are backing up to an external drive, you can skip the check:
```bash
./argo_sd_backup.sh user@host -y --no-space-check
```

## Remote Storage Configuration

### Using Existing Remote Machine

The Argo system is configured to use:
- **Host**: `sensors-tobidh87.lan.ini.uzh.ch`
- **User**: `tobi`
- **Directory**: `~/argo/` or `~/Dropbox/GitHub/SensorsINI/argo/`

### SSH Setup

Ensure SSH key authentication is configured:

```bash
# Generate SSH key if needed
ssh-keygen -t rsa -b 4096

# Copy public key to remote
ssh-copy-id user@host

# Test connection
ssh user@host
```

## Backup Lifecycle

### When to Create Backups

**Regular Backups:**
- After major system changes
- Before significant updates
- After hardware modifications
- Periodically (monthly recommended)

**Emergency Backups:**
- Before dangerous operations
- When system is working well
- Before field deployment

### Backup Naming Convention

Backups are automatically named:
```
argo_hostname_YYYYMMDD_HHMMSS.img.gz
```

Example:
```
argo_orangepizero2w_20240115_143022.img.gz
```

### Storage Management

**Remote Storage:**
- Backups stored on remote machine
- Keep multiple versions for recovery options
- Archive old backups after hardware changes

**Local Storage:**
- Temporary backup location: `~/sd_backups/`
- Can be removed after successful remote transfer
- Option to keep local copy for faster restores

## Restore Process

### Before Restoring

1. **Stop all services** to ensure clean state
2. **Unmount SD card** if possible
3. **Verify backup file** exists and is not corrupted
4. **Confirm device path** is correct (`/dev/mmcblk0`)

### Restore Steps

1. **Boot from different medium** (USB, network, etc.) OR remove SD card
2. **Run restore script** with `sudo`
3. **Confirm device** is correct
4. **Wait for completion** (15-25 minutes)
5. **Insert into Orange Pi** and boot
6. **Verify system** boots and functions correctly

### Post-Restore Verification

```bash
# Check disk space
df -h

# Verify services
systemctl status argo_launch_standard.service

# Test ROS2 nodes
ros2 node list

# Check hardware
i2cdetect -y 0

# Verify network
ping -c 3 8.8.8.8
```

## Best Practices

### Backup Strategy

1. **Regular Backups**: Create backups after major milestones
2. **Version Control**: Keep multiple backup versions
3. **Test Restores**: Periodically verify backups work
4. **Document Changes**: Note what changed between backups
5. **Secure Storage**: Keep backups in safe, accessible locations

### Safety Precautions

- **Never backup while mounted** if possible (use `dd` from different boot)
- **Verify device** before restore operations
- **Test restores** on spare hardware first
- **Keep multiple versions** of backups
- **Document restore procedures** for team members

### Troubleshooting

**Backup Issues:**
- **Permission denied**: Run with appropriate permissions
- **Out of space**: Check available storage space
- **Network timeout**: Use more stable connection
- **Compression errors**: Retry with different compression

**Restore Issues:**
- **Device not found**: Check `lsblk` for correct device
- **Write errors**: Verify SD card is not damaged
- **Boot failures**: Check hardware compatibility
- **Data corruption**: Restore from known-good backup

## Alternative Backup Methods

### Using dd Directly

```bash
# Create backup
sudo dd if=/dev/mmcblk0 bs=4M status=progress | gzip > backup.img.gz

# Restore
gunzip -c backup.img.gz | sudo dd of=/dev/mmcblk0 bs=4M status=progress
```

### Using rsync (Partial Backup)

```bash
# Backup user files only
rsync -av /home/orangepi/ remote_user@host:~/argo_backup/
```

### Using tar (File-Level Backup)

```bash
# Backup filesystem
sudo tar -czf backup.tar.gz /
```

## Script Output

### Backup Script Output

```
Argo SD Card Backup
===================

Current system: orangepizero2w
SD Card: /dev/mmcblk0 (29.7 GiB)

Enter remote destination (user@host) or 'local' for local only: tobi@sensors-tobidh87.lan.ini.uzh.ch

⚠️  WARNING: This will create a backup of the entire SD card
   Backup size: ~30GB (compressed to ~10-15GB)
   This will take 20-40 minutes depending on network speed

Continue? (yes/no): yes

Starting SD card backup...
=========================================
SD Device: /dev/mmcblk0
Backup name: argo_orangepizero2w_20240115_143022.img.gz

Step 1/2: Creating local backup...
62333952+0 records in
62333952+0 records out
31914983424 bytes (29.7 GiB) copied, 1256.12 s, 25.4 MB/s

Step 2/2: Transferring to remote destination...
backup.img.gz                          100% 12500MB 3.2MB/s   65:23

✅ Backup transferred successfully!
Remote location: tobi@sensors-tobidh87.lan.ini.uzh.ch:~/argo_orangepizero2w_20240115_143022.img.gz
Local copy kept at: ~/sd_backups/argo_orangepizero2w_20240115_143022.img.gz

Remove local backup to save space? (yes/no): yes
Local backup removed.

Backup process completed!
```

### Restore Script Output

```
⚠️  DANGER: SD CARD RESTORE OPERATION  ⚠️
═══════════════════════════════════════════════════

This will COMPLETELY DESTROY all data on:
  /dev/mmcblk0

Make sure this is the correct device!

Type 'YES' to proceed (anything else will cancel): YES

Configuration:
  Device: /dev/mmcblk0
  Backup: ~/sd_backups/argo_img.gz

Starting SD card restore...
=========================================
Device: /dev/mmcblk0
Backup: ~/sd_backups/argo_img.gz

This will take 15-25 minutes...

62333952+0 records in
62333952+0 records out
31914983424 bytes (29.7 GiB) copied, 1023.45 s, 31.2 MB/s

✅ Restore completed successfully!

Next steps:
  1. Safely remove the SD card
  2. Insert into Orange Pi and boot
  3. Verify the system boots correctly
```

## See Also

- [Makefile](../Makefile) - System management commands
- [Launch System](../launch/README.md) - ROS2 service management
- [Power Control](../power_control/README.md) - Hardware control
- [README.md](../README.md) - Main project documentation
