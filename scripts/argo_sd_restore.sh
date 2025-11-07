#!/bin/bash
# Argo SD Card Restore Script
# Restores a backup image to the Orange Pi SD card
#
# ⚠️  WARNING: This will OVERWRITE the SD card - use with extreme caution!
#
# Usage:
#   ./argo_sd_restore.sh image_file.img.gz
#   ./argo_sd_restore.sh /path/to/backup.img.gz
#   ./argo_sd_restore.sh user@host:~/backup.img.gz  # Restore from remote
#
# The SD card should be unmounted or mounted as read-only before running this.

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Show usage
usage() {
    cat << EOF
Argo SD Card Restore Script
============================

⚠️  WARNING: This will COMPLETELY OVERWRITE the SD card!

Usage:
    ./argo_sd_restore.sh [backup_file]

Options:
    -h, --help      Show this help message
    --device DEV    Specify SD card device (default: interactive detection)

Examples:
    # Restore from local file (supports .gz and .7z)
    ./argo_sd_restore.sh ~/sd_backups/argo_img.gz

    # Restore from remote (typically .gz format from streaming backup)
    ./argo_sd_restore.sh user@host:~/argo_hostname_30GB_20251103_191209.img.gz

    # Restore local .7z backup (from local-only backup mode)
    ./argo_sd_restore.sh ~/sd_backups/argo_hostname_30GB_20251103_191209.img.7z

    # Custom device
    ./argo_sd_restore.sh --device /dev/sda backup.img.gz

EOF
}

# Helper function to get disk devices
get_disks() {
    lsblk -d -n -o NAME,TYPE | grep -w "disk" | awk '{print "/dev/"$1}' | sort
}

# Parse arguments
BACKUP_FILE=""
DEVICE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        *)
            if [ -z "$BACKUP_FILE" ]; then
                BACKUP_FILE="$1"
            else
                echo -e "${RED}Error: Multiple backup files specified${NC}"
                exit 1
            fi
            shift
            ;;
    esac
done

# Check if running as root first (REQUIRED for restore)
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}❌ Error: This script MUST be run as root (sudo)${NC}"
    echo "Example: sudo $0 $*"
    exit 1
fi

# Validate that a backup file was specified before anything else
if [ -z "$BACKUP_FILE" ]; then
    echo -e "${RED}Error: Backup file not specified${NC}"
    usage
    exit 1
fi

# Validate that the backup file exists (local or remote) BEFORE device detection
echo "Validating backup source..."
if [[ "$BACKUP_FILE" == *":"* ]]; then
    # Remote file: Check for existence via SSH
    REMOTE_HOST=$(echo "$BACKUP_FILE" | cut -d: -f1)
    REMOTE_PATH=$(echo "$BACKUP_FILE" | cut -d: -f2)
    if ! ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "test -f '$REMOTE_PATH'"; then
        echo -e "${RED}❌ Error: Remote backup file not found or SSH connection failed.${NC}"
        echo "   Checked for '$REMOTE_PATH' on host '$REMOTE_HOST'."
        echo "   Please verify the path and ensure SSH key-based authentication is set up."
        exit 1
    fi
    echo -e "${GREEN}✅ Remote backup file found.${NC}"
else
    # Local file: Check for existence
    if [ ! -f "$BACKUP_FILE" ]; then
        echo -e "${RED}❌ Error: Local backup file not found: $BACKUP_FILE${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ Local backup file found.${NC}"
fi
echo ""

# Auto-detect device if not specified
if [ -z "$DEVICE" ]; then
    echo -e "${YELLOW}Device not specified. Starting interactive detection...${NC}"
    echo ""

    # Get devices before insertion
    printf "Please ensure the SD card is ${RED}NOT connected${NC}, then press [Enter].\n"
    read -p ""

    BEFORE_DEVICES=$(get_disks)

    echo ""
    printf "Now ${GREEN}connect the SD card${NC}, wait for it to be recognized, then press [Enter].\n"
    read -p ""

    # Wait a moment for the device to register
    sleep 3

    AFTER_DEVICES=$(get_disks)

    # Find the new device
    NEW_DEVICE=$(comm -13 <(echo "$BEFORE_DEVICES") <(echo "$AFTER_DEVICES"))

    NUM_NEW_DEVICES=$(echo "$NEW_DEVICE" | wc -l | xargs)

    if [ "$NUM_NEW_DEVICES" -eq 1 ]; then
        echo -e "${GREEN}✅ Detected new device: $NEW_DEVICE${NC}"
        DEVICE="$NEW_DEVICE"
    elif [ "$NUM_NEW_DEVICES" -eq 0 ]; then
        echo -e "${RED}❌ Error: No new storage device was detected.${NC}"
        echo "Please check the connection and try again."
        exit 1
    else
        echo -e "${RED}❌ Error: Multiple new storage devices were detected:${NC}"
        echo "$NEW_DEVICE"
        echo "Please specify the correct device manually using the --device flag."
        exit 1
    fi
    echo ""
fi

# Check for and handle mounted partitions on the target device
MOUNTED_PARTITIONS=$(lsblk -ln -o NAME,MOUNTPOINT "$DEVICE" | grep -v "^${DEVICE##*/}$" | awk '$2!="" {print "/"$1" mounted on "$2}')
if [ -n "$MOUNTED_PARTITIONS" ]; then
    echo -e "${YELLOW}Warning: The target device has mounted partitions:${NC}"
    echo "$MOUNTED_PARTITIONS"
    echo ""
    read -p "Shall I attempt to unmount them before proceeding? (yes/no): " UNMOUNT_CONFIRM
    if [ "$UNMOUNT_CONFIRM" = "yes" ]; then
        echo "Attempting to unmount..."
        lsblk -ln -o NAME "$DEVICE" | tail -n +2 | while read -r PART; do
            umount "/dev/$PART" 2>/dev/null || true
        done
        # Verify unmount was successful
        sleep 1
        UPDATED_MOUNTS=$(lsblk -ln -o NAME,MOUNTPOINT "$DEVICE" | grep -v "^${DEVICE##*/}$" | awk '$2!="" {print "/"$1}')
        if [ -n "$UPDATED_MOUNTS" ]; then
            echo -e "${RED}❌ Error: Failed to unmount all partitions. Please unmount manually.${NC}"
            exit 1
        else
            echo -e "${GREEN}✅ All partitions unmounted successfully.${NC}"
        fi
    else
        echo "Restore cancelled. Please unmount the partitions manually and try again."
        exit 1
    fi
    echo ""
fi

# Get details for the selected device for user confirmation
echo "Fetching device details..."
DEVICE_DETAILS=$(lsblk -d -n -o SIZE,VENDOR,MODEL "$DEVICE" || echo "N/A N/A N/A")
DEVICE_SIZE_STR=$(echo "$DEVICE_DETAILS" | awk '{print $1}')
DEVICE_VENDOR=$(echo "$DEVICE_DETAILS" | awk '{print $2}')
DEVICE_MODEL=$(echo "$DEVICE_DETAILS" | awk '{print $3}')
PARTITION_INFO=$(lsblk -n -o NAME,SIZE,FSTYPE,MOUNTPOINT "$DEVICE" | tail -n +2 | sed 's/^/    /')

# Attempt to parse required size from backup filename (e.g., argo_..._32GB_....img.gz or .img.7z)
BACKUP_SIZE_TAG=$(echo "$BACKUP_FILE" | grep -oP '_\K[0-9]+GB(?=_)')
if [ -n "$BACKUP_SIZE_TAG" ]; then
    REQUIRED_SIZE_GB=${BACKUP_SIZE_TAG%GB}
    DEVICE_SIZE_BYTES=$(lsblk -b -d -n -o SIZE "$DEVICE")
    DEVICE_SIZE_GB=$((DEVICE_SIZE_BYTES / 1024 / 1024 / 1024))

    echo -e "${GREEN}Backup filename indicates it requires a ${REQUIRED_SIZE_GB}GB card.${NC}"
    
    if (( DEVICE_SIZE_GB < REQUIRED_SIZE_GB )); then
        echo -e "${RED}❌ Error: Target device is too small.${NC}"
        echo "   Backup requires a ${REQUIRED_SIZE_GB}GB card, but target is only ${DEVICE_SIZE_GB}GB."
        exit 1
    else
        echo -e "${GREEN}✅ Target device size is sufficient.${NC}"
    fi
    echo ""
fi

# Safety confirmation
echo ""
echo -e "${RED}═══════════════════════════════════════════════════${NC}"
echo -e "${RED}  ⚠️   DANGER: SD CARD RESTORE OPERATION  ⚠️${NC}"
echo -e "${RED}═══════════════════════════════════════════════════${NC}"
echo ""
echo "This will COMPLETELY DESTROY all data on the following device:"
echo ""
echo -e "${CYAN}  Device:         $DEVICE${NC}"
echo -e "${CYAN}  Size:           $DEVICE_SIZE_STR${NC}"
echo -e "${CYAN}  Vendor / Model: ${DEVICE_VENDOR:-N/A} / ${DEVICE_MODEL:-N/A}${NC} (Note: May be the USB adapter)"
echo ""
echo -e "${CYAN}  Partitions Found:${NC}"
echo -e "${CYAN}    NAME    SIZE FSTYPE MOUNTPOINT${NC}"
echo -e "${CYAN}${PARTITION_INFO}${NC}"
echo ""
echo -e "${YELLOW}The script cannot know the original backup size. You must ensure the target device (${DEVICE_SIZE_STR}) is large enough.${NC}"
echo -e "${YELLOW}Restoring to a smaller device will fail and may corrupt the card.${NC}"
echo ""
echo "Please confirm the device details and size are correct before proceeding."
echo ""
read -p "Type 'YES' to proceed with overwriting the device (anything else will cancel): " CONFIRM

if [ "$CONFIRM" != "YES" ]; then
    echo "Restore cancelled."
    exit 0
fi

# Show what will happen
echo ""
echo "Configuration:"
echo "  Device: $DEVICE"
echo "  Backup: $BACKUP_FILE"
echo ""

# Check if device exists
if [ ! -e "$DEVICE" ]; then
    echo -e "${RED}Error: Device $DEVICE does not exist${NC}"
    exit 1
fi

# Check device size to ensure compatibility
echo "Checking device size..."
# This check is now less critical as details are displayed above, but kept as a basic guardrail
EXISTING_DEVICE_SIZE=$(sudo fdisk -l "$DEVICE" | grep "Disk ${DEVICE}:" | awk '{print $3 $4}' | tr -d ',')
echo "  Device size: $EXISTING_DEVICE_SIZE"
echo ""

# Handle remote files
if [[ "$BACKUP_FILE" == *":"* ]]; then
    # Remote file - download first
    echo "Downloading backup from remote location..."
    REMOTE_FILE="$BACKUP_FILE"
    
    # Determine extension for temp file (remote backups now use .gz, local may use .7z)
    if [[ "$REMOTE_FILE" == *.gz ]]; then
        TMP_FILE="/tmp/argo_restore_temp_$(date +%s).img.gz"
    elif [[ "$REMOTE_FILE" == *.7z ]]; then
        TMP_FILE="/tmp/argo_restore_temp_$(date +%s).img.7z"
    else
        # Default to .gz (primary format for remote backups)
        TMP_FILE="/tmp/argo_restore_temp_$(date +%s).img.gz"
    fi
    
    scp "$REMOTE_FILE" "$TMP_FILE"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: Failed to download backup file${NC}"
        exit 1
    fi
    
    BACKUP_FILE="$TMP_FILE"
    echo "Downloaded to: $BACKUP_FILE"
    echo ""
fi

# Verify backup file exists (this check is now redundant, but harmless)
if [ ! -f "$BACKUP_FILE" ]; then
    echo -e "${RED}Error: Backup file not found: $BACKUP_FILE${NC}"
    exit 1
fi

# Final confirmation
echo ""
echo -e "${RED}⚠️  FINAL WARNING: About to overwrite $DEVICE${NC}"
echo ""
read -p "Proceed with restore? (yes/no): " FINAL_CONFIRM

if [ "$FINAL_CONFIRM" != "yes" ]; then
    echo "Restore cancelled."
    exit 0
fi

# Perform the restore
echo ""
echo -e "${GREEN}Starting SD card restore...${NC}"
echo "========================================="
echo "Device: $DEVICE"
echo "Backup: $BACKUP_FILE"
echo ""
echo "This will take 15-25 minutes..."
echo ""

# Create a temporary file to capture dd's stderr
DD_ERROR_LOG=$(mktemp)

# Enable pipefail to catch errors in any part of the pipe
set -o pipefail

# Decompress and write to device with progress
# Note: Remote backups use .gz (streaming), local backups may use .7z (better compression)
RESTORE_SUCCESS=true
if [[ "$BACKUP_FILE" == *.gz ]]; then
    echo "Decompressing gzip file..."
    if ! ( gunzip -c "$BACKUP_FILE" | pv | sudo dd of="$DEVICE" bs=4M conv=fsync ) 2> "$DD_ERROR_LOG"; then
        RESTORE_SUCCESS=false
    fi
elif [[ "$BACKUP_FILE" == *.7z ]]; then
    if ! command -v 7z &> /dev/null; then
        echo -e "${RED}❌ Error: '7z' command not found.${NC}"
        echo "   Please install p7zip-full to restore .7z archives."
        echo "   Run: sudo apt update && sudo apt install p7zip-full"
        exit 1
    fi
    echo "Decompressing 7z file..."
    if ! ( 7z x -so "$BACKUP_FILE" | pv | sudo dd of="$DEVICE" bs=4M conv=fsync ) 2> "$DD_ERROR_LOG"; then
        RESTORE_SUCCESS=false
    fi
else
    echo -e "${RED}❌ Error: Unsupported backup file format.${NC}"
    echo "   Only .gz (remote backups) and .7z (local backups) compressed images are supported."
    exit 1
fi

# Disable pipefail
set +o pipefail

if [ "$RESTORE_SUCCESS" = true ]; then
    echo ""
    echo -e "${GREEN}✅ Restore completed successfully!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Safely remove the SD card"
    echo "  2. Insert into Orange Pi and boot"
    echo "  3. Verify the system boots correctly"
else
    echo ""
    echo -e "${RED}❌ Restore failed!${NC}"
    # Check for the specific "No space left" error
    if grep -q "No space left on device" "$DD_ERROR_LOG"; then
        echo -e "${YELLOW}   Specific Error: The target SD card is too small for the backup image.${NC}"
        echo "   You must use an SD card that is the same size as or larger than the one the backup was made from."
    else
        echo "   The SD card may be in an inconsistent state."
        echo "   You may need to format and restore again."
        echo "   Error details from dd:"
        cat "$DD_ERROR_LOG"
    fi
    exit 1
fi

# Clean up temporary file if used
if [ -n "$TMP_FILE" ] && [ -f "$TMP_FILE" ]; then
    rm "$TMP_FILE"
    echo "Cleaned up temporary file."
fi
rm -f "$DD_ERROR_LOG"
