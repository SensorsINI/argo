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
BOLD_RED='\033[1;31m'
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
    --allow-root-target  Allow writing to current root/system disk (dangerous; blocked by default)

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

resolve_target_device() {
    local input="$1"
    local candidate=""
    if [ ! -b "$input" ]; then
        echo -e "${RED}❌ Error: Target device '$input' does not exist or is not a block device.${NC}"
        exit 1
    fi
    candidate=$(lsblk -no PKNAME "$input" 2>/dev/null | head -1)
    if [ -n "$candidate" ] && [ -b "/dev/$candidate" ]; then
        DEVICE="/dev/$candidate"
    else
        DEVICE="$input"
    fi
}

get_root_system_device() {
    local root_source=""
    local root_pkname=""
    root_source=$(findmnt -n -o SOURCE / | head -1)
    root_pkname=$(lsblk -no PKNAME "$root_source" 2>/dev/null | head -1)
    if [ -n "$root_pkname" ] && [ -b "/dev/$root_pkname" ]; then
        echo "/dev/$root_pkname"
    elif [ -b "$root_source" ]; then
        echo "$root_source"
    fi
}

# Metadata / image size (set by load_backup_metadata before target size check)
SOURCE_SIZE_BYTES=""
SOURCE_DEVICE_FROM_META=""
SOURCE_TIMESTAMP_FROM_META=""
REMOTE_HOST=""
REMOTE_PATH=""

extract_size_from_meta() {
    local content="$1"
    if [[ "$content" =~ size_bytes=([0-9]+) ]]; then
        SOURCE_SIZE_BYTES="${BASH_REMATCH[1]}"
    fi
    if [[ "$content" =~ sd_device=([^[:space:]]+) ]]; then
        SOURCE_DEVICE_FROM_META="${BASH_REMATCH[1]}"
    fi
    if [[ "$content" =~ timestamp=([^[:space:]]+) ]]; then
        SOURCE_TIMESTAMP_FROM_META="${BASH_REMATCH[1]}"
    fi
}

# Load companion .meta and/or exact byte size from filename — MUST run before target size check.
load_backup_metadata() {
    SOURCE_SIZE_BYTES=""
    SOURCE_DEVICE_FROM_META=""
    SOURCE_TIMESTAMP_FROM_META=""
    local meta_content=""
    local base=""

    if [[ "$BACKUP_FILE" == *":"* ]]; then
        REMOTE_HOST=$(echo "$BACKUP_FILE" | cut -d: -f1)
        REMOTE_PATH=$(echo "$BACKUP_FILE" | cut -d: -f2)
        base=${REMOTE_PATH%.img.gz}
        base=${base%.img.7z}
        if ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "test -f '${base}.meta'" 2>/dev/null; then
            meta_content=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "cat '${base}.meta'" 2>/dev/null || true)
        fi
    else
        base=${BACKUP_FILE%.img.gz}
        base=${base%.img.7z}
        if [ -f "${base}.meta" ]; then
            meta_content=$(cat "${base}.meta")
        fi
    fi

    if [ -n "$meta_content" ]; then
        extract_size_from_meta "$meta_content"
        echo -e "${GREEN}✅ Loaded backup metadata (${base}.meta).${NC}"
    fi

    if [ -z "$SOURCE_SIZE_BYTES" ]; then
        if [[ "$BACKUP_FILE" =~ _([0-9]+)B_ ]]; then
            SOURCE_SIZE_BYTES="${BASH_REMATCH[1]}"
            echo -e "${GREEN}✅ Parsed source size from filename: ${SOURCE_SIZE_BYTES} bytes.${NC}"
        elif [[ "$BACKUP_FILE" == *":"* ]] && [[ "$REMOTE_PATH" =~ _([0-9]+)B_ ]]; then
            SOURCE_SIZE_BYTES="${BASH_REMATCH[1]}"
            echo -e "${GREEN}✅ Parsed source size from remote path: ${SOURCE_SIZE_BYTES} bytes.${NC}"
        fi
    fi
}

# Parse arguments
BACKUP_FILE=""
DEVICE=""
ALLOW_ROOT_TARGET=false

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
        --allow-root-target)
            ALLOW_ROOT_TARGET=true
            shift
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

# Load .meta and/or exact source size from filename BEFORE any destructive or long-running step.
load_backup_metadata
echo ""

# Determine approximate size of backup and ensure there is sufficient space for temporary storage/decompression.
if [[ "$BACKUP_FILE" == *.gz ]]; then
    if command -v gzip >/dev/null 2>&1; then
        if [[ "$BACKUP_FILE" == *":"* ]]; then
            REMOTE_HOST=$(echo "$BACKUP_FILE" | cut -d: -f1)
            REMOTE_PATH=$(echo "$BACKUP_FILE" | cut -d: -f2)
            REMOTE_SIZE=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "gzip -l '$REMOTE_PATH'" | awk 'NR==2 {print $2}')
            COMPRESSED_SIZE=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "stat -c%s '$REMOTE_PATH'")
        else
            REMOTE_SIZE=0
            COMPRESSED_SIZE=$(stat -c%s "$BACKUP_FILE")
        fi
        if [ -z "$REMOTE_SIZE" ] || [ "$REMOTE_SIZE" -eq 0 ]; then
            if [ "$COMPRESSED_SIZE" -gt 0 ]; then
                REMOTE_SIZE=$((COMPRESSED_SIZE * 3))
            else
                REMOTE_SIZE=0
            fi
        else
            UNCOMPRESSED_SIZE_BYTES=$REMOTE_SIZE
        fi
        COMPRESSED_SIZE_BYTES=$COMPRESSED_SIZE
        REQUIRED_TEMP_BYTES=$((REMOTE_SIZE + COMPRESSED_SIZE + (500*1024*1024)))
        TMP_FREE_BYTES=$(df -P /tmp | awk 'NR==2 {print $4}')
        TMP_FREE_BYTES=$((TMP_FREE_BYTES * 1024))
        if [ "$TMP_FREE_BYTES" -lt "$REQUIRED_TEMP_BYTES" ]; then
            echo -e "${RED}❌ Error: Not enough free space in /tmp for decompression.${NC}"
            echo "   Required approx: $((REQUIRED_TEMP_BYTES / 1024 / 1024)) MB"
            echo "   Available: $((TMP_FREE_BYTES / 1024 / 1024)) MB"
            echo "   Reduce the size of /tmp usage or set TMPDIR to a larger location."
            exit 1
        fi
    fi
elif [[ "$BACKUP_FILE" == *.7z ]]; then
    if command -v 7z >/dev/null 2>&1; then
        if [[ "$BACKUP_FILE" == *":"* ]]; then
            REMOTE_HOST=$(echo "$BACKUP_FILE" | cut -d: -f1)
            REMOTE_PATH=$(echo "$BACKUP_FILE" | cut -d: -f2)
            REMOTE_SIZE=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "7z l '$REMOTE_PATH'" | awk '/^-----------/ {getline; print $3; exit}')
            COMPRESSED_SIZE=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_HOST" "stat -c%s '$REMOTE_PATH'")
        else
            REMOTE_SIZE=$(7z l "$BACKUP_FILE" | awk '/^-----------/ {getline; print $3; exit}')
            COMPRESSED_SIZE=$(stat -c%s "$BACKUP_FILE")
        fi
        UNCOMPRESSED_SIZE_BYTES=$REMOTE_SIZE
        COMPRESSED_SIZE_BYTES=$COMPRESSED_SIZE
        REQUIRED_TEMP_BYTES=$((REMOTE_SIZE + COMPRESSED_SIZE + (1024*1024*1024)))
        TMP_FREE_BYTES=$(df -P /tmp | awk 'NR==2 {print $4}')
        TMP_FREE_BYTES=$((TMP_FREE_BYTES * 1024))
        if [ "$TMP_FREE_BYTES" -lt "$REQUIRED_TEMP_BYTES" ]; then
            echo -e "${RED}❌ Error: Not enough free space in /tmp for decompression.${NC}"
            echo "   Required approx: $((REQUIRED_TEMP_BYTES / 1024 / 1024)) MB"
            echo "   Available: $((TMP_FREE_BYTES / 1024 / 1024)) MB"
            echo "   Reduce the size of /tmp usage or set TMPDIR to a larger location."
            exit 1
        fi
    else
        echo -e "${RED}❌ Error: '7z' command not found.${NC}"
        echo "   Please install p7zip-full to restore .7z archives."
        echo "   Run: sudo apt update && sudo apt install p7zip-full"
        exit 1
    fi
fi

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

    NUM_NEW_DEVICES=$(echo "$NEW_DEVICE" | sed '/^$/d' | wc -l | xargs)

    if [ "$NUM_NEW_DEVICES" -eq 1 ]; then
        DEVICE=$(echo "$NEW_DEVICE")
        echo -e "${GREEN}✅ Detected new device: $DEVICE${NC}"
        echo ""
        echo "Detected device summary:"
        lsblk -d -o NAME,SIZE,MODEL,SERIAL,TYPE,TRAN "$DEVICE"
        echo ""
        echo "Existing partitions on $DEVICE:"
        lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT "$DEVICE"
    elif [ "$NUM_NEW_DEVICES" -eq 0 ]; then
        echo -e "${RED}❌ Error: No new storage device was detected.${NC}"
        echo "Please check the connection and try again."
        echo "Hint: Disconnect all removable drives, run the script again,"
        echo "      then connect the SD card when prompted."
        exit 1
    else
        echo -e "${YELLOW}Multiple new storage devices were detected:${NC}"
        mapfile -t NEW_DEVICE_LIST < <(echo "$NEW_DEVICE" | sed '/^$/d')
        idx=0
        for dev in "${NEW_DEVICE_LIST[@]}"; do
            idx=$((idx + 1))
            mounts=$(lsblk -nrpo MOUNTPOINT "$dev" 2>/dev/null | awk 'NF && !seen[$0]++ {print $0}')
            if [ -n "$mounts" ]; then
                mounts_joined=$(printf "%s" "$mounts" | paste -sd ',' -)
            else
                mounts_joined="(no mountpoints)"
            fi
            echo "  [$idx] $dev ($(lsblk -dn -o SIZE "$dev" 2>/dev/null)) mounts: $mounts_joined"
            if [ "$mounts_joined" != "(no mountpoints)" ]; then
                while IFS= read -r mp; do
                    [ -z "$mp" ] && continue
                    df -h "$mp" 2>/dev/null | awk 'NR==2 {printf "      df: %s size=%s used=%s avail=%s use=%s\n", $6, $2, $3, $4, $5}'
                done <<< "$mounts"
            fi
        done
        echo ""
        read -p "Enter selection number: " PICK
        if ! [[ "$PICK" =~ ^[0-9]+$ ]] || [ "$PICK" -lt 1 ] || [ "$PICK" -gt "${#NEW_DEVICE_LIST[@]}" ]; then
            echo -e "${RED}❌ Error: Invalid selection.${NC}"
            exit 1
        fi
        DEVICE="${NEW_DEVICE_LIST[$((PICK - 1))]}"
        echo -e "${GREEN}✅ Selected device: $DEVICE${NC}"
    fi
    echo ""
fi

# If a partition device was passed, normalize to parent disk.
resolve_target_device "$DEVICE"

# Safety guard: never write to currently running system disk unless explicitly allowed.
ROOT_SYSTEM_DEVICE=$(get_root_system_device)
if [ -n "$ROOT_SYSTEM_DEVICE" ] && [ "$DEVICE" = "$ROOT_SYSTEM_DEVICE" ] && [ "$ALLOW_ROOT_TARGET" = false ]; then
    echo -e "${RED}❌ Error: Refusing to restore to current root/system disk: $DEVICE${NC}"
    echo "   This is almost certainly the computer you are currently running on."
    echo "   Use --device with your SD card target (e.g., /dev/sde or /dev/sde1)."
    echo "   If intentional, pass --allow-root-target."
    exit 1
fi

if [ -n "$ROOT_SYSTEM_DEVICE" ] && [ "$DEVICE" = "$ROOT_SYSTEM_DEVICE" ] && [ "$ALLOW_ROOT_TARGET" = true ]; then
    echo -e "${YELLOW}⚠️  WARNING: --allow-root-target enabled. You are restoring to current root/system disk: $DEVICE${NC}"
fi

# Check for and handle mounted partitions on the target device
MOUNTED_PARTITIONS=$(lsblk -ln -o NAME,MOUNTPOINT "$DEVICE" | grep -v "^${DEVICE##*/}$" | awk '$2!="" {print "/"$1" " $2}')
if [ -n "$MOUNTED_PARTITIONS" ]; then
    echo -e "${YELLOW}Warning: The target device has mounted partitions. The restore must run on an unmounted SD card to avoid corrupting the live filesystem.${NC}"
    echo "$MOUNTED_PARTITIONS"
    echo ""
    while true; do
        MOUNTED_PARTITIONS=$(lsblk -ln -o NAME,MOUNTPOINT "$DEVICE" | grep -v "^${DEVICE##*/}$" | awk '$2!="" {print "/"$1" " $2}')
        if [ -z "$MOUNTED_PARTITIONS" ]; then
            break
        fi
        echo "Current mounts:"
        echo "$MOUNTED_PARTITIONS"
        echo ""
        read -p "Attempt to unmount automatically? (yes/manual/quit): " UNMOUNT_MODE
        if [ "$UNMOUNT_MODE" = "quit" ]; then
            echo "Restore cancelled. Please unmount the partitions manually and try again."
            exit 1
        elif [ "$UNMOUNT_MODE" = "manual" ]; then
            echo "Please unmount the partitions manually, then press [Enter] to re-check."
            read -p ""
            continue
        else
            echo "Attempting to unmount..."
            lsblk -ln -o NAME "$DEVICE" | tail -n +2 | while read -r PART; do
                umount "/dev/$PART" 2>/dev/null || true
            done
            sleep 1
            UPDATED_MOUNTS=$(lsblk -ln -o NAME,MOUNTPOINT "$DEVICE" | grep -v "^${DEVICE##*/}$" | awk '$2!="" {print "/"$1" " $2}')
            if [ -n "$UPDATED_MOUNTS" ]; then
                echo -e "${RED}❌ Unable to unmount all partitions automatically.${NC}"
                echo "Processes holding the device:"
                echo "$UPDATED_MOUNTS" | while read -r ENTRY; do
                    MP=$(echo "$ENTRY" | awk '{print $2}')
                    sudo lsof +f -- "$MP" 2>/dev/null || true
                done
                echo "Please close the above processes and unmount manually. Press [Enter] to retry or type 'quit' to abort."
                read USER_CHOICE
                if [ "$USER_CHOICE" = "quit" ]; then
                    echo "Restore cancelled."
                    exit 1
                fi
            else
                echo -e "${GREEN}✅ All partitions unmounted successfully.${NC}"
            fi
        fi
    done
    echo ""
fi

# Get details for the selected device for user confirmation
echo "Fetching device details..."
DEVICE_DETAILS=$(lsblk -d -n -o SIZE,VENDOR,MODEL "$DEVICE" || echo "N/A N/A N/A")
DEVICE_SIZE_STR=$(echo "$DEVICE_DETAILS" | awk '{print $1}')
DEVICE_VENDOR=$(echo "$DEVICE_DETAILS" | awk '{print $2}')
DEVICE_MODEL=$(echo "$DEVICE_DETAILS" | awk '{print $3}')
PARTITION_INFO=$(lsblk -n -o NAME,SIZE,FSTYPE,MOUNTPOINT "$DEVICE" | tail -n +2 | sed 's/^/    /')

# Required image size: prefer exact bytes from metadata/filename (loaded earlier).
# Fallback: marketing "NNGB" in filename (approximate; two same-rated cards can still differ).
REQUIRED_SIZE_BYTES=""
BACKUP_SIZE_TAG=""
if echo "$BACKUP_FILE" | grep -qE '_[0-9]+GB_'; then
    BACKUP_SIZE_TAG=$(echo "$BACKUP_FILE" | sed -n 's/.*_\([0-9][0-9]*\)GB_.*/\1/p' | head -1)
fi
SOURCE_SIZE_GIB=""
if [ -n "$SOURCE_SIZE_BYTES" ]; then
    REQUIRED_SIZE_BYTES=$SOURCE_SIZE_BYTES
    SOURCE_SIZE_GIB=$(awk "BEGIN {printf \"%.2f\", $SOURCE_SIZE_BYTES/1024/1024/1024}")
    echo "Source image exact size (from metadata/filename): $SOURCE_SIZE_BYTES bytes (~${SOURCE_SIZE_GIB} GiB)"
elif [ -n "$BACKUP_SIZE_TAG" ]; then
    REQUIRED_SIZE_GB=$BACKUP_SIZE_TAG
    REQUIRED_SIZE_BYTES=$((REQUIRED_SIZE_GB * 1024 * 1024 * 1024))
    SOURCE_SIZE_GIB=$(awk "BEGIN {printf \"%.2f\", $REQUIRED_SIZE_BYTES/1024/1024/1024}")
    echo -e "${YELLOW}⚠️  No exact byte size in metadata/filename; using filename tag ${REQUIRED_SIZE_GB}GB → ${REQUIRED_SIZE_BYTES} bytes (approx).${NC}"
    echo "   Prefer a companion .meta file or a filename containing _NNNNNNNNNNB_ for a strict check."
else
    echo -e "${YELLOW}⚠️  Could not determine image size from metadata or filename; skipping strict size pre-check.${NC}"
fi

# Compare target capacity to required image size BEFORE confirmations (uses kernel block size).
if [ -n "$REQUIRED_SIZE_BYTES" ]; then
    if ! command -v blockdev >/dev/null 2>&1; then
        echo -e "${RED}❌ Error: blockdev not found; cannot verify target size.${NC}"
        exit 1
    fi
    DEVICE_SIZE_BYTES=$(blockdev --getsize64 "$DEVICE")
    DEVICE_SIZE_GIB=$(awk "BEGIN {printf \"%.2f\", $DEVICE_SIZE_BYTES/1024/1024/1024}")

    if [ -n "$SOURCE_SIZE_BYTES" ]; then
        # Strict: exact raw image length must fit on target (same check dd will eventually enforce).
        if (( DEVICE_SIZE_BYTES < REQUIRED_SIZE_BYTES )); then
            SHORT=$((REQUIRED_SIZE_BYTES - DEVICE_SIZE_BYTES))
            echo ""
            echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
            echo -e "${BOLD_RED}  FATAL: TARGET DEVICE IS TOO SMALL FOR THIS BACKUP IMAGE${NC}"
            echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
            echo ""
            echo "  Image size (from metadata/filename): ${REQUIRED_SIZE_BYTES} bytes (~${SOURCE_SIZE_GIB} GiB)"
            echo "  Target ${DEVICE} capacity (blockdev):   ${DEVICE_SIZE_BYTES} bytes (~${DEVICE_SIZE_GIB} GiB)"
            echo "  Short by: ${SHORT} bytes — restore would fail at the end with \"No space left on device\"."
            echo "  Use a card with equal or larger raw capacity than the source disk, or restore to a larger card."
            echo ""
            exit 1
        fi
        echo -e "${GREEN}✅ Target capacity OK: ${DEVICE_SIZE_BYTES} bytes >= image ${REQUIRED_SIZE_BYTES} bytes.${NC}"
    else
        TOLERANCE_BYTES=$((REQUIRED_SIZE_BYTES * 95 / 100))
        if (( DEVICE_SIZE_BYTES < TOLERANCE_BYTES )); then
            echo ""
            echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
            echo -e "${BOLD_RED}  FATAL: TARGET DEVICE LOOKS TOO SMALL (APPROXIMATE CHECK)${NC}"
            echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
            echo "  Approximate required: ${REQUIRED_SIZE_BYTES} bytes (~${SOURCE_SIZE_GIB} GiB)"
            echo "  Target ${DEVICE}: ${DEVICE_SIZE_BYTES} bytes (~${DEVICE_SIZE_GIB} GiB)"
            echo "  Add a .meta file or use a filename with _NNNNNNNNNNB_ for an exact byte comparison."
            echo ""
            exit 1
        fi
        echo -e "${GREEN}✅ Target capacity passes approximate check.${NC}"
    fi
    echo ""
fi

# Show parsed source metadata (if available) before destructive confirmation.
echo ""
echo "Backup metadata summary:"
if [ -n "$SOURCE_DEVICE_FROM_META" ]; then
    echo "  Source device (from meta): $SOURCE_DEVICE_FROM_META"
else
    echo "  Source device (from meta): N/A"
fi
if [ -n "$SOURCE_SIZE_BYTES" ]; then
    SOURCE_SIZE_MIB=$(awk "BEGIN {printf \"%.1f\", $SOURCE_SIZE_BYTES/1024/1024}")
    SOURCE_SIZE_GIB=$(awk "BEGIN {printf \"%.2f\", $SOURCE_SIZE_BYTES/1024/1024/1024}")
    echo "  Source size (from meta/name): ${SOURCE_SIZE_BYTES} bytes (${SOURCE_SIZE_MIB} MiB / ${SOURCE_SIZE_GIB} GiB)"
else
    echo "  Source size (from meta/name): N/A"
fi
if [ -n "$SOURCE_TIMESTAMP_FROM_META" ]; then
    echo "  Backup timestamp (from meta): $SOURCE_TIMESTAMP_FROM_META"
else
    echo "  Backup timestamp (from meta): N/A"
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
if [ -n "$SOURCE_SIZE_BYTES" ]; then
    echo -e "${GREEN}Size pre-check: image length ${SOURCE_SIZE_BYTES} bytes fits on ${DEVICE} (verified above).${NC}"
else
    echo -e "${YELLOW}Exact image size unknown: confirm the target (${DEVICE_SIZE_STR}) is at least as large as the source card.${NC}"
    echo -e "${YELLOW}Restoring to a smaller device will fail and may corrupt the card.${NC}"
fi
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
echo "Expected duration: ~45 minutes at ~12 MB/s write rate"
echo "Note: Restoring a live-system backup may require fsck cleanup; script will attempt this automatically."
echo ""

START_TIME=$(date +%s)
DD_ERROR_LOG=$(mktemp)
LOG_PV=$(mktemp)
RESTORE_SUCCESS=false

cleanup() {
    trap - INT TERM
    echo -e "\n${YELLOW}⚠️  Restore interrupted. Cleaning up...${NC}"
    if [ -n "$RESTORE_PGID" ]; then
        kill -- -"$RESTORE_PGID" 2>/dev/null || true
    else
        kill -- -$$ 2>/dev/null || true
    fi
    sync
    if [ -n "$TMP_FILE" ] && [ -f "$TMP_FILE" ]; then
        rm -f "$TMP_FILE"
    fi
    rm -f "$DD_ERROR_LOG" "$LOG_PV"
    exit 1
}

trap cleanup INT TERM

PV_ARGS="-f -N Restore -i 30" # only newline every 30s
if [ -n "$UNCOMPRESSED_SIZE_BYTES" ] && [ "$UNCOMPRESSED_SIZE_BYTES" -gt 0 ]; then
    PV_ARGS="-f -N Restore -s $UNCOMPRESSED_SIZE_BYTES"
fi
PV_ENV="PV_LINE_MODE=1"
if [[ "$BACKUP_FILE" == *.gz ]]; then
    echo "Decompressing and writing SD card (gzip)..."
    set +o pipefail
    ( set -o pipefail; gunzip -c "$BACKUP_FILE" \
        | env $PV_ENV pv $PV_ARGS \
        | sudo dd of="$DEVICE" bs=4M conv=fsync status=progress \
            2> >(tee "$DD_ERROR_LOG" >&2) ) &
    RESTORE_PID=$!
    RESTORE_PGID=$(ps -o pgid= "$RESTORE_PID" | tr -d ' ')
    wait $RESTORE_PID
    RC=$?
    if [ $RC -ne 0 ]; then
        RESTORE_SUCCESS=false
        kill -- -"$RESTORE_PGID" 2>/dev/null || true
    else
        if [ -s "$DD_ERROR_LOG" ]; then
            FILTERED_ERRORS=$(grep -Ev '(^[0-9]+\+[0-9]+ records (in|out)$|^[0-9]+ bytes .*copied.*$|Restore:)' "$DD_ERROR_LOG")
            if [ -n "$FILTERED_ERRORS" ]; then
                RESTORE_SUCCESS=false
            else
                RESTORE_SUCCESS=true
                : > "$DD_ERROR_LOG"
            fi
        else
            RESTORE_SUCCESS=true
        fi
    fi
    set -o pipefail
elif [[ "$BACKUP_FILE" == *.7z ]]; then
    if ! command -v 7z &> /dev/null; then
        echo -e "${RED}❌ Error: '7z' command not found.${NC}"
        echo "   Please install p7zip-full to restore .7z archives."
        echo "   Run: sudo apt update && sudo apt install p7zip-full"
        exit 1
    fi
    echo "Decompressing and writing SD card (7z)..."
    set +o pipefail
    ( set -o pipefail; 7z x -so "$BACKUP_FILE" \
        | env $PV_ENV pv $PV_ARGS \
        | sudo dd of="$DEVICE" bs=4M conv=fsync status=progress \
            2> >(tee "$DD_ERROR_LOG" >&2) ) &
    RESTORE_PID=$!
    RESTORE_PGID=$(ps -o pgid= "$RESTORE_PID" | tr -d ' ')
    wait $RESTORE_PID
    RC=$?
    if [ $RC -ne 0 ]; then
        RESTORE_SUCCESS=false
        kill -- -"$RESTORE_PGID" 2>/dev/null || true
    else
        if [ -s "$DD_ERROR_LOG" ]; then
            FILTERED_ERRORS=$(grep -Ev '(^[0-9]+\+[0-9]+ records (in|out)$|^[0-9]+ bytes .*copied.*$|Restore:)' "$DD_ERROR_LOG")
            if [ -n "$FILTERED_ERRORS" ]; then
                RESTORE_SUCCESS=false
            else
                RESTORE_SUCCESS=true
                : > "$DD_ERROR_LOG"
            fi
        else
            RESTORE_SUCCESS=true
        fi
    fi
    set -o pipefail
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
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    printf "Restore duration: %d min %d sec\n" $((ELAPSED/60)) $((ELAPSED%60))
    echo "Running partprobe to inform kernel of updated partition table..."
    sudo partprobe "$DEVICE" || true
    echo "Running fsck on ext partitions to ensure clean boot..."
    lsblk -ln -o NAME,FSTYPE "$DEVICE" | tail -n +2 | while read -r PART FST; do
        if [ "$FST" = "ext4" ] || [ "$FST" = "ext3" ] || [ "$FST" = "ext2" ]; then
            echo "  Checking /dev/$PART..."
            sudo fsck -fy "/dev/$PART"
        fi
    done
    echo "If fsck reported unresolved errors, consider re-imaging or creating a fresh backup while the source system is offline."
    echo ""
    echo "Next steps:"
    echo "  1. Safely remove the SD card"
    echo "  2. Insert into Orange Pi and boot"
    echo "  3. Verify the system boots correctly"
else
    echo ""
    echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD_RED}  RESTORE FAILED — DO NOT ASSUME THE CARD IS USABLE${NC}"
    echo -e "${BOLD_RED}════════════════════════════════════════════════════════════${NC}"
    echo ""
    # Check for the specific "No space left" error
    if grep -q "No space left on device" "$DD_ERROR_LOG" 2>/dev/null; then
        echo -e "${BOLD_RED}ERROR: Target medium ran out of space (image larger than device capacity).${NC}"
        echo "   The card may be partially written and inconsistent — treat it as corrupt until re-imaged."
        echo "   Use a card with raw capacity ≥ the backup image size (see .meta size_bytes or filename _...B_)."
    else
        echo -e "${BOLD_RED}ERROR: Restore did not complete successfully.${NC}"
        echo "   The SD card may be in an inconsistent state."
        echo "   You may need to format and restore again."
        if [ -s "$DD_ERROR_LOG" ]; then
            echo ""
            echo "Captured stderr from dd/pipeline:"
            tail -n +1 "$DD_ERROR_LOG"
        else
            echo "   No additional error output was captured."
        fi
    fi
    echo ""
    exit 1
fi

# Clean up temporary file if used
if [ -n "$TMP_FILE" ] && [ -f "$TMP_FILE" ]; then
    rm "$TMP_FILE"
    echo "Cleaned up temporary file."
fi
rm -f "$DD_ERROR_LOG"
trap - INT TERM
