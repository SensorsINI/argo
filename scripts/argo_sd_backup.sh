#!/bin/bash
# Argo SD Card Backup Script
# Creates a complete backup of the Orange Pi SD card to remote storage
#
# Usage:
#   ./argo_sd_backup.sh                    # Interactive mode
#   ./argo_sd_backup.sh remote_user@host   # Explicit remote destination
#   ./argo_sd_backup.sh --local --source-device /dev/sde1
#
# The script creates a compressed image of the SD card and transfers it to remote storage.

set -e

# Configuration
BACKUP_DIR="$HOME/sd_backups"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
HOSTNAME=$(hostname)
START_TIME=$(date +%s)
# Default remote destination for self-backups on Argo OPi.
# Override with ARGO_SD_BACKUP_REMOTE_DEST env var.
DEFAULT_REMOTE_DEST="${ARGO_SD_BACKUP_REMOTE_DEST:-tobi@sensors-tobidh87.lan.ini.uzh.ch}"
SOURCE_DEVICE=""
SOURCE_MOUNT=""
SD_DEVICE=""
SIZE_BYTES=""
SIZE_GB=""
SIZE_TAG=""
BACKUP_BYTES_TAG=""
BACKUP_BASE=""
BACKUP_NAME=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Cleanup function to run on exit/interrupt
cleanup() {
    echo -e "\n${YELLOW}⚠️  Backup interrupted. Cleaning up...${NC}"
    # For local backups, a partial file might exist.
    if [ "$LOCAL_ONLY" = true ]; then
        echo "   Removing partial local backup: $LOCAL_DIR/$BACKUP_NAME"
        rm -f "$LOCAL_DIR/$BACKUP_NAME"
    else
        echo "   For remote backups, you may need to manually remove the partial file on the remote host:"
        echo "   $DESTINATION:~/$BACKUP_NAME"
    fi
    exit 1
}

# Trap signals to ensure cleanup
trap cleanup INT TERM

create_meta_content() {
    cat <<EOF
size_bytes=$SIZE_BYTES
size_gb=$SIZE_GB
sd_device=$SD_DEVICE
timestamp=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
$(lsblk -b -o NAME,SIZE,FSTYPE,MOUNTPOINT "$SD_DEVICE")
EOF
}

# Get disk size for progress bar
get_disk_size() {
    sudo fdisk -l "$SD_DEVICE" | grep "Disk ${SD_DEVICE}:" | awk '{print $3 $4}' | tr -d ','
}

# Check if 7z is installed
check_7z_installed() {
    if ! command -v 7z &> /dev/null; then
        echo -e "${RED}❌ Error: '7z' command not found.${NC}"
        echo "   Please install p7zip-full package to use this script."
        echo "   Run: sudo apt update && sudo apt install p7zip-full"
        exit 1
    fi
}

# Check if SSH connection to remote host is possible
check_ssh_connection() {
    DEST=$1
    echo "Checking SSH connection to $DEST..."
    if ssh -o BatchMode=yes -o ConnectTimeout=10 "$DEST" exit 2>/dev/null; then
        echo -e "${GREEN}✅ SSH connection successful${NC}"
    else
        echo -e "${RED}❌ Error: SSH connection to $DEST failed.${NC}"
        echo "   Please check the following:"
        echo "   - The user and hostname are correct."
        echo "   - You have SSH key-based authentication set up (no password prompt)."
        echo "   - The remote host is reachable on the network."
        exit 1
    fi
}

# Show usage
usage() {
    cat << EOF
Argo SD Card Backup Script
==========================

Creates a complete compressed backup of the Orange Pi SD card.

Usage:
    ./argo_sd_backup.sh [remote_user@host] [options]

Options:
    -h, --help      Show this help message
    -l, --local     Save backup locally only (no remote transfer)
    -d, --destination PATH    Local backup directory (default: ~/sd_backups)
    -s, --source-device DEV   Source device to back up (partition or disk, e.g. /dev/sde1 or /dev/sde)
    -m, --source-mount PATH   Source mountpoint to back up (e.g. /media/tobi/argo_sd)
    --allow-root-source Allow using current root/system disk as source (normally blocked on non-Argo hosts)
    -y, --yes       Non-interactive mode (no prompts, requires destination)
    --no-space-check Skip the local disk space check

Examples:
    # Interactive mode (prompts for remote destination)
    ./argo_sd_backup.sh

    # Explicit remote destination (streams directly, no local copy)
    ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

    # Local backup only
    ./argo_sd_backup.sh --local

    # Local backup from mounted SD partition (/dev/sdX may change each time)
    ./argo_sd_backup.sh --local --source-device /dev/sde1

    # Local backup by mountpoint
    ./argo_sd_backup.sh --local --source-mount /media/tobi/argo_sd

    # Explicitly allow backing up current root disk (advanced use)
    ./argo_sd_backup.sh --allow-root-source

    # Unattended remote backup (streams directly via SSH)
    nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y &

The backup file will be named: argo_${HOSTNAME}_${SIZE_TAG}_YYYYMMDD_HHMMSS.img.7z

EOF
}

resolve_source_device() {
    local input="$1"
    local candidate=""

    if [ ! -b "$input" ]; then
        echo -e "${RED}❌ Error: Source device '$input' does not exist or is not a block device.${NC}"
        exit 1
    fi

    # If a partition is provided, resolve to parent disk.
    candidate=$(lsblk -no PKNAME "$input" 2>/dev/null | head -1)
    if [ -n "$candidate" ] && [ -b "/dev/$candidate" ]; then
        SD_DEVICE="/dev/$candidate"
    else
        SD_DEVICE="$input"
    fi

    if [ ! -b "$SD_DEVICE" ]; then
        echo -e "${RED}❌ Error: Could not resolve source disk from '$input'.${NC}"
        exit 1
    fi
}

auto_detect_source_device() {
    local root_source=""
    local root_pkname=""
    local root_device=""
    local disk=""
    local candidate_count=0
    local selected_index=0
    local candidates=()
    local i=0
    local mounts=""
    local mp=""

    # Resolve current root filesystem parent disk so we never select it on desktop hosts.
    root_source=$(findmnt -n -o SOURCE / | head -1)
    root_pkname=$(lsblk -no PKNAME "$root_source" 2>/dev/null | head -1)
    if [ -n "$root_pkname" ] && [ -b "/dev/$root_pkname" ]; then
        root_device="/dev/$root_pkname"
    elif [ -b "$root_source" ]; then
        root_device="$root_source"
    fi

    if is_argo_opi_host; then
        # On Argo OPi, default to backing up the active system disk.
        if [ -z "$root_device" ] || [ ! -b "$root_device" ]; then
            echo -e "${RED}❌ Error: Could not auto-detect Argo system disk.${NC}"
            echo "   Tried root filesystem parent: $root_device"
            echo "   Use --source-device /dev/mmcblk0"
            exit 1
        fi
        SD_DEVICE="$root_device"
        return
    fi

    # On non-Argo hosts, never default to the root disk.
    # Collect likely external/removable disks.
    while IFS= read -r disk; do
        [ -z "$disk" ] && continue
        if [ "$disk" != "$root_device" ]; then
            candidates+=("$disk")
        fi
    done < <(
        lsblk -pn -d -o NAME,TYPE,RM,TRAN | awk '
            $2=="disk" {
                name=$1; rm=$3; tr=$4;
                # Prefer explicit removable drives, plus common external buses.
                if (rm=="1" || tr=="usb" || tr=="mmc") print name;
            }
        '
    )

    candidate_count=${#candidates[@]}

    if [ "$candidate_count" -eq 0 ]; then
        echo -e "${RED}❌ Error: Could not auto-detect external SD card source on this host.${NC}"
        echo "   Root/system disk is: $root_device"
        echo "   Please specify source explicitly:"
        echo "   --source-device /dev/sdX1  or  --source-mount /path/to/mount"
        exit 1
    fi

    if [ "$candidate_count" -eq 1 ]; then
        SD_DEVICE="${candidates[0]}"
        return
    fi

    # Multiple candidates: require explicit choice for safety.
    if [ "$NON_INTERACTIVE" = true ]; then
        echo -e "${RED}❌ Error: Multiple external disks detected; refusing to guess in non-interactive mode.${NC}"
        for disk in "${candidates[@]}"; do
            echo "   Candidate: $disk"
        done
        echo "   Use --source-device /dev/sdX1 or --source-mount /path/to/mount"
        exit 1
    fi

    echo ""
    echo -e "${YELLOW}Multiple external/removable disks detected.${NC}"
    echo "Select source disk to back up:"
    for disk in "${candidates[@]}"; do
        i=$((i + 1))
        mounts=$(lsblk -nrpo MOUNTPOINT "$disk" 2>/dev/null | awk 'NF && !seen[$0]++ {print $0}')
        if [ -n "$mounts" ]; then
            mounts=$(printf "%s" "$mounts" | paste -sd ',' -)
        else
            mounts="(no mountpoints)"
        fi

        echo "  [$i] $disk ($(lsblk -dn -o SIZE "$disk" 2>/dev/null)) mounts: $mounts"

        # Show df-style context for each mountpoint to make selection unambiguous.
        if [ "$mounts" != "(no mountpoints)" ]; then
            while IFS= read -r mp; do
                [ -z "$mp" ] && continue
                df -h "$mp" 2>/dev/null | awk 'NR==2 {printf "      df: %s size=%s used=%s avail=%s use=%s\n", $6, $2, $3, $4, $5}'
            done < <(lsblk -nrpo MOUNTPOINT "$disk" 2>/dev/null | awk 'NF && !seen[$0]++ {print $0}')
        fi
    done
    echo ""
    read -p "Enter selection number: " selected_index

    if ! [[ "$selected_index" =~ ^[0-9]+$ ]] || [ "$selected_index" -lt 1 ] || [ "$selected_index" -gt "$candidate_count" ]; then
        echo -e "${RED}❌ Error: Invalid selection.${NC}"
        exit 1
    fi

    SD_DEVICE="${candidates[$((selected_index - 1))]}"
}

is_argo_opi_host() {
    local model=""
    local uname_r=""
    local uname_m=""
    local armbian=false
    local opi_model=false
    local opi_kernel=false
    local arm_arch=false
    local argo_layout=false

    # Hardware model can be exposed in either location depending on kernel.
    if [ -r /proc/device-tree/model ]; then
        model=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)
    elif [ -r /sys/firmware/devicetree/base/model ]; then
        model=$(tr -d '\0' < /sys/firmware/devicetree/base/model 2>/dev/null || true)
    fi

    uname_r=$(uname -r 2>/dev/null || true)
    uname_m=$(uname -m 2>/dev/null || true)

    if [ -f /etc/armbian-release ]; then
        armbian=true
    elif [ -r /etc/os-release ] && grep -Eqi '(^ID=armbian$|Armbian)' /etc/os-release >/dev/null 2>&1; then
        armbian=true
    fi

    if echo "$model" | grep -Eqi 'orange[[:space:]]*pi' >/dev/null 2>&1; then
        opi_model=true
    fi

    if echo "$uname_r" | grep -Eqi 'orangepi|sunxi' >/dev/null 2>&1; then
        opi_kernel=true
    fi

    if echo "$uname_m" | grep -Eqi '^(arm|aarch64)' >/dev/null 2>&1; then
        arm_arch=true
    fi

    # Additional Argo deployment hint on target board.
    if [ -d /home/orangepi/argo ] || [ "$HOSTNAME" = "orangepi" ]; then
        argo_layout=true
    fi

    # Robust decision:
    # - Prefer Armbian + Orange Pi hardware/kernel on ARM.
    # - Require at least one Argo-specific hint to avoid false positives on other Orange Pi systems.
    if [ "$armbian" = true ] && [ "$arm_arch" = true ] && { [ "$opi_model" = true ] || [ "$opi_kernel" = true ]; } && [ "$argo_layout" = true ]; then
        return 0
    fi

    return 1
}

# Check if running as root (should NOT be run as root for safety)
check_not_root() {
    if [ "$EUID" -eq 0 ]; then
        echo -e "${RED}❌ Error: This script should NOT be run as root (sudo)${NC}"
        echo "Running as root could corrupt the SD card or create dangerous backups."
        echo "The script will use sudo for specific commands that require it."
        exit 1
    fi
}

# Check for sufficient disk space (only for local backups)
check_disk_space() {
    # 7z has better compression, so we can lower the required space estimate
    REQUIRED_SPACE_GB=12
    AVAILABLE_SPACE_GB=$(df -BG "$LOCAL_DIR" | awk 'NR==2 {print $4}' | sed 's/G//')

    echo "Checking local disk space..."
    echo "  Required: ~${REQUIRED_SPACE_GB}GB"
    echo "  Available in $LOCAL_DIR: ${AVAILABLE_SPACE_GB}GB"

    if (( $(echo "$AVAILABLE_SPACE_GB < $REQUIRED_SPACE_GB" | bc -l) )); then
        echo -e "${RED}❌ Error: Not enough disk space in $LOCAL_DIR${NC}"
        echo "   You need at least ${REQUIRED_SPACE_GB}GB of free space to create the local backup file."
        echo "   Clean up files or use a different backup directory with the -d flag."
        exit 1
    else
        echo -e "${GREEN}✅ Disk space check passed${NC}"
    fi
}

# Parse arguments
DESTINATION=""
LOCAL_ONLY=false
LOCAL_DIR="$BACKUP_DIR"
NON_INTERACTIVE=false
SKIP_SPACE_CHECK=false
ALLOW_ROOT_SOURCE=false
IS_ARGO_OPI=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -l|--local)
            LOCAL_ONLY=true
            shift
            ;;
        -d|--destination)
            LOCAL_DIR="$2"
            shift 2
            ;;
        -s|--source-device)
            SOURCE_DEVICE="$2"
            shift 2
            ;;
        -m|--source-mount)
            SOURCE_MOUNT="$2"
            shift 2
            ;;
        --allow-root-source)
            ALLOW_ROOT_SOURCE=true
            shift
            ;;
        -y|--yes)
            NON_INTERACTIVE=true
            shift
            ;;
        --no-space-check)
            SKIP_SPACE_CHECK=true
            shift
            ;;
        *)
            if [ -z "$DESTINATION" ]; then
                DESTINATION="$1"
            else
                echo -e "${RED}Error: Multiple destinations specified${NC}"
                exit 1
            fi
            shift
            ;;
    esac
done

# Validate source arguments
if [ -n "$SOURCE_DEVICE" ] && [ -n "$SOURCE_MOUNT" ]; then
    echo -e "${RED}Error: Use only one of --source-device or --source-mount.${NC}"
    exit 1
fi

if is_argo_opi_host; then
    IS_ARGO_OPI=true
fi

# On Argo OPi, default to remote self-backup when no destination was provided.
if [ "$LOCAL_ONLY" = false ] && [ -z "$DESTINATION" ] && [ "$IS_ARGO_OPI" = true ]; then
    DESTINATION="$DEFAULT_REMOTE_DEST"
    echo "Detected Argo Orange Pi environment."
    echo "Using default remote destination: $DESTINATION"
fi

# Resolve source device
if [ -n "$SOURCE_MOUNT" ]; then
    MOUNT_SOURCE=$(findmnt -n -o SOURCE --target "$SOURCE_MOUNT" 2>/dev/null || true)
    if [ -z "$MOUNT_SOURCE" ]; then
        echo -e "${RED}❌ Error: Could not resolve device from mountpoint '$SOURCE_MOUNT'.${NC}"
        exit 1
    fi
    resolve_source_device "$MOUNT_SOURCE"
elif [ -n "$SOURCE_DEVICE" ]; then
    resolve_source_device "$SOURCE_DEVICE"
else
    auto_detect_source_device
fi

# Safety guard: on non-Argo hosts, block root/system disk backup unless explicitly allowed.
ROOT_SOURCE=$(findmnt -n -o SOURCE / | head -1)
ROOT_PKNAME=$(lsblk -no PKNAME "$ROOT_SOURCE" 2>/dev/null | head -1)
ROOT_SYSTEM_DEVICE=""
if [ -n "$ROOT_PKNAME" ] && [ -b "/dev/$ROOT_PKNAME" ]; then
    ROOT_SYSTEM_DEVICE="/dev/$ROOT_PKNAME"
elif [ -b "$ROOT_SOURCE" ]; then
    ROOT_SYSTEM_DEVICE="$ROOT_SOURCE"
fi

if [ -n "$ROOT_SYSTEM_DEVICE" ] && [ "$SD_DEVICE" = "$ROOT_SYSTEM_DEVICE" ] && [ "$IS_ARGO_OPI" = false ] && [ "$ALLOW_ROOT_SOURCE" = false ]; then
    echo -e "${RED}❌ Error: Refusing to back up current root/system disk on non-Argo host: $SD_DEVICE${NC}"
    echo "   This is a safety check to prevent imaging your desktop/laptop boot drive."
    echo "   Choose external SD source with --source-device or --source-mount."
    echo "   If you really want this, pass --allow-root-source."
    exit 1
fi

if [ "$ALLOW_ROOT_SOURCE" = true ] && [ -n "$ROOT_SYSTEM_DEVICE" ] && [ "$SD_DEVICE" = "$ROOT_SYSTEM_DEVICE" ] && [ "$IS_ARGO_OPI" = false ]; then
    echo -e "${YELLOW}⚠️  WARNING: --allow-root-source enabled. Using current root/system disk as source: $SD_DEVICE${NC}"
fi

# Build size-dependent backup naming now that source is resolved
SIZE_BYTES=$(lsblk -b -d -n -o SIZE "$SD_DEVICE")
if [ -z "$SIZE_BYTES" ]; then
    echo -e "${RED}❌ Error: Could not read size for source device '$SD_DEVICE'.${NC}"
    exit 1
fi
SIZE_GB=$(( (SIZE_BYTES + 1024*1024*1024 - 1) / (1024*1024*1024) ))
SIZE_TAG="${SIZE_GB}GB"
BACKUP_BYTES_TAG="${SIZE_BYTES}B"
BACKUP_BASE="argo_${HOSTNAME}_${SIZE_TAG}_${BACKUP_BYTES_TAG}_${TIMESTAMP}"
BACKUP_NAME="${BACKUP_BASE}.img.7z"

# Validate arguments for non-interactive mode
if [ "$NON_INTERACTIVE" = true ] && [ -z "$DESTINATION" ] && [ "$LOCAL_ONLY" = false ]; then
    echo -e "${RED}Error: Destination (user@host) or --local must be specified in non-interactive mode (-y)${NC}"
    exit 1
fi

# Check dependencies
check_7z_installed

# Check not running as root
check_not_root

# Create local backup directory
mkdir -p "$LOCAL_DIR"

# Check disk space before starting (local only)
if [ "$LOCAL_ONLY" = true ] && [ "$SKIP_SPACE_CHECK" = false ]; then
    check_disk_space
fi

# If this is a remote backup, check the SSH connection first
if [ "$LOCAL_ONLY" = false ] && [ -n "$DESTINATION" ]; then
    check_ssh_connection "$DESTINATION"
fi

# If interactive mode, prompt for remote destination
if [ -z "$DESTINATION" ] && [ "$LOCAL_ONLY" = false ] && [ "$NON_INTERACTIVE" = false ]; then
    echo "Argo SD Card Backup"
    echo "==================="
    echo ""
    echo "Current system: $HOSTNAME"
    echo "SD Card: $SD_DEVICE ($(get_disk_size))"
    echo ""
    read -p "Enter remote destination (user@host) or 'local' for local only: " DESTINATION_INPUT
    
    if [ "$DESTINATION_INPUT" = "local" ]; then
        LOCAL_ONLY=true
    else
        DESTINATION=$DESTINATION_INPUT
        # Check the connection now that we have it from the user
        check_ssh_connection "$DESTINATION"
    fi
fi

# Confirm before proceeding
if [ "$NON_INTERACTIVE" = false ]; then
    echo ""
    echo -e "${YELLOW}⚠️  WARNING: This will create a backup of the entire SD card${NC}"
    echo "   Backup size: ~30GB (compressed to ~10-15GB)"
    echo "   Expected duration: up to ~3 hours over Wi-Fi (≈2–3 MB/s observed)"
    echo "   For a clean snapshot, perform the backup while the system is idle or offline"
    echo ""
    read -p "Continue? (yes/no): " CONFIRM

    if [ "$CONFIRM" != "yes" ]; then
        echo "Backup cancelled."
        exit 0
    fi
fi

# Estimate time and show progress
echo ""
echo -e "${GREEN}Starting SD card backup...${NC}"
echo "========================================="
echo "SD Device: $SD_DEVICE"
echo "Backup name: $BACKUP_NAME"
echo ""

# Start the backup process with progress monitoring
if [ "$LOCAL_ONLY" = true ]; then
    # Local backup only
    echo "Saving to: $LOCAL_DIR/$BACKUP_NAME"
    echo "Compression with 7z (ultra) will be slow but space-efficient."
    echo ""
    # Reduce pv progress updates to once per minute to avoid excessive newlines in logs
    sudo dd if="$SD_DEVICE" bs=4M status=progress | pv -i 60 -s "$SIZE_BYTES" | 7z a -t7z -mx=9 -si "$LOCAL_DIR/$BACKUP_NAME"
    
    echo ""
    echo -e "${GREEN}✅ Local backup completed successfully!${NC}"
    echo "Location: $LOCAL_DIR/$BACKUP_NAME"
    ls -lh "$LOCAL_DIR/$BACKUP_NAME"
    META_FILE="$LOCAL_DIR/${BACKUP_BASE}.meta"
    create_meta_content > "$META_FILE"
    echo "Metadata: $META_FILE"
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    printf "Backup duration: %d min %d sec\n" $((ELAPSED/60)) $((ELAPSED%60))
else
    # Remote backup streamed over SSH
    # Note: Using gzip instead of 7z because p7zip doesn't support -so (stdout) on this system
    BACKUP_NAME_GZ="${BACKUP_NAME%.7z}.img.gz"
    echo "Streaming backup directly to: $DESTINATION:~/$BACKUP_NAME_GZ"
    echo "This process will not use significant local disk space."
    echo "Compression with gzip will be fast and space-efficient."
    echo ""
    
    # Reduce pv progress updates to once per minute to avoid excessive newlines in logs
    sudo dd if="$SD_DEVICE" bs=4M status=progress | pv -i 60 -s "$SIZE_BYTES" | gzip -9 | ssh "$DESTINATION" "cat > ~/$BACKUP_NAME_GZ"

    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✅ Remote backup streamed successfully!${NC}"
        echo "Remote location: $DESTINATION:~/$BACKUP_NAME_GZ"
        META_REMOTE="${BACKUP_BASE}.meta"
        META_CONTENT=$(create_meta_content)
        printf "%s\n" "$META_CONTENT" | ssh "$DESTINATION" "cat > ~/$META_REMOTE"
        echo "Metadata: $DESTINATION:~/$META_REMOTE"
    else
        echo -e "${RED}❌ Remote backup failed.${NC}"
        echo "   You may need to manually remove the partial file on the remote host."
        exit 1
    fi
fi

# Disable the trap on successful completion
trap - INT TERM

echo ""
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))
printf "Backup duration: %d min %d sec\n" $((ELAPSED/60)) $((ELAPSED%60))
echo -e "${GREEN}Backup process completed!${NC}"
