#!/bin/bash
# Argo SD Card Backup Script
# Creates a complete backup of the Orange Pi SD card to remote storage
#
# Usage:
#   ./argo_sd_backup.sh                    # Interactive mode
#   ./argo_sd_backup.sh remote_user@host   # Explicit remote destination
#
# The script creates a compressed image of the SD card and transfers it to remote storage.

set -e

# Configuration
SD_DEVICE="/dev/mmcblk0"
BACKUP_DIR="$HOME/sd_backups"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
HOSTNAME=$(hostname)
# Get device size for filename tag
SIZE_BYTES=$(lsblk -b -d -n -o SIZE "$SD_DEVICE")
# Round up to nearest GB for a user-friendly tag
SIZE_GB=$(( (SIZE_BYTES + 1024*1024*1024 - 1) / (1024*1024*1024) ))
SIZE_TAG="${SIZE_GB}GB"
BACKUP_NAME="argo_${HOSTNAME}_${SIZE_TAG}_${TIMESTAMP}.img.7z"

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
    -y, --yes       Non-interactive mode (no prompts, requires destination)
    --rm-local      Remove local backup after successful remote transfer (for non-interactive mode)
    --no-space-check Skip the local disk space check

Examples:
    # Interactive mode (prompts for remote destination)
    ./argo_sd_backup.sh

    # Explicit remote destination
    ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch

    # Local backup only
    ./argo_sd_backup.sh --local

    # Unattended remote backup (immune to SSH hangups)
    nohup ./argo_sd_backup.sh tobi@sensors-tobidh87.lan.ini.uzh.ch -y &

The backup file will be named: argo_${HOSTNAME}_${SIZE_TAG}_YYYYMMDD_HHMMSS.img.7z

EOF
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
        echo "   You need at least ${REQUIRED_SPACE_GB}GB of free space to create the local backup file before transfer."
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
REMOVE_LOCAL_AFTER_TRANSFER=false
SKIP_SPACE_CHECK=false

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
        -y|--yes)
            NON_INTERACTIVE=true
            shift
            ;;
        --rm-local)
            REMOVE_LOCAL_AFTER_TRANSFER=true
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
    echo "   This will take 20-40 minutes depending on network speed"
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
    sudo dd if="$SD_DEVICE" bs=4M status=progress | pv -s 30G | 7z a -t7z -mx=9 -si "$LOCAL_DIR/$BACKUP_NAME"
    
    echo ""
    echo -e "${GREEN}✅ Local backup completed successfully!${NC}"
    echo "Location: $LOCAL_DIR/$BACKUP_NAME"
    ls -lh "$LOCAL_DIR/$BACKUP_NAME"
else
    # Remote backup streamed over SSH
    echo "Streaming backup directly to: $DESTINATION:~/$BACKUP_NAME"
    echo "This process will not use significant local disk space."
    echo "Compression with 7z (ultra) will be slow but space-efficient."
    echo ""
    
    sudo dd if="$SD_DEVICE" bs=4M status=progress | pv -s 30G | 7z a -t7z -mx=9 -si -so | ssh "$DESTINATION" "cat > ~/$BACKUP_NAME"

    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✅ Remote backup streamed successfully!${NC}"
        echo "Remote location: $DESTINATION:~/$BACKUP_NAME"
    else
        echo -e "${RED}❌ Remote backup failed.${NC}"
        echo "   You may need to manually remove the partial file on the remote host."
        exit 1
    fi
fi

# Disable the trap on successful completion
trap - INT TERM

echo ""
echo -e "${GREEN}Backup process completed!${NC}"
