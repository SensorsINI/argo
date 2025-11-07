#!/bin/bash
# Argo Recording Backup Script
# Finds the latest recording bag and SCPs it to remote storage as a tgz archive
#
# Usage:
#   ./argo_backup_latest_recording.sh                    # Use default destination
#   ./argo_backup_latest_recording.sh user@host:/path    # Custom destination
#
# The script finds the latest recording in ~/argo/bags, creates a tgz archive,
# and transfers it to the remote location.

set -e

# Configuration
BAGS_DIR="$HOME/argo/bags"
DEFAULT_DEST="tobi@tobidh87:/home/tobi/Dropbox/GitHub/SensorsINI/argo/bags"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Cleanup function
cleanup() {
    if [ -n "$TEMP_ARCHIVE" ] && [ -f "$TEMP_ARCHIVE" ]; then
        echo -e "\n${YELLOW}⚠️  Cleaning up temporary archive...${NC}"
        rm -f "$TEMP_ARCHIVE"
    fi
}
trap cleanup EXIT INT TERM

# Check if bags directory exists
if [ ! -d "$BAGS_DIR" ]; then
    echo -e "${RED}❌ Error: Bags directory not found: $BAGS_DIR${NC}"
    exit 1
fi

# Find the latest recording directory
LATEST_BAG=$(find "$BAGS_DIR" -maxdepth 1 -type d -name "argo_*" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)

if [ -z "$LATEST_BAG" ]; then
    echo -e "${RED}❌ Error: No recording bags found in $BAGS_DIR${NC}"
    exit 1
fi

# Get just the directory name (without path)
BAG_NAME=$(basename "$LATEST_BAG")

# Determine destination
if [ -n "$1" ]; then
    DEST="$1"
else
    DEST="$DEFAULT_DEST"
fi

# Extract remote host and path
if [[ "$DEST" == *":"* ]]; then
    REMOTE_HOST=$(echo "$DEST" | cut -d: -f1)
    REMOTE_PATH=$(echo "$DEST" | cut -d: -f2-)
else
    echo -e "${RED}❌ Error: Invalid destination format. Expected: user@host:/path${NC}"
    exit 1
fi

# Create archive name
ARCHIVE_NAME="${BAG_NAME}.tgz"
TEMP_ARCHIVE="/tmp/${ARCHIVE_NAME}"

echo -e "${CYAN}📦 Argo Recording Backup${NC}"
echo -e "${CYAN}========================${NC}"
echo -e "Latest recording: ${GREEN}$BAG_NAME${NC}"
echo -e "Source: ${CYAN}$LATEST_BAG${NC}"
echo -e "Destination: ${CYAN}$DEST${NC}"
echo ""

# Get bag size
BAG_SIZE=$(du -sh "$LATEST_BAG" | cut -f1)
echo -e "${CYAN}📊 Bag size: ${GREEN}$BAG_SIZE${NC}"
echo ""

# Create tgz archive
echo -e "${CYAN}🗜️  Creating archive: $ARCHIVE_NAME${NC}"
cd "$BAGS_DIR"
tar -czf "$TEMP_ARCHIVE" "$BAG_NAME"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Error: Failed to create archive${NC}"
    exit 1
fi

ARCHIVE_SIZE=$(du -sh "$TEMP_ARCHIVE" | cut -f1)
echo -e "${GREEN}✅ Archive created: ${ARCHIVE_SIZE}${NC}"
echo ""

# Transfer to remote
echo -e "${CYAN}📤 Transferring to ${REMOTE_HOST}...${NC}"
scp "$TEMP_ARCHIVE" "$DEST/"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Error: Failed to transfer archive${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✅ Successfully backed up recording!${NC}"
echo -e "   Remote location: ${CYAN}$DEST/$ARCHIVE_NAME${NC}"
