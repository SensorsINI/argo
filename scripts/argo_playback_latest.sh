#!/bin/bash
# Argo Latest Recording Playback Script
# Finds the latest recording bag and launches playback with visualization support
#
# Usage:
#   ./argo_playback_latest.sh                                    # Play latest recording
#   ./argo_playback_latest.sh bags/argo_20251105_141014/        # Play specific recording
#   ./argo_playback_latest.sh bags/argo_20251105_141014 --no-foxglove  # With options
#   ./argo_playback_latest.sh --no-foxglove                     # Latest with options
#
# Note: Bash tab completion works for bag folder paths!
#
# This script works from any directory and finds the Argo project root automatically.

# Note: We don't use 'set -e' here because we want to handle user prompts gracefully

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Find Argo project root (directory containing bags/ folder)
# Works by finding the script's location and navigating to project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BAGS_DIR="$ARGO_DIR/bags"

# Check if bags directory exists
if [ ! -d "$BAGS_DIR" ]; then
    echo -e "${RED}❌ Error: Bags directory not found: $BAGS_DIR${NC}"
    echo -e "   Make sure you're running this from the Argo project directory"
    exit 1
fi

# Check if argo_launch_standard.service is running
if systemctl is-active --quiet argo_launch_standard.service 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Argo launch service is currently running${NC}"
    echo -e "   Service: ${CYAN}argo_launch_standard.service${NC}"
    echo ""
    echo -e "${YELLOW}⚠️  Warning: Playing back a recording while Argo is running will create${NC}"
    echo -e "   duplicate publishers for topics, which may cause conflicts."
    echo ""
    read -p "Stop argo_launch_standard.service and continue with playback? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}⚠️  Playback cancelled. Argo service is still running.${NC}"
        echo -e "   To stop manually: ${CYAN}aq${NC} or ${CYAN}sudo systemctl stop argo_launch_standard.service${NC}"
        exit 0
    fi
    
    echo -e "${CYAN}🛑 Stopping argo_launch_standard.service...${NC}"
    if sudo systemctl stop argo_launch_standard.service; then
        echo -e "${GREEN}✅ Argo service stopped successfully${NC}"
        echo ""
        # Wait a moment for services to fully stop
        sleep 2
    else
        echo -e "${RED}❌ Error: Failed to stop argo_launch_standard.service${NC}"
        echo -e "   Please stop it manually before running playback"
        exit 1
    fi
fi

# Parse bag folder argument (optional)
if [ -n "$1" ] && [[ "$1" != --* ]]; then
    # User provided a bag folder path (not an option starting with --)
    if [[ "$1" == /* ]]; then
        # Absolute path
        BAG_PATH="$1"
    elif [[ "$1" == bags/* ]] || [[ "$1" == */bags/* ]]; then
        # Path starts with "bags/" - resolve relative to project root
        BAG_PATH="$ARGO_DIR/$1"
    else
        # Relative path - resolve relative to bags directory
        BAG_PATH="$BAGS_DIR/$1"
    fi
    
    # Remove trailing slash if present
    BAG_PATH="${BAG_PATH%/}"
    
    # Check if the path exists and is a directory
    if [ ! -d "$BAG_PATH" ]; then
        echo -e "${RED}❌ Error: Bag folder not found: $BAG_PATH${NC}"
        echo -e "   Make sure the path is correct and the folder exists"
        exit 1
    fi
    
    # Check if it looks like a bag folder (contains .db3 files or metadata.yaml)
    if [ ! -f "$BAG_PATH/metadata.yaml" ] && [ -z "$(find "$BAG_PATH" -maxdepth 1 -name "*.db3" 2>/dev/null)" ]; then
        echo -e "${YELLOW}⚠️  Warning: $BAG_PATH doesn't appear to be a valid bag folder${NC}"
        echo -e "   (missing metadata.yaml or .db3 files)"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 0
        fi
    fi
    
    SELECTED_BAG="$BAG_PATH"
    BAG_NAME=$(basename "$SELECTED_BAG")
    echo -e "${CYAN}📦 Argo Recording Playback${NC}"
    echo -e "${CYAN}===========================${NC}"
    echo -e "Selected recording: ${GREEN}$BAG_NAME${NC}"
    echo -e "Source: ${CYAN}$SELECTED_BAG${NC}"
else
    # No argument provided - find the latest recording
    LATEST_BAG=$(find "$BAGS_DIR" -maxdepth 1 -type d -name "argo_*" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
    
    if [ -z "$LATEST_BAG" ]; then
        echo -e "${RED}❌ Error: No recording bags found in $BAGS_DIR${NC}"
        exit 1
    fi
    
    SELECTED_BAG="$LATEST_BAG"
    BAG_NAME=$(basename "$SELECTED_BAG")
    echo -e "${CYAN}📦 Argo Latest Recording Playback${NC}"
    echo -e "${CYAN}==================================${NC}"
    echo -e "Latest recording: ${GREEN}$BAG_NAME${NC}"
    echo -e "Source: ${CYAN}$SELECTED_BAG${NC}"
fi

echo ""

# Get bag size
BAG_SIZE=$(du -sh "$SELECTED_BAG" 2>/dev/null | cut -f1 || echo "Unknown")
echo -e "${CYAN}📊 Bag size: ${GREEN}$BAG_SIZE${NC}"
echo ""

# Parse command line arguments (options like --no-foxglove)
# Skip first argument if it was a bag path
USE_FOXGLOVE="true"
USE_SAILING_AREA="true"
USE_VISUALIZATION="true"
USE_TRANSFORM="true"

# Shift arguments if first one was a bag path
if [ -n "$1" ] && [[ "$1" != --* ]]; then
    shift  # Remove bag path argument, now $1 is first option
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-foxglove)
            USE_FOXGLOVE="false"
            shift
            ;;
        --no-sailing)
            USE_SAILING_AREA="false"
            shift
            ;;
        --no-visualization)
            USE_VISUALIZATION="false"
            shift
            ;;
        --no-transform)
            USE_TRANSFORM="false"
            shift
            ;;
        *)
            echo -e "${YELLOW}⚠️  Unknown option: $1${NC}"
            echo "Available options: --no-foxglove, --no-sailing, --no-visualization, --no-transform"
            shift
            ;;
    esac
done

echo -e "${CYAN}🚀 Launching bag playback with visualization...${NC}"
echo -e "   Foxglove bridge: ${GREEN}$USE_FOXGLOVE${NC}"
echo -e "   Sailing area: ${GREEN}$USE_SAILING_AREA${NC}"
echo -e "   Visualization: ${GREEN}$USE_VISUALIZATION${NC}"
echo -e "   Transform publisher: ${GREEN}$USE_TRANSFORM${NC}"
echo ""

# Build launch command with proper path handling
# Use absolute path to handle spaces in bag file names
LAUNCH_CMD="ros2 launch $ARGO_DIR/launch/argo_bag_playback.py"
LAUNCH_CMD="$LAUNCH_CMD bag_file:=\"$SELECTED_BAG\""
LAUNCH_CMD="$LAUNCH_CMD use_foxglove:=$USE_FOXGLOVE"
LAUNCH_CMD="$LAUNCH_CMD use_sailing_area:=$USE_SAILING_AREA"
LAUNCH_CMD="$LAUNCH_CMD use_visualization:=$USE_VISUALIZATION"
LAUNCH_CMD="$LAUNCH_CMD use_transform:=$USE_TRANSFORM"

echo -e "${CYAN}📡 Connect Foxglove to: ${GREEN}ws://localhost:8765${NC}"
echo ""
echo -e "${CYAN}💡 Tip: If connecting from another computer, use your machine's IP:${NC}"
echo -e "   ${CYAN}ws://$(hostname -I | awk '{print $1}'):8765${NC}"
echo ""
echo -e "${CYAN}▶️  Starting playback...${NC}"
echo ""

# Execute the launch command
eval "$LAUNCH_CMD"
