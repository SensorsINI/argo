#!/bin/bash
# Argo Bag Re-recording Script
# Adds visualization markers to an existing bag file for direct Foxglove import
#
# Usage:
#   ./argo_rerecord_bag.sh                                    # Re-record latest recording (100x speed)
#   ./argo_rerecord_bag.sh bags/argo_20251105_141014/        # Re-record specific recording
#   ./argo_rerecord_bag.sh bags/argo_20251105_141014/ output_name  # Specify output name
#   ./argo_rerecord_bag.sh bags/argo_20251105_141014/ output_name --no-sailing  # With options
#   ./argo_rerecord_bag.sh bags/argo_20251105_141014/ output_name --rate 50.0  # Custom playback rate
#   ./argo_rerecord_bag.sh bags/argo_20251105_141014/ output_name -y  # Skip confirmation prompt
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
    echo -e "${YELLOW}⚠️  Warning: Re-recording while Argo is running may create${NC}"
    echo -e "   duplicate publishers for topics, which may cause conflicts."
    echo ""
    read -p "Stop argo_launch_standard.service and continue with re-recording? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}⚠️  Re-recording cancelled. Argo service is still running.${NC}"
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
        echo -e "   Please stop it manually before running re-recording"
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
    # Default output name: input name with " (rerecorded)" appended
    DEFAULT_OUTPUT_BAG="${BAG_NAME} (rerecorded)"
    echo -e "${CYAN}📦 Argo Bag Re-recording${NC}"
    echo -e "${CYAN}========================${NC}"
    echo -e "Selected recording: ${GREEN}$BAG_NAME${NC}"
    echo -e "Source: ${CYAN}$SELECTED_BAG${NC}"
else
    # No argument provided - find the latest physical recording
    # Exclude re-recordings that end with " (rerecorded)"
    # Filter out directories containing " (rerecorded)" in their name
    LATEST_BAG=$(find "$BAGS_DIR" -maxdepth 1 -type d -name "argo_*" -printf '%T@ %p\n' 2>/dev/null | \
        grep -v " (rerecorded)" | sort -n | tail -1 | cut -d' ' -f2-)
    
    if [ -z "$LATEST_BAG" ]; then
        echo -e "${RED}❌ Error: No recording bags found in $BAGS_DIR${NC}"
        echo -e "   (excluding re-recordings ending with ' (rerecorded)')"
        exit 1
    fi
    
    SELECTED_BAG="$LATEST_BAG"
    BAG_NAME=$(basename "$SELECTED_BAG")
    # Default output name: input name with " (rerecorded)" appended
    DEFAULT_OUTPUT_BAG="${BAG_NAME} (rerecorded)"
    echo -e "${CYAN}📦 Argo Latest Recording Re-recording${NC}"
    echo -e "${CYAN}=====================================${NC}"
    echo -e "Latest recording: ${GREEN}$BAG_NAME${NC}"
    echo -e "Source: ${CYAN}$SELECTED_BAG${NC}"
fi

echo ""

# Get bag size
BAG_SIZE=$(du -sh "$SELECTED_BAG" 2>/dev/null | cut -f1 || echo "Unknown")
echo -e "${CYAN}📊 Input bag size: ${GREEN}$BAG_SIZE${NC}"
echo ""

# Parse output bag name and options
# Default to input name + " (rerecorded)" if not specified
OUTPUT_BAG=""

# Check if first argument is a bag path (not an option)
if [ -n "$1" ] && [[ "$1" != --* ]]; then
    # First arg is bag path, check if second arg is output name (not an option)
    if [ -n "$2" ] && [[ "$2" != --* ]]; then
        OUTPUT_BAG="$2"
        shift 2  # Remove both bag path and output name
    else
        shift 1  # Remove only bag path
    fi
fi

# Use default output name if not specified
if [ -z "$OUTPUT_BAG" ]; then
    OUTPUT_BAG="$DEFAULT_OUTPUT_BAG"
fi

# Parse command line arguments (options like --no-sailing, --rate)
USE_SAILING_AREA="true"
USE_VISUALIZATION="true"
USE_TRANSFORM="true"
PLAYBACK_RATE="100.0"  # Default to 100x speed for maximum throughput
SKIP_CONFIRMATION="false"

while [[ $# -gt 0 ]]; do
    case $1 in
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
        --rate)
            if [ -n "$2" ] && [[ "$2" =~ ^[0-9]+\.?[0-9]*$ ]]; then
                PLAYBACK_RATE="$2"
                shift 2
            else
                echo -e "${RED}❌ Error: --rate requires a numeric value${NC}"
                echo "   Example: --rate 50.0"
                exit 1
            fi
            ;;
        -y|--yes)
            SKIP_CONFIRMATION="true"
            shift
            ;;
        *)
            echo -e "${YELLOW}⚠️  Unknown option: $1${NC}"
            echo "Available options: --no-sailing, --no-visualization, --no-transform, --rate <value>, -y/--yes"
            shift
            ;;
    esac
done

# Build launch command with proper path handling
# Use absolute path to handle spaces in bag file names
LAUNCH_CMD="ros2 launch $ARGO_DIR/scripts/argo_bag_rerecord.py"
LAUNCH_CMD="$LAUNCH_CMD input_bag:=\"$SELECTED_BAG\""
LAUNCH_CMD="$LAUNCH_CMD output_bag:=\"$OUTPUT_BAG\""
LAUNCH_CMD="$LAUNCH_CMD use_sailing_area:=$USE_SAILING_AREA"
LAUNCH_CMD="$LAUNCH_CMD use_visualization:=$USE_VISUALIZATION"
LAUNCH_CMD="$LAUNCH_CMD use_transform:=$USE_TRANSFORM"
LAUNCH_CMD="$LAUNCH_CMD playback_rate:=$PLAYBACK_RATE"

# Display what will be done
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}📦 Re-recording Plan${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "   ${CYAN}Input bag:${NC}  ${GREEN}$SELECTED_BAG${NC}"
echo -e "   ${CYAN}Input size:${NC} ${GREEN}$BAG_SIZE${NC}"
echo ""
echo -e "   ${CYAN}Output bag:${NC} ${GREEN}$BAGS_DIR/$OUTPUT_BAG${NC}"
echo ""
echo -e "   ${CYAN}Configuration:${NC}"
echo -e "      • Sailing area:          ${GREEN}$USE_SAILING_AREA${NC}"
echo -e "      • Visualization markers: ${GREEN}$USE_VISUALIZATION${NC}"
echo -e "      • Transform publisher:   ${GREEN}$USE_TRANSFORM${NC}"
echo -e "      • Playback rate:         ${GREEN}${PLAYBACK_RATE}x${NC} ${CYAN}(maximum speed)${NC}"
echo ""
echo -e "   ${CYAN}Process:${NC}"
echo -e "      1. Play back the original bag file at ${GREEN}${PLAYBACK_RATE}x${NC} speed"
echo -e "      2. Run visualization nodes to generate markers"
echo -e "      3. Record everything (original topics + visualization) to new bag"
echo -e "      4. Stop automatically when playback completes"
echo ""
echo -e "   ${CYAN}Output format:${NC} ${GREEN}MCAP${NC} (configurable via nodes/record.yaml)"
echo ""
echo -e "${YELLOW}💡 Tip: The output bag can be imported directly into Foxglove Studio${NC}"
echo -e "${YELLOW}   No need to run foxglove_bridge - just import the bag file!${NC}"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Check for MCAP plugin dependency before proceeding
echo -e "${CYAN}🔍 Checking dependencies...${NC}"
if ! ros2 bag record -s mcap --help >/dev/null 2>&1; then
    echo -e "${RED}❌ ERROR: MCAP storage plugin is not installed!${NC}"
    echo ""
    echo -e "The bag re-recording requires the MCAP storage plugin for ros2 bag record."
    echo -e "Install it with:"
    echo ""
    echo -e "   ${CYAN}source /opt/ros/humble/setup.bash${NC}"
    echo -e "   ${CYAN}make install-rosbag2-mcap${NC}"
    echo ""
    echo -e "Or install all ROS2 dependencies:"
    echo ""
    echo -e "   ${CYAN}source /opt/ros/humble/setup.bash${NC}"
    echo -e "   ${CYAN}make install-deps${NC}"
    echo ""
    echo -e "After installation, try the re-recording again."
    exit 1
fi
echo -e "${GREEN}✅ MCAP storage plugin is available${NC}"
echo ""

# Confirmation prompt (unless -y/--yes flag is set)
if [ "$SKIP_CONFIRMATION" != "true" ]; then
    read -p "Proceed with re-recording? (Y/n): " -n 1 -r
    echo ""
    # Default to Yes (empty reply or Y/y), only exit on explicit N/n
    if [[ $REPLY =~ ^[Nn]$ ]]; then
        echo -e "${YELLOW}⚠️  Re-recording cancelled by user${NC}"
        exit 0
    fi
    echo ""
fi

# Silently remove existing re-recording if it exists (to allow overwriting)
OUTPUT_PATH="$BAGS_DIR/$OUTPUT_BAG"
if [ -d "$OUTPUT_PATH" ]; then
    echo -e "${CYAN}🗑️  Removing previous re-recording: ${GREEN}$OUTPUT_BAG${NC}"
    rm -rf "$OUTPUT_PATH"
fi

echo -e "${CYAN}▶️  Starting re-recording...${NC}"
echo ""

# Execute the launch command
eval "$LAUNCH_CMD"
EXIT_CODE=$?

# Display summary after completion
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}📦 Re-recording Summary${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✅ Re-recording completed successfully!${NC}"
else
    echo -e "${RED}❌ Re-recording completed with errors (exit code: $EXIT_CODE)${NC}"
fi
echo ""
echo -e "   Input bag:  ${CYAN}$SELECTED_BAG${NC}"
echo -e "   Output bag: ${GREEN}$BAGS_DIR/$OUTPUT_BAG${NC}"
echo ""
if [ $EXIT_CODE -eq 0 ]; then
    # Check if output bag exists
    if [ -d "$BAGS_DIR/$OUTPUT_BAG" ]; then
        OUTPUT_SIZE=$(du -sh "$BAGS_DIR/$OUTPUT_BAG" 2>/dev/null | cut -f1 || echo "Unknown")
        echo -e "   Output size: ${GREEN}$OUTPUT_SIZE${NC}"
        echo ""
        echo -e "${YELLOW}💡 Next steps:${NC}"
        echo -e "   • Import the output bag directly into Foxglove Studio"
        echo -e "   • Open Foxglove Studio → Open data source → Local file"
        echo -e "   • Select: ${CYAN}$BAGS_DIR/$OUTPUT_BAG${NC}"
    else
        echo -e "${RED}❌ Error: Output bag folder not found at expected location${NC}"
        echo -e "   Expected: ${CYAN}$BAGS_DIR/$OUTPUT_BAG${NC}"
        echo ""
        echo -e "${YELLOW}⚠️  The re-recording process reported success, but the output bag was not created.${NC}"
        echo -e "${YELLOW}   This may indicate:${NC}"
        echo -e "   • The bag recording process failed silently"
        echo -e "   • The output path was incorrect"
        echo -e "   • The bag was created in a different location"
        echo ""
        # Try to find the bag in case it was created elsewhere
        FOUND_BAG=$(find "$BAGS_DIR" -maxdepth 1 -type d -name "*${OUTPUT_BAG}*" 2>/dev/null | head -1)
        if [ -n "$FOUND_BAG" ]; then
            echo -e "${CYAN}💡 Found similar bag at: ${GREEN}$FOUND_BAG${NC}"
        else
            echo -e "${CYAN}💡 Searching for recently created bags...${NC}"
            RECENT_BAGS=$(find "$BAGS_DIR" -maxdepth 1 -type d -name "argo_*" -newer "$SELECTED_BAG" 2>/dev/null | head -3)
            if [ -n "$RECENT_BAGS" ]; then
                echo -e "${CYAN}   Recent bags found:${NC}"
                echo "$RECENT_BAGS" | while read -r bag; do
                    echo -e "   ${GREEN}$bag${NC}"
                done
            fi
        fi
        # Update exit code to indicate failure
        EXIT_CODE=1
    fi
fi
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

exit $EXIT_CODE

