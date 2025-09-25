#!/bin/bash
"""
Argo Status GUI Launcher
========================

Shell wrapper for the Argo Status Desktop GUI application.
Ensures proper environment setup and launches the GUI.

Usage: ./argo_status_gui.sh
"""

set -e  # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚢 Starting Argo Status GUI...${NC}"

# Check if we're in the right directory
if [[ ! -f "$SCRIPT_DIR/argo_status_gui.py" ]]; then
    echo -e "${RED}❌ Error: argo_status_gui.py not found in $SCRIPT_DIR${NC}"
    echo -e "${YELLOW}Make sure you're running this script from the launch directory${NC}"
    exit 1
fi

# Source ROS2 environment if available
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    echo -e "${GREEN}📦 Sourcing ROS2 Humble environment...${NC}"
    source /opt/ros/humble/setup.bash
elif [[ -f "$HOME/.bashrc" ]]; then
    echo -e "${YELLOW}⚠️  ROS2 Humble not found, sourcing ~/.bashrc...${NC}"
    source "$HOME/.bashrc"
fi

# Source local ROS2 workspace if it exists
if [[ -f "$ARGO_ROOT/install/setup.bash" ]]; then
    echo -e "${GREEN}🔧 Sourcing local Argo workspace...${NC}"
    source "$ARGO_ROOT/install/setup.bash"
fi

# Set environment variables
export ROS_VERSION=2
export ROS_DISTRO=humble
export ROS_PYTHON_VERSION=3
export ROS_LOCALHOST_ONLY=0
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Check Python dependencies
echo -e "${BLUE}🔍 Checking Python dependencies...${NC}"

# Check if tkinter is available
if ! python3 -c "import tkinter" 2>/dev/null; then
    echo -e "${RED}❌ Error: tkinter not available${NC}"
    echo -e "${YELLOW}Install with: sudo apt-get install python3-tk${NC}"
    exit 1
fi

# Check if required Argo modules are available
cd "$SCRIPT_DIR"
if ! python3 -c "from argo_node_utils import ArgoNodeManager" 2>/dev/null; then
    echo -e "${RED}❌ Error: Required Argo modules not found${NC}"
    echo -e "${YELLOW}Make sure argo_node_utils.py is present${NC}"
    exit 1
fi

# Check if argo_status_check.py is available (optional - GUI will show deprecation message if not)
if ! python3 -c "from argo_status_check import OptimizedArgoStatusChecker" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Warning: argo_status_check.py not found${NC}"
    echo -e "${YELLOW}⚠️  This GUI is deprecated. Use 'argo_status' command instead.${NC}"
    echo -e "${YELLOW}⚠️  The GUI will show limited functionality.${NC}"
fi

echo -e "${GREEN}✅ All dependencies satisfied${NC}"

# Launch the GUI
echo -e "${BLUE}🚀 Launching Argo Status GUI...${NC}"
echo -e "${YELLOW}💡 Tip: Use Ctrl+C to close the application${NC}"
echo ""

# Run the GUI application
exec python3 "$SCRIPT_DIR/argo_status_gui.py" "$@"
