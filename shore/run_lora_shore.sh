#!/bin/bash
# Launcher script for shore-side LoRa node
# Handles conda environments and ROS2 sourcing automatically

# Deactivate conda if active
if [ -n "$CONDA_DEFAULT_ENV" ] || [ -n "$CONDA_PREFIX" ]; then
    echo "Deactivating conda environment: $CONDA_DEFAULT_ENV"
    # Use conda's deactivate function if available, otherwise unset variables
    if type conda &>/dev/null; then
        eval "$(conda shell.bash hook)"
        conda deactivate 2>/dev/null || true
    else
        # Manually unset conda variables
        unset CONDA_DEFAULT_ENV
        unset CONDA_PREFIX
        unset CONDA_PROMPT_MODIFIER
        unset CONDA_SHLVL
        unset CONDA_PYTHON_EXE
        # Restore original PATH (remove conda paths)
        if [ -n "$_CONDA_BACKUP_PATH" ]; then
            export PATH="$_CONDA_BACKUP_PATH"
        fi
    fi
fi

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "Sourcing ROS2 Humble environment..."
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/"
    echo "Please install ROS2 Humble first. See INSTALL.md"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the shore node with any provided arguments
echo "Starting shore-side LoRa node..."
echo ""
exec python3 "$SCRIPT_DIR/lora_shore.py" "$@"

