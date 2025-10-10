#!/bin/bash
# Wrapper to run IMU plotting without conda interference

# Deactivate conda if active
if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
    conda deactivate
fi

# Run IMU with venv
cd "$(dirname "$0")"
source .venv/bin/activate
./imu.py --plot_calib "$@"

