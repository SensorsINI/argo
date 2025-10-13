#!/bin/bash
# /usr/local/bin/argo-cpu-setup.sh

set -e

# Set conservative governor for all CPUs
for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
    if [ -f "${cpu}/cpufreq/scaling_governor" ]; then
        echo "conservative" > "${cpu}/cpufreq/scaling_governor"
    fi
done

# Set freq_step for the conservative governor, if the file exists
if [ -f "/sys/devices/system/cpu/cpufreq/conservative/freq_step" ]; then
    echo 5 > /sys/devices/system/cpu/cpufreq/conservative/freq_step
fi
