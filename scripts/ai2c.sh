#!/bin/bash
# I2C bus scanner for Argo sensors
# Scans both external buses used on v4 wiring:
#   - Bus 0: battery/water + IMU devices (TWI0)
#   - TWI2 runtime bus: wind + mast LED devices (pins 27/28)
# Times out after 1 second per bus if a bus is malfunctioning.

set -euo pipefail

TIMEOUT=1
TWI2_DEFAULT_LINUX_BUS=4
TWI2_CONTROLLER_BASENAME="5002800.i2c"

resolve_twi2_linux_bus() {
    # Match anem.py behavior: resolve from the TWI2 controller basename.
    local matches=()
    local bus_name

    # shellcheck disable=SC2206
    matches=(/sys/devices/platform/soc*/"${TWI2_CONTROLLER_BASENAME}"/i2c-*)
    if [ -e "${matches[0]:-}" ]; then
        bus_name=$(basename "${matches[0]}")  # e.g. i2c-3
        echo "${bus_name#i2c-}"
        return 0
    fi

    echo "$TWI2_DEFAULT_LINUX_BUS"
}

TWI2_BUS="$(resolve_twi2_linux_bus)"

declare -A BUS_LABELS=(
    ["0"]="Bus 0 (TWI0: battery/water + IMU)"
    ["$TWI2_BUS"]="Bus ${TWI2_BUS} (TWI2 pins 27/28: wind + mast LED)"
)

# Expected I2C addresses for Argo devices by bus.
# Note: MP2672 (0x4B) excluded - using standalone mode (GPIO only), not I2C.
# Mast LED: PCA9632 shows as 0x62 (individual) and 0x70 (All Call).
declare -A EXPECTED_BUS0=(
    ["0x34"]="ADC (MAX11612) - Battery/Water"
    ["0x44"]="Temperature/Humidity (SHT45)"
    ["0x4a"]="IMU (BNO085)"
)
declare -A EXPECTED_TWI2=(
    ["0x21"]="Wind sensor 1 (SDP3x)"
    ["0x22"]="Wind sensor 2 (SDP3x)"
    ["0x23"]="Wind sensor 3 (SDP3x)"
    ["0x62"]="Mast LED (PCA9632 individual)"
    ["0x70"]="Mast LED (PCA9632 All Call)"
)

TOTAL_FOUND=0
TOTAL_EXPECTED=0
TOTAL_MISSING=0
FAILED_BUSES=0
# Set in check_bus when bus 0 lists 0x4a (BNO085).
BUS0_IMU_FOUND=0

parse_detected_addresses() {
    local i2c_output="$1"
    local parsed=()

    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]*[0-9a-fA-F]{2}:[[:space:]] ]]; then
            for addr in $line; do
                if [[ "$addr" =~ ^[0-9a-fA-F]{2}:$ ]] || [[ "$addr" == "--" ]]; then
                    continue
                fi
                if [[ "$addr" =~ ^[0-9a-fA-F]{2}$ ]]; then
                    addr_lower=$(echo "$addr" | tr '[:upper:]' '[:lower:]')
                    parsed+=("0x${addr_lower}")
                fi
            done
        fi
    done <<< "$i2c_output"

    echo "${parsed[*]}"
}

check_bus() {
    local bus="$1"
    local expected_map_name="$2"
    local overlay_hint="$3"
    local scan_ok=true
    local detected_addresses=()

    echo "Scanning I2C ${BUS_LABELS[$bus]}..."

    local start_time end_time elapsed elapsed_ms i2c_output exit_code
    exit_code=0
    start_time=$(date +%s.%N)
    set +e
    i2c_output=$(timeout "${TIMEOUT}"s i2cdetect -y "$bus" 2>&1)
    exit_code=$?
    set -e
    if [ "$exit_code" -ne 0 ]; then
        scan_ok=false
    fi
    end_time=$(date +%s.%N)
    elapsed=$(echo "$end_time - $start_time" | bc)
    elapsed_ms=$(echo "$elapsed * 1000" | bc | sed 's/\.0*$//')

    if [ "$scan_ok" = false ]; then
        FAILED_BUSES=$((FAILED_BUSES + 1))
        if [ "$exit_code" -eq 124 ]; then
            echo "❌ I2C bus ${bus} scan TIMED OUT after ${TIMEOUT}s (${elapsed_ms}ms)"
        else
            echo "❌ I2C bus ${bus} scan FAILED (exit code: ${exit_code})"
            echo "   Error output: ${i2c_output}"
        fi
        echo "   Check dtoverlay configuration (${overlay_hint}) and wiring."
        echo ""
        return
    fi

    echo "Scan completed in ${elapsed_ms}ms"
    local parsed
    parsed="$(parse_detected_addresses "$i2c_output")"
    if [ -n "$parsed" ]; then
        # shellcheck disable=SC2206
        detected_addresses=($parsed)
    fi

    if [ ${#detected_addresses[@]} -eq 0 ]; then
        echo "⚠️  No I2C devices detected on bus ${bus}"
    else
        echo "Detected I2C addresses: ${detected_addresses[*]}"
    fi
    echo ""

    # Iterate expected map via nameref (bash 4.3+)
    # shellcheck disable=SC2178
    declare -n expected_map="$expected_map_name"

    local found_count=0
    local missing_count=0
    local bus_expected=${#expected_map[@]}

    for addr in "${!expected_map[@]}"; do
        local device_name="${expected_map[$addr]}"
        local found=false
        for detected in "${detected_addresses[@]}"; do
            if [[ "${addr,,}" == "${detected,,}" ]]; then
                found=true
                break
            fi
        done

        if [ "$found" = true ]; then
            echo "🟢 ${addr}: ${device_name}"
            found_count=$((found_count + 1))
            if [ "$bus" = "0" ] && [[ "${addr,,}" == "0x4a" ]]; then
                BUS0_IMU_FOUND=1
            fi
        else
            echo "🔴 ${addr}: ${device_name} - MISSING"
            missing_count=$((missing_count + 1))
        fi
    done

    TOTAL_EXPECTED=$((TOTAL_EXPECTED + bus_expected))
    TOTAL_FOUND=$((TOTAL_FOUND + found_count))
    TOTAL_MISSING=$((TOTAL_MISSING + missing_count))

    echo ""
    echo "Bus ${bus} summary: ${found_count}/${bus_expected} expected devices found"
    echo ""
}

# When I2C sees the IMU, optionally verify the ROS bridge is publishing real health + IMU data.
check_imu_ros_live() {
    echo "--- IMU (BNO085) live stack (ROS) ---"
    if [ "$BUS0_IMU_FOUND" -eq 0 ]; then
        echo "⚠️  Skipping live IMU check: 0x4a not seen on bus 0 (I2C probe)."
        echo ""
        return 0
    fi
    echo "🟢 I2C: BNO085 present at 0x4a on bus 0"

    local humble=/opt/ros/humble/setup.bash
    if [ ! -f "$humble" ]; then
        echo "⚠️  ROS Humble not at ${humble} — cannot verify /imu_health or /imu."
        echo ""
        return 0
    fi

    local svc_active
    svc_active="$(systemctl is-active argo_bno085.service 2>/dev/null || echo unknown)"
    echo "   argo_bno085.service: ${svc_active}"

    local health_out health_ec imu_out imu_ec
    set +e
    health_out="$(timeout 10 bash -c "source ${humble} && command -v ros2 >/dev/null && ros2 topic echo /imu_health --once" 2>&1)"
    health_ec=$?
    imu_out="$(timeout 10 bash -c "source ${humble} && command -v ros2 >/dev/null && ros2 topic echo /imu --once" 2>&1)"
    imu_ec=$?
    set -e

    if [ "$health_ec" -eq 127 ] || echo "$health_out" | grep -qi 'command not found'; then
        echo "⚠️  ros2 not available after sourcing Humble — install ros-humble-* CLI or skip this check."
        echo ""
        return 0
    fi

    if echo "$health_out" | grep -qiE 'does not exist|not found|Unable to find|no topic|Waiting for'; then
        echo "🔴 /imu_health: no message (topic missing or no publisher)."
        echo "   Start: sudo systemctl start argo_bno085.service"
        echo ""
        return 0
    fi

    if echo "$health_out" | grep -qiE '^data:\s*true\b'; then
        echo "🟢 /imu_health: HEALTHY (bridge reports recent driver data)"
    elif echo "$health_out" | grep -qiE '^data:\s*false\b'; then
        echo "🔴 /imu_health: UNHEALTHY (no recent /imu from driver — see journalctl -u argo_bno085.service)"
    else
        if [ "$health_ec" -eq 124 ]; then
            echo "🔴 /imu_health: timed out (no Bool within 10s)."
        else
            echo "⚠️  /imu_health: unexpected output (exit ${health_ec}):"
            echo "$health_out" | head -n 5
        fi
    fi

    if echo "$imu_out" | grep -qiE 'does not exist|not found|Unable to find|no topic|Waiting for'; then
        echo "🔴 /imu: no message (driver not streaming)."
    elif echo "$imu_out" | grep -qiE 'header:|linear_acceleration:|angular_velocity:'; then
        echo "🟢 /imu: received sensor message (driver streaming)"
    else
        if [ "$imu_ec" -eq 124 ]; then
            echo "🔴 /imu: timed out (no SensorMessage within 10s)."
        else
            echo "⚠️  /imu: no recognizable sensor fields (exit ${imu_ec})."
        fi
    fi
    echo ""
}

echo "Resolved TWI2 runtime bus: i2c-${TWI2_BUS} (default fallback: i2c-${TWI2_DEFAULT_LINUX_BUS})"
echo ""

check_bus 0 EXPECTED_BUS0 "pi-i2c0"
check_bus "$TWI2_BUS" EXPECTED_TWI2 "pi-i2c2"

if [ "$FAILED_BUSES" -eq 0 ]; then
    check_imu_ros_live
fi

echo "Overall summary: ${TOTAL_FOUND}/${TOTAL_EXPECTED} expected devices found"

if [ "$FAILED_BUSES" -gt 0 ]; then
    echo "❌ ${FAILED_BUSES} bus scan(s) failed - investigate overlays/wiring first."
    exit 1
fi

if [ "$TOTAL_MISSING" -gt 0 ]; then
    echo "⚠️  ${TOTAL_MISSING} device(s) missing - check connections and power."
    exit 1
fi

echo "✅ All expected devices present on both buses."
exit 0
