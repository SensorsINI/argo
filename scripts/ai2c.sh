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

HUMBLE_SETUP='/opt/ros/humble/setup.bash'

# Run ros2 in a fresh bash with Humble sourced (matches bno085.py status checks).
run_ros2_cmd() {
    local timeout_sec="$1"
    shift
    local cmd="$*"
    timeout "$timeout_sec" bash -c "source '${HUMBLE_SETUP}' && ${cmd}" 2>&1
}

ros2_cli_broken() {
    local output="$1"
    echo "$output" | grep -qiE '!rclpy\.ok\(\)|rclpy\.ok\(\)'
}

ensure_ros2_daemon() {
    local probe
    probe="$(run_ros2_cmd 5 'ros2 topic list' || true)"
    if ros2_cli_broken "$probe"; then
        echo "   Restarting ros2 daemon (stale !rclpy.ok())..."
        run_ros2_cmd 5 'ros2 daemon stop' >/dev/null 2>&1 || true
        run_ros2_cmd 5 'ros2 daemon start' >/dev/null 2>&1 || true
        sleep 1
        probe="$(run_ros2_cmd 5 'ros2 topic list' || true)"
        if ros2_cli_broken "$probe"; then
            echo "🔴 ros2 CLI still broken after daemon restart — ROS topic checks unreliable."
            return 1
        fi
        echo "   🟢 ros2 daemon recovered"
    fi
    return 0
}

# Start ros2 topic echo --once in the background; write output and exit code to temp files.
ros2_topic_echo_once_async() {
    local topic="$1"
    local timeout_sec="${2:-10}"
    local out_file="$3"
    local ec_file="$4"
    local -n _pid_ref="$5"

    (
        set +e
        timeout "$timeout_sec" bash -c "source '${HUMBLE_SETUP}' && ros2 topic echo '${topic}' --once" \
            >"$out_file" 2>&1
        echo $? >"$ec_file"
    ) &
    _pid_ref=$!
}

# Wait for parallel topic echo jobs; print visible newline progress (no silent \r-only wait).
# Args: timeout_sec topic pid out_file ec_file [topic pid out_file ec_file ...]
wait_parallel_ros2_topic_echo() {
    local timeout_sec="$1"
    shift

    local -a topics=() pids=() out_files=() ec_files=() elapsed=() done_flags=()
    local topic pid out_file ec_file
    local count=0 any_running=1 i

    while [ $# -gt 0 ]; do
        topic="$1"
        pid="$2"
        out_file="$3"
        ec_file="$4"
        topics+=("$topic")
        pids+=("$pid")
        out_files+=("$out_file")
        ec_files+=("$ec_file")
        elapsed+=(0)
        done_flags+=(0)
        count=$((count + 1))
        shift 4
    done

    if [ "$count" -eq 0 ]; then
        return 0
    fi

    while [ "$any_running" -eq 1 ]; do
        any_running=0
        for i in $(seq 0 $((count - 1))); do
            if [ "${done_flags[$i]}" -eq 1 ]; then
                continue
            fi
            if kill -0 "${pids[$i]}" 2>/dev/null; then
                any_running=1
            else
                done_flags[$i]=1
                wait "${pids[$i]}" 2>/dev/null || true
                printf '   %s: done (%ds)\n' "${topics[$i]}" "${elapsed[$i]}" >&2
            fi
        done
        if [ "$any_running" -eq 0 ]; then
            break
        fi
        sleep 1
        for i in $(seq 0 $((count - 1))); do
            if [ "${done_flags[$i]}" -eq 0 ] && kill -0 "${pids[$i]}" 2>/dev/null; then
                elapsed[$i]=$((${elapsed[$i]} + 1))
                printf '   %s: waiting for message... %ds / %ds\n' \
                    "${topics[$i]}" "${elapsed[$i]}" "$timeout_sec" >&2
            fi
        done
    done
}

report_imu_health_echo() {
    local health_out="$1"
    local health_ec="$2"

    if ros2_cli_broken "$health_out"; then
        echo "🔴 /imu_health: ros2 CLI broken (!rclpy.ok) — run: ros2 daemon stop && ros2 daemon start"
        return 1
    fi

    if echo "$health_out" | grep -qiE 'does not exist|not found|Unable to find|no topic'; then
        echo "🔴 /imu_health: no message (topic missing or no publisher)."
        echo "   Start: sudo systemctl start argo_bno085.service"
        return 1
    fi

    if echo "$health_out" | grep -qiE '^data:\s*true\b'; then
        echo "🟢 /imu_health: HEALTHY (sustained /imu from driver)"
    elif echo "$health_out" | grep -qiE '^data:\s*false\b'; then
        echo "🔴 /imu_health: UNHEALTHY (stale/unstable /imu — see journalctl -u argo_bno085.service)"
    else
        if [ "$health_ec" -eq 124 ]; then
            echo "🔴 /imu_health: timed out (no Bool within 10s)."
        else
            echo "⚠️  /imu_health: unexpected output (exit ${health_ec}):"
            echo "$health_out" | head -n 5
        fi
    fi
    return 0
}

report_imu_echo() {
    local imu_out="$1"
    local imu_ec="$2"

    if ros2_cli_broken "$imu_out"; then
        echo "🔴 /imu: ros2 CLI broken (!rclpy.ok) — run: ros2 daemon stop && ros2 daemon start"
        return 0
    fi

    if echo "$imu_out" | grep -qiE 'does not exist|not found|Unable to find|no topic'; then
        echo "🔴 /imu: no message (driver not streaming)."
        return 0
    fi

    if echo "$imu_out" | grep -qiE 'header:|linear_acceleration:|angular_velocity:'; then
        echo "🟢 /imu: received sensor message (driver streaming)"
    else
        if [ "$imu_ec" -eq 124 ]; then
            echo "🔴 /imu: timed out (no SensorMessage within 10s)."
        else
            echo "⚠️  /imu: no recognizable sensor fields (exit ${imu_ec})."
        fi
    fi
    return 0
}

report_imu_journal_recent() {
    local health_claimed_healthy="${1:-}"
    local journal_out watchdog_count unhealthy_count
    journal_out="$(journalctl -u argo_bno085.service --since "2 min ago" --no-pager 2>/dev/null || true)"
    if [ -z "$journal_out" ]; then
        return 0
    fi
    watchdog_count="$(echo "$journal_out" | grep -c 'Watchdog timeout' || true)"
    unhealthy_count="$(echo "$journal_out" | grep -c 'BNO085 health: UNHEALTHY' || true)"
    if [ "$watchdog_count" -gt 0 ]; then
        echo "🔴 Driver watchdog: ${watchdog_count} reset(s) in last 2 min (I2C/sensor unstable)"
    fi
    if [ "$unhealthy_count" -gt 0 ]; then
        echo "🔴 Bridge health: ${unhealthy_count} UNHEALTHY event(s) in last 2 min"
    fi
    if [ "$health_claimed_healthy" = "true" ] && { [ "$watchdog_count" -gt 0 ] || [ "$unhealthy_count" -gt 0 ]; }; then
        echo "⚠️  /imu_health says HEALTHY but journal shows instability — restart argo_bno085.service after bridge update"
    fi
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

    if [ ! -f "$HUMBLE_SETUP" ]; then
        echo "⚠️  ROS Humble not at ${HUMBLE_SETUP} — cannot verify /imu_health or /imu."
        report_imu_journal_recent
        echo ""
        return 0
    fi

    # is-active exits 3 for inactive — do not use || echo (that masks inactive as unknown).
    local svc_active
    svc_active="$(systemctl show -p ActiveState --value argo_bno085.service 2>/dev/null || echo unknown)"
    echo "   argo_bno085.service: ${svc_active}"

    if ! timeout 2 bash -c "source '${HUMBLE_SETUP}' && command -v ros2" >/dev/null 2>&1; then
        echo "⚠️  ros2 not available after sourcing Humble — install ros-humble-* CLI or skip this check."
        report_imu_journal_recent
        echo ""
        return 0
    fi

    ensure_ros2_daemon || true

    # Fast registration check before blocking on echo --once.
    echo "   Checking ROS topic registration..."
    local topic_list topic_list_ec
    set +e
    topic_list="$(run_ros2_cmd 5 'ros2 topic list')"
    topic_list_ec=$?
    set -e
    if ros2_cli_broken "$topic_list"; then
        echo "🔴 ros2 topic list: CLI broken (!rclpy.ok) — journal check below."
    elif [ "$topic_list_ec" -eq 124 ]; then
        echo "⚠️  ros2 topic list timed out (5s) — continuing with echo checks."
    elif [ "$topic_list_ec" -ne 0 ]; then
        echo "⚠️  ros2 topic list failed (exit ${topic_list_ec}) — continuing with echo checks."
    else
        for topic in /imu_health /imu; do
            if echo "$topic_list" | grep -qx "$topic"; then
                echo "   🟢 ${topic}: registered"
            else
                echo "   🔴 ${topic}: not listed"
            fi
        done
    fi

    local health_out_file health_ec_file imu_out_file imu_ec_file
    local health_pid imu_pid health_out health_ec imu_out imu_ec
    health_out_file="$(mktemp)"
    health_ec_file="$(mktemp)"
    imu_out_file="$(mktemp)"
    imu_ec_file="$(mktemp)"

    set +e
    for topic in /imu_health /imu; do
        printf '   %s: waiting for message... 0s / 10s\n' "$topic" >&2
    done
    ros2_topic_echo_once_async /imu_health 10 "$health_out_file" "$health_ec_file" health_pid
    ros2_topic_echo_once_async /imu 10 "$imu_out_file" "$imu_ec_file" imu_pid
    wait_parallel_ros2_topic_echo 10 \
        /imu_health "$health_pid" "$health_out_file" "$health_ec_file" \
        /imu "$imu_pid" "$imu_out_file" "$imu_ec_file"

    health_out="$(cat "$health_out_file")"
    health_ec="$(cat "$health_ec_file")"
    imu_out="$(cat "$imu_out_file")"
    imu_ec="$(cat "$imu_ec_file")"
    rm -f "$health_out_file" "$health_ec_file" "$imu_out_file" "$imu_ec_file"

    local health_claimed_healthy="false"
    if report_imu_health_echo "$health_out" "$health_ec"; then
        if echo "$health_out" | grep -qiE '^data:\s*true\b'; then
            health_claimed_healthy="true"
        fi
    else
        report_imu_journal_recent
        echo ""
        return 0
    fi
    report_imu_echo "$imu_out" "$imu_ec"
    report_imu_journal_recent "$health_claimed_healthy"
    set -e
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
