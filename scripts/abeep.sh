#!/usr/bin/env bash
#
# abeep.sh
# --------
# Argo buzzer pulse helper.
#
# Usage: abeep.sh --help
#
# Notes:
# - Buzzer is active HIGH on GPIO 258 (PI2 / pin 35).
# - PCB has a 10k pulldown on BUZZER to keep it OFF during floating boot states.

set -euo pipefail

WAIT_MODE=false
SOS_MODE=false
DURATION="0.5"
GPIO_CHIP="/dev/gpiochip0"
BUZZER_LINE=258
# Integer, decimal, or leading dot (e.g. 1, 0.5, .1).
DURATION_RE='^([0-9]+([.][0-9]+)?|\.[0-9]+)$'

print_usage() {
    cat <<'EOF'
Argo buzzer pulse (GPIO 258, PI2, active HIGH).

Usage:
  abeep.sh                     Async 0.5s pulse (returns immediately).
  abeep.sh DURATION            Async pulse; DURATION is seconds (e.g. 0.5, 1, .1).
  abeep.sh --sos               Async SOS (...---...) ~1.5s total.
  abeep.sh --wait              Blocking 0.5s pulse.
  abeep.sh --wait DURATION     Blocking pulse for DURATION seconds.
  abeep.sh --wait --sos        Blocking SOS.

Options:
  --wait       Run in foreground (default is background).
  --sos        Morse SOS instead of a single pulse.
  -h, --help   Show this help.

Examples:
  abeep.sh
  abeep.sh 1.0
  abeep.sh --wait 0.2
  abeep.sh .1
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        --wait)
            WAIT_MODE=true
            ;;
        --sos)
            SOS_MODE=true
            ;;
        *)
            if [[ "$1" =~ $DURATION_RE ]]; then
                DURATION="$1"
            else
                echo "Error: unknown argument '$1' (try --help)" >&2
                exit 1
            fi
            ;;
    esac
    shift
done

if ! [[ "$DURATION" =~ $DURATION_RE ]]; then
    echo "Error: duration must be numeric seconds (e.g. 0.5, 1, .1)" >&2
    exit 1
fi

run_beep_worker() {
local pulse_duration="$1"
# gpioset duration parsing below expects a digit before the dot (e.g. 0.1 not .1).
if [[ "$pulse_duration" =~ ^\.[0-9]+$ ]]; then
    pulse_duration="0${pulse_duration}"
fi
if command -v python3 >/dev/null 2>&1; then
    set +e
    python3 - "$GPIO_CHIP" "$BUZZER_LINE" "$pulse_duration" <<'PY'
import sys
import time

chip_path = sys.argv[1]
line_no = int(sys.argv[2])
duration = float(sys.argv[3])

try:
    import gpiod
except Exception:
    sys.exit(2)

chip = None
line = None
try:
    chip = gpiod.Chip(chip_path)
    line = chip.get_line(line_no)
    line.request(consumer="argo_abeep", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
    line.set_value(1)
    time.sleep(duration)
    line.set_value(0)
finally:
    try:
        if line is not None:
            line.release()
    except Exception:
        pass
    try:
        if chip is not None:
            chip.close()
    except Exception:
        pass
PY
    rc=$?
    set -e
    if [ "$rc" -eq 0 ]; then
        return 0
    fi
fi

if command -v gpioset >/dev/null 2>&1; then
    # Keep line HIGH for pulse_duration then release (must use $1, not global DURATION,
    # when run_beep_worker is called with a different length e.g. SOS dots).
    sec_part="${pulse_duration%.*}"
    frac_part="${pulse_duration#*.}"
    if [ "$sec_part" = "$pulse_duration" ]; then
        frac_part="0"
    fi
    frac_part="$(printf '%-6s' "$frac_part" | tr ' ' '0' | cut -c1-6)"
    usec_part="${frac_part#-}"

    set +e
    sudo gpioset --mode=time --sec="$sec_part" --usec="$usec_part" gpiochip0 "${BUZZER_LINE}=1"
    rc=$?
    set -e
    if [ "$rc" -eq 0 ]; then
        return 0
    fi
fi

return 1
}

run_sos_worker() {
    # Tightened SOS cadence for a more continuous audible pattern:
    # dit-dit-dit-dah-dah-dah-dit-dit-dit
    # (slightly reduced gaps between symbols, especially dots)
    local dot="0.056"
    local dash="0.167"
    local intra="0.02"      # tighter gap between symbols
    local letter_gap="0.105" # modest gap between S and O groups
    local i

    for i in 1 2 3; do
        run_beep_worker "$dot" || return 1
        if [ "$i" -lt 3 ]; then
            sleep "$intra"
        else
            sleep "$letter_gap"
        fi
    done

    for i in 1 2 3; do
        run_beep_worker "$dash" || return 1
        if [ "$i" -lt 3 ]; then
            sleep "$intra"
        else
            sleep "$letter_gap"
        fi
    done

    for i in 1 2 3; do
        run_beep_worker "$dot" || return 1
        if [ "$i" -lt 3 ]; then
            sleep "$intra"
        fi
    done
}

if [ "$WAIT_MODE" = true ]; then
    if [ "$SOS_MODE" = true ]; then
        echo "Buzzer SOS (blocking): ...---... (~1.5s)"
        if run_sos_worker; then
            echo "Buzzer SOS complete."
            exit 0
        fi
        echo "Error: buzzer SOS failed. GPIO ${BUZZER_LINE} may be busy (or tool unavailable)." >&2
        exit 1
    fi

    echo "Buzzer pulse (blocking): line ${BUZZER_LINE} HIGH for ${DURATION}s"
    if run_beep_worker "$DURATION"; then
        echo "Buzzer pulse complete."
        exit 0
    fi
    echo "Error: buzzer pulse failed. GPIO ${BUZZER_LINE} may be busy (or tool unavailable)." >&2
    exit 1
fi

if [ "$SOS_MODE" = true ]; then
    (
        run_sos_worker >/dev/null 2>&1 || true
    ) &
    echo "Buzzer SOS started in background (...---..., ~1.5s total)"
    exit 0
fi

(
    run_beep_worker "$DURATION" >/dev/null 2>&1 || true
) &

echo "Buzzer pulse started in background (duration ${DURATION}s)"
exit 0
