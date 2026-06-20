#!/bin/bash
# Delay i2c-0 consumers until the BNO085 C++ driver finishes SHTP init.
# Used as ExecStartPre for argo_battery_water.service at boot.
#
# The driver exits quickly on "Failed to get product IDs"; a stable bno08x_driver
# process for several seconds means init succeeded and short ADC/SHT45 polls are safe.

set -euo pipefail

readonly SERVICE=argo_bno085.service
readonly TIMEOUT_SEC=45
readonly STABLE_SEC=3
readonly POLL_SEC=1

log() {
    echo "wait_for_bno085: $*"
}

driver_running() {
    pgrep -f 'bno08x_driver' >/dev/null 2>&1
}

if ! systemctl is-enabled -q "$SERVICE" 2>/dev/null; then
    log "IMU service not enabled; skipping wait"
    exit 0
fi

if ! systemctl is-active -q "$SERVICE" 2>/dev/null; then
    log "Waiting for $SERVICE to become active..."
    active_deadline=$((SECONDS + 15))
    while (( SECONDS < active_deadline )); do
        systemctl is-active -q "$SERVICE" && break
        sleep "$POLL_SEC"
    done
fi

if ! systemctl is-active -q "$SERVICE" 2>/dev/null; then
    log "WARNING: $SERVICE not active; starting battery/water without IMU wait"
    exit 0
fi

log "Waiting up to ${TIMEOUT_SEC}s for BNO085 driver init (stable ${STABLE_SEC}s)..."
deadline=$((SECONDS + TIMEOUT_SEC))
stable=0
while (( SECONDS < deadline )); do
    if ! systemctl is-active -q "$SERVICE" 2>/dev/null; then
        log "WARNING: $SERVICE stopped during wait; continuing"
        exit 0
    fi
    if driver_running; then
        stable=$((stable + 1))
        if (( stable >= STABLE_SEC )); then
            log "BNO085 driver ready; releasing i2c-0 for battery/water"
            exit 0
        fi
    else
        stable=0
    fi
    sleep "$POLL_SEC"
done

log "WARNING: BNO085 not ready within ${TIMEOUT_SEC}s; starting battery/water anyway"
exit 0
