#!/bin/bash
# Argo persistent log management utility
# - Rotates Argo service logs on boot so each boot has a dedicated log snapshot
# - Prunes old log artifacts when the persistent log filesystem is nearly full

set -euo pipefail

LOG_DIR="/var/log.hdd/persistent"
DEFAULT_THRESHOLD=85

# Logs that correspond to systemd services and should be rotated at boot
SERVICE_LOGS=(
  "argo-power-control.log"
  "argo-battery-water.log"
  "argo-health-monitor.log"
  "argo-launch-standard.log"
  "argo-bno085.log"
  "argo-radio-servo-module.log"
)

ROTATE=false
PRUNE=false
THRESHOLD="$DEFAULT_THRESHOLD"

print_usage() {
  cat <<'EOF'
Usage: argo_persistent_log_manager.sh [OPTIONS]

Options:
  --rotate-on-boot      Rotate Argo service logs into timestamped boot snapshots
  --prune               Remove oldest persistent logs if disk usage exceeds threshold
  --threshold PERCENT   Override default prune threshold (default: 85)
  -h, --help            Show this help

When run without options the script performs no action.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rotate-on-boot)
      ROTATE=true
      shift
      ;;
    --prune)
      PRUNE=true
      shift
      ;;
    --threshold)
      shift
      [[ $# -gt 0 ]] || { echo "Error: --threshold requires a value" >&2; exit 1; }
      THRESHOLD="$1"
      if ! [[ "$THRESHOLD" =~ ^[0-9]+$ ]] || (( THRESHOLD <= 0 || THRESHOLD >= 100 )); then
        echo "Error: threshold must be an integer between 1 and 99" >&2
        exit 1
      fi
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      print_usage >&2
      exit 1
      ;;
  esac
done

# Ensure persistent directory exists
mkdir -p "$LOG_DIR"

rotate_service_logs() {
  local timestamp boot_id base src dest
  timestamp=$(date +%Y%m%d-%H%M%S)
  boot_id=$(journalctl --list-boots 2>/dev/null | tail -n 1 | awk '{print $2}')

  for log_name in "${SERVICE_LOGS[@]}"; do
    src="$LOG_DIR/$log_name"
    base="${log_name%.log}"

    if [[ -s "$src" ]]; then
      dest="$LOG_DIR/${base}-${timestamp}"
      if [[ -n "$boot_id" ]]; then
        dest+="-${boot_id}"
      fi
      dest+=".boot.log"

      if cp --preserve=mode,ownership,timestamps "$src" "$dest"; then
        # Compress boot log snapshots to save space (typically 90%+ reduction)
        if gzip "$dest" 2>/dev/null; then
          echo "Snapshot saved and compressed $log_name -> $(basename "$dest").gz"
        else
          echo "Snapshot saved $log_name -> $(basename "$dest") (compression failed)"
        fi
      else
        echo "Warning: failed to copy $log_name to $(basename "$dest")" >&2
      fi
    fi

    # Ensure a fresh log file exists with correct ownership and permissions
    : > "$src"
    chown orangepi:orangepi "$src" 2>/dev/null || true
    chmod 0664 "$src" 2>/dev/null || true
  done
}

prune_if_needed() {
  local usage
  usage=$(df -P "$LOG_DIR" | awk 'NR==2 {gsub("%", "", $5); print $5}')

  if [[ -z "$usage" ]]; then
    echo "Warning: unable to determine disk usage for $LOG_DIR" >&2
    return
  fi

  if (( usage < THRESHOLD )); then
    return
  fi

  echo "Persistent log usage at ${usage}% (threshold ${THRESHOLD}%), pruning oldest files..."

  # Delete oldest log artifacts until we drop below threshold or run out of files
  while (( usage >= THRESHOLD )); do
    oldest=$(find "$LOG_DIR" -maxdepth 1 -type f \
      \( -name 'argo-*.boot.log' -o -name 'journalctl-*.log*' -o -name 'dmesg-*.log*' -o -name '*-monitor-*.log*' -o -name '*.gz' \) \
      -printf '%T@ %p\n' | sort -n | head -n 1 | cut -d' ' -f2-)

    if [[ -z "$oldest" ]]; then
      echo "No more removable files in $LOG_DIR but usage remains high (${usage}%)." >&2
      break
    fi

    echo "Removing oldest log: $(basename "$oldest")"
    rm -f "$oldest"

    usage=$(df -P "$LOG_DIR" | awk 'NR==2 {gsub("%", "", $5); print $5}')
  done
}

if $ROTATE; then
  rotate_service_logs
fi

if $PRUNE; then
  prune_if_needed
fi

exit 0
