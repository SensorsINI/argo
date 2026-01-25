#!/bin/bash
# Argo persistent log management utility
# - Rotates Argo service logs on boot so each boot has a dedicated log snapshot
# - Prunes old log artifacts to reduce clutter and cap retention

set -euo pipefail

LOG_DIR="/var/log.hdd/persistent"
DEFAULT_THRESHOLD=85
DEFAULT_RETENTION_DAYS=2
DEFAULT_RETENTION_BOOTS=4

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
RETENTION_DAYS="$DEFAULT_RETENTION_DAYS"
RETENTION_BOOTS="$DEFAULT_RETENTION_BOOTS"

print_usage() {
  cat <<'EOF'
Usage: argo_persistent_log_manager.sh [OPTIONS]

Options:
  --rotate-on-boot      Rotate Argo service logs into timestamped boot snapshots
  --prune               Prune persistent logs using retention rules (and disk threshold as a backstop)
  --threshold PERCENT   Override default prune threshold (default: 85)
  --retention-days N    Keep only last N days of logs (default: 2)
  --retention-boots N   Keep only last N boots of per-boot artifacts (default: 4)
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
    --retention-days)
      shift
      [[ $# -gt 0 ]] || { echo "Error: --retention-days requires a value" >&2; exit 1; }
      RETENTION_DAYS="$1"
      if ! [[ "$RETENTION_DAYS" =~ ^[0-9]+$ ]] || (( RETENTION_DAYS <= 0 || RETENTION_DAYS > 3650 )); then
        echo "Error: retention-days must be a positive integer" >&2
        exit 1
      fi
      shift
      ;;
    --retention-boots)
      shift
      [[ $# -gt 0 ]] || { echo "Error: --retention-boots requires a value" >&2; exit 1; }
      RETENTION_BOOTS="$1"
      if ! [[ "$RETENTION_BOOTS" =~ ^[0-9]+$ ]] || (( RETENTION_BOOTS <= 0 || RETENTION_BOOTS > 100 )); then
        echo "Error: retention-boots must be a positive integer" >&2
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

_get_usage_percent() {
  df -P "$LOG_DIR" | awk 'NR==2 {gsub("%", "", $5); print $5}'
}

_get_keep_boot_ids() {
  # Return up to RETENTION_BOOTS most recent boot IDs (one per line).
  # If journalctl is unavailable or returns nothing, return empty set.
  journalctl --list-boots 2>/dev/null | tail -n "${RETENTION_BOOTS}" | awk '{print $2}' | sed '/^$/d' || true
}

prune_with_retention() {
  local cutoff_epoch
  cutoff_epoch=$(date -d "-${RETENTION_DAYS} days" +%s)

  # Build a fast lookup set for boot IDs
  local -a keep_boot_ids
  mapfile -t keep_boot_ids < <(_get_keep_boot_ids)

  # Use python for robust parsing + to avoid pipefail/SIGPIPE issues in sort|head pipelines.
  # Retention policy:
  # - For per-boot artifacts that contain a boot ID: keep if (mtime >= cutoff) AND (boot_id is in last RETENTION_BOOTS).
  # - For date-stamped daily logs and other artifacts: keep if (mtime >= cutoff).
  # This implements "keep last RETENTION_DAYS or RETENTION_BOOTS, whichever is less clutter" by keeping the intersection.
  python3 - <<'PY' "$LOG_DIR" "$cutoff_epoch" "${keep_boot_ids[@]}"
import os
import re
import sys
import time
from pathlib import Path

log_dir = Path(sys.argv[1])
cutoff = int(sys.argv[2])
keep_boot_ids = set(sys.argv[3:])

always_keep = {
    "boot-history.log",
    "journalctl-boot-index.log",
}

# Patterns we consider "managed clutter" and eligible for retention pruning.
# Keep this explicit to avoid deleting live service logs or logrotate-managed argo-*.log.N(.gz).
patterns = [
    # Per-boot snapshots/exports
    ("argo_boot_snapshot", re.compile(r"^argo-.*-(?P<bootid>[0-9a-f]{32})\.boot\.log(?:\.gz)?$")),
    ("journalctl_export", re.compile(r"^journalctl-(?:\d{8}-\d{6}-)?(?P<bootid>[0-9a-f]{32})\.log(?:\.gz)?$")),
    ("boot_history_snapshot", re.compile(r"^boot-history-\d{8}-\d{6}-(?P<bootid>[0-9a-f]{32})\.log$")),

    # Daily / periodic logs
    ("dmesg", re.compile(r"^dmesg-\d{8}(?:-\d{6})?\.log(?:\.gz)?$")),
    ("shutdown", re.compile(r"^shutdown-\d{8}-\d{6}\.log$")),
    ("thermal", re.compile(r"^thermal-\d{8}\.log$")),
    ("wifi", re.compile(r"^wifi-reconnect-\d{8}\.log$")),
    ("battery_csv", re.compile(r"^battery-monitor-\d{8}\.csv$")),
    ("orangepi_monitor", re.compile(r"^orangepi-monitor-\d{8}\.log$")),
    ("memory_monitor", re.compile(r"^memory-\d{8}\.log$")),
    ("processes_monitor", re.compile(r"^processes-\d{8}\.log$")),
    ("cursor_processes", re.compile(r"^cursor-processes-\d{8}\.log$")),
]

def match_bucket(name: str):
    for bucket, rx in patterns:
        m = rx.match(name)
        if m:
            return bucket, m.groupdict()
    return None, {}

to_delete = []
deleted_bytes = 0

now = int(time.time())

for p in log_dir.iterdir():
    if not p.is_file():
        continue
    name = p.name
    if name in always_keep:
        continue

    bucket, groups = match_bucket(name)
    if not bucket:
        continue

    st = p.stat()
    mtime = int(st.st_mtime)

    # Retention by time
    if mtime < cutoff:
        to_delete.append((mtime, p, st.st_size, f"{bucket}: older than {sys.argv[2]}"))
        continue

    # Extra retention by boot count (only if boot ID is present AND we have keep set)
    bootid = groups.get("bootid")
    if bootid and keep_boot_ids:
        if bootid not in keep_boot_ids:
            to_delete.append((mtime, p, st.st_size, f"{bucket}: bootid {bootid} not in keep set"))

for _, p, size, reason in sorted(to_delete, key=lambda x: x[0]):
    try:
        os.remove(p)
        deleted_bytes += size
        print(f"Removed {p.name} ({size} bytes) [{reason}]")
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"Warning: failed to remove {p}: {e}", file=sys.stderr)

print(f"Retention prune complete: removed_files={len(to_delete)} removed_bytes={deleted_bytes}")
PY
}

prune_if_needed() {
  local usage
  usage=$(df -P "$LOG_DIR" | awk 'NR==2 {gsub("%", "", $5); print $5}')

  if [[ -z "$usage" ]]; then
    echo "Warning: unable to determine disk usage for $LOG_DIR" >&2
    return
  fi

  # Always run retention prune to reduce clutter, even when under threshold.
  echo "Pruning with retention: keep last ${RETENTION_DAYS} days AND last ${RETENTION_BOOTS} boots (intersection)."
  prune_with_retention

  usage=$(_get_usage_percent)
  if [[ -z "$usage" ]]; then
    echo "Warning: unable to determine disk usage for $LOG_DIR after retention prune" >&2
    return
  fi

  if (( usage < THRESHOLD )); then
    return
  fi

  echo "Persistent log usage at ${usage}% (threshold ${THRESHOLD}%), doing emergency pruning..."

  # Delete oldest log artifacts until we drop below threshold or run out of files
  while (( usage >= THRESHOLD )); do
    # Choose oldest among known "archive-ish" artifacts. Avoid sort|head pipelines (pipefail/SIGPIPE).
    oldest=$(python3 - <<'PY' "$LOG_DIR"
import os
import re
import sys
from pathlib import Path

log_dir = Path(sys.argv[1])
rx = re.compile(r"^(argo-.*\.boot\.log(\.gz)?|journalctl-.*\.log(\.gz)?|dmesg-.*\.log(\.gz)?|boot-history-.*\.log|shutdown-.*\.log|thermal-.*\.log|wifi-reconnect-.*\.log|battery-monitor-.*\.csv|orangepi-monitor-.*\.log|memory-.*\.log|processes-.*\.log|cursor-processes-.*\.log)$")

oldest = None
oldest_mtime = None
for p in log_dir.iterdir():
    if not p.is_file():
        continue
    if p.name in ("boot-history.log", "journalctl-boot-index.log"):
        continue
    if not rx.match(p.name):
        continue
    st = p.stat()
    if oldest is None or st.st_mtime < oldest_mtime:
        oldest = p
        oldest_mtime = st.st_mtime

if oldest is not None:
    print(str(oldest))
PY
)

    if [[ -z "$oldest" ]]; then
      echo "No more removable files in $LOG_DIR but usage remains high (${usage}%)." >&2
      break
    fi

    echo "Removing oldest log: $(basename "$oldest")"
    rm -f "$oldest"

    usage=$(_get_usage_percent)
  done
}

if $ROTATE; then
  rotate_service_logs
fi

if $PRUNE; then
  prune_if_needed
fi

exit 0
