#!/usr/bin/env bash
# akill.sh - Forcefully kill Argo simulation/process groups by pattern (with PGID handling)
# Usage:
#   akill.sh                  # default: kill /argo/ and foxglove_bridge groups with -9
#   akill.sh -TERM            # graceful terminate default patterns
#   akill.sh 9                # numeric signal converted to -9 for default patterns
#   akill.sh -9 python3       # kill groups matching 'python3'
#   akill.sh -TERM '/home/tobi/argo/nodes/controller.py' '/home/tobi/argo/nodes/sailing_area_publisher.py'
set -euo pipefail

log() { printf '%s\n' "$*" >&2; }

# No-arg killer: always target the full simulation set and ensure complete shutdown
patterns=(
  "/home/tobi/argo/launch/argo_lifecycle_manager.py"
  "/home/tobi/argo/nodes/.*\\.py"
  "foxglove_bridge"
  "/argo/"
)

# Collect PIDs for all patterns
declare -a pids
for pat in "${patterns[@]}"; do
  while IFS= read -r pid; do
    [[ -n "${pid:-}" ]] && pids+=("$pid")
  done < <(pgrep -f -- "$pat" 2>/dev/null || true)
done

if [[ ${#pids[@]} -eq 0 ]]; then
  log "Patterns: ${patterns[*]}"
  echo "✅ No matching processes."
  exit 0
fi

# Unique PIDs
mapfile -t upids < <(printf "%s\n" "${pids[@]}" | awk 'NF && !seen[$0]++')
log "Matched patterns: ${patterns[*]}"
log "Matched PIDs: ${upids[*]:-<none>}"

# Map to PGIDs
declare -a pgids
for pid in "${upids[@]}"; do
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  [[ -n "${pgid:-}" ]] && pgids+=("$pgid")
done

if [[ ${#pgids[@]} -eq 0 ]]; then
  log "Patterns: ${patterns[*]}"
  log "PIDs: ${upids[*]:-<none>}"
  echo "⚠️  Found PIDs but could not resolve process groups. Proceeding with PID-based termination."
  # SIGCONT -> SIGTERM -> wait -> SIGKILL on direct PIDs
  for pid in "${upids[@]}"; do kill -CONT "$pid" 2>/dev/null || true; done
  sleep 0.05
  for pid in "${upids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${upids[@]}"; do kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true; done
else
  # Unique PGIDs
  mapfile -t upgids < <(printf "%s\n" "${pgids[@]}" | awk 'NF && !seen[$0]++')
  log "PGIDs: ${upgids[*]:-<none>}"
  # Resume stopped groups and individual PIDs
  for pid in "${upids[@]}"; do kill -CONT "$pid" 2>/dev/null || true; done
  for pgid in "${upgids[@]}"; do kill -CONT -"$pgid" 2>/dev/null || true; done
  sleep 0.05
  # 1) Graceful termination
  for pid in "${upids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; endone=true; done
  for pgid in "${upgids[@]}"; do kill -TERM -"$pgid" 2>/dev/null || true; done
  echo "⏳ Sent SIGTERM to ${#upgids[@]} groups / ${#upids[@]} processes. Waiting 2s..."
  sleep 2
  # 2) Force-kill survivors
  for pid in "${upids[@]}"; do kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true; done
  for pgid in "${upgids[@]}"; do kill -0 -"$pgid" 2>/dev/null && kill -KILL -"$pgid" 2>/dev/null || true; done
fi

# Display leftovers
re="$(printf '%s|' "${patterns[@]}")"; re="${re%|}"
if ps aux | grep -E "$re" | grep -v grep | grep -v cursor | grep -v akill.sh >/dev/null; then
  echo "⚠️  Some processes still present (showing up to 12):"
  ps aux | grep -E "$re" | grep -v grep | grep -v cursor | grep -v akill.sh | head -12
  exit 1
else
  echo "✅ All matching processes terminated."
fi


