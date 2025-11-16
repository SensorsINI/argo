#!/usr/bin/env bash
# akill.sh - Forcefully kill Argo simulation/process groups by pattern (with PGID handling)

set -euo pipefail

log() { printf '%s\n' "$*" >&2; }

# No-arg killer: always target the full simulation set and ensure complete shutdown
patterns=(
  "foxglove_bridge"
  "/argo/"
)

# Find all matching processes (excluding this script and its parent shells)
find_matching_pids() {
  local current_pid=$$
  local parent_pid=$PPID
  local combined_pattern=""
  
  # Build a combined regex pattern for pgrep
  for i in "${!patterns[@]}"; do
    if [[ $i -eq 0 ]]; then
      combined_pattern="${patterns[$i]}"
    else
      combined_pattern="${combined_pattern}|${patterns[$i]}"
    fi
  done
  
  # Use single pgrep call with combined pattern
  local all_pids
  all_pids=$(pgrep -f "$combined_pattern" 2>/dev/null || true)
  
  # Filter out this script, parent shell, and any process containing "akill"
  if [[ -n "$all_pids" ]]; then
    echo "$all_pids" | while read -r pid; do
      if [[ "$pid" != "$current_pid" && "$pid" != "$parent_pid" ]]; then
        # Check if the process command contains "akill"
        local cmd
        cmd=$(ps -p "$pid" -o cmd= 2>/dev/null || true)
        if [[ -n "$cmd" && ! "$cmd" =~ akill ]]; then
          echo "$pid"
        fi
      fi
    done
  fi
}

# Get process command lines
show_processes() {
  local pids=("$@")
  if [[ ${#pids[@]} -eq 0 ]]; then
    return
  fi
  log "Found ${#pids[@]} process(es):"
  for pid in "${pids[@]}"; do
    if ps -p "$pid" >/dev/null 2>&1; then
      local cmd
      cmd=$(ps -p "$pid" -o pid=,cmd= 2>/dev/null || echo "$pid <no info>")
      log "  $cmd"
    fi
  done
}

# Main execution
log "Searching for Argo processes..."
pids=($(find_matching_pids))

if [[ ${#pids[@]} -eq 0 ]]; then
  log "✓ No Argo processes found"
  exit 0
fi

# Show what we found
show_processes "${pids[@]}"

# Attempt graceful termination with SIGTERM (15)
log ""
log "Sending SIGTERM (15) to ${#pids[@]} process(es)..."
for pid in "${pids[@]}"; do
  if kill -15 "$pid" 2>/dev/null; then
    log "  Sent SIGTERM to $pid"
  fi
done

# Wait for processes to terminate
log "Waiting 2 seconds for graceful shutdown..."
sleep 2

# Check for survivors
survivors=($(find_matching_pids))

if [[ ${#survivors[@]} -gt 0 ]]; then
  log ""
  log "⚠ ${#survivors[@]} process(es) did not terminate gracefully:"
  show_processes "${survivors[@]}"
  
  # Force kill with SIGKILL (9)
  log ""
  log "Sending SIGKILL (9) to ${#survivors[@]} process(es)..."
  for pid in "${survivors[@]}"; do
    if kill -9 "$pid" 2>/dev/null; then
      log "  Sent SIGKILL to $pid"
    fi
  done
  
  # Brief wait for force kill to complete
  sleep 0.5
fi

# Final verification
final_check=($(find_matching_pids))

if [[ ${#final_check[@]} -eq 0 ]]; then
  log ""
  log "✓ All Argo processes have been terminated"
  exit 0
else
  log ""
  log "✗ WARNING: ${#final_check[@]} process(es) still running:"
  show_processes "${final_check[@]}"
  exit 1
fi
