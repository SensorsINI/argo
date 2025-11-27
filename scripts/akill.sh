#!/usr/bin/env bash
# akill.sh - Forcefully kill Argo simulation/process groups by pattern (with PGID handling)
# CRITICAL: This script preserves critical systemd services that must always stay running

set -euo pipefail

log() { printf '%s\n' "$*" >&2; }

# Critical systemd services that must NEVER be killed (battery monitoring, power control, health monitoring)
# These services are essential for system safety and must remain running
CRITICAL_SERVICES=(
  "argo_battery_water.service"
  "argo_power_control.service"
  "argo_health_monitor.service"
)

# No-arg killer: always target the full simulation set and ensure complete shutdown
patterns=(
  "foxglove_bridge"
  "/argo/"
)

# Check if a process belongs to a critical systemd service
is_critical_service_process() {
  local pid=$1
  local cgroup_file="/proc/${pid}/cgroup"
  
  # Check if process exists and cgroup file is readable
  if [[ ! -r "$cgroup_file" ]]; then
    return 1  # Process doesn't exist or can't read cgroup
  fi
  
  # Read cgroup information (systemd v2 uses unified hierarchy)
  local cgroup_info
  cgroup_info=$(cat "$cgroup_file" 2>/dev/null || echo "")
  
  # Check if process belongs to any critical service
  for service in "${CRITICAL_SERVICES[@]}"; do
    if echo "$cgroup_info" | grep -q "/system.slice/${service}\|/user.slice/.*${service}"; then
      return 0  # Process belongs to critical service
    fi
  done
  
  return 1  # Process does not belong to critical service
}

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
  
  # Filter out this script, parent shell, any process containing "akill", and critical service processes
  if [[ -n "$all_pids" ]]; then
    echo "$all_pids" | while read -r pid; do
      if [[ "$pid" != "$current_pid" && "$pid" != "$parent_pid" ]]; then
        # Check if the process command contains "akill"
        local cmd
        cmd=$(ps -p "$pid" -o cmd= 2>/dev/null || true)
        if [[ -n "$cmd" && ! "$cmd" =~ akill ]]; then
          # CRITICAL: Skip processes belonging to critical systemd services
          if ! is_critical_service_process "$pid"; then
            echo "$pid"
          fi
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
log "⚠️  CRITICAL: Preserving critical systemd services:"
for service in "${CRITICAL_SERVICES[@]}"; do
  log "   - ${service}"
done
log ""

pids=($(find_matching_pids))

if [[ ${#pids[@]} -eq 0 ]]; then
  log "✓ No Argo processes found (critical services preserved)"
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
