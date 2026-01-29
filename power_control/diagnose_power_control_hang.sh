#!/usr/bin/env bash
# Diagnose argo_power_control.py appearing hung (no logs, LEDs not updating).
# Uses /proc to sample syscalls; suggests strace if installed.
# Run: bash power_control/diagnose_power_control_hang.sh
# Or: sudo bash ... for full /proc/syscall access.

set -e
PID=""
# Must be the Python process, not the bash wrapper (bash cmdline also contains "argo_power_control.py")
for p in $(pgrep -f "argo_power_control\.py" 2>/dev/null); do
  exe=$(readlink -f /proc/$p/exe 2>/dev/null) || true
  if [[ "$exe" == *python* ]]; then
    PID=$p
    break
  fi
done
if [ -z "$PID" ]; then
  echo "No argo_power_control.py process found."
  exit 1
fi

echo "=== argo_power_control.py PID: $PID ==="
echo ""

# Syscall names (aarch64 common)
sc_name() {
  case "$1" in
    63) echo "read" ;;
    72) echo "epoll_pwait" ;;
    98) echo "futex" ;;
    207) echo "waitid" ;;
    *) echo "syscall_$1" ;;
  esac
}

echo "--- Current syscalls (main + threads) ---"
for tid in /proc/$PID/task/*; do
  t=$(basename "$tid")
  s=$(cat "$tid/syscall" 2>/dev/null | awk '{print $1}')
  [ -z "$s" ] && s=$(sudo cat "$tid/syscall" 2>/dev/null | awk '{print $1}')
  [ -n "$s" ] && echo "  TID $t: $s ($(sc_name $s))"
done
echo ""

# Tee (sibling under same bash)
PARENT_PID=$(cat /proc/$PID/status 2>/dev/null | awk '/^PPid:/{print $2}')
TEE_PID=$(ps -eo pid,ppid,cmd 2>/dev/null | awk -v p="$PID" -v pp="$PARENT_PID" '$2==pp && $1!=p && /tee/{print $1; exit}')
if [ -n "$TEE_PID" ]; then
  echo "--- Tee process (PID $TEE_PID) ---"
  s=$(sudo cat /proc/$TEE_PID/syscall 2>/dev/null | awk '{print $1}')
  echo "  Syscall: $s ($(sc_name $s))"
  echo "  (Tee in read = waiting for Python to write; pipe empty.)"
  echo ""
fi

echo "--- FDs (stdout/stderr) ---"
for f in 1 2; do
  [ -r /proc/$PID/fdinfo/$f ] || continue
  ino=$(awk '/ino:/{print $2}' /proc/$PID/fdinfo/$f 2>/dev/null)
  echo "  fd $f: ino=$ino (pipe to tee)"
done
echo ""

echo "--- Summary ---"
echo "Process is running; threads are in normal wait states (futex/epoll_pwait/read)."
echo "If no logs appear: Python is not writing to stdout (stuck before log or pipe full)."
echo "If LEDs do not update: LED heartbeat thread may be blocked (e.g. in _check_network_status timeout)."
echo ""
echo "To trace system calls (recommended):"
echo "  sudo apt install -y strace"
echo "  sudo strace -p $PID -f 2>&1 | head -300"
echo "Run strace for 10–15s to see where threads block. Ctrl+C to stop."
echo ""
echo "To rule out pipe blocking: restart without tee so logs go only to journal:"
echo "  sudo systemctl stop argo_power_control.service"
echo "  Then edit ExecStart to remove '|& tee -a ...' and use only journal; restart and check journalctl -u argo_power_control.service -f"
