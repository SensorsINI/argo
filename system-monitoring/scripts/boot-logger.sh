#!/bin/bash
# Boot history logger - saves boot info to persistent storage

PERSIST_DIR="/var/log.hdd/persistent"
SUMMARY_LOG="$PERSIST_DIR/boot-history.log"
INDEX_LOG="$PERSIST_DIR/journalctl-boot-index.log"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
BOOT_TIME=$(who -b | awk '{print $3, $4}')
BOOT_LIST=$(journalctl --list-boots 2>/dev/null)
BOOT_ID=$(echo "$BOOT_LIST" | tail -n 1 | awk '{print $2}')

# Create persistent directory if it doesn't exist
mkdir -p "$PERSIST_DIR"

# Rotate boot history summary if needed and append new entry
BOOT_SUMMARY_ENTRY="$PERSIST_DIR/boot-history-$(date +%Y%m%d-%H%M%S)"
if [[ -n "$BOOT_ID" ]]; then
  BOOT_SUMMARY_ENTRY+="-${BOOT_ID}"
fi
BOOT_SUMMARY_ENTRY+=".log"

{
  echo "================================================================================"
  echo "Boot Event: $TIMESTAMP"
  echo "Boot Time: $BOOT_TIME"
  echo "Boot ID: ${BOOT_ID:-unknown}"
  echo "Uptime: $(uptime)"
  echo "--------------------------------------------------------------------------------"
  echo "$BOOT_LIST"
  echo "--------------------------------------------------------------------------------"
} | tee -a "$SUMMARY_LOG" > "$BOOT_SUMMARY_ENTRY"

# Maintain boot index snapshot
echo "$BOOT_LIST" > "$INDEX_LOG"

# Ensure summary log has predictable ownership
chown orangepi:orangepi "$SUMMARY_LOG" "$INDEX_LOG" 2>/dev/null || true
chmod 0664 "$SUMMARY_LOG" "$INDEX_LOG" 2>/dev/null || true

# Capture previous boot journal if available and not already archived
PREV_BOOT_ID=$(echo "$BOOT_LIST" | tail -n 2 | head -n 1 | awk '{print $2}')
if [[ -n "$PREV_BOOT_ID" ]]; then
  PREV_LOG="$PERSIST_DIR/journalctl-${PREV_BOOT_ID}.log"
  if [[ ! -f "$PREV_LOG" ]]; then
    journalctl -b -1 > "$PREV_LOG"
  fi
fi

# Save dmesg with timestamp in filename (append mode for same day)
DMESG_FILE="$PERSIST_DIR/dmesg-$(date +%Y%m%d).log"
echo "=== Boot at $TIMESTAMP ===" >> "$DMESG_FILE"
dmesg >> "$DMESG_FILE"
echo "" >> "$DMESG_FILE"

# Copy current journal to persistent backup
CURRENT_JOURNAL_LOG="$PERSIST_DIR/journalctl-$(date +%Y%m%d-%H%M%S)"
if [[ -n "$BOOT_ID" ]]; then
  CURRENT_JOURNAL_LOG+="-${BOOT_ID}"
fi
CURRENT_JOURNAL_LOG+=".log"
journalctl -b 0 > "$CURRENT_JOURNAL_LOG"

echo "Boot logging completed at $(date '+%Y-%m-%d %H:%M:%S')" >> "$SUMMARY_LOG"
echo "" >> "$SUMMARY_LOG"
