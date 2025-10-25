#!/bin/bash
# Boot history logger - saves boot info to persistent storage

PERSIST_DIR="/var/log.hdd/persistent"
BOOT_LOG="$PERSIST_DIR/boot-history.log"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
BOOT_TIME=$(who -b | awk '{print $3, $4}')
BOOT_ID=$(journalctl --list-boots | tail -1 | awk '{print $2}')

# Create persistent directory if it doesn't exist
mkdir -p "$PERSIST_DIR"

# Log boot event
echo "================================================================================" >> "$BOOT_LOG"
echo "Boot Event: $TIMESTAMP" >> "$BOOT_LOG"
echo "Boot Time: $BOOT_TIME" >> "$BOOT_LOG"
echo "Boot ID: $BOOT_ID" >> "$BOOT_LOG"
echo "Uptime: $(uptime)" >> "$BOOT_LOG"
echo "--------------------------------------------------------------------------------" >> "$BOOT_LOG"

# Save dmesg with timestamp in filename (append mode for same day)
DMESG_FILE="$PERSIST_DIR/dmesg-$(date +%Y%m%d).log"
echo "=== Boot at $TIMESTAMP ===" >> "$DMESG_FILE"
dmesg >> "$DMESG_FILE"
echo "" >> "$DMESG_FILE"

# Copy current journal to persistent backup
journalctl -b 0 > "$PERSIST_DIR/journalctl-$(date +%Y%m%d-%H%M%S).log"

echo "Boot logging completed at $(date '+%Y-%m-%d %H:%M:%S')" >> "$BOOT_LOG"
echo "" >> "$BOOT_LOG"
