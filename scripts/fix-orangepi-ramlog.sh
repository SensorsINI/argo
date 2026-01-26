#!/bin/bash
# Fix orangepi-ramlog service so it does NOT sync persistent logs into RAM
# and also does NOT delete persistent logs on disk.
#
# Why:
# - On Orange Pi, orangepi-ramlog mounts `/var/log` on zram/tmpfs.
# - Argo persistent logs live on disk at `/var/log.hdd/persistent/`.
# - If orangepi-ramlog syncs `persistent/` from disk -> RAM, it can fill zram
#   and cause rsyslog/other services to fail with ENOSPC.
# - We patch orangepi-ramlog to exclude `persistent/` in BOTH directions:
#   - syncToDisk: prevent `--delete` from deleting disk persistent logs
#   - syncFromDisk: prevent copying disk persistent logs into zram

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_FILE="$SCRIPT_DIR/../patches/orangepi-ramlog-exclude-persistent.patch"
TARGET_SCRIPT="/usr/lib/orangepi/orangepi-ramlog"

echo "🔧 Fixing orangepi-ramlog service to preserve persistent logs..."

# Check if the target script exists
if [ ! -f "$TARGET_SCRIPT" ]; then
    echo "❌ Error: $TARGET_SCRIPT not found. This fix is only for Orange Pi systems."
    exit 1
fi

# Check if the patch file exists
if [ ! -f "$PATCH_FILE" ]; then
    echo "❌ Error: Patch file $PATCH_FILE not found."
    exit 1
fi

# Create backup of original script
if [ ! -f "${TARGET_SCRIPT}.backup" ]; then
    echo "📋 Creating backup of original script..."
    sudo cp "$TARGET_SCRIPT" "${TARGET_SCRIPT}.backup"
fi

# If already patched, exit successfully (idempotent)
EXCLUDE_COUNT="$(sudo bash -c "grep -c -- '--exclude \"persistent\"' \"$TARGET_SCRIPT\" 2>/dev/null || true")"
if [ "${EXCLUDE_COUNT:-0}" -ge 2 ]; then
    echo "✅ orangepi-ramlog already patched (persistent excluded in both directions)."
    exit 0
fi

# Apply the patch
echo "🔨 Applying patch to exclude persistent/ directory from deletion..."
set +e
sudo patch -N "$TARGET_SCRIPT" < "$PATCH_FILE"
PATCH_EXIT=$?
set -e

# Treat as success if the expected changes are present after patch attempt.
EXCLUDE_COUNT_AFTER="$(sudo bash -c "grep -c -- '--exclude \"persistent\"' \"$TARGET_SCRIPT\" 2>/dev/null || true")"
if [ "${EXCLUDE_COUNT_AFTER:-0}" -ge 2 ]; then
    echo "✅ Patch applied successfully (persistent excluded in both directions)."

    # Clean up any reject file left behind by patch -N
    sudo rm -f "${TARGET_SCRIPT}.rej" 2>/dev/null || true

    # Restart the service to apply changes
    echo "🔄 Restarting orangepi-ramlog service..."
    sudo systemctl restart orangepi-ramlog.service

    echo "✅ orangepi-ramlog service fixed and restarted!"
    exit 0
fi

echo "❌ Failed to apply patch (exit code: ${PATCH_EXIT})."
echo "↩︎ Restoring backup..."
sudo cp "${TARGET_SCRIPT}.backup" "$TARGET_SCRIPT"
sudo rm -f "${TARGET_SCRIPT}.rej" 2>/dev/null || true
exit 1








