#!/bin/bash
# Fix orangepi-ramlog service to exclude persistent/ directory from deletion
# This prevents the service from deleting rotated log files in /var/log.hdd/persistent/

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

# Apply the patch
echo "🔨 Applying patch to exclude persistent/ directory from deletion..."
if sudo patch "$TARGET_SCRIPT" < "$PATCH_FILE"; then
    echo "✅ Patch applied successfully!"
    
    # Restart the service to apply changes
    echo "🔄 Restarting orangepi-ramlog service..."
    sudo systemctl restart orangepi-ramlog.service
    
    echo "✅ orangepi-ramlog service fixed and restarted!"
    echo "📝 The persistent/ directory will now be preserved during log synchronization."
else
    echo "❌ Failed to apply patch. Restoring backup..."
    sudo cp "${TARGET_SCRIPT}.backup" "$TARGET_SCRIPT"
    exit 1
fi






