#!/bin/bash
# Setup script for Argo GUI Storage Notifications

# Determine scripts directory relative to this file
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🚢 Setting up Argo GUI Storage Notifications..."

# Make scripts executable
chmod +x "$SCRIPT_DIR/storage_notifications.py"

# Check if notify-send is available
if ! command -v notify-send &> /dev/null; then
    echo "⚠️  notify-send not found. Installing libnotify-bin..."
    sudo apt update
    sudo apt install -y libnotify-bin
fi

# Copy systemd service files
echo "📋 Installing systemd services..."
sudo cp "$SCRIPT_DIR/argo-storage-monitor.service" /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable services
echo "🔧 Enabling services..."
sudo systemctl enable argo-storage-monitor.service

echo ""
echo "✅ Argo GUI notification setup complete!"
echo ""
echo "Services installed:"
echo "  - argo-storage-monitor.service (monitors storage every 5 minutes)"
echo ""
echo "To start services now:"
echo "  sudo systemctl start argo-storage-monitor.service"
echo ""
echo "To check service status:"
echo "  sudo systemctl status argo-storage-monitor.service"
echo ""
echo "To test notifications manually:"
echo "  python3 $SCRIPT_DIR/storage_notifications.py startup"
echo "  python3 $SCRIPT_DIR/storage_notifications.py check"
echo ""
echo "To disable services:"
echo "  sudo systemctl disable argo-storage-monitor.service"
