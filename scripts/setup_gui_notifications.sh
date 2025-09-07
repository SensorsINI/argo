#!/bin/bash
# Setup script for Argo GUI Storage Notifications

SCRIPT_DIR="/home/orangepi/argo/scripts"

echo "🚢 Setting up Argo GUI Storage Notifications..."

# Make scripts executable
chmod +x "$SCRIPT_DIR/storage_notifications.py"
chmod +x "$SCRIPT_DIR/argo_startup_notification.sh"

# Check if notify-send is available
if ! command -v notify-send &> /dev/null; then
    echo "⚠️  notify-send not found. Installing libnotify-bin..."
    sudo apt update
    sudo apt install -y libnotify-bin
fi

# Copy systemd service files
echo "📋 Installing systemd services..."
sudo cp "$SCRIPT_DIR/argo-storage-monitor.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/argo-startup-notification.service" /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable services
echo "🔧 Enabling services..."
sudo systemctl enable argo-startup-notification.service
sudo systemctl enable argo-storage-monitor.service

echo ""
echo "✅ Argo GUI notification setup complete!"
echo ""
echo "Services installed:"
echo "  - argo-startup-notification.service (sends notification on boot)"
echo "  - argo-storage-monitor.service (monitors storage every 5 minutes)"
echo ""
echo "To start services now:"
echo "  sudo systemctl start argo-startup-notification.service"
echo "  sudo systemctl start argo-storage-monitor.service"
echo ""
echo "To check service status:"
echo "  sudo systemctl status argo-startup-notification.service"
echo "  sudo systemctl status argo-storage-monitor.service"
echo ""
echo "To test notifications manually:"
echo "  python3 $SCRIPT_DIR/storage_notifications.py startup"
echo "  python3 $SCRIPT_DIR/storage_notifications.py check"
echo ""
echo "To disable services:"
echo "  sudo systemctl disable argo-startup-notification.service"
echo "  sudo systemctl disable argo-storage-monitor.service"
