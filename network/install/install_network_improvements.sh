#!/bin/bash

# Argo Network Improvements Installation Script
# Installs WiFi reconnection system and NetworkManager optimizations

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_DIR="$(dirname "$SCRIPT_DIR")"
ARGO_ROOT="$(dirname "$NETWORK_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')] ✓${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[$(date '+%Y-%m-%d %H:%M:%S')] ⚠${NC} $1"
}

log_error() {
    echo -e "${RED}[$(date '+%Y-%m-%d %H:%M:%S')] ✗${NC} $1"
}

# Check if running as root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

# Install WiFi reconnection script
install_wifi_script() {
    log "Installing WiFi reconnection script..."
    
    # Make script executable in repo (no copy needed - systemd runs from repo)
    chmod +x "$NETWORK_DIR/scripts/wifi_reconnect.sh"
    
    log_success "WiFi reconnection script made executable in repo: $NETWORK_DIR/scripts/wifi_reconnect.sh"
}

# Install NetworkManager configuration
install_networkmanager_config() {
    log "Installing NetworkManager configuration..."
    
    # Copy configuration file
    cp "$NETWORK_DIR/config/argo_wifi_scan.conf" /etc/NetworkManager/conf.d/
    
    log_success "NetworkManager configuration installed to /etc/NetworkManager/conf.d/argo_wifi_scan.conf"
}

# Setup systemd service and timer for WiFi reconnection
setup_systemd_service() {
    log "Setting up systemd service and timer for WiFi reconnection..."
    
    # Map template repo path to this checkout. Do not substitute /home/orangepi with $HOME:
    # under sudo, HOME is /root and would rewrite .../home/orangepi/argo... into /root/argo.
    sed -e "s#/home/orangepi/argo#$ARGO_ROOT#g" \
        "$NETWORK_DIR/config/argo_wifi_reconnect.service" > /tmp/argo_wifi_reconnect.service
    
    # Copy configured service and timer files
    cp /tmp/argo_wifi_reconnect.service /etc/systemd/system/
    cp "$NETWORK_DIR/config/argo_wifi_reconnect.timer" /etc/systemd/system/
    rm /tmp/argo_wifi_reconnect.service
    
    # Create log file with correct permissions
    # The service runs as root to handle log rotation properly
    # Log file is owned by root with 644 permissions (readable by all)
    LOG_FILE="/var/log.hdd/persistent/wifi-reconnect.log"
    mkdir -p "$(dirname "$LOG_FILE")"
    touch "$LOG_FILE"
    chmod 644 "$LOG_FILE"
    log_success "WiFi reconnection log file created with correct permissions: $LOG_FILE"
    
    # Reload systemd and enable timer
    systemctl daemon-reload
    systemctl enable argo_wifi_reconnect.timer
    systemctl start argo_wifi_reconnect.timer
    
    # Check if timer is active
    if systemctl is-active --quiet argo_wifi_reconnect.timer; then
        log_success "WiFi reconnection timer started successfully"
        log "Timer status:"
        systemctl status argo_wifi_reconnect.timer --no-pager -l | sed 's/^/  /'
    else
        log_error "Failed to start WiFi reconnection timer"
        exit 1
    fi
}

# Restart NetworkManager
restart_networkmanager() {
    log "Restarting NetworkManager to apply configuration changes..."
    
    systemctl restart NetworkManager
    
    # Wait a moment for NetworkManager to fully restart
    sleep 3
    
    if systemctl is-active --quiet NetworkManager; then
        log_success "NetworkManager restarted successfully"
    else
        log_error "NetworkManager failed to restart"
        exit 1
    fi
}

# Test the installation
test_installation() {
    log "Testing WiFi reconnection script..."
    
    if "$NETWORK_DIR/scripts/wifi_reconnect.sh"; then
        log_success "WiFi reconnection script test passed"
    else
        log_warning "WiFi reconnection script test had issues (this may be normal if no preferred networks are available)"
    fi
    
    # Check if log file was created
    if [ -f "/var/log.hdd/persistent/wifi-reconnect.log" ]; then
        log_success "WiFi reconnection log file created"
        log "Recent log entries:"
        tail -3 /var/log.hdd/persistent/wifi-reconnect.log | sed 's/^/  /'
    else
        log_warning "WiFi reconnection log file not found (may be created on first run)"
    fi
}

# Display installation summary
show_summary() {
    echo
    log_success "Argo Network Improvements Installation Complete!"
    echo
    echo "Installed components:"
    echo "  • WiFi reconnection script: $NETWORK_DIR/scripts/wifi_reconnect.sh (runs from repo)"
    echo "  • NetworkManager config: /etc/NetworkManager/conf.d/argo_wifi_scan.conf"
    echo "  • Systemd service: argo_wifi_reconnect.service"
    echo "  • Systemd timer: argo_wifi_reconnect.timer (runs every 1 minute)"
    echo "  • Log file: /var/log.hdd/persistent/wifi-reconnect.log"
    echo
    echo "Network priorities:"
    echo "  • tobi-s24: Priority 15 (highest)"
    echo "  • tobi-wlan: Priority 10 (medium)"
    echo "  • uzh-iot: Priority 5 (lowest)"
    echo
    echo "The system will now automatically switch to preferred networks when available."
    echo "Monitor the log file to see reconnection activity:"
    echo "  tail -f /var/log.hdd/persistent/wifi-reconnect.log"
    echo
}

# Main installation process
main() {
    log "Starting Argo Network Improvements Installation..."
    
    check_root
    install_wifi_script
    install_networkmanager_config
    setup_systemd_service
    restart_networkmanager
    test_installation
    show_summary
    
    log_success "Installation completed successfully!"
}

# Run main function
main "$@"
