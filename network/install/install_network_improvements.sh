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
    
    # Copy script to system location
    cp "$NETWORK_DIR/scripts/wifi_reconnect.sh" /usr/local/bin/
    chmod +x /usr/local/bin/wifi_reconnect.sh
    
    log_success "WiFi reconnection script installed to /usr/local/bin/wifi_reconnect.sh"
}

# Install NetworkManager configuration
install_networkmanager_config() {
    log "Installing NetworkManager configuration..."
    
    # Copy configuration file
    cp "$NETWORK_DIR/config/argo_wifi_scan.conf" /etc/NetworkManager/conf.d/
    
    log_success "NetworkManager configuration installed to /etc/NetworkManager/conf.d/argo_wifi_scan.conf"
}

# Setup cron job for WiFi reconnection
setup_cron_job() {
    log "Setting up cron job for WiFi reconnection..."
    
    # Check if cron job already exists
    if crontab -l 2>/dev/null | grep -q "wifi_reconnect.sh"; then
        log_warning "Cron job for WiFi reconnection already exists"
    else
        # Add cron job to run every 2 minutes
        (crontab -l 2>/dev/null; echo "*/2 * * * * /usr/local/bin/wifi_reconnect.sh") | crontab -
        log_success "Cron job added to run WiFi reconnection every 2 minutes"
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
    
    if /usr/local/bin/wifi_reconnect.sh; then
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
    echo "  • WiFi reconnection script: /usr/local/bin/wifi_reconnect.sh"
    echo "  • NetworkManager config: /etc/NetworkManager/conf.d/argo_wifi_scan.conf"
    echo "  • Cron job: Runs every 2 minutes"
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
    setup_cron_job
    restart_networkmanager
    test_installation
    show_summary
    
    log_success "Installation completed successfully!"
}

# Run main function
main "$@"
