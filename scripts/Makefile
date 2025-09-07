# Makefile for Orange Pi Zero 2W Power Control System
# This Makefile installs the power control daemon and systemd service

SERVICE_NAME = power_control
SCRIPT_DIR = $(shell pwd)
PYTHON_SCRIPT = power_control.py
SERVICE_FILE = power_control.service
LOG_FILE = /var/log/power_control.log

.PHONY: install_power_control check_deps install_service enable_service create_logs clean uninstall help

# Default target
all: install_power_control

# Main installation target
install_power_control: check_deps install_service enable_service create_logs
	@echo ""
	@echo "✅ Power Control System Installation Complete!"
	@echo ""
	@echo "Next steps:"
	@echo "1. The power control service will start automatically on boot"
	@echo "2. Check service status with: sudo systemctl status $(SERVICE_NAME)"
	@echo "3. View logs with: sudo journalctl -u $(SERVICE_NAME) -f"
	@echo "4. Test with: sudo systemctl start $(SERVICE_NAME)"
	@echo ""
	@echo "Hardware connections:"
	@echo "- PI3 (Pin 40): Connect to !POW relay control (open drain)"
	@echo "- PI9 (Pin 28): Connect to power button (input, external pullup)"
	@echo "- PH4 (Pin 18): Connect to green LED in power button"
	@echo "- PI1 (Pin 12): Connect to blue LED in power button"
	@echo ""
	@echo "Power button behavior:"
	@echo "- Short press (< 1 second): No action"
	@echo "- Long press (>= 1 second): Initiate shutdown"
	@echo "- System will de-energize relay after shutdown completes"

# Check and install dependencies
check_deps:
	@echo "🔍 Checking dependencies..."
	@if ! python3 -c "import gpiod" 2>/dev/null; then \
		echo "📦 Installing python3-libgpiod..."; \
		sudo apt-get update; \
		sudo apt-get install -y python3-libgpiod; \
	else \
		echo "✅ python3-libgpiod already installed"; \
	fi

# Install systemd service
install_service:
	@echo "📋 Installing systemd service..."
	@sudo cp $(SERVICE_FILE) /etc/systemd/system/
	@sudo chmod +x $(PYTHON_SCRIPT)
	@sudo systemctl daemon-reload
	@echo "✅ Service installed and systemd reloaded"

# Enable and start service
enable_service:
	@echo "🚀 Enabling power control service..."
	@sudo systemctl enable $(SERVICE_NAME)
	@sudo systemctl start $(SERVICE_NAME)
	@echo "✅ Service enabled and started"

# Create log file
create_logs:
	@echo "📝 Setting up log file..."
	@sudo touch $(LOG_FILE)
	@sudo chown root:root $(LOG_FILE)
	@sudo chmod 644 $(LOG_FILE)
	@echo "✅ Log file created: $(LOG_FILE)"

# Clean up installation
clean:
	@echo "🧹 Cleaning up installation..."
	@sudo systemctl stop $(SERVICE_NAME) 2>/dev/null || true
	@sudo systemctl disable $(SERVICE_NAME) 2>/dev/null || true
	@sudo rm -f /etc/systemd/system/$(SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@echo "✅ Cleanup complete"

# Uninstall everything
uninstall: clean
	@echo "🗑️  Uninstalling power control system..."
	@sudo rm -f $(LOG_FILE)
	@echo "✅ Uninstall complete"

# Show help
help:
	@echo "Orange Pi Zero 2W Power Control System Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  install_power_control  - Install the complete power control system (default)"
	@echo "  check_deps            - Check and install required dependencies"
	@echo "  install_service       - Install systemd service only"
	@echo "  enable_service        - Enable and start the service"
	@echo "  create_logs           - Create log file with proper permissions"
	@echo "  clean                 - Stop and disable service, remove service file"
	@echo "  uninstall             - Complete removal of power control system"
	@echo "  help                  - Show this help message"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install_power_control    # Full installation"
	@echo "  make clean                    # Stop and remove service"
	@echo "  make uninstall                # Complete removal"
	@echo "  make help                     # Show this help"

