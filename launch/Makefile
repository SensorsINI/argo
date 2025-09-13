# Makefile for Argo Launch Services
# This Makefile manages systemd services for ROS2 Argo robot and power control system

# Service configuration
SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
RECORD_SERVICE = argo-record.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

# Power control configuration
POWER_SERVICE_NAME = argo_power_control
SCRIPT_DIR = /home/orangepi/argo/launch
PYTHON_SCRIPT = argo_power_control.py
POWER_SERVICE_FILE = argo_power_control.service
LOG_FILE = /var/log/argo_power_control.log

.PHONY: help install uninstall enable disable start stop restart clean record stop-record install_argo_power_control check_deps install_service enable_service create_logs clean_power_control uninstall_power_control

# Default target
all: install

# ==================== SERVICE MANAGEMENT ====================

install:
	@echo "Installing Argo services..."
	sudo cp $(LAUNCH_SERVICE) $(SERVICE_DIR)/
	sudo cp $(RECORD_SERVICE) $(SERVICE_DIR)/
	sudo systemctl daemon-reload
	mkdir -p $(BAGFILES_DIR)
	@echo "Services installed successfully!"
	@echo "Run 'make enable' to enable automatic startup"

uninstall:
	@echo "Uninstalling Argo services..."
	sudo systemctl stop $(LAUNCH_SERVICE) $(RECORD_SERVICE) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE) $(RECORD_SERVICE) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE)
	sudo rm -f $(SERVICE_DIR)/$(RECORD_SERVICE)
	sudo systemctl daemon-reload
	@echo "Services uninstalled successfully!"

enable:
	@echo "Enabling Argo services for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE)
	sudo systemctl enable $(RECORD_SERVICE)
	@echo "Services enabled!"

disable:
	@echo "Disabling automatic startup..."
	sudo systemctl disable $(LAUNCH_SERVICE)
	sudo systemctl disable $(RECORD_SERVICE)
	@echo "Automatic startup disabled!"

start:
	@echo "Starting Argo launch service with monitoring..."
	sudo systemctl start $(LAUNCH_SERVICE)
	@echo "✅ Service started! Monitoring for node failures (30s timeout)..."
	@echo "================================================"
	@echo "🔍 Monitoring journal for errors in real-time..."
	@# Monitor journal for 30 seconds, consuming output and checking for errors
	@timeout 30s journalctl -u $(LAUNCH_SERVICE) -f --no-pager | \
		( \
			ERRORS_FOUND=0; \
			PROCESS_DEATHS=0; \
			CRITICAL_ERRORS=0; \
			DEAD_NODES=""; \
			FAILURE_CAUSES=""; \
			while IFS= read -r line; do \
				echo "$$line"; \
				if echo "$$line" | grep -i "process has died" > /dev/null 2>&1; then \
					PROCESS_DEATHS=1; \
					ERRORS_FOUND=1; \
					NODE_NAME=$$(echo "$$line" | sed -n 's/.*\[\([^-]*\)-[0-9]*\]:.*process has died.*/\1/p'); \
					EXIT_CODE=$$(echo "$$line" | sed -n 's/.*exit code \([0-9]*\).*/\1/p'); \
					if [ -n "$$NODE_NAME" ]; then \
						DEAD_NODES="$$DEAD_NODES $$NODE_NAME"; \
					fi; \
				fi; \
				if echo "$$line" | grep -i -E "(CRITICAL|error.*exiting|failed.*exiting)" | grep -v -i "gps.*satellite" | grep -v -i "gps.*fix" > /dev/null 2>&1; then \
					CRITICAL_ERRORS=1; \
					ERRORS_FOUND=1; \
					ERROR_MSG=$$(echo "$$line" | sed 's/.*\[[^-]*-[0-9]*\]:[[:space:]]*//'); \
					if [ -n "$$ERROR_MSG" ]; then \
						FAILURE_CAUSES="$$FAILURE_CAUSES\n  - $$ERROR_MSG"; \
					fi; \
				fi; \
			done; \
			echo ""; \
			echo "================================================"; \
			echo "📊 Monitoring Summary:"; \
			if [ "$$PROCESS_DEATHS" -eq 1 ]; then \
				echo "❌ CRITICAL: Node processes have died!"; \
				echo ""; \
				echo "Failed nodes:"; \
				for node in $$DEAD_NODES; do \
					echo "  - $$node"; \
				done; \
				if [ -n "$$FAILURE_CAUSES" ]; then \
					echo ""; \
					echo "Failure causes:"; \
					echo -e "$$FAILURE_CAUSES"; \
				fi; \
			elif [ "$$CRITICAL_ERRORS" -eq 1 ]; then \
				echo "❌ WARNING: Critical errors detected!"; \
				if [ -n "$$FAILURE_CAUSES" ]; then \
					echo ""; \
					echo "Error details:"; \
					echo -e "$$FAILURE_CAUSES"; \
				fi; \
			else \
				echo "✅ No critical errors detected during 30s monitoring"; \
			fi; \
		) || \
		( \
			echo ""; \
			echo "================================================"; \
			echo "📊 Monitoring Summary (timeout reached):"; \
			echo "✅ 30-second monitoring completed"; \
		)

stop:
	@echo "Stopping all Argo services..."
	sudo systemctl stop $(RECORD_SERVICE) 2>/dev/null || true
	sudo systemctl stop $(LAUNCH_SERVICE) 2>/dev/null || true
	@echo "All Argo services stopped!"

restart:
	@echo "Restarting Argo launch service..."
	sudo systemctl restart $(LAUNCH_SERVICE)
	@echo "Argo service restarted!"

record:
	@echo "Starting ROS2 bag recording..."
	sudo systemctl start $(RECORD_SERVICE)
	@echo "Recording started!"

stop-record:
	@echo "Stopping ROS2 bag recording..."
	sudo systemctl stop $(RECORD_SERVICE)
	@echo "Recording stopped!"

clean:
	@echo "Cleaning old bag files (>7 days)..."
	find $(BAGFILES_DIR) -name "argo_*" -type d -mtime +7 -exec rm -rf {} \; 2>/dev/null || true
	@echo "Old bag files cleaned!"

# ==================== POWER CONTROL SYSTEM ====================

# Main power control installation target
install_argo_power_control: check_deps install_service enable_service create_logs
	@echo ""
	@echo "✅ Argo Power Control System Installation Complete!"
	@echo ""
	@echo "Next steps:"
	@echo "1. If you were just added to the gpio group, log out and log back in"
	@echo "2. The power control service will start automatically on boot"
	@echo "3. Check service status with: sudo systemctl status $(POWER_SERVICE_NAME)"
	@echo "4. View logs with: sudo journalctl -u $(POWER_SERVICE_NAME) -f"
	@echo "5. Test with: sudo systemctl start $(POWER_SERVICE_NAME)"
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
	@echo "🔧 Setting up GPIO permissions..."
	@if ! getent group gpio > /dev/null 2>&1; then \
		echo "📦 Creating gpio group..."; \
		sudo groupadd gpio; \
	else \
		echo "✅ gpio group already exists"; \
	fi
	@if ! groups $(USER) | grep -q gpio; then \
		echo "👤 Adding $(USER) to gpio group..."; \
		sudo usermod -a -G gpio $(USER); \
		echo "⚠️  IMPORTANT: You must log out and log back in for group changes to take effect!"; \
		echo "   Or run: newgrp gpio"; \
	else \
		echo "✅ $(USER) is already a member of gpio group"; \
	fi

# Install systemd service
install_service:
	@echo "📋 Installing systemd service..."
	@echo "🔧 Configuring service for user: $(USER)"
	@sed "s/User=orangepi/User=$(USER)/g" $(SCRIPT_DIR)/$(POWER_SERVICE_FILE) > /tmp/$(POWER_SERVICE_FILE)
	@sudo cp /tmp/$(POWER_SERVICE_FILE) /etc/systemd/system/$(POWER_SERVICE_FILE)
	@rm -f /tmp/$(POWER_SERVICE_FILE)
	@sudo chmod +x $(SCRIPT_DIR)/$(PYTHON_SCRIPT)
	@sudo systemctl daemon-reload
	@echo "✅ Service installed and systemd reloaded"

# Enable and start service
enable_service:
	@echo "🚀 Enabling power control service..."
	@sudo systemctl enable $(POWER_SERVICE_NAME)
	@sudo systemctl start $(POWER_SERVICE_NAME)
	@echo "✅ Service enabled and started"

# Create log file
create_logs:
	@echo "📝 Setting up log file..."
	@sudo touch $(LOG_FILE)
	@sudo chown root:root $(LOG_FILE)
	@sudo chmod 644 $(LOG_FILE)
	@echo "✅ Log file created: $(LOG_FILE)"

# Clean up power control installation
clean_power_control:
	@echo "🧹 Cleaning up power control installation..."
	@sudo systemctl stop $(POWER_SERVICE_NAME) 2>/dev/null || true
	@sudo systemctl disable $(POWER_SERVICE_NAME) 2>/dev/null || true
	@sudo rm -f /etc/systemd/system/$(POWER_SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@echo "✅ Power control cleanup complete"

# Uninstall power control system
uninstall_power_control: clean_power_control
	@echo "🗑️  Uninstalling power control system..."
	@sudo rm -f $(LOG_FILE)
	@echo "✅ Power control uninstall complete"

# Show help
help:
	@echo "Argo Launch Services Makefile"
	@echo ""
	@echo "Service Management:"
	@echo "  install              - Install service files to systemd"
	@echo "  uninstall            - Remove service files from systemd"
	@echo "  enable               - Enable services for automatic startup"
	@echo "  disable              - Disable automatic startup"
	@echo "  start                - Start argo-launch service with 30s monitoring"
	@echo "  stop                 - Stop all argo services"
	@echo "  restart              - Restart argo-launch service"
	@echo "  record               - Start ROS2 bag recording"
	@echo "  stop-record          - Stop ROS2 bag recording"
	@echo "  clean                - Clean old bag files (>7 days)"
	@echo ""
	@echo "Power Control System:"
	@echo "  install_argo_power_control  - Install the complete power control system"
	@echo "  check_deps            - Check and install required dependencies"
	@echo "  install_service       - Install systemd service only (auto-configures for current user)"
	@echo "  enable_service        - Enable and start the service"
	@echo "  create_logs           - Create log file with proper permissions"
	@echo "  clean_power_control   - Stop and disable power control service, remove service file"
	@echo "  uninstall_power_control - Complete removal of power control system"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install                   # Install launch services"
	@echo "  make start                     # Start with monitoring"
	@echo "  make install_argo_power_control # Install power control"
	@echo "  make help                      # Show this help"

