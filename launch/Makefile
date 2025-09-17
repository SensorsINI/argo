# Makefile for Argo Launch Services
# This Makefile manages systemd services for ROS2 Argo robot and power control system

# Service configuration
SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

# Power control configuration moved to power_control/ directory

.PHONY: help install uninstall enable disable start stop restart clean record stop-record install_storage_monitor enable_storage_monitor start_storage_monitor stop_storage_monitor restart_storage_monitor status_storage_monitor disable_storage_monitor uninstall_storage_monitor

# Default target
all: install

# ==================== SERVICE MANAGEMENT ====================

install: install_storage_monitor
	@echo "Installing Argo services..."
	sudo cp $(LAUNCH_SERVICE) $(SERVICE_DIR)/
	sudo systemctl daemon-reload
	mkdir -p $(BAGFILES_DIR)
	@echo "Services installed successfully!"
	@echo "Run 'make enable' to enable automatic startup"

uninstall: uninstall_storage_monitor
	@echo "Uninstalling Argo services..."
	sudo systemctl stop $(LAUNCH_SERVICE) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE)
	sudo systemctl daemon-reload
	@echo "Services uninstalled successfully!"

enable: enable_storage_monitor
	@echo "Enabling Argo services for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE)
	@echo "Services enabled!"

disable: disable_storage_monitor
	@echo "Disabling automatic startup..."
	sudo systemctl disable $(LAUNCH_SERVICE)
	@echo "Automatic startup disabled!"


record:
	@echo "Starting ROS2 bag recording via ROS2 service..."
	@if systemctl is-active --quiet $(LAUNCH_SERVICE); then \
		ros2 service call /argo/recording/start std_srvs/srv/Empty; \
		echo "Recording started via ROS2 service!"; \
	else \
		echo "❌ Error: Argo launch service is not running"; \
		echo "   Start it first with: make start"; \
		exit 1; \
	fi

stop-record:
	@echo "Stopping ROS2 bag recording via ROS2 service..."
	@if systemctl is-active --quiet $(LAUNCH_SERVICE); then \
		ros2 service call /argo/recording/stop std_srvs/srv/Empty; \
		echo "Recording stopped via ROS2 service!"; \
	else \
		echo "❌ Error: Argo launch service is not running"; \
		echo "   Start it first with: make start"; \
		exit 1; \
	fi

clean:
	@echo "Cleaning old bag files (>7 days)..."
	find $(BAGFILES_DIR) -name "argo_*" -type d -mtime +7 -exec rm -rf {} \; 2>/dev/null || true
	@echo "Old bag files cleaned!"

# ==================== STORAGE MONITORING SYSTEM ====================

# Storage monitoring service configuration
STORAGE_SERVICE_NAME = argo-storage-monitor
STORAGE_SERVICE_FILE = argo-storage-monitor.service

# Install storage monitoring service
install_storage_monitor:
	@echo "📋 Installing argo storage monitor systemd service..."
	@sudo cp $(STORAGE_SERVICE_FILE) $(SERVICE_DIR)/
	@sudo systemctl daemon-reload
	@echo "✅ Storage monitor service installed"

# Enable storage monitoring service
enable_storage_monitor:
	@echo "🚀 Enabling argo storage monitor service..."
	@sudo systemctl enable $(STORAGE_SERVICE_NAME)
	@echo "✅ Storage monitor service enabled for automatic startup"

# Start storage monitoring service
start_storage_monitor:
	@echo "▶️  Starting argo storage monitor service..."
	@sudo systemctl start $(STORAGE_SERVICE_NAME)
	@echo "✅ Storage monitor service started"

# Stop storage monitoring service
stop_storage_monitor:
	@echo "⏹️  Stopping argo storage monitor service..."
	@sudo systemctl stop $(STORAGE_SERVICE_NAME)
	@echo "✅ Storage monitor service stopped"

# Restart storage monitoring service
restart_storage_monitor:
	@echo "🔄 Restarting argo storage monitor service..."
	@sudo systemctl restart $(STORAGE_SERVICE_NAME)
	@echo "✅ Storage monitor service restarted"

# Check storage monitoring service status
status_storage_monitor:
	@echo "📊 Argo Storage Monitor Service Status:"
	@sudo systemctl status $(STORAGE_SERVICE_NAME) --no-pager

# Disable storage monitoring service
disable_storage_monitor:
	@echo "🚫 Disabling argo storage monitor service..."
	@sudo systemctl disable $(STORAGE_SERVICE_NAME)
	@echo "✅ Storage monitor service disabled"

# Uninstall storage monitoring service
uninstall_storage_monitor: stop_storage_monitor disable_storage_monitor
	@echo "🗑️  Uninstalling argo storage monitor service..."
	@sudo rm -f $(SERVICE_DIR)/$(STORAGE_SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@echo "✅ Storage monitor service uninstalled"

# ==================== POWER CONTROL SYSTEM ====================
# Power control system has been moved to power_control/ directory
# Use 'make -C ../power_control <target>' to manage power control

# Show help
help:
	@echo "Argo Launch Services Makefile"
	@echo ""
	@echo "Service Management:"
	@echo "  install              - Install ALL services (launch, record, storage monitor)"
	@echo "  uninstall            - Remove ALL services from systemd"
	@echo "  enable               - Enable ALL services for automatic startup"
	@echo "  disable              - Disable automatic startup for ALL services"
	@echo "  start                - Start argo-launch service with 30s monitoring"
	@echo "  stop                 - Stop ALL argo services"
	@echo "  restart              - Restart argo-launch service"
	@echo "  record               - Start ROS2 bag recording (via ROS2 service)"
	@echo "  stop-record          - Stop ROS2 bag recording (via ROS2 service)"
	@echo "  clean                - Clean old bag files (>7 days)"
	@echo ""
	@echo "Storage Monitoring System:"
	@echo "  install_storage_monitor     - Install storage monitor systemd service"
	@echo "  enable_storage_monitor      - Enable storage monitor for automatic startup"
	@echo "  start_storage_monitor       - Start storage monitor service"
	@echo "  stop_storage_monitor        - Stop storage monitor service"
	@echo "  restart_storage_monitor     - Restart storage monitor service"
	@echo "  status_storage_monitor      - Show storage monitor service status"
	@echo "  disable_storage_monitor     - Disable automatic startup"
	@echo "  uninstall_storage_monitor   - Complete removal of storage monitor service"
	@echo ""
	@echo "Power Control System:"
	@echo "  (Power control has been moved to power_control/ directory)"
	@echo "  make -C ../power_control install  - Install the complete power control system"
	@echo "  make -C ../power_control help     - Show power control help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install                   # Install launch services"
	@echo "  make start                     # Start with monitoring"
	@echo "  make -C ../power_control install # Install power control"
	@echo "  make help                      # Show this help"

