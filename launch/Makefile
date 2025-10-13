# Makefile for Argo Launch Services
# This Makefile manages systemd services for ROS2 Argo robot and power control system

# Service configuration
SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo_launch.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

# Detect installing user and repo directory
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)
REPO_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)

# Power control configuration moved to power_control/ directory

.PHONY: help install uninstall enable disable start stop restart clean install_storage_monitor enable_storage_monitor start_storage_monitor stop_storage_monitor restart_storage_monitor status_storage_monitor disable_storage_monitor uninstall_storage_monitor install_thermal_monitor enable_thermal_monitor start_thermal_monitor stop_thermal_monitor status_thermal_monitor disable_thermal_monitor uninstall_thermal_monitor

# Default target
all: install

# ==================== SERVICE MANAGEMENT ====================

install: install_storage_monitor install_thermal_monitor
	@echo "Installing Argo services..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)#" \
	    -e "s#^EnvironmentFile=.*#EnvironmentFile=$(REPO_DIR)/launch/argo.env#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(LAUNCH_SERVICE) > /tmp/$(LAUNCH_SERVICE)
	@sudo cp /tmp/$(LAUNCH_SERVICE) $(SERVICE_DIR)/
	@rm -f /tmp/$(LAUNCH_SERVICE)
	sudo systemctl daemon-reload
	mkdir -p $(BAGFILES_DIR)
	@echo "Services installed successfully!"
	@echo "Run 'make enable' to enable automatic startup"

uninstall: uninstall_storage_monitor uninstall_thermal_monitor
	@echo "Uninstalling Argo services..."
	sudo systemctl stop $(LAUNCH_SERVICE) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE)
	sudo systemctl daemon-reload
	@echo "Services uninstalled successfully!"

enable: enable_storage_monitor enable_thermal_monitor
	@echo "Enabling Argo services for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE)
	@echo "Services enabled!"

disable: disable_storage_monitor disable_thermal_monitor
	@echo "Disabling automatic startup..."
	sudo systemctl disable $(LAUNCH_SERVICE)
	@echo "Automatic startup disabled!"



clean:
	@echo "Cleaning old bag files (>7 days)..."
	find $(BAGFILES_DIR) -name "argo_*" -type d -mtime +7 -exec rm -rf {} \; 2>/dev/null || true
	@echo "Old bag files cleaned!"

# ==================== STORAGE MONITORING SYSTEM ====================

# Storage monitoring service configuration
STORAGE_SERVICE_NAME = argo_storage_monitor
STORAGE_SERVICE_FILE = argo_storage_monitor.service

# Install storage monitoring service
install_storage_monitor:
	@echo "📋 Installing argo storage monitor systemd service..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(STORAGE_SERVICE_FILE) > /tmp/$(STORAGE_SERVICE_FILE)
	@sudo cp /tmp/$(STORAGE_SERVICE_FILE) $(SERVICE_DIR)/
	@rm -f /tmp/$(STORAGE_SERVICE_FILE)
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

# ==================== THERMAL MONITORING SYSTEM ====================

# Thermal monitoring service configuration
THERMAL_SERVICE_NAME = argo_thermal_monitor
THERMAL_SERVICE_FILE = argo_thermal_monitor.service
THERMAL_SCRIPT = argo_thermal_monitor.sh
SCRIPT_INSTALL_DIR = /usr/local/bin

# Install thermal monitoring service
install_thermal_monitor:
	@echo "📋 Installing thermal monitor service..."
	@sudo cp $(THERMAL_SCRIPT) $(SCRIPT_INSTALL_DIR)/$(THERMAL_SCRIPT)
	@sudo chmod +x $(SCRIPT_INSTALL_DIR)/$(THERMAL_SCRIPT)
	@sudo cp $(THERMAL_SERVICE_FILE) $(SERVICE_DIR)/$(THERMAL_SERVICE_FILE)
	@sudo systemctl daemon-reload
	@echo "✅ Thermal monitor service installed"

# Enable thermal monitoring service
enable_thermal_monitor:
	@echo "🚀 Enabling thermal monitor service..."
	@sudo systemctl enable $(THERMAL_SERVICE_NAME)
	@echo "✅ Thermal monitor service enabled for automatic startup"

# Start thermal monitoring service
start_thermal_monitor:
	@echo "▶️  Starting thermal monitor service..."
	@sudo systemctl start $(THERMAL_SERVICE_NAME)
	@echo "✅ Thermal monitor service started"

# Stop thermal monitoring service
stop_thermal_monitor:
	@echo "⏹️  Stopping thermal monitor service..."
	@sudo systemctl stop $(THERMAL_SERVICE_NAME)
	@echo "✅ Thermal monitor service stopped"

# Check thermal monitoring service status
status_thermal_monitor:
	@echo "📊 Thermal Monitor Service Status:"
	@sudo systemctl status $(THERMAL_SERVICE_NAME) --no-pager

# Disable thermal monitoring service
disable_thermal_monitor:
	@echo "🚫 Disabling thermal monitor service..."
	@sudo systemctl disable $(THERMAL_SERVICE_NAME)
	@echo "✅ Thermal monitor service disabled"

# Uninstall thermal monitoring service
uninstall_thermal_monitor: stop_thermal_monitor disable_thermal_monitor
	@echo "🗑️  Uninstalling thermal monitor service..."
	@sudo rm -f $(SERVICE_DIR)/$(THERMAL_SERVICE_FILE)
	@sudo rm -f $(SCRIPT_INSTALL_DIR)/$(THERMAL_SCRIPT)
	@sudo systemctl daemon-reload
	@echo "✅ Thermal monitor service uninstalled"

# ==================== POWER CONTROL SYSTEM ====================
# Power control system has been moved to power_control/ directory
# Use 'make -C ../power_control <target>' to manage power control

# Show help
help:
	@echo "Argo Launch Services Makefile"
	@echo ""
	@echo "Service Management:"
	@echo "  install              - Install ALL services (launch, storage monitor)"
	@echo "  uninstall            - Remove ALL services from systemd"
	@echo "  enable               - Enable ALL services for automatic startup"
	@echo "  disable              - Disable automatic startup for ALL services"
	@echo "  start                - Start argo-launch service with 30s monitoring"
	@echo "  stop                 - Stop ALL argo services"
	@echo "  restart              - Restart argo-launch service"
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
	@echo "Thermal Monitoring System:"
	@echo "  install_thermal_monitor     - Install thermal monitor systemd service"
	@echo "  enable_thermal_monitor      - Enable thermal monitor for automatic startup"
	@echo "  start_thermal_monitor       - Start thermal monitor service"
	@echo "  stop_thermal_monitor        - Stop thermal monitor service"
	@echo "  status_thermal_monitor      - Show thermal monitor service status"
	@echo "  disable_thermal_monitor     - Disable automatic startup"
	@echo "  uninstall_thermal_monitor   - Complete removal of thermal monitor service"
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

