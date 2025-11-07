# Makefile for Argo Launch Services
# This Makefile manages systemd services for ROS2 Argo robot and power control system

# Service configuration
SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE_LEGACY = argo_launch.service
LAUNCH_SERVICE_STANDARD = argo_launch_standard.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

# Detect installing user and repo directory
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)
REPO_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)

# Power control configuration moved to power_control/ directory

.PHONY: help install uninstall enable disable start stop restart clean install-legacy uninstall-legacy enable-legacy disable-legacy install-standard uninstall-standard enable-standard disable-standard start-standard stop-standard restart-standard install_storage_monitor enable_storage_monitor start_storage_monitor stop_storage_monitor restart_storage_monitor status_storage_monitor disable_storage_monitor uninstall_storage_monitor install_thermal_monitor enable_thermal_monitor start_thermal_monitor stop_thermal_monitor status_thermal_monitor disable_thermal_monitor uninstall_thermal_monitor install_health_monitor uninstall_health_monitor install_battery_water_monitor install_bno085

# Default target
all: install

# ==================== SERVICE MANAGEMENT (New Standard ROS2 Launch) ====================

install: install-standard

uninstall: uninstall-standard

enable: enable-standard

disable: disable-standard

start: start-standard

stop: stop-standard

restart: restart-standard

install-standard:
	@echo "Installing Argo STANDARD ROS2 launch service..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)#" \
	    -e "s#^EnvironmentFile=.*#EnvironmentFile=$(REPO_DIR)/launch/argo.env#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(LAUNCH_SERVICE_STANDARD) > /tmp/$(LAUNCH_SERVICE_STANDARD)
	@sudo cp /tmp/$(LAUNCH_SERVICE_STANDARD) $(SERVICE_DIR)/
	@rm -f /tmp/$(LAUNCH_SERVICE_STANDARD)
	sudo systemctl daemon-reload
	@echo "Standard service installed successfully!"
	@echo "Run 'make enable' to enable automatic startup"

uninstall-standard:
	@echo "Uninstalling Argo STANDARD ROS2 launch service..."
	sudo systemctl stop $(LAUNCH_SERVICE_STANDARD) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE_STANDARD) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE_STANDARD)
	sudo systemctl daemon-reload
	@echo "Standard service uninstalled successfully!"

enable-standard:
	@echo "Enabling Argo STANDARD service for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE_STANDARD)
	@echo "Standard service enabled!"

disable-standard:
	@echo "Disabling automatic startup for STANDARD service..."
	sudo systemctl disable $(LAUNCH_SERVICE_STANDARD)
	@echo "Automatic startup disabled for standard service!"

start-standard:
	@echo "Starting Argo STANDARD ROS2 launch service..."
	@sudo systemctl start $(LAUNCH_SERVICE_STANDARD)
	@echo "Standard service started."

stop-standard:
	@echo "Stopping Argo STANDARD ROS2 launch service..."
	@sudo systemctl stop $(LAUNCH_SERVICE_STANDARD)
	@echo "Standard service stopped."

restart-standard:
	@echo "Restarting Argo STANDARD ROS2 launch service..."
	@sudo systemctl restart $(LAUNCH_SERVICE_STANDARD)
	@echo "Standard service restarted."


# ==================== SERVICE MANAGEMENT (Old Legacy Lifecycle Manager) ====================

install-legacy: install_storage_monitor install_thermal_monitor install_battery_water_monitor install_bno085
	@echo "Installing Argo LEGACY services..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)#" \
	    -e "s#^EnvironmentFile=.*#EnvironmentFile=$(REPO_DIR)/launch/argo.env#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(LAUNCH_SERVICE_LEGACY) > /tmp/$(LAUNCH_SERVICE_LEGACY)
	@sudo cp /tmp/$(LAUNCH_SERVICE_LEGACY) $(SERVICE_DIR)/
	@rm -f /tmp/$(LAUNCH_SERVICE_LEGACY)
	sudo systemctl daemon-reload
	mkdir -p $(BAGFILES_DIR)
	@echo "LEGACY services installed successfully!"
	@echo "Run 'make enable-legacy' to enable automatic startup"

uninstall-legacy: uninstall_storage_monitor uninstall_thermal_monitor
	@echo "Uninstalling Argo LEGACY services..."
	sudo systemctl stop $(LAUNCH_SERVICE_LEGACY) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE_LEGACY) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE_LEGACY)
	sudo systemctl daemon-reload
	@echo "LEGACY services uninstalled successfully!"

enable-legacy: enable_storage_monitor enable_thermal_monitor
	@echo "Enabling Argo LEGACY services for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE_LEGACY)
	@echo "LEGACY services enabled!"

disable-legacy: disable_storage_monitor disable_thermal_monitor
	@echo "Disabling automatic startup for LEGACY services..."
	sudo systemctl disable $(LAUNCH_SERVICE_LEGACY)
	@echo "Automatic startup disabled for LEGACY services!"

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

# ==================== BATTERY & WATER MONITORING SYSTEM ====================

# Battery & Water monitoring service configuration
BATTERY_WATER_SERVICE_NAME = argo_battery_water
BATTERY_WATER_SERVICE_FILE = ../nodes/argo_battery_water.service

# Install Health Monitor service
HEALTH_MONITOR_SERVICE_NAME = argo_health_monitor
HEALTH_MONITOR_SERVICE_FILE = argo_health_monitor.service

install_health_monitor:
	@echo "📋 Installing argo health monitor systemd service..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)/launch#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(HEALTH_MONITOR_SERVICE_FILE) > /tmp/$(HEALTH_MONITOR_SERVICE_NAME).service
	@sudo cp /tmp/$(HEALTH_MONITOR_SERVICE_NAME).service $(SERVICE_DIR)/
	@rm -f /tmp/$(HEALTH_MONITOR_SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@sudo systemctl enable $(HEALTH_MONITOR_SERVICE_NAME).service
	@echo "✅ Health monitor service installed and enabled!"

uninstall_health_monitor:
	@echo "🗑️  Uninstalling argo health monitor service..."
	@sudo systemctl stop $(HEALTH_MONITOR_SERVICE_NAME).service 2>/dev/null || true
	@sudo systemctl disable $(HEALTH_MONITOR_SERVICE_NAME).service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/$(HEALTH_MONITOR_SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@echo "✅ Health monitor service uninstalled!"

# Install Battery & Water monitoring service
install_battery_water_monitor:
	@echo "📋 Installing argo battery & water monitor systemd service..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)/nodes#" \
	    -e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
	    $(BATTERY_WATER_SERVICE_FILE) > /tmp/$(BATTERY_WATER_SERVICE_NAME).service
	@sudo cp /tmp/$(BATTERY_WATER_SERVICE_NAME).service $(SERVICE_DIR)/
	@rm -f /tmp/$(BATTERY_WATER_SERVICE_NAME).service
	@sudo systemctl daemon-reload
	@echo "✅ Battery & Water monitor service installed"

# ==================== BNO085 IMU SYSTEM ====================

# BNO085 service configuration
BNO085_SERVICE_NAME = argo_bno085
BNO085_SERVICE_FILE = ../nodes/argo_bno085.service

# Install BNO085 service
install_bno085:
	@echo "📋 Installing argo_bno085 IMU systemd service..."
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
		-e "s/^Group=.*/Group=$(INSTALL_USER)/" \
		-e "s#^WorkingDirectory=.*#WorkingDirectory=$(REPO_DIR)/nodes#" \
		-e "s#/home/orangepi/argo#$(REPO_DIR)#g" \
		$(BNO085_SERVICE_FILE) > /tmp/$(BNO085_SERVICE_NAME).service
	@sudo cp /tmp/$(BNO085_SERVICE_NAME).service $(SERVICE_DIR)/
	@rm -f /tmp/$(BNO085_SERVICE_NAME).service
	@# Also make the launcher script executable
	@sudo chmod +x $(REPO_DIR)/nodes/bno085_driver_launcher.sh
	@sudo systemctl daemon-reload
	@echo "✅ BNO085 IMU service installed"

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
	@echo "Primary Service Management (Standard ROS2 Launch):"
	@echo "  install              - Install the standard ROS2 launch service"
	@echo "  uninstall            - Remove the standard service from systemd"
	@echo "  enable               - Enable standard service for automatic startup"
	@echo "  disable              - Disable automatic startup for standard service"
	@echo "  start                - Start the standard ROS2 launch service"
	@echo "  stop                 - Stop the standard ROS2 launch service"
	@echo "  restart              - Restart the standard ROS2 launch service"
	@echo ""
	@echo "Legacy Service Management (Old Lifecycle Manager):"
	@echo "  install-legacy       - Install the legacy lifecycle manager service"
	@echo "  uninstall-legacy     - Remove the legacy service from systemd"
	@echo "  enable-legacy        - Enable legacy service for automatic startup"
	@echo "  disable-legacy       - Disable automatic startup for legacy service"
	@echo ""
	@echo "Other Commands:"
	@echo "  clean                - Clean old bag files (>7 days)"
	@echo ""
	@echo "Component Services:"
	@echo "  install_storage_monitor     - Install storage monitor systemd service"
	@echo "  install_health_monitor      - Install health monitor systemd service"
	@echo "  uninstall_health_monitor    - Uninstall health monitor systemd service"
	@echo "  (see other targets for storage, thermal, power, etc.)"
	@echo ""
	@echo "Power Control System:"
	@echo "  (Power control has been moved to power_control/ directory)"
	@echo "  make -C ../power_control install  - Install the complete power control system"
	@echo "  make -C ../power_control help     - Show power control help"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install                   # Install standard ROS2 launch service"
	@echo "  make start                     # Start standard service"
	@echo "  make install-legacy            # Install old lifecycle manager service"
	@echo "  make -C ../power_control install # Install power control"
	@echo "  make help                      # Show this help"

