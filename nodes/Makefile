# Makefile for Argo Battery Water Service Installation
# This manages the independent battery_water.service for critical battery monitoring

SYSTEMD_USER_DIR := $(HOME)/.config/systemd/user
SYSTEMD_SYSTEM_DIR := /etc/systemd/system
SERVICE_FILE := argo_battery_water.service
SCRIPT_FILE := battery_water.py

.PHONY: help install uninstall status enable disable restart start stop logs check

help:
	@echo "Argo Battery Water Service Installation"
	@echo "========================================"
	@echo ""
	@echo "Available targets:"
	@echo "  install  - Install battery_water service (requires sudo)"
	@echo "  uninstall - Remove battery_water service (requires sudo)"
	@echo "  enable   - Enable service to start on boot"
	@echo "  disable  - Disable service from starting on boot"
	@echo "  start    - Start the battery service"
	@echo "  stop     - Stop the battery service"
	@echo "  restart  - Restart the battery service"
	@echo "  status   - Show service status"
	@echo "  logs     - Show service logs"
	@echo "  check    - Verify dependencies"
	@echo ""
	@echo "Quick start:"
	@echo "  sudo make install enable start"

check:
	@echo "Checking basic dependencies..."
	@command -v python3 >/dev/null 2>&1 || { echo "❌ python3 not found"; exit 1; }
	@command -v systemctl >/dev/null 2>&1 || { echo "❌ systemctl not found"; exit 1; }
	@test -f $(SCRIPT_FILE) || { echo "❌ $(SCRIPT_FILE) not found"; exit 1; }
	@test -f $(SERVICE_FILE) || { echo "❌ $(SERVICE_FILE) not found"; exit 1; }
	@echo "✅ Basic dependencies met"
	@echo "ℹ️  Python package dependencies will be checked when service starts"

install:
	@echo "Installing argo_battery_water service..."
	@if systemctl is-active --quiet argo_battery_water.service; then \
		echo "⚠️  Service is currently running. Stopping it..."; \
		sudo systemctl stop argo_battery_water.service; \
	fi
	@echo "📝 Installing service file..."
	@sudo cp $(SERVICE_FILE) $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE)
	@sudo chmod 644 $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE)
	@echo "🔄 Reloading systemd daemon..."
	@sudo systemctl daemon-reload
	@echo "🚀 Enabling and starting service..."
	@sudo systemctl enable argo_battery_water.service
	@sudo systemctl start argo_battery_water.service
	@sleep 2
	@echo ""
	@echo "✅ Argo battery water service installed successfully"
	@echo ""
	@if systemctl is-active --quiet argo_battery_water.service; then \
		echo "🟢 Service is RUNNING"; \
		echo "✅ Auto-start ENABLED"; \
	else \
		echo "❌ Service failed to start!"; \
		echo "Check logs: sudo journalctl -u argo_battery_water.service -n 20"; \
		exit 1; \
	fi
	@echo ""
	@echo "Quick status check:"
	@$(MAKE) status

uninstall:
	@echo "Uninstalling argo_battery_water service..."
	@if systemctl is-active --quiet argo_battery_water.service; then \
		echo "⚠️  Stopping service..."; \
		sudo systemctl stop argo_battery_water.service; \
	fi
	@if systemctl is-enabled --quiet argo_battery_water.service 2>/dev/null; then \
		echo "⚠️  Disabling service..."; \
		sudo systemctl disable argo_battery_water.service; \
	fi
	@if [ -f $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE) ]; then \
		echo "🗑️  Removing service file..."; \
		sudo rm -f $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE); \
	fi
	@sudo systemctl daemon-reload
	@echo "✅ Argo battery water service uninstalled"

enable:
	@echo "Enabling argo_battery_water service..."
	@sudo systemctl enable argo_battery_water.service
	@echo "✅ Service will start automatically on boot"

disable:
	@echo "Disabling argo_battery_water service..."
	@sudo systemctl disable argo_battery_water.service
	@echo "✅ Service will not start on boot"

start:
	@echo "Starting argo_battery_water service..."
	@sudo systemctl start argo_battery_water.service
	@sleep 2
	@$(MAKE) status

stop:
	@echo "Stopping argo_battery_water service..."
	@sudo systemctl stop argo_battery_water.service
	@sleep 1
	@$(MAKE) status

restart:
	@echo "Restarting argo_battery_water service..."
	@sudo systemctl restart argo_battery_water.service
	@sleep 2
	@$(MAKE) status

status:
	@echo "Argo Battery Water Service Status:"
	@echo "==================================="
	@sudo systemctl status argo_battery_water.service --no-pager -l || true
	@echo ""
	@echo "Quick status:"
	@systemctl is-active argo_battery_water.service >/dev/null 2>&1 && echo "🟢 Service is RUNNING" || echo "🔴 Service is STOPPED"
	@systemctl is-enabled argo_battery_water.service >/dev/null 2>&1 && echo "✅ Auto-start ENABLED" || echo "⚠️  Auto-start DISABLED"

logs:
	@echo "Argo Battery Water Service Logs (last 50 lines):"
	@echo "================================================="
	@sudo journalctl -u argo_battery_water.service -n 50 --no-pager

logs-follow:
	@echo "Following argo_battery_water service logs (Ctrl+C to stop):"
	@sudo journalctl -u argo_battery_water.service -f --no-pager

# ============================================================================
# BNO08x Driver (C++ ROS2 driver for BNO085 IMU)
# ============================================================================

# BNO08x driver paths and configuration
# - BNO08X_SUBMODULE_DIR: where the bno08x_driver source submodule is located
# - BNO08X_BUILD_DIR: where the bno08x_driver build workspace is located  
# - BNO08X_CONFIG: where the bno08x_driver configuration file is located, important for the I2C bus configuration
# Note: see also the build target and BNO85_README.md for more details about patch applied to the source code
BNO08X_SUBMODULE_DIR := vendor/bno08x_driver
BNO08X_BUILD_DIR := argo_bno08x_driver_workspace
BNO08X_CONFIG := vendor/bno08x_driver_argo.yaml

.PHONY: bno08x-help bno08x-init bno08x-build bno08x-test bno08x-launch bno08x-calibrate bno08x-clean

bno08x-help:
	@echo "BNO08x Driver Management"
	@echo "========================"
	@echo ""
	@echo "Targets:"
	@echo "  bno08x-init        - Initialize BNO08x submodule (if not already done)"
	@echo "  bno08x-build       - Build the BNO08x ROS2 driver with colcon"
	@echo "  bno08x-i2c-test    - Test if BNO085 hardware is detected"
	@echo "  bno08x-launch      - Launch the BNO08x C++ driver only"
	@echo "  bno08x-launch-full - Launch complete system (C++ driver + Python bridge)"
	@echo "  bno08x-calibrate   - Run interactive calibration tool (requires driver running)"
	@echo "  bno08x-clean       - Clean build artifacts"
	@echo ""
	@echo "Quick start:"
	@echo "  make bno08x-build bno08x-test bno08x-launch-full"
	@echo ""
	@echo "For Argo integration, use: bno08x-launch-full"
	@echo "This provides /compass, /pose, /accel, /gyro topics"
	@echo ""
	@echo "For calibration (with driver already running):"
	@echo "  make bno08x-calibrate"
	@echo ""
	@echo "Debugging:"
	@echo "  To enable debug output, edit vendor/bno08x_driver/include/bno08x_driver/logger.h"
	@echo "  Uncomment '#define DEBUG_LOG_ENABLED' and run 'make bno08x-build'"
	@echo ""

bno08x-init:
	@echo "Checking BNO08x submodule..."
	@if [ ! -d "$(BNO08X_SUBMODULE_DIR)/.git" ]; then \
		echo "Initializing BNO08x submodule..."; \
		git submodule update --init --recursive $(BNO08X_SUBMODULE_DIR); \
		echo "✅ BNO08x submodule initialized"; \
	else \
		echo "✅ BNO08x submodule already initialized"; \
	fi

bno08x-build: bno08x-init
	@echo "Building BNO08x ROS2 driver..."
	@echo "Creating workspace at $(BNO08X_BUILD_DIR)..."
	@mkdir -p $(BNO08X_BUILD_DIR)/src
	@if [ ! -L "$(BNO08X_BUILD_DIR)/src/bno08x_driver" ]; then \
		echo "Creating symlink to submodule..."; \
		ln -sf $(PWD)/$(BNO08X_SUBMODULE_DIR) $(BNO08X_BUILD_DIR)/src/bno08x_driver; \
	fi
	@echo "Applying Argo-specific configuration (I2C bus fix)..."
	@if grep -q '"/dev/i2c-7"' $(BNO08X_SUBMODULE_DIR)/config/bno085_i2c.yaml 2>/dev/null; then \
		echo "⚠️  Patching YAML config: I2C bus i2c-7 → i2c-0..."; \
		sed -i 's|bus: "/dev/i2c-7"|bus: "/dev/i2c-0"|g' $(BNO08X_SUBMODULE_DIR)/config/bno085_i2c.yaml; \
		echo "✅ YAML config patched"; \
	else \
		echo "✅ YAML config already patched"; \
	fi
	@if grep -q '"/dev/i2c-7"' $(BNO08X_SUBMODULE_DIR)/src/bno08x_ros.cpp 2>/dev/null; then \
		echo "⚠️  Patching C++ source: I2C bus default i2c-7 → i2c-0..."; \
		sed -i 's|"/dev/i2c-7"|"/dev/i2c-0"|g' $(BNO08X_SUBMODULE_DIR)/src/bno08x_ros.cpp; \
		echo "✅ C++ source patched"; \
	else \
		echo "✅ C++ source already patched"; \
	fi
	@echo "Sourcing ROS2 environment..."
	@bash -c "source /opt/ros/humble/setup.bash && \
		cd $(BNO08X_BUILD_DIR) && \
		echo 'Running colcon build...' && \
		colcon build --packages-select bno08x_driver --symlink-install"
	@echo "✅ BNO08x driver built successfully"
	@echo ""
	@echo "To use the driver, source the workspace:"
	@echo "  source $(BNO08X_BUILD_DIR)/install/setup.bash"

bno08x-i2c-test:
	@echo "Testing BNO085 I2C connection..."
	@echo "Checking if i2c-0 exists..."
	@if [ ! -e "/dev/i2c-0" ]; then \
		echo "❌ /dev/i2c-0 not found"; \
		echo "   Run: sudo modprobe i2c-dev"; \
		exit 1; \
	fi
	@echo "✅ /dev/i2c-0 exists"
	@echo ""
	@echo "Scanning I2C bus 0 for BNO085 at 0x4a..."
	@if command -v i2cdetect >/dev/null 2>&1; then \
		i2cdetect -y 0 | grep -E "(4a|UU)" && \
		echo "✅ BNO085 detected at 0x4a on bus 0" || \
		echo "❌ BNO085 not detected - check wiring"; \
	else \
		echo "⚠️  i2cdetect not found, install i2c-tools: sudo apt install i2c-tools"; \
	fi

bno08x-launch: bno08x-build
	@echo "Launching BNO08x driver with Argo config..."
	@bash -c "source /opt/ros/humble/setup.bash && \
		source $(BNO08X_BUILD_DIR)/install/setup.bash && \
		ros2 run bno08x_driver bno08x_driver --ros-args --params-file $(BNO08X_CONFIG)"

bno08x-launch-full: bno08x-build
	@echo "Launching BNO085 complete system (C++ driver + Python bridge)..."
	@echo "Starting C++ driver in background..."
	@bash -c "source /opt/ros/humble/setup.bash && \
		source $(BNO08X_BUILD_DIR)/install/setup.bash && \
		ros2 run bno08x_driver bno08x_driver --ros-args --params-file $(BNO08X_CONFIG) &"
	@sleep 3
	@echo "Starting Python bridge..."
	@bash -c "source /opt/ros/humble/setup.bash && \
		python3 bno085.py bridge"

bno08x-calibrate: bno08x-build
	@echo "Starting BNO085 calibration tool..."
	@echo "This will guide you through the calibration process."
	@echo "Make sure the BNO08x driver is running!"
	@echo ""
	@bash -c "source /opt/ros/humble/setup.bash && \
		python3 bno085.py calibrate --duration 120"

bno08x-clean:
	@echo "Cleaning BNO08x build artifacts..."
	@rm -rf $(BNO08X_BUILD_DIR)
	@echo "✅ Build artifacts cleaned"

# ============================================================================
# BNO085 Service Management (Systemd integration)
# ============================================================================

BNO085_SERVICE_FILE := argo_bno085.service
BNO085_SERVICE_DEST := /etc/systemd/system/$(BNO085_SERVICE_FILE)

.PHONY: bno085-service-install bno085-service-uninstall bno085-service-enable bno085-service-disable
.PHONY: bno085-service-start bno085-service-stop bno085-service-restart bno085-service-status bno085-service-logs

bno085-service-install: bno08x-build
	@echo "Installing argo_bno085 service..."
	@if systemctl is-active --quiet argo_bno085.service; then \
		echo "⏸️  Stopping existing service..."; \
		sudo systemctl stop argo_bno085.service; \
	fi
	@echo "📝 Installing service file with symbolic link to source file..."
	@sudo ln -sf $(PWD)/$(BNO085_SERVICE_FILE) $(BNO085_SERVICE_DEST)
	@sudo systemctl daemon-reload
	@echo "✅ Argo BNO085 service installed successfully"

bno085-service-uninstall:
	@echo "Uninstalling argo_bno085 service..."
	@if systemctl is-active --quiet argo_bno085.service; then \
		echo "⏸️  Stopping service..."; \
		sudo systemctl stop argo_bno085.service; \
	fi
	@if systemctl is-enabled --quiet argo_bno085.service 2>/dev/null; then \
		echo "🔧 Disabling service..."; \
		sudo systemctl disable argo_bno085.service; \
	fi
	@if [ -f "$(BNO085_SERVICE_DEST)" ]; then \
		sudo rm $(BNO085_SERVICE_DEST); \
		sudo systemctl daemon-reload; \
	fi
	@echo "✅ Argo BNO085 service uninstalled"

bno085-service-enable:
	@echo "Enabling argo_bno085 service..."
	@sudo systemctl enable argo_bno085.service
	@echo "✅ Service will start automatically on boot"

bno085-service-disable:
	@echo "Disabling argo_bno085 service..."
	@sudo systemctl disable argo_bno085.service
	@echo "✅ Service will not start automatically on boot"

bno085-service-start:
	@echo "Starting argo_bno085 service..."
	@sudo systemctl start argo_bno085.service
	@sleep 1
	@systemctl is-active argo_bno085.service && echo "✅ Service started" || echo "❌ Service failed to start"

bno085-service-stop:
	@echo "Stopping argo_bno085.service..."
	@sudo systemctl stop argo_bno085.service
	@echo "✅ Service stopped"

bno085-service-restart:
	@echo "Restarting argo_bno085 service..."
	@sudo systemctl restart argo_bno085.service
	@sleep 1
	@systemctl is-active argo_bno085.service && echo "✅ Service restarted" || echo "❌ Service failed to restart"

bno085-service-status:
	@echo "Argo BNO085 Service Status:"
	@echo ""
	@sudo systemctl status argo_bno085.service --no-pager -l || true
	@echo ""
	@systemctl is-active argo_bno085.service >/dev/null 2>&1 && echo "🟢 Service is RUNNING" || echo "🔴 Service is STOPPED"
	@systemctl is-enabled argo_bno085.service >/dev/null 2>&1 && echo "✅ Auto-start ENABLED" || echo "⚠️  Auto-start DISABLED"

bno085-service-logs:
	@echo "Argo BNO085 Service Logs (last 50 lines):"
	@echo ""
	@sudo journalctl -u argo_bno085.service -n 50 --no-pager

