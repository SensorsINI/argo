# Makefile for Argo Power Control System
# This Makefile manages the power control system including GPIO overlay and systemd service

# Service configuration
SERVICE_DIR = /etc/systemd/system
POWER_SERVICE_NAME = argo_power_control
# Directory containing this Makefile (power_control directory in repo)
SCRIPT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
PYTHON_SCRIPT = argo_power_control.py
POWER_SERVICE_FILE = argo_power_control.service

# Detect the user performing installation (supports sudo)
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)

.PHONY: help install uninstall enable disable start stop restart check_deps install_service enable_service clean_power_control install_shutdown_hook install_boot_beep_hook install-orangpienv install-sys-heartbeat

# Default target
all: install

# ==================== POWER CONTROL SYSTEM ====================

# Main power control installation target
install: install-orangpienv install-sys-heartbeat check_deps install_service enable_service install_shutdown_hook install_boot_beep_hook
	@echo ""
	@echo "✅ Argo Power Control System Installation Complete!"
	@echo ""
	@echo "Next steps:"
	@echo "1. If you were just added to the gpio group, log out and log back in"
	@echo "2. The power control service will start automatically on boot"
	@echo "3. Check service status with: sudo systemctl status $(POWER_SERVICE_NAME)"
	@echo "4. View logs with: sudo journalctl -u $(POWER_SERVICE_NAME) -f"
	@echo "5. Test with: sudo systemctl start $(POWER_SERVICE_NAME)"
	@echo "6. Note: GPIO shutdown requires kernel driver support (checked during installation)"
	@echo ""

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

# Install authoritative orangepiEnv boot config from repo to /boot
install-orangpienv:
	@echo "📋 Installing authoritative orangepiEnv.txt to /boot..."
	@if [ ! -f "$(SCRIPT_DIR)/orangepiEnv.txt" ]; then \
		echo "❌ Source file not found: $(SCRIPT_DIR)/orangepiEnv.txt"; \
		exit 1; \
	fi
	@if [ -f /boot/orangepiEnv.txt ]; then \
		backup="/boot/orangepiEnv.txt.bak.$$(date +%Y%m%d_%H%M%S)"; \
		echo "📦 Backing up existing /boot/orangepiEnv.txt to $$backup"; \
		sudo cp /boot/orangepiEnv.txt "$$backup"; \
	fi
	@sudo install -m 0644 "$(SCRIPT_DIR)/orangepiEnv.txt" /boot/orangepiEnv.txt
	@echo "✅ Installed /boot/orangepiEnv.txt from $(SCRIPT_DIR)/orangepiEnv.txt"
	@echo "🔍 Checking for gpioset tool..."
	@if ! command -v gpioset >/dev/null 2>&1; then \
		echo "📦 Installing gpiod (or libgpiod-tools)..."; \
		sudo apt-get update; \
		sudo apt-get install -y gpiod || sudo apt-get install -y libgpiod-tools; \
	else \
		echo "✅ gpioset present"; \
	fi
	@echo "🔧 Setting up GPIO permissions..."
	@if ! getent group gpio > /dev/null 2>&1; then \
		echo "📦 Creating gpio group..."; \
		sudo groupadd gpio; \
	else \
		echo "✅ gpio group already exists"; \
	fi
	@if ! groups $(INSTALL_USER) | grep -q gpio; then \
		echo "👤 Adding $(INSTALL_USER) to gpio group..."; \
		sudo usermod -a -G gpio $(INSTALL_USER); \
		echo "⚠️  IMPORTANT: You must log out and log back in for group changes to take effect!"; \
		echo "   Or run: newgrp gpio"; \
	else \
		echo "✅ $(INSTALL_USER) is already a member of gpio group"; \
	fi

# Install systemd service
install_service:
	@echo "📋 Installing argo power control systemd service..."
	@echo "🔧 Configuring service for user: $(INSTALL_USER) in home: $(INSTALL_HOME)"
	@sed -e "s/^User=.*/User=$(INSTALL_USER)/" \
	    -e "s/^Group=.*/Group=$(INSTALL_USER)/" \
	    -e "s#^WorkingDirectory=.*#WorkingDirectory=$(INSTALL_HOME)/argo/power_control#" \
	    -e "s#^Environment=HOME=.*#Environment=HOME=$(INSTALL_HOME)#" \
	    -e "s#/home/orangepi/argo#$(INSTALL_HOME)/argo#g" \
	    $(SCRIPT_DIR)/$(POWER_SERVICE_FILE) > /tmp/$(POWER_SERVICE_FILE)
	@sudo cp /tmp/$(POWER_SERVICE_FILE) /etc/systemd/system/$(POWER_SERVICE_FILE)
	@rm -f /tmp/$(POWER_SERVICE_FILE)
	@sudo chmod +x $(SCRIPT_DIR)/$(PYTHON_SCRIPT)
	@sudo systemctl daemon-reload
	@echo "✅ Service installed and systemd reloaded"

# Install late shutdown hook (runs in system-shutdown stage)
install_shutdown_hook:
	@echo "📋 Installing late shutdown hook..."
	@sudo install -m 0755 $(SCRIPT_DIR)/argo_poweroff.shutdown /lib/systemd/system-shutdown/argo_poweroff
	@echo "✅ Late shutdown hook installed"

# Install early boot beep hook (runs once during early boot)
install_boot_beep_hook:
	@echo "📋 Installing early boot beep hook..."
	@sudo install -m 0755 $(SCRIPT_DIR)/argo_bootbeep.startup /usr/local/sbin/argo_bootbeep
	@sudo install -m 0644 $(SCRIPT_DIR)/argo_bootbeep.service /etc/systemd/system/argo_bootbeep.service
	@sudo systemctl daemon-reload
	@sudo systemctl enable argo_bootbeep.service
	@echo "✅ Early boot beep hook installed and enabled"

# Install PH4 sys_heartbeat overlay and permissions service
install-sys-heartbeat:
	@echo "📋 Installing PH4 sys_heartbeat overlay..."
	@$(MAKE) -C sys_heartbeat install
	@echo "✅ PH4 sys_heartbeat overlay installed"

# # Install early boot power-on hook (runs once at early boot)
# DISABLED: not needed anymore with rev3 board that uses dual coil relay
# install_startup_hook:
# 	@echo "📋 Installing early boot startup hook..."
# 	@sudo install -m 0755 $(SCRIPT_DIR)/argo_poweron.startup /usr/local/sbin/argo_poweron
# 	@sed -e "s|^ExecStart=.*|ExecStart=/usr/local/sbin/argo_poweron|" \
# 	    $(SCRIPT_DIR)/argo_poweron.service > /tmp/argo_poweron.service
# 	@sudo install -m 0644 /tmp/argo_poweron.service /etc/systemd/system/argo_poweron.service
# 	@rm -f /tmp/argo_poweron.service
# 	@sudo systemctl daemon-reload
# 	@sudo systemctl enable argo_poweron.service
# 	@echo "✅ Early boot startup hook installed and enabled"

# Enable and start service
enable_service:
	@echo "🚀 Enabling argo power control service..."
	@sudo systemctl enable $(POWER_SERVICE_NAME)
	@echo "🔄 Starting argo power control service..."
	@if systemctl is-active --quiet $(POWER_SERVICE_NAME); then \
		echo "✅ Service is already running"; \
	else \
		sudo systemctl start $(POWER_SERVICE_NAME); \
		echo "⏳ Waiting for service to initialize..."; \
		sleep 1; \
	fi
	@echo "📊 Checking service status..."
	@if systemctl is-active --quiet $(POWER_SERVICE_NAME); then \
		echo "✅ Service is active"; \
		if systemctl is-failed --quiet $(POWER_SERVICE_NAME); then \
			echo "❌ Service is in failed state"; \
			sudo systemctl status $(POWER_SERVICE_NAME) --no-pager; \
			exit 1; \
		fi; \
		if pgrep -f "argo_power_control.py" > /dev/null; then \
			echo "✅ argo_power_control.py process is running"; \
			echo "✅ Service enabled and running successfully"; \
		else \
			echo "❌ argo_power_control.py process not found"; \
			echo "📋 Service status:"; \
			sudo systemctl status $(POWER_SERVICE_NAME) --no-pager; \
			echo "📋 Recent logs:"; \
			sudo journalctl -u $(POWER_SERVICE_NAME) -n 10 --no-pager; \
			exit 1; \
		fi; \
	else \
		echo "❌ Service failed to start or is not active"; \
		sudo systemctl status $(POWER_SERVICE_NAME) --no-pager; \
		exit 1; \
	fi

# Start service
start:
	@echo "▶️  Starting argo power control service..."
	@sudo systemctl start $(POWER_SERVICE_NAME)
	@echo "✅ Service started"

# Stop service
stop:
	@echo "⏹️  Stopping argo power control service..."
	@sudo systemctl stop $(POWER_SERVICE_NAME)
	@echo "✅ Service stopped"

# Restart service
restart:
	@echo "🔄 Restarting argo power control service..."
	@sudo systemctl restart $(POWER_SERVICE_NAME)
	@echo "✅ Service restarted"

# Enable service for automatic startup
enable:
	@echo "🚀 Enabling argo power control service for automatic startup..."
	@sudo systemctl enable $(POWER_SERVICE_NAME)
	@echo "✅ Service enabled for automatic startup"

# Disable automatic startup
disable:
	@echo "🚫 Disabling argo power control service automatic startup..."
	@sudo systemctl disable $(POWER_SERVICE_NAME)
	@echo "✅ Service disabled for automatic startup"

# Check service status
status:
	@echo "📊 Argo Power Control Service Status:"
	@sudo systemctl status $(POWER_SERVICE_NAME) --no-pager

# Clean up power control installation
clean_power_control:
	@echo "🧹 Cleaning up power control installation..."
	@sudo systemctl stop $(POWER_SERVICE_NAME) 2>/dev/null || true
	@sudo systemctl disable $(POWER_SERVICE_NAME) 2>/dev/null || true
	@sudo rm -f /etc/systemd/system/$(POWER_SERVICE_NAME).service
	# DISABLED: not needed anymore with rev3 board that uses dual coil relay
	# @sudo systemctl disable argo_poweron.service 2>/dev/null || true
	# @sudo rm -f /etc/systemd/system/argo_poweron.service
	# @sudo rm -f /usr/local/sbin/argo_poweron
	@sudo systemctl disable argo_bootbeep.service 2>/dev/null || true
	@sudo rm -f /etc/systemd/system/argo_bootbeep.service
	@sudo rm -f /usr/local/sbin/argo_bootbeep
	@sudo rm -f /lib/systemd/system-shutdown/argo_poweroff
	@sudo systemctl daemon-reload
	@echo "✅ Power control cleanup complete"

# Uninstall power control system
uninstall: clean_power_control
	@echo "🗑️  Uninstalling power control system..."
	@echo "✅ Power control uninstall complete"

# Show help
help:
	@echo "Argo Power Control System Makefile"
	@echo ""
	@echo "Main Targets:"
	@echo "  install              - Install the complete power control system"
	@echo "  uninstall            - Remove the power control system"
	@echo "  enable               - Enable service for automatic startup"
	@echo "  disable              - Disable automatic startup"
	@echo "  start                - Start the power control service"
	@echo "  stop                 - Stop the power control service"
	@echo "  restart              - Restart the power control service"
	@echo "  status               - Show service status"
	@echo ""
	@echo "Component Targets:"
	@echo "  check_deps           - Check and install required dependencies"
	@echo "  install-orangpienv   - Install power_control/orangepiEnv.txt to /boot"
	@echo "  install-sys-heartbeat - Install PH4 LED overlay + permissions service"
	@echo "  install_service      - Install systemd service only"
	@echo "  enable_service       - Enable and start the service"
	@echo "  install_shutdown_hook - Install late shutdown hook (poweroff/halt only, not reboot)"
	@echo "  install_boot_beep_hook - Install early boot startup beep hook"
	@echo "  clean_power_control  - Stop and disable service, remove service file"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install         # Install complete power control system"
	@echo "  make start           # Start the service"
	@echo "  make status          # Check service status"
	@echo "  make help            # Show this help"

# Makefile for Argo Power Control system

SERVICE_DIR = /etc/systemd/system
CPU_SETUP_SERVICE = argo_cpu_setup.service
CPU_SETUP_SCRIPT = argo_cpu_setup.sh

.PHONY: help install-cpu-tuning uninstall-cpu-tuning

help:
	@echo "Argo Power Control Management"
	@echo "============================="
	@echo ""
	@echo "CPU Frequency Tuning:"
	@echo "  install-cpu-tuning   - Install and enable the CPU governor tuning service"
	@echo "  uninstall-cpu-tuning - Disable and remove the CPU governor tuning service"

install-cpu-tuning:
	@echo "Installing Argo CPU governor tuning service..."
	@sudo cp $(CPU_SETUP_SCRIPT) /usr/local/bin/
	@sudo chmod +x /usr/local/bin/$(CPU_SETUP_SCRIPT)
	@sudo cp $(CPU_SETUP_SERVICE) $(SERVICE_DIR)/
	@sudo systemctl daemon-reload
	@sudo systemctl enable $(CPU_SETUP_SERVICE)
	@sudo systemctl start $(CPU_SETUP_SERVICE)
	@echo "✅ Argo CPU governor tuning service installed and enabled."

uninstall-cpu-tuning:
	@echo "Uninstalling Argo CPU governor tuning service..."
	@sudo systemctl stop $(CPU_SETUP_SERVICE) || true
	@sudo systemctl disable $(CPU_SETUP_SERVICE) || true
	@sudo rm -f $(SERVICE_DIR)/$(CPU_SETUP_SERVICE)
	@sudo rm -f /usr/local/bin/$(CPU_SETUP_SCRIPT)
	@sudo systemctl daemon-reload
	@echo "✅ Argo CPU governor tuning service uninstalled."
