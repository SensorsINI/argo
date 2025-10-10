# Makefile for Argo Battery Water Service Installation
# This manages the independent battery_water.service for critical battery monitoring

SYSTEMD_USER_DIR := $(HOME)/.config/systemd/user
SYSTEMD_SYSTEM_DIR := /etc/systemd/system
SERVICE_FILE := battery_water.service
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
	@echo "Installing battery_water service..."
	@if systemctl is-active --quiet battery_water.service; then \
		echo "⚠️  Service is currently running. Stopping it..."; \
		sudo systemctl stop battery_water.service; \
	fi
	@echo "📝 Installing service file..."
	@sudo cp $(SERVICE_FILE) $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE)
	@sudo chmod 644 $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE)
	@echo "🔄 Reloading systemd daemon..."
	@sudo systemctl daemon-reload
	@echo "🚀 Enabling and starting service..."
	@sudo systemctl enable battery_water.service
	@sudo systemctl start battery_water.service
	@sleep 2
	@echo ""
	@echo "✅ Battery water service installed successfully"
	@echo ""
	@if systemctl is-active --quiet battery_water.service; then \
		echo "🟢 Service is RUNNING"; \
		echo "✅ Auto-start ENABLED"; \
	else \
		echo "❌ Service failed to start!"; \
		echo "Check logs: sudo journalctl -u battery_water.service -n 20"; \
		exit 1; \
	fi
	@echo ""
	@echo "Quick status check:"
	@$(MAKE) status

uninstall:
	@echo "Uninstalling battery_water service..."
	@if systemctl is-active --quiet battery_water.service; then \
		echo "⚠️  Stopping service..."; \
		sudo systemctl stop battery_water.service; \
	fi
	@if systemctl is-enabled --quiet battery_water.service 2>/dev/null; then \
		echo "⚠️  Disabling service..."; \
		sudo systemctl disable battery_water.service; \
	fi
	@if [ -f $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE) ]; then \
		echo "🗑️  Removing service file..."; \
		sudo rm -f $(SYSTEMD_SYSTEM_DIR)/$(SERVICE_FILE); \
	fi
	@sudo systemctl daemon-reload
	@echo "✅ Battery water service uninstalled"

enable:
	@echo "Enabling battery_water service..."
	@sudo systemctl enable battery_water.service
	@echo "✅ Service will start automatically on boot"

disable:
	@echo "Disabling battery_water service..."
	@sudo systemctl disable battery_water.service
	@echo "✅ Service will not start on boot"

start:
	@echo "Starting battery_water service..."
	@sudo systemctl start battery_water.service
	@sleep 2
	@$(MAKE) status

stop:
	@echo "Stopping battery_water service..."
	@sudo systemctl stop battery_water.service
	@sleep 1
	@$(MAKE) status

restart:
	@echo "Restarting battery_water service..."
	@sudo systemctl restart battery_water.service
	@sleep 2
	@$(MAKE) status

status:
	@echo "Battery Water Service Status:"
	@echo "=============================="
	@sudo systemctl status battery_water.service --no-pager -l || true
	@echo ""
	@echo "Quick status:"
	@systemctl is-active battery_water.service >/dev/null 2>&1 && echo "🟢 Service is RUNNING" || echo "🔴 Service is STOPPED"
	@systemctl is-enabled battery_water.service >/dev/null 2>&1 && echo "✅ Auto-start ENABLED" || echo "⚠️  Auto-start DISABLED"

logs:
	@echo "Battery Water Service Logs (last 50 lines):"
	@echo "==========================================="
	@sudo journalctl -u battery_water.service -n 50 --no-pager

logs-follow:
	@echo "Following battery_water service logs (Ctrl+C to stop):"
	@sudo journalctl -u battery_water.service -f --no-pager

