# Argo Battery Status Monitor Installation Makefile
# Installs and configures battery status monitoring for XFCE4 desktop

.PHONY: install-battery-monitor install-genmon-plugin setup-battery-panel test-battery-status clean-battery-monitor help

# Default target
all: install-battery-monitor

# Install the xfce4-genmon-plugin package
install-genmon-plugin:
	@echo "🔧 Installing xfce4-genmon-plugin..."
	@if ! dpkg -l | grep -q xfce4-genmon-plugin; then \
		sudo apt update && sudo apt install -y xfce4-genmon-plugin; \
		echo "✅ xfce4-genmon-plugin installed successfully"; \
	else \
		echo "✅ xfce4-genmon-plugin already installed"; \
	fi

# Make all battery status scripts executable
make-scripts-executable:
	@echo "🔧 Making battery status scripts executable..."
	@chmod +x battery_status_*.sh
	@chmod +x battery_status_*.py
	@chmod +x setup_battery_panel.sh
	@echo "✅ All battery status scripts are now executable"

# Install battery status monitor (main target)
install-battery-monitor: install-genmon-plugin make-scripts-executable
	@echo "🔋 Installing Argo Battery Status Monitor..."
	@echo "✅ Battery status monitor installation complete"
	@echo ""
	@echo "📋 Next steps:"
	@echo "1. Run 'make setup-battery-panel' to configure the XFCE4 panel"
	@echo "2. Or follow the manual setup instructions in docs/BATTERY_PANEL_SETUP.md"

# Setup battery panel in XFCE4
setup-battery-panel: install-battery-monitor
	@echo "🎯 Setting up battery status panel in XFCE4..."
	@./setup_battery_panel.sh

# Test battery status functionality
test-battery-status:
	@echo "🧪 Testing battery status functionality..."
	@echo "Testing simple status display..."
	@./battery_status_wrapper.sh
	@echo ""
	@echo "Testing genmon XML output..."
	@./battery_status_genmon_wrapper.sh
	@echo ""
	@echo "✅ All battery status tests completed"

# Test ROS2 service availability
test-battery-service:
	@echo "🔍 Testing battery service availability..."
	@if ros2 service list 2>/dev/null | grep -q "/battery_status"; then \
		echo "✅ Battery service is available"; \
		ros2 service call /battery_status std_srvs/srv/Trigger; \
	else \
		echo "❌ Battery service not available - make sure battery_water node is running"; \
		echo "   Start with: python3 ../nodes/battery_water.py"; \
	fi


# Clean up battery monitor files
clean-battery-monitor:
	@echo "🧹 Cleaning up battery monitor files..."
	@rm -f /tmp/argo_battery_status.txt
	@echo "✅ Battery monitor cleanup complete"

# Show help
help:
	@echo "Argo Battery Status Monitor Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  install-battery-monitor  - Install battery status monitor (default)"
	@echo "  install-genmon-plugin    - Install xfce4-genmon-plugin package"
	@echo "  make-scripts-executable  - Make all battery scripts executable"
	@echo "  setup-battery-panel      - Configure XFCE4 panel with battery status"
	@echo "  test-battery-status      - Test battery status scripts"
	@echo "  test-battery-service     - Test ROS2 battery service"
	@echo "  clean-battery-monitor    - Clean up temporary files"
	@echo "  help                     - Show this help message"
	@echo ""
	@echo "Quick start:"
	@echo "  make install-battery-monitor  # Install everything"
	@echo "  make setup-battery-panel      # Configure XFCE4 panel"
	@echo "  make test-battery-status      # Test functionality"
