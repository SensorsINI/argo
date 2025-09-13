# Argo Robot Services Makefile
# Manages systemd services for ROS2 Argo robot

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
RECORD_SERVICE = argo-record.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

.PHONY: help install-services uninstall enable-services disable-services start stop restart clean install-argo-cli install_argo_power_control install-deps install-foxglove-bridge check-deps aliases-activate aliases-force aliases-install install-hardware install-all install-python-deps

help:
	@echo "Argo Robot Services Management"
	@echo "=============================="
	@echo ""
	@echo "Installation:"
	@echo "  install-deps         - Install all ROS2 dependencies (foxglove-bridge, etc.)"
	@echo "  install-foxglove-bridge - Install foxglove-bridge package only"
	@echo "  install-python-deps  - Install Python runtime dependencies (smbus2, pyserial, etc.)"
	@echo "  check-deps           - Check status of all dependencies"
	@echo "  install-hardware     - Install PWM capture module and hardware configuration"
	@echo "  install-services     - Install service files to systemd"
	@echo "  install-all          - Install hardware and services (complete setup)"
	@echo "  uninstall            - Remove service files from systemd"
	@echo "  enable-services      - Enable services for automatic startup"
	@echo "  disable-services     - Disable automatic startup"
	@echo ""
	@echo "Service Control:"
	@echo "  start       - Start argo-launch service with 30s monitoring"
	@echo "  stop        - Stop all argo services"
	@echo "  restart     - Restart argo-launch service"
	@echo ""
	@echo "Recording Control:"
	@echo "  record      - Start ROS2 bag recording"
	@echo "  stop-record - Stop ROS2 bag recording"
	@echo ""
	@echo "Utilities:"
	@echo "  clean       - Clean old bag files (>7 days)"
	@echo "  aliases-install - Install/update aliases and activate them immediately"
	@echo "  aliases-force - Force reinstall/update aliases (overwrites existing)"
	@echo "  aliases-activate - Print command to activate aliases in current shell"
	@echo "  install-argo-cli - Install Argo CLI (aliases, functions, dotfiles) to ~/.bashrc"
	@echo "  install_argo_power_control - Install GPIO permissions for power control (udev rules, gpio group)"
	@echo ""
	@echo "Quick Commands (after installing CLI):"
	@echo "  al  - Launch argo service (with 30s monitoring)"
	@echo "  aq  - Quit argo service"
	@echo "  ar  - Record data"
	@echo "  ac  - Close recording"
	@echo "  af  - Launch argo with Foxglove visualization"

# ==================== DEPENDENCY INSTALLATION ====================

install-deps: install-foxglove-bridge
	@echo "✅ All ROS2 dependencies installed successfully!"
	@echo ""
	@echo "Installed packages:"
	@echo "  - ros-$(ROS_DISTRO)-foxglove-bridge (C++ WebSocket bridge for Foxglove Studio)"
	@echo ""
	@echo "Usage:"
	@echo "  ros2 run foxglove_bridge foxglove_bridge                    # Start foxglove bridge"
	@echo "  ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765  # Custom port"
	@echo ""
	@echo "Connect from Foxglove Studio: ws://$(shell hostname -I | awk '{print $$1}'):8765"

install-foxglove-bridge:
	@echo "Installing Foxglove Bridge for ROS2..."
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "❌ ROS_DISTRO environment variable not set!"; \
		echo "   Make sure ROS2 is properly sourced: source /opt/ros/*/setup.bash"; \
		exit 1; \
	fi
	@echo "Installing ros-$(ROS_DISTRO)-foxglove-bridge..."
	sudo apt update
	sudo apt install -y ros-$(ROS_DISTRO)-foxglove-bridge
	@echo "✅ Foxglove Bridge installed successfully!"
	@echo ""
	@echo "🔧 Quick Start:"
	@echo "  1. Run your Python ROS2 node: python3 nodes/battery_water.py"
	@echo "  2. In another terminal: ros2 run foxglove_bridge foxglove_bridge"
	@echo "  3. Connect Foxglove Studio to: ws://$(shell hostname -I | awk '{print $$1}'):8765"

check-deps:
	@echo "Checking ROS2 Dependencies Status"
	@echo "================================="
	@echo ""
	@echo "🔍 Environment:"
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "❌ ROS_DISTRO: Not set (ROS2 not sourced?)"; \
	else \
		echo "✅ ROS_DISTRO: $(ROS_DISTRO)"; \
	fi
	@echo ""
	@echo "🔍 Package Status:"
	@if dpkg -l | grep -q "ros-$(ROS_DISTRO)-foxglove-bridge"; then \
		version=$$(dpkg -l | grep "ros-$(ROS_DISTRO)-foxglove-bridge" | awk '{print $$3}'); \
		echo "✅ foxglove-bridge: Installed ($$version)"; \
	else \
		echo "❌ foxglove-bridge: Not installed"; \
		echo "   Install with: make install-foxglove-bridge"; \
	fi
	@echo ""
	@echo "🔍 Network Info:"
	@echo "   Your IP: $(shell hostname -I | awk '{print $$1}')"
	@echo "   Foxglove connection: ws://$(shell hostname -I | awk '{print $$1}'):8765"
	@echo ""
	@echo "🔍 Quick Test:"
	@echo "   Test foxglove-bridge: ros2 run foxglove_bridge foxglove_bridge --help"

# ==================== SERVICE MANAGEMENT ====================
# These targets delegate to the launch Makefile

install-services:
	@$(MAKE) -C launch install

uninstall:
	@$(MAKE) -C launch uninstall

enable-services:
	@$(MAKE) -C launch enable

disable-services:
	@$(MAKE) -C launch disable

start:
	@$(MAKE) -C launch start

stop:
	@$(MAKE) -C launch stop

restart:
	@$(MAKE) -C launch restart

record:
	@$(MAKE) -C launch record

stop-record:
	@$(MAKE) -C launch stop-record

clean:
	@$(MAKE) -C launch clean

install-argo-cli:
	@echo "Installing Argo CLI (aliases, functions, and dotfiles)..."
	@if ! grep -q "source.*dotfiles.*\.bashrc" ~/.bashrc 2>/dev/null; then \
		echo "" >> ~/.bashrc; \
		echo "# Source Argo dotfiles" >> ~/.bashrc; \
		echo "source ~/argo/dotfiles/.bashrc" >> ~/.bashrc; \
		echo "✅ Added dotfiles sourcing to ~/.bashrc"; \
	else \
		echo "✅ Dotfiles already sourced in ~/.bashrc"; \
	fi
	@if [ -f dotfiles/.tmux.conf ]; then \
		cp dotfiles/.tmux.conf ~/.tmux.conf; \
		echo "✅ Installed .tmux.conf"; \
	else \
		echo "❌ dotfiles/.tmux.conf not found"; \
	fi
	@echo ""
	@echo "🔄 To activate CLI in this terminal, run:"
	@echo "   source ~/.bashrc"
	@echo ""
	@echo "💡 In NEW terminals, CLI will be auto-loaded from ~/.bashrc"
	@echo ""
	@echo "Available commands:"
	@echo "  al   - Launch argo service (with 30s monitoring)"
	@echo "  aq   - Quit argo service"
	@echo "  ar   - Record data"
	@echo "  ac   - Close recording"
	@echo "  as   - Show Argo status (via argo_status function)"
	@echo "  ars  - Restart argo service"
	@echo "  argo_status - Show detailed Argo status"
	@echo "  argo_help - Show detailed help"
	@echo ""
	@echo "✅ Argo CLI installation complete!"

install_argo_power_control:
	@$(MAKE) -C launch install_argo_power_control

install-python-deps:
	@echo "Installing Python runtime dependencies..."
	@echo "Installing packages from requirements-runtime.txt..."
	pip3 install -r requirements-runtime.txt
	@echo "✅ Python dependencies installed successfully!"
	@echo ""
	@echo "Installed packages:"
	@echo "  - rclpy: ROS2 Python client library"
	@echo "  - std-msgs, geometry-msgs: ROS2 message types"
	@echo "  - smbus2: I2C communication for sensors"
	@echo "  - pyserial: Serial communication for GPS"
	@echo "  - numpy: Numerical computations"
	@echo "  - PyYAML: Configuration file parsing"
	@echo "  - matplotlib: Optional package (ADC testing plots)"

install-hardware:
	@echo "Installing PWM capture module and hardware configuration..."
	@$(MAKE) -C nodes/pwm_capture_module all
	@echo "✅ Hardware installation complete!"
	@echo "⚠️  Reboot required to apply hardware configuration changes."

install-all: install-python-deps install-hardware install-services
	@echo "✅ Complete Argo installation finished!"
	@echo "Next steps:"
	@echo "1. Reboot to apply hardware configuration"
	@echo "2. Run 'make enable-services' to enable automatic startup"
	@echo "3. Run 'make start' to launch the system"

