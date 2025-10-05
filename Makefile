# Argo Robot Services Makefile
# Manages systemd services for ROS2 Argo robot

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
# Recording is now handled via ROS2 service, not systemd
BAGFILES_DIR = $(HOME)/bagfiles
# Resolve repository directory and installing user
REPO_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)
ARGO_DIR = $(REPO_DIR)

.PHONY: help install-argo-cli install-deps install-foxglove-bridge check-deps aliases-activate aliases-force aliases-install install-hardware install-all install-python-deps install-power-control start-power-control stop-power-control status-power-control uninstall-power-control submodule-init submodule-update submodule-status install-cpu-tuning

help:
	@echo "Argo Robot Services Management"
	@echo "=============================="
	@echo ""
	@echo "Hardware & Dependencies:"
	@echo "  install-deps         - Install all ROS2 dependencies (foxglove-bridge, etc.)"
	@echo "  install-foxglove-bridge - Install foxglove-bridge package only"
	@echo "  install-python-deps  - Install Python runtime dependencies (smbus2, pyserial, etc.)"
	@echo "  check-deps           - Check status of all dependencies"
	@echo "  install-hardware     - Install PWM capture module and hardware configuration"
	@echo "  install-all          - Install hardware and dependencies (complete setup)"
	@echo ""
	@echo "Service Management (in launch/ directory):"
	@echo "  make -C launch install     - Install all services (launch, storage)"
	@echo "  make -C launch start       - Start argo-launch service with 30s monitoring"
	@echo "  make -C launch stop        - Stop all argo services"
	@echo "  make -C launch restart     - Restart argo-launch service"
	@echo "  make -C launch clean       - Clean old bag files (>7 days)"
	@echo "  make -C launch help        - Show detailed service management help"
	@echo ""
	@echo "CPU Governor Tuning:"
	@echo "  install-cpu-tuning   - Install and enable CPU governor tuning service"
	@echo ""
	@echo "Power Control System (in power_control/ directory):"
	@echo "  make -C power_control install  - Install power control system"
	@echo "  make -C power_control start    - Start power control service"
	@echo "  make -C power_control stop     - Stop power control service"
	@echo "  make -C power_control status   - Show power control status"
	@echo "  make -C power_control help     - Show power control help"
	@echo ""
	@echo "Power Control Convenience Targets:"
	@echo "  install-power-control  - Install power control system"
	@echo "  start-power-control    - Start power control service"
	@echo "  stop-power-control     - Stop power control service"
	@echo "  status-power-control   - Show power control status"
	@echo "  uninstall-power-control - Uninstall power control system"
	@echo ""
	@echo "Utilities:"
	@echo "  aliases-install - Install/update aliases and activate them immediately"
	@echo "  aliases-force - Force reinstall/update aliases (overwrites existing)"
	@echo "  aliases-activate - Print command to activate aliases in current shell"
	@echo "  install-argo-cli - Install Argo CLI (aliases, functions, dotfiles) to ~/.bashrc"
	@echo ""
	@echo "Git Submodule Management:"
	@echo "  submodule-init    - Initialize and checkout sailboat-playground submodule"
	@echo "  submodule-update  - Update sailboat-playground submodule to latest version"
	@echo "  submodule-status  - Show submodule status and current commit"
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
# Service management targets are now in launch/Makefile
# Use 'make -C launch <target>' to run service targets directly

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

# Power control installation is now in power_control/Makefile
# Use 'make -C power_control install' to install power control

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
	@echo "  - pynmea2: NMEA sentence parsing for GPS NavSatFix messages"
	@echo "  - numpy: Numerical computations"
	@echo "  - PyYAML: Configuration file parsing"
	@echo "  - matplotlib: Optional package (ADC testing plots)"

install-hardware:
	@echo "Installing PWM capture module and hardware configuration..."
	@$(MAKE) -C nodes/pwm_capture_module all
	@echo "✅ Hardware installation complete!"
	@echo "⚠️  Reboot required to apply hardware configuration changes."

install-all: install-python-deps install-hardware install-cpu-tuning
	@echo "✅ Complete Argo hardware installation finished!"
	@echo "Next steps:"
	@echo "1. Reboot to apply hardware configuration"
	@echo "2. Run 'make -C launch install' to install services"
	@echo "3. Run 'make -C launch enable' to enable automatic startup"
	@echo "4. Run 'make -C launch start' to launch the system"

# ==================== CPU GOVERNOR TUNING ====================

install-cpu-tuning:
	@echo "Installing CPU governor tuning service..."
	@$(MAKE) -C power_control install-cpu-tuning

# ==================== POWER CONTROL CONVENIENCE TARGETS ====================

install-power-control:
	@echo "Installing power control system..."
	@$(MAKE) -C power_control install

start-power-control:
	@echo "Starting power control service..."
	@$(MAKE) -C power_control start

stop-power-control:
	@echo "Stopping power control service..."
	@$(MAKE) -C power_control stop

status-power-control:
	@echo "Checking power control status..."
	@$(MAKE) -C power_control status

uninstall-power-control:
	@echo "Uninstalling power control system..."
	@$(MAKE) -C power_control uninstall

# ==================== BATTERY MONITORING ====================
# Battery monitoring is now integrated into battery_water.py node
# CSV files are automatically created in /var/log.hdd/persistent/
# No separate service installation needed

# ==================== GIT SUBMODULE MANAGEMENT ====================

submodule-init:
	@echo "Initializing sailboat-playground submodule..."
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		echo "   This repository doesn't have submodules configured."; \
		exit 1; \
	fi
	@echo "Checking submodule configuration..."
	@if ! grep -q "simulator/sailboat-playground" .gitmodules; then \
		echo "❌ 'simulator/sailboat-playground' submodule not found in .gitmodules"; \
		echo "   Available submodules:"; \
		grep "path = " .gitmodules | sed 's/.*path = /     - /'; \
		exit 1; \
	fi
	@echo "Initializing and checking out submodule..."
	git submodule update --init --recursive simulator/sailboat-playground
	@echo "✅ sailboat-playground submodule initialized successfully!"
	@echo ""
	@echo "Submodule location: simulator/sailboat-playground/"
	@echo "Customizations location: simulator/customizations/sailboat-playground/"
	@echo "Source repository: https://github.com/SensorsINI/sailboat-playground.git"
	@echo ""
	@echo "Next steps:"
	@echo "  1. Review the submodule code in simulator/sailboat-playground/"
	@echo "  2. Use 'make submodule-update' to get latest changes"
	@echo "  3. Use 'make submodule-status' to check current version"

submodule-update:
	@echo "Updating sailboat-playground submodule..."
	@if [ ! -d simulator/sailboat-playground ]; then \
		echo "❌ Submodule not initialized!"; \
		echo "   Run 'make submodule-init' first."; \
		exit 1; \
	fi
	@echo "Fetching latest changes from remote..."
	git submodule update --remote --merge simulator/sailboat-playground
	@echo "✅ sailboat-playground submodule updated successfully!"
	@echo ""
	@echo "Current submodule status:"
	@git submodule status simulator/sailboat-playground
	@echo ""
	@echo "Note: The submodule has been updated to the latest version."
	@echo "      Commit this change to lock the submodule to this version:"
	@echo "      git add simulator/sailboat-playground"
	@echo "      git commit -m 'Update sailboat-playground submodule'"

submodule-status:
	@echo "Sailboat-Playground Submodule Status"
	@echo "===================================="
	@echo ""
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		echo "   This repository doesn't have submodules configured."; \
		exit 1; \
	fi
	@echo "🔍 Submodule Configuration:"
	@if grep -q "simulator/sailboat-playground" .gitmodules; then \
		echo "✅ simulator/sailboat-playground submodule configured"; \
		url=$$(grep -A1 "simulator/sailboat-playground" .gitmodules | grep "url" | cut -d'=' -f2 | tr -d ' \t'); \
		echo "   URL: $$url"; \
	else \
		echo "❌ simulator/sailboat-playground submodule not found in .gitmodules"; \
		echo "   Available submodules:"; \
		grep "path = " .gitmodules | sed 's/.*path = /     - /'; \
		exit 1; \
	fi
	@echo ""
	@echo "🔍 Submodule Status:"
	@if [ -d simulator/sailboat-playground ]; then \
		echo "✅ Submodule directory exists: simulator/sailboat-playground/"; \
		if [ -f simulator/sailboat-playground/.git ] || [ -d simulator/sailboat-playground/.git ]; then \
			echo "✅ Submodule is initialized"; \
			echo ""; \
			echo "Current commit:"; \
			cd simulator/sailboat-playground && git log --oneline -1 && cd ../..; \
			echo ""; \
			echo "Branch information:"; \
			cd simulator/sailboat-playground && git branch -v && cd ../..; \
			echo ""; \
			echo "Remote status:"; \
			cd simulator/sailboat-playground && git status -sb && cd ../..; \
		else \
			echo "❌ Submodule directory exists but not initialized"; \
			echo "   Run 'make submodule-init' to initialize"; \
		fi; \
	else \
		echo "❌ Submodule directory not found: simulator/sailboat-playground/"; \
		echo "   Run 'make submodule-init' to initialize"; \
	fi
	@echo ""
	@echo "🔍 Local Customization Files:"
	@if [ -d simulator/customizations/sailboat-playground ]; then \
		echo "✅ Customizations directory exists: simulator/customizations/sailboat-playground/"; \
		echo "   This contains Argo-specific configuration files"; \
		echo "   (separate from the submodule source code)"; \
		ls -1 simulator/customizations/sailboat-playground/ | sed 's/^/     - /'; \
	else \
		echo "❌ Customizations directory not found: simulator/customizations/sailboat-playground/"; \
		echo "   This may affect simulator functionality"; \
	fi
	@echo ""
	@echo "💡 Usage:"
	@echo "  make submodule-init    - Initialize submodule (first time setup)"
	@echo "  make submodule-update  - Update to latest version"
	@echo "  make submodule-status  - Show this status information"

