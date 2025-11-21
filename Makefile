# ============================================================================
# Argo Autonomous Sailboat - Top Level Makefile
# ============================================================================
#
# This Makefile provides centralized management for the Argo ROS2 autonomous
# sailboat system, including hardware setup, dependencies, services, and
# system configuration.
#
# QUICK START:
#   make help                    - Show all available commands
#   make install-all            - Complete hardware and dependency setup
#   make install-argo-cli       - Install command-line aliases (al, aq, ar, ac, etc.)
#   make -C launch install      - Install and enable Argo launch services
#   make -C power_control install - Install power control system
#
# MAIN COMPONENTS:
#   • Hardware & Dependencies   - ROS2 packages, Python deps, PWM modules
#   • Service Management        - Lifecycle, power control, battery monitoring
#   • System Configuration      - CPU tuning, WiFi, logging, MOTD
#   • Simulation                - Local/remote simulation modes
#   • Development Tools         - Git submodules, testing, debugging
#
# SERVICE ORGANIZATION:
#   This top-level Makefile delegates to component Makefiles:
#   • launch/Makefile           - ROS2 node lifecycle and launch services
#   • power_control/Makefile    - Power button, LED, shutdown control
#   • scripts/Makefile          - Battery monitoring and status display
#   • system-monitoring/Makefile - Optional debugging and monitoring services
#
# IMPORTANT NOTES:
#   • Most service operations are in launch/Makefile (use 'make -C launch <target>')
#   • Convenience targets here forward to component Makefiles
#   • Run 'make help' for complete list of available commands
#   • Hardware changes typically require reboot to take effect
#
# For detailed documentation, see:
#   • README.md                 - Project overview and getting started
#   • launch/README.md          - Service management and lifecycle
#   • power_control/README.md   - Power control system documentation
# ============================================================================

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo_launch.service
# Recording is now handled via ROS2 service, not systemd
BAGFILES_DIR = $(HOME)/bagfiles
# Resolve repository directory and installing user
REPO_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)
ARGO_DIR = $(REPO_DIR)

# ==================== PLATFORM DETECTION ====================
# Detect if we're on the Argo robot boat (Orange Pi) or host computer
IS_ORANGEPI := $(shell if [ -f /proc/device-tree/compatible ] && grep -q "orangepi" /proc/device-tree/compatible 2>/dev/null; then echo 1; else echo 0; fi)
REQUIREMENTS_FILE := $(if $(filter 1,$(IS_ORANGEPI)),requirements.txt,requirements-host.txt)
PLATFORM_NAME := $(if $(filter 1,$(IS_ORANGEPI)),Orange Pi Robot,Host Computer)

.PHONY: help install-argo-cli install-deps install-ros2-minimal install-foxglove-bridge install-rosbag2-mcap check-deps aliases-activate aliases-force aliases-install install-hardware install-all install-python-deps install-power-control start-power-control stop-power-control status-power-control uninstall-power-control submodule-init submodule-update submodule-status install-cpu-tuning fix-orangepi-ramlog install-motd uninstall-motd test-motd setup-wifi-networks install-battery-monitor setup-battery-panel test-battery-status install-network-improvements test-wifi-reconnection wifi-test-status wifi-test-results wifi-reconnect-status wifi-reconnect-logs wifi-reconnect-stop wifi-reconnect-start install-system-monitoring uninstall-system-monitoring simulate-local simulate-remote setup-vm clean-vm update-vm

VM_REQUIREMENTS_FILE := requirements-host.txt
VM_INSTALL_SCOPE := $(shell if [ $$(id -u) -eq 0 ]; then printf 'system'; else printf 'user'; fi)
VM_HAS_UV := $(shell command -v uv >/dev/null 2>&1 && printf '1' || printf '0')
VM_PIP_INSTALL_CMD := $(shell \
	if command -v uv >/dev/null 2>&1 && [ $$(id -u) -eq 0 ]; then \
		printf 'uv pip install --upgrade'; \
	elif [ $$(id -u) -eq 0 ]; then \
		printf 'pip3 install --upgrade'; \
	elif command -v uv >/dev/null 2>&1; then \
		printf 'pip3 install --user --upgrade'; \
	else \
		printf 'pip3 install --user --upgrade'; \
	fi)
VM_PIP_UNINSTALL_CMD := $(shell \
	if command -v uv >/dev/null 2>&1 && [ $$(id -u) -eq 0 ]; then \
		printf 'uv pip uninstall -y'; \
	else \
		printf 'pip3 uninstall -y'; \
	fi)

help:
	@echo "Argo Robot Services Management"
	@echo "=============================="
	@echo ""
	@echo "Hardware & Dependencies:"
	@echo "  install-deps         - Install all ROS2 dependencies (minimal ROS2, foxglove-bridge, rosbag2-mcap)"
	@echo "  install-ros2-minimal - Install minimal ROS2 Humble packages (rclpy, messages, services, TF2, launch, rosbag2)"
	@echo "  install-foxglove-bridge - Install foxglove-bridge package only"
	@echo "  install-rosbag2-mcap - Install MCAP storage plugin for rosbag2 only"
	@echo "  install-python-deps  - Auto-detect platform and install correct Python dependencies"
	@echo "                         (requirements.txt on robot, requirements-host.txt on host)"
	@echo "  install-simulation-deps - Install Python dependencies for the simulator"
	@echo "  check-deps           - Check status of all dependencies"
	@echo "  install-hardware     - Install PWM capture module and hardware configuration"
	@echo "  install-all          - Install hardware and dependencies (complete setup)"
	@echo ""
	@echo "Service Management (in launch/ directory):"
	@echo "  make -C launch install       - Install Argo launch systemd services"
	@echo "  make -C launch uninstall     - Uninstall all Argo launch services"
	@echo "  make -C launch start       - Start argo_launch_standard service"
	@echo "  make -C launch stop        - Stop argo_launch_standard service"
	@echo "  make -C launch restart     - Restart argo_launch_standard service"
	@echo "  make -C launch clean       - Clean old bag files (>7 days)"
	@echo "  make -C launch help        - Show detailed service management help"
	@echo ""
	@echo "Simulation Management:"
	@echo "  simulate-local       - Start Argo in local simulation mode"
	@echo "  simulate-remote      - Start Argo in remote simulation mode"
	@echo ""
	@echo "CPU Governor Tuning:"
	@echo "  install-cpu-tuning   - Install and enable CPU governor tuning service"
	@echo ""
	@echo "System Fixes:"
	@echo "  fix-orangepi-ramlog  - Fix orangepi-ramlog to preserve persistent logs"
	@echo "  setup-wifi-networks  - Configure WiFi networks with proper priority order"
	@echo "  install-network-improvements - Install WiFi reconnection system and NetworkManager optimizations"
	@echo ""
	@echo "Network Testing:"
	@echo "  test-wifi-reconnection - Start WiFi reconnection test (3min, background)"
	@echo "  wifi-test-status      - Check WiFi test status and progress"
	@echo "  wifi-test-results     - Show WiFi test results and logs"
	@echo ""
	@echo "WiFi Reconnection Service:"
	@echo "  wifi-reconnect-status - Check systemd timer status"
	@echo "  wifi-reconnect-logs   - Show service logs (last hour)"
	@echo "  wifi-reconnect-stop   - Stop WiFi reconnection service"
	@echo "  wifi-reconnect-start  - Start WiFi reconnection service"
	@echo ""
	@echo "MOTD Customization:"
	@echo "  install-motd    - Install Argo shutdown status MOTD script"
	@echo "  uninstall-motd  - Uninstall Argo MOTD script"
	@echo "  test-motd       - Test MOTD script output (safe, no installation)"
	@echo ""
	@echo "Power Control System:"
	@echo "  make -C power_control install - Install the power control system"
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
	@echo "Battery Status Monitor (in scripts/ directory):"
	@echo "  make -C scripts install-battery-monitor - Install battery status monitor"
	@echo "  make -C scripts setup-battery-panel     - Configure XFCE4 panel"
	@echo "  make -C scripts test-battery-status     - Test battery status scripts"
	@echo "  make -C scripts help                    - Show battery monitor help"
	@echo ""
	@echo "Battery Monitor Convenience Targets:"
	@echo "  install-battery-monitor - Install battery status monitor"
	@echo "  setup-battery-panel     - Configure XFCE4 panel with battery status"
	@echo "  test-battery-status     - Test battery status functionality"
	@echo ""
	@echo "System Monitoring Services (Optional - for debugging):"
	@echo "  install-system-monitoring - Install all system monitoring services"
	@echo "  uninstall-system-monitoring - Uninstall all system monitoring services"
	@echo "  make -C system-monitoring help - Show detailed system monitoring help"
	@echo ""
	@echo "Utilities:"
	@echo "  aliases-install - Install/update aliases and activate them immediately"
	@echo "  aliases-force - Force reinstall/update aliases (overwrites existing)"
	@echo "  aliases-activate - Print command to activate aliases in current shell"
	@echo "  install-argo-cli - Install Argo CLI (aliases, functions, dotfiles) to ~/.bashrc"
	@echo ""
	@echo "Git Submodule Management:"
	@echo "  submodule-init    - Initialize and checkout all submodules (sailboat-playground, bno08x_driver)"
	@echo "  submodule-update  - Update all submodules to latest version"
	@echo "  submodule-status  - Show submodule status and current commits"
	@echo "  submodule-sync    - Show commands to sync submodule references (clean up dirty submodule state)"
	@echo ""
	@echo "Quick Commands (after installing CLI):"
	@echo "  al  - Launch argo service (with 30s monitoring)"
	@echo "  aq  - Quit argo service"
	@echo "  ar  - Record data"
	@echo "  ac  - Close recording"
	@echo "  af  - Launch argo with Foxglove visualization"
	@echo "Development Environment (Host Machine Only):"
	@echo "  setup-venv     - Create Python venv with uv (automatic activation)"
	@echo "  update-venv    - Update venv dependencies"
	@echo "  clean-venv     - Remove venv"
	@echo "  setup-vm       - Install host dependencies to VM user/system site-packages"
	@echo "  update-vm      - Update host dependencies on VM"
	@echo "  clean-vm       - Uninstall host dependencies from VM"
# ==================== SIMULATION MANAGEMENT ====================

simulate-local:
	@echo "Starting Argo in LOCAL simulation mode via launch script..."
	@./scripts/launch_simulation.sh

simulate-remote:
	@echo "Starting Argo in REMOTE simulation mode..."
	@python3 launch/argo_lifecycle_manager.py simulate_remote


# ==================== DEVELOPMENT ENVIRONMENT (HOST ONLY) ====================

setup-venv:
	@echo "Setting up Python virtual environment for shore-side development..."
	@if command -v uv >/dev/null 2>&1; then \
		echo "Using uv (fast Python package installer)..."; \
		uv venv .venv; \
		echo "Installing dependencies with uv..."; \
		uv pip install -r requirements-host.txt; \
	else \
		echo "uv not found, using standard venv..."; \
		echo "💡 Install uv for faster dependency management: curl -LsSf https://astral.sh/uv/install.sh | sh"; \
		python3 -m venv .venv; \
		.venv/bin/pip install --upgrade pip; \
		.venv/bin/pip install -r requirements-host.txt; \
	fi
	@echo "✅ Virtual environment created at .venv"
	@echo ""
	@echo "The venv will activate automatically when you open a terminal."
	@echo "Or manually activate: source .venv/bin/activate"

clean-venv:
	@echo "Removing Python virtual environment..."
	@rm -rf .venv
	@echo "✅ Virtual environment removed"

update-venv:
	@echo "Updating Python dependencies in venv..."
	@if [ ! -d .venv ]; then \
		echo "❌ No venv found. Run 'make setup-venv' first."; \
		exit 1; \
	fi
	@if command -v uv >/dev/null 2>&1; then \
		uv pip install --upgrade -r requirements.txt; \
	else \
		.venv/bin/pip install --upgrade -r requirements.txt; \
	fi
	@echo "✅ Dependencies updated"

setup-vm:
	@echo "Installing Python host dependencies on dedicated VM ($(VM_INSTALL_SCOPE) site-packages)..."
	@echo "Using $(if $(filter 1,$(VM_HAS_UV)),uv,pip3) for package management."
	@$(VM_PIP_INSTALL_CMD) -r $(VM_REQUIREMENTS_FILE)
	@echo "✅ Host dependencies installed for VM environment"

update-vm:
	@echo "Updating Python host dependencies on dedicated VM ($(VM_INSTALL_SCOPE) site-packages)..."
	@echo "Using $(if $(filter 1,$(VM_HAS_UV)),uv,pip3) for package management."
	@$(VM_PIP_INSTALL_CMD) -r $(VM_REQUIREMENTS_FILE)
	@echo "✅ Host dependencies updated for VM environment"

clean-vm:
	@echo "Uninstalling Python host dependencies from dedicated VM..."
	@echo "Using $(if $(filter 1,$(VM_HAS_UV)),uv,pip3) for package management."
	@$(VM_PIP_UNINSTALL_CMD) -r $(VM_REQUIREMENTS_FILE) || true
	@echo "✅ Host dependencies removed from VM environment"


# ==================== DEPENDENCY INSTALLATION ====================

install-deps: install-ros2-minimal install-foxglove-bridge install-rosbag2-mcap
	@echo "✅ All ROS2 dependencies installed successfully!"
	@echo ""
	@echo "Installed packages:"
	@echo "  - Minimal ROS2 Humble core packages (rclpy, messages, services, TF2, launch, rosbag2)"
	@echo "  - ros-$(ROS_DISTRO)-foxglove-bridge (C++ WebSocket bridge for Foxglove Studio)"
	@echo "  - ros-$(ROS_DISTRO)-rosbag2-storage-mcap (MCAP storage format for rosbag2)"
	@echo ""
	@echo "Usage:"
	@echo "  ros2 run foxglove_bridge foxglove_bridge                    # Start foxglove bridge"
	@echo "  ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765  # Custom port"
	@echo ""
	@echo "Connect from Foxglove Studio: ws://$(shell hostname -I | awk '{print $$1}'):8765"
	@echo ""
	@echo "MCAP Recording:"
	@echo "  ros2 bag record -s mcap -a -o my_bag  # Record with MCAP format"

install-ros2-minimal:
	@echo "Installing minimal ROS2 Humble packages for Argo..."
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "❌ ROS_DISTRO environment variable not set!"; \
		echo "   Make sure ROS2 is properly sourced: source /opt/ros/*/setup.bash"; \
		exit 1; \
	fi
	@echo "Installing core ROS2 Python runtime..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-rclpy \
		ros-$(ROS_DISTRO)-rosidl-runtime-py \
		ros-$(ROS_DISTRO)-rosidl-generator-py
	@echo "Installing message and service types..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-std-msgs \
		ros-$(ROS_DISTRO)-std-srvs \
		ros-$(ROS_DISTRO)-geometry-msgs \
		ros-$(ROS_DISTRO)-sensor-msgs \
		ros-$(ROS_DISTRO)-visualization-msgs \
		ros-$(ROS_DISTRO)-rosgraph-msgs \
		ros-$(ROS_DISTRO)-rcl-interfaces \
		ros-$(ROS_DISTRO)-diagnostic-msgs \
		ros-$(ROS_DISTRO)-builtin-interfaces
	@echo "Installing TF2 (transform library)..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-tf2 \
		ros-$(ROS_DISTRO)-tf2-ros \
		ros-$(ROS_DISTRO)-tf2-ros-py \
		ros-$(ROS_DISTRO)-tf2-geometry-msgs \
		ros-$(ROS_DISTRO)-tf2-msgs \
		ros-$(ROS_DISTRO)-tf2-py
	@echo "Installing ROS2 CLI tools..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-ros2cli \
		ros-$(ROS_DISTRO)-ros2run \
		ros-$(ROS_DISTRO)-ros2launch \
		ros-$(ROS_DISTRO)-ros2node \
		ros-$(ROS_DISTRO)-ros2param \
		ros-$(ROS_DISTRO)-ros2pkg \
		ros-$(ROS_DISTRO)-ros2service \
		ros-$(ROS_DISTRO)-ros2topic
	@echo "Installing Rosbag2 (for recording)..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-rosbag2 \
		ros-$(ROS_DISTRO)-rosbag2-cpp \
		ros-$(ROS_DISTRO)-rosbag2-storage
	@echo "Installing GPS support..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-nmea-msgs \
		ros-$(ROS_DISTRO)-nmea-navsat-driver
	@echo "Installing launch system dependencies..."
	sudo apt install -y \
		ros-$(ROS_DISTRO)-launch \
		ros-$(ROS_DISTRO)-launch-ros
	@echo "✅ Minimal ROS2 Humble installation complete!"
	@echo ""
	@echo "To verify installation, run:"
	@echo "  source /opt/ros/$(ROS_DISTRO)/setup.bash"
	@echo "  python3 -c 'from std_srvs.srv import Trigger; print(\"✅ std_srvs OK\")'"

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
	@echo "  1. Run your Python ROS2 node: python3 nodes/argo_battery_water.py"
	@echo "  2. In another terminal: ros2 run foxglove_bridge foxglove_bridge"
	@echo "  3. Connect Foxglove Studio to: ws://$(shell hostname -I | awk '{print $$1}'):8765"

install-rosbag2-mcap:
	@echo "Installing MCAP storage plugin for rosbag2..."
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "❌ ROS_DISTRO environment variable not set!"; \
		echo "   Make sure ROS2 is properly sourced: source /opt/ros/*/setup.bash"; \
		exit 1; \
	fi
	@echo "Installing ros-$(ROS_DISTRO)-rosbag2-storage-mcap..."
	sudo apt update
	sudo apt install -y ros-$(ROS_DISTRO)-rosbag2-storage-mcap
	@echo "✅ MCAP storage plugin installed successfully!"
	@echo ""
	@echo "🔧 MCAP Recording:"
	@echo "  The Argo record node now uses MCAP format by default (configurable in nodes/record.yaml)"
	@echo "  MCAP provides better performance, crash recovery, and cross-platform compatibility"
	@echo ""
	@echo "  Manual usage:"
	@echo "    ros2 bag record -s mcap -a -o my_bag  # Record with MCAP format"
	@echo "    ros2 bag play -s mcap path/to/bag.mcap  # Playback MCAP bag"

check-deps:
	@echo "Checking ROS2 Dependencies Status"
	@echo "================================="
	@echo ""
	@echo "🔍 Environment:"
	@if [ -z "$(ROS_DISTRO)" ]; then \
		echo "❌ ROS_DISTRO: Not set (ROS2 not sourced?)"; \
		echo "   Source ROS2: source /opt/ros/*/setup.bash"; \
	else \
		echo "✅ ROS_DISTRO: $(ROS_DISTRO)"; \
	fi
	@echo ""
	@echo "🔍 Core ROS2 Packages:"
	@if [ -n "$(ROS_DISTRO)" ]; then \
		if dpkg -l | grep -q "ros-$(ROS_DISTRO)-rclpy"; then \
			echo "✅ rclpy: Installed"; \
		else \
			echo "❌ rclpy: Not installed (run: make install-ros2-minimal)"; \
		fi; \
		if dpkg -l | grep -q "ros-$(ROS_DISTRO)-std-srvs"; then \
			echo "✅ std-srvs: Installed"; \
		else \
			echo "❌ std-srvs: Not installed (run: make install-ros2-minimal)"; \
		fi; \
		if dpkg -l | grep -q "ros-$(ROS_DISTRO)-ros2launch"; then \
			echo "✅ ros2launch: Installed"; \
		else \
			echo "❌ ros2launch: Not installed (run: make install-ros2-minimal)"; \
		fi; \
		if dpkg -l | grep -q "ros-$(ROS_DISTRO)-ros2run"; then \
			echo "✅ ros2run: Installed"; \
		else \
			echo "❌ ros2run: Not installed (run: make install-ros2-minimal)"; \
		fi; \
	fi
	@echo ""
	@echo "🔍 Optional Packages:"
	@if dpkg -l | grep -q "ros-$(ROS_DISTRO)-foxglove-bridge"; then \
		version=$$(dpkg -l | grep "ros-$(ROS_DISTRO)-foxglove-bridge" | awk '{print $$3}'); \
		echo "✅ foxglove-bridge: Installed ($$version)"; \
	else \
		echo "❌ foxglove-bridge: Not installed"; \
		echo "   Install with: make install-foxglove-bridge"; \
	fi
	@if dpkg -l | grep -q "ros-$(ROS_DISTRO)-rosbag2-storage-mcap"; then \
		version=$$(dpkg -l | grep "ros-$(ROS_DISTRO)-rosbag2-storage-mcap" | awk '{print $$3}'); \
		echo "✅ rosbag2-storage-mcap: Installed ($$version)"; \
	else \
		echo "❌ rosbag2-storage-mcap: Not installed"; \
		echo "   Install with: make install-rosbag2-mcap"; \
	fi
	@echo ""
	@echo "🔍 Network Info:"
	@echo "   Your IP: $(shell hostname -I | awk '{print $$1}')"
	@echo "   Foxglove connection: ws://$(shell hostname -I | awk '{print $$1}'):8765"
	@echo ""
	@echo "🔍 Quick Test:"
	@echo "   Test foxglove-bridge: ros2 run foxglove_bridge foxglove_bridge --help"
	@echo "   Test MCAP storage: ros2 bag record -s mcap --help"

install-simulation-deps: install-foxglove-bridge
	@echo "Installing system and Python dependencies for sailboat-playground simulator..."
	@echo "--- Installing system libraries (requires sudo) ---"
	sudo apt-get update
	sudo apt-get install -y libglu1-mesa-dev
	@echo "--- Installing Python packages ---"
	@if [ -f simulator/sailboat-playground/requirements.txt ]; then \
		pip3 install -r simulator/sailboat-playground/requirements.txt; \
	else \
		echo "❌ Error: simulator/sailboat-playground/requirements.txt not found!"; \
		echo "   Have you initialized the submodule? Run: make submodule-init"; \
		exit 1; \
	fi
	@echo "✅ Simulation dependencies installed successfully!"

# ==================== SERVICE MANAGEMENT ====================
# Service management targets are now in launch/Makefile
# Use 'make -C launch <target>' to run service targets directly

install-argo-cli:
	@echo "Installing Argo CLI (aliases, functions, and dotfiles)..."
	@# Remove existing Argo dotfiles sourcing to prevent duplicates and handle name changes.
	@sed -i.bak \
		-e '/^# Source Argo dotfiles$$/d' \
		-e '/source.*dotfiles\/\.bashrc/d' \
		-e '/source.*dotfiles\/bashrc/d' \
		~/.bashrc
	@# Add the correct sourcing line for the new dotfiles.
	@echo "" >> ~/.bashrc
	@echo "# Source Argo dotfiles" >> ~/.bashrc
	@echo "source ~/argo/dotfiles/bashrc" >> ~/.bashrc
	@echo "✅ Updated dotfiles sourcing in ~/.bashrc"
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
	@echo "======================================================================="
	@echo "Auto-detecting platform..."
	@echo "Platform: $(PLATFORM_NAME)"
	@echo "Requirements file: $(REQUIREMENTS_FILE)"
	@echo "======================================================================="
	@echo ""
	@echo "Installing Python dependencies from $(REQUIREMENTS_FILE)..."
	pip3 install -r $(REQUIREMENTS_FILE)
	@echo ""
	@echo "✅ Python dependencies installed successfully for $(PLATFORM_NAME)!"
	@echo ""
ifeq ($(IS_ORANGEPI),1)
	@echo "Robot-specific packages installed:"
	@echo "  - Hardware I/O: smbus2, pyserial, pynmea2, gpiod, spidev"
	@echo "  - ROS2: rclpy, std-msgs, geometry-msgs, sensor-msgs, diagnostic-msgs"
	@echo "  - Configuration: PyYAML, ruamel.yaml (comment-preserving)"
	@echo "  - System: numpy, psutil"
	@echo ""
	@echo "Optional packages (may fail gracefully if unavailable):"
	@echo "  - matplotlib, pandas, diagnostic-updater"
else
	@echo "Host computer packages installed:"
	@echo "  - Analysis & Plotting: numpy, pandas, matplotlib"
	@echo "  - Configuration: PyYAML, ruamel.yaml (comment-preserving)"
	@echo "  - Simulation: pyglet==1.5.27"
	@echo "  - System: psutil, argparse, argcomplete"
	@echo ""
	@echo "💡 Note: ROS2 packages (rclpy, etc.) should be installed via apt:"
	@echo "   sudo apt install ros-humble-desktop"
	@echo "   (or ros-humble-ros-base for minimal footprint)"
endif
	@echo ""
	@echo "Both environments include ruamel.yaml for safe YAML comment preservation!"

install-hardware:
	@echo "Installing PWM capture module and hardware configuration..."
	@$(MAKE) -C nodes/pwm_capture_module all
	@echo "✅ Hardware installation complete!"
	@echo "⚠️  Reboot required to apply hardware configuration changes."

install-all: install-python-deps
ifeq ($(IS_ORANGEPI),1)
	@echo "Robot platform detected - installing hardware and system configurations..."
	@$(MAKE) install-hardware install-cpu-tuning fix-orangepi-ramlog install-network-improvements
	@echo "✅ Complete Argo robot installation finished!"
	@echo ""
	@echo "Next steps:"
	@echo "1. Reboot to apply hardware configuration"
	@echo "2. Run 'make -C launch install' to install services"
	@echo "3. Run 'make -C launch enable' to enable automatic startup"
	@echo "4. Run 'make -C launch start' to launch the system"
else
	@echo "✅ Host computer Python dependencies installed!"
	@echo ""
	@echo "Host computer setup complete. Hardware installation skipped (not on robot)."
	@echo ""
	@echo "Next steps for host computer:"
	@echo "1. Ensure ROS2 is installed: sudo apt install ros-humble-desktop"
	@echo "2. Initialize simulator submodule: make submodule-init"
	@echo "3. Install simulation dependencies: make install-simulation-deps"
	@echo "4. Run local simulation: make simulate-local"
endif

# ==================== CPU GOVERNOR TUNING ====================

install-cpu-tuning:
	@echo "Installing CPU governor tuning service..."
	@$(MAKE) -C power_control install-cpu-tuning

# ==================== SYSTEM FIXES ====================

fix-orangepi-ramlog:
	@echo "Fixing orangepi-ramlog service to preserve persistent logs..."
	@if [ -f scripts/fix-orangepi-ramlog.sh ]; then \
		./scripts/fix-orangepi-ramlog.sh; \
	else \
		echo "❌ Error: scripts/fix-orangepi-ramlog.sh not found!"; \
		echo "   Make sure you're running this from the Argo project root directory."; \
		exit 1; \
	fi

setup-wifi-networks:
	@echo "Setting up Argo WiFi networks with proper priority order..."
	@if [ -f scripts/setup_wifi_networks.sh ]; then \
		./scripts/setup_wifi_networks.sh; \
	else \
		echo "❌ Error: scripts/setup_wifi_networks.sh not found!"; \
		echo "   Make sure you're running this from the Argo project root directory."; \
		exit 1; \
	fi

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
# Battery monitoring is now integrated into argo_battery_water.py node
# CSV files are automatically created in /var/log.hdd/persistent/
# No separate service installation needed

# ==================== GIT SUBMODULE MANAGEMENT ====================

SAILBOAT_SUBMODULE_PATH := simulator/sailboat-playground
SAILBOAT_SUBMODULE_NAME := simulator/sailboat-playground
BNO08X_SUBMODULE_PATH := nodes/vendor/bno08x_driver
BNO08X_SUBMODULE_NAME := nodes/vendor/bno08x_driver

# Helper function to initialize a single submodule
define init_submodule
	@echo "Initializing $(1) submodule..."
	@if [ ! -d $(1) ]; then \
		git submodule update --init --recursive $(1); \
	fi
	@submodule_url=$$(git config -f .gitmodules submodule.$(2).url); \
	if [ -z "$$submodule_url" ]; then \
		echo "⚠️  Warning: Unable to determine submodule remote URL from .gitmodules for $(1)"; \
	else \
		cd $(1) && \
		current_url=$$(git remote get-url origin 2>/dev/null || echo ""); \
		if [ -z "$$current_url" ] || [ "$$current_url" != "$$submodule_url" ]; then \
			git remote set-url origin "$$submodule_url" 2>/dev/null || git remote add origin "$$submodule_url"; \
			echo "   ➜ Updated origin to $$submodule_url"; \
		fi; \
		git fetch origin --prune 2>/dev/null || true; \
		if git show-ref --verify --quiet refs/remotes/origin/master 2>/dev/null; then \
			target_branch=master; \
		elif git show-ref --verify --quiet refs/remotes/origin/main 2>/dev/null; then \
			target_branch=main; \
		else \
			echo "   ⚠️  Warning: origin/master and origin/main not found for $(1)"; \
			target_branch=""; \
		fi; \
		if [ -n "$$target_branch" ]; then \
			current_commit=$$(git rev-parse HEAD 2>/dev/null || echo ""); \
			if [ -n "$$current_commit" ]; then \
				git checkout -B $$target_branch $$current_commit >/dev/null 2>&1 || true; \
			fi; \
			if git rev-parse --verify --quiet origin/$$target_branch >/dev/null 2>&1; then \
				git branch --set-upstream-to=origin/$$target_branch $$target_branch >/dev/null 2>&1 || true; \
			fi; \
			echo "   ➜ Checked out $$target_branch tracking origin/$$target_branch"; \
		fi; \
		cd ..; \
	fi
endef

submodule-init:
	@echo "Initializing all submodules..."
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		echo "   This repository doesn't have submodules configured."; \
		exit 1; \
	fi
	@echo ""
	$(call init_submodule,$(SAILBOAT_SUBMODULE_PATH),$(SAILBOAT_SUBMODULE_NAME))
	@echo ""
	$(call init_submodule,$(BNO08X_SUBMODULE_PATH),$(BNO08X_SUBMODULE_NAME))
	@echo ""
	@echo "✅ All submodules initialized and configured successfully!"
	@echo ""
	@echo "Submodule locations:"
	@echo "  - $(SAILBOAT_SUBMODULE_PATH)/"
	@echo "  - $(BNO08X_SUBMODULE_PATH)/"
	@echo ""
	@echo "Next steps:"
	@echo "  1. Review the submodule code"
	@echo "  2. Use 'make submodule-update' to get latest changes"
	@echo "  3. Use 'make submodule-status' to check current versions"

# Helper function to update a single submodule
define update_submodule
	@echo "Updating $(1) submodule..."
	@if [ ! -d $(1) ]; then \
		echo "⚠️  Submodule not initialized, initializing first..."; \
		git submodule update --init --recursive $(1) || true; \
	fi
	@submodule_url=$$(git config -f .gitmodules submodule.$(2).url); \
	if [ -z "$$submodule_url" ]; then \
		echo "⚠️  Warning: Unable to determine submodule remote URL from .gitmodules for $(1)"; \
	else \
		cd $(1) && \
		current_url=$$(git remote get-url origin 2>/dev/null || echo ""); \
		if [ -z "$$current_url" ] || [ "$$current_url" != "$$submodule_url" ]; then \
			git remote set-url origin "$$submodule_url" 2>/dev/null || git remote add origin "$$submodule_url"; \
			echo "   ➜ Updated origin to $$submodule_url"; \
		fi; \
		echo "   Fetching latest changes from remote..."; \
		git fetch origin --prune 2>/dev/null || true; \
		if git show-ref --verify --quiet refs/remotes/origin/master 2>/dev/null; then \
			target_branch=master; \
		elif git show-ref --verify --quiet refs/remotes/origin/main 2>/dev/null; then \
			target_branch=main; \
		else \
			echo "   ⚠️  Warning: origin/master and origin/main not found for $(1)"; \
			target_branch=""; \
		fi; \
		if [ -n "$$target_branch" ]; then \
			git checkout $$target_branch >/dev/null 2>&1 || git checkout -B $$target_branch origin/$$target_branch >/dev/null 2>&1 || true; \
			if git rev-parse --verify --quiet origin/$$target_branch >/dev/null 2>&1; then \
				git branch --set-upstream-to=origin/$$target_branch $$target_branch >/dev/null 2>&1 || true; \
			fi; \
			git pull --ff-only origin $$target_branch 2>/dev/null || echo "   ⚠️  Could not fast-forward, submodule may have local changes"; \
		fi; \
		cd ..; \
		echo "✅ $(1) submodule updated successfully!"; \
	fi
endef

submodule-update:
	@echo "Updating all submodules..."
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		echo "   This repository doesn't have submodules configured."; \
		exit 1; \
	fi
	@echo ""
	$(call update_submodule,$(SAILBOAT_SUBMODULE_PATH),$(SAILBOAT_SUBMODULE_NAME))
	@echo ""
	$(call update_submodule,$(BNO08X_SUBMODULE_PATH),$(BNO08X_SUBMODULE_NAME))
	@echo ""
	@echo "Current submodule status:"
	@echo "   $(SAILBOAT_SUBMODULE_PATH):"
	@cd $(SAILBOAT_SUBMODULE_PATH) && git rev-parse --short HEAD 2>/dev/null | sed 's/^/     Commit: /' && cd ../.. || echo "     (not initialized)"
	@echo "   $(BNO08X_SUBMODULE_PATH):"
	@cd $(BNO08X_SUBMODULE_PATH) && git rev-parse --short HEAD 2>/dev/null | sed 's/^/     Commit: /' && cd ../.. || echo "     (not initialized)"
	@echo ""
	@echo "Note: Submodules have been updated to the latest version."
	@echo "      Commit these changes to lock the submodules to these versions:"
	@echo "      git add $(SAILBOAT_SUBMODULE_PATH) $(BNO08X_SUBMODULE_PATH)"
	@echo "      git commit -m 'Update submodules'"

# Helper function to show status of a single submodule
define status_submodule
	@echo "🔍 $(1) Submodule:"
	@if grep -q "$(1)" .gitmodules 2>/dev/null; then \
		echo "✅ $(1) submodule configured"; \
		url=$$(grep -A1 "$(1)" .gitmodules | grep "url" | cut -d'=' -f2 | tr -d ' \t'); \
		echo "   URL: $$url"; \
	else \
		echo "❌ $(1) submodule not found in .gitmodules"; \
	fi
	@if [ -d $(1) ]; then \
		echo "✅ Submodule directory exists: $(1)/"; \
		if [ -f $(1)/.git ] || [ -d $(1)/.git ]; then \
			echo "✅ Submodule is initialized"; \
			echo ""; \
			echo "   Current commit:"; \
			cd $(1) && git log --oneline -1 2>/dev/null | sed 's/^/     /' && cd ../..; \
			echo ""; \
			echo "   Branch information:"; \
			cd $(1) && git branch -v 2>/dev/null | sed 's/^/     /' && cd ../..; \
			echo ""; \
			echo "   Remote status:"; \
			cd $(1) && git status -sb 2>/dev/null | sed 's/^/     /' && cd ../..; \
			echo ""; \
			echo "   Remote URL:"; \
			cd $(1) && git remote get-url origin 2>/dev/null | sed 's/^/     /' && cd ../..; \
		else \
			echo "❌ Submodule directory exists but not initialized"; \
			echo "   Run 'make submodule-init' to initialize"; \
		fi; \
	else \
		echo "❌ Submodule directory not found: $(1)/"; \
		echo "   Run 'make submodule-init' to initialize"; \
	fi
	@echo ""
endef

submodule-status:
	@echo "Argo Submodule Status"
	@echo "====================="
	@echo ""
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		echo "   This repository doesn't have submodules configured."; \
		exit 1; \
	fi
	@echo "🔍 All Configured Submodules:"
	@grep "path = " .gitmodules | sed 's/.*path = /     - /'
	@echo ""
	@echo "===================================="
	@echo ""
	$(call status_submodule,$(SAILBOAT_SUBMODULE_PATH))
	@echo "===================================="
	@echo ""
	$(call status_submodule,$(BNO08X_SUBMODULE_PATH))
	@echo "===================================="
	@echo ""
	@echo "🔍 Local Customization Files:"
	@if [ -d simulator/customizations/sailboat-playground ]; then \
		echo "✅ Customizations directory exists: simulator/customizations/sailboat-playground/"; \
		echo "   This contains Argo-specific configuration files"; \
		echo "   (separate from the submodule source code)"; \
		ls -1 simulator/customizations/sailboat-playground/ 2>/dev/null | sed 's/^/     - /' || echo "     (empty)"; \
	else \
		echo "❌ Customizations directory not found: simulator/customizations/sailboat-playground/"; \
		echo "   This may affect simulator functionality"; \
	fi
	@echo ""
	@echo "💡 Usage:"
	@echo "  make submodule-init    - Initialize all submodules (first time setup)"
	@echo "  make submodule-update  - Update all submodules to latest version"
	@echo "  make submodule-status  - Show this status information"
	@echo "  make submodule-sync    - Sync submodule references (record current commits in parent repo)"

submodule-sync:
	@echo "Syncing submodule references..."
	@echo "This will record the current submodule commits in the parent repository."
	@echo ""
	@if [ ! -f .gitmodules ]; then \
		echo "❌ .gitmodules file not found!"; \
		exit 1; \
	fi
	@echo "Current submodule status:"
	@for submod in $(SAILBOAT_SUBMODULE_PATH) $(BNO08X_SUBMODULE_PATH); do \
		if [ -d "$$submod" ]; then \
			commit=$$(cd "$$submod" && git rev-parse HEAD 2>/dev/null); \
			if [ -n "$$commit" ]; then \
				echo "   $$submod: $$commit"; \
			fi; \
		fi; \
	done
	@echo ""
	@echo "To sync submodule references, run:"
	@echo "  git add $(SAILBOAT_SUBMODULE_PATH) $(BNO08X_SUBMODULE_PATH)"
	@echo "  git commit -m 'Sync submodule references'"
	@echo ""
	@echo "This will lock the submodules to their current commits."

# ==================== MOTD CUSTOMIZATION ====================

MOTD_SCRIPT = scripts/15-argo-shutdown-status
MOTD_TARGET = /etc/update-motd.d/15-argo-shutdown-status

install-motd:
	@echo "Installing Argo shutdown status MOTD script..."
	@if [ ! -f $(MOTD_SCRIPT) ]; then \
		echo "❌ Error: $(MOTD_SCRIPT) not found!"; \
		echo "   Make sure you're running this from the Argo project root directory."; \
		exit 1; \
	fi
	@echo "Copying MOTD script to /etc/update-motd.d/..."
	sudo cp $(MOTD_SCRIPT) $(MOTD_TARGET)
	sudo chmod +x $(MOTD_TARGET)
	@echo "✅ Argo MOTD script installed successfully!"
	@echo ""
	@echo "The MOTD will now show:"
	@echo "  - Critical battery shutdown alerts"
	@echo "  - Low battery warnings from previous session"
	@echo "  - Normal power button shutdown events"
	@echo "  - Current battery status (if concerning)"
	@echo ""
	@echo "📋 Test the MOTD output:"
	@echo "   make test-motd"
	@echo ""
	@echo "🔄 To see it on next login:"
	@echo "   ssh argo (from another terminal)"
	@echo ""
	@echo "💡 MOTD is displayed automatically on SSH login"

uninstall-motd:
	@echo "Uninstalling Argo shutdown status MOTD script..."
	@if [ -f $(MOTD_TARGET) ]; then \
		sudo rm -f $(MOTD_TARGET); \
		echo "✅ Argo MOTD script removed from /etc/update-motd.d/"; \
	else \
		echo "ℹ️  Argo MOTD script was not installed"; \
	fi
	@echo ""
	@echo "MOTD has been restored to default Orange Pi configuration"

test-motd:
	@echo "Testing Argo MOTD script output..."
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@if [ -f $(MOTD_SCRIPT) ]; then \
		bash $(MOTD_SCRIPT); \
	else \
		echo "❌ Error: $(MOTD_SCRIPT) not found!"; \
		exit 1; \
	fi
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo ""
	@echo "💡 This is what will be displayed on SSH login"
	@echo "   To install: make install-motd"

# ==================== BATTERY STATUS MONITOR CONVENIENCE TARGETS ====================

install-battery-monitor:
	@echo "Installing battery status monitor..."
	@$(MAKE) -C scripts install-battery-monitor

setup-battery-panel:
	@echo "Setting up battery status panel in XFCE4..."
	@$(MAKE) -C scripts setup-battery-panel

test-battery-status:
	@echo "Testing battery status functionality..."
	@$(MAKE) -C scripts test-battery-status

# ==================== NETWORK IMPROVEMENTS ====================

install-network-improvements:
	@echo "Installing WiFi reconnection system and NetworkManager optimizations..."
	@if [ -f network/install/install_network_improvements.sh ]; then \
		sudo ./network/install/install_network_improvements.sh; \
	else \
		echo "❌ Error: network/install/install_network_improvements.sh not found!"; \
		echo "   Make sure you're running this from the Argo project root directory."; \
		exit 1; \
	fi

test-wifi-reconnection:
	@echo "Starting WiFi reconnection test..."
	@if [ -f network/scripts/run_wifi_test.sh ]; then \
		./network/scripts/run_wifi_test.sh start; \
	else \
		echo "❌ Error: network/scripts/run_wifi_test.sh not found!"; \
		echo "   Make sure you're running this from the Argo project root directory."; \
		exit 1; \
	fi

wifi-test-status:
	@echo "Checking WiFi test status..."
	@if [ -f network/scripts/run_wifi_test.sh ]; then \
		./network/scripts/run_wifi_test.sh status; \
	else \
		echo "❌ Error: network/scripts/run_wifi_test.sh not found!"; \
		exit 1; \
	fi

wifi-test-results:
	@echo "Showing WiFi test results..."
	@if [ -f network/scripts/run_wifi_test.sh ]; then \
		./network/scripts/run_wifi_test.sh results; \
	else \
		echo "❌ Error: network/scripts/run_wifi_test.sh not found!"; \
		exit 1; \
	fi

wifi-reconnect-status:
	@echo "Checking WiFi reconnection service status..."
	@systemctl status argo_wifi_reconnect.timer --no-pager -l

wifi-reconnect-logs:
	@echo "Showing WiFi reconnection service logs..."
	@journalctl -u argo_wifi_reconnect.service --since "1 hour ago" --no-pager

wifi-reconnect-stop:
	@echo "Stopping WiFi reconnection service..."
	@sudo systemctl stop argo_wifi_reconnect.timer
	@echo "✅ WiFi reconnection service stopped"

wifi-reconnect-start:
	@echo "Starting WiFi reconnection service..."
	@sudo systemctl start argo_wifi_reconnect.timer
	@echo "✅ WiFi reconnection service started"

# ==================== SYSTEM MONITORING SERVICES ====================

# Install all system monitoring services (optional - for debugging)
install-system-monitoring:
	@echo "🔧 Installing system monitoring services..."
	@make -C system-monitoring install-all
	@echo "✅ System monitoring services installed successfully!"
	@echo ""
	@echo "📋 Installed services:"
	@echo "  • Boot history logger - Boot event logging"
	@echo "  • Cursor process monitor - Cursor IDE process monitoring"
	@echo "  • Memory monitor - Memory and process monitoring"
	@echo "  • Orange Pi monitor - Orange Pi hardware monitoring"
	@echo "  • Persistent dmesg - Kernel message logging"
	@echo "  • Crash dump collector - Crash dump collection"
	@echo "  • Temperature logger - Temperature logging"
	@echo ""
	@echo "📝 Log files are stored in: /var/log.hdd/persistent"
	@echo "🔍 Monitor logs with: tail -f /var/log.hdd/persistent/*.log"
	@echo "📊 Check service status with: make -C system-monitoring status"

# Uninstall all system monitoring services
uninstall-system-monitoring:
	@echo "🗑️  Uninstalling system monitoring services..."
	@make -C system-monitoring uninstall-all
	@echo "✅ System monitoring services uninstalled successfully!"
