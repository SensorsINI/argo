# Argo Robot Services Makefile
# Manages systemd services for ROS2 Argo robot

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
RECORD_SERVICE = argo-record.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

.PHONY: help install uninstall enable disable start stop restart clean install-argo-cli install_power_control install-deps install-foxglove-bridge check-deps aliases-activate aliases-force aliases-install

help:
	@echo "Argo Robot Services Management"
	@echo "=============================="
	@echo ""
	@echo "Installation:"
	@echo "  install-deps         - Install all ROS2 dependencies (foxglove-bridge, etc.)"
	@echo "  install-foxglove-bridge - Install foxglove-bridge package only"
	@echo "  check-deps           - Check status of all dependencies"
	@echo "  install              - Install service files to systemd"
	@echo "  uninstall            - Remove service files from systemd"
	@echo "  enable               - Enable services for automatic startup"
	@echo "  disable              - Disable automatic startup"
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
	@echo "  install_power_control - Install GPIO permissions for power control (udev rules, gpio group)"
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
	@echo "  1. Run your Python ROS2 node: python3 scripts/battery_water.py"
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

install:
	@echo "Installing Argo services..."
	sudo cp launch/$(LAUNCH_SERVICE) $(SERVICE_DIR)/
	sudo cp launch/$(RECORD_SERVICE) $(SERVICE_DIR)/
	sudo systemctl daemon-reload
	mkdir -p $(BAGFILES_DIR)
	@echo "Services installed successfully!"
	@echo "Run 'make enable' to enable automatic startup"

uninstall:
	@echo "Uninstalling Argo services..."
	sudo systemctl stop $(LAUNCH_SERVICE) $(RECORD_SERVICE) 2>/dev/null || true
	sudo systemctl disable $(LAUNCH_SERVICE) $(RECORD_SERVICE) 2>/dev/null || true
	sudo rm -f $(SERVICE_DIR)/$(LAUNCH_SERVICE)
	sudo rm -f $(SERVICE_DIR)/$(RECORD_SERVICE)
	sudo systemctl daemon-reload
	@echo "Services uninstalled successfully!"

enable:
	@echo "Enabling Argo services for automatic startup..."
	sudo systemctl enable $(LAUNCH_SERVICE)
	sudo systemctl enable $(RECORD_SERVICE)
	@echo "Services enabled!"

disable:
	@echo "Disabling automatic startup..."
	sudo systemctl disable $(LAUNCH_SERVICE)
	sudo systemctl disable $(RECORD_SERVICE)
	@echo "Automatic startup disabled!"

start:
	@echo "Starting Argo launch service with monitoring..."
	sudo systemctl start $(LAUNCH_SERVICE)
	@echo "✅ Service started! Monitoring for node failures (30s timeout)..."
	@echo "================================================"
	@echo "🔍 Monitoring journal for errors in real-time..."
	@# Monitor journal for 30 seconds, consuming output and checking for errors
	@timeout 30s journalctl -u $(LAUNCH_SERVICE) -f --no-pager | \
		( \
			ERRORS_FOUND=0; \
			PROCESS_DEATHS=0; \
			CRITICAL_ERRORS=0; \
			DEAD_NODES=""; \
			FAILURE_CAUSES=""; \
			while IFS= read -r line; do \
				echo "$$line"; \
				if echo "$$line" | grep -i "process has died" > /dev/null 2>&1; then \
					PROCESS_DEATHS=1; \
					ERRORS_FOUND=1; \
					NODE_NAME=$$(echo "$$line" | sed -n 's/.*\[\([^-]*\)-[0-9]*\]:.*process has died.*/\1/p'); \
					EXIT_CODE=$$(echo "$$line" | sed -n 's/.*exit code \([0-9]*\).*/\1/p'); \
					if [ -n "$$NODE_NAME" ]; then \
						DEAD_NODES="$$DEAD_NODES $$NODE_NAME"; \
					fi; \
				fi; \
				if echo "$$line" | grep -i -E "(CRITICAL|error.*exiting|failed.*exiting)" | grep -v -i "gps.*satellite" | grep -v -i "gps.*fix" > /dev/null 2>&1; then \
					CRITICAL_ERRORS=1; \
					ERRORS_FOUND=1; \
					ERROR_MSG=$$(echo "$$line" | sed 's/.*\[[^-]*-[0-9]*\]:[[:space:]]*//'); \
					if [ -n "$$ERROR_MSG" ]; then \
						FAILURE_CAUSES="$$FAILURE_CAUSES\n  - $$ERROR_MSG"; \
					fi; \
				fi; \
			done; \
			echo ""; \
			echo "================================================"; \
			echo "📊 Monitoring Summary:"; \
			if [ "$$PROCESS_DEATHS" -eq 1 ]; then \
				echo "❌ CRITICAL: Node processes have died!"; \
				echo ""; \
				echo "Failed nodes:"; \
				for node in $$DEAD_NODES; do \
					echo "  - $$node"; \
				done; \
				if [ -n "$$FAILURE_CAUSES" ]; then \
					echo ""; \
					echo "Failure causes:"; \
					echo -e "$$FAILURE_CAUSES"; \
				fi; \
			elif [ "$$CRITICAL_ERRORS" -eq 1 ]; then \
				echo "❌ WARNING: Critical errors detected!"; \
				if [ -n "$$FAILURE_CAUSES" ]; then \
					echo ""; \
					echo "Error details:"; \
					echo -e "$$FAILURE_CAUSES"; \
				fi; \
			else \
				echo "✅ No critical errors detected during 30s monitoring"; \
			fi; \
		) || \
		( \
			echo ""; \
			echo "================================================"; \
			echo "📊 Monitoring Summary (timeout reached):"; \
			echo "✅ 30-second monitoring completed"; \
		)


stop:
	@echo "Stopping all Argo services..."
	sudo systemctl stop $(RECORD_SERVICE) 2>/dev/null || true
	sudo systemctl stop $(LAUNCH_SERVICE) 2>/dev/null || true
	@echo "All Argo services stopped!"

restart:
	@echo "Restarting Argo launch service..."
	sudo systemctl restart $(LAUNCH_SERVICE)
	@echo "Argo service restarted!"

record:
	@echo "Starting ROS2 bag recording..."
	sudo systemctl start $(RECORD_SERVICE)
	@echo "Recording started!"

stop-record:
	@echo "Stopping ROS2 bag recording..."
	sudo systemctl stop $(RECORD_SERVICE)
	@echo "Recording stopped!"

clean:
	@echo "Cleaning old bag files (>7 days)..."
	find $(BAGFILES_DIR) -name "argo_*" -type d -mtime +7 -exec rm -rf {} \; 2>/dev/null || true
	@echo "Old bag files cleaned!"

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

install_power_control:
	@echo "Installing power control GPIO permissions..."
	@echo "Creating gpio group (if it doesn't exist)..."
	@sudo groupadd gpio 2>/dev/null || echo "gpio group already exists"
	@echo "Installing udev rules for GPIO access..."
	@echo "# GPIO permissions for Orange Pi Zero 2W" | sudo tee /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "# Allow members of the 'gpio' group to access GPIO devices" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "# GPIO chip devices" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo 'SUBSYSTEM=="gpio", GROUP="gpio", MODE="0664"' | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "# GPIO character devices (gpiochip)" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo 'KERNEL=="gpiochip*", GROUP="gpio", MODE="0664"' | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "# GPIO sysfs interface" | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo 'SUBSYSTEM=="gpio", KERNEL=="export", GROUP="gpio", MODE="0664"' | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo 'SUBSYSTEM=="gpio", KERNEL=="unexport", GROUP="gpio", MODE="0664"' | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo 'SUBSYSTEM=="gpio", KERNEL=="gpio*", GROUP="gpio", MODE="0664"' | sudo tee -a /etc/udev/rules.d/99-gpio-permissions.rules > /dev/null
	@echo "Adding current user to gpio group..."
	@sudo usermod -a -G gpio $$(whoami)
	@echo "Reloading udev rules..."
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger
	@echo ""
	@echo "✅ Power control GPIO permissions installed successfully!"
	@echo ""
	@echo "📋 Next steps:"
	@echo "  1. Log out and log back in, OR run: newgrp gpio"
	@echo "  2. Test with: ./scripts/power_control.py --test-mode"
	@echo ""
	@echo "🔧 What was installed:"
	@echo "  - Created 'gpio' group"
	@echo "  - Added udev rules for GPIO device access"
	@echo "  - Added user '$$(whoami)' to gpio group"
	@echo "  - Reloaded udev rules to apply changes"

