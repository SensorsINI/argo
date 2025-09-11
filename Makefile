# Argo Robot Services Makefile
# Manages systemd services for ROS2 Argo robot

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
RECORD_SERVICE = argo-record.service
BAGFILES_DIR = $(HOME)/bagfiles
ARGO_DIR = $(HOME)/argo

.PHONY: help install uninstall enable disable start stop restart status clean aliases install-dotfiles install_power_control install-deps install-foxglove-bridge check-deps aliases-activate aliases-force aliases-install

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
	@echo "  status      - Show status of all services"
	@echo ""
	@echo "Recording Control:"
	@echo "  record      - Start ROS2 bag recording"
	@echo "  stop-record - Stop ROS2 bag recording"
	@echo ""
	@echo "Utilities:"
	@echo "  clean       - Clean old bag files (>7 days)"
	@echo "  aliases-install - Install/update aliases and activate them immediately"
	@echo "  aliases     - Install shell aliases (then run: eval \$$(make aliases-activate))"
	@echo "  aliases-force - Force reinstall/update aliases (overwrites existing)"
	@echo "  aliases-activate - Print command to activate aliases in current shell"
	@echo "  install-dotfiles - Install dotfiles (.bashrc, .bash_aliases, .tmux.conf) to home directory"
	@echo "  install_power_control - Install GPIO permissions for power control (udev rules, gpio group)"
	@echo ""
	@echo "Quick Commands (after installing aliases):"
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

status:
	@echo "Argo Services Status:"
	@echo "===================="
	@systemctl is-active $(LAUNCH_SERVICE) --quiet && echo "✅ Launch service: RUNNING" || echo "❌ Launch service: STOPPED"
	@systemctl is-active $(RECORD_SERVICE) --quiet && echo "✅ Record service: RUNNING" || echo "❌ Record service: STOPPED"
	@echo ""
	@echo "Detailed Status:"
	@systemctl status $(LAUNCH_SERVICE) --no-pager -l || true
	@echo ""
	@systemctl status $(RECORD_SERVICE) --no-pager -l || true

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

aliases:
	@echo "Installing shell aliases..."
	@touch ~/.bash_aliases
	@if ! grep -q "# Argo Robot Control Aliases" ~/.bash_aliases 2>/dev/null; then \
		echo "" >> ~/.bash_aliases; \
		echo "# Argo Robot Control Aliases" >> ~/.bash_aliases; \
		echo "alias al='make -C $(ARGO_DIR) start'" >> ~/.bash_aliases; \
		echo "alias aq='make -C $(ARGO_DIR) stop'" >> ~/.bash_aliases; \
		echo "alias ar='make -C $(ARGO_DIR) record'" >> ~/.bash_aliases; \
		echo "alias ac='make -C $(ARGO_DIR) stop-record'" >> ~/.bash_aliases; \
		echo "alias as='make -C $(ARGO_DIR) status && echo \"\" && echo \"🔍 Recent Argo Errors (last 5m):\" && echo \"===============================================\" && journalctl --since \"5 minutes ago\" -u argo-launch.service -u argo-record.service --priority=err --no-pager -n 20 2>/dev/null || echo \"No recent errors found\"'" >> ~/.bash_aliases; \
		echo "alias ars='make -C $(ARGO_DIR) restart'" >> ~/.bash_aliases; \
		echo "alias argo_help='bash $(ARGO_DIR)/scripts/argo_help.sh'" >> ~/.bash_aliases; \
		echo "" >> ~/.bash_aliases; \
		echo "✅ Aliases installed to ~/.bash_aliases"; \
	else \
		echo "⚠️  Argo aliases already exist in ~/.bash_aliases"; \
		if ! grep -q "afb.*foxglove_bridge" ~/.bash_aliases 2>/dev/null; then \
			echo "🔄 Some aliases appear outdated. Run 'make aliases-force' to update them."; \
		else \
			echo "✅ Aliases appear up-to-date."; \
		fi; \
	fi
	@if ! grep -q "source.*\.bash_aliases" ~/.bashrc 2>/dev/null; then \
		echo "" >> ~/.bashrc; \
		echo "# Source bash aliases if available" >> ~/.bashrc; \
		echo "if [ -f ~/.bash_aliases ]; then" >> ~/.bashrc; \
		echo "    . ~/.bash_aliases" >> ~/.bashrc; \
		echo "fi" >> ~/.bashrc; \
		echo "✅ Added .bash_aliases sourcing to ~/.bashrc"; \
	else \
		echo "✅ .bash_aliases already sourced in ~/.bashrc"; \
	fi
	@echo "" >> ~/.bashrc; \
	echo "# Argo daily reminder (once per day)" >> ~/.bashrc; \
	echo "if [ ! -f ~/.argo_reminder_date ] || [ \"\$$(date +%Y-%m-%d)\" != \"\$$(cat ~/.argo_reminder_date)\" ]; then" >> ~/.bashrc; \
	echo "    echo \"🚢 Argo: al=launch, aq=quit, ar=record, ac=close, as=status, ars=restart, af=foxglove\"" >> ~/.bashrc; \
	echo "    date +%Y-%m-%d > ~/.argo_reminder_date" >> ~/.bashrc; \
	echo "fi" >> ~/.bashrc; \
	echo "✅ Added daily Argo reminder to ~/.bashrc"
	@echo ""
	@echo "🔄 To activate aliases in this terminal, run:"
	@echo "   eval \$$(make aliases-activate)"
	@echo ""
	@echo "💡 In NEW terminals, aliases will be auto-loaded from ~/.bashrc"
	@echo ""
	@echo "Available aliases:"
	@echo "  al   - Launch argo service (with 30s monitoring)"
	@echo "  aq   - Quit argo service"
	@echo "  ar   - Record data"
	@echo "  ac   - Close recording"
	@echo "  as   - Show service status"
	@echo "  ars  - Restart argo service"
	@echo "  argo_help - Show detailed help"

install-dotfiles:
	@echo "Installing dotfiles to home directory..."
	@if [ -f dotfiles/.bashrc ]; then \
		cp dotfiles/.bashrc ~/.bashrc; \
		echo "✅ Installed .bashrc"; \
	else \
		echo "❌ dotfiles/.bashrc not found"; \
	fi
	@if [ -f dotfiles/.bash_aliases ]; then \
		cp dotfiles/.bash_aliases ~/.bash_aliases; \
		echo "✅ Installed .bash_aliases"; \
	else \
		echo "❌ dotfiles/.bash_aliases not found"; \
	fi
	@if [ -f dotfiles/.tmux.conf ]; then \
		cp dotfiles/.tmux.conf ~/.tmux.conf; \
		echo "✅ Installed .tmux.conf"; \
	else \
		echo "❌ dotfiles/.tmux.conf not found"; \
	fi
	@echo ""
	@echo "✅ Dotfiles installation complete!"
	@echo "Run 'source ~/.bashrc' or open a new terminal to apply changes."

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

aliases-force:
	@echo "Force updating shell aliases..."
	@touch ~/.bash_aliases
	@# Remove existing Argo aliases section
	@sed -i '/^# Argo Robot Control Aliases/,/^$$/d' ~/.bash_aliases
	@# Add updated aliases
	@echo "" >> ~/.bash_aliases
	@echo "# Argo Robot Control Aliases" >> ~/.bash_aliases
	@echo "alias al='make -C $(ARGO_DIR) start'" >> ~/.bash_aliases
	@echo "alias aq='make -C $(ARGO_DIR) stop'" >> ~/.bash_aliases
	@echo "alias ar='make -C $(ARGO_DIR) record'" >> ~/.bash_aliases
	@echo "alias ac='make -C $(ARGO_DIR) stop-record'" >> ~/.bash_aliases
	@echo "alias as='make -C $(ARGO_DIR) status && echo \"\" && echo \"🔍 Recent Argo Errors (last 5m):\" && echo \"===============================================\" && journalctl --since \"5 minutes ago\" -u argo-launch.service -u argo-record.service --priority=err --no-pager -n 20 2>/dev/null || echo \"No recent errors found\"'" >> ~/.bash_aliases
	@echo "alias ars='make -C $(ARGO_DIR) restart'" >> ~/.bash_aliases
	@echo "alias argo_help='bash $(ARGO_DIR)/scripts/argo_help.sh'" >> ~/.bash_aliases
	@echo "alias af='ros2 launch $(ARGO_DIR)/launch/argo_launch.py'" >> ~/.bash_aliases
	@echo "alias afb='ros2 run foxglove_bridge foxglove_bridge'" >> ~/.bash_aliases
	@echo "" >> ~/.bash_aliases
	@echo "✅ Aliases force-updated in ~/.bash_aliases"
	@if ! grep -q "source.*\.bash_aliases" ~/.bashrc 2>/dev/null; then \
		echo "" >> ~/.bashrc; \
		echo "# Source bash aliases if available" >> ~/.bashrc; \
		echo "if [ -f ~/.bash_aliases ]; then" >> ~/.bashrc; \
		echo "    . ~/.bash_aliases" >> ~/.bashrc; \
		echo "fi" >> ~/.bashrc; \
		echo "✅ Added .bash_aliases sourcing to ~/.bashrc"; \
	else \
		echo "✅ .bash_aliases already sourced in ~/.bashrc"; \
	fi
	@echo ""
	@echo "🔄 To activate aliases in this terminal, run:"
	@echo "   eval \$$(make aliases-activate)"
	@echo ""
	@echo "💡 In NEW terminals, aliases will be auto-loaded from ~/.bashrc"
	@echo ""
	@echo "Available aliases:"
	@echo "  al   - Launch argo service (with 30s monitoring)"
	@echo "  aq   - Quit argo service"
	@echo "  ar   - Record data"
	@echo "  ac   - Close recording"
	@echo "  as   - Show service status"
	@echo "  ars  - Restart argo service"
	@echo "  af   - Launch argo with integrated Foxglove Bridge"
	@echo "  afb  - Launch Foxglove Bridge (recommended)"
	@echo "  argo_help - Show detailed help"

aliases-install: aliases-force
	@echo ""
	@echo "🔄 Creating activation script..."
	@echo '#!/bin/bash' > /tmp/argo_activate_aliases.sh
	@echo 'source ~/.bash_aliases' >> /tmp/argo_activate_aliases.sh
	@echo 'echo "🚀 Aliases activated! Try these commands:"' >> /tmp/argo_activate_aliases.sh
	@echo 'echo "  afb  # Start Foxglove Bridge"' >> /tmp/argo_activate_aliases.sh
	@echo 'echo "  al   # Launch Argo"' >> /tmp/argo_activate_aliases.sh
	@echo 'echo "  aq   # Quit Argo"' >> /tmp/argo_activate_aliases.sh
	@echo 'echo ""' >> /tmp/argo_activate_aliases.sh
	@echo 'echo "💡 Aliases are now available in this terminal session!"' >> /tmp/argo_activate_aliases.sh
	@chmod +x /tmp/argo_activate_aliases.sh
	@echo ""
	@echo "🚀 To activate aliases in this terminal, run:"
	@echo "   source /tmp/argo_activate_aliases.sh"
	@echo ""
	@echo "⚡ Quick activation: \$$SHELL -c 'source ~/.bash_aliases && \$$SHELL'"

aliases-activate:
	@echo "source ~/.bash_aliases"
