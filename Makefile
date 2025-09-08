# Argo Robot Services Makefile
# Manages systemd services for ROS2 Argo robot

SERVICE_DIR = /etc/systemd/system
LAUNCH_SERVICE = argo-launch.service
RECORD_SERVICE = argo-record.service
BAGFILES_DIR = /home/orangepi/bagfiles

.PHONY: help install uninstall enable disable start stop restart status clean aliases

help:
	@echo "Argo Robot Services Management"
	@echo "=============================="
	@echo ""
	@echo "Installation:"
	@echo "  install     - Install service files to systemd"
	@echo "  uninstall   - Remove service files from systemd"
	@echo "  enable      - Enable services for automatic startup"
	@echo "  disable     - Disable automatic startup"
	@echo ""
	@echo "Service Control:"
	@echo "  start       - Start argo-launch service"
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
	@echo "  aliases     - Install shell aliases (run: source ~/.bashrc after)"
	@echo ""
	@echo "Quick Commands (after installing aliases):"
	@echo "  al  - Launch argo service"
	@echo "  aq  - Quit argo service"
	@echo "  ar  - Record data"
	@echo "  ac  - Close recording"

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
	@echo "Starting Argo launch service..."
	sudo systemctl start $(LAUNCH_SERVICE)
	@echo "Argo service started!"

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
		echo "alias al='make -C /home/orangepi/argo start'" >> ~/.bash_aliases; \
		echo "alias aq='make -C /home/orangepi/argo stop'" >> ~/.bash_aliases; \
		echo "alias ar='make -C /home/orangepi/argo record'" >> ~/.bash_aliases; \
		echo "alias ac='make -C /home/orangepi/argo stop-record'" >> ~/.bash_aliases; \
		echo "alias as='make -C /home/orangepi/argo status'" >> ~/.bash_aliases; \
		echo "alias ars='make -C /home/orangepi/argo restart'" >> ~/.bash_aliases; \
		echo "alias argo_help='bash /home/orangepi/argo/scripts/argo_help.sh'" >> ~/.bash_aliases; \
		echo "" >> ~/.bash_aliases; \
		echo "✅ Aliases installed to ~/.bash_aliases"; \
	else \
		echo "⚠️  Argo aliases already exist in ~/.bash_aliases"; \
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
	echo "    echo \"🚢 Argo: al=launch, aq=quit, ar=record, ac=close, as=status, ars=restart\"" >> ~/.bashrc; \
	echo "    date +%Y-%m-%d > ~/.argo_reminder_date" >> ~/.bashrc; \
	echo "fi" >> ~/.bashrc; \
	echo "✅ Added daily Argo reminder to ~/.bashrc"
	@echo ""
	@echo "Run: source ~/.bashrc (or open new terminal)"
	@echo ""
	@echo "Available aliases:"
	@echo "  al   - Launch argo service"
	@echo "  aq   - Quit argo service"
	@echo "  ar   - Record data"
	@echo "  ac   - Close recording"
	@echo "  as   - Show service status"
	@echo "  ars  - Restart argo service"
	@echo "  argo_help - Show detailed help"
