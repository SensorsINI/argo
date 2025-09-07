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
	@echo "  argo_start  - Start argo service"
	@echo "  argo_stop   - Stop argo service"
	@echo "  argo_record - Start recording"
	@echo "  argo_close  - Stop recording"

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
	@echo "" >> ~/.bashrc
	@echo "# Argo Robot Control Aliases" >> ~/.bashrc
	@echo "alias argo_start='make -C /home/orangepi/argo start'" >> ~/.bashrc
	@echo "alias argo_stop='make -C /home/orangepi/argo stop'" >> ~/.bashrc
	@echo "alias argo_record='make -C /home/orangepi/argo record'" >> ~/.bashrc
	@echo "alias argo_close='make -C /home/orangepi/argo stop-record'" >> ~/.bashrc
	@echo "alias argo_status='make -C /home/orangepi/argo status'" >> ~/.bashrc
	@echo "alias argo_restart='make -C /home/orangepi/argo restart'" >> ~/.bashrc
	@echo "" >> ~/.bashrc
	@echo "Aliases installed! Run: source ~/.bashrc"
	@echo ""
	@echo "Available aliases:"
	@echo "  argo_start   - Start argo service"
	@echo "  argo_stop    - Stop argo service"
	@echo "  argo_record  - Start recording"
	@echo "  argo_close   - Stop recording"
	@echo "  argo_status  - Show service status"
	@echo "  argo_restart - Restart argo service"
