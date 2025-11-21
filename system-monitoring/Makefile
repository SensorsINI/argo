# System Monitoring Services Makefile
# Optional system-level monitoring services for Argo development and debugging
# These services are NOT installed by default and must be explicitly installed

# Service configuration
SERVICE_DIR = /etc/systemd/system
SCRIPT_DIR = /usr/local/bin
PERSISTENT_LOG_DIR = /var/log.hdd/persistent

# Detect the user performing installation (supports sudo)
INSTALL_USER := $(shell if [ -n "$$SUDO_USER" ]; then echo "$$SUDO_USER"; else id -un; fi)
INSTALL_HOME := $(shell getent passwd $(INSTALL_USER) | cut -d: -f6)

# Directory containing this Makefile
MONITORING_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

.PHONY: help install uninstall enable disable start stop restart status install-all uninstall-all clean-logs test-services

# Default target
all: help

# ==================== INDIVIDUAL SERVICE MANAGEMENT ====================

# Boot History Logger Service
install-boot-logger:
	@echo "📋 Installing boot history logger service..."
	@sudo cp $(MONITORING_DIR)/services/boot-history-logger.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/scripts/boot-logger.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/boot-logger.sh
	@sudo systemctl daemon-reload
	@sudo systemctl enable boot-history-logger.service
	@echo "✅ Boot history logger service installed and enabled"

uninstall-boot-logger:
	@echo "🗑️  Uninstalling boot history logger service..."
	@sudo systemctl stop boot-history-logger.service 2>/dev/null || true
	@sudo systemctl disable boot-history-logger.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/boot-history-logger.service
	@sudo rm -f $(SCRIPT_DIR)/boot-logger.sh
	@sudo systemctl daemon-reload
	@echo "✅ Boot history logger service uninstalled"

# Cursor Process Monitor Service
install-cursor-monitor:
	@echo "📋 Installing cursor process monitor service..."
	@sudo cp $(MONITORING_DIR)/services/cursor-monitor.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/scripts/cursor-monitor.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/cursor-monitor.sh
	@sudo systemctl daemon-reload
	@sudo systemctl enable cursor-monitor.service
	@sudo systemctl start cursor-monitor.service
	@echo "✅ Cursor process monitor service installed and started"

uninstall-cursor-monitor:
	@echo "🗑️  Uninstalling cursor process monitor service..."
	@sudo systemctl stop cursor-monitor.service 2>/dev/null || true
	@sudo systemctl disable cursor-monitor.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/cursor-monitor.service
	@sudo rm -f $(SCRIPT_DIR)/cursor-monitor.sh
	@sudo systemctl daemon-reload
	@echo "✅ Cursor process monitor service uninstalled"

# Memory Monitor Service
install-memory-monitor:
	@echo "📋 Installing memory monitor service..."
	@sudo cp $(MONITORING_DIR)/services/memory-monitor.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/scripts/memory-monitor.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/memory-monitor.sh
	@sudo systemctl daemon-reload
	@sudo systemctl enable memory-monitor.service
	@sudo systemctl start memory-monitor.service
	@echo "✅ Memory monitor service installed and started"

uninstall-memory-monitor:
	@echo "🗑️  Uninstalling memory monitor service..."
	@sudo systemctl stop memory-monitor.service 2>/dev/null || true
	@sudo systemctl disable memory-monitor.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/memory-monitor.service
	@sudo rm -f $(SCRIPT_DIR)/memory-monitor.sh
	@sudo systemctl daemon-reload
	@echo "✅ Memory monitor service uninstalled"

# Orange Pi Monitor Service
install-orangepi-monitor:
	@echo "📋 Installing Orange Pi monitor service..."
	@sudo cp $(MONITORING_DIR)/services/orangepi-monitor.service $(SERVICE_DIR)/
	@sudo systemctl daemon-reload
	@sudo systemctl enable orangepi-monitor.service
	@sudo systemctl start orangepi-monitor.service
	@echo "✅ Orange Pi monitor service installed and started"

uninstall-orangepi-monitor:
	@echo "🗑️  Uninstalling Orange Pi monitor service..."
	@sudo systemctl stop orangepi-monitor.service 2>/dev/null || true
	@sudo systemctl disable orangepi-monitor.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/orangepi-monitor.service
	@sudo systemctl daemon-reload
	@echo "✅ Orange Pi monitor service uninstalled"

# Persistent dmesg Service
install-persistent-dmesg:
	@echo "📋 Installing persistent dmesg service..."
	@sudo cp $(MONITORING_DIR)/services/persistent-dmesg.service $(SERVICE_DIR)/
	@sudo systemctl daemon-reload
	@sudo systemctl enable persistent-dmesg.service
	@echo "✅ Persistent dmesg service installed and enabled"

uninstall-persistent-dmesg:
	@echo "🗑️  Uninstalling persistent dmesg service..."
	@sudo systemctl stop persistent-dmesg.service 2>/dev/null || true
	@sudo systemctl disable persistent-dmesg.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/persistent-dmesg.service
	@sudo systemctl daemon-reload
	@echo "✅ Persistent dmesg service uninstalled"

# Persistent Log Manager Service
install-persistent-log-manager:
	@echo "📋 Installing Argo persistent log manager..."
	@sudo cp $(MONITORING_DIR)/services/argo-persistent-logs.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/services/argo-persistent-logs-prune.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/services/argo-persistent-logs-prune.timer $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/../scripts/argo_persistent_log_manager.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/argo_persistent_log_manager.sh
	@if [ -f /etc/logrotate.d/persistent-logs ]; then \
		 sudo mv /etc/logrotate.d/persistent-logs /etc/logrotate.d/persistent-logs.disabled; \
		 echo "↪︎ Disabled legacy logrotate policy for persistent logs"; \
	 fi
	@sudo systemctl daemon-reload
	@sudo systemctl enable argo-persistent-logs.service
	@sudo systemctl enable argo-persistent-logs-prune.timer
	@sudo systemctl start argo-persistent-logs.service
	@sudo systemctl start argo-persistent-logs-prune.timer
	@echo "✅ Argo persistent log manager installed"

uninstall-persistent-log-manager:
	@echo "🗑️  Uninstalling Argo persistent log manager..."
	@sudo systemctl stop argo-persistent-logs-prune.timer 2>/dev/null || true
	@sudo systemctl disable argo-persistent-logs-prune.timer 2>/dev/null || true
	@sudo systemctl disable argo-persistent-logs.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/argo-persistent-logs.service
	@sudo rm -f $(SERVICE_DIR)/argo-persistent-logs-prune.service
	@sudo rm -f $(SERVICE_DIR)/argo-persistent-logs-prune.timer
	@sudo rm -f $(SCRIPT_DIR)/argo_persistent_log_manager.sh
	@if [ -f /etc/logrotate.d/persistent-logs.disabled ] && [ ! -f /etc/logrotate.d/persistent-logs ]; then \
		 sudo mv /etc/logrotate.d/persistent-logs.disabled /etc/logrotate.d/persistent-logs; \
		 echo "↩︎ Restored original logrotate policy for persistent logs"; \
	 fi
	@sudo systemctl daemon-reload
	@echo "✅ Argo persistent log manager uninstalled"

# Crash Dump Service
install-crash-dump:
	@echo "📋 Installing crash dump service..."
	@sudo cp $(MONITORING_DIR)/services/crash-dump.service $(SERVICE_DIR)/
	@sudo systemctl daemon-reload
	@sudo systemctl enable crash-dump.service
	@echo "✅ Crash dump service installed and enabled"

uninstall-crash-dump:
	@echo "🗑️  Uninstalling crash dump service..."
	@sudo systemctl stop crash-dump.service 2>/dev/null || true
	@sudo systemctl disable crash-dump.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/crash-dump.service
	@sudo systemctl daemon-reload
	@echo "✅ Crash dump service uninstalled"

# Temperature Logger Service
install-temp-logger:
	@echo "📋 Installing temperature logger service..."
	@sudo cp $(MONITORING_DIR)/services/temp-logger.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/services/temp-logger.timer $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/scripts/temp-logger.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/temp-logger.sh
	@sudo systemctl daemon-reload
	@sudo systemctl enable temp-logger.timer
	@sudo systemctl start temp-logger.timer
	@echo "✅ Temperature logger service installed and started"

uninstall-temp-logger:
	@echo "🗑️  Uninstalling temperature logger service..."
	@sudo systemctl stop temp-logger.timer 2>/dev/null || true
	@sudo systemctl disable temp-logger.timer 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/temp-logger.service
	@sudo rm -f $(SERVICE_DIR)/temp-logger.timer
	@sudo rm -f $(SCRIPT_DIR)/temp-logger.sh
	@sudo systemctl daemon-reload
	@echo "✅ Temperature logger service uninstalled"

# Shutdown Logger Service
install-shutdown-logger:
	@echo "📋 Installing shutdown logger service..."
	@sudo cp $(MONITORING_DIR)/shutdown-logger.service $(SERVICE_DIR)/
	@sudo cp $(MONITORING_DIR)/scripts/shutdown-logger.sh $(SCRIPT_DIR)/
	@sudo chmod +x $(SCRIPT_DIR)/shutdown-logger.sh
	@sudo systemctl daemon-reload
	@sudo systemctl enable shutdown-logger.service
	@echo "✅ Shutdown logger service installed and enabled"

uninstall-shutdown-logger:
	@echo "🗑️  Uninstalling shutdown logger service..."
	@sudo systemctl disable shutdown-logger.service 2>/dev/null || true
	@sudo rm -f $(SERVICE_DIR)/shutdown-logger.service
	@sudo rm -f $(SCRIPT_DIR)/shutdown-logger.sh
	@sudo systemctl daemon-reload
	@echo "✅ Shutdown logger service uninstalled"

# ==================== BULK OPERATIONS ====================

# Install all system monitoring services
install-all: install-boot-logger install-cursor-monitor install-memory-monitor install-orangepi-monitor install-persistent-dmesg install-persistent-log-manager install-crash-dump install-temp-logger install-shutdown-logger
	@echo ""
	@echo "✅ All system monitoring services installed successfully!"
	@echo ""
	@echo "📋 Installed services:"
	@echo "  • boot-history-logger.service - Boot event logging"
	@echo "  • cursor-monitor.service - Cursor IDE process monitoring"
	@echo "  • memory-monitor.service - Memory and process monitoring"
	@echo "  • orangepi-monitor.service - Orange Pi hardware monitoring"
	@echo "  • persistent-dmesg.service - Kernel message logging"
	@echo "  • argo-persistent-logs.service - Argo persistent log rotation & pruning"
	@echo "  • crash-dump.service - Crash dump collection"
	@echo "  • temp-logger.service - Temperature logging"
	@echo "  • shutdown-logger.service - Shutdown event logging"
	@echo ""
	@echo "📝 Log files are stored in: $(PERSISTENT_LOG_DIR)"
	@echo "🔍 Monitor logs with: tail -f $(PERSISTENT_LOG_DIR)/*.log"

# Uninstall all system monitoring services
uninstall-all: uninstall-boot-logger uninstall-cursor-monitor uninstall-memory-monitor uninstall-orangepi-monitor uninstall-persistent-dmesg uninstall-persistent-log-manager uninstall-crash-dump uninstall-temp-logger uninstall-shutdown-logger
	@echo ""
	@echo "✅ All system monitoring services uninstalled successfully!"

# ==================== SERVICE MANAGEMENT ====================

# Enable all services
enable: install-all
	@echo "✅ All system monitoring services enabled"

# Disable all services
disable: uninstall-all
	@echo "✅ All system monitoring services disabled"

# Start all services
start:
	@echo "▶️  Starting all system monitoring services..."
	@sudo systemctl start boot-history-logger.service 2>/dev/null || true
	@sudo systemctl start cursor-monitor.service 2>/dev/null || true
	@sudo systemctl start memory-monitor.service 2>/dev/null || true
	@sudo systemctl start orangepi-monitor.service 2>/dev/null || true
	@sudo systemctl start persistent-dmesg.service 2>/dev/null || true
	@sudo systemctl start crash-dump.service 2>/dev/null || true
	@sudo systemctl start temp-logger.timer 2>/dev/null || true
	@echo "✅ All system monitoring services started"

# Stop all services
stop:
	@echo "⏹️  Stopping all system monitoring services..."
	@sudo systemctl stop boot-history-logger.service 2>/dev/null || true
	@sudo systemctl stop cursor-monitor.service 2>/dev/null || true
	@sudo systemctl stop memory-monitor.service 2>/dev/null || true
	@sudo systemctl stop orangepi-monitor.service 2>/dev/null || true
	@sudo systemctl stop persistent-dmesg.service 2>/dev/null || true
	@sudo systemctl stop crash-dump.service 2>/dev/null || true
	@sudo systemctl stop temp-logger.timer 2>/dev/null || true
	@echo "✅ All system monitoring services stopped"

# Restart all services
restart: stop start
	@echo "✅ All system monitoring services restarted"

# Check status of all services
status:
	@echo "📊 System Monitoring Services Status:"
	@echo ""
	@echo "Boot History Logger:"
	@sudo systemctl status boot-history-logger.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Cursor Monitor:"
	@sudo systemctl status cursor-monitor.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Memory Monitor:"
	@sudo systemctl status memory-monitor.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Orange Pi Monitor:"
	@sudo systemctl status orangepi-monitor.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Persistent dmesg:"
	@sudo systemctl status persistent-dmesg.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Crash Dump:"
	@sudo systemctl status crash-dump.service --no-pager -l | sed 's/^/  /'
	@echo ""
	@echo "Temperature Logger:"
	@sudo systemctl status temp-logger.timer --no-pager -l | sed 's/^/  /'

# ==================== UTILITIES ====================

# Clean old log files (older than 30 days)
clean-logs:
	@echo "🧹 Cleaning old log files (older than 30 days)..."
	@find $(PERSISTENT_LOG_DIR) -name "*.log" -mtime +30 -type f -exec rm -f {} \; 2>/dev/null || true
	@find $(PERSISTENT_LOG_DIR) -name "*.csv" -mtime +30 -type f -exec rm -f {} \; 2>/dev/null || true
	@echo "✅ Old log files cleaned"

# Test all services
test-services:
	@echo "🧪 Testing system monitoring services..."
	@echo ""
	@echo "Testing boot logger script..."
	@sudo $(SCRIPT_DIR)/boot-logger.sh
	@echo "✅ Boot logger test completed"
	@echo ""
	@echo "Testing cursor monitor script..."
	@timeout 10 sudo $(SCRIPT_DIR)/cursor-monitor.sh || echo "✅ Cursor monitor test completed (timeout expected)"
	@echo ""
	@echo "Testing memory monitor script..."
	@timeout 10 sudo $(SCRIPT_DIR)/memory-monitor.sh || echo "✅ Memory monitor test completed (timeout expected)"
	@echo ""
	@echo "Testing temperature logger script..."
	@sudo $(SCRIPT_DIR)/temp-logger.sh
	@echo "✅ Temperature logger test completed"
	@echo ""
	@echo "✅ All service tests completed"

# Show help
help:
	@echo "System Monitoring Services Makefile"
	@echo "===================================="
	@echo ""
	@echo "This Makefile manages optional system-level monitoring services"
	@echo "that are useful for debugging and development but NOT installed by default."
	@echo ""
	@echo "Individual Service Management:"
	@echo "  install-boot-logger      - Install boot history logger service"
	@echo "  install-cursor-monitor   - Install cursor process monitor service"
	@echo "  install-memory-monitor   - Install memory monitor service"
	@echo "  install-orangepi-monitor - Install Orange Pi monitor service"
	@echo "  install-persistent-dmesg - Install persistent dmesg service"
	@echo "  install-persistent-log-manager - Install Argo persistent log manager"
	@echo "  install-crash-dump       - Install crash dump service"
	@echo "  install-temp-logger      - Install temperature logger service"
	@echo "  install-shutdown-logger  - Install shutdown event logger service"
	@echo ""
	@echo "Bulk Operations:"
	@echo "  install-all              - Install ALL system monitoring services"
	@echo "  uninstall-all            - Uninstall ALL system monitoring services"
	@echo "  enable                   - Enable all services (same as install-all)"
	@echo "  disable                  - Disable all services (same as uninstall-all)"
	@echo ""
	@echo "Service Management:"
	@echo "  start                    - Start all installed services"
	@echo "  stop                     - Stop all services"
	@echo "  restart                  - Restart all services"
	@echo "  status                   - Show status of all services"
	@echo ""
	@echo "Utilities:"
	@echo "  clean-logs               - Clean log files older than 30 days"
	@echo "  test-services            - Test all service scripts"
	@echo "  help                     - Show this help message"
	@echo ""
	@echo "Usage examples:"
	@echo "  make install-all         # Install all monitoring services"
	@echo "  make status              # Check service status"
	@echo "  make clean-logs          # Clean old log files"
	@echo "  make uninstall-all       # Remove all services"
	@echo ""
	@echo "Log files are stored in: $(PERSISTENT_LOG_DIR)"
	@echo "Monitor logs with: tail -f $(PERSISTENT_LOG_DIR)/*.log"
