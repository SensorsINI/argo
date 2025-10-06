#!/bin/bash

# Test script for ROS2 service communication
# This script installs, starts, and tests the service communication

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_DIR="/etc/systemd/system"

echo "=== ROS2 Service Communication Test ==="
echo "Script directory: $SCRIPT_DIR"

# Function to check if ROS2 is available
check_ros2() {
    echo "Checking ROS2 installation..."
    if command -v ros2 &> /dev/null; then
        echo "✓ ROS2 found at: $(which ros2)"
        ros2 --version
    else
        echo "✗ ROS2 not found in PATH"
        echo "Please source ROS2 setup: source /opt/ros/humble/setup.bash"
        exit 1
    fi
}

# Function to install systemd services
install_services() {
    echo "Installing systemd services..."
    
    # Copy service files to systemd directory
    sudo cp "$SCRIPT_DIR/test-service-server.service" "$SERVICE_DIR/"
    sudo cp "$SCRIPT_DIR/test-service-client.service" "$SERVICE_DIR/"
    
    # Reload systemd
    sudo systemctl daemon-reload
    
    echo "✓ Services installed successfully"
}

# Function to start and test services
test_services() {
    echo "Testing service communication..."
    
    # Enable services
    sudo systemctl enable test-service-server.service
    
    # Start the server
    echo "Starting service server..."
    sudo systemctl start test-service-server.service
    
    # Wait a moment for the server to start
    sleep 3
    
    # Check if server is running
    if sudo systemctl is-active --quiet test-service-server.service; then
        echo "✓ Service server is running"
    else
        echo "✗ Service server failed to start"
        sudo systemctl status test-service-server.service
        return 1
    fi
    
    # Test the client
    echo "Running service client test..."
    sudo systemctl start test-service-client.service
    
    # Check the result
    if [ $? -eq 0 ]; then
        echo "✓ Service client test completed successfully"
    else
        echo "✗ Service client test failed"
        sudo systemctl status test-service-client.service
        return 1
    fi
}

# Function to show logs
show_logs() {
    echo "=== Service Server Logs ==="
    sudo journalctl -u test-service-server.service --no-pager -n 20
    
    echo "=== Service Client Logs ==="
    sudo journalctl -u test-service-client.service --no-pager -n 20
}

# Function to cleanup
cleanup() {
    echo "Cleaning up services..."
    
    # Stop services
    sudo systemctl stop test-service-client.service 2>/dev/null || true
    sudo systemctl stop test-service-server.service 2>/dev/null || true
    
    # Disable services
    sudo systemctl disable test-service-server.service 2>/dev/null || true
    sudo systemctl disable test-service-client.service 2>/dev/null || true
    
    # Remove service files
    sudo rm -f "$SERVICE_DIR/test-service-server.service"
    sudo rm -f "$SERVICE_DIR/test-service-client.service"
    
    # Reload systemd
    sudo systemctl daemon-reload
    
    echo "✓ Cleanup completed"
}

# Main execution
main() {
    case "${1:-test}" in
        "install")
            check_ros2
            install_services
            ;;
        "test")
            check_ros2
            test_services
            show_logs
            ;;
        "logs")
            show_logs
            ;;
        "cleanup")
            cleanup
            ;;
        *)
            echo "Usage: $0 [install|test|logs|cleanup]"
            echo "  install: Install systemd services"
            echo "  test:    Run the service communication test"
            echo "  logs:    Show service logs"
            echo "  cleanup: Remove services and cleanup"
            exit 1
            ;;
    esac
}

main "$@"
