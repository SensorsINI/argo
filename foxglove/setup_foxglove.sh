#!/bin/bash
# Foxglove Setup Script for Argo ROS2 Project
# This script helps set up Foxglove visualization for your argo sailboat

echo "🚢 Argo Foxglove Setup Script"
echo "=============================="

# Check if rosbridge_server is installed
if ! dpkg -l | grep -q ros-humble-rosbridge-suite; then
    echo "📦 Installing rosbridge_server..."
    sudo apt update
    sudo apt install -y ros-humble-rosbridge-suite
    echo "✅ rosbridge_server installed"
else
    echo "✅ rosbridge_server already installed"
fi

# Create a launch file for rosbridge
echo "📝 Creating rosbridge launch file..."
cat > /home/orangepi/argo/foxglove/rosbridge_launch.py << 'EOF'
#!/usr/bin/env python3
"""
Launch file for rosbridge_server to enable Foxglove connection
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen',
            shell=True
        )
    ])
EOF

chmod +x /home/orangepi/argo/foxglove/rosbridge_launch.py

echo "✅ rosbridge launch file created"

# Create a combined launch file that includes rosbridge
echo "📝 Creating combined argo + rosbridge launch file..."
cat > /home/orangepi/argo/foxglove/argo_with_foxglove_launch.py << 'EOF'
#!/usr/bin/env python3
"""
Combined launch file for Argo sailboat with Foxglove support
Launches all sensor nodes, control system, and rosbridge_server
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Declare launch arguments
    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='true',
        description='Whether to launch rosbridge_server for Foxglove'
    )
    
    # Get script directory
    script_dir = '/home/orangepi/argo/scripts'
    
    # Define all argo nodes
    argo_nodes = [
        # Anemometer node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'anem.py')],
            name='anem',
            output='screen'
        ),
        
        # PWM node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'pwm.py')],
            name='pwm',
            output='screen'
        ),
        
        # GPS node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'gps.py')],
            name='gps',
            output='screen'
        ),
        
        # IMU node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'imu.py')],
            name='imu',
            output='screen'
        ),
        
        # Control node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'control.py')],
            name='control',
            output='screen'
        ),
        
        # Battery/Water node
        ExecuteProcess(
            cmd=[os.path.join(script_dir, 'battery_water.py')],
            name='battery_water',
            output='screen'
        ),
    ]
    
    # Rosbridge node for Foxglove
    rosbridge_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rosbridge')),
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        use_rosbridge_arg,
        *argo_nodes,
        rosbridge_node,
    ])
EOF

chmod +x /home/orangepi/argo/foxglove/argo_with_foxglove_launch.py

echo "✅ Combined launch file created"

# Create a simple connection test script
echo "📝 Creating connection test script..."
cat > /home/orangepi/argo/foxglove/test_connection.py << 'EOF'
#!/usr/bin/env python3
"""
Test script to verify Foxglove connection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('foxglove_test_publisher')
        self.publisher = self.create_publisher(String, 'foxglove_test', 10)
        self.timer = self.create_timer(1.0, self.publish_test_message)
        self.counter = 0
        
    def publish_test_message(self):
        msg = String()
        msg.data = f'Foxglove test message {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x /home/orangepi/argo/foxglove/test_connection.py

echo "✅ Connection test script created"

echo ""
echo "🎉 Foxglove setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Start your argo system with Foxglove support:"
echo "   ros2 launch /home/orangepi/argo/foxglove/argo_with_foxglove_launch.py"
echo ""
echo "2. Open Foxglove Studio in your browser:"
echo "   https://studio.foxglove.dev/"
echo ""
echo "3. Connect to your robot:"
echo "   - Click 'Open connection'"
echo "   - Select 'Rosbridge (WebSocket)'"
echo "   - Enter: ws://YOUR_ROBOT_IP:9090"
echo "   - Replace YOUR_ROBOT_IP with your Orange Pi's IP address"
echo ""
echo "4. Load the layout:"
echo "   - Click 'Layout' in the top menu"
echo "   - Select 'Import layout'"
echo "   - Choose: /home/orangepi/argo/foxglove/argo_comprehensive_layout.json"
echo ""
echo "5. Test the connection:"
echo "   ros2 run argo /home/orangepi/argo/foxglove/test_connection.py"
echo ""
echo "🔍 To find your robot's IP address:"
echo "   ip addr show | grep 'inet ' | grep -v '127.0.0.1'"
echo ""
echo "📊 Available layouts:"
echo "   - argo_layout.json (basic 4-panel layout)"
echo "   - argo_comprehensive_layout.json (full 6-panel layout)"
echo ""
