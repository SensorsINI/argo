#!/usr/bin/env python3
"""
Argo Web Dashboard Node
========================

Mobile-friendly web interface for monitoring and controlling Argo sailboat.

Features:
- Real-time status monitoring (nodes, battery, GPS, sensors)
- Controller switching (Proportional, Wind-Aware, Return-to-Home)
- System control (start/stop, pause, recording)
- Optimized for phone screens with large touch targets

Access: http://ORANGEPI_IP:8080
"""

import os
import sys
import json
import time
import subprocess
import threading
from pathlib import Path
from typing import Dict, Any, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

# Flask web server
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS

# Add launch directory to path for ArgoNodeManager
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'launch'))
from argo_node_utils import ArgoNodeManager


class ArgoWebDashboard(Node):
    """ROS2 node providing web-based monitoring and control interface."""
    
    def __init__(self):
        super().__init__('argo_web_dashboard')
        self.get_logger().info('Starting Argo Web Dashboard...')
        
        # Initialize state storage (thread-safe with lock)
        self.state_lock = threading.Lock()
        self.state = {
            # System status
            'nodes_running': 0,
            'nodes_total': 0,
            'nodes_list': {},
            'system_running': False,
            'system_paused': False,
            
            # Battery status
            'battery_voltage': None,
            'battery_pct': None,
            'battery_charging': None,
            'battery_usb_power': None,
            'battery_time_to_full': None,
            'battery_time_to_empty': None,
            
            # Temperature
            'cpu_temp': None,
            
            # GPS
            'gps_locked': False,
            'gps_satellites': 0,
            'gps_latitude': None,
            'gps_longitude': None,
            'gps_cog': None,
            'gps_sog': None,
            
            # Navigation
            'compass_heading': None,
            
            # Wind
            'wind_speed': None,
            'wind_angle': None,
            'wind_temp': None,
            
            # Controller
            'human_controlled': True,
            'controller_type': 'Unknown',
            
            # Recording
            'recording': False,
            
            # Home position
            'home_latitude': None,
            'home_longitude': None,
            'distance_to_home': None,
            'bearing_to_home': None,
            
            # Timestamps
            'last_update': time.time()
        }
        
        # Initialize ArgoNodeManager for system status
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # ROS2 Service clients
        self.toggle_pause_client = self.create_client(Trigger, '/toggle_pause')
        self.battery_status_client = self.create_client(Trigger, '/battery_status')
        self.recording_start_client = self.create_client(Trigger, '/argo/recording/start')
        self.recording_stop_client = self.create_client(Trigger, '/argo/recording/stop')
        self.controller_switch_client = self.create_client(Trigger, '/controller_node/switch_controller')
        
        # ROS2 Subscriptions for real-time data
        self.create_subscription(Bool, '/human_controlled', self.human_control_cb, 10)
        self.create_subscription(Float32, '/battery_voltage', self.battery_voltage_cb, 10)
        self.create_subscription(Float32, '/battery_remaining_pct', self.battery_pct_cb, 10)
        self.create_subscription(Vector3, '/compass', self.compass_cb, 10)
        self.create_subscription(Vector3, '/pose', self.pose_cb, 10)
        self.create_subscription(Float64, '/gps_cog', self.gps_cog_cb, 10)
        self.create_subscription(Float64, '/gps_sog', self.gps_sog_cb, 10)
        self.create_subscription(Vector3, '/anem_speed_angle_temp', self.wind_cb, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_fix_cb, 10)
        self.create_subscription(String, '/controller_state', self.controller_state_cb, 10)
        
        # Timer for periodic status updates
        self.create_timer(2.0, self.update_system_status)
        
        # Flask app setup
        self.app = Flask(__name__, 
                        template_folder=os.path.join(os.path.dirname(__file__), 'templates'),
                        static_folder=os.path.join(os.path.dirname(__file__), 'static'))
        CORS(self.app)
        self.setup_routes()
        
        # Start Flask in separate thread
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info('🌐 Web dashboard started on http://0.0.0.0:8081')
        self.get_logger().info('   Access from phone: http://ORANGEPI_IP:8081')
    
    # ==================== ROS2 Callbacks ====================
    
    def human_control_cb(self, msg):
        with self.state_lock:
            self.state['human_controlled'] = msg.data
    
    def battery_voltage_cb(self, msg):
        with self.state_lock:
            self.state['battery_voltage'] = msg.data
    
    def battery_pct_cb(self, msg):
        with self.state_lock:
            self.state['battery_pct'] = msg.data
    
    def compass_cb(self, msg):
        with self.state_lock:
            self.state['compass_heading'] = msg.z
    
    def pose_cb(self, msg):
        with self.state_lock:
            self.state['compass_heading'] = msg.z
    
    def gps_cog_cb(self, msg):
        with self.state_lock:
            self.state['gps_cog'] = msg.data
    
    def gps_sog_cb(self, msg):
        with self.state_lock:
            self.state['gps_sog'] = msg.data
    
    def wind_cb(self, msg):
        with self.state_lock:
            self.state['wind_speed'] = msg.x
            self.state['wind_angle'] = msg.y
            self.state['wind_temp'] = msg.z
    
    def gps_fix_cb(self, msg):
        with self.state_lock:
            self.state['gps_locked'] = (msg.status.status >= 0)
            self.state['gps_latitude'] = msg.latitude if msg.latitude != 0.0 else None
            self.state['gps_longitude'] = msg.longitude if msg.longitude != 0.0 else None
            
            # Set home position on first valid fix
            if (self.state['home_latitude'] is None and 
                self.state['gps_latitude'] is not None):
                self.state['home_latitude'] = self.state['gps_latitude']
                self.state['home_longitude'] = self.state['gps_longitude']
                self.get_logger().info(
                    f"🏠 Home position set: {self.state['home_latitude']:.6f}°, "
                    f"{self.state['home_longitude']:.6f}°")
            
            # Calculate distance and bearing to home
            if (self.state['home_latitude'] is not None and 
                self.state['gps_latitude'] is not None):
                self.state['distance_to_home'] = self._calculate_distance(
                    self.state['gps_latitude'], self.state['gps_longitude'],
                    self.state['home_latitude'], self.state['home_longitude'])
                self.state['bearing_to_home'] = self._calculate_bearing(
                    self.state['gps_latitude'], self.state['gps_longitude'],
                    self.state['home_latitude'], self.state['home_longitude'])
    
    def controller_state_cb(self, msg):
        with self.state_lock:
            self.state['controller_type'] = msg.data
    
    # ==================== Utility Functions ====================
    
    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance in nautical miles using Haversine formula."""
        import math
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        return 3440.065 * c  # Earth radius in nautical miles
    
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing in degrees."""
        import math
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        return (math.degrees(bearing) + 360) % 360
    
    def update_system_status(self):
        """Periodically update system status (nodes, battery, CPU temp)."""
        try:
            # Update node status
            node_status = self.node_manager.get_node_status()
            running_nodes = [node for node, info in node_status.items() if info.get('running', False)]
            
            with self.state_lock:
                self.state['nodes_running'] = len(running_nodes)
                self.state['nodes_total'] = len(node_status)
                self.state['nodes_list'] = {
                    node: '🟢 RUNNING' if info.get('running', False) else '🔴 STOPPED'
                    for node, info in node_status.items()
                }
                self.state['system_running'] = len(running_nodes) > 0
            
            # Get battery status via service
            self._update_battery_status()
            
            # Get CPU temperature
            self._update_cpu_temp()
            
            with self.state_lock:
                self.state['last_update'] = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Error updating system status: {e}")
    
    def _update_battery_status(self):
        """Get detailed battery status from battery service."""
        try:
            if not self.battery_status_client.wait_for_service(timeout_sec=0.5):
                return
            
            request = Trigger.Request()
            future = self.battery_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done() and future.result().success:
                data = json.loads(future.result().message)
                raw_data = data.get('raw_data', {})
                
                with self.state_lock:
                    self.state['battery_charging'] = raw_data.get('charging_status')
                    self.state['battery_usb_power'] = raw_data.get('ac_power_present')
                    self.state['battery_time_to_full'] = raw_data.get('time_to_full_hours')
                    self.state['battery_time_to_empty'] = raw_data.get('time_to_empty_hours')
                    
        except Exception as e:
            self.get_logger().debug(f"Battery status update error: {e}")
    
    def _update_cpu_temp(self):
        """Read CPU temperature from thermal zone."""
        try:
            with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                with self.state_lock:
                    self.state['cpu_temp'] = temp_millicelsius // 1000
        except Exception:
            pass
    
    # ==================== Flask Routes ====================
    
    def setup_routes(self):
        """Setup Flask routes for web interface."""
        
        @self.app.route('/')
        def index():
            """Main dashboard page."""
            return render_template('dashboard.html')
        
        @self.app.route('/api/status')
        def get_status():
            """Get current system status as JSON."""
            with self.state_lock:
                return jsonify(self.state.copy())
        
        @self.app.route('/api/toggle_pause', methods=['POST'])
        def toggle_pause():
            """Toggle pause state of all nodes."""
            return self._call_service(self.toggle_pause_client, '/toggle_pause')
        
        @self.app.route('/api/controller/switch', methods=['POST'])
        def switch_controller():
            """Switch controller type."""
            data = request.get_json()
            controller_type = data.get('type', '')
            
            if controller_type not in ['proportional', 'wind_aware', 'return_to_home']:
                return jsonify({'success': False, 'message': 'Invalid controller type'}), 400
            
            # Create request with controller type in data field (non-standard Trigger usage)
            # Note: Trigger service doesn't have data field, so we need custom service type
            # For now, use topic publishing as fallback
            try:
                # Direct service call - controller.py needs to parse this
                if not self.controller_switch_client.wait_for_service(timeout_sec=1.0):
                    return jsonify({'success': False, 'message': 'Service unavailable'}), 503
                
                # Use Trigger service - pass type via separate parameter topic
                # Workaround: publish controller type to a topic, then call service
                # Better solution: define custom service type with string parameter
                
                # For now, use subprocess to call with ros2 cli
                cmd = [
                    'bash', '-c',
                    f'source /opt/ros/humble/setup.bash && '
                    f'ros2 service call /controller_node/switch_controller std_srvs/srv/Trigger'
                ]
                
                # TODO: This needs proper implementation with custom service type
                # For now, return not implemented
                return jsonify({
                    'success': False, 
                    'message': f'Controller switching to {controller_type} - needs implementation'
                }), 501
                
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)}), 500
        
        @self.app.route('/api/recording/start', methods=['POST'])
        def start_recording():
            """Start data recording."""
            return self._call_service(self.recording_start_client, '/argo/recording/start')
        
        @self.app.route('/api/recording/stop', methods=['POST'])
        def stop_recording():
            """Stop data recording."""
            return self._call_service(self.recording_stop_client, '/argo/recording/stop')
        
        @self.app.route('/api/lifecycle/start', methods=['POST'])
        def lifecycle_start():
            """Start Argo system via lifecycle manager."""
            try:
                # Use subprocess to call lifecycle manager
                result = subprocess.run([
                    'python3', 
                    os.path.join(self.argo_dir, 'launch', 'argo_lifecycle_manager.py'),
                    'run'
                ], capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0:
                    return jsonify({'success': True, 'message': 'System starting...'})
                else:
                    return jsonify({'success': False, 'message': 'Failed to start system'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)}), 500
        
        @self.app.route('/api/lifecycle/stop', methods=['POST'])
        def lifecycle_stop():
            """Stop Argo system via lifecycle manager."""
            try:
                result = subprocess.run([
                    'python3',
                    os.path.join(self.argo_dir, 'launch', 'argo_lifecycle_manager.py'),
                    'stop'
                ], capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    return jsonify({'success': True, 'message': 'System stopped'})
                else:
                    return jsonify({'success': False, 'message': 'Failed to stop system'}), 500
                    
            except Exception as e:
                return jsonify({'success': False, 'message': str(e)}), 500
    
    def _call_service(self, client, service_name):
        """Generic service call wrapper."""
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return jsonify({'success': False, 'message': 'Service unavailable'}), 503
            
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                return jsonify({'success': response.success, 'message': response.message})
            else:
                return jsonify({'success': False, 'message': 'Service call timeout'}), 504
                
        except Exception as e:
            return jsonify({'success': False, 'message': str(e)}), 500
    
    def run_flask(self):
        """Run Flask server in separate thread."""
        self.app.run(host='0.0.0.0', port=8081, debug=False, threaded=True, use_reloader=False)


def main(args=None):
    rclpy.init(args=args)
    node = ArgoWebDashboard()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down web dashboard...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

