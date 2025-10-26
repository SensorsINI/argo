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

# For complete usage, setup, and troubleshooting, see docs/WEB_DASHBOARD_README.md

Access: http://ORANGEPI_IP:8080
"""

import os
import sys
import json
import time
import subprocess
import threading
import argparse
import logging
import signal
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float32, String, Int32, UInt8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool

# Flask web server
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS

# Add launch directory to path for ArgoNodeManager
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'launch'))
from argo_node_utils import ArgoNodeManager

# Add support directory to path for ArgoBaseNode
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

UPDATE_RATE = 1  # Hz

class ArgoWebDashboard(ArgoBaseNode):
    """ROS2 node providing web-based monitoring and control interface."""
    
    def __init__(self, debug_mode=False):
        super().__init__('argo_web_dashboard', enable_health_service=True, enable_health_publisher=True)
        self.debug_mode = debug_mode
        
        # Check for running web dashboard processes before starting
        self._check_for_running_dashboard()
        
        self.get_logger().info('Starting Argo Web Dashboard...')
        
        # Health monitoring - track data reception
        self.last_boat_data_received = 0
        self.boat_data_timeout = 10.0  # seconds - consider unhealthy if no boat data for 10s
        self.last_health_log_time = 0  # Track when we last logged health status
        self.last_logged_health_state = None  # Track last logged state (True=healthy, False=unhealthy)
        self.health_log_throttle_s = 30.0  # Only log unchanged health status every 30s
        
        # Service callback threading fix - use flag-based approach
        self.health_service_requested = False
        self.health_service_response = None
        
        # Initialize state storage (thread-safe with lock)
        self.state_lock = threading.Lock()
        self.state = {
            # System status
            'nodes_running': 0,
            'nodes_total': 0,
            'nodes_list': {},
            'system_running': False,
            'controller_paused': False,
            
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
            
            # Data source tracking
            'data_source': 'WiFi',  # 'WiFi', 'LoRa', or 'Offline'
            'wifi_data_age': None,  # seconds since last WiFi update
            'lora_data_age': None,  # seconds since last LoRa update
            'lora_signal_strength': None,  # RSSI in dBm
            'lora_packet_loss_rate': None,  # percentage
            
            # Timestamps
            'last_update': time.time()
        }
        
        # Timestamps for each data type
        self.last_wifi_update = {}
        self.last_lora_update = {}
        
        # Initialize ArgoNodeManager for system status
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # ROS2 Service clients
        self.controller_pause_client = self.create_client(SetBool, '/controller_node/pause')
        self.controller_pause_state_sub = self.create_subscription(
            Bool, '/controller_pause_state', self.controller_pause_state_cb, 10)
        self.battery_status_client = self.create_client(Trigger, '/battery_status')
        self.recording_start_client = self.create_client(Trigger, '/argo/recording/start')
        self.recording_stop_client = self.create_client(Trigger, '/argo/recording/stop')
        self.controller_switch_client = self.create_client(Trigger, '/controller_node/switch_controller')
        
        # ROS2 Subscriptions for real-time data (WiFi sources)
        self.create_subscription(Bool, '/human_controlled', lambda msg: self.human_control_cb(msg, 'wifi'), 10)
        self.create_subscription(Float32, '/battery_voltage', lambda msg: self.battery_voltage_cb(msg, 'wifi'), 10)
        self.create_subscription(Float32, '/battery_remaining_pct', lambda msg: self.battery_pct_cb(msg, 'wifi'), 10)
        self.create_subscription(Vector3, '/compass', lambda msg: self.compass_cb(msg, 'wifi'), 10)
        self.create_subscription(Vector3, '/pose', lambda msg: self.pose_cb(msg, 'wifi'), 10)
        self.create_subscription(Float64, '/gps_cog', lambda msg: self.gps_cog_cb(msg, 'wifi'), 10)
        self.create_subscription(Float64, '/gps_sog', lambda msg: self.gps_sog_cb(msg, 'wifi'), 10)
        self.create_subscription(Vector3, '/anem_speed_angle_temp', lambda msg: self.wind_cb(msg, 'wifi'), 10)
        self.create_subscription(NavSatFix, '/fix', lambda msg: self.gps_fix_cb(msg, 'wifi'), 10)
        self.create_subscription(UInt8, '/gps_num_satellites', lambda msg: self.gps_satellites_cb(msg, 'wifi'), 10)
        self.create_subscription(String, '/controller_state', self.controller_state_cb, 10)
        
        # LoRa sources (fallback when WiFi unavailable)
        self.create_subscription(Bool, 'lora/human_controlled', lambda msg: self.human_control_cb(msg, 'lora'), 10)
        self.create_subscription(Float64, 'lora/battery_voltage', lambda msg: self.battery_voltage_cb(msg, 'lora'), 10)
        self.create_subscription(Vector3, 'lora/compass', lambda msg: self.compass_cb(msg, 'lora'), 10)
        self.create_subscription(Float64, 'lora/gps_cog', lambda msg: self.gps_cog_cb(msg, 'lora'), 10)
        self.create_subscription(Float64, 'lora/gps_sog', lambda msg: self.gps_sog_cb(msg, 'lora'), 10)
        self.create_subscription(NavSatFix, 'lora/fix', lambda msg: self.gps_fix_cb(msg, 'lora'), 10)
        self.create_subscription(UInt8, 'lora/gps_num_satellites', lambda msg: self.gps_satellites_cb(msg, 'lora'), 10)
        
        # LoRa-specific monitoring
        self.create_subscription(Int32, 'lora/rssi', self.lora_rssi_cb, 10)
        self.create_subscription(String, 'lora/last_contact', self.lora_contact_cb, 10)
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1/UPDATE_RATE, self.update_system_status)
        
        # Timer for periodic health status checks
        self.health_timer = self.create_timer(2.0, self._check_health_status)
        
        # Flask app setup
        self.app = Flask(__name__, 
                        template_folder=os.path.join(os.path.dirname(__file__), 'templates'),
                        static_folder=os.path.join(os.path.dirname(__file__), 'static'))
        CORS(self.app)
        self.setup_routes()
        
        # Configure Flask logging based on debug mode
        if not self.debug_mode:
            # Suppress Flask's default request logging to reduce journal clutter
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            # Also suppress Flask's own logging
            self.app.logger.setLevel(logging.ERROR)
        else:
            self.get_logger().info('🐛 Debug mode enabled - HTTP request logs will be shown')
            # Configure Flask to use our custom formatter with node name prefix
            self._configure_flask_logging()
        
        # Flask shutdown control
        self.flask_shutdown_requested = False
        self.signal_received = False  # Prevent recursive signal handling
        
        # Start Flask in separate thread (daemon so it exits when main thread exits)
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        # Add signal handler for graceful Flask shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.get_logger().info('🌐 Web dashboard started on http://0.0.0.0:8081')
        self.get_logger().info('   Access from phone: http://ORANGEPI_IP:8081')
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully (only once)"""
        # Prevent recursive signal handling
        if self.signal_received:
            return
        self.signal_received = True
        
        self.get_logger().info(f"Received signal {signum}, initiating shutdown...")
        self.set_unhealthy("Node shutting down")
        
        # Mark Flask as needing shutdown
        self.flask_shutdown_requested = True
        
        # Cancel timers to stop health check spam
        try:
            if hasattr(self, 'status_timer'):
                self.status_timer.cancel()
            if hasattr(self, 'health_timer'):
                self.health_timer.cancel()
        except Exception:
            pass
        
        # Trigger ROS2 shutdown to stop executor
        raise KeyboardInterrupt()
    
    def _check_for_running_dashboard(self):
        """Check for running web dashboard processes and refuse to start if found."""
        try:
            # Check for processes using port 8081 (more reliable than pgrep)
            result = subprocess.run([
                'lsof', '-ti:8081'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                self.get_logger().error(f"❌ Port 8081 is already in use!")
                self.get_logger().error(f"   Processes using port 8081: {', '.join(pids)}")
                self.get_logger().error(f"   Please stop the conflicting process first:")
                self.get_logger().error(f"   sudo kill {' '.join(pids)}")
                raise RuntimeError("Port 8081 already in use - conflict prevented")
            
            # Note: Port check is sufficient - if port 8081 is free, we can start
            # pgrep can find zombie processes, so we rely on port availability instead
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timeout checking for running processes - proceeding anyway")
        except FileNotFoundError:
            # pgrep or lsof not available, skip check
            if self.debug_mode:
                self.get_logger().debug("pgrep/lsof not available - skipping process check")
        except Exception as e:
            self.get_logger().error(f"Error checking for running processes: {e}")
            raise
    
    def _configure_flask_logging(self):
        """Configure Flask logging to include node name prefix."""
        # Use a simpler approach - just modify the format string
        node_prefix_format = '[argo_web_dashboard] %(message)s'
        
        # Configure werkzeug logger (handles HTTP requests)
        werkzeug_logger = logging.getLogger('werkzeug')
        # Remove existing handlers and add our custom one
        werkzeug_logger.handlers.clear()
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(node_prefix_format))
        werkzeug_logger.addHandler(handler)
        werkzeug_logger.setLevel(logging.INFO)
        
        # Configure Flask app logger
        self.app.logger.handlers.clear()
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(node_prefix_format))
        self.app.logger.addHandler(handler)
        self.app.logger.setLevel(logging.INFO)
    
    # ==================== ROS2 Callbacks ====================
    
    def human_control_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            # Always update if this is newer data or first data
            if source == 'wifi':
                self.last_wifi_update['human_controlled'] = now
                self.state['human_controlled'] = msg.data
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['human_controlled'] = now
                # Only use LoRa data if WiFi is stale (>2 seconds old)
                wifi_age = now - self.last_wifi_update.get('human_controlled', 0)
                if wifi_age > 2.0:
                    self.state['human_controlled'] = msg.data
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
    
    def battery_voltage_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['battery_voltage'] = now
                self.state['battery_voltage'] = msg.data
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['battery_voltage'] = now
                wifi_age = now - self.last_wifi_update.get('battery_voltage', 0)
                if wifi_age > 2.0:
                    self.state['battery_voltage'] = msg.data
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - battery voltage is boat data
        self._update_boat_data_received(f"battery_voltage_{source}")
    
    def battery_pct_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['battery_pct'] = now
                self.state['battery_pct'] = msg.data
                self.state['data_source'] = 'WiFi'
            
            self._update_data_age_indicators()
        
        # Update health status - battery percentage is boat data
        self._update_boat_data_received(f"battery_pct_{source}")
    
    def compass_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['compass_heading'] = now
                self.state['compass_heading'] = msg.z
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['compass_heading'] = now
                wifi_age = now - self.last_wifi_update.get('compass_heading', 0)
                if wifi_age > 2.0:
                    self.state['compass_heading'] = msg.z
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - compass heading is boat data
        self._update_boat_data_received(f"compass_{source}")
    
    def pose_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['compass_heading'] = now
                self.state['compass_heading'] = msg.z
                self.state['data_source'] = 'WiFi'
            
            self._update_data_age_indicators()
        
        # Update health status - pose data is boat data
        self._update_boat_data_received(f"pose_{source}")
    
    def gps_cog_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_cog'] = now
                self.state['gps_cog'] = msg.data
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['gps_cog'] = now
                wifi_age = now - self.last_wifi_update.get('gps_cog', 0)
                if wifi_age > 2.0:
                    self.state['gps_cog'] = msg.data
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - GPS COG is boat data
        self._update_boat_data_received(f"gps_cog_{source}")
    
    def gps_sog_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_sog'] = now
                self.state['gps_sog'] = msg.data
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['gps_sog'] = now
                wifi_age = now - self.last_wifi_update.get('gps_sog', 0)
                if wifi_age > 2.0:
                    self.state['gps_sog'] = msg.data
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - GPS SOG is boat data
        self._update_boat_data_received(f"gps_sog_{source}")
    
    def wind_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['wind'] = now
                self.state['wind_speed'] = msg.x
                self.state['wind_angle'] = msg.y
                self.state['wind_temp'] = msg.z
                self.state['data_source'] = 'WiFi'
            
            self._update_data_age_indicators()
        
        # Update health status - wind data is boat data
        self._update_boat_data_received(f"wind_{source}")
    
    def gps_fix_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_fix'] = now
                self.state['gps_locked'] = (msg.status.status >= 0)
                self.state['gps_latitude'] = msg.latitude if msg.latitude != 0.0 else None
                self.state['gps_longitude'] = msg.longitude if msg.longitude != 0.0 else None
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['gps_fix'] = now
                wifi_age = now - self.last_wifi_update.get('gps_fix', 0)
                if wifi_age > 2.0:
                    self.state['gps_locked'] = (msg.status.status >= 0)
                    self.state['gps_latitude'] = msg.latitude if msg.latitude != 0.0 else None
                    self.state['gps_longitude'] = msg.longitude if msg.longitude != 0.0 else None
                    self.state['data_source'] = 'LoRa'
            
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
            
            self._update_data_age_indicators()
        
        # Update health status - GPS fix data is boat data
        self._update_boat_data_received(f"gps_fix_{source}")
    
    def gps_satellites_cb(self, msg, source='wifi'):
        """Unified callback for GPS satellite count"""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_satellites'] = now
                self.state['gps_satellites'] = msg.data
                self.state['data_source'] = 'WiFi'
                if self.debug_mode:
                    self.get_logger().debug(f"GPS satellites (WiFi): {msg.data}")
            elif source == 'lora':
                self.last_lora_update['gps_satellites'] = now
                wifi_age = now - self.last_wifi_update.get('gps_satellites', 0)
                if wifi_age > 2.0:
                    self.state['gps_satellites'] = msg.data
                    self.state['data_source'] = 'LoRa'
                    if self.debug_mode:
                        self.get_logger().debug(f"GPS satellites (LoRa): {msg.data}")
            
            self._update_data_age_indicators()
        
        # Update health status - GPS satellite count is boat data
        self._update_boat_data_received(f"gps_satellites_{source}")
    
    def lora_rssi_cb(self, msg):
        """Receive LoRa signal strength"""
        with self.state_lock:
            self.state['lora_signal_strength'] = msg.data
    
    def lora_contact_cb(self, msg):
        """Receive LoRa last contact timestamp"""
        with self.state_lock:
            # Parse timestamp and calculate age
            try:
                contact_time = datetime.strptime(msg.data, "%Y-%m-%d %H:%M:%S").timestamp()
                self.state['lora_data_age'] = time.time() - contact_time
            except:
                pass
    
    def _update_data_age_indicators(self):
        """Update data age indicators for both WiFi and LoRa sources"""
        now = time.time()
        
        # Calculate WiFi data age
        if self.last_wifi_update:
            self.state['wifi_data_age'] = now - max(self.last_wifi_update.values())
        else:
            self.state['wifi_data_age'] = None
        
        # Calculate LoRa data age
        if self.last_lora_update:
            self.state['lora_data_age'] = now - max(self.last_lora_update.values())
        else:
            self.state['lora_data_age'] = None
        
        # Determine overall data source status
        if self.state['wifi_data_age'] is not None and self.state['wifi_data_age'] < 5.0:
            self.state['data_source'] = 'WiFi'
        elif self.state['lora_data_age'] is not None and self.state['lora_data_age'] < 15.0:
            self.state['data_source'] = 'LoRa'
        else:
            self.state['data_source'] = 'Offline'
    
    def _update_boat_data_received(self, data_type: str):
        """Update timestamp when boat data is received (not just human_controlled)"""
        now = time.time()
        self.last_boat_data_received = now
        
        # Update health status based on data reception
        self._check_health_status()
        
        if self.debug_mode:
            self.get_logger().debug(f"Boat data received: {data_type} at {now}")
    
    def _check_health_status(self):
        """Check health status based on boat data reception (with throttled logging)"""
        now = time.time()
        time_since_data = now - self.last_boat_data_received
        
        # Determine current health state
        is_healthy = time_since_data < self.boat_data_timeout
        
        # Check if we should log (state changed or enough time passed)
        state_changed = self.last_logged_health_state != is_healthy
        time_since_last_log = now - self.last_health_log_time
        should_log = state_changed or time_since_last_log >= self.health_log_throttle_s
        
        if is_healthy:
            # We have recent boat data - healthy
            data_age = f"{time_since_data:.1f}s ago"
            if should_log:
                self.set_healthy(f"Receiving boat data ({data_age})")
                self.last_health_log_time = now
                self.last_logged_health_state = True
            else:
                # Update health status silently (no logging)
                self.health_status = True
                self.health_details = f"Receiving boat data ({data_age})"
                self.last_health_update = now
        else:
            # No recent boat data - unhealthy
            data_age = f"{time_since_data:.1f}s ago"
            if should_log:
                self.set_unhealthy(f"No boat data received ({data_age})")
                self.last_health_log_time = now
                self.last_logged_health_state = False
            else:
                # Update health status silently (no logging)
                self.health_status = False
                self.health_details = f"No boat data received ({data_age})"
                self.last_health_update = now
    
    def controller_state_cb(self, msg):
        with self.state_lock:
            self.state['controller_type'] = msg.data
    
    def controller_pause_state_cb(self, msg):
        with self.state_lock:
            self.state['controller_paused'] = msg.data
    
    def _handle_health_request(self, request, response):
        """Override ArgoBaseNode health service callback with flag-based approach"""
        self.get_logger().info("🔥 HEALTH SERVICE CALLBACK CALLED!")
        try:
            # Set flag to indicate health service was requested
            self.health_service_requested = True
            
            # Create response data
            health_data = {
                'healthy': self.health_status,
                'details': self.health_details,
                'timestamp': self.last_health_update,
                'node_name': self.node_name
            }
            
            # Store response for Flask thread to use
            self.health_service_response = health_data
            
            # Return immediate response
            response.success = True
            response.message = json.dumps(health_data)
            
            self.get_logger().info(f"🔥 Health service response: {response.message}")
            return response
            
        except Exception as e:
            self.get_logger().error(f"🔥 Health service callback error: {e}")
            response.success = False
            response.message = f"Health check failed: {e}"
            return response
    
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
            """Toggle controller pause state."""
            try:
                # Get current pause state and toggle it
                current_paused = self.state.get('controller_paused', False)
                new_pause_state = not current_paused
                
                # Create request
                request = SetBool.Request()
                request.data = new_pause_state
                
                # Call service
                if not self.controller_pause_client.wait_for_service(timeout_sec=2.0):
                    return jsonify({'success': False, 'message': 'Controller pause service not available'})
                
                future = self.controller_pause_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    return jsonify({
                        'success': response.success,
                        'message': response.message,
                        'paused': new_pause_state
                    })
                else:
                    return jsonify({'success': False, 'message': 'Service call timed out'})
                    
            except Exception as e:
                return jsonify({'success': False, 'message': f'Error: {str(e)}'})
        
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
        self.app.run(host='0.0.0.0', port=8081, debug=self.debug_mode, threaded=True, use_reloader=False)


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Argo Web Dashboard - Mobile-friendly monitoring and control interface',
        epilog='''
Purpose:
  Provides a web-based interface for monitoring and controlling the Argo autonomous sailboat.
  Optimized for mobile devices with large touch targets and real-time status updates.

Features:
  - Real-time system monitoring (nodes, battery, GPS, sensors, wind)
  - Controller switching (Proportional, Wind-Aware, Return-to-Home)
  - System control (start/stop, pause, recording)
  - GPS tracking with satellite count and fix status
  - LoRa communication monitoring and fallback
  - Mobile-optimized interface for phone/tablet access
  - Health monitoring service and status publishing

Local Access:
  http://localhost:8081 or http://127.0.0.1:8081

Remote Access:
  http://ORANGEPI_IP:8081 (replace ORANGEPI_IP with actual IP address)
  
  To find the IP address:
    hostname -I
    or
    ip addr show | grep inet

Usage Examples:
  # Start with normal logging
  python3 argo_web_dashboard.py
  
  # Start with debug logging (shows HTTP requests)
  python3 argo_web_dashboard.py --debug
  
  # Run as ROS2 node
  ros2 run argo argo_web_dashboard --debug

Health Monitoring:
  - Service: ros2 service call /argo_web_dashboard/health std_srvs/srv/Trigger
  - Topic: ros2 topic echo /argo_web_dashboard_health
  - Health status based on boat data reception (not just human_controlled)

Troubleshooting:
  - If port 8081 is in use, the dashboard will refuse to start and show conflicting processes
  - Use --debug to see detailed HTTP request logs
  - Check ROS2 topics are publishing: ros2 topic list | grep -E "(gps|battery|compass)"
  - Verify system is running: ros2 node list
  - Check health status: ros2 service call /argo_web_dashboard/health std_srvs/srv/Trigger
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug mode (shows HTTP request logs and detailed topic data)')
    known_args, unknown_args = parser.parse_known_args(args)
    
    # Initialize ROS2
    rclpy.init(args=unknown_args)
    
    # Create node with MultiThreadedExecutor to handle service callbacks properly
    node = ArgoWebDashboard(debug_mode=known_args.debug)
    
    # Use MultiThreadedExecutor to handle service callbacks in separate threads
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down web dashboard...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Ensure proper cleanup
        try:
            node.destroy_node()
        except Exception:
            pass
            
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

