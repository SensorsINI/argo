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
import yaml
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, Float32, String, Int32, UInt8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool

# Flask web server
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
from werkzeug.serving import ThreadedWSGIServer

# Add launch directory to path for ArgoNodeManager
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'launch'))
from argo_node_utils import ArgoNodeManager

# Add support directory to path for ArgoBaseNode
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

UPDATE_RATE = .2  # Hz

class ArgoWebDashboard(ArgoBaseNode):
    """ROS2 node providing web-based monitoring and control interface."""
    
    def __init__(self, debug_mode=False):
        super().__init__('argo_web_dashboard', enable_health_service=True, enable_health_publisher=True)
        self.debug_mode = debug_mode
        
        # Enable DEBUG logging if requested
        if self.debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Check for running web dashboard processes before starting
        self._check_for_running_dashboard()
        
        self.get_logger().info('Starting Argo Web Dashboard...')
        self.get_logger().debug('🔍 DEBUG mode enabled - verbose logging active')
 
        
        # Health monitoring - track data reception
        self.last_boat_data_received = 0
        self.boat_data_timeout = 10.0  # seconds - consider unhealthy if no boat data for 10s
        self.last_health_log_time = 0  # Track when we last logged health status
        self.last_logged_health_state = None  # Track last logged state (True=healthy, False=unhealthy)
        self.health_log_throttle_s = 30.0  # Only log unchanged health status every 30s
        
        # Service callback threading fix - use flag-based approach
        self.health_service_requested = False
        self.health_service_response = None
        
        # Add flags to trigger service calls from the main spin loop
        self._trigger_health_query = False
        self._trigger_battery_query = False
        
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
            'pcb_temp': None,
            'air_temp': None,
            
            # Node health (from health service)
            'nodes_healthy': 0,
            'nodes_unhealthy': 0,
            'nodes_expected_total': 0,  # Total from argo_nodes.yaml (includes excluded services)
            
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
        
        # Viewer activity tracking for CPU optimization
        self.last_viewer_request_time = time.time()  # Track last HTTP request
        self.viewer_timeout = 30.0  # Enter low-power mode after 30s of no requests
        self.low_power_mode = False  # Start in high power mode, but if no queries for some time, we unsubscribe aagin
        
        # QoS profile for subscriptions that should not receive stale data from transient_local publishers
        self.volatile_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.status_timer_period_active = 1/UPDATE_RATE  # 5 seconds when active
        self.status_timer_period_idle = 30.0  # 30 seconds when idle (low-power)
        self.health_timer_period_active = 1/UPDATE_RATE  # 5 seconds when active
        self.health_timer_period_idle = 60.0  # 60 seconds when idle (low-power)
        
        # Battery service query rate: 1/10 Hz (10 seconds) - only for local fallback when topics don't have data
        self.battery_service_query_period = 10.0  # 10 seconds (1/10 Hz)
        
        # Health service query rate: 1/5 Hz (5 seconds) - for node health counts (includes excluded services)
        self.health_service_query_period = 5.0  # 5 seconds (1/5 Hz)
        
        # Timestamps for each data type
        self.last_wifi_update = {}
        self.last_lora_update = {}
        
        # Initialize ArgoNodeManager for system status
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # ROS2 Service clients
        self.controller_pause_client = self.create_client(SetBool, '/controller_node/pause')
        self.recording_start_client = self.create_client(Trigger, '/argo/recording/start')
        self.recording_stop_client = self.create_client(Trigger, '/argo/recording/stop')
        self.controller_switch_client = self.create_client(Trigger, '/controller_node/switch_controller')
        self.battery_status_client = self.create_client(Trigger, '/battery_status')
        self.health_status_client = self.create_client(Trigger, '/argo/health/status')
        
        # Load node configuration from argo_nodes.yaml to know expected total count
        (self.physical_robot_nodes, 
         self.physical_robot_special_nodes, 
         self.all_nodes_including_excluded) = self._load_node_lists_from_yaml()
        self.state['nodes_total'] = len(self.physical_robot_nodes) + len(self.physical_robot_special_nodes)
        self.state['nodes_expected_total'] = len(self.all_nodes_including_excluded)
        
        # Store subscription references for mass unsubscribe/resubscribe
        # Use _topic_subscriptions to avoid conflict with ROS2 Node's subscriptions property
        # Start with empty list - subscriptions created lazily on first viewer access
        self._topic_subscriptions = []

        if not self.low_power_mode: # only create initial subscriptions if not in low-power mode, now default
            self._create_all_subscriptions()
        
        # Lazy subscriptions: Don't subscribe to topics until a viewer accesses the dashboard
        # This minimizes startup CPU usage when dashboard is not being used
        
        # Timer for periodic status updates (start with idle frequency since we're in low-power mode)
        # self.status_timer = self.create_timer(self.status_timer_period_idle, self.update_system_status)
        self.get_logger().debug("Created status timer")
        
        # Timer for periodic health status checks (start with idle frequency since we're in low-power mode)
        # self.health_timer = self.create_timer(self.health_timer_period_idle, self._check_health_status)
        
        # Timer for battery service queries (low rate: 1/10 Hz) - only for local fallback when topics don't have data
        # This runs independently of HTTP requests and at a low rate to minimize network traffic
        # self.battery_service_timer = self.create_timer(self.battery_service_query_period, self._trigger_battery_query_flag)
        
        # Timer for health service queries (low rate: 1/5 Hz) - for node health counts
        # This runs independently of HTTP requests and at a low rate to minimize network traffic
        # self.health_service_timer = self.create_timer(self.health_service_query_period, self._trigger_health_query_flag)
        
        # Timer to check viewer activity and adjust power mode - REMOVED, handled in main loop
        # self.viewer_activity_timer = self.create_timer(5.0, self._check_viewer_activity)
        
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
        self.wsgi_server = ThreadedWSGIServer(host='0.0.0.0', port=8081, app=self.app)
        
        # Start Flask in separate thread (daemon so it exits when main thread exits)
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()
        
        # Add signal handler for graceful Flask shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.get_logger().info('🌐 Web dashboard started on http://0.0.0.0:8081')
        self.get_logger().info('   Access from phone: http://ORANGEPI_IP:8081')
    
    def shutdown(self):
        """Custom shutdown logic for graceful termination."""
        if self.signal_received:
            self.get_logger().debug("Shutdown already requested - skipping")
            return
        self.signal_received = True
        self.get_logger().info("Initiating graceful shutdown of web dashboard...")
        self.set_unhealthy("Node shutting down")

        # Stop Flask server directly
        if hasattr(self, 'wsgi_server') and self.wsgi_server:
            try:
                self.get_logger().info("Shutting down Werkzeug WSGI server...")
                self.wsgi_server.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error shutting down Werkzeug server: {e}")
        else:
            self.get_logger().warn("Could not shut down Flask server: wsgi_server not found.")

        # Cancel all timers
        for timer_name in ['status_timer', 'health_timer', 'battery_service_timer', 'health_service_timer', 'viewer_activity_timer']:
            if hasattr(self, timer_name):
                try:
                    getattr(self, timer_name).cancel()
                except Exception:
                    pass
        
        # Give a moment for shutdown to propagate
        time.sleep(0.5)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully (only once)"""
        if self.signal_received:
            self.get_logger().debug("Signal handler called but shutdown already in progress.")
            return

        self.get_logger().info(f"Received signal {signum}, requesting shutdown...")
        # Setting this flag prevents re-entry
        self.signal_received = True

        # To break the main loop in main(), we must call rclpy.shutdown().
        # It's safest to do this from within the ROS context, so we create a short
        # one-shot timer to schedule the shutdown. This avoids race conditions.
        self.create_timer(1.0, lambda: rclpy.shutdown())
    
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
    
    # ==================== Subscription Management ====================
    
    def _create_all_subscriptions(self):
        """Create all ROS2 subscriptions and store references for mass unsubscribe/resubscribe."""
        # Clear existing subscriptions list
        self._topic_subscriptions = []
        
        # Controller pause state subscription
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/controller_pause_state', self.controller_pause_state_cb, 10)
        )
        
        # ROS2 Subscriptions for real-time data (WiFi sources)
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/human_controlled', lambda msg: self.human_control_cb(msg, 'wifi'), 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/battery_voltage', lambda msg: self.battery_voltage_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/battery_remaining_pct', lambda msg: self.battery_pct_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/charging_status', self.charging_status_cb, 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/ac_power_present', self.ac_power_cb, 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Vector3, '/compass', lambda msg: self.compass_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Vector3, '/pose', lambda msg: self.pose_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float64, '/gps_cog', lambda msg: self.gps_cog_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float64, '/gps_sog', lambda msg: self.gps_sog_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Vector3, '/anem_speed_angle_temp', lambda msg: self.wind_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(NavSatFix, '/fix', lambda msg: self.gps_fix_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(UInt8, '/gps_num_satellites', lambda msg: self.gps_satellites_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/temperature_pcb', self.pcb_temp_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/temperature_air', self.air_temp_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(String, '/controller_state', self.controller_state_cb, 10)
        )
        
        # LoRa sources (fallback when WiFi unavailable)
        self._topic_subscriptions.append(
            self.create_subscription(Bool, 'lora/human_controlled', lambda msg: self.human_control_cb(msg, 'lora'), 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float64, 'lora/battery_voltage', lambda msg: self.battery_voltage_cb(msg, 'lora'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Vector3, 'lora/compass', lambda msg: self.compass_cb(msg, 'lora'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float64, 'lora/gps_cog', lambda msg: self.gps_cog_cb(msg, 'lora'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float64, 'lora/gps_sog', lambda msg: self.gps_sog_cb(msg, 'lora'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(NavSatFix, 'lora/fix', lambda msg: self.gps_fix_cb(msg, 'lora'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(UInt8, 'lora/gps_num_satellites', lambda msg: self.gps_satellites_cb(msg, 'lora'), self.volatile_qos)
        )
        
        # LoRa-specific monitoring
        self._topic_subscriptions.append(
            self.create_subscription(Int32, 'lora/rssi', self.lora_rssi_cb, 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(String, 'lora/last_contact', self.lora_contact_cb, 10)
        )
    
    def _destroy_all_subscriptions(self):
        """Destroy all ROS2 subscriptions to stop receiving messages."""
        for sub in self._topic_subscriptions:
            try:
                self.destroy_subscription(sub)
            except Exception as e:
                self.get_logger().warn(f"Error destroying subscription: {e}")
        self._topic_subscriptions = []
    
    # ==================== ROS2 Callbacks ====================
    
    def human_control_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        self.get_logger().debug(f"Battery voltage: {msg.data}")
    
    def battery_pct_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['battery_pct'] = now
                self.state['battery_pct'] = msg.data
                self.state['data_source'] = 'WiFi'
            
            self._update_data_age_indicators()
        
        # Update health status - battery percentage is boat data
        self._update_boat_data_received(f"battery_pct_{source}")
    
    def charging_status_cb(self, msg):
        """Callback for battery charging status."""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['battery_charging'] = msg.data
            self.get_logger().debug(f"Battery charging status: {msg.data}")
    
    def ac_power_cb(self, msg):
        """Callback for AC/USB power present."""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['battery_usb_power'] = msg.data
    
    def pcb_temp_cb(self, msg):
        """Callback for PCB temperature."""
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['pcb_temp'] = msg.data
    
    def air_temp_cb(self, msg):
        """Callback for air temperature."""
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['air_temp'] = msg.data
    
    def compass_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return

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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['wind'] = now
                self.state['wind_speed'] = msg.x
                self.state['wind_angle'] = msg.y
                self.state['wind_temp'] = msg.z
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['wind'] = now
                wifi_age = now - self.last_wifi_update.get('wind', 0)
                if wifi_age > 2.0:
                    self.state['wind_speed'] = msg.x
                    self.state['wind_angle'] = msg.y
                    self.state['wind_temp'] = msg.z
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - wind data is boat data
        self._update_boat_data_received(f"wind_{source}")
    
    def gps_fix_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        # Skip processing in low-power mode (no viewers) - except home position detection
        # We still want to detect home position even in low-power mode
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
            
            # Set home position on first valid fix (always do this, even in low-power mode)
            if (self.state['home_latitude'] is None and 
                self.state['gps_latitude'] is not None):
                self.state['home_latitude'] = self.state['gps_latitude']
                self.state['home_longitude'] = self.state['gps_longitude']
                self.get_logger().info(
                    f"🏠 Home position set: {self.state['home_latitude']:.6f}°, "
                    f"{self.state['home_longitude']:.6f}°")
            
            # In low-power mode, skip remaining processing after home position check
            if self.low_power_mode:
                self._update_data_age_indicators()
                return
            
            # Calculate distance and bearing to home (skip expensive calculations in low-power mode)
            if (self.state['home_latitude'] is not None and 
                self.state['gps_latitude'] is not None):
                self.state['distance_to_home'] = self._calculate_distance(
                    self.state['gps_latitude'], self.state['gps_longitude'],
                    self.state['home_latitude'], self.state['home_longitude'])
                self.state['bearing_to_home'] = self._calculate_bearing(
                    self.state['gps_latitude'], self.state['gps_longitude'],
                    self.state['home_latitude'], self.state['home_longitude'])
            
            self._update_data_age_indicators()
        
        # Update health status - GPS fix data is boat data (skip in low-power mode)
        if not self.low_power_mode:
            self._update_boat_data_received(f"gps_fix_{source}")
    
    def gps_satellites_cb(self, msg, source='wifi'):
        """Unified callback for GPS satellite count"""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['lora_signal_strength'] = msg.data
    
    def lora_contact_cb(self, msg):
        """Receive LoRa last contact timestamp"""
        # Skip processing in low-power mode (no viewers)
        if self.low_power_mode:
            return
        
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
        
        # Update health status based on data reception (skip in low-power mode)
        if not self.low_power_mode:
            self._check_health_status()
        
        if self.debug_mode:
            self.get_logger().debug(f"Boat data received: {data_type} at {now} (age {now - self.last_boat_data_received:.2f}s)")
    
    def _check_viewer_activity(self):
        """Check viewer activity and adjust power mode accordingly."""
        now = time.time()
        time_since_last_request = now - self.last_viewer_request_time
        
        # Determine if we should be in low-power mode
        should_be_low_power = time_since_last_request > self.viewer_timeout
        
        # Only log transitions to avoid spam
        if should_be_low_power != self.low_power_mode:
            if should_be_low_power:
                self.get_logger().info(f"💤 Entering low-power mode (no viewers for {time_since_last_request:.1f}s)")
                self._enter_low_power_mode()
            else:
                self.get_logger().info("👁️  Exiting low-power mode (viewer activity detected)")
                self._exit_low_power_mode()
    
    def _enter_low_power_mode(self):
        """Enter low-power mode: unsubscribe from all topics and reduce timer frequencies."""
        self.low_power_mode = True
        
        # Destroy all subscriptions to eliminate callback overhead (if they exist)
        if self._topic_subscriptions:
            subscription_count = len(self._topic_subscriptions)
            self.get_logger().info(f"Unsubscribing from {subscription_count} topics")
            self._destroy_all_subscriptions()
        # If no subscriptions exist yet, we're already in the optimal state (lazy initialization)
        
        # Adjust status timer to run less frequently - REMOVED, handled in main loop
        # self.status_timer.cancel()
        # self.status_timer = self.create_timer(self.status_timer_period_idle, self.update_system_status)
        
        # Adjust health timer to run less frequently - REMOVED, handled in main loop
        # self.health_timer.cancel()
        # self.health_timer = self.create_timer(self.health_timer_period_idle, self._check_health_status)
    
    def _exit_low_power_mode(self, source_ip=None):
        """Exit low-power mode: subscribe/resubscribe to all topics and restore normal timer frequencies."""
        self.low_power_mode = False
        
        # Log exit with source IP if available
        if source_ip:
            self.get_logger().info(f"👁️  Exiting low-power mode (viewer activity detected from {source_ip})")
        else:
            self.get_logger().info("👁️  Exiting low-power mode (viewer activity detected)")
        
        # Create subscriptions if they don't exist yet (lazy initialization)
        # or recreate them if they were previously destroyed
        if not self._topic_subscriptions:
            self.get_logger().info("Subscribing to all topics (lazy initialization)")
            self._create_all_subscriptions()
        else:
            self.get_logger().info("Resubscribing to all topics")
            self._create_all_subscriptions()
        
        # Restore status timer to normal frequency - REMOVED, handled in main loop
        # self.status_timer.cancel()
        # self.status_timer = self.create_timer(self.status_timer_period_active, self.update_system_status)
        
        # Restore health timer to normal frequency - REMOVED, handled in main loop
        # self.health_timer.cancel()
        # self.health_timer = self.create_timer(self.health_timer_period_active, self._check_health_status)
        
        # Immediately update status and health when exiting low-power mode
        self.update_system_status()
        self._update_node_health_from_service()  # Trigger immediate health check
    
    def _record_viewer_activity(self, source_ip=None):
        """Record that a viewer made an HTTP request."""
        was_in_low_power = self.low_power_mode
        previous_request_time = self.last_viewer_request_time
        self.last_viewer_request_time = time.time()
        
        # If we were in low-power mode, exit it immediately
        if was_in_low_power:
            self._exit_low_power_mode(source_ip)
        elif self.debug_mode:
            # In debug mode, log all viewer activity for troubleshooting
            time_since_last = self.last_viewer_request_time - previous_request_time if previous_request_time > 0 else 0
            ip_info = f" from {source_ip}" if source_ip else ""
            self.get_logger().debug(f"Viewer activity detected{ip_info} (last request {time_since_last:.1f}s ago)")
    
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
        self.get_logger().debug("Updating system status")
        try:
            # In low-power mode, skip expensive operations
            if self.low_power_mode:
                # Only update timestamp, skip node queries and CPU temp
                with self.state_lock:
                    self.state['last_update'] = time.time()
                self.get_logger().debug("Skipping system status update in low-power mode")
                return
            
            # Get CPU temperature (file I/O - skip in low-power mode)
            self._update_cpu_temp()

            # Update node status (expensive operation - skip in low-power mode)
            node_status = self.node_manager.get_node_status()
            
            # Normalize node lists: strip .py extension from physical_robot_nodes for comparison
            # node_manager returns keys without .py (e.g., "gps"), but physical_robot_nodes has .py (e.g., "gps.py")
            normalized_physical_robot_nodes = {
                node.rstrip('.py') if node.endswith('.py') else node 
                for node in self.physical_robot_nodes
            }
            
            # --- REFACTORED: Filter status to only include nodes from the physical_robot group ---
            filtered_status = {
                node: info for node, info in node_status.items()
                if node in normalized_physical_robot_nodes or node in self.physical_robot_special_nodes
            }

            running_nodes = [node for node, info in filtered_status.items() if info.get('running', False)]
            
            with self.state_lock:
                self.state['nodes_running'] = len(running_nodes)
                self.state['nodes_total'] = len(self.physical_robot_nodes) + len(self.physical_robot_special_nodes)
                self.state['nodes_list'] = {
                    node: '🟢 RUNNING' if info.get('running', False) else '🔴 STOPPED'
                    for node, info in filtered_status.items()
                }
                self.state['system_running'] = len(running_nodes) > 0
            
            
            with self.state_lock:
                self.state['nodes_expected_total'] = len(self.all_nodes_including_excluded)
                self.state['last_update'] = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Error updating system status: {e}")
    
    def _update_cpu_temp(self):
        """Read CPU temperature from thermal zone."""
        try:
            with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                self.get_logger().debug(f"CPU temperature: {temp_millicelsius} millicelsius")   # DEBUG
                with self.state_lock:
                    self.state['cpu_temp'] = temp_millicelsius // 1000
        except Exception as e:
            self.get_logger().error(f"Error reading CPU temperature: {e}")
    
    def _load_node_lists_from_yaml(self) -> (list, list, list):
        """Load the list of nodes for the physical robot from argo_nodes.yaml.

        Returns:
            A tuple containing:
            - A list of regular nodes that are launched by the lifecycle manager.
            - A list of special nodes (e.g., foxglove_bridge) launched by the manager.
            - A list of all nodes, including those that run as excluded services.
        """
        try:
            config_path = os.path.join(self.argo_dir, 'launch', 'argo_nodes.yaml')
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            all_node_configs = {node['name']: node for node in config.get('nodes', [])}
            groups = config.get('groups', {})
            physical_robot_group_names = groups.get('physical_robot', [])

            launched_nodes = []
            special_nodes = []

            for name in physical_robot_group_names:
                if name in all_node_configs:
                    node_cfg = all_node_configs[name]
                    if node_cfg.get('special'):
                        special_nodes.append(name)
                    else:
                        script_name = os.path.basename(node_cfg.get('executable', ''))
                        launched_nodes.append(script_name)

            # Get the list of all nodes defined in the YAML, including excluded ones, for health checks
            all_defined_nodes = list(all_node_configs.keys())
            
            return launched_nodes, special_nodes, all_defined_nodes
        except Exception as e:
            self.get_logger().error(f"Could not load node lists from YAML: {e}")
            return [], [], []

    def _trigger_health_query_flag(self):
        """Timer callback to set the flag for a health query."""
        self._trigger_health_query = True

    def _trigger_battery_query_flag(self):
        """Timer callback to set the flag for a battery query."""
        self._trigger_battery_query = True

    def _load_expected_nodes_count(self) -> int:
        """DEPRECATED: This method is replaced by _load_node_lists_from_yaml."""
        return len(self.all_nodes_including_excluded)
    
    def _update_node_health_from_service(self):
        """Query health service to get healthy/unhealthy node counts.
        
        Uses /argo/health/status service which includes all nodes from argo_nodes.yaml,
        including excluded services (argo_power_control, argo_battery_water, bno085).
        Runs at low rate (1/5 Hz) independently of HTTP requests to minimize network traffic.
        """
        try:
            if not self.health_status_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().debug("Health status service not available")
                return  # Service not available, skip
            
            request = Trigger.Request()
            future = self.health_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    try:
                        health_data = json.loads(response.message)
                        nodes_data = health_data.get('nodes', {})
                        
                        # Count healthy and unhealthy nodes
                        healthy_count = 0
                        unhealthy_count = 0
                        
                        for node_name, node_info in nodes_data.items():
                            health_status = node_info.get('healthy')
                            if health_status is True:
                                healthy_count += 1
                            elif health_status is False:
                                unhealthy_count += 1
                        
                        with self.state_lock:
                            self.state['nodes_healthy'] = healthy_count
                            self.state['nodes_unhealthy'] = unhealthy_count
                            self.get_logger().debug(f"Updated health counts: {healthy_count} healthy, {unhealthy_count} unhealthy")
                    except (json.JSONDecodeError, KeyError) as e:
                        self.get_logger().warn(f"Error parsing health service response: {e}")
                else:
                    self.get_logger().debug(f"Health service returned success=False: {response.message}")
            else:
                self.get_logger().debug("Health service call timed out")
        except Exception as e:
            self.get_logger().warn(f"Error querying health service: {e}")
    
    def _update_battery_status_from_service(self):
        """Query battery status service at low rate (1/10 Hz) to get charging/USB power status.
        
        This runs as a fallback when topics don't have data. Only updates state if topic data is None.
        Runs independently of HTTP requests to minimize remote network traffic.
        """
        try:
            # Skip if in low-power mode (no viewers) - battery status not needed when dashboard not in use
            if self.low_power_mode:
                self.get_logger().debug("Skipping battery status update in low-power mode")
                return
            
            # Check if topic data is missing - only query service if needed
            with self.state_lock:
                charging_from_topic = self.state.get('battery_charging')
                usb_from_topic = self.state.get('battery_usb_power')
                voltage_from_topic = self.state.get('battery_voltage')
                pct_from_topic = self.state.get('battery_pct')
                pcb_temp_from_topic = self.state.get('pcb_temp')
            
            # Only query service if any topic data is missing (None)
            # Query if charging, USB power, voltage, pct, or PCB temp is missing
            if (charging_from_topic is not None and usb_from_topic is not None and 
                voltage_from_topic is not None and pct_from_topic is not None and 
                pcb_temp_from_topic is not None):
                self.get_logger().debug("Topics have all data, no need to query service")
                return  # Topics have all data, no need to query service
            
            if not self.battery_status_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().debug("Battery status service not available after 5 seconds")
                return  # Service not available, skip
            
            request = Trigger.Request()
            future = self.battery_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    try:
                        battery_data = json.loads(response.message)
                        raw_data = battery_data.get('raw_data', {})
                        charging_status = raw_data.get('charging_status')
                        usb_power_status = raw_data.get('ac_power_present')
                        battery_voltage = raw_data.get('battery_voltage')
                        battery_pct = raw_data.get('battery_remaining_pct')
                        pcb_temperature = raw_data.get('pcb_temperature')
                        
                        # Only update if we got valid data and topic data is still missing
                        # This prevents overriding topic data if it arrived between check and response
                        with self.state_lock:
                            self.get_logger().debug(f"Battery status response: {battery_data}")
                            if self.state.get('battery_charging') is None and charging_status is not None:
                                self.get_logger().debug(f"Updating battery charging status: {charging_status}")
                                self.state['battery_charging'] = charging_status
                            if self.state.get('battery_usb_power') is None and usb_power_status is not None:
                                self.state['battery_usb_power'] = usb_power_status
                            # Also update voltage and percentage if missing from topics
                            if self.state.get('battery_voltage') is None and battery_voltage is not None:
                                self.state['battery_voltage'] = battery_voltage
                            if self.state.get('battery_pct') is None and battery_pct is not None:
                                self.state['battery_pct'] = battery_pct
                            # Update PCB temperature if missing from topic
                            if self.state.get('pcb_temp') is None and pcb_temperature is not None:
                                self.state['pcb_temp'] = pcb_temperature
                    except (json.JSONDecodeError, KeyError) as e:
                        self.get_logger().debug(f"Error parsing battery status response: {e}")
        except Exception as e:
            # Silently fail - this is just a fallback, topics are primary source
            pass
    
    # ==================== Flask Routes ====================
    
    def setup_routes(self):
        """Setup Flask routes for web interface."""
        
        # Track viewer activity for all HTTP requests (except static files)
        @self.app.before_request
        def track_viewer_activity():
            """Track viewer activity to optimize CPU usage."""
            # Record activity for API endpoints and main page
            # Skip static files (images, CSS, JS) to avoid false positives
            if request.path.startswith('/api/') or request.path == '/':
                # Extract source IP address
                source_ip = request.remote_addr
                self._record_viewer_activity(source_ip)
        
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
                
        @self.app.route('/api/shutdown', methods=['POST'])
        def shutdown():
            """Internal endpoint to shut down the Flask server."""
            self.get_logger().info("Flask shutdown route called.")
            if hasattr(self, 'wsgi_server'):
                # Shutdown must be in a thread, as it blocks until the server is fully down
                shutdown_thread = threading.Thread(target=self.wsgi_server.shutdown)
                shutdown_thread.start()
                return jsonify({'success': True, 'message': 'Server shutting down...'})
            else:
                self.get_logger().error('Not running with a managed Werkzeug Server, cannot shutdown!')
                return jsonify({'success': False, 'message': 'Not running with managed Werkzeug server.'})
    
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
        self.get_logger().info("Starting Werkzeug WSGI server...")
        self.wsgi_server.serve_forever()
        self.get_logger().info("Werkzeug WSGI server has shut down.")


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
        # executor.spin()
        # Custom spin loop to handle service calls on the main thread
        # and manually trigger periodic tasks, as ROS2 timers are not reliable
        # with this custom executor model.
        
        last_status_update = time.time()
        last_health_check = time.time()
        last_viewer_check = time.time()
        last_health_service_query = time.time()
        last_battery_service_query = time.time()
        
        while rclpy.ok():
            # Spin once with a short timeout to handle ROS callbacks
            executor.spin_once(timeout_sec=0.1)
            
            now = time.time()
            
            # Determine current update intervals based on power mode
            status_interval = node.status_timer_period_idle if node.low_power_mode else node.status_timer_period_active
            health_interval = node.health_timer_period_idle if node.low_power_mode else node.health_timer_period_active
            viewer_check_interval = 5.0 # Constant 5s check
            
            # --- Manually trigger periodic tasks ---
            if now - last_status_update > status_interval:
                node.update_system_status()
                last_status_update = now
                
            if now - last_health_check > health_interval:
                node._check_health_status()
                last_health_check = now
            
            if now - last_viewer_check > viewer_check_interval:
                node._check_viewer_activity()
                last_viewer_check = now
            
            # --- Query health service for node counts (1/5 Hz = 5 seconds) ---
            if now - last_health_service_query > node.health_service_query_period:
                node._update_node_health_from_service()
                last_health_service_query = now
            
            # --- Query battery service as fallback (1/10 Hz = 10 seconds) ---
            if now - last_battery_service_query > node.battery_service_query_period:
                node._update_battery_status_from_service()
                last_battery_service_query = now
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down web dashboard...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Ensure proper cleanup
        if node:
            node.shutdown()
            node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
        print("🛑 Web dashboard shutdown complete.")


if __name__ == '__main__':
    main()
