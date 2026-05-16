#!/usr/bin/env python3
"""
Argo Web Dashboard Node
========================

Mobile-friendly web interface for monitoring and controlling Argo sailboat.

Features:
- Real-time status monitoring (nodes, battery, GPS, sensors)
- Controller switching (Proportional, Wind-Aware, Return-to-Home)
- System control (start/stop, recording)
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
import re
import shutil
import tempfile
import yaml
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, Float32, String, Int32, UInt8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType

# Flask web server
from flask import Flask, render_template, jsonify, request, send_file, after_this_request
from flask_cors import CORS
from werkzeug.serving import ThreadedWSGIServer

# Add launch directory to path for ArgoNodeManager
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'launch'))
from argo_node_utils import ArgoNodeManager, get_service_node_names, is_health_monitored_node

# Add support directory to path for ArgoBaseNode
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode
from geofence_manager import GeofenceManager

UPDATE_RATE = .2  # Hz

class ArgoWebDashboard(ArgoBaseNode):
    """ROS2 node providing web-based monitoring and control interface."""
    
    def __init__(self, debug_mode=False):
        super().__init__('argo_web_dashboard', enable_health_service=True, enable_health_publisher=True)
        self.debug_mode = debug_mode
        self.mac_id = self._load_argo_mac_id()
        
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
        
        # GPS data staleness tracking
        self.last_gps_data_time = 0  # Timestamp of last GPS data received
        self.gps_data_timeout = 5.0  # Consider GPS data stale if no update for 5 seconds
        
        # Service callback threading fix - use flag-based approach
        self.health_service_requested = False
        self.health_service_response = None
        
        # Add flags to trigger service calls from the main spin loop
        self._trigger_health_query = False
        self._trigger_battery_query = False
        
        # Track previous human_controlled state for change detection
        self._prev_human_controlled_state = None
        
        # Restart progress tracking
        self.restart_in_progress = False
        self.restart_progress_messages = []
        self.restart_progress_lock = threading.Lock()
        
        # Initialize state storage (thread-safe with lock)
        self.state_lock = threading.Lock()
        self.state = {
            # System status
            'nodes_running': 0,
            'nodes_total': 0,
            'nodes_list': {},
            'system_running': False,
            # Battery status
            'battery_voltage': None,
            'battery_pct': None,
            'battery_charging': None,
            'battery_usb_power': None,
            'battery_time_to_full': None,
            'battery_low_alert': False,  # Low battery warning from argo_battery_water
            'battery_time_to_empty': None,
            'storage_rundown_active': False,  # astore: discharge to 7.6V then shut down
            'battery_charging_fault_detected': False,  # GPIO-based charging fault detection
            'battery_charging_fault_frequency': None,  # Frequency of blinking if fault detected
            'battery_mp2672_fault_summary': None,  # MP2672 fault summary (GPIO or I2C based)
            
            # Temperature
            'cpu_temp': None,
            'pcb_temp': None,
            'air_temp': None,
            
            # Humidity and saltwater sensor
            'relative_humidity': None,
            'saltwater_voltage': None,
            
            # Node health (from health service)
            'nodes_healthy': 0,
            'nodes_unhealthy': 0,
            'nodes_unknown': 0,  # Nodes with unknown/TBD health status
            'nodes_unhealthy_list': [],  # List of unhealthy node names
            'health_data_received': False,  # Track if health data has been received from service
            'nodes_expected_total': 0,  # Total health-monitored nodes (argo_nodes.yaml + services)
            
            # GPS
            'gps_locked': False,
            'gps_satellites': 0,  # Satellites in view (from gps.py publish_satellite_count)
            'gps_satellites_used': 0,  # Satellites used in navigation solution (from NavSatFix or parsed separately)
            'gps_snr_avg': 0.0,
            'gps_position_accuracy': None,  # Position accuracy in meters (from NavSatFix covariance)
            'gps_latitude': None,
            'gps_longitude': None,
            'gps_cog': None,
            'gps_sog': None,
            'gps_last_valid_latitude': None,  # Last valid GPS coordinates (preserved when fix is lost)
            'gps_last_valid_longitude': None,  # Last valid GPS coordinates (preserved when fix is lost)
            'gps_node_healthy': True,  # GPS node health status
            'gps_data_stale': False,  # GPS data is stale (no updates or node unhealthy)
            
            # Navigation
            'compass_heading': None,
            'imu_healthy': None,
            'imu_health_age': None,
            
            # Wind
            'wind_speed': None,
            'wind_angle': None,
            'wind_temp': None,
            
            # Controller
            'human_controlled': None,  # None = unknown until first message received
            'last_human_controlled_update': None,  # Timestamp of last /human_controlled message
            'controller_type': 'Unknown',
            
            # Recording
            'recording': False,
            
            # Critical failures
            'i2c_failure': False,  # Critical I2C bus failure status
            
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
        
        # QoS profile for subscriptions that should receive the last published value (transient_local)
        # Used for recording status so dashboard gets current state immediately when subscribing
        self.transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self.argo_yaml_path = os.path.join(self.argo_dir, 'nodes', 'argo.yaml')
        self.available_geofence_maps = self._load_available_geofence_maps()
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # ROS2 Service clients
        self.recording_start_client = self.create_client(Trigger, '/argo/recording/start')
        self.recording_stop_client = self.create_client(Trigger, '/argo/recording/stop')
        self.recording_get_status_client = self.create_client(Trigger, '/argo/recording/get_status')
        self.controller_switch_client = self.create_client(Trigger, '/controller_node/switch_controller')
        self.battery_status_client = self.create_client(Trigger, '/battery_status')
        self.health_status_client = self.create_client(Trigger, '/argo/health/status')
        self.power_shutdown_client = self.create_client(Trigger, '/argo/power/shutdown')
        
        # Parameter service client for setting controller type parameter
        # ROS2 nodes expose parameter services when start_parameter_services=True (default)
        self.controller_param_client = self.create_client(SetParameters, '/controller_node/set_parameters')
        
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
            # Query initial recording status after subscriptions are created
            # This ensures we get the current state even if the topic hasn't published yet
            self._query_recording_status()
        
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
        
        # Wait for Flask thread to finish (with timeout)
        if hasattr(self, 'flask_thread') and self.flask_thread.is_alive():
            self.get_logger().info("Waiting for Flask thread to finish...")
            self.flask_thread.join(timeout=2.0)
            if self.flask_thread.is_alive():
                self.get_logger().warn("Flask thread did not finish within timeout")
        
        # Cancel all timers (if any were created)
        for timer_name in ['status_timer', 'health_timer', 'battery_service_timer', 'health_service_timer', 'viewer_activity_timer']:
            if hasattr(self, timer_name):
                try:
                    getattr(self, timer_name).cancel()
                except Exception:
                    pass

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully (only once)"""
        if self.signal_received:
            self.get_logger().debug("Signal handler called but shutdown already in progress.")
            return

        self.get_logger().info(f"Received signal {signum}, requesting shutdown...")
        # Setting this flag prevents re-entry and allows main loop to break
        self.signal_received = True
        
        # Shutdown ROS2 context immediately to break any blocking operations
        # This will cause rclpy.ok() to return False, breaking the main loop
        try:
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().warn(f"Error shutting down ROS2 context: {e}")
    
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
        
        # ROS2 Subscriptions for real-time data (WiFi sources)
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/human_controlled', lambda msg: self.human_control_cb(msg, 'wifi'), 10)
        )
        self.get_logger().info('📡 Subscribed to /human_controlled topic (from rudder_sail_radio_node)')
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/battery_voltage', lambda msg: self.battery_voltage_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/battery_remaining_pct', lambda msg: self.battery_pct_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/battery_low_alert', self.battery_low_alert_cb, 10)
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
            self.create_subscription(UInt8, '/gps_num_satellites_used', lambda msg: self.gps_satellites_used_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/gps_snr_avg', lambda msg: self.gps_snr_cb(msg, 'wifi'), self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/temperature_pcb', self.pcb_temp_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/temperature_air', self.air_temp_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/relative_humidity', self.humidity_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Float32, '/saltwater_voltage', self.saltwater_voltage_cb, self.volatile_qos)
        )
        self._topic_subscriptions.append(
            self.create_subscription(String, '/controller_state', self.controller_state_cb, 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/argo/recording/status', self.recording_status_cb, self.transient_local_qos)
        )
        
        # Critical I2C failure monitoring
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/argo/critical/i2c_failure', self.i2c_failure_cb, 10)
        )
        self._topic_subscriptions.append(
            self.create_subscription(Bool, '/imu_health', self.imu_health_cb, self.volatile_qos)
        )
        
        # LoRa sources (fallback when WiFi unavailable) - only subscribe if lora_node is enabled
        if 'lora_node' in self.physical_robot_nodes:
            self.get_logger().info('📡 LoRa node enabled - subscribing to LoRa topics')
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
            self._topic_subscriptions.append(
                self.create_subscription(Float32, 'lora/gps_snr_avg', lambda msg: self.gps_snr_cb(msg, 'lora'), self.volatile_qos)
            )
            
            # LoRa-specific monitoring
            self._topic_subscriptions.append(
                self.create_subscription(Int32, 'lora/rssi', self.lora_rssi_cb, 10)
            )
            self._topic_subscriptions.append(
                self.create_subscription(String, 'lora/last_contact', self.lora_contact_cb, 10)
            )
        else:
            self.get_logger().info('📡 LoRa node disabled - skipping LoRa topic subscriptions')
    
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
            # Track previous state for change detection
            prev_state = self.state.get('human_controlled')
            new_state = msg.data
            
            # Always update if this is newer data or first data
            if source == 'wifi':
                self.last_wifi_update['human_controlled'] = now
                self.state['human_controlled'] = new_state
                self.state['last_human_controlled_update'] = now
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['human_controlled'] = now
                # Only use LoRa data if WiFi is stale (>2 seconds old)
                wifi_age = now - self.last_wifi_update.get('human_controlled', 0)
                if wifi_age > 2.0:
                    self.state['human_controlled'] = new_state
                    self.state['last_human_controlled_update'] = now
                    self.state['data_source'] = 'LoRa'
                else:
                    # Using WiFi data, don't update state
                    return
            
            # Log state changes with INFO level
            state_changed = prev_state is not None and prev_state != new_state
            if state_changed:
                mode_str = '👤 HUMAN' if new_state else '🤖 ROBOT'
                prev_mode_str = '👤 HUMAN' if prev_state else '🤖 ROBOT'
                self.get_logger().info(
                    f"Control mode changed: {prev_mode_str} → {mode_str} "
                    f"(source: {source.upper()}, topic: /human_controlled)"
                )
            elif prev_state is None:
                # First message received
                mode_str = '👤 HUMAN' if new_state else '🤖 ROBOT'
                self.get_logger().info(
                    f"Control mode initialized: {mode_str} "
                    f"(source: {source.upper()}, topic: /human_controlled)"
                )
            else:
                # State unchanged, log at debug level
                mode_str = '👤 HUMAN' if new_state else '🤖 ROBOT'
                self.get_logger().debug(
                    f"Control mode unchanged: {mode_str} "
                    f"(source: {source.upper()}, topic: /human_controlled)"
                )
            
            self._update_data_age_indicators()
        
        # Trigger immediate UI update for critical state change (no race condition - state already updated)
        if state_changed:
            self._trigger_critical_state_update()
    
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
    
    def battery_low_alert_cb(self, msg):
        """Callback for battery low alert."""
        # CRITICAL: Don't skip in low-power mode - battery warnings are safety-critical
        with self.state_lock:
            self.state['battery_low_alert'] = msg.data
    
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
    
    def humidity_cb(self, msg):
        """Callback for relative humidity."""
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['relative_humidity'] = msg.data
    
    def saltwater_voltage_cb(self, msg):
        """Callback for saltwater sensor voltage."""
        if self.low_power_mode:
            return
        
        with self.state_lock:
            self.state['saltwater_voltage'] = msg.data
    
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
        # /pose z is mathematical yaw (0°=East, CCW), same as simulator; convert for display.
        heading_math = float(msg.z) % 360.0
        compass_heading = (450.0 - heading_math) % 360.0
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['compass_heading'] = now
                self.state['compass_heading'] = compass_heading
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
                # Also update air_temp for UI display (temperature from anemometer)
                self.state['air_temp'] = msg.z
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['wind'] = now
                wifi_age = now - self.last_wifi_update.get('wind', 0)
                if wifi_age > 2.0:
                    self.state['wind_speed'] = msg.x
                    self.state['wind_angle'] = msg.y
                    self.state['wind_temp'] = msg.z
                    # Also update air_temp for UI display (temperature from anemometer)
                    self.state['air_temp'] = msg.z
                    self.state['data_source'] = 'LoRa'
            
            self._update_data_age_indicators()
        
        # Update health status - wind data is boat data
        self._update_boat_data_received(f"wind_{source}")
    
    def gps_fix_cb(self, msg, source='wifi'):
        """Unified callback that tracks source and timestamp"""
        # Skip processing in low-power mode (no viewers) - except home position detection
        # We still want to detect home position even in low-power mode
        now = time.time()
        
        # Update GPS data timestamp for staleness tracking
        self.last_gps_data_time = now
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_fix'] = now
                is_locked = (msg.status.status >= 0)
                self.state['gps_locked'] = is_locked
                
                # Only update coordinates if we have a valid fix
                if is_locked and msg.latitude != 0.0 and msg.longitude != 0.0:
                    self.state['gps_latitude'] = msg.latitude
                    self.state['gps_longitude'] = msg.longitude
                    # Store last valid coordinates for display when fix is lost
                    self.state['gps_last_valid_latitude'] = msg.latitude
                    self.state['gps_last_valid_longitude'] = msg.longitude
                # Don't clear coordinates when fix is lost - keep showing last valid position
                
                # Extract position accuracy from covariance (if available)
                if msg.position_covariance_type != 0:  # Not COVARIANCE_TYPE_UNKNOWN
                    var_east = msg.position_covariance[0]  # East-East variance
                    var_north = msg.position_covariance[4]  # North-North variance
                    
                    if var_east > 0 and var_north > 0:
                        # Calculate 95% confidence circle radius (2-sigma)
                        import math
                        std_dev = math.sqrt((var_east + var_north) / 2)
                        accuracy_95 = 2.0 * std_dev
                        self.state['gps_position_accuracy'] = accuracy_95
                
                self.state['data_source'] = 'WiFi'
            elif source == 'lora':
                self.last_lora_update['gps_fix'] = now
                wifi_age = now - self.last_wifi_update.get('gps_fix', 0)
                if wifi_age > 2.0:
                    is_locked = (msg.status.status >= 0)
                    self.state['gps_locked'] = is_locked
                    
                    # Only update coordinates if we have a valid fix
                    if is_locked and msg.latitude != 0.0 and msg.longitude != 0.0:
                        self.state['gps_latitude'] = msg.latitude
                        self.state['gps_longitude'] = msg.longitude
                        # Store last valid coordinates for display when fix is lost
                        self.state['gps_last_valid_latitude'] = msg.latitude
                        self.state['gps_last_valid_longitude'] = msg.longitude
                    # Don't clear coordinates when fix is lost - keep showing last valid position
                    
                    self.state['data_source'] = 'LoRa'
            
            # Set home position on first valid fix (always do this, even in low-power mode)
            # Use last valid coordinates if current coordinates are None
            current_lat_for_home = self.state['gps_latitude'] if self.state['gps_latitude'] is not None else self.state['gps_last_valid_latitude']
            current_lon_for_home = self.state['gps_longitude'] if self.state['gps_longitude'] is not None else self.state['gps_last_valid_longitude']
            if (self.state['home_latitude'] is None and 
                current_lat_for_home is not None):
                self.state['home_latitude'] = current_lat_for_home
                self.state['home_longitude'] = current_lon_for_home
                self.get_logger().info(
                    f"🏠 Home position set: {self.state['home_latitude']:.6f}°, "
                    f"{self.state['home_longitude']:.6f}°")
            
            # In low-power mode, skip remaining processing after home position check
            if self.low_power_mode:
                self._update_data_age_indicators()
                return
            
            # Calculate distance and bearing to home (skip expensive calculations in low-power mode)
            # Use current GPS coordinates if available, otherwise use last valid coordinates
            current_lat = self.state['gps_latitude'] if self.state['gps_latitude'] is not None else self.state['gps_last_valid_latitude']
            current_lon = self.state['gps_longitude'] if self.state['gps_longitude'] is not None else self.state['gps_last_valid_longitude']
            if (self.state['home_latitude'] is not None and 
                current_lat is not None):
                self.state['distance_to_home'] = self._calculate_distance(
                    current_lat, current_lon,
                    self.state['home_latitude'], self.state['home_longitude'])
                self.state['bearing_to_home'] = self._calculate_bearing(
                    current_lat, current_lon,
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
    
    def gps_satellites_used_cb(self, msg, source='wifi'):
        """Callback for GPS satellites used in navigation solution updates."""
        now = time.time()
        
        with self.state_lock:
            if source == 'wifi':
                self.last_wifi_update['gps_satellites_used'] = now
                self.state['gps_satellites_used'] = msg.data
                if self.debug_mode:
                    self.get_logger().debug(f"GPS satellites used (WiFi): {msg.data}")
            elif source == 'lora':
                self.last_lora_update['gps_satellites_used'] = now
                wifi_age = now - self.last_wifi_update.get('gps_satellites_used', 0)
                if wifi_age > 2.0:
                    self.state['gps_satellites_used'] = msg.data
                    if self.debug_mode:
                        self.get_logger().debug(f"GPS satellites used (LoRa): {msg.data}")
        
        self._update_boat_data_received(f"gps_satellites_used_{source}")
    
    def gps_snr_cb(self, msg, source='wifi'):
        """Callback for GPS average SNR updates."""
        now = time.time()
        
        if source == 'wifi':
            self.last_wifi_update['gps_snr'] = now
            self.state['gps_snr_avg'] = msg.data
            if self.debug_mode:
                self.get_logger().debug(f"GPS SNR (WiFi): {msg.data:.1f} dBHz")
        else:  # lora
            self.last_lora_update['gps_snr'] = now
            wifi_age = now - self.last_wifi_update.get('gps_snr', 0)
            if wifi_age > self.lora_fallback_threshold:
                self.state['gps_snr_avg'] = msg.data
                if self.debug_mode:
                    self.get_logger().debug(f"GPS SNR (LoRa): {msg.data:.1f} dBHz")
        
        self._update_boat_data_received(f"gps_snr_{source}")
    
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

    def imu_health_cb(self, msg):
        """Track IMU health status and data freshness for dashboard visibility."""
        if self.low_power_mode:
            return

        now = time.time()
        with self.state_lock:
            self.state['imu_healthy'] = bool(msg.data)
            self.last_wifi_update['imu_health'] = now
            self._update_data_age_indicators()
    
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

        # Calculate IMU health age from most recent /imu_health update
        imu_health_ts = self.last_wifi_update.get('imu_health')
        if imu_health_ts:
            self.state['imu_health_age'] = now - imu_health_ts
        else:
            self.state['imu_health_age'] = None
        
        # Determine overall data source status
        if self.state['wifi_data_age'] is not None and self.state['wifi_data_age'] < 5.0:
            self.state['data_source'] = 'WiFi'
        elif self.state['lora_data_age'] is not None and self.state['lora_data_age'] < 15.0:
            self.state['data_source'] = 'LoRa'
        else:
            self.state['data_source'] = 'Offline'
    
    def _trigger_critical_state_update(self):
        """Trigger immediate UI update for critical state changes.
        
        This is a no-op on the backend - the frontend polls /api/status/critical
        frequently to get reactive updates. We just log that state changed.
        """
        # State is already updated in the callback with proper locking
        # Frontend will pick it up via fast polling of /api/status/critical
        # No race condition because state update happened before this call
        pass
    
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
            viewer_idle = None
            if self.last_viewer_request_time:
                viewer_idle = now - self.last_viewer_request_time
            
            if self.low_power_mode:
                if viewer_idle is not None:
                    reason = f"Low-power mode: unsubscribed (no viewers for {viewer_idle:.1f}s; last boat data {data_age})"
                else:
                    reason = f"Low-power mode: unsubscribed from boat data (last boat data {data_age})"
            else:
                reason = f"No boat data received ({data_age})"
            
            if should_log:
                self.set_unhealthy(reason)
                self.last_health_log_time = now
                self.last_logged_health_state = False
            else:
                # Update health status silently (no logging)
                self.health_status = False
                self.health_details = reason
                self.last_health_update = now
    
    def controller_state_cb(self, msg):
        with self.state_lock:
            prev_controller = self.state.get('controller_type')
            self.state['controller_type'] = msg.data
            controller_changed = prev_controller != msg.data
        
        # Trigger immediate UI update for critical state change (no race condition - state already updated)
        if controller_changed:
            self._trigger_critical_state_update()
    
    def recording_status_cb(self, msg):
        """Callback for recording status updates."""
        if self.low_power_mode:
            return
        
        with self.state_lock:
            prev_recording = self.state.get('recording')
            self.state['recording'] = msg.data
            recording_changed = prev_recording != msg.data
        
        # Trigger immediate UI update for critical state change (no race condition - state already updated)
        if recording_changed:
            self._trigger_critical_state_update()
    
    def i2c_failure_cb(self, msg):
        """Callback for critical I2C failure status updates."""
        # Always process critical failures, even in low-power mode
        with self.state_lock:
            prev_i2c_failure = self.state.get('i2c_failure')
            self.state['i2c_failure'] = msg.data
            i2c_failure_changed = prev_i2c_failure != msg.data
        
        # Log only on state transitions (battery_water republishes True ~every 5s for late subscribers)
        if msg.data and i2c_failure_changed:
            self.get_logger().error(
                "🔴 CRITICAL I2C / ADC path failure (from /argo/critical/i2c_failure) — "
                "battery ADC monitoring unavailable; other I2C devices may still work. "
                "Controller should switch to RTH if configured.")
        elif i2c_failure_changed and not msg.data:
            self.get_logger().info("✅ I2C BUS RECOVERY - Critical sensors restored")
        
        # Trigger immediate UI update for critical state change
        if i2c_failure_changed:
            self._trigger_critical_state_update()
    
    def _query_recording_status(self):
        """Query the current recording status from the service to get initial state."""
        try:
            if not self.recording_get_status_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().debug("Recording status service not available, will wait for topic message")
                return
            
            request = Trigger.Request()
            future = self.recording_get_status_client.call_async(request)
            
            # Wait for response with timeout
            timeout = 3.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                if self.signal_received or not rclpy.ok():
                    break
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception:
                    if self.signal_received or not rclpy.ok():
                        break
            
            if future.done():
                try:
                    response = future.result()
                    with self.state_lock:
                        prev_recording = self.state.get('recording')
                        self.state['recording'] = response.success
                        if prev_recording != response.success:
                            self.get_logger().info(
                                f"📹 Initial recording status queried: {'ACTIVE' if response.success else 'INACTIVE'}"
                            )
                except Exception as e:
                    self.get_logger().debug(f"Error getting recording status response: {e}")
        except Exception as e:
            # Silently fail - topic subscription will provide the status
            self.get_logger().debug(f"Could not query recording status service: {e}")
    
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
    
    def _update_gps_staleness(self):
        """Check GPS data staleness - runs frequently even in low-power mode (safety-critical)."""
        now = time.time()
        gps_data_age = now - self.last_gps_data_time if self.last_gps_data_time > 0 else float('inf')
        gps_data_stale = (gps_data_age > self.gps_data_timeout)
        
        # Update GPS staleness in state (lightweight operation)
        with self.state_lock:
            self.state['gps_data_stale'] = gps_data_stale
    
    def update_system_status(self):
        """Periodically update system status (nodes, battery, CPU temp)."""
        self.get_logger().debug("Updating system status")
        try:
            # GPS staleness is now handled by _update_gps_staleness() in main loop
            
            # In low-power mode, skip expensive operations but keep GPS staleness updated above
            if self.low_power_mode:
                # Only update timestamp, skip node queries and CPU temp
                with self.state_lock:
                    self.state['last_update'] = time.time()
                self.get_logger().debug("Skipping system status update in low-power mode")
                return
            
            # Check for stale human_controlled messages
            # If we haven't received a message in 5 seconds (2s timeout + 3s buffer), assume robot control
            now = time.time()
            with self.state_lock:
                last_update = self.state.get('last_human_controlled_update')
                prev_state = self.state.get('human_controlled')
                if last_update is not None:
                    age = now - last_update
                    if age > 5.0:  # 5 seconds (2s timeout + 3s buffer)
                        if self.state['human_controlled']:
                            self.get_logger().info(
                                f"⚠️ Human control message stale ({age:.1f}s old), updating to ROBOT mode "
                                f"(last message from /human_controlled topic)"
                            )
                            self.state['human_controlled'] = False
                elif self.state['human_controlled']:
                    # If we've never received a message but state is True, check if topic is publishing
                    # If no message received for 5 seconds, assume robot control
                    self.get_logger().warn(
                        "⚠️ No human_controlled message received yet from /human_controlled topic, "
                        "checking if rudder_sail_radio_node is publishing"
                    )
                self._update_data_age_indicators()
            
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
            
            # Check GPS node health (already have data staleness from above)
            gps_node_running = filtered_status.get('gps_node', {}).get('running', False) or filtered_status.get('gps', {}).get('running', False)
            
            # Refine staleness: also mark stale if node is not running
            if not gps_node_running:
                gps_data_stale = True
            
            with self.state_lock:
                self.state['nodes_running'] = len(running_nodes)
                self.state['nodes_total'] = len(self.physical_robot_nodes) + len(self.physical_robot_special_nodes)
                self.state['nodes_list'] = {
                    node: '🟢 RUNNING' if info.get('running', False) else '🔴 STOPPED'
                    for node, info in filtered_status.items()
                }
                self.state['system_running'] = len(running_nodes) > 0
                
                # Update GPS node health and refined staleness
                self.state['gps_node_healthy'] = gps_node_running
                self.state['gps_data_stale'] = gps_data_stale
            
            
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
            - A list of health-monitored nodes (excluded systemd services; not disabled nodes).
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

            # Nodes tracked by health monitor (excluded systemd services yes; disabled nodes no)
            service_node_names = get_service_node_names(config)
            all_defined_nodes = [
                name for name, node_cfg in all_node_configs.items()
                if is_health_monitored_node(node_cfg, service_node_names)
            ]
            
            # Also include services from the services section (they're ROS2 nodes too)
            services_config = config.get('services', [])
            for service in services_config:
                service_name = service.get('name')
                if service_name and service_name not in all_defined_nodes:
                    # Add service to total count if not already in nodes section
                    all_defined_nodes.append(service_name)
            
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
                        
                        # Count healthy and unhealthy nodes, and collect unhealthy node names
                        # Note: Nodes with health_status=None are not counted (status unknown)
                        healthy_count = 0
                        unhealthy_count = 0
                        unknown_count = 0  # Track nodes with unknown health status
                        unhealthy_nodes = []
                        total_nodes_with_status = 0  # Total nodes with known status (healthy + unhealthy)
                        
                        for node_name, node_info in nodes_data.items():
                            health_status = node_info.get('healthy')
                            if health_status is True:
                                healthy_count += 1
                                total_nodes_with_status += 1
                            elif health_status is False:
                                unhealthy_count += 1
                                total_nodes_with_status += 1
                                # Store node name (remove .py extension for display)
                                display_name = node_name.rstrip('.py') if node_name.endswith('.py') else node_name
                                unhealthy_nodes.append(display_name)
                            else:
                                # health_status is None - status unknown/TBD
                                unknown_count += 1
                        
                        # Total expected nodes = nodes with known status + nodes with unknown status
                        # This matches what health monitor actually returns
                        total_expected = len(nodes_data)  # Total nodes returned by health monitor
                        
                        with self.state_lock:
                            self.state['nodes_healthy'] = healthy_count
                            self.state['nodes_unhealthy'] = unhealthy_count
                            self.state['nodes_unhealthy_list'] = unhealthy_nodes
                            self.state['nodes_unknown'] = unknown_count  # Track unknown health status
                            self.state['nodes_expected_total'] = total_expected  # Update from actual health service response
                            self.state['health_data_received'] = True  # Mark that we've received health data
                            self.get_logger().debug(f"Updated health counts: {healthy_count} healthy, {unhealthy_count} unhealthy, {unknown_count} unknown, {total_expected} total: {unhealthy_nodes}")
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
            
            # Check if topic data or time estimates are missing
            # Time estimates are ONLY available from service, not topics
            with self.state_lock:
                charging_from_topic = self.state.get('battery_charging')
                usb_from_topic = self.state.get('battery_usb_power')
                voltage_from_topic = self.state.get('battery_voltage')
                pct_from_topic = self.state.get('battery_pct')
                pcb_temp_from_topic = self.state.get('pcb_temp')
                time_to_full = self.state.get('battery_time_to_full')
                time_to_empty = self.state.get('battery_time_to_empty')
            
            # Always query service if time estimates are missing (topics don't provide these).
            # Also re-query when charging (from topics) disagrees with which estimate we have —
            # otherwise we skip the service forever while stale e.g. battery_time_to_full remains
            # after unplugging (needs_time_estimates would be false because TTF is still set).
            needs_time_estimates = (time_to_full is None and time_to_empty is None)
            if charging_from_topic is True:
                needs_time_estimates = needs_time_estimates or time_to_full is None or time_to_empty is not None
            elif charging_from_topic is False:
                needs_time_estimates = needs_time_estimates or time_to_empty is None or time_to_full is not None
            needs_other_data = (charging_from_topic is None or usb_from_topic is None or 
                               voltage_from_topic is None or pct_from_topic is None or 
                               pcb_temp_from_topic is None)
            
            if not needs_time_estimates and not needs_other_data:
                self.get_logger().debug("Topics have all data and time estimates available, skipping service query")
                return  # All data available, no need to query service
            
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
                        relative_humidity = raw_data.get('relative_humidity')
                        saltwater_voltage = raw_data.get('saltwater_voltage')
                        time_to_full = raw_data.get('time_to_full_hours')
                        time_to_empty = raw_data.get('time_to_empty_hours')
                        charging_fault_detected = raw_data.get('charging_fault_detected', False)
                        charging_fault_frequency = raw_data.get('charging_fault_frequency')
                        mp2672_fault_summary = raw_data.get('mp2672_fault_summary')
                        storage_rundown = raw_data.get('battery_storage_rundown', False)
                        # NOTE: I2C failure status comes ONLY from topic subscription (/argo/critical/i2c_failure)
                        # Service call timeout indicates battery service not running, which is a different error
                        
                        # Only update if we got valid data and topic data is still missing
                        # This prevents overriding topic data if it arrived between check and response
                        with self.state_lock:
                            # I2C failure state is managed ONLY by topic subscription (i2c_failure_cb)
                            # Do not update i2c_failure from service - service timeout is different error
                            self.get_logger().debug(f"Battery status response: {battery_data}")
                            # Always update charging status from service (service is authoritative)
                            if charging_status is not None:
                                old_charging = self.state.get('battery_charging')
                                self.state['battery_charging'] = charging_status
                                if old_charging != charging_status:
                                    self.get_logger().debug(f"Battery charging status changed: {old_charging} -> {charging_status}")
                            # Always update USB power status from service (service is authoritative)
                            if usb_power_status is not None:
                                old_usb = self.state.get('battery_usb_power')
                                self.state['battery_usb_power'] = usb_power_status
                                if old_usb != usb_power_status:
                                    self.get_logger().debug(f"Battery USB power status changed: {old_usb} -> {usb_power_status}")
                            # Update voltage and percentage from service (service is authoritative for consistency)
                            # Service provides consistent data with time estimates, so prefer service values
                            if battery_voltage is not None:
                                self.state['battery_voltage'] = battery_voltage
                            if battery_pct is not None:
                                self.state['battery_pct'] = battery_pct
                            # Update PCB temperature if missing from topic
                            if self.state.get('pcb_temp') is None and pcb_temperature is not None:
                                self.state['pcb_temp'] = pcb_temperature
                            # Update humidity if missing from topic
                            if self.state.get('relative_humidity') is None and relative_humidity is not None:
                                self.state['relative_humidity'] = relative_humidity
                            # Update saltwater voltage if missing from topic
                            if self.state.get('saltwater_voltage') is None and saltwater_voltage is not None:
                                self.state['saltwater_voltage'] = saltwater_voltage
                            # Always update fault information from service (topics don't provide this)
                            self.state['battery_charging_fault_detected'] = charging_fault_detected
                            self.state['battery_charging_fault_frequency'] = charging_fault_frequency
                            self.state['battery_mp2672_fault_summary'] = mp2672_fault_summary
                            # Storage rundown (astore): discharge to 7.6V then shut down
                            self.state['storage_rundown_active'] = storage_rundown
                            # Always update time estimates from service (topics don't provide this)
                            # CRITICAL: Always update (even if None) to clear stale values
                            # Respect charging status: if charging, clear TTE; if discharging, clear TTF
                            # Use state['battery_charging'] as authoritative (may be from topics), fallback to service value
                            authoritative_charging_status = self.state.get('battery_charging')
                            if authoritative_charging_status is None:
                                # If state doesn't have charging status, use service value
                                authoritative_charging_status = charging_status
                            
                            if authoritative_charging_status is True:
                                # Charging: use TTF, clear TTE
                                self.state['battery_time_to_full'] = time_to_full
                                self.state['battery_time_to_empty'] = None
                                if time_to_full is not None:
                                    self.get_logger().debug(f"Updated battery time to full: {time_to_full} hours")
                            elif authoritative_charging_status is False:
                                # Discharging: use TTE, clear TTF
                                self.state['battery_time_to_full'] = None
                                self.state['battery_time_to_empty'] = time_to_empty
                                if time_to_empty is not None:
                                    self.get_logger().debug(f"Updated battery time to empty: {time_to_empty} hours")
                            else:
                                # Unknown charging status: clear both to avoid showing wrong estimate
                                # Don't update time estimates if we don't know charging status
                                self.state['battery_time_to_full'] = None
                                self.state['battery_time_to_empty'] = None
                                self.get_logger().debug("Charging status unknown - cleared time estimates")
                    except (json.JSONDecodeError, KeyError) as e:
                        self.get_logger().debug(f"Error parsing battery status response: {e}")
        except Exception as e:
            # Silently fail - this is just a fallback, topics are primary source
            pass
    
    def _call_trigger_service_simple(self, service_name, timeout_sec=10.0):
        """Call a Trigger service and return (success: bool, message: str) tuple."""
        try:
            # Create a temporary client for this service call
            client = self.create_client(Trigger, service_name)
            if not client.wait_for_service(timeout_sec=min(5.0, timeout_sec)):
                return False, f'Service {service_name} unavailable'
            
            request = Trigger.Request()
            future = client.call_async(request)
            
            # Wait for response with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout_sec:
                if self.signal_received or not rclpy.ok():
                    return False, 'Shutdown requested'
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception:
                    if self.signal_received or not rclpy.ok():
                        return False, 'Context shutting down'
            
            if future.done():
                try:
                    response = future.result()
                    return response.success, response.message if response.message else 'Service call completed'
                except Exception as e:
                    return False, f'Error getting response: {str(e)}'
            else:
                return False, f'Service call timed out after {timeout_sec}s'
                
        except Exception as e:
            return False, f'Service call error: {str(e)}'
    
    def _run_restart_with_progress(self, reason: str = "manual restart"):
        """Run restart via argo_lifecycle_manager.py and capture progress messages."""
        def add_progress_message(msg, level='info'):
            """Add a progress message with timestamp."""
            timestamp = datetime.now().strftime('%H:%M:%S')
            with self.restart_progress_lock:
                self.restart_progress_messages.append({
                    'timestamp': timestamp,
                    'level': level,
                    'message': msg
                })
                # Keep only last 100 messages
                if len(self.restart_progress_messages) > 100:
                    self.restart_progress_messages = self.restart_progress_messages[-100:]
        
        try:
            add_progress_message(
                f"🔄 Starting Argo restart ({reason}, consistent with CLI 'ars' alias)...",
                'info'
            )
            
            # Use same approach as CLI 'ars' alias: stop then start
            # But first check if recording is active and stop it (like lifecycle manager restart() does)
            
            # Check recording status
            add_progress_message("Checking recording status...", 'info')
            try:
                success, message = self._call_trigger_service_simple('/argo/recording/get_status', timeout_sec=5.0)
                if success and message.lower() != 'not active':
                    add_progress_message("⚠️ Recording is active - stopping recording before restart...", 'warning')
                    success, message = self._call_trigger_service_simple('/argo/recording/stop', timeout_sec=10.0)
                    if success:
                        add_progress_message("✅ Recording stopped successfully", 'success')
                        time.sleep(2)  # Allow recording to fully stop
                    else:
                        add_progress_message(f"⚠️ Failed to stop recording: {message}", 'warning')
                        add_progress_message("⚠️ Proceeding with restart anyway...", 'warning')
            except Exception as e:
                add_progress_message(f"⚠️ Could not check recording status: {e}", 'warning')
            
            # Stop nodes (using same method as CLI 'aq' alias)
            add_progress_message("🛑 Stopping Argo nodes...", 'info')
            stop_script = os.path.join(self.argo_dir, 'launch', 'argo_stop_standard.sh')
            stop_result = subprocess.run(
                ['bash', stop_script],
                capture_output=True, text=True, timeout=30
            )
            
            # Parse stop output for progress messages (show all meaningful lines)
            if stop_result.stdout:
                for line in stop_result.stdout.split('\n'):
                    line = line.strip()
                    if line and not line.startswith('#'):  # Skip comments
                        # Determine level based on content
                        if any(emoji in line for emoji in ['✅', '🟢']):
                            add_progress_message(line, 'success')
                        elif any(emoji in line for emoji in ['❌', '🔴']):
                            add_progress_message(line, 'error')
                        elif any(emoji in line for emoji in ['⚠️', '🟡']):
                            add_progress_message(line, 'warning')
                        else:
                            add_progress_message(line, 'info')
            
            if stop_result.stderr:
                for line in stop_result.stderr.strip().split('\n'):
                    if line.strip():
                        add_progress_message(line.strip(), 'warning')
            
            if stop_result.returncode == 0:
                if not any('stopped' in msg['message'].lower() for msg in self.restart_progress_messages[-5:]):
                    add_progress_message("✅ Nodes stopped successfully", 'success')
            else:
                add_progress_message(f"⚠️ Stop command returned code {stop_result.returncode}", 'warning')
            
            time.sleep(1)  # Brief pause between stop and start
            
            # Start nodes (using same method as CLI 'al' alias)
            add_progress_message("🚀 Starting Argo nodes...", 'info')
            start_script = os.path.join(self.argo_dir, 'launch', 'argo_start_standard.sh')
            start_result = subprocess.run(
                ['bash', start_script],
                capture_output=True, text=True, timeout=90  # Start can take longer
            )
            
            # Parse start output for progress messages (show all meaningful lines)
            if start_result.stdout:
                for line in start_result.stdout.split('\n'):
                    line = line.strip()
                    if line and not line.startswith('#'):  # Skip comments
                        # Determine level based on content
                        if any(emoji in line for emoji in ['✅', '🟢']):
                            add_progress_message(line, 'success')
                        elif any(emoji in line for emoji in ['❌', '🔴']):
                            add_progress_message(line, 'error')
                        elif any(emoji in line for emoji in ['⚠️', '🟡']):
                            add_progress_message(line, 'warning')
                        elif any(emoji in line for emoji in ['⏳', '📊', '🔍']):
                            add_progress_message(line, 'info')
                        else:
                            add_progress_message(line, 'info')
            
            if start_result.stderr:
                for line in start_result.stderr.strip().split('\n'):
                    if line.strip():
                        add_progress_message(line.strip(), 'warning')
            
            if start_result.returncode == 0:
                if not any('started' in msg['message'].lower() or 'active' in msg['message'].lower() 
                          for msg in self.restart_progress_messages[-10:]):
                    add_progress_message("✅ Nodes started successfully", 'success')
            else:
                add_progress_message(f"❌ Start command returned code {start_result.returncode}", 'error')
                if start_result.stdout:
                    # Show last few lines of output for debugging
                    lines = [l.strip() for l in start_result.stdout.strip().split('\n') if l.strip()]
                    for line in lines[-5:]:
                        if line:
                            add_progress_message(line, 'error')
            
            # Final status
            add_progress_message("✅ Restart completed", 'success')
            
        except subprocess.TimeoutExpired:
            add_progress_message("❌ Restart timed out", 'error')
        except Exception as e:
            add_progress_message(f"❌ Error during restart: {str(e)}", 'error')
            self.get_logger().error(f"Restart error: {e}")
        finally:
            with self.restart_progress_lock:
                self.restart_in_progress = False

    def _start_background_restart(self, reason: str) -> Dict[str, Any]:
        """Start restart thread if no restart is currently running."""
        with self.restart_progress_lock:
            if self.restart_in_progress:
                return {'success': False, 'message': 'Restart already in progress', 'status_code': 409}
            self.restart_in_progress = True
            self.restart_progress_messages = []

        self.get_logger().info(f"Starting Argo restart via web dashboard ({reason})")

        restart_thread = threading.Thread(
            target=self._run_restart_with_progress,
            kwargs={'reason': reason},
            daemon=True
        )
        restart_thread.start()
        return {
            'success': True,
            'message': 'Restart started - use /api/lifecycle/restart/progress to monitor'
        }

    def _load_argo_mac_id(self) -> str:
        """Load Argo's frozen WiFi MAC ID from repo file (fallback to wlan0 MAC)."""
        try:
            repo_root = Path(os.path.dirname(os.path.dirname(__file__)))
            mac_file = repo_root / "network" / "ARGO_MAC_ID.txt"
            if mac_file.exists():
                for line in mac_file.read_text(encoding="utf-8").splitlines():
                    s = line.strip()
                    if not s or s.startswith("#"):
                        continue
                    return s.lower()
        except Exception:
            pass

        try:
            return (
                Path("/sys/class/net/wlan0/address")
                .read_text(encoding="utf-8")
                .strip()
                .lower()
            )
        except Exception:
            return "unknown"

    def _get_current_ip_address(self) -> Optional[str]:
        """Best-effort current IPv4 on wlan0 (computed per request)."""
        try:
            result = subprocess.run(
                ["ip", "-4", "addr", "show", "dev", "wlan0"],
                capture_output=True,
                text=True,
                timeout=1,
                check=False,
            )
            m = re.search(r"\binet\s+(\d+\.\d+\.\d+\.\d+)\b", result.stdout or "")
            return m.group(1) if m else None
        except Exception:
            return None

    def _maps_dir(self) -> Path:
        return Path(self.argo_dir) / "foxglove" / "maps"

    # Recording bags live under <argo>/bags (one folder per recording); same layout as nodes/record.py
    _BAG_FOLDER_NAME_RE = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]*$")

    def _bags_dir(self) -> Path:
        return Path(self.argo_dir) / "bags"

    def _resolve_safe_bag_folder(self, name: str) -> Optional[Path]:
        """Return path to a recording folder under bags/, or None if invalid or not a directory."""
        if not name or not self._BAG_FOLDER_NAME_RE.match(name.strip()):
            return None
        name = name.strip()
        root = self._bags_dir().resolve()
        candidate = (root / name).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return None
        if not candidate.is_dir():
            return None
        return candidate

    def _list_bag_recordings(self) -> Dict[str, Any]:
        """Subdirectories of bags/, newest first by filesystem mtime."""
        bags_root = self._bags_dir()
        if not bags_root.is_dir():
            return {"success": True, "recordings": []}
        recordings = []
        for p in bags_root.iterdir():
            if not p.is_dir():
                continue
            if not self._BAG_FOLDER_NAME_RE.match(p.name):
                continue
            try:
                st = p.stat()
                mtime = st.st_mtime
            except OSError:
                continue
            recordings.append({
                "name": p.name,
                "mtime": mtime,
                "mtime_iso": datetime.fromtimestamp(mtime).isoformat(timespec="seconds"),
            })
        recordings.sort(key=lambda x: x["mtime"], reverse=True)
        return {"success": True, "recordings": recordings}

    def _load_available_geofence_maps(self) -> list:
        """Load available geofence map names from foxglove/maps at dashboard startup."""
        maps_dir = self._maps_dir()
        if not maps_dir.exists():
            return []
        maps = [p.stem for p in maps_dir.glob("*.geojson")]
        maps.sort(key=lambda x: x.lower())
        return maps

    def _get_current_geofence_map_name(self) -> Optional[str]:
        """Read current geofence_map_name from nodes/argo.yaml."""
        try:
            with open(self.argo_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            root = (data.get("/**") or {}).get("ros__parameters") or {}
            map_name = root.get("geofence_map_name")
            if isinstance(map_name, str):
                normalized = GeofenceManager.normalize_map_name(map_name.strip())
                return normalized or None
        except Exception:
            pass

        # Fallback: line-based parse (handles malformed "name#" without space before comment)
        try:
            text = Path(self.argo_yaml_path).read_text(encoding="utf-8")
            match = re.search(r'^\s*geofence_map_name\s*:\s*([^\n#]+)', text, flags=re.MULTILINE)
            if match:
                normalized = GeofenceManager.normalize_map_name(
                    match.group(1).strip().strip("'\""))
                return normalized or None
        except Exception:
            pass
        return None

    def _geofence_map_inline_comment(self, active_map: str) -> str:
        """Build a YAML-safe inline comment listing other available maps."""
        others = [m for m in (self.available_geofence_maps or []) if m != active_map]
        if not others:
            return "geofence map"
        return " | ".join(others)

    def _format_geofence_map_yaml_line(self, map_name: str, indent: str) -> str:
        """Format geofence_map_name with required space before '#' (YAML comment)."""
        map_name = GeofenceManager.normalize_map_name(map_name)
        comment = self._geofence_map_inline_comment(map_name)
        return f"{indent}geofence_map_name: {map_name}  # {comment}"

    def _set_geofence_map_name(self, new_map_name: str) -> Dict[str, Any]:
        """Update geofence_map_name in nodes/argo.yaml with valid YAML comment spacing."""
        new_map_name = GeofenceManager.normalize_map_name((new_map_name or "").strip())
        if not new_map_name:
            return {"success": False, "message": "Map name is required"}

        available = set(self.available_geofence_maps or [])
        if new_map_name not in available:
            return {"success": False, "message": f"Unknown map: '{new_map_name}'"}

        try:
            yaml_path = Path(self.argo_yaml_path)
            original = yaml_path.read_text(encoding="utf-8")
            line_pattern = r'^(\s*)geofence_map_name\s*:.*$'
            match = re.search(line_pattern, original, flags=re.MULTILINE)
            if not match:
                return {"success": False, "message": "Could not find geofence_map_name in argo.yaml"}
            indent = match.group(1)
            new_line = self._format_geofence_map_yaml_line(new_map_name, indent)
            updated = re.sub(line_pattern, new_line, original, count=1, flags=re.MULTILINE)
            yaml_path.write_text(updated, encoding="utf-8")
            return {"success": True, "message": f"Updated geofence map to '{new_map_name}'"}
        except Exception as e:
            return {"success": False, "message": f"Failed to update argo.yaml: {e}"}

    # ==================== WiFi Management Helpers ====================

    def _extra_wifi_config_path(self) -> Path:
        # Keep this under the repo so both the dashboard (orangepi) and wifi_reconnect.sh (root) can read it.
        repo_root = Path(os.path.dirname(os.path.dirname(__file__)))
        return repo_root / "network" / "config" / "extra_preferred_networks.txt"

    def _run_nmcli(self, args, timeout_s: float = 8.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["nmcli", *args],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def _get_active_wifi_connection(self) -> Optional[str]:
        try:
            p = self._run_nmcli(["-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"], timeout_s=3.0)
            for line in (p.stdout or "").splitlines():
                parts = line.split(":")
                if len(parts) >= 3 and parts[1] == "802-11-wireless":
                    return parts[0].strip()
        except Exception:
            pass
        return None

    def _wifi_scan(self) -> Dict[str, Any]:
        # Rescan + list (does not switch connections)
        try:
            self.get_logger().info("📡 WiFi scan requested (rescan + list)")
        except Exception:
            pass
        rescan = self._run_nmcli(["dev", "wifi", "rescan"], timeout_s=8.0)
        if rescan.returncode != 0:
            try:
                self.get_logger().warning(
                    f"📡 WiFi rescan failed rc={rescan.returncode}: {(rescan.stderr or rescan.stdout or '').strip()}"
                )
            except Exception:
                pass
            return {"success": False, "message": (rescan.stderr or rescan.stdout or "rescan failed").strip()}

        p = self._run_nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list"], timeout_s=10.0)
        if p.returncode != 0:
            try:
                self.get_logger().warning(
                    f"📡 WiFi list failed rc={p.returncode}: {(p.stderr or p.stdout or '').strip()}"
                )
            except Exception:
                pass
            return {"success": False, "message": (p.stderr or p.stdout or "scan failed").strip()}

        seen = {}
        for line in (p.stdout or "").splitlines():
            # format: SSID:SIGNAL:SECURITY ; SSID can be empty (hidden)
            parts = line.split(":", 2)
            if not parts:
                continue
            ssid = (parts[0] or "").strip()
            if not ssid:
                continue
            signal = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            security = (parts[2] if len(parts) > 2 else "").strip()
            # de-dupe by SSID, keep best signal
            prev = seen.get(ssid)
            if prev is None or signal > prev["signal"]:
                seen[ssid] = {"ssid": ssid, "signal": signal, "security": security}

        networks = sorted(seen.values(), key=lambda x: (-x["signal"], x["ssid"].lower()))
        try:
            self.get_logger().info(f"📡 WiFi scan complete: {len(networks)} networks")
        except Exception:
            pass
        return {"success": True, "networks": networks}

    def _wifi_saved(self) -> Dict[str, Any]:
        active = self._get_active_wifi_connection()
        try:
            self.get_logger().info(f"📋 WiFi saved connections requested (active={active or 'none'})")
        except Exception:
            pass
        p = self._run_nmcli(["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY", "connection", "show"], timeout_s=6.0)
        if p.returncode != 0:
            try:
                self.get_logger().warning(
                    f"📋 WiFi saved list failed rc={p.returncode}: {(p.stderr or p.stdout or '').strip()}"
                )
            except Exception:
                pass
            return {"success": False, "message": (p.stderr or p.stdout or "nmcli failed").strip()}

        conns = []
        for line in (p.stdout or "").splitlines():
            parts = line.split(":")
            if len(parts) < 4:
                continue
            name, ctype, ac, pr = parts[0], parts[1], parts[2], parts[3]
            if ctype != "802-11-wireless":
                continue
            try:
                pr_i = int(pr) if pr != "" else 0
            except Exception:
                pr_i = 0
            conns.append({
                "name": name,
                "autoconnect": (ac.strip().lower() == "yes"),
                "autoconnect_priority": pr_i,
                "active": (active == name),
            })

        conns.sort(key=lambda c: (-int(c.get("autoconnect_priority") or 0), c["name"].lower()))
        return {"success": True, "connections": conns}

    def _append_extra_preferred_network(self, ssid: str) -> None:
        ssid = ssid.strip()
        if not ssid:
            return
        path = self._extra_wifi_config_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        existing = set()
        if path.exists():
            for line in path.read_text(encoding="utf-8").splitlines():
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                existing.add(s)
        if ssid not in existing:
            with path.open("a", encoding="utf-8") as f:
                if path.stat().st_size == 0:
                    f.write("# Extra preferred WiFi networks (appended after core list)\n")
                f.write(ssid + "\n")
            try:
                self.get_logger().info(f"📝 Added SSID to extra preferred list: {ssid} ({path})")
            except Exception:
                pass

    def _wifi_add(self, ssid: str, password: str, priority: int) -> Dict[str, Any]:
        ssid = (ssid or "").strip()
        if not ssid:
            return {"success": False, "message": "SSID is required"}

        # Enforce safety: never outrank hotspots
        # matebook 20, s24 15 are reserved top tiers
        if priority >= 15:
            priority = 5
        try:
            pw_info = "with password" if bool(password) else "open/empty password"
            self.get_logger().info(f"➕ WiFi add/update requested: ssid='{ssid}', priority={priority} ({pw_info})")
        except Exception:
            pass

        # Create or modify a connection profile named the same as SSID
        # We do NOT bring it up here.
        exists = self._run_nmcli(["connection", "show", ssid], timeout_s=3.0)
        if exists.returncode == 0:
            try:
                self.get_logger().info(f"➕ WiFi connection exists, modifying: {ssid}")
            except Exception:
                pass
            cmds = [
                ["connection", "modify", ssid, "connection.autoconnect", "yes"],
                ["connection", "modify", ssid, "connection.autoconnect-priority", str(priority)],
            ]
            if password:
                cmds += [
                    ["connection", "modify", ssid, "wifi-sec.key-mgmt", "wpa-psk"],
                    ["connection", "modify", ssid, "wifi-sec.psk", password],
                ]
            else:
                # open network: clear key-mgmt/psk if previously set (best effort)
                cmds += [
                    ["connection", "modify", ssid, "wifi-sec.key-mgmt", ""],
                    ["connection", "modify", ssid, "wifi-sec.psk", ""],
                ]
            for c in cmds:
                p = self._run_nmcli(c, timeout_s=6.0)
                if p.returncode != 0:
                    try:
                        self.get_logger().warning(
                            f"➕ nmcli failed rc={p.returncode} cmd={' '.join(c)} err={(p.stderr or p.stdout or '').strip()}"
                        )
                    except Exception:
                        pass
                    return {"success": False, "message": (p.stderr or p.stdout or "nmcli modify failed").strip()}
        else:
            # add new
            try:
                self.get_logger().info(f"➕ Creating new WiFi connection profile: {ssid}")
            except Exception:
                pass
            base = ["connection", "add", "type", "wifi", "con-name", ssid, "ifname", "wlan0", "ssid", ssid]
            if password:
                base += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password]
            base += ["connection.autoconnect", "yes", "connection.autoconnect-priority", str(priority)]
            p = self._run_nmcli(base, timeout_s=10.0)
            if p.returncode != 0:
                try:
                    self.get_logger().warning(
                        f"➕ nmcli add failed rc={p.returncode}: {(p.stderr or p.stdout or '').strip()}"
                    )
                except Exception:
                    pass
                return {"success": False, "message": (p.stderr or p.stdout or "nmcli add failed").strip()}

        # Ensure it participates in reconnect preference (without editing script)
        self._append_extra_preferred_network(ssid)
        try:
            self.get_logger().info(f"✅ WiFi add/update complete: {ssid} (priority {priority})")
        except Exception:
            pass
        return {"success": True, "message": f"Added/updated connection '{ssid}'", "priority": priority}

    def _wifi_switch_with_rollback(self, ssid: str, rollback_seconds: int = 60) -> Dict[str, Any]:
        ssid = (ssid or "").strip()
        if not ssid:
            return {"success": False, "message": "SSID is required"}

        current = self._get_active_wifi_connection()
        try:
            self.get_logger().warning(
                f"📶 WiFi switch requested: from='{current or 'none'}' to='{ssid}' (rollback {rollback_seconds}s)"
            )
        except Exception:
            pass

        # Rollback target: always prefer matebook then s24
        rollback_targets = ["tobi-matebook", "tobi-s24"]
        if current and current not in rollback_targets:
            rollback_targets.append(current)
        try:
            self.get_logger().info(f"📶 Rollback targets: {rollback_targets}")
        except Exception:
            pass

        rollback_event = threading.Event()

        def rollback_worker():
            try:
                if rollback_event.wait(timeout=rollback_seconds):
                    return  # canceled
                try:
                    self.get_logger().warning(
                        f"↩️ WiFi rollback triggered after {rollback_seconds}s (attempting {rollback_targets})"
                    )
                except Exception:
                    pass
                # Attempt rollback targets in order
                for target in rollback_targets:
                    p = self._run_nmcli(["connection", "up", target], timeout_s=30.0)
                    if p.returncode == 0:
                        try:
                            self.get_logger().warning(f"↩️ WiFi rollback succeeded: {target}")
                        except Exception:
                            pass
                        return
                    try:
                        self.get_logger().warning(
                            f"↩️ WiFi rollback failed rc={p.returncode} target={target}: {(p.stderr or p.stdout or '').strip()}"
                        )
                    except Exception:
                        pass
            except Exception:
                return

        t = threading.Thread(target=rollback_worker, daemon=True)
        t.start()

        # Attempt switch
        p = self._run_nmcli(["connection", "up", ssid], timeout_s=45.0)
        if p.returncode != 0:
            try:
                self.get_logger().warning(
                    f"📶 WiFi switch failed rc={p.returncode} ssid={ssid}: {(p.stderr or p.stdout or '').strip()}"
                )
            except Exception:
                pass
            return {"success": False, "message": (p.stderr or p.stdout or "switch failed").strip()}

        # Basic success: cancel rollback and return new IP (best effort)
        rollback_event.set()
        time.sleep(0.5)
        new_ip = self._get_current_ip_address()
        try:
            self.get_logger().warning(f"✅ WiFi switch succeeded: now on '{ssid}', ip={new_ip or 'unknown'}")
        except Exception:
            pass
        return {"success": True, "message": f"Switched to '{ssid}'", "new_ip": new_ip}
    
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
            return render_template(
                'dashboard.html',
                mac_id=self.mac_id,
                ip_address=self._get_current_ip_address(),
                current_wifi_ssid=self._get_active_wifi_connection() or '—',
                geofence_map_name=self._get_current_geofence_map_name() or '—',
            )
        
        @self.app.route('/api/status')
        def get_status():
            """Get current system status as JSON."""
            STORAGE_RUNDOWN_FLAG = Path('/tmp/argo_battery_storage_rundown')
            with self.state_lock:
                state_copy = self.state.copy()
                # Storage rundown is toggled via astore or dashboard; reflect current file so UI updates without battery service poll
                state_copy['storage_rundown_active'] = STORAGE_RUNDOWN_FLAG.exists()
                # Drop misleading time estimates when charging topic disagrees (stale until next service refresh)
                chg = state_copy.get('battery_charging')
                if chg is True:
                    state_copy['battery_time_to_empty'] = None
                elif chg is False:
                    state_copy['battery_time_to_full'] = None
                return jsonify(state_copy)
        
        @self.app.route('/api/status/critical')
        def get_critical_status():
            """Get only critical status fields (human_controlled, recording, controller_type, i2c_failure, battery_low_alert) for fast polling."""
            with self.state_lock:
                return jsonify({
                    'human_controlled': self.state.get('human_controlled'),
                    'recording': self.state.get('recording'),
                    'controller_type': self.state.get('controller_type'),
                    'i2c_failure': self.state.get('i2c_failure', False),
                    'battery_low_alert': self.state.get('battery_low_alert', False)
                })

        # ==================== WiFi Management API ====================

        @self.app.route('/api/wifi/scan', methods=['GET'])
        def wifi_scan():
            try:
                return jsonify(self._wifi_scan())
            except Exception as e:
                return jsonify({"success": False, "message": str(e)})

        @self.app.route('/api/wifi/saved', methods=['GET'])
        def wifi_saved():
            try:
                return jsonify(self._wifi_saved())
            except Exception as e:
                return jsonify({"success": False, "message": str(e)})

        @self.app.route('/api/wifi/add', methods=['POST'])
        def wifi_add():
            try:
                payload = request.get_json(force=True, silent=True) or {}
                ssid = payload.get("ssid", "")
                password = payload.get("password", "") or ""
                try:
                    priority = int(payload.get("priority", 5))
                except Exception:
                    priority = 5
                return jsonify(self._wifi_add(ssid=ssid, password=password, priority=priority))
            except Exception as e:
                return jsonify({"success": False, "message": str(e)})

        @self.app.route('/api/wifi/switch', methods=['POST'])
        def wifi_switch():
            try:
                payload = request.get_json(force=True, silent=True) or {}
                ssid = payload.get("ssid", "")
                # This can disconnect the client; rollback is scheduled locally.
                return jsonify(self._wifi_switch_with_rollback(ssid=ssid, rollback_seconds=60))
            except Exception as e:
                return jsonify({"success": False, "message": str(e)})

        @self.app.route('/api/geofence/maps', methods=['GET'])
        def geofence_maps():
            try:
                return jsonify({
                    "success": True,
                    "maps": self.available_geofence_maps,
                    "current_map": self._get_current_geofence_map_name(),
                    "current_wifi_ssid": self._get_active_wifi_connection(),
                })
            except Exception as e:
                return jsonify({"success": False, "message": str(e)})

        @self.app.route('/api/geofence/map', methods=['POST'])
        def geofence_set_map():
            try:
                payload = request.get_json(force=True, silent=True) or {}
                map_name = (payload.get("map_name", "") or "").strip()
                with self.restart_progress_lock:
                    if self.restart_in_progress:
                        return jsonify({
                            "success": False,
                            "message": "Cannot change map while restart is in progress"
                        }), 409
                result = self._set_geofence_map_name(map_name)
                if not result.get("success"):
                    return jsonify(result), 400

                restart_result = self._start_background_restart(
                    reason=f"map change to '{map_name}'"
                )
                status_code = restart_result.pop('status_code', 200)
                return jsonify({
                    "success": restart_result.get("success", False),
                    "message": (
                        f"{result.get('message')}. Restart triggered for map-dependent nodes."
                        if restart_result.get("success", False)
                        else f"{result.get('message')}. {restart_result.get('message', '')}"
                    ),
                    "current_map": map_name,
                }), status_code
            except Exception as e:
                return jsonify({"success": False, "message": str(e)}), 500

        @self.app.route('/api/bags', methods=['GET'])
        def bags_list():
            """List recording folders under bags/ (newest first)."""
            try:
                return jsonify(self._list_bag_recordings())
            except Exception as e:
                return jsonify({"success": False, "message": str(e), "recordings": []}), 500

        @self.app.route('/api/bags/download', methods=['GET'])
        def bags_download():
            """Zip one recording folder and send as attachment."""
            name = (request.args.get('name') or '').strip()
            folder = self._resolve_safe_bag_folder(name)
            if not folder:
                return jsonify({"success": False, "message": "Invalid or unknown recording folder"}), 400

            bags_root = self._bags_dir().resolve()
            fd, tmp_zip = tempfile.mkstemp(suffix='.zip', prefix='argo_bag_dl_')
            os.close(fd)
            try:
                os.unlink(tmp_zip)
            except OSError:
                pass
            archive_base = tmp_zip[:-4]
            try:
                archive_path = shutil.make_archive(
                    archive_base,
                    'zip',
                    root_dir=str(bags_root),
                    base_dir=name,
                )
            except Exception as e:
                return jsonify({"success": False, "message": str(e)}), 500

            @after_this_request
            def _cleanup_zip(response):
                try:
                    os.unlink(archive_path)
                except OSError:
                    pass
                return response

            return send_file(
                archive_path,
                as_attachment=True,
                download_name=f'{name}.zip',
                mimetype='application/zip',
            )

        @self.app.route('/api/controllers', methods=['GET'])
        def get_controllers():
            """Get list of available controllers from controller.py."""
            controllers = [
                {
                    'type': 'proportional',
                    'display_name': 'Proportional',
                    'icon': '🎯',
                    'description': 'Simple proportional heading control that maintains last human heading'
                },
                {
                    'type': 'wind_aware',
                    'display_name': 'Wind Aware',
                    'icon': '🌬️',
                    'description': 'Enhanced last-human heading control with wind-based sail adjustment'
                },
                {
                    'type': 'return_to_home',
                    'display_name': 'Return to Home',
                    'icon': '🏠',
                    'description': 'GPS-based navigation to return to Home position'
                },
                {
                    'type': 'crosser',
                    'display_name': 'Crosser',
                    'icon': '↔️',
                    'description': 'Crosses geofence area from side to side, targeting middle waypoint or center of sailing area'
                },
                {
                    'type': 'human',
                    'display_name': 'Human',
                    'icon': '👤',
                    'description': 'Manual control via keyboard/radio commands (pass-through mode)'
                }
            ]
            return jsonify({'success': True, 'controllers': controllers})
        
        @self.app.route('/api/controller/switch', methods=['POST'])
        def switch_controller():
            """Switch controller type using ROS2 parameters."""
            data = request.get_json()
            controller_type = data.get('type', '')
            
            # Valid controller types from controller.py _on_parameters_set callback
            valid_types = ['proportional', 'wind_aware', 'return_to_home', 'crosser', 'human']
            if controller_type not in valid_types:
                return jsonify({'success': False, 'message': f'Invalid controller type. Valid types: {", ".join(valid_types)}'}), 400
            
            try:
                # Check if parameter service is available
                if not self.controller_param_client.wait_for_service(timeout_sec=2.0):
                    return jsonify({'success': False, 'message': 'Controller parameter service unavailable'}), 503
                
                # Set the controller_type parameter using parameter service
                # The parameter change callback will automatically switch the controller
                param_msg = ParameterMsg()
                param_msg.name = 'controller_type'
                param_msg.value = ParameterValue()
                param_msg.value.type = ParameterType.PARAMETER_STRING
                param_msg.value.string_value = controller_type
                
                set_param_request = SetParameters.Request()
                set_param_request.parameters = [param_msg]
                
                set_future = self.controller_param_client.call_async(set_param_request)
                rclpy.spin_until_future_complete(self, set_future, timeout_sec=2.0)
                
                if not set_future.done():
                    return jsonify({
                        'success': False,
                        'message': 'Parameter set operation timed out'
                    }), 504
                
                set_result = set_future.result()
                if not set_result.results or not set_result.results[0].successful:
                    reason = set_result.results[0].reason if set_result.results else "Unknown error"
                    return jsonify({
                        'success': False,
                        'message': f'Failed to set parameter: {reason}'
                    }), 500
                
                # Parameter change callback will automatically switch the controller
                # No need to call the service - the parameter change triggers the switch
                # The parameter service result already indicates if the switch was successful
                if set_result.results[0].successful:
                    return jsonify({
                        'success': True,
                        'message': f'Controller switched to {controller_type} (parameter change callback handled the switch)'
                    })
                else:
                    return jsonify({
                        'success': False,
                        'message': f'Failed to switch controller: {set_result.results[0].reason}'
                    }), 500
                    
            except Exception as e:
                self.get_logger().error(f"Error switching controller: {e}")
                return jsonify({'success': False, 'message': str(e)}), 500
        
        @self.app.route('/api/recording/toggle', methods=['POST'])
        def toggle_recording():
            """Toggle recording state (start if stopped, stop if started)."""
            try:
                # First, query the actual recording status to avoid race conditions
                # This ensures we have the most up-to-date state before toggling
                if self.recording_get_status_client.wait_for_service(timeout_sec=2.0):
                    request = Trigger.Request()
                    future = self.recording_get_status_client.call_async(request)
                    
                    # Wait for response with short timeout
                    timeout = 2.0
                    start_time = time.time()
                    while not future.done() and (time.time() - start_time) < timeout:
                        if not rclpy.ok():
                            break
                        try:
                            rclpy.spin_once(self, timeout_sec=0.1)
                        except Exception:
                            if not rclpy.ok():
                                break
                    
                    if future.done():
                        try:
                            response = future.result()
                            is_recording = response.success
                            # Update state with actual status
                            with self.state_lock:
                                self.state['recording'] = is_recording
                        except Exception:
                            # Fall back to state dictionary if service call fails
                            with self.state_lock:
                                is_recording = self.state.get('recording', False)
                    else:
                        # Fall back to state dictionary if service call times out
                        with self.state_lock:
                            is_recording = self.state.get('recording', False)
                else:
                    # Fall back to state dictionary if service is not available
                    with self.state_lock:
                        is_recording = self.state.get('recording', False)
                
                # Choose appropriate client and service name based on current state
                if is_recording:
                    client = self.recording_stop_client
                    service_name = '/argo/recording/stop'
                else:
                    client = self.recording_start_client
                    service_name = '/argo/recording/start'
                
                return self._call_service(client, service_name)
            except Exception as e:
                return jsonify({
                    'success': False, 
                    'message': f'Error toggling recording: {str(e)}'
                }), 500
        
        # Keep old endpoints for backwards compatibility
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
        
        @self.app.route('/api/lifecycle/toggle', methods=['POST'])
        def toggle_lifecycle():
            """Toggle Argo system lifecycle (start if stopped, stop if started)."""
            try:
                # Get current system state
                with self.state_lock:
                    is_running = self.state.get('system_running', False)
                
                # Choose appropriate command based on current state
                if is_running:
                    command = 'stop'
                    action = 'stopping'
                else:
                    command = 'run'
                    action = 'starting'
                
                result = subprocess.run([
                    'python3',
                    os.path.join(self.argo_dir, 'launch', 'argo_lifecycle_manager.py'),
                    command
                ], capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    return jsonify({
                        'success': True, 
                        'message': f'System {action}...'
                    })
                else:
                    error_msg = result.stderr.strip() if result.stderr else 'Unknown error'
                    return jsonify({
                        'success': False, 
                        'message': f'Failed to {action} system: {error_msg}'
                    }), 500
                    
            except Exception as e:
                return jsonify({
                    'success': False, 
                    'message': f'Error toggling system: {str(e)}'
                }), 500
        
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
        
        @self.app.route('/api/lifecycle/restart', methods=['POST'])
        def lifecycle_restart():
            """Restart Argo nodes using argo_lifecycle_manager.py restart method (consistent with CLI 'ars' alias)."""
            try:
                restart_result = self._start_background_restart(reason="manual dashboard request")
                status_code = restart_result.pop('status_code', 200)
                return jsonify(restart_result), status_code
                    
            except Exception as e:
                with self.restart_progress_lock:
                    self.restart_in_progress = False
                self.get_logger().error(f"Error starting restart: {e}")
                return jsonify({'success': False, 'message': f'Error: {str(e)}'}), 500
        
        @self.app.route('/api/lifecycle/restart/progress', methods=['GET'])
        def lifecycle_restart_progress():
            """Get restart progress messages."""
            with self.restart_progress_lock:
                return jsonify({
                    'in_progress': self.restart_in_progress,
                    'messages': self.restart_progress_messages.copy()
                })
                
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
        
        @self.app.route('/api/system/shutdown', methods=['POST'])
        def system_shutdown():
            """Shutdown the entire system via power control service."""
            self.get_logger().info("System shutdown requested from web dashboard")
            return self._call_service(self.power_shutdown_client, '/argo/power/shutdown')
        
        @self.app.route('/api/storage/toggle', methods=['POST'])
        def storage_toggle():
            """Toggle storage rundown mode: create flag file if missing (activate), remove if present (deactivate). Mode cleared on reboot."""
            STORAGE_RUNDOWN_FLAG = Path('/tmp/argo_battery_storage_rundown')
            try:
                if STORAGE_RUNDOWN_FLAG.exists():
                    STORAGE_RUNDOWN_FLAG.unlink()
                    with self.state_lock:
                        self.state['storage_rundown_active'] = False
                    self.get_logger().info("Storage rundown mode deactivated from web dashboard")
                    return jsonify({
                        'success': True,
                        'active': False,
                        'message': 'Storage rundown disabled.'
                    })
                else:
                    STORAGE_RUNDOWN_FLAG.touch()
                    with self.state_lock:
                        self.state['storage_rundown_active'] = True
                    self.get_logger().info("Storage rundown mode activated from web dashboard")
                    return jsonify({
                        'success': True,
                        'active': True,
                        'message': 'Storage rundown enabled. System will discharge to 7.6V then shut down (mode cleared on reboot).'
                    })
            except OSError as e:
                self.get_logger().error(f"Failed to toggle storage mode: {e}")
                return jsonify({'success': False, 'message': str(e)}), 500
    
    def _call_service(self, client, service_name):
        """Generic service call wrapper with improved timeout and error handling."""
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return jsonify({
                    'success': False, 
                    'message': f'Service {service_name} unavailable (timeout waiting for service)'
                }), 503
            
            request = Trigger.Request()
            future = client.call_async(request)
            
            # Use loop with shutdown check instead of blocking spin_until_future_complete
            # This allows shutdown signals to interrupt long-running service calls
            timeout = 10.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                if self.signal_received or not rclpy.ok():
                    self.get_logger().debug("Shutdown requested during service call, cancelling")
                    break
                # Use short timeout to allow checking shutdown flag frequently
                try:
                    rclpy.spin_once(self, timeout_sec=0.1)
                except Exception:
                    # Context may be shutting down
                    if self.signal_received or not rclpy.ok():
                        break
            
            if future.done():
                try:
                    response = future.result()
                    # Always include the service response message for detailed information
                    return jsonify({
                        'success': response.success, 
                        'message': response.message if response.message else 'Service call completed'
                    })
                except Exception as e:
                    # Handle case where future.result() raises exception
                    return jsonify({
                        'success': False, 
                        'message': f'Error getting service response: {str(e)}'
                    }), 500
            else:
                return jsonify({
                    'success': False, 
                    'message': f'Service call to {service_name} timed out after 10 seconds'
                }), 504
                
        except Exception as e:
            return jsonify({
                'success': False, 
                'message': f'Service call error: {str(e)}'
            }), 500
    
    def run_flask(self):
        """Run Flask server in separate thread."""
        self.get_logger().info("Starting Werkzeug WSGI server...")
        try:
            self.wsgi_server.serve_forever()
        except Exception as e:
            if not self.signal_received:
                self.get_logger().error(f"Flask server error: {e}")
        finally:
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
  - System control (start/stop, recording)
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
        last_gps_staleness_check = time.time()
        
        while rclpy.ok() and not node.signal_received:
            # Spin once with a short timeout to handle ROS callbacks
            executor.spin_once(timeout_sec=0.1)
            
            # Check shutdown flag immediately after spin
            if node.signal_received:
                break
            
            now = time.time()
            
            # Determine current update intervals based on power mode
            status_interval = node.status_timer_period_idle if node.low_power_mode else node.status_timer_period_active
            health_interval = node.health_timer_period_idle if node.low_power_mode else node.health_timer_period_active
            viewer_check_interval = 5.0 # Constant 5s check
            gps_staleness_interval = 1.0  # Check GPS staleness every 1 second (safety-critical)
            
            # --- Manually trigger periodic tasks ---
            # GPS staleness check (ALWAYS runs at 1Hz regardless of power mode - safety critical!)
            if now - last_gps_staleness_check > gps_staleness_interval:
                node._update_gps_staleness()
                last_gps_staleness_check = now
            
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
        if node:
            node.signal_received = True  # Set flag to ensure clean shutdown
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Ensure proper cleanup
        if node:
            node.shutdown()
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
            
        # Only shutdown if context is still valid (may already be shut down by signal handler)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # Context may already be shutdown - this is expected
            pass
        print("🛑 Web dashboard shutdown complete.")


if __name__ == '__main__':
    main()
