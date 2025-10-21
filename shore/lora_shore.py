#!/usr/bin/env python3
"""
Shore-Side LoRa Communication Node
Interfaces with Waveshare USB-TO-LoRa module for bidirectional communication with Argo sailboat

Receives status packets from Argo and sends commands to Argo.
Republishes Argo data to ROS2 topics for Foxglove visualization.

Hardware: Waveshare USB-TO-LoRa-LF-B (SX1262)
Interface: USB Serial (/dev/ttyACM0, 115200 baud)
Protocol: Transparent mode with optional Waveshare 4-byte header stripping

Dependencies:
  - ROS2 Humble: sudo apt install ros-humble-ros-base
  - pyserial: pip3 install pyserial
  - See INSTALL.md for complete setup instructions
"""

import sys
import os
import json
import time
import threading
from datetime import datetime

# Check for ROS2 before other imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Float64, Bool, Int32
    from geometry_msgs.msg import Vector3
except ImportError as e:
    print("=" * 70)
    print("ERROR: ROS2 (rclpy) not found!")
    print("=" * 70)
    print()
    
    # Check if in conda environment
    if 'CONDA_DEFAULT_ENV' in os.environ or 'CONDA_PREFIX' in os.environ:
        conda_env = os.environ.get('CONDA_DEFAULT_ENV', 'unknown')
        print(f"⚠️  WARNING: You are in a conda environment: ({conda_env})")
        print()
        print("Conda Python interferes with ROS2. Please deactivate conda first:")
        print()
        print("  conda deactivate")
        print("  source /opt/ros/humble/setup.bash")
        print("  python3 lora_shore.py")
        print()
        print("Or create a shell script that handles this:")
        print("  #!/bin/bash")
        print("  source /opt/ros/humble/setup.bash")
        print("  exec python3 \"$@\"")
        print()
    else:
        print("This node requires ROS2 Humble to be installed.")
        print()
        print("Quick fix:")
        print("  1. Source ROS2 environment:")
        print("     source /opt/ros/humble/setup.bash")
        print()
        print("  2. Or install ROS2 Humble:")
        print("     sudo apt install ros-humble-ros-base")
        print()
    
    print("See shore/INSTALL.md for detailed installation instructions.")
    print()
    print("=" * 70)
    sys.exit(1)

# Check for pyserial
try:
    import serial
except ImportError:
    print("=" * 70)
    print("ERROR: pyserial not found!")
    print("=" * 70)
    print()
    print("This node requires pyserial for serial communication.")
    print()
    print("Install with:")
    print("  pip3 install pyserial")
    print()
    print("Or install all dependencies:")
    print("  pip3 install -r shore/requirements.txt")
    print()
    print("=" * 70)
    sys.exit(1)

class LoRaShoreNode(Node):
    def __init__(self):
        super().__init__('lora_shore_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('enable_debug', False)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('enable_debug').get_parameter_value().bool_value
        
        # Serial connection state
        self.ser = None
        self.connection_attempts = 0
        self.last_connection_attempt = 0
        self.reconnect_interval = 1.0  # 1 Hz retry rate
        
        # Initialize serial connection
        self.connect_serial()
        
        # Standard QoS for real-time data
        self.standard_qos = 10
        
        # Persistent QoS for status info
        self.persistent_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Publishers - Republish Argo data with 'argo/' prefix
        self.pub_argo_gps_sog = self.create_publisher(Float64, 'argo/gps_sog', self.standard_qos)
        self.pub_argo_gps_cog = self.create_publisher(Float64, 'argo/gps_cog', self.standard_qos)
        self.pub_argo_battery = self.create_publisher(Float64, 'argo/battery_voltage', self.standard_qos)
        self.pub_argo_human_controlled = self.create_publisher(Bool, 'argo/human_controlled', self.persistent_qos)
        self.pub_argo_rssi = self.create_publisher(Int32, 'argo/lora_rssi', self.standard_qos)
        self.pub_argo_last_contact = self.create_publisher(String, 'argo/last_contact', self.standard_qos)
        self.pub_argo_raw_data = self.create_publisher(String, 'argo/lora_raw', self.standard_qos)
        self.pub_shore_status = self.create_publisher(String, 'shore/lora_status', self.persistent_qos)
        
        # Subscriber - Receive commands to send to Argo
        self.sub_remote_cmd = self.create_subscription(
            String,
            'argo/remote_command',
            self.remote_command_callback,
            self.standard_qos
        )
        
        # State tracking
        self.last_packet_time = None
        self.packet_count = 0
        self.command_count = 0
        
        # Start serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info("Shore-side LoRa node ready. Listening for Argo packets...")
    
    def connect_serial(self):
        """Connect or reconnect to serial port with throttled retry"""
        current_time = time.time()
        
        # Throttle connection attempts to 1 Hz
        if current_time - self.last_connection_attempt < self.reconnect_interval:
            return False
        
        self.last_connection_attempt = current_time
        self.connection_attempts += 1
        
        # Close existing connection if any
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        
        # Attempt to open serial port
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(
                f"Connected to Waveshare LoRa on {self.serial_port} at {self.baud_rate} baud "
                f"(attempt #{self.connection_attempts})"
            )
            self.connection_attempts = 0  # Reset counter on success
            return True
        except serial.SerialException as e:
            self.get_logger().warn(
                f"Connection attempt #{self.connection_attempts} failed: {e}. "
                f"Retrying in {self.reconnect_interval}s..."
            )
            self.ser = None
            return False
    
    def serial_reader_loop(self):
        """Background thread that reads from serial port and processes packets"""
        buffer = ""
        
        while self.running:
            # Check if we need to (re)connect
            if not self.ser or not self.ser.is_open:
                if self.connect_serial():
                    buffer = ""  # Clear buffer on new connection
                else:
                    time.sleep(0.1)  # Wait before checking connection again
                    continue
            
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    text = data.decode('utf-8', errors='ignore')
                    buffer += text
                    
                    # Process complete JSON packets in buffer
                    while '{' in buffer and '}' in buffer:
                        json_start = buffer.index('{')
                        try:
                            json_end = buffer.index('}', json_start) + 1
                            json_str = buffer[json_start:json_end]
                            buffer = buffer[json_end:]  # Remove processed packet
                            
                            # Strip trailing Waveshare status bytes (e.g., "30", "33")
                            # They appear after the closing brace
                            json_str = json_str.strip()
                            
                            # Try to parse JSON
                            try:
                                packet = json.loads(json_str)
                                self.process_argo_packet(packet, json_str)
                            except json.JSONDecodeError as e:
                                if self.debug:
                                    self.get_logger().warn(f"JSON decode error: {e}, data: {json_str}")
                        except ValueError:
                            # No closing brace yet
                            break
                
                time.sleep(0.01)  # 100 Hz check rate
                
            except (serial.SerialException, OSError) as e:
                # Connection lost - log once and trigger reconnection
                self.get_logger().error(f"Serial connection lost: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                    self.ser = None
                buffer = ""  # Clear buffer on disconnect
                
            except Exception as e:
                self.get_logger().error(f"Unexpected error in serial reader: {e}")
                time.sleep(0.1)
    
    def process_argo_packet(self, packet: dict, raw_json: str):
        """Process received packet from Argo and publish to ROS2 topics"""
        try:
            self.packet_count += 1
            self.last_packet_time = time.time()
            
            if self.debug:
                self.get_logger().info(f"Packet #{self.packet_count}: {packet}")
            
            # Publish raw data
            raw_msg = String()
            raw_msg.data = raw_json
            self.pub_argo_raw_data.publish(raw_msg)
            
            # Extract and publish individual fields
            if 'gps_sog' in packet:
                msg = Float64()
                msg.data = float(packet['gps_sog'])
                self.pub_argo_gps_sog.publish(msg)
            
            if 'gps_cog' in packet:
                msg = Float64()
                msg.data = float(packet['gps_cog'])
                self.pub_argo_gps_cog.publish(msg)
            
            if 'battery_voltage' in packet:
                msg = Float64()
                msg.data = float(packet['battery_voltage'])
                self.pub_argo_battery.publish(msg)
            
            if 'human_controlled' in packet:
                msg = Bool()
                msg.data = bool(packet['human_controlled'])
                self.pub_argo_human_controlled.publish(msg)
            
            # Publish last contact time
            contact_msg = String()
            contact_msg.data = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.pub_argo_last_contact.publish(contact_msg)
            
            # Note: RSSI is appended by Waveshare as trailing bytes, not in JSON
            # Would need separate parsing if needed
            
        except Exception as e:
            self.get_logger().error(f"Error processing Argo packet: {e}")
    
    def remote_command_callback(self, msg):
        """Receive command from ROS2 and send to Argo via LoRa"""
        try:
            command = msg.data
            
            if not self.ser or not self.ser.is_open:
                self.get_logger().error("Serial port not connected, cannot send command. Waiting for reconnection...")
                return
            
            self.command_count += 1
            self.get_logger().info(f"Sending command to Argo: {command}")
            
            # Send command as plain text (Waveshare transparent mode)
            # Argo will receive it with Waveshare header already stripped by firmware
            self.ser.write(command.encode('utf-8'))
            self.ser.flush()
            
            self.get_logger().info(f"Command sent: {command} ({len(command)} bytes)")
            
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Error sending command (connection lost): {e}")
            # Mark connection as lost to trigger reconnection
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending command: {e}")
    
    def publish_status(self):
        """Publish shore-side status"""
        try:
            is_connected = self.ser is not None and self.ser.is_open
            
            status = {
                'port': self.serial_port,
                'baud': self.baud_rate,
                'connected': is_connected,
                'connection_attempts': self.connection_attempts,
                'packets_received': self.packet_count,
                'commands_sent': self.command_count,
                'time_since_last_packet': time.time() - self.last_packet_time if self.last_packet_time else None
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.pub_shore_status.publish(msg)
            
            # Warn if not connected
            if not is_connected:
                self.get_logger().warn(
                    f"LoRa not connected to {self.serial_port}. "
                    f"Attempting reconnection (attempt #{self.connection_attempts})..."
                )
            # Warn if no packets received recently
            elif self.last_packet_time and (time.time() - self.last_packet_time) > 30:
                self.get_logger().warn(f"No packets from Argo for {int(time.time() - self.last_packet_time)}s")
        
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down shore-side LoRa node...")
        self.running = False
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = LoRaShoreNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unexpected error: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

