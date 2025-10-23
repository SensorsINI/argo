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
import argparse
from datetime import datetime

# Check for ROS2 before other imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Float64, Bool, Int32
    from geometry_msgs.msg import Vector3
    from sensor_msgs.msg import NavSatFix
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
    def __init__(self, debug=False):
        super().__init__('lora_shore_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('enable_debug', debug)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('enable_debug').get_parameter_value().bool_value
        
        # Serial connection state
        self.ser = None
        self.connection_attempts = 0
        self.last_connection_attempt = 0
        self.reconnect_interval = 1.0  # 1 Hz retry rate
        self.shutting_down = False  # Flag to prevent logging during shutdown
        
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
        
        # Publishers - Republish Argo data with 'lora/' prefix
        self.pub_argo_gps_sog = self.create_publisher(Float64, 'lora/gps_sog', self.standard_qos)
        self.pub_argo_gps_cog = self.create_publisher(Float64, 'lora/gps_cog', self.standard_qos)
        self.pub_argo_battery = self.create_publisher(Float64, 'lora/battery_voltage', self.standard_qos)
        self.pub_argo_human_controlled = self.create_publisher(Bool, 'lora/human_controlled', self.persistent_qos)
        self.pub_argo_gps_fix = self.create_publisher(NavSatFix, 'lora/fix', self.standard_qos)
        self.pub_argo_compass = self.create_publisher(Vector3, 'lora/compass', self.standard_qos)
        self.pub_argo_rssi = self.create_publisher(Int32, 'lora/rssi', self.standard_qos)
        self.pub_argo_last_contact = self.create_publisher(String, 'lora/last_contact', self.standard_qos)
        self.pub_argo_raw_data = self.create_publisher(String, 'lora/raw', self.standard_qos)
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
        
        # Ping mechanism
        self.ping_sequence = 0
        self.ping_timer = self.create_timer(5.0, self.send_ping)
        
        # Start serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_reader_loop, daemon=True)
        self.reader_thread.start()
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info("Shore-side LoRa node ready. Listening for Argo packets...")
    
    def connect_serial(self):
        """Connect or reconnect to serial port with throttled retry"""
        if self.shutting_down:
            return False  # Don't attempt connection during shutdown
            
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
            if not self.shutting_down:
                self.get_logger().info(
                    f"Connected to Waveshare LoRa on {self.serial_port} at {self.baud_rate} baud "
                    f"(attempt #{self.connection_attempts})"
                )
            self.connection_attempts = 0  # Reset counter on success
            return True
        except serial.SerialException as e:
            if not self.shutting_down:
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
                    
                    if self.debug:
                        self.get_logger().debug(f"Serial data received: {len(data)} bytes, buffer now: {len(buffer)} chars")
                        if len(text) > 0:
                            self.get_logger().debug(f"Raw text: {repr(text)}")
                    
                    # Process complete JSON packets in buffer
                    while '{' in buffer and '}' in buffer:
                        json_start = buffer.index('{')
                        try:
                            json_end = buffer.index('}', json_start) + 1
                            json_str = buffer[json_start:json_end]
                            buffer = buffer[json_end:]  # Remove processed packet
                            
                            if self.debug:
                                self.get_logger().debug(f"Extracted JSON packet: {len(json_str)} chars")
                            
                            # Strip trailing Waveshare status bytes (e.g., "30", "33")
                            # They appear after the closing brace
                            json_str = json_str.strip()
                            
                            if self.debug:
                                self.get_logger().debug(f"Stripped JSON: {repr(json_str)}")
                            
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
                if not self.shutting_down:
                    self.get_logger().error(f"Serial connection lost: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                    self.ser = None
                buffer = ""  # Clear buffer on disconnect
                
            except Exception as e:
                if not self.shutting_down:
                    self.get_logger().error(f"Unexpected error in serial reader: {e}")
                time.sleep(0.1)
    
    def process_argo_packet(self, packet: dict, raw_json: str):
        """Process received packet from Argo and publish to ROS2 topics"""
        if self.shutting_down:
            return  # Don't process packets during shutdown
            
        try:
            self.packet_count += 1
            self.last_packet_time = time.time()
            
            if self.debug:
                self.get_logger().info(f"=== PACKET #{self.packet_count} RECEIVED ===")
                self.get_logger().info(f"Raw JSON: {raw_json}")
                self.get_logger().info(f"Parsed packet: {packet}")
                self.get_logger().info(f"Packet keys: {list(packet.keys())}")
                self.get_logger().info(f"Timestamp: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
            
            # Publish raw data
            raw_msg = String()
            raw_msg.data = raw_json
            self.pub_argo_raw_data.publish(raw_msg)
            
            # Parse abbreviated keys and publish to lora/ prefixed topics
            if 'sog' in packet:
                msg = Float64()
                msg.data = float(packet['sog'])
                self.pub_argo_gps_sog.publish(msg)
            
            if 'cog' in packet:
                msg = Float64()
                msg.data = float(packet['cog'])
                self.pub_argo_gps_cog.publish(msg)
            
            if 'bat' in packet:
                msg = Float64()
                msg.data = float(packet['bat'])
                self.pub_argo_battery.publish(msg)
            
            if 'hum' in packet:
                msg = Bool()
                msg.data = bool(packet['hum'])
                self.pub_argo_human_controlled.publish(msg)
            
            # GPS position
            if 'lat' in packet and 'lon' in packet:
                fix_msg = NavSatFix()
                fix_msg.latitude = float(packet['lat'])
                fix_msg.longitude = float(packet['lon'])
                fix_msg.header.stamp = self.get_clock().now().to_msg()
                fix_msg.header.frame_id = 'map'
                self.pub_argo_gps_fix.publish(fix_msg)
            
            # Compass heading
            if 'hdg' in packet:
                heading_msg = Vector3()
                heading_msg.z = float(packet['hdg'])
                self.pub_argo_compass.publish(heading_msg)
            
            # Publish last contact time
            contact_msg = String()
            contact_msg.data = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.pub_argo_last_contact.publish(contact_msg)
            
            if self.debug:
                self.get_logger().info(f"=== PACKET #{self.packet_count} PROCESSING COMPLETE ===")
            
            # Note: RSSI is appended by Waveshare as trailing bytes, not in JSON
            # Would need separate parsing if needed
            
        except Exception as e:
            if not self.shutting_down:
                self.get_logger().error(f"Error processing Argo packet: {e}")
    
    def send_ping(self):
        """Send periodic ping to Argo for connection monitoring"""
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            self.ping_sequence += 1
            ping_msg = json.dumps({'cmd': 'ping', 'seq': self.ping_sequence}, separators=(',', ':'))
            
            # Add Waveshare stream mode header: [0x00, 0x00, 0x12, 0x11]
            waveshare_header = bytes([0x00, 0x00, 0x12, 0x11])
            packet_with_header = waveshare_header + ping_msg.encode('utf-8')
            
            self.ser.write(packet_with_header)
            self.ser.flush()
            
            if self.debug:
                self.get_logger().debug(f"Sent ping #{self.ping_sequence}: {ping_msg}")
                self.get_logger().debug(f"Packet with header: {len(packet_with_header)} bytes")
            else:
                self.get_logger().debug(f"Sent ping #{self.ping_sequence} with Waveshare header")
        except Exception as e:
            self.get_logger().debug(f"Error sending ping: {e}")
    
    def remote_command_callback(self, msg):
        """Receive command from ROS2 and send to Argo via LoRa"""
        if self.shutting_down:
            return  # Don't send commands during shutdown
            
        try:
            command = msg.data
            
            if not self.ser or not self.ser.is_open:
                if not self.shutting_down:
                    self.get_logger().error("Serial port not connected, cannot send command. Waiting for reconnection...")
                return
            
            self.command_count += 1
            
            if self.debug:
                self.get_logger().info(f"=== SENDING COMMAND #{self.command_count} ===")
                self.get_logger().info(f"Command: {command}")
                self.get_logger().info(f"Command length: {len(command)} bytes")
            
            # Send command with Waveshare stream mode header
            # Add Waveshare stream mode header: [0x00, 0x00, 0x12, 0x11]
            waveshare_header = bytes([0x00, 0x00, 0x12, 0x11])
            packet_with_header = waveshare_header + command.encode('utf-8')
            
            self.ser.write(packet_with_header)
            self.ser.flush()
            
            if self.debug:
                self.get_logger().info(f"Packet with header: {len(packet_with_header)} bytes")
                self.get_logger().info(f"=== COMMAND #{self.command_count} SENT ===")
            else:
                self.get_logger().info(f"Command sent: {command} ({len(command)} bytes)")
            
        except (serial.SerialException, OSError) as e:
            if not self.shutting_down:
                self.get_logger().error(f"Error sending command (connection lost): {e}")
            # Mark connection as lost to trigger reconnection
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
        except Exception as e:
            if not self.shutting_down:
                self.get_logger().error(f"Unexpected error sending command: {e}")
    
    def publish_status(self):
        """Publish shore-side status"""
        if self.shutting_down:
            return  # Don't publish or log during shutdown
            
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
            if not self.shutting_down:
                self.get_logger().error(f"Error publishing status: {e}")
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        # Only log if we're not already shutting down (to prevent context errors)
        if not self.shutting_down:
            try:
                self.get_logger().info("Shutting down shore-side LoRa node...")
            except:
                pass  # Context may already be invalid
        
        # Set shutdown flag to prevent any further logging from threads/timers
        self.shutting_down = True
        
        # Stop reader thread first
        self.running = False
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)  # Give more time for clean exit
        
        # Close serial port
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        
        # Destroy timers explicitly before destroying node
        if hasattr(self, 'status_timer'):
            self.status_timer.cancel()
        
        # Now destroy the node and invalidate ROS context
        super().destroy_node()

def print_help():
    """Print detailed help information"""
    print("""
Shore-Side LoRa Communication Node
==================================

This node interfaces with a Waveshare USB-TO-LoRa module to provide bidirectional
communication with the Argo sailboat. It receives status packets from Argo and
sends commands to Argo via LoRa radio.

USAGE:
    python3 lora_shore.py [OPTIONS]

OPTIONS:
    --help, -h          Show this help message and exit
    --debug, -d         Enable verbose debug logging for packet reception
    --port PORT         Serial port device (default: /dev/ttyACM0)
    --baud RATE         Serial baud rate (default: 115200)

EXAMPLES:
    # Basic usage
    python3 lora_shore.py

    # Enable debug logging
    python3 lora_shore.py --debug

    # Use different serial port
    python3 lora_shore.py --port /dev/ttyUSB0

    # Debug with custom port
    python3 lora_shore.py --debug --port /dev/ttyACM1

HARDWARE REQUIREMENTS:
    - Waveshare USB-TO-LoRa-LF-B (SX1262) module
    - USB connection to computer
    - Proper antenna connected to module

ROS2 TOPICS:
    Published:
        lora/gps_sog          - GPS speed over ground (knots)
        lora/gps_cog          - GPS course over ground (degrees)
        lora/battery_voltage  - Battery voltage (volts)
        lora/human_controlled - Human control status (boolean)
        lora/fix              - GPS position (NavSatFix)
        lora/compass          - Compass heading (degrees)
        lora/rssi             - Signal strength (dBm)
        lora/last_contact     - Last contact timestamp
        lora/raw              - Raw received data
        shore/lora_status     - Shore-side status information

    Subscribed:
        argo/remote_command   - Commands to send to Argo

DEBUG MODE:
    When --debug is enabled, the node provides detailed LoRa radio link logging:
    - Raw serial data reception from Waveshare module
    - JSON packet parsing and validation
    - LoRa packet transmission details (pings and commands)
    - Serial connection status and reconnection attempts
    - Packet reception timing and sequence numbers

TROUBLESHOOTING:
    - Ensure Waveshare module is connected and recognized
    - Check serial port permissions (may need to add user to dialout group)
    - Verify module is in stream mode (MODE=1)
    - Check network ID matches Argo (NETID=18)
    - Ensure proper antenna is connected

For more information, see shore/README.md and shore/INSTALL.md
""")

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='Shore-Side LoRa Communication Node for Argo Sailboat',
        add_help=False  # We'll handle help manually
    )
    
    parser.add_argument('--help', '-h', action='store_true',
                       help='Show help message and exit')
    parser.add_argument('--debug', '-d', action='store_true',
                       help='Enable verbose debug logging for packet reception')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0',
                       help='Serial port device (default: /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                       help='Serial baud rate (default: 115200)')
    
    return parser.parse_args()

def main(args=None):
    # Parse command line arguments
    cli_args = parse_arguments()
    
    # Handle help
    if cli_args.help:
        print_help()
        return
    
    # Initialize ROS2 with our arguments
    ros_args = []
    if cli_args.debug:
        ros_args.extend(['--ros-args', '-p', 'enable_debug:=true'])
    if cli_args.port != '/dev/ttyACM0':
        ros_args.extend(['--ros-args', '-p', f'serial_port:={cli_args.port}'])
    if cli_args.baud != 115200:
        ros_args.extend(['--ros-args', '-p', f'baud_rate:={cli_args.baud}'])
    
    rclpy.init(args=ros_args)
    node = None
    
    try:
        node = LoRaShoreNode(debug=cli_args.debug)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Don't try to log here - ROS context may be shutting down
        # Set shutdown flag to prevent any further logging attempts
        if node:
            node.shutting_down = True
        import sys
        print(f"Unexpected error: {e}", file=sys.stderr)
    finally:
        if node:
            # Set shutdown flag BEFORE calling destroy_node to prevent race conditions
            node.shutting_down = True
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

