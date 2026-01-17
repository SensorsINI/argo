#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
# ROS2 Mast LED Controller Node - LP5814DLR RGBW LED Driver

# This ROS2 node controls the LP5814DLR RGBW LED controller from Texas Instruments
# located on the same PCB as the wind sensor. The LED controller is used to provide
# visual status indication on the mast.
#
# Hardware Setup:
# - Uses I2C bus 0 (same as wind sensor)
# - LP5814DLR at I2C address 0x21
# - Four LED channels: Red, Green, Blue, White (RGBW)
# - Run "sudo i2cdetect -y 0" to verify device connection
# - Device should show as address 21 in hex
#
# Features:
# - Four individual subscriptions (one per color channel: R, G, B, W)
# - One combined 4-component subscription for synchronized RGBW control
# - Power button RGB mirroring: mirrors RGB channels from power button LEDs
# - Internal synchronization ensures all color updates take effect together
# - Normalized brightness control (0.0-1.0) converted to 8-bit (0-255) for hardware
# - Automatic I2C error recovery with health monitoring
# - Future-ready structure for LED animation programming and playback
#
# Topics Subscribed:
# /mastled_r (std_msgs/Float32):
#   Red channel brightness (0.0-1.0, where 1.0 = maximum brightness)
# /mastled_g (std_msgs/Float32):
#   Green channel brightness (0.0-1.0)
# /mastled_b (std_msgs/Float32):
#   Blue channel brightness (0.0-1.0)
# /mastled_w (std_msgs/Float32):
#   White channel brightness (0.0-1.0)
# /mastled_rgbw (std_msgs/Float32MultiArray):
#   Combined RGBW control (data array with 4 elements: [R, G, B, W] as Float32 0.0-1.0)
# /argo/power_button/rgb (geometry_msgs/Vector3):
#   Power button RGB mirroring (x=red, y=green, z=blue as 0.0-1.0)
#   Mirrors RGB channels from power button LEDs controlled by argo_power_control.py
#   White channel is not affected by power button mirroring
#
# Command line options:
# --debug: Enable debug logging of LED values and I2C operations

# Import path setup (must be before other imports)
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))

# Standard library imports
import time
import argparse
import argcomplete

# ROS2 imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3

# I2C communication
import smbus

# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode

# ============================================================================
# Constants - LP5814DLR Configuration
# ============================================================================

# I2C Configuration
I2C_BUS = 0  # I2C bus number (same as wind sensor)
LP5814_I2C_ADDRESS = 0x21  # 7-bit I2C address of LP5814DLR

# LP5814DLR Register Addresses
# Note: These are typical register addresses for LP5814DLR family
# Adjust based on actual LP5814DLR datasheet if needed
REG_DEVICE_CONFIG = 0x00  # Device configuration register
REG_OUT0_PWM = 0x04  # Output channel 0 PWM (Red) register
REG_OUT1_PWM = 0x05  # Output channel 1 PWM (Green) register
REG_OUT2_PWM = 0x06  # Output channel 2 PWM (Blue) register
REG_OUT3_PWM = 0x07  # Output channel 3 PWM (White) register

# Device Configuration Bits
DEV_CONFIG_ENABLE = 0x01  # Enable device
DEV_CONFIG_MANUAL_MODE = 0x02  # Manual PWM mode (not animation mode)

# Brightness Configuration
BRIGHTNESS_MIN = 0.0  # Minimum normalized brightness
BRIGHTNESS_MAX = 1.0  # Maximum normalized brightness
BRIGHTNESS_HW_MIN = 0  # Minimum hardware brightness (8-bit)
BRIGHTNESS_HW_MAX = 255  # Maximum hardware brightness (8-bit)

# Topic Names
TOPIC_RED = '/mastled_r'
TOPIC_GREEN = '/mastled_g'
TOPIC_BLUE = '/mastled_b'
TOPIC_WHITE = '/mastled_w'
TOPIC_RGBW = '/mastled_rgbw'
# Power button LED mirroring topic
TOPIC_POWER_BUTTON_RGB = '/argo/power_button/rgb'

# I2C Error Recovery
I2C_RETRY_DELAY_S = 1.0  # Delay between I2C retry attempts in seconds (initial retries)
I2C_ERROR_LOG_THROTTLE_S = 5.0  # Maximum I2C error logging frequency in seconds
I2C_MAX_INITIAL_RETRIES = 3  # Maximum retries during initial startup before entering low-CPU mode
I2C_LOW_CPU_CHECK_INTERVAL_S = 60.0  # Interval for checking device availability in low-CPU mode (60 seconds)


class MastLEDsNode(ArgoBaseNode):
    """ROS2 node for controlling mast RGBW LEDs via LP5814DLR driver."""

    def __init__(self, debug_mode: bool = False):
        super().__init__('mastleds_node')

        self.get_logger().info('Initializing Mast LED Controller node...')

        # Debug mode flag
        self.debug_mode = debug_mode

        # Internal LED state (normalized 0.0-1.0)
        self._red_brightness = 0.0
        self._green_brightness = 0.0
        self._blue_brightness = 0.0
        self._white_brightness = 0.0

        # I2C bus and device state
        self.bus = None
        self.device_ready = False
        self.device_unavailable = False  # True when device is permanently unavailable (low-CPU mode)
        self._last_i2c_error_log_time = 0.0
        self._consecutive_i2c_errors = 0
        self._initial_retry_count = 0  # Count of initial retry attempts

        # Initialize I2C bus and device
        if self._initialize_i2c():
            if self._initialize_device():
                self.device_ready = True
                self.set_healthy("LED controller initialized and ready")
            else:
                self.set_unhealthy("Failed to initialize LP5814DLR device")
                self._initial_retry_count = 1
                self._start_retry_timer()
        else:
            self.set_unhealthy("Failed to open I2C bus")
            self._initial_retry_count = 1
            self._start_retry_timer()

        # Create subscriptions
        self._create_subscriptions()

        self.get_logger().info('Mast LED Controller node initialized')

    def _initialize_i2c(self):
        """Initialize I2C bus, returns True on success."""
        try:
            if not self.bus:
                self.bus = smbus.SMBus(I2C_BUS)
                self.get_logger().info(f'Opened I2C bus {I2C_BUS}')
                return True
        except FileNotFoundError:
            self.get_logger().error("I2C bus not found. Is I2C enabled?")
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            return False
        return False

    def _initialize_device(self):
        """Initialize LP5814DLR device, returns True on success."""
        if not self.bus:
            return False

        try:
            # Configure device: enable and set to manual PWM mode
            config_value = DEV_CONFIG_ENABLE | DEV_CONFIG_MANUAL_MODE
            self._write_register(REG_DEVICE_CONFIG, config_value)
            
            # Initialize all channels to off
            self._write_register(REG_OUT0_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT1_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT2_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT3_PWM, BRIGHTNESS_HW_MIN)

            self.get_logger().info('LP5814DLR device initialized successfully')
            return True

        except IOError as e:
            self._handle_i2c_error(f"Failed to initialize LP5814DLR: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error initializing device: {e}")
            return False

    def _write_register(self, register: int, value: int):
        """Write a single byte to an LP5814DLR register."""
        if not self.bus:
            raise IOError("I2C bus not initialized")
        
        # Clamp value to 8-bit range
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(LP5814_I2C_ADDRESS, register, value)

    def _handle_i2c_error(self, error_msg: str):
        """Handle I2C errors with throttled logging."""
        current_time = time.time()
        self._consecutive_i2c_errors += 1

        # Log error with throttling
        if current_time - self._last_i2c_error_log_time >= I2C_ERROR_LOG_THROTTLE_S:
            self.get_logger().warn(
                f"I2C error: {error_msg} (Total consecutive errors: {self._consecutive_i2c_errors})")
            self._last_i2c_error_log_time = current_time

        # Mark as unhealthy after first error
        if self.device_ready:
            self.device_ready = False
            self.set_unhealthy("I2C communication error")

    def _start_retry_timer(self):
        """Start timer to periodically retry device initialization."""
        if not hasattr(self, 'retry_timer') or self.retry_timer is None:
            self.get_logger().info("Starting device initialization retry timer (1 Hz)")
            self.retry_timer = self.create_timer(I2C_RETRY_DELAY_S, self._retry_initialization)

    def _retry_initialization(self):
        """Callback for retry timer - attempts to reinitialize device."""
        if self.device_ready:
            if self.retry_timer:
                self.retry_timer.cancel()
            self.device_unavailable = False  # Clear unavailable flag if device is ready
            return

        # If device is marked as unavailable, use low-CPU check interval
        if self.device_unavailable:
            # Check device only periodically in low-CPU mode
            if not hasattr(self, '_last_low_cpu_check'):
                self._last_low_cpu_check = 0.0
            
            current_time = time.time()
            if current_time - self._last_low_cpu_check < I2C_LOW_CPU_CHECK_INTERVAL_S:
                return  # Not time to check yet
            
            self._last_low_cpu_check = current_time

        # During initial retries, count attempts
        if not self.device_unavailable:
            self._initial_retry_count += 1

        if not self.bus:
            if self._initialize_i2c():
                # I2C bus initialized, try device init
                if self._initialize_device():
                    self.device_ready = True
                    self.device_unavailable = False
                    self.set_healthy("LED controller reinitialized successfully")
                    if self.retry_timer:
                        self.retry_timer.cancel()
                    self._consecutive_i2c_errors = 0
                    self._initial_retry_count = 0
        else:
            # Bus exists, just retry device init
            if self._initialize_device():
                self.device_ready = True
                self.device_unavailable = False
                self.set_healthy("LED controller reinitialized successfully")
                if self.retry_timer:
                    self.retry_timer.cancel()
                self._consecutive_i2c_errors = 0
                self._initial_retry_count = 0
            else:
                # Device init failed - check if we should enter low-CPU mode
                # Only enter unavailable mode if I2C bus is available but device not found
                # (I2C bus unavailable is a system config issue, not missing hardware)
                if not self.device_unavailable and self._initial_retry_count >= I2C_MAX_INITIAL_RETRIES:
                    self._enter_device_unavailable_mode()

    def _enter_device_unavailable_mode(self):
        """Enter low-CPU mode when device is permanently unavailable (missing hardware)."""
        self.device_unavailable = True
        self.device_ready = False
        self.set_unhealthy("LP5814DLR LED controller not found at I2C address 0x21 - using original wind sensor without LED controller")
        self.get_logger().error(
            "LP5814DLR LED controller not found at I2C address 0x21. "
            "This indicates the original wind sensor (without LED controller) is in use. "
            "Node will continue running in low-CPU mode, ignoring LED control subscriptions. "
            "Hardware will be checked periodically for device appearance.")
        
        # Switch retry timer to low-CPU interval if it exists
        if hasattr(self, 'retry_timer') and self.retry_timer:
            self.retry_timer.cancel()
            # Start low-CPU periodic check timer (60s interval)
            self.retry_timer = self.create_timer(I2C_LOW_CPU_CHECK_INTERVAL_S, self._retry_initialization)

    def _normalize_brightness(self, value: float) -> float:
        """Clamp brightness value to valid range (0.0-1.0)."""
        if value is None:
            return BRIGHTNESS_MIN
        return max(BRIGHTNESS_MIN, min(BRIGHTNESS_MAX, float(value)))

    def _brightness_to_hw(self, normalized: float) -> int:
        """Convert normalized brightness (0.0-1.0) to hardware value (0-255)."""
        normalized = self._normalize_brightness(normalized)
        return int(round(normalized * BRIGHTNESS_HW_MAX))

    def _update_hardware(self):
        """Update all LED channels on hardware in synchronized manner.
        
        Silently ignores updates if device is not ready or unavailable (low-CPU mode).
        """
        if not self.device_ready or not self.bus or self.device_unavailable:
            # Silently ignore hardware updates when device unavailable
            # This allows subscriptions to work without errors, but hardware is not updated
            return

        try:
            # Write all channels in quick succession for synchronization
            self._write_register(REG_OUT0_PWM, self._brightness_to_hw(self._red_brightness))
            self._write_register(REG_OUT1_PWM, self._brightness_to_hw(self._green_brightness))
            self._write_register(REG_OUT2_PWM, self._brightness_to_hw(self._blue_brightness))
            self._write_register(REG_OUT3_PWM, self._brightness_to_hw(self._white_brightness))

            # Reset error counter on successful write
            self._consecutive_i2c_errors = 0

            if self.debug_mode:
                self.get_logger().debug(
                    f"LED update: R={self._red_brightness:.3f} G={self._green_brightness:.3f} "
                    f"B={self._blue_brightness:.3f} W={self._white_brightness:.3f}")

        except IOError as e:
            self._handle_i2c_error(f"Failed to update LEDs: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error updating LEDs: {e}")

    def _create_subscriptions(self):
        """Create all ROS2 subscriptions for LED control."""
        # Individual color channel subscriptions
        self.sub_red = self.create_subscription(
            Float32, TOPIC_RED, self._red_callback, 10)
        self.sub_green = self.create_subscription(
            Float32, TOPIC_GREEN, self._green_callback, 10)
        self.sub_blue = self.create_subscription(
            Float32, TOPIC_BLUE, self._blue_callback, 10)
        self.sub_white = self.create_subscription(
            Float32, TOPIC_WHITE, self._white_callback, 10)

        # Combined RGBW subscription
        self.sub_rgbw = self.create_subscription(
            Float32MultiArray, TOPIC_RGBW, self._rgbw_callback, 10)

        # Power button RGB mirroring subscription (mirrors RGB from power control node)
        self.sub_power_button_rgb = self.create_subscription(
            Vector3, TOPIC_POWER_BUTTON_RGB, self._power_button_rgb_callback, 10)

        self.get_logger().info(f'Subscribed to {TOPIC_RED}, {TOPIC_GREEN}, {TOPIC_BLUE}, {TOPIC_WHITE}, {TOPIC_RGBW}, {TOPIC_POWER_BUTTON_RGB}')

    def _red_callback(self, msg: Float32):
        """Callback for red channel subscription."""
        # Update internal state even if device unavailable (allows recovery)
        self._red_brightness = self._normalize_brightness(msg.data)
        # Hardware update will be ignored if device unavailable
        self._update_hardware()

    def _green_callback(self, msg: Float32):
        """Callback for green channel subscription."""
        # Update internal state even if device unavailable (allows recovery)
        self._green_brightness = self._normalize_brightness(msg.data)
        # Hardware update will be ignored if device unavailable
        self._update_hardware()

    def _blue_callback(self, msg: Float32):
        """Callback for blue channel subscription."""
        # Update internal state even if device unavailable (allows recovery)
        self._blue_brightness = self._normalize_brightness(msg.data)
        # Hardware update will be ignored if device unavailable
        self._update_hardware()

    def _white_callback(self, msg: Float32):
        """Callback for white channel subscription."""
        # Update internal state even if device unavailable (allows recovery)
        self._white_brightness = self._normalize_brightness(msg.data)
        # Hardware update will be ignored if device unavailable
        self._update_hardware()

    def _rgbw_callback(self, msg: Float32MultiArray):
        """Callback for combined RGBW subscription."""
        if len(msg.data) >= 4:
            # Update all channels from array [R, G, B, W]
            # Update internal state even if device unavailable (allows recovery)
            self._red_brightness = self._normalize_brightness(msg.data[0])
            self._green_brightness = self._normalize_brightness(msg.data[1])
            self._blue_brightness = self._normalize_brightness(msg.data[2])
            self._white_brightness = self._normalize_brightness(msg.data[3])
            # Hardware update will be ignored if device unavailable
            self._update_hardware()
        else:
            self.get_logger().warn(
                f"RGBW message has {len(msg.data)} elements, expected 4. Ignoring.")

    def _power_button_rgb_callback(self, msg: Vector3):
        """Callback for power button RGB mirroring subscription.
        
        Mirrors RGB channels from power button LEDs to mast head LEDs.
        x = red, y = green, z = blue (normalized 0.0-1.0).
        White channel is not affected by power button mirroring.
        
        Silently ignores updates if device unavailable (low-CPU mode).
        """
        # Update RGB channels from power button (Vector3: x=R, y=G, z=B)
        # Update internal state even if device unavailable (allows recovery)
        self._red_brightness = self._normalize_brightness(msg.x)
        self._green_brightness = self._normalize_brightness(msg.y)
        self._blue_brightness = self._normalize_brightness(msg.z)
        # White channel is NOT affected by power button mirroring
        # (power button only has RGB, not RGBW)
        # Hardware update will be ignored if device unavailable
        self._update_hardware()

    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.get_logger().info('Shutting down Mast LED Controller node')

        # Turn off all LEDs
        if self.device_ready and self.bus:
            try:
                self._red_brightness = 0.0
                self._green_brightness = 0.0
                self._blue_brightness = 0.0
                self._white_brightness = 0.0
                self._update_hardware()
            except Exception as e:
                self.get_logger().warn(f"Error turning off LEDs on shutdown: {e}")

        # Publish health=false on shutdown
        self.set_unhealthy("Node shutting down")

        super().destroy_node()


def main(args=None):
    """Main entry point for mastleds node."""
    node = None
    try:
        parser = ArgoBaseNode.create_standard_parser(
            'Mast LED Controller Node for ROS2 - LP5814DLR RGBW LED Driver',
            epilog="""
This ROS2 node controls the LP5814DLR RGBW LED controller from Texas Instruments
located on the same PCB as the wind sensor.

Hardware Setup:
  - Uses I2C bus 0 (same as wind sensor)
  - LP5814DLR at I2C address 0x21
  - Four LED channels: Red, Green, Blue, White (RGBW)
  - Run "sudo i2cdetect -y 0" to verify device connection

Features:
  - Four individual subscriptions (one per color channel: R, G, B, W)
  - One combined 4-component subscription for synchronized RGBW control
  - Power button RGB mirroring: automatically mirrors RGB channels from power button LEDs
  - Internal synchronization ensures all color updates take effect together
  - Normalized brightness control (0.0-1.0) converted to 8-bit (0-255) for hardware
  - Automatic I2C error recovery with health monitoring
  - Future-ready structure for LED animation programming and playback

Topics Subscribed:
  /mastled_r (std_msgs/Float32):
    Red channel brightness (0.0-1.0, where 1.0 = maximum brightness)
  /mastled_g (std_msgs/Float32):
    Green channel brightness (0.0-1.0)
  /mastled_b (std_msgs/Float32):
    Blue channel brightness (0.0-1.0)
  /mastled_w (std_msgs/Float32):
    White channel brightness (0.0-1.0)
  /mastled_rgbw (std_msgs/Float32MultiArray):
    Combined RGBW control (data array with 4 elements: [R, G, B, W] as Float32 0.0-1.0)
  /argo/power_button/rgb (geometry_msgs/Vector3):
    Power button RGB mirroring (x=red, y=green, z=blue as 0.0-1.0)
    Automatically mirrors RGB channels from power button LEDs controlled by argo_power_control.py
    White channel is not affected by power button mirroring

Configuration Constants (modify at top of file):
  - I2C_BUS: I2C bus number (default: 0)
  - LP5814_I2C_ADDRESS: I2C address of LP5814DLR (default: 0x21)
  - REG_OUT0_PWM through REG_OUT3_PWM: PWM register addresses for R, G, B, W channels
  - BRIGHTNESS_MIN/MAX: Normalized brightness range (0.0-1.0)
  - BRIGHTNESS_HW_MIN/MAX: Hardware brightness range (0-255)
            """
        )

        parsed_args, unknown_args = parser.parse_known_args(args)
        argcomplete.autocomplete(parser)

        rclpy.init(args=unknown_args)

        node = MastLEDsNode(debug_mode=parsed_args.debug)

        if parsed_args.debug:
            node.get_logger().set_level(LoggingSeverity.DEBUG)
            node.get_logger().info('Debug logging enabled')

        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("\nKeyboard interrupt, shutting down mast LED controller node.")
    except ExternalShutdownException:
        if node:
            node.get_logger().info("External shutdown signal received, exiting gracefully.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
