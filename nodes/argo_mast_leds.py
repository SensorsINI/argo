#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
# ROS2 Mast LED Controller Node - LP5814DLR RGBW LED Driver

# This ROS2 node controls the LP5814DLR RGBW LED controller from Texas Instruments
# located on the same PCB as the wind sensor. The LED controller is used to provide
# visual status indication on the mast.
#
# Hardware Setup:
# - Uses I2C bus 0 (same as wind sensor)
# - LP5814DLR at I2C address 0x2c
# - Four LED channels: Red, Green, Blue, White (RGBW)
# - Run "sudo i2cdetect -y 0" to verify device connection
# - Device should show as address 2c in hex
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
# --diagnostic: Print comprehensive device diagnostics on startup (FLAG register, configuration, issues)

# Import path setup (must be before other imports)
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))

# Standard library imports
import time
import argparse
import argcomplete
from typing import Optional

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
# Reference: LP5814DLR datasheet Table 7-7 (Register Maps)
# ============================================================================

# I2C Configuration
I2C_BUS = 0  # I2C bus number (same as wind sensor)
LP5814_I2C_ADDRESS = 0x2c  # 7-bit I2C address of LP5814DLR (matches PCB schematic)

# LP5814DLR Register Addresses (from datasheet Table 7-7)
REG_CHIP_EN = 0x00          # CHIP_EN - Chip enable register
REG_DEV_CONFIG0 = 0x01      # DEV_CONFIG0 - Device configuration 0 (MAX_CURRENT)
REG_DEV_CONFIG1 = 0x02      # DEV_CONFIG1 - Device configuration 1 (OUT3_EN, OUT2_EN, OUT1_EN, OUT0_EN)
REG_DEV_CONFIG2 = 0x03      # DEV_CONFIG2 - Device configuration 2 (LED_FADE_TIME, OUTx_FADE_EN)
REG_DEV_CONFIG3 = 0x04      # DEV_CONFIG3 - Device configuration 3 (OUTx_EXP_EN, OUTx_AUTO_EN)
REG_DEV_CONFIG4 = 0x05      # DEV_CONFIG4 - Device configuration 4 (OUTx_ENGINE_CH)
REG_UPDATE_CMD = 0x0F       # UPDATE_CMD - Update command to make configuration take effect
REG_OUT0_DC = 0x14          # OUT0_DC - Output 0 dimming current (Red)
REG_OUT1_DC = 0x15          # OUT1_DC - Output 1 dimming current (Green)
REG_OUT2_DC = 0x16          # OUT2_DC - Output 2 dimming current (Blue)
REG_OUT3_DC = 0x17          # OUT3_DC - Output 3 dimming current (White)
REG_OUT0_MANUAL_PWM = 0x18  # OUT0_MANUAL_PWM - Manual PWM for output 0 (Red)
REG_OUT1_MANUAL_PWM = 0x19  # OUT1_MANUAL_PWM - Manual PWM for output 1 (Green)
REG_OUT2_MANUAL_PWM = 0x1A  # OUT2_MANUAL_PWM - Manual PWM for output 2 (Blue)
REG_OUT3_MANUAL_PWM = 0x1B  # OUT3_MANUAL_PWM - Manual PWM for output 3 (White)
REG_FLAG = 0x40             # FLAG - Status register (OUTx_EN, ENGINE_BUSY, TSD, POR)

# CHIP_EN Register Bits (REG_CHIP_EN = 0x00)
CHIP_EN_BIT = 0x01          # Bit 0: CHIP_EN - Enable chip

# DEV_CONFIG0 Register Bits (REG_DEV_CONFIG0 = 0x01)
# MAX_CURRENT bits: 00=25.5mA, 01=51mA, 10=76.5mA, 11=102mA
MAX_CURRENT_51MA = 0x01     # MAX_CURRENT = 01b (51mA maximum output LED current)

# DEV_CONFIG1 Register Bits (REG_DEV_CONFIG1 = 0x02)
OUT0_EN_BIT = 0x01           # Bit 0: OUT0_EN - Enable output 0 (Red)
OUT1_EN_BIT = 0x02           # Bit 1: OUT1_EN - Enable output 1 (Green)
OUT2_EN_BIT = 0x04           # Bit 2: OUT2_EN - Enable output 2 (Blue)
OUT3_EN_BIT = 0x08           # Bit 3: OUT3_EN - Enable output 3 (White)
OUT_ALL_EN = OUT0_EN_BIT | OUT1_EN_BIT | OUT2_EN_BIT | OUT3_EN_BIT  # Enable all outputs (0x0F)

# Maximum current settings for each output (dimming current registers)
# 0xFF = 51mA for red LED, 0xCC = 40.8mA for green/blue/white LEDs
OUT0_DC_MAX = 0xFF          # Red LED: 51mA maximum current
OUT1_DC_MAX = 0xCC          # Green LED: 40.8mA maximum current
OUT2_DC_MAX = 0xCC          # Blue LED: 40.8mA maximum current
OUT3_DC_MAX = 0xCC          # White LED: 40.8mA maximum current

# UPDATE_CMD Register (REG_UPDATE_CMD = 0x0F)
UPDATE_CMD_VALUE = 0x01     # Send UPDATE_CMD to make configuration take effect

# DEV_CONFIG3 Register Bits (REG_DEV_CONFIG3 = 0x04)
# OUTx_AUTO_EN bits: When set, output uses animation engine. When cleared, uses manual PWM.
# For manual mode, all OUTx_AUTO_EN bits must be 0
OUT0_AUTO_EN_BIT = 0x01     # Bit 0: OUT0_AUTO_EN - Auto mode for output 0
OUT1_AUTO_EN_BIT = 0x02     # Bit 1: OUT1_AUTO_EN - Auto mode for output 1
OUT2_AUTO_EN_BIT = 0x04     # Bit 2: OUT2_AUTO_EN - Auto mode for output 2
OUT3_AUTO_EN_BIT = 0x08     # Bit 3: OUT3_AUTO_EN - Auto mode for output 3
# For manual PWM mode, all AUTO_EN bits should be 0
DEV_CONFIG3_MANUAL_MODE = 0x00  # All AUTO_EN bits cleared = manual PWM mode

# FLAG Register Bits (REG_FLAG = 0x40)
FLAG_OUT0_EN_BIT = 0x01     # Bit 0: OUT0_EN status
FLAG_OUT1_EN_BIT = 0x02     # Bit 1: OUT1_EN status
FLAG_OUT2_EN_BIT = 0x04     # Bit 2: OUT2_EN status
FLAG_OUT3_EN_BIT = 0x08     # Bit 3: OUT3_EN status
FLAG_ENGINE_BUSY_BIT = 0x10 # Bit 4: ENGINE_BUSY status
FLAG_TSD_BIT = 0x20         # Bit 5: TSD (Thermal Shutdown) status
FLAG_POR_BIT = 0x40         # Bit 6: POR (Power-On Reset) status

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

    def __init__(self, debug_mode: bool = False, diagnostic_mode: bool = False):
        super().__init__('mastleds_node')

        self.get_logger().info('Initializing Mast LED Controller node...')

        # Debug mode flag
        self.debug_mode = debug_mode
        self.diagnostic_mode = diagnostic_mode

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
                # Print diagnostics if requested
                if self.diagnostic_mode:
                    self.print_diagnostics()
                # Perform startup test sequence
                self._startup_test_sequence()
            else:
                self.set_unhealthy("Failed to initialize LP5814DLR device")
                # Print diagnostics on failure if requested
                if self.diagnostic_mode:
                    self.print_diagnostics()
                self._initial_retry_count = 1
                self._start_retry_timer()
        else:
            self.set_unhealthy("Failed to open I2C bus")
            # Print diagnostics on failure if requested
            if self.diagnostic_mode:
                self.print_diagnostics()
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
        """Initialize LP5814DLR device following datasheet Section 8.2.3.1 Program Procedure.
        
        Procedure:
        1. Wait ~1ms after power up
        2. Set CHIP_EN = 1 to enable device
        3. Set MAX_CURRENT in DEV_CONFIG0
        4. Set maximum current for each output (OUTx_DC registers)
        5. Enable all outputs (DEV_CONFIG1)
        6. Send UPDATE_CMD to make configuration take effect
        7. Initialize manual PWM registers to 0 (off)
        """
        if not self.bus:
            return False

        try:
            if self.debug_mode:
                self.get_logger().debug(f"Initializing LP5814DLR at I2C address 0x{LP5814_I2C_ADDRESS:02x}")
            
            # Step 1: Wait ~1ms after power up (per datasheet Section 8.2.3.2)
            time.sleep(0.001)
            
            # Step 2: Set CHIP_EN = 1 to enable device (Write 0x01 to register 0x00)
            self._write_register(REG_CHIP_EN, CHIP_EN_BIT)
            if self.debug_mode:
                self.get_logger().debug(f"CHIP_EN: 0x{CHIP_EN_BIT:02x} (chip enabled)")
            
            # Step 3: Set MAX_CURRENT = 0x01 to set 51mA maximum output LED current (Write 0x01 to register 0x01)
            self._write_register(REG_DEV_CONFIG0, MAX_CURRENT_51MA)
            if self.debug_mode:
                self.get_logger().debug(f"DEV_CONFIG0: 0x{MAX_CURRENT_51MA:02x} (MAX_CURRENT = 51mA)")
            
            # Step 4: Set maximum current for each output
            # Red LED: 51mA (0xFF), Green/Blue/White: 40.8mA (0xCC)
            self._write_register(REG_OUT0_DC, OUT0_DC_MAX)  # Red: 51mA
            self._write_register(REG_OUT1_DC, OUT1_DC_MAX)  # Green: 40.8mA
            self._write_register(REG_OUT2_DC, OUT2_DC_MAX)  # Blue: 40.8mA
            self._write_register(REG_OUT3_DC, OUT3_DC_MAX)  # White: 40.8mA
            if self.debug_mode:
                self.get_logger().debug(f"OUTx_DC: Red=0x{OUT0_DC_MAX:02x}, Green=0x{OUT1_DC_MAX:02x}, Blue=0x{OUT2_DC_MAX:02x}, White=0x{OUT3_DC_MAX:02x}")
            
            # Step 5: Enable all 4 LEDs (Write 0x0F to register 0x02)
            self._write_register(REG_DEV_CONFIG1, OUT_ALL_EN)
            if self.debug_mode:
                self.get_logger().debug(f"DEV_CONFIG1: 0x{OUT_ALL_EN:02x} (all outputs enabled)")
            
            # Step 5b: Configure DEV_CONFIG3 for manual PWM mode (clear all OUTx_AUTO_EN bits)
            # This ensures outputs use manual PWM registers instead of animation engine
            self._write_register(REG_DEV_CONFIG3, DEV_CONFIG3_MANUAL_MODE)
            if self.debug_mode:
                self.get_logger().debug(f"DEV_CONFIG3: 0x{DEV_CONFIG3_MANUAL_MODE:02x} (manual PWM mode, all AUTO_EN cleared)")
            
            # Step 6: Send UPDATE_CMD to make device configuration take effect (Write 0x01 to register 0x0F)
            self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE)
            if self.debug_mode:
                self.get_logger().debug(f"UPDATE_CMD: 0x{UPDATE_CMD_VALUE:02x} (configuration applied)")
            
            # Step 7: Initialize all channels to off (manual PWM registers)
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)

            self.get_logger().info('LP5814DLR device initialized successfully')
            if self.debug_mode:
                self.get_logger().debug("All LED channels initialized to OFF (0x00)")
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
        
        if self.debug_mode:
            register_names = {
                REG_CHIP_EN: "CHIP_EN",
                REG_DEV_CONFIG0: "DEV_CONFIG0",
                REG_DEV_CONFIG1: "DEV_CONFIG1",
                REG_DEV_CONFIG2: "DEV_CONFIG2",
                REG_DEV_CONFIG3: "DEV_CONFIG3",
                REG_DEV_CONFIG4: "DEV_CONFIG4",
                REG_UPDATE_CMD: "UPDATE_CMD",
                REG_OUT0_DC: "OUT0_DC (Red max current)",
                REG_OUT1_DC: "OUT1_DC (Green max current)",
                REG_OUT2_DC: "OUT2_DC (Blue max current)",
                REG_OUT3_DC: "OUT3_DC (White max current)",
                REG_OUT0_MANUAL_PWM: "OUT0_MANUAL_PWM (Red)",
                REG_OUT1_MANUAL_PWM: "OUT1_MANUAL_PWM (Green)",
                REG_OUT2_MANUAL_PWM: "OUT2_MANUAL_PWM (Blue)",
                REG_OUT3_MANUAL_PWM: "OUT3_MANUAL_PWM (White)",
                REG_FLAG: "FLAG (status)"
            }
            reg_name = register_names.get(register, f"REG_0x{register:02x}")
            self.get_logger().debug(
                f"I2C write: addr=0x{LP5814_I2C_ADDRESS:02x}, {reg_name}=0x{value:02x} ({value})")
        
        self.bus.write_byte_data(LP5814_I2C_ADDRESS, register, value)
    
    def _read_register(self, register: int) -> Optional[int]:
        """Read a single byte from an LP5814DLR register.
        
        Returns:
            Register value (0-255) or None if read failed
        """
        if not self.bus:
            return None
        
        try:
            value = self.bus.read_byte_data(LP5814_I2C_ADDRESS, register)
            
            if self.debug_mode:
                register_names = {
                    REG_CHIP_EN: "CHIP_EN",
                    REG_DEV_CONFIG0: "DEV_CONFIG0",
                    REG_DEV_CONFIG1: "DEV_CONFIG1",
                    REG_DEV_CONFIG2: "DEV_CONFIG2",
                    REG_DEV_CONFIG3: "DEV_CONFIG3",
                    REG_DEV_CONFIG4: "DEV_CONFIG4",
                    REG_OUT0_DC: "OUT0_DC",
                    REG_OUT1_DC: "OUT1_DC",
                    REG_OUT2_DC: "OUT2_DC",
                    REG_OUT3_DC: "OUT3_DC",
                    REG_OUT0_MANUAL_PWM: "OUT0_MANUAL_PWM",
                    REG_OUT1_MANUAL_PWM: "OUT1_MANUAL_PWM",
                    REG_OUT2_MANUAL_PWM: "OUT2_MANUAL_PWM",
                    REG_OUT3_MANUAL_PWM: "OUT3_MANUAL_PWM",
                    REG_FLAG: "FLAG (status)"
                }
                reg_name = register_names.get(register, f"REG_0x{register:02x}")
                self.get_logger().debug(
                    f"I2C read: addr=0x{LP5814_I2C_ADDRESS:02x}, {reg_name}=0x{value:02x} ({value})")
            
            return value
        except IOError as e:
            self._handle_i2c_error(f"Failed to read register 0x{register:02x}: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading register 0x{register:02x}: {e}")
            return None

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
        self.set_unhealthy(f"LP5814DLR LED controller not found at I2C address 0x{LP5814_I2C_ADDRESS:02x} - using original wind sensor without LED controller")
        self.get_logger().error(
            f"LP5814DLR LED controller not found at I2C address 0x{LP5814_I2C_ADDRESS:02x}. "
            "This indicates the original wind sensor (without LED controller) is in use. "
            "Node will continue running in low-CPU mode, ignoring LED control subscriptions. "
            "Hardware will be checked periodically for device appearance.")
        
        # Switch retry timer to low-CPU interval if it exists
        if hasattr(self, 'retry_timer') and self.retry_timer:
            self.retry_timer.cancel()
            # Start low-CPU periodic check timer (60s interval)
            self.retry_timer = self.create_timer(I2C_LOW_CPU_CHECK_INTERVAL_S, self._retry_initialization)

    def read_device_status(self) -> Optional[dict]:
        """Read and decode the FLAG register to diagnose device status.
        
        Returns:
            Dictionary with status information, or None if read failed:
            {
                'flag_register': int,  # Raw FLAG register value
                'outputs_enabled': [bool, bool, bool, bool],  # [OUT0, OUT1, OUT2, OUT3]
                'engine_busy': bool,
                'thermal_shutdown': bool,
                'power_on_reset': bool,
                'status_ok': bool  # True if no errors detected
            }
        """
        flag_value = self._read_register(REG_FLAG)
        if flag_value is None:
            return None
        
        status = {
            'flag_register': flag_value,
            'outputs_enabled': [
                bool(flag_value & FLAG_OUT0_EN_BIT),
                bool(flag_value & FLAG_OUT1_EN_BIT),
                bool(flag_value & FLAG_OUT2_EN_BIT),
                bool(flag_value & FLAG_OUT3_EN_BIT)
            ],
            'engine_busy': bool(flag_value & FLAG_ENGINE_BUSY_BIT),
            'thermal_shutdown': bool(flag_value & FLAG_TSD_BIT),
            'power_on_reset': bool(flag_value & FLAG_POR_BIT),
            'status_ok': not (bool(flag_value & FLAG_TSD_BIT) or bool(flag_value & FLAG_POR_BIT))
        }
        
        return status
    
    def diagnose_device(self) -> dict:
        """Perform comprehensive device diagnostics.
        
        Returns:
            Dictionary with diagnostic information including:
            - I2C communication status
            - Device configuration registers
            - Current PWM values
            - FLAG register status
            - Recommendations for fixing issues
        """
        diagnostics = {
            'i2c_communication': False,
            'device_initialized': False,
            'chip_enabled': False,
            'outputs_enabled': [False, False, False, False],
            'manual_mode': False,
            'current_pwm': [0, 0, 0, 0],
            'flag_status': None,
            'issues': [],
            'recommendations': []
        }
        
        if not self.bus:
            diagnostics['issues'].append("I2C bus not initialized")
            diagnostics['recommendations'].append("Check I2C bus configuration and permissions")
            return diagnostics
        
        # Test I2C communication by reading CHIP_EN register
        try:
            chip_en = self._read_register(REG_CHIP_EN)
            if chip_en is not None:
                diagnostics['i2c_communication'] = True
                diagnostics['chip_enabled'] = bool(chip_en & CHIP_EN_BIT)
            else:
                diagnostics['issues'].append("Cannot read CHIP_EN register - I2C communication failed")
                diagnostics['recommendations'].append("Check I2C wiring, power supply, and device address (should be 0x2c)")
                return diagnostics
        except Exception as e:
            diagnostics['issues'].append(f"I2C read error: {e}")
            diagnostics['recommendations'].append("Check I2C bus connection and device power")
            return diagnostics
        
        # Read device configuration
        try:
            dev_config1 = self._read_register(REG_DEV_CONFIG1)
            if dev_config1 is not None:
                diagnostics['outputs_enabled'] = [
                    bool(dev_config1 & OUT0_EN_BIT),
                    bool(dev_config1 & OUT1_EN_BIT),
                    bool(dev_config1 & OUT2_EN_BIT),
                    bool(dev_config1 & OUT3_EN_BIT)
                ]
            
            dev_config3 = self._read_register(REG_DEV_CONFIG3)
            if dev_config3 is not None:
                # Manual mode = all AUTO_EN bits cleared
                diagnostics['manual_mode'] = (dev_config3 & 0x0F) == 0
            
            # Read current PWM values
            pwm0 = self._read_register(REG_OUT0_MANUAL_PWM)
            pwm1 = self._read_register(REG_OUT1_MANUAL_PWM)
            pwm2 = self._read_register(REG_OUT2_MANUAL_PWM)
            pwm3 = self._read_register(REG_OUT3_MANUAL_PWM)
            
            if pwm0 is not None:
                diagnostics['current_pwm'] = [
                    pwm0 if pwm0 is not None else 0,
                    pwm1 if pwm1 is not None else 0,
                    pwm2 if pwm2 is not None else 0,
                    pwm3 if pwm3 is not None else 0
                ]
        except Exception as e:
            diagnostics['issues'].append(f"Error reading configuration registers: {e}")
        
        # Read FLAG register for status
        diagnostics['flag_status'] = self.read_device_status()
        
        if diagnostics['flag_status']:
            if diagnostics['flag_status']['thermal_shutdown']:
                diagnostics['issues'].append("Thermal shutdown detected - device may be overheating")
                diagnostics['recommendations'].append("Check device temperature and ensure adequate cooling")
            
            if diagnostics['flag_status']['power_on_reset']:
                diagnostics['issues'].append("Power-on reset detected - device may have lost power")
                diagnostics['recommendations'].append("Check power supply connections and voltage")
        
        # Check for common issues
        if not diagnostics['chip_enabled']:
            diagnostics['issues'].append("Chip not enabled (CHIP_EN bit not set)")
            diagnostics['recommendations'].append("Re-initialize device or check initialization sequence")
        
        if not diagnostics['manual_mode']:
            diagnostics['issues'].append("Device not in manual PWM mode (AUTO_EN bits may be set)")
            diagnostics['recommendations'].append("Set DEV_CONFIG3 to 0x00 to enable manual PWM mode")
        
        if not any(diagnostics['outputs_enabled']):
            diagnostics['issues'].append("No outputs enabled in DEV_CONFIG1")
            diagnostics['recommendations'].append("Set DEV_CONFIG1 to 0x0F to enable all outputs")
        
        if all(pwm == 0 for pwm in diagnostics['current_pwm']):
            diagnostics['issues'].append("All PWM values are 0 - LEDs will be off")
            diagnostics['recommendations'].append("Send brightness commands to /mastled_r, /mastled_g, /mastled_b, /mastled_w topics")
        
        diagnostics['device_initialized'] = (
            diagnostics['chip_enabled'] and
            diagnostics['manual_mode'] and
            any(diagnostics['outputs_enabled'])
        )
        
        return diagnostics
    
    def print_diagnostics(self):
        """Print comprehensive device diagnostics to logger."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("LP5814DLR Device Diagnostics")
        self.get_logger().info("=" * 60)
        
        diag = self.diagnose_device()
        
        self.get_logger().info(f"I2C Communication: {'OK' if diag['i2c_communication'] else 'FAILED'}")
        self.get_logger().info(f"Device Initialized: {'YES' if diag['device_initialized'] else 'NO'}")
        self.get_logger().info(f"Chip Enabled: {'YES' if diag['chip_enabled'] else 'NO'}")
        self.get_logger().info(f"Manual Mode: {'YES' if diag['manual_mode'] else 'NO'}")
        self.get_logger().info(f"Outputs Enabled: R={diag['outputs_enabled'][0]}, G={diag['outputs_enabled'][1]}, "
                              f"B={diag['outputs_enabled'][2]}, W={diag['outputs_enabled'][3]}")
        self.get_logger().info(f"Current PWM: R={diag['current_pwm'][0]}, G={diag['current_pwm'][1]}, "
                              f"B={diag['current_pwm'][2]}, W={diag['current_pwm'][3]}")
        
        if diag['flag_status']:
            flag = diag['flag_status']
            self.get_logger().info(f"FLAG Register: 0x{flag['flag_register']:02x}")
            self.get_logger().info(f"  Output Status: R={flag['outputs_enabled'][0]}, G={flag['outputs_enabled'][1]}, "
                                  f"B={flag['outputs_enabled'][2]}, W={flag['outputs_enabled'][3]}")
            self.get_logger().info(f"  Engine Busy: {flag['engine_busy']}")
            self.get_logger().info(f"  Thermal Shutdown: {flag['thermal_shutdown']}")
            self.get_logger().info(f"  Power-On Reset: {flag['power_on_reset']}")
            self.get_logger().info(f"  Status OK: {flag['status_ok']}")
        
        if diag['issues']:
            self.get_logger().warn("Issues Detected:")
            for issue in diag['issues']:
                self.get_logger().warn(f"  - {issue}")
        
        if diag['recommendations']:
            self.get_logger().info("Recommendations:")
            for rec in diag['recommendations']:
                self.get_logger().info(f"  - {rec}")
        
        self.get_logger().info("=" * 60)
    
    def _startup_test_sequence(self):
        """Perform startup test sequence to verify all LEDs are working.
        
        Sequence:
        1. All LEDs at maximum brightness for 1 second
        2. Red LED only for 1 second
        3. Green LED only for 1 second
        4. Blue LED only for 1 second
        5. White LED only for 1 second
        6. All LEDs off
        """
        if not self.device_ready or not self.bus:
            return
        
        try:
            self.get_logger().info("Starting LED test sequence...")
            
            # Step 1: All LEDs at maximum brightness for 1 second
            self.get_logger().info("Test: All LEDs maximum brightness (1s)")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Red
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Green
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Blue
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # White
            time.sleep(1.0)
            
            # Step 2: Red LED only for 1 second
            self.get_logger().info("Test: Red LED only (1s)")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Red ON
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)
            
            # Step 3: Green LED only for 1 second
            self.get_logger().info("Test: Green LED only (1s)")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Green ON
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)
            
            # Step 4: Blue LED only for 1 second
            self.get_logger().info("Test: Blue LED only (1s)")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # Blue ON
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)
            
            # Step 5: White LED only for 1 second
            self.get_logger().info("Test: White LED only (1s)")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MAX)  # White ON
            time.sleep(1.0)
            
            # Step 6: All LEDs off
            self.get_logger().info("Test: All LEDs off")
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)  # White OFF
            
            # Update internal state to match hardware
            self._red_brightness = 0.0
            self._green_brightness = 0.0
            self._blue_brightness = 0.0
            self._white_brightness = 0.0
            
            self.get_logger().info("LED test sequence complete")
            
        except Exception as e:
            self.get_logger().error(f"Error during startup test sequence: {e}")

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
            self._write_register(REG_OUT0_MANUAL_PWM, self._brightness_to_hw(self._red_brightness))
            self._write_register(REG_OUT1_MANUAL_PWM, self._brightness_to_hw(self._green_brightness))
            self._write_register(REG_OUT2_MANUAL_PWM, self._brightness_to_hw(self._blue_brightness))
            self._write_register(REG_OUT3_MANUAL_PWM, self._brightness_to_hw(self._white_brightness))

            # Reset error counter on successful write
            self._consecutive_i2c_errors = 0

            if self.debug_mode:
                hw_r = self._brightness_to_hw(self._red_brightness)
                hw_g = self._brightness_to_hw(self._green_brightness)
                hw_b = self._brightness_to_hw(self._blue_brightness)
                hw_w = self._brightness_to_hw(self._white_brightness)
                self.get_logger().debug(
                    f"LED update: R={self._red_brightness:.3f} ({hw_r}) "
                    f"G={self._green_brightness:.3f} ({hw_g}) "
                    f"B={self._blue_brightness:.3f} ({hw_b}) "
                    f"W={self._white_brightness:.3f} ({hw_w})")

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
  - LP5814DLR at I2C address 0x2c
  - Four LED channels: Red, Green, Blue, White (RGBW)
  - Run "sudo i2cdetect -y 0" to verify device connection
  - Device should show as address 2c in hex

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
  - LP5814_I2C_ADDRESS: I2C address of LP5814DLR (default: 0x2c)
  - REG_OUT0_MANUAL_PWM through REG_OUT3_MANUAL_PWM: Manual PWM register addresses for R, G, B, W channels
  - BRIGHTNESS_MIN/MAX: Normalized brightness range (0.0-1.0)
  - BRIGHTNESS_HW_MIN/MAX: Hardware brightness range (0-255)

Debug Mode:
  Use --debug flag to enable detailed logging:
  - I2C register writes (address, register name, value in hex and decimal)
  - LED brightness updates (normalized 0.0-1.0 and hardware 0-255 values)
  - All I2C operations are logged for troubleshooting

Diagnostic Mode:
  Use --diagnostic flag to print comprehensive device diagnostics on startup:
  - I2C communication status
  - Device configuration registers (CHIP_EN, DEV_CONFIG1, DEV_CONFIG3)
  - Current PWM values for all channels
  - FLAG register status (output enables, thermal shutdown, power-on reset)
  - Issues detected and recommendations for fixing problems
  - Useful for troubleshooting when LEDs don't turn on
            """
        )

        # Add diagnostic argument
        parser.add_argument('--diagnostic', action='store_true',
                          help='Print comprehensive device diagnostics on startup')
        
        parsed_args, unknown_args = parser.parse_known_args(args)
        argcomplete.autocomplete(parser)

        rclpy.init(args=unknown_args)

        node = MastLEDsNode(debug_mode=parsed_args.debug, diagnostic_mode=parsed_args.diagnostic)

        if parsed_args.debug:
            node.get_logger().set_level(LoggingSeverity.DEBUG)
            node.get_logger().info('Debug logging enabled')
        
        if parsed_args.diagnostic:
            node.get_logger().info('Diagnostic mode enabled - will print device status on startup')

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
