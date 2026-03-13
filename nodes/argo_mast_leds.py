#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
# ROS2 Mast LED Controller Node - PCA9632 RGBW LED Driver

# This ROS2 node controls the PCA9632 4-bit I2C LED driver (NXP) located on the same
# PCB as the wind sensor. The LED controller is used to provide visual status on the mast.
#
# Hardware Setup:
# - Uses I2C bus 1 (same as wind sensor, different from other nodes)
#   for electrical isolation of the LED controller in case of short circuit from the mast and salt water intrusion.
# - PCA9632 individual address 0x62 (hardware pins A1 A0; 0x70 is LED All Call, not used here)
# - Four LED channels: Red, Green, Blue, White (RGBW) via LED0-LED3
# - Run "i2cdetect -y 1" to verify: expect 0x62 (individual) and 0x70 (All Call) for PCA9632
#
# Features:
# - Four individual subscriptions (one per color channel: R, G, B, W)
# - One combined 4-component subscription for synchronized RGBW control
# - Power button RGB mirroring: mirrors RGB channels from power button LEDs
# - Individual brightness mode: 8-bit PWM per channel (1.5625 kHz)
# - Normalized brightness (0.0-1.0) converted to 8-bit (0-255) for hardware
# - Automatic I2C error recovery with health monitoring
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
#   White channel is not affected by power button mirroring
#
# Command line options:
# --debug: Enable debug logging of LED values and I2C operations
# --diagnostic: Print comprehensive device diagnostics on startup (MODE1, LEDOUT, PWM, etc.)

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
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Vector3

# I2C communication
import smbus

# Import ArgoBaseNode for standardized functionality
from argo_base_node import ArgoBaseNode

# ============================================================================
# Constants - PCA9632 Configuration
# Reference: NXP PCA9632 datasheet (Table 6 Register summary, Section 7.3)
# ============================================================================

# I2C Configuration
I2C_BUS = 1  # I2C bus number (same as wind sensor)
# 7-bit individual address (hardware pins A1 A0; 0x70 is LED All Call, active at power-up)
PCA9632_I2C_ADDRESS = 0x62

# PCA9632 register addresses (control byte D[3:0]; first byte after I2C addr)
REG_MODE1 = 0x00    # Mode register 1 (SLEEP bit 4; 0 = normal, 1 = low power)
REG_MODE2 = 0x01    # Mode register 2 (OUTDRV, INVRT, OCH)
REG_PWM0 = 0x02     # LED0 brightness (Red)
REG_PWM1 = 0x03     # LED1 brightness (Green)
REG_PWM2 = 0x04     # LED2 brightness (Blue)
REG_PWM3 = 0x05     # LED3 brightness (White)
REG_GRPPWM = 0x06   # Group duty cycle (unused in individual brightness mode)
REG_GRPFREQ = 0x07  # Group frequency (unused)
REG_LEDOUT = 0x08   # LED output state: LDRx 00=off, 01=on, 10=individual PWM, 11=individual+group

# MODE1: bit 4 SLEEP (1 = oscillator off). We clear SLEEP for normal operation.
MODE1_SLEEP_BIT = 0x10   # Default 1 at power-up; write 0 to wake
MODE1_NORMAL = 0x01      # SLEEP=0, oscillator on (default has ALLCALL=1, we keep 0x01)

# LEDOUT: two bits per channel. 10 = individual PWM control via PWMx register.
LDR_OFF = 0x00       # 00 - LED off
LDR_ON = 0x01        # 01 - LED fully on (no PWM)
LDR_PWM = 0x02       # 10 - Individual brightness via PWMx
LDR_PWM_GRP = 0x03   # 11 - Individual + group (unused here)
# All four channels in individual PWM: LDR3 LDR2 LDR1 LDR0 = 10 10 10 10 = 0xAA
LEDOUT_ALL_INDIVIDUAL_PWM = 0xAA

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
TOPIC_POWER_BUTTON_RGB = '/argo/power_button/rgb'  # Legacy - kept for backward compatibility
TOPIC_POWER_BUTTON_PATTERN = '/argo/power_button/pattern'  # New pattern-based topic

# I2C Error Recovery
I2C_RETRY_DELAY_S = 1.0  # Delay between I2C retry attempts in seconds (initial retries)
I2C_ERROR_LOG_THROTTLE_S = 5.0  # Maximum I2C error logging frequency in seconds
I2C_MAX_INITIAL_RETRIES = 3  # Maximum retries during initial startup before entering low-CPU mode
I2C_LOW_CPU_CHECK_INTERVAL_S = 60.0  # Interval for checking device availability in low-CPU mode (60 seconds)


class MastLEDsNode(ArgoBaseNode):
    """ROS2 node for controlling mast RGBW LEDs via PCA9632 driver."""

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
        
        # Pattern-based mirroring state
        self._pattern_active = False
        self._pattern_category = None
        self._pattern_string = None
        self._pattern_cycle_duration = 3.0
        self._pattern_slot_duration = 0.3
        self._pattern_on_period = 0.03
        self._pattern_off_period = 0.27
        self._pattern_thread = None
        self._pattern_stop_event = None

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
                self.set_unhealthy("Failed to initialize PCA9632 device")
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
        """Initialize PCA9632 device: wake from sleep, set LEDOUT for individual PWM, PWMx=0.
        
        Procedure (datasheet Section 7.3):
        1. MODE1: clear SLEEP bit (bit 4) to start oscillator
        2. Wait up to 500 µs for oscillator
        3. LEDOUT: set all channels to individual PWM mode (LDRx = 10)
        4. PWM0–PWM3: set to 0 (off)
        """
        if not self.bus:
            return False

        try:
            if self.debug_mode:
                self.get_logger().debug(f"Initializing PCA9632 at I2C address 0x{PCA9632_I2C_ADDRESS:02x}")
            
            # Step 1: Wake from sleep (MODE1 default 0x11; clear bit 4 → 0x01)
            self._write_register(REG_MODE1, MODE1_NORMAL)
            if self.debug_mode:
                self.get_logger().debug("MODE1: 0x01 (SLEEP=0, oscillator on)")
            
            # Step 2: Datasheet: allow up to 500 µs for oscillator
            time.sleep(0.001)
            
            # Step 3: LEDOUT = 0xAA → all four channels in individual PWM mode (LDRx = 10)
            self._write_register(REG_LEDOUT, LEDOUT_ALL_INDIVIDUAL_PWM)
            if self.debug_mode:
                self.get_logger().debug(f"LEDOUT: 0x{LEDOUT_ALL_INDIVIDUAL_PWM:02x} (individual PWM)")
            
            # Step 4: All PWM channels off
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN)
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN)
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN)
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN)

            self.get_logger().info('PCA9632 device initialized successfully')
            if self.debug_mode:
                self.get_logger().debug("All LED channels initialized to OFF (0x00)")
            return True

        except IOError as e:
            self._handle_i2c_error(f"Failed to initialize PCA9632: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error initializing device: {e}")
            return False

    def _write_register(self, register: int, value: int):
        """Write a single byte to a PCA9632 register (control byte = register select, then data)."""
        if not self.bus:
            raise IOError("I2C bus not initialized")
        
        value = max(0, min(255, int(value)))
        
        if self.debug_mode:
            register_names = {
                REG_MODE1: "MODE1",
                REG_MODE2: "MODE2",
                REG_PWM0: "PWM0 (Red)",
                REG_PWM1: "PWM1 (Green)",
                REG_PWM2: "PWM2 (Blue)",
                REG_PWM3: "PWM3 (White)",
                REG_GRPPWM: "GRPPWM",
                REG_GRPFREQ: "GRPFREQ",
                REG_LEDOUT: "LEDOUT",
            }
            reg_name = register_names.get(register, f"REG_0x{register:02x}")
            self.get_logger().debug(
                f"I2C write: addr=0x{PCA9632_I2C_ADDRESS:02x}, {reg_name}=0x{value:02x} ({value})")
        
        self.bus.write_byte_data(PCA9632_I2C_ADDRESS, register, value)

    def _read_register(self, register: int) -> Optional[int]:
        """Read a single byte from a PCA9632 register (control byte selects register).
        
        Returns:
            Register value (0-255) or None if read failed
        """
        if not self.bus:
            return None

        try:
            value = self.bus.read_byte_data(PCA9632_I2C_ADDRESS, register)

            if self.debug_mode:
                register_names = {
                    REG_MODE1: "MODE1",
                    REG_MODE2: "MODE2",
                    REG_PWM0: "PWM0",
                    REG_PWM1: "PWM1",
                    REG_PWM2: "PWM2",
                    REG_PWM3: "PWM3",
                    REG_LEDOUT: "LEDOUT",
                }
                reg_name = register_names.get(register, f"REG_0x{register:02x}")
                self.get_logger().debug(
                    f"I2C read: addr=0x{PCA9632_I2C_ADDRESS:02x}, {reg_name}=0x{value:02x} ({value})")

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
        self.set_unhealthy(f"PCA9632 LED controller not found at I2C address 0x{PCA9632_I2C_ADDRESS:02x} - using wind sensor without LED controller")
        self.get_logger().error(
            f"PCA9632 LED controller not found at I2C address 0x{PCA9632_I2C_ADDRESS:02x}. "
            "Node will continue running in low-CPU mode, ignoring LED control subscriptions. "
            "Hardware will be checked periodically for device appearance.")
        
        # Switch retry timer to low-CPU interval if it exists
        if hasattr(self, 'retry_timer') and self.retry_timer:
            self.retry_timer.cancel()
            # Start low-CPU periodic check timer (60s interval)
            self.retry_timer = self.create_timer(I2C_LOW_CPU_CHECK_INTERVAL_S, self._retry_initialization)

    def read_device_status(self) -> Optional[dict]:
        """Read MODE1 and LEDOUT to diagnose device status.
        
        Returns:
            Dictionary with status information, or None if read failed:
            {
                'mode1': int,
                'ledout': int,
                'sleep': bool,  # True if oscillator off
                'individual_pwm': bool,  # True if LEDOUT == 0xAA
                'status_ok': bool
            }
        """
        mode1 = self._read_register(REG_MODE1)
        ledout = self._read_register(REG_LEDOUT)
        if mode1 is None or ledout is None:
            return None

        sleep = bool(mode1 & MODE1_SLEEP_BIT)
        status = {
            'mode1': mode1,
            'ledout': ledout,
            'sleep': sleep,
            'individual_pwm': (ledout == LEDOUT_ALL_INDIVIDUAL_PWM),
            'status_ok': not sleep and (ledout == LEDOUT_ALL_INDIVIDUAL_PWM),
        }
        return status
    
    def diagnose_device(self) -> dict:
        """Perform comprehensive device diagnostics.
        
        Returns:
            Dictionary with diagnostic information including:
            - I2C communication status
            - MODE1, LEDOUT, current PWM values
            - Recommendations for fixing issues
        """
        diagnostics = {
            'i2c_communication': False,
            'device_initialized': False,
            'mode1_ok': False,
            'ledout_ok': False,
            'current_pwm': [0, 0, 0, 0],
            'device_status': None,
            'issues': [],
            'recommendations': []
        }

        if not self.bus:
            diagnostics['issues'].append("I2C bus not initialized")
            diagnostics['recommendations'].append("Check I2C bus configuration and permissions")
            return diagnostics

        try:
            mode1 = self._read_register(REG_MODE1)
            if mode1 is not None:
                diagnostics['i2c_communication'] = True
                diagnostics['mode1_ok'] = not (mode1 & MODE1_SLEEP_BIT)
            else:
                diagnostics['issues'].append("Cannot read MODE1 - I2C communication failed")
                diagnostics['recommendations'].append("Check I2C wiring, power supply, and device address (0x62 individual, 0x70 All Call)")
                return diagnostics
        except Exception as e:
            diagnostics['issues'].append(f"I2C read error: {e}")
            diagnostics['recommendations'].append("Check I2C bus connection and device power")
            return diagnostics

        try:
            ledout = self._read_register(REG_LEDOUT)
            if ledout is not None:
                diagnostics['ledout_ok'] = (ledout == LEDOUT_ALL_INDIVIDUAL_PWM)
            pwm0 = self._read_register(REG_PWM0)
            pwm1 = self._read_register(REG_PWM1)
            pwm2 = self._read_register(REG_PWM2)
            pwm3 = self._read_register(REG_PWM3)
            if pwm0 is not None:
                diagnostics['current_pwm'] = [
                    pwm0 if pwm0 is not None else 0,
                    pwm1 if pwm1 is not None else 0,
                    pwm2 if pwm2 is not None else 0,
                    pwm3 if pwm3 is not None else 0
                ]
        except Exception as e:
            diagnostics['issues'].append(f"Error reading registers: {e}")

        diagnostics['device_status'] = self.read_device_status()

        if diagnostics['device_status'] and diagnostics['device_status'].get('sleep'):
            diagnostics['issues'].append("Device in sleep (MODE1 SLEEP=1) - oscillator off")
            diagnostics['recommendations'].append("Write MODE1=0x01 to wake device")
        if diagnostics['device_status'] and not diagnostics['device_status'].get('individual_pwm'):
            diagnostics['issues'].append("LEDOUT not in individual PWM mode (expected 0xAA)")
            diagnostics['recommendations'].append("Write LEDOUT=0xAA for individual brightness control")
        if all(p == 0 for p in diagnostics['current_pwm']):
            diagnostics['issues'].append("All PWM values are 0 - LEDs off")
            diagnostics['recommendations'].append("Send brightness to /mastled_r, /mastled_g, /mastled_b, /mastled_w")

        diagnostics['device_initialized'] = (
            diagnostics['i2c_communication'] and
            diagnostics['mode1_ok'] and
            diagnostics['ledout_ok']
        )
        return diagnostics
    
    def print_diagnostics(self):
        """Print comprehensive device diagnostics to logger."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("PCA9632 Device Diagnostics")
        self.get_logger().info("=" * 60)

        diag = self.diagnose_device()

        self.get_logger().info(f"I2C Communication: {'OK' if diag['i2c_communication'] else 'FAILED'}")
        self.get_logger().info(f"Device Initialized: {'YES' if diag['device_initialized'] else 'NO'}")
        self.get_logger().info(f"MODE1 OK (not sleep): {'YES' if diag['mode1_ok'] else 'NO'}")
        self.get_logger().info(f"LEDOUT individual PWM: {'YES' if diag['ledout_ok'] else 'NO'}")
        self.get_logger().info(f"Current PWM: R={diag['current_pwm'][0]}, G={diag['current_pwm'][1]}, "
                              f"B={diag['current_pwm'][2]}, W={diag['current_pwm'][3]}")

        if diag['device_status']:
            st = diag['device_status']
            self.get_logger().info(f"MODE1: 0x{st['mode1']:02x}, LEDOUT: 0x{st['ledout']:02x}")
            self.get_logger().info(f"  Sleep: {st['sleep']}, Individual PWM: {st['individual_pwm']}, Status OK: {st['status_ok']}")

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
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MAX)  # Red
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MAX)  # Green
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MAX)  # Blue
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MAX)  # White
            time.sleep(1.0)

            # Step 2: Red LED only for 1 second
            self.get_logger().info("Test: Red LED only (1s)")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MAX)  # Red ON
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)

            # Step 3: Green LED only for 1 second
            self.get_logger().info("Test: Green LED only (1s)")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MAX)  # Green ON
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)

            # Step 4: Blue LED only for 1 second
            self.get_logger().info("Test: Blue LED only (1s)")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MAX)  # Blue ON
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN)  # White OFF
            time.sleep(1.0)

            # Step 5: White LED only for 1 second
            self.get_logger().info("Test: White LED only (1s)")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MAX)  # White ON
            time.sleep(1.0)

            # Step 6: All LEDs off
            self.get_logger().info("Test: All LEDs off")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN)  # Red OFF
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN)  # Green OFF
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN)  # Blue OFF
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN)  # White OFF
            
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
            self._write_register(REG_PWM0, self._brightness_to_hw(self._red_brightness))
            self._write_register(REG_PWM1, self._brightness_to_hw(self._green_brightness))
            self._write_register(REG_PWM2, self._brightness_to_hw(self._blue_brightness))
            self._write_register(REG_PWM3, self._brightness_to_hw(self._white_brightness))

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

        # Power button RGB mirroring subscription (legacy - kept for backward compatibility)
        self.sub_power_button_rgb = self.create_subscription(
            Vector3, TOPIC_POWER_BUTTON_RGB, self._power_button_rgb_callback, 10)
        
        # Power button pattern subscription (new pattern-based approach)
        self.sub_power_button_pattern = self.create_subscription(
            String, TOPIC_POWER_BUTTON_PATTERN, self._power_button_pattern_callback, 10)

        self.get_logger().info(f'Subscribed to {TOPIC_RED}, {TOPIC_GREEN}, {TOPIC_BLUE}, {TOPIC_WHITE}, {TOPIC_RGBW}, {TOPIC_POWER_BUTTON_RGB}, {TOPIC_POWER_BUTTON_PATTERN}')

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
        """Callback for power button RGB mirroring subscription (legacy).
        
        DEPRECATED: This callback is kept for backward compatibility but should
        not be used. Use pattern-based mirroring via _power_button_pattern_callback instead.
        
        Mirrors RGB channels from power button LEDs to mast head LEDs.
        x = red, y = green, z = blue (normalized 0.0-1.0).
        White channel is not affected by power button mirroring.
        
        Silently ignores updates if device unavailable (low-CPU mode).
        """
        # Only process if pattern-based mirroring is not active
        if not self._pattern_active:
            # Update RGB channels from power button (Vector3: x=R, y=G, z=B)
            # Update internal state even if device unavailable (allows recovery)
            self._red_brightness = self._normalize_brightness(msg.x)
            self._green_brightness = self._normalize_brightness(msg.y)
            self._blue_brightness = self._normalize_brightness(msg.z)
            # White channel is NOT affected by power button mirroring
            # (power button only has RGB, not RGBW)
            # Hardware update will be ignored if device unavailable
            self._update_hardware()
    
    def _power_button_pattern_callback(self, msg: String):
        """Callback for power button pattern subscription.
        
        Receives pattern description and recreates it locally on mast LEDs.
        Pattern format: JSON string with category, pattern, cycle_duration, etc.
        Recreates pattern with max 3s lag to stay synchronized.
        """
        try:
            import json
            import threading
            import time
            
            pattern_data = json.loads(msg.data)
            category = pattern_data.get('category', 'heartbeat')
            pattern = pattern_data.get('pattern', '')
            cycle_duration = pattern_data.get('cycle_duration', 3.0)
            slot_duration = pattern_data.get('slot_duration', 0.3)
            on_period = pattern_data.get('on_period', 0.03)
            off_period = pattern_data.get('off_period', 0.27)
            
            # Stop any existing pattern thread
            if self._pattern_active and self._pattern_stop_event:
                self._pattern_stop_event.set()
                if self._pattern_thread and self._pattern_thread.is_alive():
                    self._pattern_thread.join(timeout=0.5)
            
            # Store pattern parameters
            self._pattern_category = category
            self._pattern_string = pattern
            self._pattern_cycle_duration = cycle_duration
            self._pattern_slot_duration = slot_duration
            self._pattern_on_period = on_period
            self._pattern_off_period = off_period
            
            # Start pattern recreation thread
            self._pattern_stop_event = threading.Event()
            self._pattern_active = True
            self._pattern_thread = threading.Thread(
                target=self._recreate_pattern,
                args=(pattern, slot_duration, on_period, off_period),
                daemon=True
            )
            self._pattern_thread.start()
            
        except Exception as e:
            self.get_logger().warn(f"Failed to process pattern message: {e}")
    
    def _recreate_pattern(self, pattern: str, slot_duration: float, on_period: float, off_period: float):
        """Recreate LED pattern locally on mast LEDs.
        
        Executes the pattern continuously until stopped.
        Pattern string: 'g'=green, 'r'=red, 'b'=blue, '.'=off
        """
        try:
            while not self._pattern_stop_event.is_set():
                for slot_char in pattern:
                    if self._pattern_stop_event.is_set():
                        break
                    
                    # Set LEDs for this slot
                    red_on = (slot_char == 'r')
                    green_on = (slot_char == 'g')
                    blue_on = (slot_char == 'b')
                    
                    # ON period
                    self._red_brightness = 1.0 if red_on else 0.0
                    self._green_brightness = 1.0 if green_on else 0.0
                    self._blue_brightness = 1.0 if blue_on else 0.0
                    self._update_hardware()
                    
                    # Sleep for ON period
                    if self._pattern_stop_event.wait(timeout=on_period):
                        break
                    
                    # OFF period - all LEDs off
                    self._red_brightness = 0.0
                    self._green_brightness = 0.0
                    self._blue_brightness = 0.0
                    self._update_hardware()
                    
                    # Sleep for OFF period
                    if self._pattern_stop_event.wait(timeout=off_period):
                        break
                        
        except Exception as e:
            self.get_logger().error(f"Error in pattern recreation: {e}")
        finally:
            self._pattern_active = False

    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.get_logger().info('Shutting down Mast LED Controller node')

        # Stop pattern recreation thread
        if self._pattern_active and self._pattern_stop_event:
            self._pattern_stop_event.set()
            if self._pattern_thread and self._pattern_thread.is_alive():
                self._pattern_thread.join(timeout=1.0)

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
            'Mast LED Controller Node for ROS2 - PCA9632 RGBW LED Driver',
            epilog="""
This ROS2 node controls the PCA9632 4-bit I2C LED driver (NXP) on the wind sensor PCB.

Hardware Setup:
  - Uses I2C bus 0 (same as wind sensor)
  - PCA9632 individual address 0x62 (0x70 is LED All Call; i2cdetect shows 7-bit addresses)
  - Four LED channels: Red, Green, Blue, White (RGBW) via LED0-LED3
  - Run "i2cdetect -y 0" to verify: expect 0x62 and 0x70 for PCA9632

Configuration Constants (modify at top of file):
  - I2C_BUS: I2C bus number (default: 0)
  - PCA9632_I2C_ADDRESS: individual device address (default: 0x62)
  - REG_PWM0-REG_PWM3: PWM registers for R, G, B, W

Debug: --debug for I2C and LED value logging.
Diagnostic: --diagnostic for MODE1, LEDOUT, PWM and recommendations on startup.
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
