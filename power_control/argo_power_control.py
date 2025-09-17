#!/usr/bin/env python3
#
# argo_power_control.py
#
# Argo Power Control System
# =====================================
#
# DESCRIPTION:
#   Intelligent power control system for Orange Pi Zero 2W using external relays
#   and GPIO pins. Implements safe shutdown procedures, power button monitoring,
#   and LED status indicators. Relay de-energization is handled automatically
#   by GPIO pin state reversion on system halt.
#
# HARDWARE CONFIGURATION:
#   - PI3 (Pin 40): !POW - Open drain output to control power relay
#   - PI9 (Pin 28): !POW_BUT - Input from power button (external pullup required)
#   - PH4 (Pin 18): Green LED in power button (system running indicator)
#   - PI1 (Pin 12): Blue LED in power button (status/warning indicator)
#   - Red LED: Directly connected to power button (not GPIO controlled)
#
# POWER CIRCUIT DESIGN:
#   The system uses two PFETs to control the power relay:
#   1. Direct PFET: Connected to power button for immediate power-on
#   2. GPIO PFET: Connected to !POW pin for software-controlled power management
#   
#   When !POW is pulled low (open drain), the relay is energized, providing power.
#   When !POW goes high, the relay is de-energized, cutting power.
#
# POWER BUTTON BEHAVIOR:
#   - Short press (< threshold): No action
#   - Long press (>= threshold): Initiate shutdown sequence
#   - Only new button presses after service startup are detected (prevents shutdown during boot)
#
# LED INDICATORS:
#   - Green LED: Heartbeat when system is running
#   - Both LEDs: Countdown pattern during button press (5,4,3,2,1 flashes at 3Hz, then 10 rapid flashes at 10Hz)
#   - Both LEDs: Alternating short-long pattern during shutdown sequence
#   - Red LED: Directly connected to power button (not controlled by GPIO)
#
# SHUTDOWN SEQUENCE:
#   1. Broadcast wall message to all users
#   2. Wait 5 seconds for users to see notification
#   3. Start alternating short-long LED pattern (shutdown indicator)
#   4. Execute 'sudo shutdown -h now' for graceful shutdown
#   5. GPIO pins will automatically revert to input state on halt, de-energizing relay
#
# USAGE:
#   ./argo_power_control.py [--help] [--test-mode] [--threshold SECONDS] [--keyboard-test]
#
# OPTIONS:
#   --help              Show this help message and exit
#   --test-mode         Run in test mode (disable actual shutdown and power control)
#   --threshold SECONDS Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
#   --keyboard-test     Enable keyboard test mode (s/e keys simulate button press/release)
#
# EXAMPLES:
#   ./argo_power_control.py                         # Normal operation
#   ./argo_power_control.py --test-mode             # Test mode (safe)
#   ./argo_power_control.py --keyboard-test         # Keyboard test mode with colored LED output
#   ./argo_power_control.py --threshold 2.0         # 2-second threshold
#
# REQUIREMENTS:
#   - User must be member of 'gpio' group (for GPIO access)
#   - python3-gpiod library installed
#   - External pullup resistor on power button input
#   - Proper hardware connections as described above
#
# SAFETY FEATURES:
#   - Open drain configuration prevents damage from multiple control sources
#   - Graceful shutdown ensures proper system shutdown before cutting power
#   - External pullup resistor requirement prevents floating inputs
#   - GPIO group membership requirement for controlled GPIO access
#
# AUTHOR: Generated for Orange Pi Zero 2W Power Control System
# VERSION: 1.0
# DATE: September 2024

import gpiod
import time
import signal
import sys
import subprocess
import threading
import logging
import os
import argparse
from pathlib import Path
from datetime import datetime
import select
import tty
import termios

# =============================================================================
# CONSTANTS
# =============================================================================

# ANSI Color codes for LED visualization
class Colors:
    GREEN = '\033[92m'      # Green LED
    BLUE = '\033[94m'       # Blue LED
    RED = '\033[91m'        # Red LED (not GPIO controlled)
    YELLOW = '\033[93m'     # Warning/Info
    CYAN = '\033[96m'       # Info
    MAGENTA = '\033[95m'    # Debug
    WHITE = '\033[97m'      # Normal text
    BLACK = '\033[30m'      # Black text
    BOLD = '\033[1m'        # Bold text
    RESET = '\033[0m'       # Reset all formatting
    BG_GREEN = '\033[42m'   # Green background
    BG_BLUE = '\033[44m'    # Blue background
    BG_RED = '\033[41m'     # Red background

# Button Press Configuration
DEFAULT_SHUTDOWN_THRESHOLD_S = 5.0      # Default button hold time for shutdown (seconds)
BUTTON_POLLING_HZ = 1.0                 # Button state polling frequency (1 Hz - reduced for lower CPU usage)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)

# Button Detection Configuration
# No boot delay - only detect new button presses after service starts

# LED Heartbeat Configuration
LED_HEARTBEAT_HZ = 1.0                  # Green LED heartbeat frequency (1 Hz)

# Button Press Pattern Configuration
LED_PRESS_START_FREQUENCY_HZ = 2.0      # Starting flash frequency (2 Hz)
LED_PRESS_END_FREQUENCY_HZ = 20.0       # Ending flash frequency (20 Hz)
LED_PRESS_DUTY_CYCLE = 0.5              # 50% duty cycle during button press

# Shutdown Pattern Configuration
LED_SHUTDOWN_FREQUENCY_HZ = 1.0         # Shutdown flash frequency (1 Hz)
LED_SHUTDOWN_DUTY_CYCLE = 0.05           # 5% duty cycle during shutdown

# Shutdown Sequence Timing
# No delay - shutdown immediately after wall message
TEST_MODE_SHUTDOWN_DELAY_S = 5          # Simulated shutdown delay in test mode (seconds)

# Notification Timeouts
DESKTOP_NOTIFICATION_TIMEOUT_S = 5      # Desktop notification timeout (seconds)
WALL_MESSAGE_TIMEOUT_S = 5              # Wall message timeout (seconds)
NOTIFICATION_EXPIRE_TIME_MS = 5000      # Standard notification expire time (5 seconds)
FINAL_SHUTDOWN_NOTIFICATION_MS = 0      # Final shutdown notification (no timeout - stays until dismissed)

# Main Loop Timing
MAIN_LOOP_SLEEP_S = 1                   # Main control loop sleep interval (seconds)

# Configure logging
def setup_logging():
    """Setup logging configuration"""
    handlers = [logging.StreamHandler()]
    
    # Only add file handler if we have write permission to /var/log/
    try:
        # Test if we can write to /var/log/
        test_file = '/var/log/argo_power_control.log'
        with open(test_file, 'a') as f:
            pass
        handlers.append(logging.FileHandler(test_file))
    except (PermissionError, OSError):
        # Fall back to local log file or no file logging
        try:
            handlers.append(logging.FileHandler('argo_power_control.log'))
        except (PermissionError, OSError):
            pass  # No file logging if we can't write anywhere
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )

setup_logging()
logger = logging.getLogger('argo_power_control')

class PowerController:
    def __init__(self, test_mode=False, threshold=1.0, keyboard_test=False):
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        self.keyboard_test = keyboard_test
        
        # Button state tracking
        self.initial_button_state = None
        self.button_detection_active = False  # Flag to track when button detection should be active
        
        # LED state tracking for colored output
        self.green_led_state = False
        self.blue_led_state = False
        self.red_led_state = False  # Not GPIO controlled, but tracked for display
        
        # GPIO Configuration
        self.GPIO_CHIP = '/dev/gpiochip0'
        
        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT  
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED
        
        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold
        
        # Initialize GPIO
        self.init_gpio()
        
        # Setup signal handlers for graceful service shutdown
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGQUIT, self.signal_handler)
        
        logger.info("Power controller initialized")
        
        # Record initial button state during boot
        self.initial_button_state = self.read_power_button()
        logger.info(f"Initial button state recorded: {self.initial_button_state}")
        
        # Set button detection flag based on initial state
        if self.initial_button_state == 1:  # Button not pressed at startup
            self.button_detection_active = True  # Can detect presses immediately
            logger.info("Button detection active - ready to detect presses")
        else:  # Button pressed at startup
            self.button_detection_active = False  # Wait for first release
            logger.info("Button pressed at startup - waiting for release before detection starts")


    def init_gpio(self):
        """Initialize GPIO pins"""
        try:
            logger.info(f"Attempting to open GPIO chip: {self.GPIO_CHIP}")
            self.chip = gpiod.Chip(self.GPIO_CHIP)
            logger.info(f"Successfully opened GPIO chip: {self.GPIO_CHIP}")

            # Get line objects
            self.power_relay_line = self.chip.get_line(self.POWER_RELAY_LINE)
            self.power_button_line = self.chip.get_line(self.POWER_BUTTON_LINE)
            self.green_led_line = self.chip.get_line(self.GREEN_LED_LINE)
            self.blue_led_line = self.chip.get_line(self.BLUE_LED_LINE)

            # Request power relay line
            self.power_relay_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_OPEN_DRAIN,
                default_vals=[0]  # Start with relay energized (low = on)
            )

            # Request power button line
            self.power_button_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE
            )

            # Request LED lines
            self.green_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )
            self.blue_led_line.request(
                consumer="argo_power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )
            
            logger.info("GPIO pins configured successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            logger.error(f"GPIO chip path attempted: {self.GPIO_CHIP}")
            logger.error(f"Full error details: {type(e).__name__}: {e}")
            raise

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, stopping power controller...")
        self.running = False


    def read_power_button(self):
        """Read power button state"""
        try:
            return self.power_button_line.get_value()
        except Exception as e:
            logger.error(f"Error reading power button: {e}")
            return 1  # Assume not pressed on error

    def set_power_relay(self, state):
        """Control power relay (True = energized/on, False = de-energized/off)"""
        try:
            # For open drain: 0 = energized (relay on), 1 = de-energized (relay off)
            value = 0 if state else 1
            self.power_relay_line.set_value(value)
            logger.info(f"Power relay set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling power relay: {e}")

    def set_green_led(self, state):
        """Control green LED"""
        try:
            value = 1 if state else 0
            if not self.keyboard_test:
                self.green_led_line.set_value(value)
            self.green_led_state = state
            if self.keyboard_test:
                self.print_led_status()
            logger.debug(f"Green LED set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED"""
        try:
            value = 1 if state else 0
            if not self.keyboard_test:
                self.blue_led_line.set_value(value)
            self.blue_led_state = state
            if self.keyboard_test:
                self.print_led_status()
        except Exception as e:
            logger.error(f"Error controlling blue LED: {e}")

    def print_led_status(self):
        """Print LED status with colored output for keyboard test mode"""
        if not self.keyboard_test:
            return
            
        # Clear line and print LED status
        print(f"\r{Colors.RESET}", end="", flush=True)
        
        # Green LED status
        green_status = f"{Colors.BG_GREEN}{Colors.BLACK}GREEN{Colors.RESET}" if self.green_led_state else f"{Colors.GREEN}GREEN{Colors.RESET}"
        
        # Blue LED status  
        blue_status = f"{Colors.BG_BLUE}{Colors.WHITE}BLUE{Colors.RESET}" if self.blue_led_state else f"{Colors.BLUE}BLUE{Colors.RESET}"
        
        # Red LED status (not GPIO controlled, but shown for completeness)
        red_status = f"{Colors.BG_RED}{Colors.WHITE}RED{Colors.RESET}" if self.red_led_state else f"{Colors.RED}RED{Colors.RESET}"
        
        print(f"LEDs: {green_status} {blue_status} {red_status}", end="", flush=True)

    def print_test_instructions(self):
        """Print keyboard test mode instructions"""
        if not self.keyboard_test:
            return
            
        print(f"\n{Colors.CYAN}{Colors.BOLD}=== KEYBOARD TEST MODE ==={Colors.RESET}")
        print(f"{Colors.YELLOW}Instructions:{Colors.RESET}")
        print(f"  • Press {Colors.BOLD}s{Colors.RESET} to start button press simulation")
        print(f"  • Press {Colors.BOLD}e{Colors.RESET} to end button press simulation")
        print(f"  • Hold for {self.SHUTDOWN_THRESHOLD}s to trigger shutdown sequence")
        print(f"  • Press {Colors.BOLD}q{Colors.RESET} to quit")
        print(f"  • Press {Colors.BOLD}r{Colors.RESET} to test recording function")
        print(f"  • LED status shown in colored text below")
        print(f"{Colors.CYAN}=============================={Colors.RESET}\n")

    def get_keyboard_input(self):
        """Get keyboard input for test mode"""
        if not self.keyboard_test:
            return None
            
        try:
            # Check if input is available
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1)
                return key
        except:
            pass
        return None

    def simulate_button_press(self):
        """Simulate button press for keyboard test mode"""
        if not self.keyboard_test:
            return
            
        self.button_press_start_time = time.time()
        self.power_button_pressed = True
        print(f"\n{Colors.YELLOW}🔘 Button PRESSED (simulated){Colors.RESET}")
        
        # Send desktop notification for button press
        self.send_desktop_notification(
            "Power Button Pressed (Test)", 
            "Power button pressed - hold for shutdown",
            "normal"
        )
        
        # Start gradual frequency LED pattern
        threading.Thread(
            target=self.gradual_frequency_pattern,
            daemon=True
        ).start()

    def simulate_button_release(self):
        """Simulate button release for keyboard test mode"""
        if not self.keyboard_test or not self.power_button_pressed:
            return
            
        press_duration = time.time() - self.button_press_start_time
        print(f"\n{Colors.YELLOW}🔘 Button RELEASED after {press_duration:.2f}s{Colors.RESET}")
        
        if press_duration >= self.SHUTDOWN_THRESHOLD:
            print(f"{Colors.RED}🔴 Long press detected - initiating shutdown...{Colors.RESET}")
            self.initiate_shutdown()
        else:
            print(f"{Colors.CYAN}🔵 Short press detected - no action{Colors.RESET}")
            # Send notification for short press
            self.send_desktop_notification(
                "Power Button Released (Test)", 
                f"Short press detected ({press_duration:.1f}s) - no action",
                "low"
            )
        
        self.power_button_pressed = False
        self.button_press_start_time = None

    def test_recording_function(self):
        """Test the recording function with visual feedback"""
        if not self.keyboard_test:
            return
            
        print(f"\n{Colors.MAGENTA}🎬 Testing Recording Function{Colors.RESET}")
        print(f"{Colors.CYAN}Starting recording simulation...{Colors.RESET}")
        
        # Simulate recording start with LED patterns
        for i in range(3):
            self.set_green_led(True)
            self.set_blue_led(True)
            time.sleep(0.2)
            self.set_green_led(False)
            self.set_blue_led(False)
            time.sleep(0.2)
        
        print(f"{Colors.GREEN}✅ Recording started (simulated){Colors.RESET}")
        print(f"{Colors.YELLOW}Recording indicators: Both LEDs flashing{Colors.RESET}")
        
        # Simulate recording in progress
        for i in range(10):
            self.set_green_led(True)
            time.sleep(0.5)
            self.set_green_led(False)
            time.sleep(0.5)
            print(f"{Colors.CYAN}Recording... {i+1}/10{Colors.RESET}", end="\r", flush=True)
        
        print(f"\n{Colors.RED}⏹️  Recording stopped (simulated){Colors.RESET}")
        print(f"{Colors.CYAN}Recording test completed{Colors.RESET}\n")

    def led_blink_pattern(self, led_func, duration, blink_frequency_hz):
        """Blink LED with specified pattern
        
        Args:
            led_func: Function to call to control LED state
            duration: How long to blink (seconds)
            blink_frequency_hz: Blink frequency in Hz
        """
        start_time = time.time()
        led_state = False
        blink_interval = 1.0 / blink_frequency_hz
        
        while time.time() - start_time < duration and self.running:
            led_func(led_state)
            led_state = not led_state
            time.sleep(blink_interval)

    def green_led_heartbeat(self):
        """Green LED heartbeat during normal operation"""
        led_state = True  # Start with LED ON for immediate visual feedback
        heartbeat_interval = 1.0 / LED_HEARTBEAT_HZ  # 1 second interval
        while self.running:
            self.set_green_led(led_state)
            led_state = not led_state
            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < heartbeat_interval and self.running:
                time.sleep(0.1)  # Sleep in 100ms increments
                sleep_time += 0.1


    def shutdown_led_pattern(self):
        """1Hz LED pattern with configurable duty cycle during shutdown sequence"""
        logger.info(f"Starting shutdown LED pattern (1Hz, {LED_SHUTDOWN_DUTY_CYCLE*100:.0f}% duty cycle)")
        
        # Calculate timing for 1Hz with configurable duty cycle
        period = 1.0 / LED_SHUTDOWN_FREQUENCY_HZ  # 1 second
        on_time = period * LED_SHUTDOWN_DUTY_CYCLE  # Configurable on time
        off_time = period * (1.0 - LED_SHUTDOWN_DUTY_CYCLE)  # Remaining off time
        
        while self.running and self.shutdown_initiated:
            try:
                # Turn on both LEDs
                logger.debug("Shutdown LED: ON")
                self.set_green_led(True)
                self.set_blue_led(True)
                
                # Sleep in small increments to allow for responsive shutdown
                sleep_time = 0
                while sleep_time < on_time and self.running and self.shutdown_initiated:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01
                
                if not self.running or not self.shutdown_initiated:
                    break
                    
                # Turn off both LEDs
                logger.debug("Shutdown LED: OFF")
                self.set_green_led(False)
                self.set_blue_led(False)
                
                # Sleep in small increments to allow for responsive shutdown
                sleep_time = 0
                while sleep_time < off_time and self.running and self.shutdown_initiated:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01
                
            except Exception as e:
                logger.error(f"Error in shutdown LED pattern: {e}")
                break
        
        # Turn off LEDs when done
        self.set_green_led(False)
        self.set_blue_led(False)
        logger.info("Shutdown LED pattern completed")

    def gradual_frequency_pattern(self):
        """Gradual frequency increase LED pattern during button press - 2Hz to 20Hz over threshold time"""
        start_time = time.time()
        logger.debug(f"Starting gradual frequency pattern for {self.SHUTDOWN_THRESHOLD}s threshold")
        
        while self.running and self.power_button_pressed and not self.shutdown_initiated:
            # Calculate elapsed time and progress (0.0 to 1.0)
            elapsed_time = time.time() - start_time
            progress = min(elapsed_time / self.SHUTDOWN_THRESHOLD, 1.0)
            
            # Calculate current frequency (2Hz to 20Hz)
            current_frequency = LED_PRESS_START_FREQUENCY_HZ + (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress
            
            # Calculate timing for 50% duty cycle
            period = 1.0 / current_frequency
            on_time = period * LED_PRESS_DUTY_CYCLE
            off_time = period * (1.0 - LED_PRESS_DUTY_CYCLE)
            
            # Turn on both LEDs
            self.set_green_led(True)
            self.set_blue_led(True)
            
            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < on_time and self.running and self.power_button_pressed and not self.shutdown_initiated:
                time.sleep(0.01)  # Sleep in 10ms increments
                sleep_time += 0.01
            
            if not self.running or not self.power_button_pressed or self.shutdown_initiated:
                break
                
            # Turn off both LEDs
            self.set_green_led(False)
            self.set_blue_led(False)
            
            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < off_time and self.running and self.power_button_pressed and not self.shutdown_initiated:
                time.sleep(0.01)  # Sleep in 10ms increments
                sleep_time += 0.01
        
        # Turn off LEDs when pattern stops
        self.set_green_led(False)
        self.set_blue_led(False)

    def monitor_power_button_polling(self):
        """Polling-based button monitoring - only detects new button presses after service starts"""
        logger.info("Using polling-based button monitoring")
        last_button_state = self.initial_button_state  # Start with initial state
        poll_interval = 1.0 / BUTTON_POLLING_HZ  # Calculate once
        
        while self.running:
            try:
                current_button_state = self.read_power_button()
                
                # Detect button press (falling edge) - only if detection is active
                if last_button_state == 1 and current_button_state == 0:
                    if self.button_detection_active:
                        self.button_press_start_time = time.time()
                        self.power_button_pressed = True
                        logger.info("Power button pressed (polling)")
                        
                        # Send desktop notification for button press
                        self.send_desktop_notification(
                            "Power Button Pressed", 
                            "Power button pressed - hold for shutdown",
                            "normal"
                        )
                        
                        # Start gradual frequency LED pattern
                        threading.Thread(
                            target=self.gradual_frequency_pattern,
                            daemon=True
                        ).start()
                    else:
                        logger.debug("Button press detected but detection not yet active - ignoring")
                
                # Detect button release (rising edge)
                elif last_button_state == 0 and current_button_state == 1:
                    # If button was pressed at startup and this is the first release, activate detection
                    if not self.button_detection_active and self.initial_button_state == 0:
                        self.button_detection_active = True
                        logger.info("Button released after startup - button detection now active")
                    
                    if self.power_button_pressed:
                        press_duration = time.time() - self.button_press_start_time
                        logger.info(f"Power button released after {press_duration:.2f} seconds (polling)")
                        
                        if press_duration >= self.SHUTDOWN_THRESHOLD:
                            logger.info("Long press detected, initiating shutdown...")
                            self.initiate_shutdown()
                        else:
                            logger.info("Short press detected, no action taken")
                            # Send notification for short press
                            self.send_desktop_notification(
                                "Power Button Released", 
                                f"Short press detected ({press_duration:.1f}s) - no action",
                                "low"
                            )
                    
                    self.power_button_pressed = False
                    self.button_press_start_time = None
                
                # Check for long press while button is held
                elif self.power_button_pressed and current_button_state == 0:
                    if time.time() - self.button_press_start_time >= self.SHUTDOWN_THRESHOLD:
                        if not self.shutdown_initiated:
                            logger.info("Long press threshold reached, initiating shutdown...")
                            self.initiate_shutdown()
                
                last_button_state = current_button_state
                
                # Sleep in small increments to allow for responsive shutdown
                sleep_time = 0
                while sleep_time < poll_interval and self.running:
                    time.sleep(0.01)  # Sleep in 10ms increments
                    sleep_time += 0.01
                
            except Exception as e:
                logger.error(f"Error in polling-based button monitoring: {e}")
                time.sleep(BUTTON_ERROR_RECOVERY_DELAY_S)

    def send_desktop_notification(self, title, message, urgency="normal", expire_time_ms=None):
        """Send desktop notification using notify-send"""
        try:
            # Set up environment for desktop notifications
            env = os.environ.copy()
            
            # Try to find the display for logged-in users
            try:
                # Get list of logged-in users and their displays
                result = subprocess.run(['who'], capture_output=True, text=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
                if result.returncode == 0:
                    for line in result.stdout.strip().split('\n'):
                        if line and ':' in line:
                            parts = line.split()
                            for part in parts:
                                if part.startswith(':'):
                                    display = part[1:]  # Remove the leading ':'
                                    if display.isdigit() or '.' in display:
                                        env['DISPLAY'] = f':{display}'
                                        break
                            if 'DISPLAY' in env:
                                break
            except Exception as e:
                logger.debug(f"Could not determine display from 'who' command: {e}")
            
            # If no display found, try common defaults
            if 'DISPLAY' not in env:
                env['DISPLAY'] = ':0'
            
            # Modify title and message for test mode
            if self.test_mode:
                test_title = f"[TEST] {title}"
                test_message = f"{message}\n\n(This is a test notification - no actual action taken)"
                logger.info(f"TEST MODE: Sending desktop notification - {title}: {message}")
            else:
                test_title = title
                test_message = message
            
            # Use custom expire time or default
            expire_time = expire_time_ms if expire_time_ms is not None else NOTIFICATION_EXPIRE_TIME_MS
            
            # Try to send desktop notification
            subprocess.run([
                'notify-send', 
                '--urgency', urgency,
                '--expire-time', str(expire_time),
                test_title, 
                test_message
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S, env=env)
            
            if self.test_mode:
                logger.info(f"TEST MODE: Desktop notification sent: {title}")
            else:
                logger.info(f"Desktop notification sent: {title}")
                
        except subprocess.TimeoutExpired:
            logger.warning("Desktop notification timed out")
        except subprocess.CalledProcessError as e:
            logger.warning(f"Failed to send desktop notification: {e}")
        except Exception as e:
            logger.warning(f"Unexpected error sending desktop notification: {e}")

    def broadcast_shutdown_message(self):
        """Broadcast shutdown message to all logged-in users"""
        try:
            # Create shutdown message with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nShutting down now..."
            
            # Use wall command to broadcast message to all users
            try:
                if self.test_mode:
                    # Add test prefix to wall message
                    test_message = f"TEST: {message}"
                    subprocess.run(['wall', test_message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info(f"TEST MODE: Wall message sent: {message}")
                else:
                    # Use wall command to send message to all logged-in users
                    subprocess.run(['wall', message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
                    logger.info("Shutdown message broadcasted to all users")
            except subprocess.TimeoutExpired:
                logger.warning("Wall command timed out - message may not have been sent")
            except subprocess.CalledProcessError as e:
                logger.warning(f"Failed to broadcast wall message: {e}")
            except Exception as e:
                logger.warning(f"Unexpected error broadcasting wall message: {e}")
                    
        except Exception as e:
            logger.error(f"Error preparing shutdown message: {e}")

    def initiate_shutdown(self):
        """Initiate system shutdown sequence"""
        if self.shutdown_initiated:
            return
            
        self.shutdown_initiated = True
        
        logger.info("Initiating shutdown sequence...")
        
        # Start 1Hz LED pattern immediately to show shutdown initiated
        threading.Thread(
            target=self.shutdown_led_pattern,
            daemon=True
        ).start()
        
        # Broadcast wall message to all users
        self.broadcast_shutdown_message()
        
        # Send final desktop notification
        self.send_desktop_notification(
            "System Shutdown", 
            "Shutdown initiated by power button - system shutting down now",
            "critical"
        )
        
        # Execute shutdown command
        if self.test_mode:
            logger.info("TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            # In test mode, let the shutdown LED pattern run for demonstration
            logger.info(f"TEST MODE: Running shutdown LED pattern for {TEST_MODE_SHUTDOWN_DELAY_S} seconds...")
            time.sleep(TEST_MODE_SHUTDOWN_DELAY_S)  # Let the pattern run for demonstration
            self.running = False  # Stop running after LED pattern demonstration
        else:
            logger.info("Executing shutdown command: shutdown -h now")
            subprocess.run(['shutdown', '-h', 'now'], check=True)
            logger.info("Shutdown command executed successfully")
            # Don't set self.running = False here - let the system shutdown
            # GPIO pins will automatically revert to input state on halt, de-energizing relay


    def run(self):
        """Main control loop"""
        logger.info("Power controller starting...")
        logger.info("Button monitoring active - only new button presses after startup will trigger shutdown")
        
        # Ensure relay is energized on startup
        self.set_power_relay(True)
        
        # Print test instructions if in keyboard test mode
        if self.keyboard_test:
            self.print_test_instructions()
            # Set up terminal for raw input
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        
        # Start power button monitoring in separate thread (skip if keyboard test mode)
        if not self.keyboard_test:
            logger.info("Starting polling-based button detection")
            button_thread = threading.Thread(target=self.monitor_power_button_polling, daemon=True)
            button_thread.start()
        
        # Start green LED heartbeat in separate thread
        logger.info("Starting green LED heartbeat thread")
        heartbeat_thread = threading.Thread(target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()
        
        try:
            # Main loop
            while self.running:
                if self.keyboard_test:
                    # Handle keyboard input in test mode
                    key = self.get_keyboard_input()
                    if key:
                        if key == 's' and not self.power_button_pressed:
                            # 's' key pressed - start button press simulation
                            self.simulate_button_press()
                        elif key == 'e' and self.power_button_pressed:
                            # 'e' key pressed - end button press simulation
                            self.simulate_button_release()
                        elif key == 'q':
                            # Quit
                            print(f"\n{Colors.YELLOW}Quitting keyboard test mode...{Colors.RESET}")
                            self.running = False
                        elif key == 'r':
                            # Test recording function
                            self.test_recording_function()
                        elif key == '\x03':  # Ctrl+C
                            print(f"\n{Colors.YELLOW}Received Ctrl+C, quitting...{Colors.RESET}")
                            self.running = False
                    
                    time.sleep(0.1)  # Small delay for responsive input
                else:
                    # Normal mode - just keep alive
                    time.sleep(MAIN_LOOP_SLEEP_S)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            self.running = False
        
        finally:
            # Restore terminal settings if in keyboard test mode
            if self.keyboard_test:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up power controller...")
        
        try:
            # Turn off LEDs
            self.set_green_led(False)
            self.set_blue_led(False)
            
            # Close GPIO chip
            if hasattr(self, 'chip'):
                self.chip.close()
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
        
        logger.info("Power controller cleanup complete")

def test_gradual_frequency_pattern(threshold):
    """Test the gradual frequency pattern"""
    print(f"Testing gradual frequency pattern for {threshold}s threshold...")
    print(f"Frequency will increase from {LED_PRESS_START_FREQUENCY_HZ}Hz to {LED_PRESS_END_FREQUENCY_HZ}Hz")
    print(f"Duty cycle: {LED_PRESS_DUTY_CYCLE * 100}%")
    
    start_time = time.time()
    
    while time.time() - start_time < threshold:
        # Calculate elapsed time and progress (0.0 to 1.0)
        elapsed_time = time.time() - start_time
        progress = min(elapsed_time / threshold, 1.0)
        
        # Calculate current frequency (2Hz to 20Hz)
        current_frequency = LED_PRESS_START_FREQUENCY_HZ + (LED_PRESS_END_FREQUENCY_HZ - LED_PRESS_START_FREQUENCY_HZ) * progress
        
        # Calculate timing for 50% duty cycle
        period = 1.0 / current_frequency
        on_time = period * LED_PRESS_DUTY_CYCLE
        off_time = period * (1.0 - LED_PRESS_DUTY_CYCLE)
        
        # Show current frequency every 0.5 seconds
        if int(elapsed_time * 2) != int((elapsed_time - 0.01) * 2):
            print(f"Time: {elapsed_time:.1f}s, Frequency: {current_frequency:.1f}Hz", flush=True)
        
        # Turn on both LEDs
        print("LED ON", flush=True)
        time.sleep(on_time)
        
        # Turn off both LEDs
        print("LED OFF", flush=True)
        time.sleep(off_time)
    
    print("LED OFF - Pattern complete")

def test_shutdown_pattern():
    """Test the shutdown LED pattern"""
    print("Testing shutdown LED pattern (1Hz, 20% duty cycle)...")
    
    # Calculate timing for 1Hz with 20% duty cycle
    period = 1.0 / LED_SHUTDOWN_FREQUENCY_HZ  # 1 second
    on_time = period * LED_SHUTDOWN_DUTY_CYCLE  # 0.2 seconds
    off_time = period * (1.0 - LED_SHUTDOWN_DUTY_CYCLE)  # 0.8 seconds
    
    print(f"Period: {period}s, On time: {on_time}s, Off time: {off_time}s")
    
    for cycle in range(5):  # Show 5 cycles
        print(f"Cycle {cycle + 1}: LED ON", flush=True)
        time.sleep(on_time)
        print(f"Cycle {cycle + 1}: LED OFF", flush=True)
        time.sleep(off_time)
    
    print("LED OFF - Shutdown pattern complete")

def print_help():
    """Print help message"""
    help_text = """
Orange Pi Zero 2W Power Control System
=====================================

DESCRIPTION:
  Intelligent power control system for Orange Pi Zero 2W using external relays
  and GPIO pins. Implements safe shutdown procedures, power button monitoring,
  and LED status indicators. Relay de-energization is handled automatically
  by GPIO pin state reversion on system halt.

HARDWARE CONFIGURATION:
  - PI3 (Pin 40): !POW - Open drain output to control power relay
  - PI9 (Pin 28): !POW_BUT - Input from power button (external pullup required)
  - PH4 (Pin 18): Green LED in power button (system running indicator)
  - PI1 (Pin 12): Blue LED in power button (status/warning indicator)
  - Red LED: Directly connected to power button (not GPIO controlled)

POWER BUTTON BEHAVIOR:
  - Short press (< threshold): No action
  - Long press (>= threshold): Initiate shutdown sequence
  - Only new button presses after service startup are detected (prevents shutdown during boot)

LED INDICATORS:
  - Green LED: Heartbeat when system is running
  - Both LEDs: Countdown pattern during button press (5,4,3,2,1 flashes at 3Hz, then 10 rapid flashes at 10Hz)
  - Both LEDs: Alternating short-long pattern during shutdown sequence
  - Red LED: Directly connected to power button (not controlled by GPIO)

USAGE:
  ./argo_power_control.py [OPTIONS]

OPTIONS:
  --help, -h          Show this help message and exit
  --test-mode, -t     Run in test mode (disable actual shutdown and power control)
  --keyboard-test, -k Enable keyboard test mode (s/e keys simulate button press/release)
  --threshold, -T     Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)
  --debug-shutdown-flashing, -d  Test shutdown countdown LED pattern and exit (safe for testing)
EXAMPLES:
  ./argo_power_control.py                         # Normal operation
  ./argo_power_control.py --test-mode             # Test mode (safe)
  ./argo_power_control.py -t                      # Test mode (short form)
  ./argo_power_control.py --keyboard-test         # Keyboard test mode with colored LED output
  ./argo_power_control.py -k                      # Keyboard test mode (short form)
  ./argo_power_control.py --threshold 2.0         # 2-second threshold
  ./argo_power_control.py -T 2.0                  # 2-second threshold (short form)
  ./argo_power_control.py --test-wall-message     # Test wall message (safe)
  ./argo_power_control.py -w                      # Test wall message (short form)
  ./argo_power_control.py --test-notification     # Test desktop notification (safe)
  ./argo_power_control.py -n                      # Test desktop notification (short form)
  ./argo_power_control.py --debug-shutdown-flashing # Test countdown LED pattern (safe)
  ./argo_power_control.py -d                      # Test countdown LED pattern (short form)

REQUIREMENTS:
  - User must be member of 'gpio' group (for GPIO access)
  - python3-gpiod library installed
  - External pullup resistor on power button input
  - Proper hardware connections as described above

SAFETY FEATURES:
  - Open drain configuration prevents damage from multiple control sources
  - Graceful shutdown ensures proper system shutdown before cutting power
  - GPIO pins automatically revert to input state on halt, de-energizing relay
  - External pullup resistor requirement prevents floating inputs
  - GPIO group membership requirement for controlled GPIO access
"""
    print(help_text)

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Orange Pi Zero 2W Power Control System',
        add_help=False
    )
    parser.add_argument('--help', '-h', action='store_true', 
                       help='Show help message and exit')
    parser.add_argument('--test-mode', '-t', action='store_true',
                       help='Run in test mode (disable actual shutdown and power control)')
    parser.add_argument('--keyboard-test', '-k', action='store_true',
                       help='Enable keyboard test mode (s/e keys simulate button press/release)')
    parser.add_argument('--threshold', '-T', type=float, default=DEFAULT_SHUTDOWN_THRESHOLD_S,
                       help=f'Set button press threshold for shutdown in seconds (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})')
    parser.add_argument('--test-wall-message', '-w', action='store_true',
                       help='Test wall message functionality and exit (safe for testing)')
    parser.add_argument('--test-notification', '-n', action='store_true',
                       help='Test desktop notification functionality and exit (safe for testing)')
    parser.add_argument('--debug-shutdown-flashing', '-d', action='store_true',
                       help='Test shutdown countdown LED pattern and exit (safe for testing)')
    
    args = parser.parse_args()
    
    # Handle help
    if args.help:
        print_help()
        sys.exit(0)
    
    # Handle wall message test
    if args.test_wall_message:
        print("Testing wall message functionality...")
        try:
            # Test wall message without GPIO initialization
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"TEST: SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nThis is a test message - the system will NOT shutdown (would shutdown immediately)."
            
            print(f"Broadcasting test message: {message}")
            subprocess.run(['wall', message], check=True, timeout=WALL_MESSAGE_TIMEOUT_S)
            print("Wall message test completed successfully!")
            print("Check your terminal - you should have received the wall message.")
        except subprocess.TimeoutExpired:
            print("Wall command timed out - message may not have been sent")
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Failed to broadcast wall message: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error during wall message test: {e}")
            sys.exit(1)
        sys.exit(0)
    
    # Handle notification test
    if args.test_notification:
        print("Testing desktop notification functionality...")
        try:
            # Test different notification types
            print("Sending test notifications...")
            
            # Test normal notification
            subprocess.run([
                'notify-send', 
                '--urgency', 'normal',
                '--expire-time', '3000',
                'Power Control Test', 
                'Normal notification test - power button functionality'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Normal notification sent")
            
            # Test critical notification
            subprocess.run([
                'notify-send', 
                '--urgency', 'critical',
                '--expire-time', '5000',
                'System Shutdown', 
                'Critical notification test - system shutdown warning'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Critical notification sent")
            
            # Test low priority notification
            subprocess.run([
                'notify-send', 
                '--urgency', 'low',
                '--expire-time', '2000',
                'Power Button', 
                'Low priority notification test - short press detected'
            ], check=True, timeout=DESKTOP_NOTIFICATION_TIMEOUT_S)
            print("✅ Low priority notification sent")
            
            print("Desktop notification test completed successfully!")
            print("Check your desktop - you should have received 3 test notifications.")
        except subprocess.TimeoutExpired:
            print("Desktop notification timed out - check if notify-send is working")
            sys.exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Failed to send desktop notification: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error during desktop notification test: {e}")
            sys.exit(1)
        sys.exit(0)
    
    # Handle debug shutdown flashing test
    if args.debug_shutdown_flashing:
        print("Testing LED patterns...")
        try:
            # Test gradual frequency pattern with different thresholds
            test_thresholds = [2.0, 3.0, 5.0]
            
            for threshold in test_thresholds:
                print(f"\n=== Testing gradual frequency pattern with {threshold}s threshold ===")
                test_gradual_frequency_pattern(threshold)
                print(f"=== End {threshold}s threshold test ===\n")
                time.sleep(1)  # Brief pause between tests
            
            # Test shutdown pattern
            print("\n=== Testing shutdown LED pattern ===")
            test_shutdown_pattern()
            print("=== End shutdown pattern test ===\n")
            
            print("LED pattern tests completed successfully!")
            print("Check the output above to verify the patterns work correctly.")
        except Exception as e:
            print(f"Error during LED pattern test: {e}")
            sys.exit(1)
        sys.exit(0)
    
    # Check if user is in the gpio group (required for GPIO access)
    # Skip this check if running as root (systemd service)
    if os.geteuid() != 0:
        import grp
        try:
            gpio_group = grp.getgrnam('gpio')
            if gpio_group.gr_gid not in os.getgroups():
                print("Error: This script requires GPIO access")
                print("Your user must be a member of the 'gpio' group")
                print("Run: sudo usermod -a -G gpio $USER")
                print("Then log out and log back in, or run: newgrp gpio")
                sys.exit(1)
        except KeyError:
            print("Error: GPIO group not found")
            print("Please ensure the 'gpio' group exists and you are a member")
            sys.exit(1)
    else:
        print("Running as root - skipping GPIO group check")
    
    # Validate threshold
    if args.threshold <= 0:
        print("Error: Threshold must be greater than 0")
        sys.exit(1)
    
    # Show startup information
    if args.keyboard_test:
        print("Starting power control system in KEYBOARD TEST MODE")
        print("  - 's' key starts button press simulation")
        print("  - 'e' key ends button press simulation")
        print("  - Colored LED output displayed")
        print("  - Safe for testing")
    elif args.test_mode:
        print("Starting power control system in TEST MODE")
        print("  - Shutdown commands disabled")
        print("  - Power relay control disabled")
        print("  - Safe for testing")
    else:
        print("Starting power control system in NORMAL MODE")
        print("  - Full power control enabled")
        print("  - System will shutdown and cut power on long button press")
    
    print(f"  - Button press threshold: {args.threshold} seconds")
    if not args.keyboard_test:
        print(f"  - Button detection mode: Polling ({BUTTON_POLLING_HZ} Hz)")
    print("  - Press Ctrl+C to stop")
    print()
    
    controller = PowerController(test_mode=args.test_mode, threshold=args.threshold, keyboard_test=args.keyboard_test)
    controller.run()

if __name__ == "__main__":
    main()
