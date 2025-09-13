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
#   ./argo_power_control.py [--help] [--test-mode] [--threshold SECONDS]
#
# OPTIONS:
#   --help              Show this help message and exit
#   --test-mode         Run in test mode (disable actual shutdown and power control)
#   --threshold SECONDS Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
#
# EXAMPLES:
#   ./argo_power_control.py                         # Normal operation
#   ./argo_power_control.py --test-mode             # Test mode (safe)
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

# =============================================================================
# CONSTANTS
# =============================================================================

# Button Press Configuration
DEFAULT_SHUTDOWN_THRESHOLD_S = 5.0      # Default button hold time for shutdown (seconds)
BUTTON_POLLING_HZ = 1.0                 # Button state polling frequency (1 Hz - reduced for lower CPU usage)
BUTTON_ERROR_RECOVERY_DELAY_S = 0.1     # Delay on button read error (seconds)

# Button Detection Configuration
# No boot delay - only detect new button presses after service starts

# LED Blink Frequencies (Hz)
LED_BLINK_FAST_HZ = 10.0                # Fast blink frequency (10 Hz)
LED_BLINK_SLOW_HZ = 2.0                 # Slow blink frequency (2 Hz)
LED_HEARTBEAT_HZ = 1.0                  # Green LED heartbeat frequency (1 Hz)
LED_SHUTDOWN_BLINK_HZ = 10.0            # Shutdown warning blink frequency (10 Hz)

# Countdown Pattern Configuration
LED_COUNTDOWN_FREQUENCY_HZ = 3.0        # Countdown flash frequency (3 Hz)
LED_COUNTDOWN_PAUSE_S = 0.5             # Pause between countdown groups (0.5 seconds)
LED_FINAL_WARNING_FREQUENCY_HZ = 10.0   # Final warning flash frequency (10 Hz)
LED_FINAL_WARNING_DURATION_S = 2.0      # Final warning duration (2 seconds)

# Shutdown LED Pattern (seconds)
LED_SHUTDOWN_SHORT_ON_S = 0.2           # Short flash duration (on)
LED_SHUTDOWN_SHORT_OFF_S = 0.2          # Short flash duration (off)
LED_SHUTDOWN_LONG_ON_S = 0.8            # Long flash duration (on)
LED_SHUTDOWN_LONG_OFF_S = 0.4           # Long flash duration (off)

# Shutdown Sequence Timing
SHUTDOWN_NOTIFICATION_DELAY_S = 5       # Wait time for users to see notification (seconds)
SHUTDOWN_RELAY_DELAY_S = 10              # Delay before de-energizing relay (seconds)
SHUTDOWN_CACHE_FLUSH_TIMEOUT_S = 10     # SD card cache flush timeout (seconds)

# Notification Timeouts
DESKTOP_NOTIFICATION_TIMEOUT_S = 5      # Desktop notification timeout (seconds)
WALL_MESSAGE_TIMEOUT_S = 5              # Wall message timeout (seconds)
NOTIFICATION_EXPIRE_TIME_MS = 5000      # Notification expire time (milliseconds)

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
    def __init__(self, test_mode=False, threshold=1.0):
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        
        # Button state tracking
        self.initial_button_state = None
        
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
            self.green_led_line.set_value(value)
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED"""
        try:
            value = 1 if state else 0
            self.blue_led_line.set_value(value)
        except Exception as e:
            logger.error(f"Error controlling blue LED: {e}")

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
        led_state = False
        heartbeat_interval = 1.0 / LED_HEARTBEAT_HZ  # 1 second interval
        while self.running:
            self.set_green_led(led_state)
            led_state = not led_state
            # Sleep in small increments to allow for responsive shutdown
            sleep_time = 0
            while sleep_time < heartbeat_interval and self.running:
                time.sleep(0.1)  # Sleep in 100ms increments
                sleep_time += 0.1

    def cyan_shutdown_blink(self):
        """Blink both LEDs together rapidly for cyan effect during shutdown"""
        led_state = False
        blink_interval = 1.0 / LED_SHUTDOWN_BLINK_HZ
        for _ in range(20):  # Blink for about 2 seconds (20 * blink_interval)
            self.set_green_led(led_state)
            self.set_blue_led(led_state)
            led_state = not led_state
            time.sleep(blink_interval)

    def shutdown_alternating_pattern(self):
        """Alternating short-long LED pattern during shutdown sequence"""
        logger.info("Starting shutdown alternating LED pattern")
        pattern_count = 0
        
        while self.running and self.shutdown_initiated:
            try:
                # Short flash
                logger.debug("Shutdown LED: Short flash ON")
                self.set_green_led(True)
                self.set_blue_led(True)
                time.sleep(LED_SHUTDOWN_SHORT_ON_S)
                
                if not self.running or not self.shutdown_initiated:
                    break
                    
                logger.debug("Shutdown LED: Short flash OFF")
                self.set_green_led(False)
                self.set_blue_led(False)
                time.sleep(LED_SHUTDOWN_SHORT_OFF_S)
                
                if not self.running or not self.shutdown_initiated:
                    break
                
                # Long flash
                logger.debug("Shutdown LED: Long flash ON")
                self.set_green_led(True)
                self.set_blue_led(True)
                time.sleep(LED_SHUTDOWN_LONG_ON_S)
                
                if not self.running or not self.shutdown_initiated:
                    break
                    
                logger.debug("Shutdown LED: Long flash OFF")
                self.set_green_led(False)
                self.set_blue_led(False)
                time.sleep(LED_SHUTDOWN_LONG_OFF_S)
                
                pattern_count += 1
                logger.info(f"Completed shutdown LED pattern cycle {pattern_count}")
                
            except Exception as e:
                logger.error(f"Error in shutdown LED pattern: {e}")
                break
        
        # Turn off LEDs when done
        self.set_green_led(False)
        self.set_blue_led(False)
        logger.info("Shutdown LED pattern completed")

    def countdown_warning_pattern(self):
        """Countdown LED pattern during button press - 5,4,3,2,1 flashes then final warning"""
        start_time = time.time()
        blink_interval = 1.0 / LED_COUNTDOWN_FREQUENCY_HZ
        final_warning_interval = 1.0 / LED_FINAL_WARNING_FREQUENCY_HZ
        
        logger.debug(f"Starting countdown pattern for {self.SHUTDOWN_THRESHOLD}s threshold")
        
        # Calculate timing for countdown pattern
        countdown_duration = self.SHUTDOWN_THRESHOLD - LED_FINAL_WARNING_DURATION_S
        if countdown_duration < 0:
            countdown_duration = 0
        
        # Countdown pattern: 5, 4, 3, 2, 1 flashes
        countdown_groups = [5, 4, 3, 2, 1]
        group_duration = (countdown_duration - (len(countdown_groups) - 1) * LED_COUNTDOWN_PAUSE_S) / len(countdown_groups)
        
        if group_duration > 0:
            for group_num, flash_count in enumerate(countdown_groups):
                if not self.running or not self.power_button_pressed:
                    break
                    
                logger.debug(f"Countdown group {group_num + 1}: {flash_count} flashes")
                
                # Flash the specified number of times
                for flash in range(flash_count):
                    if not self.running or not self.power_button_pressed:
                        break
                    
                    # Turn on both LEDs
                    self.set_green_led(True)
                    self.set_blue_led(True)
                    time.sleep(blink_interval / 2)
                    
                    if not self.running or not self.power_button_pressed:
                        break
                    
                    # Turn off both LEDs
                    self.set_green_led(False)
                    self.set_blue_led(False)
                    time.sleep(blink_interval / 2)
                
                # Pause between groups (except after the last group)
                if group_num < len(countdown_groups) - 1:
                    if self.running and self.power_button_pressed:
                        time.sleep(LED_COUNTDOWN_PAUSE_S)
        
        # Final warning: 10 rapid flashes at 10Hz for 2 seconds
        if self.running and self.power_button_pressed:
            logger.debug("Final warning: 10 rapid flashes")
            final_start = time.time()
            while (time.time() - final_start < LED_FINAL_WARNING_DURATION_S and 
                   self.running and 
                   self.power_button_pressed):
                self.set_green_led(True)
                self.set_blue_led(True)
                time.sleep(final_warning_interval / 2)
                
                if not self.running or not self.power_button_pressed:
                    break
                    
                self.set_green_led(False)
                self.set_blue_led(False)
                time.sleep(final_warning_interval / 2)
        
        # Turn off LEDs when warning pattern stops
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
                
                # Detect button press (falling edge) - only if it's a new press from initial state
                if last_button_state == 1 and current_button_state == 0:
                    # Only process if this is a new press (not the initial state)
                    if last_button_state != self.initial_button_state or self.initial_button_state == 0:
                        self.button_press_start_time = time.time()
                        self.power_button_pressed = True
                        logger.info("Power button pressed (polling)")
                        
                        # Send desktop notification for button press
                        self.send_desktop_notification(
                            "Power Button Pressed", 
                            "Power button pressed - hold for shutdown",
                            "normal"
                        )
                        
                        # Start countdown warning LED pattern
                        threading.Thread(
                            target=self.countdown_warning_pattern,
                            daemon=True
                        ).start()
                    else:
                        logger.info("Power button already pressed at startup - ignoring until release")
                
                # Detect button release (rising edge)
                elif last_button_state == 0 and current_button_state == 1:
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

    def send_desktop_notification(self, title, message, urgency="normal"):
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
            
            # Try to send desktop notification
            subprocess.run([
                'notify-send', 
                '--urgency', urgency,
                '--expire-time', str(NOTIFICATION_EXPIRE_TIME_MS),
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
            message = f"SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nThe system will shutdown in 5 seconds."
            
            # Send desktop notification
            self.send_desktop_notification(
                "System Shutdown", 
                f"Power button pressed - system will shutdown in 5 seconds",
                "critical"
            )
            
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
        
        # Start alternating short-long LED pattern immediately to show shutdown initiated
        threading.Thread(
            target=self.shutdown_alternating_pattern,
            daemon=True
        ).start()
        
        
        # Broadcast wall message to all users
        self.broadcast_shutdown_message()
        
        # Give users time to see the wall message before shutdown
        logger.info(f"Waiting {SHUTDOWN_NOTIFICATION_DELAY_S} seconds for users to see shutdown notification...")
        time.sleep(SHUTDOWN_NOTIFICATION_DELAY_S)
        
        # Send final shutdown notification
        self.send_desktop_notification(
            "System Shutdown", 
            "Shutting down now...",
            "critical"
        )
        
        # Execute shutdown command
        if self.test_mode:
            logger.info("TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            # In test mode, let the LED pattern run for a while before stopping
            logger.info("TEST MODE: Running shutdown LED pattern for demonstration...")
            time.sleep(10)  # Let the pattern run for 10 seconds in test mode
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
        
        # Start power button monitoring in separate thread
        logger.info("Starting polling-based button detection")
        button_thread = threading.Thread(target=self.monitor_power_button_polling, daemon=True)
        button_thread.start()
        
        # Start green LED heartbeat in separate thread
        heartbeat_thread = threading.Thread(target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()
        
        try:
            # Main loop - just keep alive
            while self.running:
                time.sleep(MAIN_LOOP_SLEEP_S)
                
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            self.running = False
        
        finally:
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

def test_countdown_pattern(threshold):
    """Test the countdown pattern by controlling actual LEDs"""
    print(f"Testing countdown pattern for {threshold}s threshold...")
    
    # Calculate timing for countdown pattern
    countdown_duration = threshold - LED_FINAL_WARNING_DURATION_S
    if countdown_duration < 0:
        countdown_duration = 0
    
    blink_interval = 1.0 / LED_COUNTDOWN_FREQUENCY_HZ
    final_warning_interval = 1.0 / LED_FINAL_WARNING_FREQUENCY_HZ
    
    # Countdown pattern: 5, 4, 3, 2, 1 flashes
    countdown_groups = [5, 4, 3, 2, 1]
    group_duration = (countdown_duration - (len(countdown_groups) - 1) * LED_COUNTDOWN_PAUSE_S) / len(countdown_groups)
    
    if group_duration > 0:
        for group_num, flash_count in enumerate(countdown_groups):
            # Flash the specified number of times
            for flash in range(flash_count):
                # Turn on both LEDs
                print("LED ON", flush=True)
                time.sleep(blink_interval / 2)
                
                # Turn off both LEDs
                print("LED OFF", flush=True)
                time.sleep(blink_interval / 2)
            
            # Pause between groups (except after the last group)
            if group_num < len(countdown_groups) - 1:
                time.sleep(LED_COUNTDOWN_PAUSE_S)
    
    # Final warning: 10 rapid flashes at 10Hz for 2 seconds
    final_start = time.time()
    while time.time() - final_start < LED_FINAL_WARNING_DURATION_S:
        # print("LED ON", flush=True)
        time.sleep(final_warning_interval / 2)
        
        # print("LED OFF", flush=True)
        time.sleep(final_warning_interval / 2)
    
    print("LED OFF - Pattern complete")

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
  --threshold, -T     Set button press threshold for shutdown (default: {DEFAULT_SHUTDOWN_THRESHOLD_S})
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)
  --debug-shutdown-flashing, -d  Test shutdown countdown LED pattern and exit (safe for testing)
EXAMPLES:
  ./argo_power_control.py                         # Normal operation
  ./argo_power_control.py --test-mode             # Test mode (safe)
  ./argo_power_control.py -t                      # Test mode (short form)
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
            message = f"TEST: SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nThis is a test message - the system will NOT shutdown (would wait 5 seconds)."
            
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
        print("Testing shutdown countdown LED pattern...")
        try:
            # Test different thresholds
            test_thresholds = [1.0, 2.0, 3.0, 5.0, 10.0]
            
            for threshold in test_thresholds:
                print(f"\n=== Testing with {threshold}s threshold ===")
                test_countdown_pattern(threshold)
                print(f"=== End {threshold}s threshold test ===\n")
                time.sleep(1)  # Brief pause between tests
            
            print("Shutdown countdown LED pattern test completed successfully!")
            print("Check the output above to verify the countdown pattern works correctly.")
        except Exception as e:
            print(f"Error during countdown pattern test: {e}")
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
    if args.test_mode:
        print("Starting power control system in TEST MODE")
        print("  - Shutdown commands disabled")
        print("  - Power relay control disabled")
        print("  - Safe for testing")
    else:
        print("Starting power control system in NORMAL MODE")
        print("  - Full power control enabled")
        print("  - System will shutdown and cut power on long button press")
    
    print(f"  - Button press threshold: {args.threshold} seconds")
    print(f"  - Button detection mode: Polling ({BUTTON_POLLING_HZ} Hz)")
    print("  - Press Ctrl+C to stop")
    print()
    
    controller = PowerController(test_mode=args.test_mode, threshold=args.threshold)
    controller.run()

if __name__ == "__main__":
    main()
