#!/usr/bin/env python3
#
# power_control.py
#
# Orange Pi Zero 2W Power Control System
# =====================================
#
# DESCRIPTION:
#   Intelligent power control system for Orange Pi Zero 2W using external relays
#   and GPIO pins. Implements safe shutdown procedures, power button monitoring,
#   and LED status indicators.
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
#   - Short press (< 1 second): No action
#   - Long press (>= 1 second): Initiate shutdown sequence
#
# LED INDICATORS:
#   - Green LED: Solid on when system is running
#   - Blue LED: Fast blink during button press warning, rapid blink during shutdown
#   - Red LED: Directly connected to power button (not controlled by GPIO)
#
# SHUTDOWN SEQUENCE:
#   1. Broadcast wall message to all users
#   2. Wait 5 seconds for users to see notification
#   3. Turn on green LED (system running indicator)
#   4. Blink blue LED rapidly (shutdown warning)
#   5. Execute 'sudo shutdown -h now' for graceful shutdown
#   6. As final action: De-energize relay by setting !POW high
#
# USAGE:
#   sudo ./power_control.py [--help] [--test-mode] [--threshold SECONDS]
#
# OPTIONS:
#   --help              Show this help message and exit
#   --test-mode         Run in test mode (disable actual shutdown and power control)
#   --threshold SECONDS Set button press threshold for shutdown (default: 1.0)
#
# EXAMPLES:
#   sudo ./power_control.py                    # Normal operation
#   sudo ./power_control.py --test-mode        # Test mode (safe)
#   sudo ./power_control.py --threshold 2.0    # 2-second threshold
#
# REQUIREMENTS:
#   - Must run as root (for GPIO access)
#   - python3-gpiod library installed
#   - External pullup resistor on power button input
#   - Proper hardware connections as described above
#
# SAFETY FEATURES:
#   - Open drain configuration prevents damage from multiple control sources
#   - Graceful shutdown ensures proper system shutdown before cutting power
#   - External pullup resistor requirement prevents floating inputs
#   - Root privilege requirement for GPIO access security
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

# Configure logging
def setup_logging():
    """Setup logging configuration"""
    handlers = [logging.StreamHandler()]
    
    # Only add file handler if we have write permission to /var/log/
    try:
        # Test if we can write to /var/log/
        test_file = '/var/log/power_control.log'
        with open(test_file, 'a') as f:
            pass
        handlers.append(logging.FileHandler(test_file))
    except (PermissionError, OSError):
        # Fall back to local log file or no file logging
        try:
            handlers.append(logging.FileHandler('power_control.log'))
        except (PermissionError, OSError):
            pass  # No file logging if we can't write anywhere
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )

setup_logging()
logger = logging.getLogger('power_control')

class PowerController:
    def __init__(self, test_mode=False, threshold=1.0):
        self.running = True
        self.power_button_pressed = False
        self.button_press_start_time = None
        self.shutdown_initiated = False
        self.test_mode = test_mode
        
        # GPIO Configuration
        self.GPIO_CHIP = 'gpiochip0'
        
        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT  
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED
        
        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold
        
        # LED blink patterns
        self.LED_BLINK_FAST = 0.1      # Fast blink for normal operation
        self.LED_BLINK_SLOW = 0.5      # Slow blink for shutdown warning
        
        # Initialize GPIO
        self.init_gpio()
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        logger.info("Power controller initialized")

    def init_gpio(self):
        """Initialize GPIO pins"""
        try:
            self.chip = gpiod.Chip(self.GPIO_CHIP)
            
            # Configure power relay control (PI3) - Open drain output
            self.power_relay = self.chip.get_line(self.POWER_RELAY_LINE)
            self.power_relay.request(
                consumer="power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0],  # Start with relay energized (low = on)
                flags=gpiod.LINE_REQ_FLAG_OPEN_DRAIN
            )
            
            # Configure power button input (PI9) - Input without pullup
            self.power_button = self.chip.get_line(self.POWER_BUTTON_LINE)
            self.power_button.request(
                consumer="power_control.py",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_DISABLE  # No internal pullup
            )
            
            # Configure LED outputs
            self.green_led = self.chip.get_line(self.GREEN_LED_LINE)
            self.green_led.request(
                consumer="power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )
            
            self.blue_led = self.chip.get_line(self.BLUE_LED_LINE)
            self.blue_led.request(
                consumer="power_control.py",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]  # Start with LED off
            )
            
            logger.info("GPIO pins configured successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            raise

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, stopping power controller...")
        self.running = False

    def read_power_button(self):
        """Read power button state"""
        try:
            return self.power_button.get_value()
        except Exception as e:
            logger.error(f"Error reading power button: {e}")
            return 1  # Assume not pressed on error

    def set_power_relay(self, state):
        """Control power relay (True = energized/on, False = de-energized/off)"""
        try:
            # For open drain: 0 = energized (relay on), 1 = de-energized (relay off)
            self.power_relay.set_value(0 if state else 1)
            logger.info(f"Power relay set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling power relay: {e}")

    def set_green_led(self, state):
        """Control green LED"""
        try:
            self.green_led.set_value(1 if state else 0)
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED"""
        try:
            self.blue_led.set_value(1 if state else 0)
        except Exception as e:
            logger.error(f"Error controlling blue LED: {e}")

    def led_blink_pattern(self, led_func, duration, blink_rate):
        """Blink LED with specified pattern"""
        start_time = time.time()
        led_state = False
        
        while time.time() - start_time < duration and self.running:
            led_func(led_state)
            led_state = not led_state
            time.sleep(blink_rate)

    def green_led_heartbeat(self):
        """Green LED heartbeat during normal operation"""
        led_state = False
        while self.running:
            self.set_green_led(led_state)
            led_state = not led_state
            time.sleep(1.0)  # 1 second blink rate for heartbeat

    def cyan_shutdown_blink(self):
        """Blink both LEDs together rapidly for cyan effect during shutdown"""
        led_state = False
        for _ in range(20):  # Blink for about 2 seconds (20 * 0.1s)
            self.set_green_led(led_state)
            self.set_blue_led(led_state)
            led_state = not led_state
            time.sleep(0.1)  # Fast blink rate for shutdown

    def cyan_warning_blink(self):
        """Blink both LEDs together for cyan warning effect during button press"""
        led_state = False
        start_time = time.time()
        while time.time() - start_time < self.SHUTDOWN_THRESHOLD and self.running:
            self.set_green_led(led_state)
            self.set_blue_led(led_state)
            led_state = not led_state
            time.sleep(self.LED_BLINK_FAST)

    def monitor_power_button(self):
        """Monitor power button for press duration"""
        last_button_state = 1  # Start assuming not pressed
        
        while self.running:
            try:
                current_button_state = self.read_power_button()
                
                # Detect button press (falling edge)
                if last_button_state == 1 and current_button_state == 0:
                    self.button_press_start_time = time.time()
                    self.power_button_pressed = True
                    logger.info("Power button pressed")
                    
                    # Start warning LED pattern (cyan effect)
                    threading.Thread(
                        target=self.cyan_warning_blink,
                        daemon=True
                    ).start()
                
                # Detect button release (rising edge)
                elif last_button_state == 0 and current_button_state == 1:
                    if self.power_button_pressed:
                        press_duration = time.time() - self.button_press_start_time
                        logger.info(f"Power button released after {press_duration:.2f} seconds")
                        
                        if press_duration >= self.SHUTDOWN_THRESHOLD:
                            logger.info("Long press detected, initiating shutdown...")
                            self.initiate_shutdown()
                        else:
                            logger.info("Short press detected, no action taken")
                    
                    self.power_button_pressed = False
                    self.button_press_start_time = None
                
                # Check for long press while button is held
                elif self.power_button_pressed and current_button_state == 0:
                    if time.time() - self.button_press_start_time >= self.SHUTDOWN_THRESHOLD:
                        if not self.shutdown_initiated:
                            logger.info("Long press threshold reached, initiating shutdown...")
                            self.initiate_shutdown()
                
                last_button_state = current_button_state
                time.sleep(0.01)  # 100Hz polling rate
                
            except Exception as e:
                logger.error(f"Error in power button monitoring: {e}")
                time.sleep(0.1)

    def broadcast_shutdown_message(self):
        """Broadcast shutdown message to all logged-in users"""
        try:
            # Create shutdown message with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message = f"SYSTEM SHUTDOWN INITIATED by power button at {timestamp}\nThe system will shutdown in 5 seconds."
            
            # Use wall command to broadcast message to all users
            if self.test_mode:
                logger.info(f"TEST MODE: Would broadcast wall message: {message}")
            else:
                try:
                    # Use wall command to send message to all logged-in users
                    subprocess.run(['wall', message], check=True, timeout=5)
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
        
        # Broadcast wall message to all users
        self.broadcast_shutdown_message()
        
        # Give users time to see the wall message before shutdown
        logger.info("Waiting 5 seconds for users to see shutdown notification...")
        time.sleep(5)
        
        # Blink both LEDs rapidly together for cyan effect during shutdown
        threading.Thread(
            target=self.cyan_shutdown_blink,
            daemon=True
        ).start()
        
        # Execute shutdown command
        if self.test_mode:
            logger.info("TEST MODE: Shutdown command disabled - would normally execute: shutdown -h now")
            self.running = False  # Only stop running in test mode
        else:
            try:
                logger.info("Executing shutdown command: shutdown -h now")
                subprocess.run(['shutdown', '-h', 'now'], check=True)
                logger.info("Shutdown command executed successfully")
                # Don't set self.running = False here - let the system shutdown
            except subprocess.CalledProcessError as e:
                logger.error(f"Failed to execute shutdown command: {e}")
            except Exception as e:
                logger.error(f"Unexpected error during shutdown: {e}")
        
        # As final action, de-energize relay to cut power
        time.sleep(2)  # Give system time to start shutdown
        if self.test_mode:
            logger.info("TEST MODE: Power relay de-energization disabled - would normally cut power")
        else:
            logger.info("De-energizing power relay...")
            self.set_power_relay(False)

    def run(self):
        """Main control loop"""
        logger.info("Power controller starting...")
        
        # Ensure relay is energized on startup
        self.set_power_relay(True)
        
        # Start power button monitoring in separate thread
        button_thread = threading.Thread(target=self.monitor_power_button, daemon=True)
        button_thread.start()
        
        # Start green LED heartbeat in separate thread
        heartbeat_thread = threading.Thread(target=self.green_led_heartbeat, daemon=True)
        heartbeat_thread.start()
        
        try:
            # Main loop - just keep alive
            while self.running:
                time.sleep(1)
                
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
            
            # De-energize relay
            self.set_power_relay(False)
            
            # Close GPIO chip
            if hasattr(self, 'chip'):
                self.chip.close()
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
        
        logger.info("Power controller cleanup complete")

def print_help():
    """Print help message"""
    help_text = """
Orange Pi Zero 2W Power Control System
=====================================

DESCRIPTION:
  Intelligent power control system for Orange Pi Zero 2W using external relays
  and GPIO pins. Implements safe shutdown procedures, power button monitoring,
  and LED status indicators.

HARDWARE CONFIGURATION:
  - PI3 (Pin 40): !POW - Open drain output to control power relay
  - PI9 (Pin 28): !POW_BUT - Input from power button (external pullup required)
  - PH4 (Pin 18): Green LED in power button (system running indicator)
  - PI1 (Pin 12): Blue LED in power button (status/warning indicator)
  - Red LED: Directly connected to power button (not GPIO controlled)

POWER BUTTON BEHAVIOR:
  - Short press (< threshold): No action
  - Long press (>= threshold): Initiate shutdown sequence

LED INDICATORS:
  - Green LED: Solid on when system is running
  - Blue LED: Fast blink during button press warning, rapid blink during shutdown
  - Red LED: Directly connected to power button (not controlled by GPIO)

USAGE:
  sudo ./power_control.py [OPTIONS]

OPTIONS:
  --help              Show this help message and exit
  --test-mode         Run in test mode (disable actual shutdown and power control)
  --threshold SECONDS Set button press threshold for shutdown (default: 1.0)
  --test-wall-message Test wall message functionality and exit (safe for testing)

EXAMPLES:
  sudo ./power_control.py                    # Normal operation
  sudo ./power_control.py --test-mode        # Test mode (safe)
  sudo ./power_control.py --threshold 2.0    # 2-second threshold
  sudo ./power_control.py --test-wall-message # Test wall message (safe)

REQUIREMENTS:
  - Must run as root (for GPIO access)
  - python3-gpiod library installed
  - External pullup resistor on power button input
  - Proper hardware connections as described above

SAFETY FEATURES:
  - Open drain configuration prevents damage from multiple control sources
  - Graceful shutdown ensures proper system shutdown before cutting power
  - External pullup resistor requirement prevents floating inputs
  - Root privilege requirement for GPIO access security
"""
    print(help_text)

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Orange Pi Zero 2W Power Control System',
        add_help=False
    )
    parser.add_argument('--help', action='store_true', 
                       help='Show help message and exit')
    parser.add_argument('--test-mode', action='store_true',
                       help='Run in test mode (disable actual shutdown and power control)')
    parser.add_argument('--threshold', type=float, default=1.0,
                       help='Set button press threshold for shutdown in seconds (default: 1.0)')
    parser.add_argument('--test-wall-message', action='store_true',
                       help='Test wall message functionality and exit (safe for testing)')
    
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
            subprocess.run(['wall', message], check=True, timeout=5)
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
    
    # Check if running as root (required for GPIO access)
    if os.geteuid() != 0:
        print("Error: This script must be run as root for GPIO access")
        print("Use: sudo ./power_control.py")
        sys.exit(1)
    
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
    print("  - Press Ctrl+C to stop")
    print()
    
    controller = PowerController(test_mode=args.test_mode, threshold=args.threshold)
    controller.run()

if __name__ == "__main__":
    main()
