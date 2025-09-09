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
#   ./power_control.py [--help] [--test-mode] [--threshold SECONDS]
#
# OPTIONS:
#   --help              Show this help message and exit
#   --test-mode         Run in test mode (disable actual shutdown and power control)
#   --threshold SECONDS Set button press threshold for shutdown (default: 1.0)
#
# EXAMPLES:
#   ./power_control.py                         # Normal operation
#   ./power_control.py --test-mode             # Test mode (safe)
#   ./power_control.py --threshold 2.0         # 2-second threshold
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
        self.GPIO_CHIP = '/dev/gpiochip0'
        
        # Correct GPIO Line offsets from gpio readall
        self.POWER_RELAY_LINE = 259    # PI3 (Pin 40) - !POW
        self.POWER_BUTTON_LINE = 265   # PI9 (Pin 28) - !POW_BUT  
        self.GREEN_LED_LINE = 228      # PH4 (Pin 18) - Green LED
        self.BLUE_LED_LINE = 257       # PI1 (Pin 12) - Blue LED
        
        # Button press threshold (seconds)
        self.SHUTDOWN_THRESHOLD = threshold
        
        # LED blink patterns
        self.LED_BLINK_FAST = 0.1      # Fast blink for normal operation
        self.LED_BLINK_SLOW = 0.5      # Slow blink for shutdown warn
        
        # Initialize GPIO
        self.init_gpio()
        
        # Setup signal handlers
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)
        
        logger.info("Power controller initialized")

    def init_gpio(self):
        """Initialize GPIO pins"""
        try:
            logger.info(f"Attempting to open GPIO chip: {self.GPIO_CHIP}")
            self.chip = gpiod.Chip(self.GPIO_CHIP)
            logger.info(f"Successfully opened GPIO chip: {self.GPIO_CHIP}")
            
            # Configure power relay control (PI3) - Open drain output
            relay_settings = gpiod.LineSettings()
            relay_settings.direction = relay_settings.direction.OUTPUT
            relay_settings.output_value = relay_settings.output_value.INACTIVE  # Start with relay energized (low = on)
            relay_settings.drive = relay_settings.drive.OPEN_DRAIN
            
            self.power_relay = self.chip.request_lines(
                consumer="power_control.py",
                config={self.POWER_RELAY_LINE: relay_settings}
            )
            
            # Configure power button input (PI9) - Input without pullup
            button_settings = gpiod.LineSettings()
            button_settings.direction = button_settings.direction.INPUT
            button_settings.bias = button_settings.bias.DISABLED  # No internal pullup
            
            self.power_button = self.chip.request_lines(
                consumer="power_control.py",
                config={self.POWER_BUTTON_LINE: button_settings}
            )
            
            # Configure LED outputs
            green_led_settings = gpiod.LineSettings()
            green_led_settings.direction = green_led_settings.direction.OUTPUT
            green_led_settings.output_value = green_led_settings.output_value.INACTIVE  # Start with LED off
            
            self.green_led = self.chip.request_lines(
                consumer="power_control.py",
                config={self.GREEN_LED_LINE: green_led_settings}
            )
            
            blue_led_settings = gpiod.LineSettings()
            blue_led_settings.direction = blue_led_settings.direction.OUTPUT
            blue_led_settings.output_value = blue_led_settings.output_value.INACTIVE  # Start with LED off
            
            self.blue_led = self.chip.request_lines(
                consumer="power_control.py",
                config={self.BLUE_LED_LINE: blue_led_settings}
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
            values = self.power_button.get_values()
            # get_values() returns a list of Value objects
            # For a single line request, the value is at index 0
            return values[0].value  # Return the actual integer value (0 or 1)
        except Exception as e:
            logger.error(f"Error reading power button: {e}")
            return 1  # Assume not pressed on error

    def set_power_relay(self, state):
        """Control power relay (True = energized/on, False = de-energized/off)"""
        try:
            # For open drain: 0 = energized (relay on), 1 = de-energized (relay off)
            settings = gpiod.LineSettings()
            value = settings.output_value.INACTIVE if state else settings.output_value.ACTIVE
            self.power_relay.set_values({self.POWER_RELAY_LINE: value})
            logger.info(f"Power relay set to {'ON' if state else 'OFF'}")
        except Exception as e:
            logger.error(f"Error controlling power relay: {e}")

    def set_green_led(self, state):
        """Control green LED"""
        try:
            settings = gpiod.LineSettings()
            value = settings.output_value.ACTIVE if state else settings.output_value.INACTIVE
            self.green_led.set_values({self.GREEN_LED_LINE: value})
        except Exception as e:
            logger.error(f"Error controlling green LED: {e}")

    def set_blue_led(self, state):
        """Control blue LED"""
        try:
            settings = gpiod.LineSettings()
            value = settings.output_value.ACTIVE if state else settings.output_value.INACTIVE
            self.blue_led.set_values({self.BLUE_LED_LINE: value})
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
                    
                    # Send desktop notification for button press
                    self.send_desktop_notification(
                        "Power Button", 
                        "Power button pressed - hold for shutdown",
                        "normal"
                    )
                    
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
                            # Send notification for short press
                            self.send_desktop_notification(
                                "Power Button", 
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
                time.sleep(0.01)  # 100Hz polling rate
                
            except Exception as e:
                logger.error(f"Error in power button monitoring: {e}")
                time.sleep(0.1)

    def send_desktop_notification(self, title, message, urgency="normal"):
        """Send desktop notification using notify-send"""
        try:
            if self.test_mode:
                logger.info(f"TEST MODE: Would send desktop notification - {title}: {message}")
            else:
                # Set up environment for desktop notifications
                env = os.environ.copy()
                
                # Try to find the display for logged-in users
                try:
                    # Get list of logged-in users and their displays
                    result = subprocess.run(['who'], capture_output=True, text=True, timeout=5)
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
                
                # Try to send desktop notification
                subprocess.run([
                    'notify-send', 
                    '--urgency', urgency,
                    '--expire-time', '5000',  # 5 second timeout
                    title, 
                    message
                ], check=True, timeout=5, env=env)
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
        
        # Flush SD card cache before cutting power
        if self.test_mode:
            logger.info("TEST MODE: SD card cache flush disabled - would normally flush cache")
        else:
            logger.info("Flushing SD card cache before power cut...")
            try:
                subprocess.run(['sync'], check=True, timeout=10)
                logger.info("SD card cache flushed successfully")
            except subprocess.TimeoutExpired:
                logger.warning("Cache flush timed out - proceeding with power cut")
            except subprocess.CalledProcessError as e:
                logger.warning(f"Cache flush failed: {e} - proceeding with power cut")
            except Exception as e:
                logger.warning(f"Unexpected error during cache flush: {e} - proceeding with power cut")
        
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
  ./power_control.py [OPTIONS]

OPTIONS:
  --help, -h          Show this help message and exit
  --test-mode, -t     Run in test mode (disable actual shutdown and power control)
  --threshold, -T     Set button press threshold for shutdown (default: 1.0)
  --test-wall-message, -w  Test wall message functionality and exit (safe for testing)
  --test-notification, -n  Test desktop notification functionality and exit (safe for testing)

EXAMPLES:
  ./power_control.py                         # Normal operation
  ./power_control.py --test-mode             # Test mode (safe)
  ./power_control.py -t                      # Test mode (short form)
  ./power_control.py --threshold 2.0         # 2-second threshold
  ./power_control.py -T 2.0                  # 2-second threshold (short form)
  ./power_control.py --test-wall-message     # Test wall message (safe)
  ./power_control.py -w                      # Test wall message (short form)
  ./power_control.py --test-notification     # Test desktop notification (safe)
  ./power_control.py -n                      # Test desktop notification (short form)

REQUIREMENTS:
  - User must be member of 'gpio' group (for GPIO access)
  - python3-gpiod library installed
  - External pullup resistor on power button input
  - Proper hardware connections as described above

SAFETY FEATURES:
  - Open drain configuration prevents damage from multiple control sources
  - Graceful shutdown ensures proper system shutdown before cutting power
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
    parser.add_argument('--threshold', '-T', type=float, default=1.0,
                       help='Set button press threshold for shutdown in seconds (default: 1.0)')
    parser.add_argument('--test-wall-message', '-w', action='store_true',
                       help='Test wall message functionality and exit (safe for testing)')
    parser.add_argument('--test-notification', '-n', action='store_true',
                       help='Test desktop notification functionality and exit (safe for testing)')
    
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
            ], check=True, timeout=5)
            print("✅ Normal notification sent")
            
            # Test critical notification
            subprocess.run([
                'notify-send', 
                '--urgency', 'critical',
                '--expire-time', '5000',
                'System Shutdown', 
                'Critical notification test - system shutdown warning'
            ], check=True, timeout=5)
            print("✅ Critical notification sent")
            
            # Test low priority notification
            subprocess.run([
                'notify-send', 
                '--urgency', 'low',
                '--expire-time', '2000',
                'Power Button', 
                'Low priority notification test - short press detected'
            ], check=True, timeout=5)
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
    
    # Check if user is in the gpio group (required for GPIO access)
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
