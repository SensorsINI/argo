#!/usr/bin/env python3
"""
argo_boot_indicator.py

Early boot LED indicator for Argo power control system.
Shows early heartbeat until main power control service takes over.

This service runs very early in boot to provide immediate visual feedback
with a simple heartbeat pattern. It continues running until the main
power control service starts and takes over GPIO control.

Systemd coordinates the handoff - when argo_power_control.service starts,
it conflicts with this service, causing systemd to gracefully terminate
this process with SIGTERM, ensuring clean GPIO release.

HARDWARE:
  - PH4 (Pin 18): Green LED (active LOW - GPIO 228)
  - LED ON = GPIO LOW (0)
  - LED OFF = GPIO HIGH (1)

USAGE:
  Automatically invoked by systemd during early boot
  Can be manually tested: sudo python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
  Terminated by: systemd when argo_power_control.service starts
"""

import time
import sys
import signal
import logging
from datetime import datetime

# Try to import gpiod
try:
    import gpiod
except ImportError:
    print("ERROR: gpiod library not available - cannot run boot indicator")
    print("Install with: sudo apt-get install python3-libgpiod")
    sys.exit(1)

# GPIO Configuration
GPIO_CHIP = '/dev/gpiochip0'
GREEN_LED_LINE = 228  # PH4 (Pin 18) - Green LED in power button

# LED polarity (Rev3 PCB - Active LOW)
LED_ON_STATE = 0   # GPIO LOW = LED ON
LED_OFF_STATE = 1  # GPIO HIGH = LED OFF

# Initial flash pattern configuration
INITIAL_FLASH_COUNT = 3       # Number of initial flashes
INITIAL_FLASH_ON_TIME = 0.15  # LED on time (seconds)
INITIAL_FLASH_OFF_TIME = 0.15 # LED off time (seconds)

# Heartbeat pattern configuration
HEARTBEAT_FREQUENCY_HZ = 2.0  # Hz boot indicator heartbeat
HEARTBEAT_DUTY_CYCLE = 0.2    # % duty cycle

# Global flag for graceful shutdown
running = True

# Persistent logging setup
PERSISTENT_LOG_DIR = "/var/log.hdd/persistent"
LOG_FILENAME = f"{PERSISTENT_LOG_DIR}/argo-boot-indicator.log"

def setup_logging():
    """Setup logging to both console and persistent log file"""
    # Create log directory if it doesn't exist
    import os
    try:
        os.makedirs(PERSISTENT_LOG_DIR, exist_ok=True)
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(sys.stdout),  # Console output
                logging.FileHandler(LOG_FILENAME)    # Persistent log file
            ]
        )
        
        logger = logging.getLogger('argo_boot_indicator')
        logger.info(f"Boot indicator logging initialized - persistent log: {LOG_FILENAME}")
        return logger
        
    except PermissionError:
        # Fallback to console-only logging if persistent log fails
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[logging.StreamHandler(sys.stdout)]
        )
        
        logger = logging.getLogger('argo_boot_indicator')
        logger.warning(f"Permission denied for persistent log {LOG_FILENAME} - using console logging only")
        return logger


def signal_handler(signum, frame):
    """Handle termination signals gracefully"""
    global running
    logger = logging.getLogger('argo_boot_indicator')
    logger.info(f"Received signal {signum}, shutting down gracefully...")
    running = False


def run_boot_indicator():
    """Run early boot heartbeat until terminated by main power control service"""
    global running
    chip = None
    green_led = None
    logger = logging.getLogger('argo_boot_indicator')

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Open GPIO chip
        chip = gpiod.Chip(GPIO_CHIP)
        logger.info(f"Boot indicator: Opened GPIO chip {GPIO_CHIP}")

        # Get GREEN LED line
        green_led = chip.get_line(GREEN_LED_LINE)

        # Request line as output (start with LED OFF)
        green_led.request(
            consumer="argo_boot_indicator",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[LED_OFF_STATE]
        )
        logger.info("Boot indicator: GPIO configured, starting early boot pattern")

        # Initial flash sequence to show boot started
        logger.info(f"Boot indicator: Initial flash sequence ({INITIAL_FLASH_COUNT} flashes)")
        for i in range(INITIAL_FLASH_COUNT):
            if not running:
                break
            green_led.set_value(LED_ON_STATE)
            time.sleep(INITIAL_FLASH_ON_TIME)
            green_led.set_value(LED_OFF_STATE)
            time.sleep(INITIAL_FLASH_OFF_TIME)

        # Brief pause after initial flashes
        time.sleep(0.3)

        # Continue with heartbeat pattern until terminated
        logger.info(f"Boot indicator: Starting heartbeat ({HEARTBEAT_FREQUENCY_HZ}Hz)")
        heartbeat_period = 1.0 / HEARTBEAT_FREQUENCY_HZ
        on_time = heartbeat_period * HEARTBEAT_DUTY_CYCLE
        off_time = heartbeat_period * (1.0 - HEARTBEAT_DUTY_CYCLE)

        while running:
            # LED ON
            green_led.set_value(LED_ON_STATE)

            # Sleep in small increments for responsive shutdown
            elapsed = 0
            while elapsed < on_time and running:
                time.sleep(0.05)
                elapsed += 0.05

            if not running:
                break

            # LED OFF
            green_led.set_value(LED_OFF_STATE)

            # Sleep in small increments for responsive shutdown
            elapsed = 0
            while elapsed < off_time and running:
                time.sleep(0.05)
                elapsed += 0.05

        logger.info("Boot indicator: Heartbeat stopped, cleaning up")
        return 0

    except FileNotFoundError:
        logger.error(f"ERROR: GPIO chip not found: {GPIO_CHIP}")
        logger.error("This may be a hardware or kernel module issue")
        return 1
    except PermissionError:
        logger.error("ERROR: Permission denied accessing GPIO")
        logger.error("This script must run as root or with gpio group access")
        return 1
    except Exception as e:
        logger.error(f"ERROR: Failed to run boot indicator: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return 1
    finally:
        # CRITICAL: Always release GPIO, even if an exception occurred
        # This ensures the main power control service can claim the GPIO
        logger.info("Boot indicator: Releasing GPIO resources")
        try:
            if green_led is not None:
                # Turn off LED before releasing
                try:
                    green_led.set_value(LED_OFF_STATE)
                except:
                    pass
                green_led.release()
                logger.info("Boot indicator: GPIO line released")
        except Exception as e:
            logger.warning(f"WARNING: Error releasing GPIO line: {e}")

        try:
            if chip is not None:
                chip.close()
                logger.info("Boot indicator: GPIO chip closed")
        except Exception as e:
            logger.warning(f"WARNING: Error closing GPIO chip: {e}")


def main():
    """Main entry point"""
    # Initialize logging first
    logger = setup_logging()
    logger.info("Argo boot indicator starting")
    
    # Run the boot indicator
    result = run_boot_indicator()
    
    logger.info(f"Argo boot indicator exiting with code {result}")
    return result


if __name__ == "__main__":
    sys.exit(main())
