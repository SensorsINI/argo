#!/usr/bin/env python3
"""
argo_boot_indicator.py

Early boot LED indicator for Argo power control system.
Briefly flashes the GREEN LED to indicate boot progress.

This is a one-shot service that runs very early in boot to provide
immediate visual feedback. It claims the GPIO briefly, flashes the LED,
then releases it for the main power control service.

HARDWARE:
  - PH4 (Pin 18): Green LED (active LOW - GPIO 228)
  - LED ON = GPIO LOW (0)
  - LED OFF = GPIO HIGH (1)

USAGE:
  Automatically invoked by systemd during early boot
  Can be manually tested: sudo python3 /home/orangepi/argo/power_control/argo_boot_indicator.py
"""

import time
import sys

# Try to import gpiod
try:
    import gpiod
except ImportError:
    print("ERROR: gpiod library not available - cannot flash boot indicator LED")
    print("Install with: sudo apt-get install python3-libgpiod")
    sys.exit(1)

# GPIO Configuration
GPIO_CHIP = '/dev/gpiochip0'
GREEN_LED_LINE = 228  # PH4 (Pin 18) - Green LED in power button

# LED polarity (Rev3 PCB - Active LOW)
LED_ON_STATE = 0   # GPIO LOW = LED ON
LED_OFF_STATE = 1  # GPIO HIGH = LED OFF

# Boot indicator pattern configuration
FLASH_COUNT = 3           # Number of times to flash
FLASH_ON_TIME = 0.15      # LED on time (seconds)
FLASH_OFF_TIME = 0.15     # LED off time (seconds)
FINAL_DELAY = 0.5         # Final delay before releasing GPIO (seconds)


def flash_boot_indicator():
    """Flash the GREEN LED to indicate boot progress"""
    chip = None
    green_led = None
    
    try:
        # Open GPIO chip
        chip = gpiod.Chip(GPIO_CHIP)
        
        # Get GREEN LED line
        green_led = chip.get_line(GREEN_LED_LINE)
        
        # Request line as output (start with LED OFF)
        green_led.request(
            consumer="argo_boot_indicator",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[LED_OFF_STATE]
        )
        
        # Flash the LED to indicate boot progress
        for i in range(FLASH_COUNT):
            # LED ON
            green_led.set_value(LED_ON_STATE)
            time.sleep(FLASH_ON_TIME)
            
            # LED OFF
            green_led.set_value(LED_OFF_STATE)
            time.sleep(FLASH_OFF_TIME)
        
        # Brief delay before releasing GPIO
        time.sleep(FINAL_DELAY)
        
        return 0
        
    except FileNotFoundError:
        print(f"ERROR: GPIO chip not found: {GPIO_CHIP}")
        print("This may be a hardware or kernel module issue")
        return 1
    except PermissionError:
        print("ERROR: Permission denied accessing GPIO")
        print("This script must run as root or with gpio group access")
        return 1
    except Exception as e:
        print(f"ERROR: Failed to flash boot indicator LED: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # CRITICAL: Always release GPIO, even if an exception occurred
        # This ensures the main power control service can claim the GPIO
        try:
            if green_led is not None:
                green_led.release()
                print("GPIO line released")
        except Exception as e:
            print(f"WARNING: Error releasing GPIO line: {e}")
        
        try:
            if chip is not None:
                chip.close()
                print("GPIO chip closed")
        except Exception as e:
            print(f"WARNING: Error closing GPIO chip: {e}")


def main():
    """Main entry point"""
    return flash_boot_indicator()


if __name__ == "__main__":
    sys.exit(main())
