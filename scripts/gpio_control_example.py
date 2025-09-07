#!/usr/bin/env python3
#
# gpio_control_example.py
#
# Description:
#   A script to demonstrate GPIO control on an Orange Pi Zero 2W using the
#   libgpiod library. This is the modern, recommended method for GPIO
#   manipulation on Linux systems.
#
# Prerequisites:
#   - The libgpiod library and its Python bindings must be installed.
#     On Debian-based systems (like Armbian for Orange Pi), you can install it with:
#     sudo apt-get update
#     sudo apt-get install gpiod python3-gpiod
#
# Pin Numbering:
#   libgpiod identifies GPIOs by their chip and line offset, not by the physical
#   pin number on the header or the SoC name (e.g., "PA6").
#
#   To find the correct chip and line offset for a physical pin, use the
#   `gpioinfo` command in your terminal.
#
#   Example for Orange Pi Zero 2W (Allwinner H616):
#   $ gpioinfo
#   gpiochip0 - 256 lines:
#       ...
#       line   6:      "PA6"       unused   input  active-high
#       ...
#       line 226:      "PH2"       unused   input  active-high
#       line 227:      "PH3"       unused   input  active-high
#       ...
#
#   From this output, we can see:
#   - The GPIO chip is 'gpiochip0'.
#   - The GPIO pin named "PA6" (physical pin 7) corresponds to line offset 6.
#   - The GPIO pin named "PH2" (physical pin 11) corresponds to line offset 226.
#   - The GPIO pin named "PH3" (physical pin 13) corresponds to line offset 227.

import gpiod
import time
import sys

# --- Configuration ---
# Use `gpioinfo` to find the correct chip for your board.
# For Orange Pi Zero 2W, it's typically 'gpiochip0'.
GPIO_CHIP = 'gpiochip0'

# Define the line offsets for the pins you want to control.
# See "Pin Numbering" section above for how to find these.
OUTPUT_PIN_OFFSET = 6      # Physical Pin 7 (PA6)
INPUT_PULL_UP_PIN_OFFSET = 226 # Physical Pin 11 (PH2)
INPUT_PULL_DOWN_PIN_OFFSET = 227 # Physical Pin 13 (PH3)

def main():
    """
    Main function to demonstrate GPIO control.
    """
    # Get a handle to the GPIO chip
    try:
        chip = gpiod.Chip(GPIO_CHIP)
    except (FileNotFoundError, PermissionError) as e:
        print(f"Error: Could not open GPIO chip '{GPIO_CHIP}': {e}", file=sys.stderr)
        print("Please check the chip name with 'gpioinfo' and run with 'sudo' or as a user in the 'gpio' group.", file=sys.stderr)
        sys.exit(1)

    # --- 1. Configure a pin as an OUTPUT ---
    output_line = chip.get_line(OUTPUT_PIN_OFFSET)
    output_line.request(
        consumer="gpio_control_example.py",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_vals=[0] # Start with the pin low
    )
    print(f"Line {OUTPUT_PIN_OFFSET} configured as OUTPUT.")

    # --- 2. Configure a pin as an INPUT with PULL-UP ---
    input_pull_up_line = chip.get_line(INPUT_PULL_UP_PIN_OFFSET)
    input_pull_up_line.request(
        consumer="gpio_control_example.py",
        type=gpiod.LINE_REQ_DIR_IN,
        flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
    )
    print(f"Line {INPUT_PULL_UP_PIN_OFFSET} configured as INPUT with PULL-UP.")

    # --- 3. Configure a pin as an INPUT with PULL-DOWN ---
    # Note: Not all SoCs/pins support pull-down. If this fails, your hardware
    # may not support it on this specific pin.
    input_pull_down_line = chip.get_line(INPUT_PULL_DOWN_PIN_OFFSET)
    try:
        input_pull_down_line.request(
            consumer="gpio_control_example.py",
            type=gpiod.LINE_REQ_DIR_IN,
            flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
        )
        print(f"Line {INPUT_PULL_DOWN_PIN_OFFSET} configured as INPUT with PULL-DOWN.")
    except OSError as e:
        print(f"Warning: Could not set PULL-DOWN on line {INPUT_PULL_DOWN_PIN_OFFSET}: {e}", file=sys.stderr)
        print("The pin may not support pull-down resistors. Continuing without it.", file=sys.stderr)
        input_pull_down_line = None

    print("\n--- Starting GPIO Demo ---")
    print("Blinking output pin and reading input pins for 10 seconds.")
    print("Press Ctrl+C to exit.")

    try:
        for i in range(20):
            # Toggle the output pin
            output_line.set_value(i % 2)

            # Read the input pins
            pull_up_val = input_pull_up_line.get_value()
            pull_down_val_str = "N/A"
            if input_pull_down_line:
                pull_down_val = input_pull_down_line.get_value()
                pull_down_val_str = "HIGH" if pull_down_val == 1 else "LOW"

            print(
                f"Output (line {OUTPUT_PIN_OFFSET}): {'HIGH' if (i % 2) == 1 else 'LOW'} | "
                f"Input PU (line {INPUT_PULL_UP_PIN_OFFSET}): {'HIGH' if pull_up_val == 1 else 'LOW'} | "
                f"Input PD (line {INPUT_PULL_DOWN_PIN_OFFSET}): {pull_down_val_str}"
            )
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        print("Releasing GPIO lines...")
        chip.close()
        print("GPIO lines released. Done.")

if __name__ == "__main__":
    main()