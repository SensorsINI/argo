#!/usr/bin/env python3
#
# Python version of test_pwm_loop.sh
# Tests the argo_radio_servo kernel module by sweeping servo outputs
# and displaying radio inputs.
#

import sys
import time
import tty
import termios
import select
from pathlib import Path

# --- Configuration ---
SYS_BASE_PATH = Path("/sys/kernel/argo_radio_servo")
SERVO_RUDDER_PATH = SYS_BASE_PATH / "servo_rudder_pw_us"
SERVO_SAIL_PATH = SYS_BASE_PATH / "servo_sail_pw_us"
RADIO_RUDDER_PATH = SYS_BASE_PATH / "radio_rudder_pw_us"
RADIO_SAIL_PATH = SYS_BASE_PATH / "radio_sail_pw_us"

MIN_PW = 900
MAX_PW = 2100
STEP = 100
DELAY_S = 0.5
DEFAULT_PW = 1500

# --- Helper Functions ---

def get_initial_pw(path: Path) -> int:
    """Reads the initial pulse width from a sysfs file, or returns a default."""
    try:
        return int(path.read_text().strip())
    except (IOError, ValueError, FileNotFoundError):
        print(f"Warning: Could not read {path}, using default {DEFAULT_PW} us.", file=sys.stderr)
        return DEFAULT_PW

def read_sysfs_pw(path: Path) -> str:
    """Reads a pulse width from a sysfs file for display."""
    try:
        return path.read_text().strip()
    except (IOError, FileNotFoundError):
        return "N/A"

def write_sysfs_pw(path: Path, value: int):
    """Writes a pulse width to a sysfs file."""
    try:
        path.write_text(str(value))
    except IOError as e:
        # Don't crash if the file isn't writable, just print an error.
        # This might happen if the kernel module is unloaded during the run.
        print(f"\nError writing to {path}: {e}", file=sys.stderr)

def display_status(rudder_pw: int, sail_pw: int, paused: bool):
    """Clears the screen and displays the current status."""
    # ANSI escape code to clear screen and move cursor to top-left
    print("\033[H\033[J", end="")
    
    print("--- Radio Control Input Pulse Widths ---")
    print(f"Radio Rudder: {read_sysfs_pw(RADIO_RUDDER_PATH)} us")
    print(f"Radio Sail:   {read_sysfs_pw(RADIO_SAIL_PATH)} us")
    print("----------------------------------------")
    print("--- Servo Motor Output Pulse Widths (Sweeping) ---")
    print(f"Rudder (PWM2): {rudder_pw} us")
    print(f"Sail (PWM4):   {sail_pw} us")
    print("--------------------------------------------------")
    if paused:
        print("STATUS: PAUSED (Press Spacebar to RESUME)")
    else:
        print("STATUS: RUNNING (Press Spacebar to PAUSE)")
    sys.stdout.flush()

def get_key_non_blocking() -> str | None:
    """Reads a single key press without blocking. Returns None if no key is pressed."""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def get_key_blocking() -> str:
    """Reads a single key press, blocking until one is received."""
    return sys.stdin.read(1)

def main():
    """Main function to run the test loop."""
    # Check if sysfs path exists for better error reporting
    if not SYS_BASE_PATH.is_dir():
        print(f"Error: Sysfs path {SYS_BASE_PATH} not found.", file=sys.stderr)
        print("Is the 'argo_radio_servo_module' kernel module loaded?", file=sys.stderr)
        sys.exit(1)

    # --- Initialize state variables ---
    current_rudder_pw = get_initial_pw(SERVO_RUDDER_PATH)
    current_sail_pw = get_initial_pw(SERVO_SAIL_PATH)
    direction_rudder = 1  # 1 for increasing, -1 for decreasing
    direction_sail = 1
    paused = False

    print("Starting Argo Radio Servo PWM Test Script...")
    print("Monitoring input pulse widths and sweeping output servo positions.")
    print("Press Spacebar to PAUSE/RESUME. Press Ctrl+C to STOP.")
    time.sleep(1.5) # Give user time to read the intro message

    # --- Terminal Setup ---
    # Save original terminal settings to restore them on exit
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Set terminal to "cbreak" mode to read keys instantly without requiring Enter
        tty.setcbreak(sys.stdin.fileno())

        # --- Main Loop ---
        while True:
            display_status(current_rudder_pw, current_sail_pw, paused)

            # --- Handle Input for Pause/Resume ---
            key_pressed = get_key_non_blocking()

            if key_pressed == ' ':
                paused = not paused
                display_status(current_rudder_pw, current_sail_pw, paused) # Update status immediately

                if paused:
                    print("\nPAUSED. Press Spacebar to RESUME...")
                    sys.stdout.flush()
                    # When paused, enter a blocking loop that waits *only* for a spacebar.
                    while True:
                        key_to_resume = get_key_blocking()
                        if key_to_resume == ' ':
                            paused = not paused
                            display_status(current_rudder_pw, current_sail_pw, paused)
                            break # Exit this inner blocking loop

            if not paused:
                # --- Update Rudder (PWM2) Pulse Width ---
                current_rudder_pw += direction_rudder * STEP
                if current_rudder_pw > MAX_PW:
                    current_rudder_pw = MAX_PW
                    direction_rudder = -1
                elif current_rudder_pw < MIN_PW:
                    current_rudder_pw = MIN_PW
                    direction_rudder = 1
                write_sysfs_pw(SERVO_RUDDER_PATH, current_rudder_pw)

                # --- Update Sail (PWM4) Pulse Width ---
                current_sail_pw += direction_sail * STEP
                if current_sail_pw > MAX_PW:
                    current_sail_pw = MAX_PW
                    direction_sail = -1
                elif current_sail_pw < MIN_PW:
                    current_sail_pw = MIN_PW
                    direction_sail = 1
                write_sysfs_pw(SERVO_SAIL_PATH, current_sail_pw)

            time.sleep(DELAY_S)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Exiting.")
    finally:
        # --- Restore Terminal ---
        # This block ensures terminal settings are always restored
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal settings restored.")

if __name__ == "__main__":
    main()