#!/bin/bash
# tests pwm

# Configuration
SYS="/sys/kernel/argo_radio_servo"
MIN_PW=900
MAX_PW=2100
STEP=100      # Step size for changing pulse width
DELAY_S=0.5   # Delay between each step in seconds

# Get initial values (or set default if files don't exist yet)
current_rudder_pw=$(cat "$SYS/servo_rudder_pw_us" 2>/dev/null || echo 1500)
current_sail_pw=$(cat "$SYS/servo_sail_pw_us" 2>/dev/null || echo 1500)

direction_rudder=1 # 1 for increasing, -1 for decreasing
direction_sail=1

paused=false # Initial state is not paused

echo "Starting Argo Radio Servo PWM Test Script..."
echo "Monitoring input pulse widths and sweeping output servo positions."
echo "Press Spacebar to PAUSE/RESUME. Press Ctrl+C to STOP."

# Function to display current state
display_status() {
    clear # Clear screen for clean output
    
    echo "--- Radio Control Input Pulse Widths ---"
    cat "$SYS/radio_rudder_pw_us" 2>/dev/null
    cat "$SYS/radio_sail_pw_us" 2>/dev/null
    echo "----------------------------------------"

    echo "--- Servo Motor Output Pulse Widths (Sweeping) ---"
    echo "Rudder (PWM2): $current_rudder_pw us"
    echo "Sail (PWM4):   $current_sail_pw us"
    echo "--------------------------------------------------"
    if $paused; then
        echo "STATUS: PAUSED (Press Spacebar to RESUME)"
    else
        echo "STATUS: RUNNING (Press Spacebar to PAUSE)"
    fi
}

# --- Terminal Setup and Restore Function ---
# Save original terminal settings globally
stty_original=$(stty -g)

# Function to restore terminal settings (called on exit)
restore_terminal() {
    stty "$stty_original" # Restore original terminal settings
    echo -e "\nTerminal settings restored. Exiting script."
    exit 0
}

# Trap Ctrl+C to call restore_terminal function
trap restore_terminal INT

# Set terminal to raw mode for single character input for non-blocking reads.
# This should be applied once.
# 'min 0 time 0' makes `dd` non-blocking with a small delay.
stty -icanon -echo min 0 time 0

# Main loop
while true; do
    display_status # Always display status at the beginning of the loop

    # --- Handle Input for Pause/Resume ---
    # Use 'dd' to read a single character from /dev/tty with a timeout.
    # We apply stty settings directly around dd as a precaution if global settings are volatile.
    key_pressed=$( ( stty -icanon -echo min 0 time 0; dd bs=1 count=1 if=/dev/tty 2>/dev/null ) )

    if [ -n "$key_pressed" ]; then # Check if a character was read
        if [ "$key_pressed" == " " ]; then
            # Correct Bash boolean toggling
            if $paused; then
                paused=false
            else
                paused=true
            fi
            display_status # Update status immediately after toggle
            
            if $paused; then
                echo -e "\nPAUSED. Press Spacebar to RESUME..."
                # When paused, enter a blocking loop that waits *only* for spacebar.
                # This makes the pause solid.
                while true; do
                    # Blocking read using dd.
                    # No timeout, will wait indefinitely for input.
                    key_to_resume=$( ( stty -icanon -echo; dd bs=1 count=1 if=/dev/tty 2>/dev/null ) )
                    if [ "$key_to_resume" == " " ]; then
                        # Correct Bash boolean toggling
                        if $paused; then
                            paused=false
                        else
                            paused=true
                        fi
                        display_status   # Update display
                        break            # Exit this inner blocking loop
                    elif [ "$key_to_resume" == $'\x03' ]; then # Check for Ctrl+C
                        restore_terminal # Ctrl+C during blocking read should restore
                    fi
                done
            fi
        fi
    fi

    if ! $paused; then
        # --- Update Rudder (PWM2) Pulse Width ---
        current_rudder_pw=$((current_rudder_pw + direction_rudder * STEP))
        if (( current_rudder_pw > MAX_PW )); then
            current_rudder_pw=$MAX_PW
            direction_rudder=-1
        elif (( current_rudder_pw < MIN_PW )); then
            current_rudder_pw=$MIN_PW
            direction_rudder=1
        fi
        echo "$current_rudder_pw" | tee "$SYS/servo_rudder_pw_us" > /dev/null

        # --- Update Sail (PWM4) Pulse Width ---
        current_sail_pw=$((current_sail_pw + direction_sail * STEP))
        if (( current_sail_pw > MAX_PW )); then
            current_sail_pw=$MAX_PW
            direction_sail=-1
        elif (( current_sail_pw < MIN_PW )); then
            current_sail_pw=$MIN_PW
            direction_sail=1
        fi
        echo "$current_sail_pw" | tee "$SYS/servo_sail_pw_us" > /dev/null
    fi

    sleep "$DELAY_S" # Control the main loop update rate
done

# This part should ideally be reached on exit, but trap handles most cases.
stty "$stty_original" # Restore original terminal settings (redundant due to trap, but safe)
