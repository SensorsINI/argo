#!/usr/bin/env python3
#
# power-test-pins.py
#
# Pin Testing Tool for Orange Pi Zero 2W Power Control System
# ==========================================================
#
# DESCRIPTION:
#   Interactive tool to test and control GPIO pins from power_control.py
#   Allows setting any pin to different modes for hardware debugging
#
# USAGE:
#   ./power-test-pins.py
#
# REQUIREMENTS:
#   - User must be member of 'gpio' group (for GPIO access)
#   - python3-gpiod library installed
#
# AUTHOR: Generated for Orange Pi Zero 2W Power Control System
# VERSION: 1.0
# DATE: September 2024

import gpiod
import time
import sys
import os
import grp
import threading
import subprocess
from datetime import datetime

class PinTester:
    def __init__(self):
        self.GPIO_CHIP = '/dev/gpiochip0'
        self.chip = None
        self.pins = {}
        self.running = True
        
        # Pin definitions from power_control.py
        self.PIN_DEFINITIONS = {
            'POWER_RELAY': {
                'name': 'PI3 (Pin 40) - !POW',
                'line': 259,
                'description': 'Power relay control (open drain output)',
                'modes': ['output', 'output_open_drain', 'input']
            },
            'POWER_BUTTON': {
                'name': 'PI9 (Pin 28) - !POW_BUT',
                'line': 265,
                'description': 'Power button input (external pullup required)',
                'modes': ['input', 'input_pullup', 'input_pulldown']
            },
            'GREEN_LED': {
                'name': 'PH4 (Pin 18) - Green LED',
                'line': 228,
                'description': 'Green LED in power button (system running indicator)',
                'modes': ['output', 'input']
            },
            'BLUE_LED': {
                'name': 'PI1 (Pin 12) - Blue LED',
                'line': 257,
                'description': 'Blue LED in power button (status/warning indicator)',
                'modes': ['output', 'input']
            }
        }
        
        self.init_gpio()
    
    def clear_screen(self):
        """Clear the terminal screen"""
        try:
            # Try to clear screen using system command
            subprocess.run(['clear'], check=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            # Fallback to ANSI escape sequence
            print('\033[2J\033[H', end='')
    
    def init_gpio(self):
        """Initialize GPIO chip"""
        try:
            print(f"Opening GPIO chip: {self.GPIO_CHIP}")
            self.chip = gpiod.Chip(self.GPIO_CHIP)
            print("✅ GPIO chip opened successfully")
        except Exception as e:
            print(f"❌ Failed to open GPIO chip: {e}")
            sys.exit(1)
    
    def show_menu(self):
        """Display the main menu"""
        print("\n" + "="*60)
        print("🔧 PIN TESTING TOOL - Orange Pi Zero 2W Power Control")
        print("="*60)
        print("\nAvailable pins:")
        for i, (pin_id, pin_info) in enumerate(self.PIN_DEFINITIONS.items(), 1):
            status = "🔴 Not configured" if pin_id not in self.pins else "🟢 Active"
            print(f"  {i}. {pin_info['name']} - {status}")
            print(f"     {pin_info['description']}")
            print(f"     Available modes: {', '.join(pin_info['modes'])}")
            print()
        
        print("Commands:")
        print("  c <pin_number> <mode>     - Configure pin (e.g., c 1 output)")
        print("  s <pin_number> <value>    - Set pin value (0/1 for output, read for input)")
        print("  r <pin_number>            - Read pin value")
        print("  l <pin_number>            - List pin status")
        print("  t <pin_number> <seconds>  - Toggle pin for specified duration")
        print("  b <pin_number> <rate>     - Blink pin at specified rate (Hz)")
        print("  d <pin_number>            - Disable/release pin")
        print("  a                         - Show all pin status")
        print("  h                         - Show this help")
        print("  q                         - Quit")
        print("="*60)
    
    def get_pin_id(self, pin_number):
        """Convert pin number to pin ID"""
        pin_list = list(self.PIN_DEFINITIONS.keys())
        if 1 <= pin_number <= len(pin_list):
            return pin_list[pin_number - 1]
        return None
    
    def configure_pin(self, pin_id, mode):
        """Configure a pin with specified mode"""
        if pin_id not in self.PIN_DEFINITIONS:
            print(f"❌ Unknown pin: {pin_id}")
            return False
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        if mode not in pin_info['modes']:
            print(f"❌ Invalid mode '{mode}' for pin {pin_info['name']}")
            print(f"   Available modes: {', '.join(pin_info['modes'])}")
            print(f"   Pin description: {pin_info['description']}")
            return False
        
        try:
            # Release existing pin if configured
            if pin_id in self.pins:
                self.pins[pin_id].release()
                del self.pins[pin_id]
            
            # Configure new settings
            settings = gpiod.LineSettings()
            
            if mode == 'output':
                settings.direction = settings.direction.OUTPUT
                settings.output_value = settings.output_value.INACTIVE
            elif mode == 'output_open_drain':
                settings.direction = settings.direction.OUTPUT
                settings.output_value = settings.output_value.INACTIVE
                settings.drive = settings.drive.OPEN_DRAIN
            elif mode == 'input':
                settings.direction = settings.direction.INPUT
                settings.bias = settings.bias.DISABLED
            elif mode == 'input_pullup':
                settings.direction = settings.direction.INPUT
                settings.bias = settings.bias.PULL_UP
            elif mode == 'input_pulldown':
                settings.direction = settings.direction.INPUT
                settings.bias = settings.bias.PULL_DOWN
            
            # Request the line
            line = self.chip.request_lines(
                consumer="power-test-pins.py",
                config={pin_info['line']: settings}
            )
            
            self.pins[pin_id] = line
            print(f"✅ Pin {pin_info['name']} configured as {mode}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to configure pin {pin_id}: {e}")
            return False
    
    def set_pin_value(self, pin_id, value):
        """Set pin value (for output pins)"""
        if pin_id not in self.pins:
            print(f"❌ Pin {pin_id} not configured")
            return False
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        try:
            settings = gpiod.LineSettings()
            if value == 0:
                pin_value = settings.output_value.INACTIVE
            elif value == 1:
                pin_value = settings.output_value.ACTIVE
            else:
                print(f"❌ Invalid value: {value} (use 0 or 1)")
                return False
            
            self.pins[pin_id].set_values({pin_info['line']: pin_value})
            print(f"✅ Pin {pin_info['name']} set to {value}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to set pin {pin_id}: {e}")
            return False
    
    def read_pin_value(self, pin_id):
        """Read pin value"""
        if pin_id not in self.pins:
            print(f"❌ Pin {pin_id} not configured")
            return None
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        try:
            values = self.pins[pin_id].get_values()
            value = values[0].value
            print(f"📖 Pin {pin_info['name']} value: {value}")
            return value
        except Exception as e:
            print(f"❌ Failed to read pin {pin_id}: {e}")
            return None
    
    def toggle_pin(self, pin_id, duration):
        """Toggle pin for specified duration"""
        if pin_id not in self.pins:
            print(f"❌ Pin {pin_id} not configured")
            return False
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        try:
            print(f"🔄 Toggling pin {pin_info['name']} for {duration} seconds...")
            start_time = time.time()
            state = False
            
            while time.time() - start_time < duration and self.running:
                self.set_pin_value(pin_id, 1 if state else 0)
                state = not state
                time.sleep(0.1)  # 100ms toggle rate
            
            # Return to low state
            self.set_pin_value(pin_id, 0)
            print(f"✅ Pin {pin_info['name']} toggle complete")
            return True
            
        except Exception as e:
            print(f"❌ Failed to toggle pin {pin_id}: {e}")
            return False
    
    def blink_pin(self, pin_id, rate):
        """Blink pin at specified rate"""
        if pin_id not in self.pins:
            print(f"❌ Pin {pin_id} not configured")
            return False
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        def blink_thread():
            try:
                print(f"💫 Blinking pin {pin_info['name']} at {rate} Hz (Ctrl+C to stop)...")
                period = 1.0 / (2 * rate)  # Half period for on/off
                state = False
                
                while self.running:
                    self.set_pin_value(pin_id, 1 if state else 0)
                    state = not state
                    time.sleep(period)
                    
            except Exception as e:
                print(f"❌ Blink error: {e}")
        
        threading.Thread(target=blink_thread, daemon=True).start()
        return True
    
    def disable_pin(self, pin_id):
        """Disable/release pin"""
        if pin_id not in self.pins:
            print(f"❌ Pin {pin_id} not configured")
            return False
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        try:
            self.pins[pin_id].release()
            del self.pins[pin_id]
            print(f"✅ Pin {pin_info['name']} disabled/released")
            return True
        except Exception as e:
            print(f"❌ Failed to disable pin {pin_id}: {e}")
            return False
    
    def show_pin_status(self, pin_id):
        """Show status of a specific pin"""
        if pin_id not in self.PIN_DEFINITIONS:
            print(f"❌ Unknown pin: {pin_id}")
            return
        
        pin_info = self.PIN_DEFINITIONS[pin_id]
        
        if pin_id in self.pins:
            try:
                # Try to read current value
                values = self.pins[pin_id].get_values()
                current_value = values[0].value
                status = f"🟢 Active (value: {current_value})"
            except:
                status = "🟡 Active (read error)"
        else:
            status = "🔴 Not configured"
        
        print(f"\n📌 Pin Status: {pin_info['name']}")
        print(f"   Description: {pin_info['description']}")
        print(f"   Status: {status}")
        print(f"   Available modes: {', '.join(pin_info['modes'])}")
    
    def show_all_status(self):
        """Show status of all pins"""
        print("\n📊 ALL PIN STATUS")
        print("="*50)
        
        for pin_id, pin_info in self.PIN_DEFINITIONS.items():
            if pin_id in self.pins:
                try:
                    values = self.pins[pin_id].get_values()
                    current_value = values[0].value
                    status = f"🟢 Active (value: {current_value})"
                except:
                    status = "🟡 Active (read error)"
            else:
                status = "🔴 Not configured"
            
            print(f"{pin_info['name']:<25} {status}")
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🧹 Cleaning up...")
        for pin_id in list(self.pins.keys()):
            self.disable_pin(pin_id)
        
        if self.chip:
            self.chip.close()
        
        print("✅ Cleanup complete")
    
    def run(self):
        """Main interactive loop"""
        print("🚀 Pin Testing Tool Started")
        print("   Press Ctrl+C to quit at any time")
        
        try:
            while self.running:
                self.clear_screen()
                self.show_menu()
                
                try:
                    command = input("\nEnter command: ").strip().lower()
                    
                    if not command:
                        continue
                    
                    parts = command.split()
                    
                    if parts[0] == 'q':
                        break
                    elif parts[0] == 'h':
                        continue  # Menu will be shown again
                    elif parts[0] == 'a':
                        self.show_all_status()
                        input("\nPress Enter to continue...")
                    elif parts[0] == 'c':
                        if len(parts) < 3:
                            print("❌ Error: Configure command requires pin number and mode")
                            print("   Usage: c <pin_number> <mode>")
                            print("   Example: c 1 output")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            mode = parts[2]
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.configure_pin(pin_id, mode)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError:
                            print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 's':
                        if len(parts) < 3:
                            print("❌ Error: Set command requires pin number and value")
                            print("   Usage: s <pin_number> <value>")
                            print("   Example: s 1 0")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            value = int(parts[2])
                            if value not in [0, 1]:
                                print(f"❌ Error: Value must be 0 or 1, got '{value}'")
                                input("\nPress Enter to continue...")
                                continue
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.set_pin_value(pin_id, value)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError as e:
                            if "invalid literal for int()" in str(e):
                                print(f"❌ Error: Pin number and value must be integers")
                                print(f"   Got pin='{parts[1]}', value='{parts[2]}'")
                            else:
                                print(f"❌ Error: {e}")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 'r':
                        if len(parts) < 2:
                            print("❌ Error: Read command requires pin number")
                            print("   Usage: r <pin_number>")
                            print("   Example: r 1")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.read_pin_value(pin_id)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError:
                            print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 'l':
                        if len(parts) < 2:
                            print("❌ Error: List command requires pin number")
                            print("   Usage: l <pin_number>")
                            print("   Example: l 1")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.show_pin_status(pin_id)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError:
                            print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 't':
                        if len(parts) < 3:
                            print("❌ Error: Toggle command requires pin number and duration")
                            print("   Usage: t <pin_number> <duration_seconds>")
                            print("   Example: t 1 2.5")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            duration = float(parts[2])
                            if duration <= 0:
                                print(f"❌ Error: Duration must be positive, got '{duration}'")
                                input("\nPress Enter to continue...")
                                continue
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.toggle_pin(pin_id, duration)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError as e:
                            if "invalid literal for int()" in str(e):
                                print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            elif "could not convert string to float" in str(e):
                                print(f"❌ Error: Duration must be a number, got '{parts[2]}'")
                            else:
                                print(f"❌ Error: {e}")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 'b':
                        if len(parts) < 3:
                            print("❌ Error: Blink command requires pin number and rate")
                            print("   Usage: b <pin_number> <rate_hz>")
                            print("   Example: b 1 2.0")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            rate = float(parts[2])
                            if rate <= 0:
                                print(f"❌ Error: Rate must be positive, got '{rate}'")
                                input("\nPress Enter to continue...")
                                continue
                            if rate > 100:
                                print(f"⚠️  Warning: High blink rate ({rate} Hz) may cause issues")
                                print("   Consider using a rate between 0.1 and 10 Hz")
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.blink_pin(pin_id, rate)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError as e:
                            if "invalid literal for int()" in str(e):
                                print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            elif "could not convert string to float" in str(e):
                                print(f"❌ Error: Rate must be a number, got '{parts[2]}'")
                            else:
                                print(f"❌ Error: {e}")
                            input("\nPress Enter to continue...")
                    elif parts[0] == 'd':
                        if len(parts) < 2:
                            print("❌ Error: Disable command requires pin number")
                            print("   Usage: d <pin_number>")
                            print("   Example: d 1")
                            input("\nPress Enter to continue...")
                            continue
                        try:
                            pin_num = int(parts[1])
                            pin_id = self.get_pin_id(pin_num)
                            if pin_id:
                                self.disable_pin(pin_id)
                                input("\nPress Enter to continue...")
                            else:
                                print(f"❌ Error: Invalid pin number {pin_num}")
                                print(f"   Valid pin numbers: 1-{len(self.PIN_DEFINITIONS)}")
                                input("\nPress Enter to continue...")
                        except ValueError:
                            print(f"❌ Error: Pin number must be an integer, got '{parts[1]}'")
                            input("\nPress Enter to continue...")
                    else:
                        print(f"❌ Error: Unknown command '{parts[0]}'")
                        print("   Type 'h' for help or use one of these commands:")
                        print("   c, s, r, l, t, b, d, a, h, q")
                        input("\nPress Enter to continue...")
                
                except ValueError as e:
                    print(f"❌ Error: Invalid input format - {e}")
                    input("\nPress Enter to continue...")
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"❌ Unexpected error: {e}")
                    print("   Please try again or type 'h' for help")
                    input("\nPress Enter to continue...")
        
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

def check_gpio_access():
    """Check if user has GPIO access"""
    try:
        gpio_group = grp.getgrnam('gpio')
        if gpio_group.gr_gid not in os.getgroups():
            print("❌ Error: GPIO access required")
            print("   Your user must be a member of the 'gpio' group")
            print("   Run: sudo usermod -a -G gpio $USER")
            print("   Then log out and log back in, or run: newgrp gpio")
            return False
    except KeyError:
        print("❌ Error: GPIO group not found")
        print("   Please ensure the 'gpio' group exists and you are a member")
        return False
    
    return True

def main():
    """Main entry point"""
    print("🔧 Orange Pi Zero 2W Power Control - Pin Testing Tool")
    print("=" * 60)
    
    # Check GPIO access
    if not check_gpio_access():
        sys.exit(1)
    
    # Create and run pin tester
    tester = PinTester()
    tester.run()

if __name__ == "__main__":
    main()
