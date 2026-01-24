#!/usr/bin/env python3
# Standalone script to test LP5814DLR RGBW LED controller
# Interactive key-based control: r/g/b/w to toggle, 1=all on, 0=all off, q=quit

import sys
import time
import smbus
import termios
import tty

# LP5814DLR Configuration
I2C_BUS = 0
LP5814_I2C_ADDRESS = 0x2c

# Register addresses (from datasheet Table 7-7)
REG_CHIP_EN = 0x00
REG_DEV_CONFIG0 = 0x01
REG_DEV_CONFIG1 = 0x02
REG_DEV_CONFIG2 = 0x03
REG_DEV_CONFIG3 = 0x04
REG_UPDATE_CMD = 0x0F
REG_OUT0_DC = 0x14
REG_OUT1_DC = 0x15
REG_OUT2_DC = 0x16
REG_OUT3_DC = 0x17
REG_OUT0_MANUAL_PWM = 0x18  # Red
REG_OUT1_MANUAL_PWM = 0x19  # Green
REG_OUT2_MANUAL_PWM = 0x1A  # Blue
REG_OUT3_MANUAL_PWM = 0x1B  # White

# Bit definitions
CHIP_EN_BIT = 0x01
MAX_CURRENT_51MA = 0x01
OUT0_EN_BIT = 0x01
OUT1_EN_BIT = 0x02
OUT2_EN_BIT = 0x04
OUT3_EN_BIT = 0x08
OUT_ALL_EN = OUT0_EN_BIT | OUT1_EN_BIT | OUT2_EN_BIT | OUT3_EN_BIT
DEV_CONFIG3_MANUAL_MODE = 0x00
UPDATE_CMD_VALUE = 0x01

# Brightness values
BRIGHTNESS_HW_MIN = 0x00
BRIGHTNESS_HW_MAX = 0xFF

# Maximum current settings
OUT0_DC_MAX = 0xFF  # Red: 51mA
OUT1_DC_MAX = 0xCC  # Green: 40.8mA
OUT2_DC_MAX = 0xCC  # Blue: 40.8mA
OUT3_DC_MAX = 0xCC  # White: 40.8mA


class LEDTester:
    def __init__(self):
        self.bus = None
        self.led_states = {
            'r': False,  # Red
            'g': False,  # Green
            'b': False,  # Blue
            'w': False   # White
        }
        self.old_settings = None
        
    def _getch(self):
        """Get a single character from stdin without requiring Enter."""
        try:
            ch = sys.stdin.read(1)
            return ch
        except:
            return None
    
    def setup_terminal(self):
        """Configure terminal for single-character input."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """Restore terminal to normal mode."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def _write_register(self, register: int, value: int):
        """Write a single byte to an LP5814DLR register."""
        if not self.bus:
            raise IOError("I2C bus not initialized")
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(LP5814_I2C_ADDRESS, register, value)
        print(f"  Write: REG_0x{register:02x} = 0x{value:02x} ({value})")
    
    def _read_register(self, register: int):
        """Read a single byte from an LP5814DLR register."""
        if not self.bus:
            return None
        try:
            value = self.bus.read_byte_data(LP5814_I2C_ADDRESS, register)
            return value
        except Exception as e:
            print(f"  Error reading register 0x{register:02x}: {e}")
            return None
    
    def initialize_device(self):
        """Initialize LP5814DLR device following datasheet Section 8.2.3.1."""
        if not self.bus:
            return False
        
        try:
            print("Initializing LP5814DLR...")
            
            # Step 1: Wait ~1ms after power up
            time.sleep(0.001)
            
            # Step 2: Enable chip
            self._write_register(REG_CHIP_EN, CHIP_EN_BIT)
            
            # Step 3: Set MAX_CURRENT = 51mA
            self._write_register(REG_DEV_CONFIG0, MAX_CURRENT_51MA)
            
            # Step 4: Set maximum current for each output
            self._write_register(REG_OUT0_DC, OUT0_DC_MAX)  # Red: 51mA
            self._write_register(REG_OUT1_DC, OUT1_DC_MAX)  # Green: 40.8mA
            self._write_register(REG_OUT2_DC, OUT2_DC_MAX)  # Blue: 40.8mA
            self._write_register(REG_OUT3_DC, OUT3_DC_MAX)  # White: 40.8mA
            
            # Step 5: Enable all outputs
            self._write_register(REG_DEV_CONFIG1, OUT_ALL_EN)
            
            # Step 5b: Configure for manual PWM mode
            self._write_register(REG_DEV_CONFIG3, DEV_CONFIG3_MANUAL_MODE)
            
            # Step 6: Send UPDATE_CMD
            self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE)
            
            # Step 7: Initialize all channels to off
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN)
            
            print("Device initialized successfully")
            return True
            
        except IOError as e:
            print(f"Failed to initialize device: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def update_led(self, channel: str, state: bool):
        """Update a single LED channel."""
        channel_map = {
            'r': (REG_OUT0_MANUAL_PWM, 'Red'),
            'g': (REG_OUT1_MANUAL_PWM, 'Green'),
            'b': (REG_OUT2_MANUAL_PWM, 'Blue'),
            'w': (REG_OUT3_MANUAL_PWM, 'White')
        }
        
        if channel not in channel_map:
            return False
        
        register, name = channel_map[channel]
        value = BRIGHTNESS_HW_MAX if state else BRIGHTNESS_HW_MIN
        self.led_states[channel] = state
        
        try:
            self._write_register(register, value)
            status = "ON" if state else "OFF"
            print(f"  {name} LED: {status}")
            return True
        except Exception as e:
            print(f"  Error updating {name} LED: {e}")
            return False
    
    def update_all_leds(self, state: bool):
        """Update all LED channels."""
        print(f"\n{'Turning all LEDs ON' if state else 'Turning all LEDs OFF'}...")
        for channel in ['r', 'g', 'b', 'w']:
            self.update_led(channel, state)
    
    def print_status(self):
        """Print current LED status."""
        print("\n" + "="*50)
        print("Current LED Status:")
        print(f"  Red:   {'ON' if self.led_states['r'] else 'OFF'}")
        print(f"  Green: {'ON' if self.led_states['g'] else 'OFF'}")
        print(f"  Blue:  {'ON' if self.led_states['b'] else 'OFF'}")
        print(f"  White: {'ON' if self.led_states['w'] else 'OFF'}")
        print("="*50)
    
    def print_help(self):
        """Print help message."""
        print("\n" + "="*50)
        print("LP5814DLR LED Tester")
        print("="*50)
        print("Controls:")
        print("  r - Toggle Red LED")
        print("  g - Toggle Green LED")
        print("  b - Toggle Blue LED")
        print("  w - Toggle White LED")
        print("  1 - Turn all LEDs ON")
        print("  0 - Turn all LEDs OFF")
        print("  s - Show status")
        print("  h - Show this help")
        print("  q - Quit")
        print("="*50)
    
    def run(self):
        """Main interactive loop."""
        try:
            # Initialize I2C bus
            print(f"Opening I2C bus {I2C_BUS}...")
            self.bus = smbus.SMBus(I2C_BUS)
            print(f"I2C bus opened successfully")
            
            # Initialize device
            if not self.initialize_device():
                print("ERROR: Failed to initialize device")
                return 1
            
            # Setup terminal for single-character input
            self.setup_terminal()
            
            # Print initial help
            self.print_help()
            self.print_status()
            print("\nPress keys to control LEDs (q to quit)...")
            
            # Main loop
            while True:
                ch = self._getch()
                
                if ch is None:
                    continue
                
                if ch == 'q' or ch == '\x03':  # q or Ctrl+C
                    break
                elif ch == 'r':
                    self.led_states['r'] = not self.led_states['r']
                    self.update_led('r', self.led_states['r'])
                elif ch == 'g':
                    self.led_states['g'] = not self.led_states['g']
                    self.update_led('g', self.led_states['g'])
                elif ch == 'b':
                    self.led_states['b'] = not self.led_states['b']
                    self.update_led('b', self.led_states['b'])
                elif ch == 'w':
                    self.led_states['w'] = not self.led_states['w']
                    self.update_led('w', self.led_states['w'])
                elif ch == '1':
                    self.update_all_leds(True)
                elif ch == '0':
                    self.update_all_leds(False)
                elif ch == 's':
                    self.print_status()
                elif ch == 'h':
                    self.print_help()
                elif ch == '\x04':  # Ctrl+D
                    break
                else:
                    if ch.isprintable():
                        print(f"Unknown key: '{ch}' (press 'h' for help)")
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        except Exception as e:
            print(f"\n\nError: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Turn off all LEDs
            print("\nTurning off all LEDs...")
            try:
                self.update_all_leds(False)
            except:
                pass
            
            # Restore terminal
            self.restore_terminal()
            
            print("\nExiting...")
        
        return 0


def main():
    """Main entry point."""
    tester = LEDTester()
    return tester.run()


if __name__ == '__main__':
    sys.exit(main())
