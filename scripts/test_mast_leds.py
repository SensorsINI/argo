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
REG_FLAG = 0x40             # FLAG - Status register (OUTx_EN, ENGINE_BUSY, TSD, POR)

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
        self.chip_enabled = True  # Track CHIP_EN state
        self.old_settings = None
        self.toggle_test_running = False  # Track if hardware test toggle loop is running
        self._test_thread = None  # Thread for hardware test
        self.toggle_test_running = False  # Track if hardware test toggle loop is running
        
    def _getch(self):
        """Get a single character from stdin without requiring Enter."""
        try:
            ch = sys.stdin.read(1)
            return ch
        except:
            return None
    
    def _print(self, *args, **kwargs):
        """Print with terminal mode handling - temporarily restore for output."""
        if self.old_settings:
            # Temporarily restore terminal for clean output
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print(*args, **kwargs, flush=True)
            # Restore raw mode for input
            tty.setraw(sys.stdin.fileno())
        else:
            print(*args, **kwargs, flush=True)
    
    def setup_terminal(self):
        """Configure terminal for single-character input."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """Restore terminal to normal mode."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def _write_register(self, register: int, value: int, comment: str = ""):
        """Write a single byte to an LP5814DLR register.
        
        Args:
            register: Register address to write.
            value: Byte value to write (0-255).
            comment: Optional human-readable description of intent.
        """
        if not self.bus:
            raise IOError("I2C bus not initialized")
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(LP5814_I2C_ADDRESS, register, value)
        msg = f"  Write: REG_0x{register:02x} = 0x{value:02x} ({value})"
        if comment:
            msg += f"  # {comment}"
        self._print(msg)
    
    def _read_register(self, register: int):
        """Read a single byte from an LP5814DLR register."""
        if not self.bus:
            return None
        try:
            # Temporarily restore terminal for clean output
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            value = self.bus.read_byte_data(LP5814_I2C_ADDRESS, register)
            # Restore raw mode
            if self.old_settings:
                tty.setraw(sys.stdin.fileno())
            return value
        except Exception as e:
            self._print(f"  Error reading register 0x{register:02x}: {e}")
            return None
    
    def initialize_device(self):
        """Initialize LP5814DLR device following datasheet Section 8.2.3.1."""
        if not self.bus:
            return False
        
        try:
            self._print("Initializing LP5814DLR...")
            
            # Step 1: Wait ~1ms after power up (per datasheet Section 8.2.3.2)
            # Allows device to complete power-on reset sequence
            time.sleep(0.001)
            
            # Step 2: Enable chip (REG_CHIP_EN = 0x00, bit 0 = 1)
            # Sets CHIP_EN bit to enable device operation
            # Device enters NORMAL mode (vs STANDBY when CHIP_EN = 0)
            self._write_register(REG_CHIP_EN, CHIP_EN_BIT, "CHIP_EN=1 (enable chip / normal mode)")
            
            # Step 3: Set MAX_CURRENT = 51mA (REG_DEV_CONFIG0 = 0x01)
            # Sets IOUT_MAX for all channels: 0x00=25.5mA, 0x01=51mA, 0x02=76.5mA, 0x03=102mA
            # This is the maximum current limit - actual current controlled by DC registers
            self._write_register(REG_DEV_CONFIG0, MAX_CURRENT_51MA, "DEV_CONFIG0: IOUT_MAX=51mA (global max current)")
            
            # Step 4: Set maximum current for each output (DC registers)
            # These set the current limit per channel (not the actual current)
            # Red LED: 51mA max (0xFF), Green/Blue/White: 40.8mA max (0xCC)
            # Actual current controlled by PWM duty cycle in production code
            self._write_register(REG_OUT0_DC, OUT0_DC_MAX, "OUT0_DC: current limit (red) 0xFF")
            self._write_register(REG_OUT1_DC, OUT1_DC_MAX, "OUT1_DC: current limit (green) 0xCC")
            self._write_register(REG_OUT2_DC, OUT2_DC_MAX, "OUT2_DC: current limit (blue) 0xCC")
            self._write_register(REG_OUT3_DC, OUT3_DC_MAX, "OUT3_DC: current limit (white) 0xCC")
            
            # Step 5: Enable all outputs (REG_DEV_CONFIG1 = 0x0F)
            # Bit 0 (OUT0_EN): Enable Red LED output
            # Bit 1 (OUT1_EN): Enable Green LED output
            # Bit 2 (OUT2_EN): Enable Blue LED output
            # Bit 3 (OUT3_EN): Enable White LED output
            # 0x0F = 0b00001111 enables all 4 outputs
            self._write_register(REG_DEV_CONFIG1, OUT_ALL_EN, "DEV_CONFIG1: enable OUT0..OUT3 (0x0F)")
            
            # Step 5b: Configure for manual mode (REG_DEV_CONFIG3 = 0x00)
            # Clears all OUTx_AUTO_EN bits to disable animation engine
            # Device uses manual PWM/DC control instead of autonomous animation
            # Required for direct control via DC or PWM registers
            self._write_register(REG_DEV_CONFIG3, DEV_CONFIG3_MANUAL_MODE, "DEV_CONFIG3: manual mode (disable engine)")
            
            # Step 6: Send UPDATE_CMD (REG_UPDATE_CMD = 0x0F, value = 0x01)
            # Applies all configuration changes from Steps 2-5b
            # UPDATE_CMD is required when CHIP_EN = 1 to make config changes take effect
            # Device will not respond to UPDATE_CMD if CHIP_EN = 0
            self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE, "UPDATE_CMD: apply configuration")
            
            # Step 7: Initialize all channels to off using PWM registers (matches production code)
            # DC registers already set to max in Step 4 (current limit)
            # PWM registers control brightness: 0x00 = 0% duty cycle (OFF), 0xFF = 100% duty (ON)
            # Setting all PWM registers to 0x00 ensures all LEDs start in OFF state
            self._write_register(REG_OUT0_MANUAL_PWM, BRIGHTNESS_HW_MIN, "OUT0_PWM: 0% duty (OFF)")
            self._write_register(REG_OUT1_MANUAL_PWM, BRIGHTNESS_HW_MIN, "OUT1_PWM: 0% duty (OFF)")
            self._write_register(REG_OUT2_MANUAL_PWM, BRIGHTNESS_HW_MIN, "OUT2_PWM: 0% duty (OFF)")
            self._write_register(REG_OUT3_MANUAL_PWM, BRIGHTNESS_HW_MIN, "OUT3_PWM: 0% duty (OFF)")
            
            # Send UPDATE_CMD to apply the DC register changes (required when CHIP_EN = 1)
            # Makes the DC register values (0x00 = OFF) take effect
            self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE, "UPDATE_CMD: apply PWM values")
            
            # Verify critical registers
            chip_en = self._read_register(REG_CHIP_EN)
            if chip_en is not None:
                if chip_en & CHIP_EN_BIT:
                    self._print(f"  CHIP_EN verified: 0x{chip_en:02x} (chip enabled)")
                else:
                    self._print(f"  ERROR: CHIP_EN not set! Read 0x{chip_en:02x}, expected bit 0 = 1")
            else:
                self._print("  WARNING: CHIP_EN readback failed")
            
            # Verify DC registers are set to 0
            dc0 = self._read_register(REG_OUT0_DC)
            if dc0 is not None and dc0 == 0:
                self._print("  All DC registers initialized to 0 (LEDs OFF)")
            elif dc0 is not None:
                self._print(f"  WARNING: DC registers may not be 0 (read 0x{dc0:02x})")
            
            self._print("Device initialized successfully")
            
            return True
            
        except IOError as e:
            self._print(f"Failed to initialize device: {e}")
            return False
        except Exception as e:
            self._print(f"Unexpected error: {e}")
            return False
    
    def update_led(self, channel: str, state: bool):
        """Update a single LED channel using PWM mode (matches production code).
        
        Production code approach:
        - DC registers set to max (current limit) during initialization
        - PWM registers control brightness: 0x00 = 0% duty (OFF), 0xFF = 100% duty (ON)
        """
        pwm_channel_map = {
            'r': (REG_OUT0_MANUAL_PWM, REG_OUT0_DC, 'Red'),
            'g': (REG_OUT1_MANUAL_PWM, REG_OUT1_DC, 'Green'),
            'b': (REG_OUT2_MANUAL_PWM, REG_OUT2_DC, 'Blue'),
            'w': (REG_OUT3_MANUAL_PWM, REG_OUT3_DC, 'White')
        }
        
        if channel not in pwm_channel_map:
            return False
        
        pwm_reg, dc_reg, name = pwm_channel_map[channel]
        # PWM controls brightness: 0x00 = 0% duty (OFF), 0xFF = 100% duty (ON)
        value = BRIGHTNESS_HW_MAX if state else BRIGHTNESS_HW_MIN
        self.led_states[channel] = state
        
        try:
            # Set PWM register to control brightness (production code approach)
            self._write_register(pwm_reg, value)
            # Send UPDATE_CMD to make the change take effect (required when CHIP_EN = 1)
            self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE)
            # Verify by reading back both registers
            pwm_readback = self._read_register(pwm_reg)
            dc_readback = self._read_register(dc_reg)
            status = "ON" if state else "OFF"
            if pwm_readback == value:
                pct = int((pwm_readback/255.0)*100)
                dc_str = f"0x{dc_readback:02x}" if dc_readback is not None else "??"
                self._print(f"  {name} LED: {status} (PWM=0x{pwm_readback:02x}={pct}% duty, DC={dc_str}, verified)")
            else:
                pwm_str = f"0x{pwm_readback:02x}" if pwm_readback is not None else "None"
                self._print(f"  {name} LED: {status} (PWM write failed: expected 0x{value:02x}, got {pwm_str})")
            return True
        except Exception as e:
            self._print(f"  Error updating {name} LED: {e}")
            return False
    
    def update_all_leds(self, state: bool):
        """Update all LED channels."""
        self._print(f"\n{'Turning all LEDs ON' if state else 'Turning all LEDs OFF'}...")
        for channel in ['r', 'g', 'b', 'w']:
            self.update_led(channel, state)
    
    def toggle_chip_enable(self):
        """Toggle CHIP_EN (shutdown/disable mode)."""
        self.chip_enabled = not self.chip_enabled
        value = CHIP_EN_BIT if self.chip_enabled else 0x00
        try:
            self._write_register(REG_CHIP_EN, value)
            # Read back to verify
            readback = self._read_register(REG_CHIP_EN)
            status = "ENABLED" if self.chip_enabled else "DISABLED (shutdown)"
            if readback is not None:
                if (readback & CHIP_EN_BIT) == value:
                    self._print(f"  CHIP_EN: {status} (verified: 0x{readback:02x})")
                else:
                    self._print(f"  CHIP_EN: {status} (write may have failed: read 0x{readback:02x})")
            else:
                self._print(f"  CHIP_EN: {status} (readback failed)")
        except Exception as e:
            self._print(f"  Error toggling CHIP_EN: {e}")
    
    def print_status(self):
        """Print current LED status."""
        self._print("\n" + "="*50)
        self._print("Current LED Status:")
        self._print(f"  CHIP_EN: {'ENABLED' if self.chip_enabled else 'DISABLED (shutdown)'}")
        self._print(f"  Red:   {'ON' if self.led_states['r'] else 'OFF'}")
        self._print(f"  Green: {'ON' if self.led_states['g'] else 'OFF'}")
        self._print(f"  Blue:  {'ON' if self.led_states['b'] else 'OFF'}")
        self._print(f"  White: {'ON' if self.led_states['w'] else 'OFF'}")
        self._print("="*50)
    
    def print_help(self):
        """Print help message."""
        self._print("\n" + "="*50)
        self._print("LP5814DLR LED Tester")
        self._print("="*50)
        self._print("Controls:")
        self._print("  r - Toggle Red LED")
        self._print("  g - Toggle Green LED")
        self._print("  b - Toggle Blue LED")
        self._print("  w - Toggle White LED")
        self._print("  1 - Turn all LEDs ON")
        self._print("  0 - Turn all LEDs OFF")
        self._print("  d - Toggle CHIP_EN (shutdown/disable mode)")
        self._print("  v - Dump all registers (verbose diagnostics)")
        self._print("  t - Hardware test (cycle all LEDs ON/OFF rapidly)")
        self._print("  s - Show status")
        self._print("  h - Show this help")
        self._print("  q - Quit")
        self._print("="*50)
    
    def _hardware_test_loop(self):
        """Internal loop for hardware test - runs in separate thread."""
        self.toggle_test_running = True
        self._print("\nHardware Test: Toggling all LEDs at 2Hz (0.5s per state)")
        self._print("Press 't' again to stop...")
        
        try:
            state = False
            cycle = 0
            while self.toggle_test_running:
                # Toggle state
                state = not state
                cycle += 1
                
                # Debug: Print cycle number and verify I2C communication
                status_str = "ON" if state else "OFF"
                self._print(f"\n[Cycle {cycle}] Toggling to {status_str}...")
                
                # Update LEDs and verify register writes
                for channel in ['r', 'g', 'b', 'w']:
                    pwm_reg, dc_reg, name = {
                        'r': (REG_OUT0_MANUAL_PWM, REG_OUT0_DC, 'Red'),
                        'g': (REG_OUT1_MANUAL_PWM, REG_OUT1_DC, 'Green'),
                        'b': (REG_OUT2_MANUAL_PWM, REG_OUT2_DC, 'Blue'),
                        'w': (REG_OUT3_MANUAL_PWM, REG_OUT3_DC, 'White')
                    }[channel]
                    
                    value = BRIGHTNESS_HW_MAX if state else BRIGHTNESS_HW_MIN
                    try:
                        self._write_register(pwm_reg, value)
                        # Verify write immediately
                        readback = self._read_register(pwm_reg)
                        if readback == value:
                            self._print(f"  {name}: OK (wrote 0x{value:02x}, read 0x{readback:02x})")
                        else:
                            self._print(f"  {name}: FAIL (wrote 0x{value:02x}, read 0x{readback:02x})")
                    except Exception as e:
                        self._print(f"  {name}: ERROR - {e}")
                
                # Send UPDATE_CMD after all writes
                try:
                    self._write_register(REG_UPDATE_CMD, UPDATE_CMD_VALUE)
                    self._print("  UPDATE_CMD sent")
                except Exception as e:
                    self._print(f"  UPDATE_CMD ERROR - {e}")
                
                # Sleep 0.5 seconds (2Hz = 2 toggles per second = 0.5s per state)
                # Break sleep into small chunks to check flag frequently
                sleep_interval = 0.05  # Check every 50ms
                total_sleep = 0.5
                slept = 0.0
                while slept < total_sleep and self.toggle_test_running:
                    time.sleep(sleep_interval)
                    slept += sleep_interval
                    
        except Exception as e:
            self._print(f"\nTest error: {e}")
            import traceback
            self._print(traceback.format_exc())
        finally:
            self.toggle_test_running = False
            self.update_all_leds(False)
            self._print("\nHardware test stopped - all LEDs OFF")
    
    def hardware_test(self):
        """Hardware test: Toggle all LEDs ON/OFF at 2Hz continuously.
        
        Press 't' to start/stop continuous toggling at 2Hz (0.5s per state).
        Toggles until 't' is pressed again to stop.
        """
        if self.toggle_test_running:
            # Stop the toggle loop
            self.toggle_test_running = False
            # Wait for thread to finish
            if self._test_thread and self._test_thread.is_alive():
                self._test_thread.join(timeout=1.0)
            return
        
        # Start the toggle loop in a separate thread
        import threading
        if self._test_thread and self._test_thread.is_alive():
            return  # Already running
        
        self._test_thread = threading.Thread(target=self._hardware_test_loop, daemon=True)
        self._test_thread.start()
    
    def dump_registers(self):
        """Dump all critical registers for debugging."""
        self._print("\n" + "="*50)
        self._print("LP5814DLR Register Dump")
        self._print("="*50)
        
        registers = {
            'CHIP_EN': REG_CHIP_EN,
            'DEV_CONFIG0': REG_DEV_CONFIG0,
            'DEV_CONFIG1': REG_DEV_CONFIG1,
            'DEV_CONFIG3': REG_DEV_CONFIG3,
            'OUT0_DC': REG_OUT0_DC,
            'OUT1_DC': REG_OUT1_DC,
            'OUT2_DC': REG_OUT2_DC,
            'OUT3_DC': REG_OUT3_DC,
            'OUT0_PWM': REG_OUT0_MANUAL_PWM,
            'OUT1_PWM': REG_OUT1_MANUAL_PWM,
            'OUT2_PWM': REG_OUT2_MANUAL_PWM,
            'OUT3_PWM': REG_OUT3_MANUAL_PWM,
            'FLAG': REG_FLAG,
        }
        
        for name, reg in registers.items():
            value = self._read_register(reg)
            if value is not None:
                self._print(f"  {name:12s} (0x{reg:02x}): 0x{value:02x} ({value:3d}) {self._format_register_bits(name, value)}")
            else:
                self._print(f"  {name:12s} (0x{reg:02x}): READ FAILED")
        
        self._print("="*50)
    
    def _format_register_bits(self, name: str, value: int) -> str:
        """Format register bits for human readability."""
        if name == 'CHIP_EN':
            return f"CHIP_EN={'ON' if value & CHIP_EN_BIT else 'OFF'}"
        elif name == 'DEV_CONFIG1':
            out0 = 'ON' if value & OUT0_EN_BIT else 'OFF'
            out1 = 'ON' if value & OUT1_EN_BIT else 'OFF'
            out2 = 'ON' if value & OUT2_EN_BIT else 'OFF'
            out3 = 'ON' if value & OUT3_EN_BIT else 'OFF'
            return f"OUT0={out0} OUT1={out1} OUT2={out2} OUT3={out3}"
        elif name == 'DEV_CONFIG0':
            mc = (value & 0x03)
            return f"MAX_CURRENT={mc} ({'25.5mA' if mc==0 else '51mA' if mc==1 else '76.5mA' if mc==2 else '102mA'})"
        elif name.startswith('OUT') and name.endswith('_DC'):
            pct = int((value / 255.0) * 100)
            return f"{pct}% of IOUT_MAX"
        elif name.startswith('OUT') and name.endswith('_PWM'):
            pct = int((value / 255.0) * 100)
            return f"{pct}% duty"
        elif name == 'FLAG':
            # FLAG register bits: bit0=OUT0_EN, bit1=OUT1_EN, bit2=OUT2_EN, bit3=OUT3_EN
            # bit4=ENGINE_BUSY, bit5=TSD (Thermal Shutdown), bit6=POR (Power-On Reset)
            out0 = 'ON' if value & 0x01 else 'OFF'
            out1 = 'ON' if value & 0x02 else 'OFF'
            out2 = 'ON' if value & 0x04 else 'OFF'
            out3 = 'ON' if value & 0x08 else 'OFF'
            engine_busy = 'BUSY' if value & 0x10 else 'IDLE'
            tsd = 'SHUTDOWN' if value & 0x20 else 'OK'
            por = 'RESET' if value & 0x40 else 'NORMAL'
            return f"OUT0={out0} OUT1={out1} OUT2={out2} OUT3={out3} ENGINE={engine_busy} TSD={tsd} POR={por}"
        return ""
    
    def run(self):
        """Main interactive loop."""
        try:
            # Initialize I2C bus
            print(f"Opening I2C bus {I2C_BUS}...")
            print(f"Target LP5814DLR I2C address: 0x{LP5814_I2C_ADDRESS:02x} ({LP5814_I2C_ADDRESS})")
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
            self._print("\nPress keys to control LEDs (q to quit)...")
            
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
                elif ch == 'd':
                    self.toggle_chip_enable()
                elif ch == 'v':
                    self.dump_registers()
                elif ch == 't':
                    self.hardware_test()
                elif ch == 's':
                    self.print_status()
                elif ch == 'h':
                    self.print_help()
                elif ch == '\x04':  # Ctrl+D
                    break
                else:
                    if ch.isprintable():
                        self._print(f"Unknown key: '{ch}' (press 'h' for help)")
        
        except KeyboardInterrupt:
            self._print("\n\nInterrupted by user")
        except Exception as e:
            self._print(f"\n\nError: {e}")
            import traceback
            # Restore terminal before printing traceback
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            traceback.print_exc()
        finally:
            # Turn off all LEDs
            self._print("\nTurning off all LEDs...")
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
