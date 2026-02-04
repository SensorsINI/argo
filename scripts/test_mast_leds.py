#!/usr/bin/env python3
# Standalone script to test PCA9632 RGBW LED controller (mast LEDs)
# Interactive key-based control: r/g/b/w to toggle, 1=all on, 0=all off, q=quit
# Uses individual I2C address 0x62 (not All Call 0x70).

import sys
import time
import smbus
import termios
import tty

# PCA9632 Configuration (NXP 4-bit I2C LED driver)
I2C_BUS = 0
PCA9632_I2C_ADDRESS = 0x62  # 7-bit individual address (0x70 is LED All Call)

# Register addresses (control byte D[3:0] - first byte after I2C addr)
REG_MODE1 = 0x00    # Mode 1: bit 4 = SLEEP (0 = normal)
REG_MODE2 = 0x01    # Mode 2
REG_PWM0 = 0x02     # LED0 Red
REG_PWM1 = 0x03     # LED1 Green
REG_PWM2 = 0x04     # LED2 Blue
REG_PWM3 = 0x05     # LED3 White
REG_GRPPWM = 0x06
REG_GRPFREQ = 0x07
REG_LEDOUT = 0x08   # LDRx: 00=off, 01=on, 10=individual PWM, 11=individual+group

MODE1_SLEEP_BIT = 0x10   # Bit 4: 1 = oscillator off
MODE1_NORMAL = 0x01      # SLEEP=0
LEDOUT_ALL_INDIVIDUAL_PWM = 0xAA  # All four channels in individual PWM (LDRx=10)

BRIGHTNESS_HW_MIN = 0x00
BRIGHTNESS_HW_MAX = 0xFF


class LEDTester:
    def __init__(self):
        self.bus = None
        self.led_states = {
            'r': False,
            'g': False,
            'b': False,
            'w': False
        }
        self.sleep_mode = False  # PCA9632 SLEEP (MODE1 bit 4)
        self.old_settings = None
        self.toggle_test_running = False
        self._test_thread = None

    def _getch(self):
        try:
            return sys.stdin.read(1)
        except Exception:
            return None

    def _print(self, *args, **kwargs):
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print(*args, **kwargs, flush=True)
            tty.setraw(sys.stdin.fileno())
        else:
            print(*args, **kwargs, flush=True)

    def setup_terminal(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def restore_terminal(self):
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def _write_register(self, register: int, value: int, comment: str = ""):
        if not self.bus:
            raise IOError("I2C bus not initialized")
        value = max(0, min(255, int(value)))
        self.bus.write_byte_data(PCA9632_I2C_ADDRESS, register, value)
        msg = f"  Write: REG_0x{register:02x} = 0x{value:02x} ({value})"
        if comment:
            msg += f"  # {comment}"
        self._print(msg)

    def _read_register(self, register: int):
        if not self.bus:
            return None
        try:
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            value = self.bus.read_byte_data(PCA9632_I2C_ADDRESS, register)
            if self.old_settings:
                tty.setraw(sys.stdin.fileno())
            return value
        except Exception as e:
            self._print(f"  Error reading register 0x{register:02x}: {e}")
            return None

    def initialize_device(self):
        """Initialize PCA9632: wake from sleep, LEDOUT=individual PWM, PWMx=0."""
        if not self.bus:
            return False
        try:
            self._print("Initializing PCA9632...")
            time.sleep(0.001)
            self._write_register(REG_MODE1, MODE1_NORMAL, "SLEEP=0 (oscillator on)")
            time.sleep(0.001)
            self._write_register(REG_LEDOUT, LEDOUT_ALL_INDIVIDUAL_PWM, "LEDOUT=0xAA (individual PWM)")
            self._write_register(REG_PWM0, BRIGHTNESS_HW_MIN, "PWM0 Red OFF")
            self._write_register(REG_PWM1, BRIGHTNESS_HW_MIN, "PWM1 Green OFF")
            self._write_register(REG_PWM2, BRIGHTNESS_HW_MIN, "PWM2 Blue OFF")
            self._write_register(REG_PWM3, BRIGHTNESS_HW_MIN, "PWM3 White OFF")
            mode1 = self._read_register(REG_MODE1)
            if mode1 is not None:
                self._print(f"  MODE1 verified: 0x{mode1:02x} (SLEEP={'1' if mode1 & MODE1_SLEEP_BIT else '0'})")
            self._print("Device initialized successfully")
            return True
        except IOError as e:
            self._print(f"Failed to initialize device: {e}")
            return False
        except Exception as e:
            self._print(f"Unexpected error: {e}")
            return False

    def update_led(self, channel: str, state: bool):
        reg_map = {'r': (REG_PWM0, 'Red'), 'g': (REG_PWM1, 'Green'),
                   'b': (REG_PWM2, 'Blue'), 'w': (REG_PWM3, 'White')}
        if channel not in reg_map:
            return False
        reg, name = reg_map[channel]
        value = BRIGHTNESS_HW_MAX if state else BRIGHTNESS_HW_MIN
        self.led_states[channel] = state
        try:
            self._write_register(reg, value)
            readback = self._read_register(reg)
            status = "ON" if state else "OFF"
            if readback == value:
                pct = int((readback / 255.0) * 100)
                self._print(f"  {name} LED: {status} (PWM=0x{readback:02x}={pct}%, verified)")
            else:
                self._print(f"  {name} LED: {status} (write 0x{value:02x}, read 0x{readback:02x})")
            return True
        except Exception as e:
            self._print(f"  Error updating {name} LED: {e}")
            return False

    def update_all_leds(self, state: bool):
        self._print(f"\n{'Turning all LEDs ON' if state else 'Turning all LEDs OFF'}...")
        for ch in ['r', 'g', 'b', 'w']:
            self.update_led(ch, state)

    def toggle_sleep(self):
        """Toggle PCA9632 SLEEP (MODE1 bit 4). When SLEEP=1, oscillator off, LEDs not driven."""
        self.sleep_mode = not self.sleep_mode
        value = (MODE1_NORMAL | MODE1_SLEEP_BIT) if self.sleep_mode else MODE1_NORMAL
        try:
            self._write_register(REG_MODE1, value)
            readback = self._read_register(REG_MODE1)
            status = "SLEEP (osc off)" if self.sleep_mode else "NORMAL"
            if readback is not None:
                self._print(f"  MODE1: {status} (0x{readback:02x})")
            else:
                self._print(f"  MODE1: {status} (readback failed)")
        except Exception as e:
            self._print(f"  Error toggling SLEEP: {e}")

    def print_status(self):
        self._print("\n" + "=" * 50)
        self._print("Current LED Status:")
        self._print(f"  MODE1: {'SLEEP (osc off)' if self.sleep_mode else 'NORMAL'}")
        self._print(f"  Red:   {'ON' if self.led_states['r'] else 'OFF'}")
        self._print(f"  Green: {'ON' if self.led_states['g'] else 'OFF'}")
        self._print(f"  Blue:  {'ON' if self.led_states['b'] else 'OFF'}")
        self._print(f"  White: {'ON' if self.led_states['w'] else 'OFF'}")
        self._print("=" * 50)

    def print_help(self):
        self._print("\n" + "=" * 50)
        self._print("PCA9632 LED Tester (mast LEDs, addr 0x62)")
        self._print("=" * 50)
        self._print("  r - Toggle Red LED")
        self._print("  g - Toggle Green LED")
        self._print("  b - Toggle Blue LED")
        self._print("  w - Toggle White LED")
        self._print("  1 - Turn all LEDs ON")
        self._print("  0 - Turn all LEDs OFF")
        self._print("  d - Toggle SLEEP (MODE1: oscillator on/off)")
        self._print("  v - Dump registers (MODE1, LEDOUT, PWM0-3)")
        self._print("  t - Hardware test (cycle all LEDs ON/OFF at 2 Hz)")
        self._print("  s - Show status")
        self._print("  h - Show this help")
        self._print("  q - Quit")
        self._print("=" * 50)

    def _hardware_test_loop(self):
        self.toggle_test_running = True
        self._print("\nHardware Test: Toggling all LEDs at 2 Hz. Press 't' again to stop...")
        regs = [REG_PWM0, REG_PWM1, REG_PWM2, REG_PWM3]
        try:
            state = False
            cycle = 0
            while self.toggle_test_running:
                state = not state
                cycle += 1
                val = BRIGHTNESS_HW_MAX if state else BRIGHTNESS_HW_MIN
                self._print(f"\n[Cycle {cycle}] {'ON' if state else 'OFF'}...")
                for reg in regs:
                    try:
                        self._write_register(reg, val)
                    except Exception as e:
                        self._print(f"  Write error: {e}")
                sleep_interval = 0.05
                total_sleep = 0.5
                slept = 0.0
                while slept < total_sleep and self.toggle_test_running:
                    time.sleep(sleep_interval)
                    slept += sleep_interval
        except Exception as e:
            self._print(f"\nTest error: {e}")
        finally:
            self.toggle_test_running = False
            self.update_all_leds(False)
            self._print("\nHardware test stopped - all LEDs OFF")

    def hardware_test(self):
        if self.toggle_test_running:
            self.toggle_test_running = False
            if self._test_thread and self._test_thread.is_alive():
                self._test_thread.join(timeout=1.0)
            return
        import threading
        if self._test_thread and self._test_thread.is_alive():
            return
        self._test_thread = threading.Thread(target=self._hardware_test_loop, daemon=True)
        self._test_thread.start()

    def dump_registers(self):
        self._print("\n" + "=" * 50)
        self._print("PCA9632 Register Dump (addr 0x62)")
        self._print("=" * 50)
        regs = [
            ('MODE1', REG_MODE1),
            ('MODE2', REG_MODE2),
            ('PWM0 (R)', REG_PWM0),
            ('PWM1 (G)', REG_PWM1),
            ('PWM2 (B)', REG_PWM2),
            ('PWM3 (W)', REG_PWM3),
            ('LEDOUT', REG_LEDOUT),
        ]
        for name, reg in regs:
            value = self._read_register(reg)
            if value is not None:
                extra = self._format_reg(name, value)
                self._print(f"  {name:12s} (0x{reg:02x}): 0x{value:02x} ({value:3d})  {extra}")
            else:
                self._print(f"  {name:12s} (0x{reg:02x}): READ FAILED")
        self._print("=" * 50)

    def _format_reg(self, name: str, value: int) -> str:
        if name == 'MODE1':
            sleep = 'SLEEP' if (value & MODE1_SLEEP_BIT) else 'NORMAL'
            return f"SLEEP={'1' if (value & MODE1_SLEEP_BIT) else '0'} ({sleep})"
        if name == 'LEDOUT':
            return f"LDR3..0 (0xAA=individual PWM)"
        if name.startswith('PWM'):
            return f"{int((value / 255.0) * 100)}% duty"
        return ""

    def run(self):
        try:
            print(f"Opening I2C bus {I2C_BUS}...")
            print(f"PCA9632 address: 0x{PCA9632_I2C_ADDRESS:02x} (individual; 0x70=All Call)")
            self.bus = smbus.SMBus(I2C_BUS)
            print("I2C bus opened")
            if not self.initialize_device():
                print("ERROR: Failed to initialize device (check wiring and i2cdetect -y 0)")
                return 1
            self.setup_terminal()
            self.print_help()
            self.print_status()
            self._print("\nPress keys to control LEDs (q to quit)...")
            while True:
                ch = self._getch()
                if ch is None:
                    continue
                if ch in ('q', '\x03'):
                    break
                if ch == 'r':
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
                    self.toggle_sleep()
                elif ch == 'v':
                    self.dump_registers()
                elif ch == 't':
                    self.hardware_test()
                elif ch == 's':
                    self.print_status()
                elif ch == 'h':
                    self.print_help()
                elif ch == '\x04':
                    break
                elif ch.isprintable():
                    self._print(f"Unknown key: '{ch}' (press 'h' for help)")
        except KeyboardInterrupt:
            self._print("\n\nInterrupted by user")
        except Exception as e:
            self._print(f"\n\nError: {e}")
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            import traceback
            traceback.print_exc()
        finally:
            self._print("\nTurning off all LEDs...")
            try:
                self.update_all_leds(False)
            except Exception:
                pass
            self.restore_terminal()
            print("Exiting...")
        return 0


def main():
    tester = LEDTester()
    return tester.run()


if __name__ == '__main__':
    sys.exit(main())
