#!/usr/bin/env python3
"""
Simple LoRa receiver test for Argo
Usage: python3 test_lora_rx.py
Press Ctrl+C to stop
"""
import spidev
import gpiod
import time
from datetime import datetime

# GPIO pins
LORA_SEL_LINE = 266  # PI10 (Pin 27)
LORA_RST_LINE = 256  # PI0  (Pin 29)

# SX1276 Registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_RSSI_VALUE = 0x1A
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_SYNC_WORD = 0x39
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42

# Modes
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05
MODE_RX_SINGLE = 0x06
MODE_LORA = 0x80

# IRQ flags
IRQ_RX_DONE = 0x40
IRQ_TX_DONE = 0x08
IRQ_RX_TIMEOUT = 0x80
IRQ_PAYLOAD_CRC_ERROR = 0x20

class SimpleLoRaRX:
    def __init__(self):
        # Initialize GPIO
        self.gpio_chip = gpiod.Chip("/dev/gpiochip0")
        self.lora_sel = self.gpio_chip.get_line(LORA_SEL_LINE)
        self.lora_sel.request(consumer="lora_rx", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        self.lora_rst = self.gpio_chip.get_line(LORA_RST_LINE)
        self.lora_rst.request(consumer="lora_rx", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
        
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(1, 0)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0
        
    def spi_write(self, addr, value):
        self.lora_sel.set_value(0)
        self.spi.xfer2([addr | 0x80, value])
        self.lora_sel.set_value(1)
    
    def spi_read(self, addr):
        self.lora_sel.set_value(0)
        result = self.spi.xfer2([addr & 0x7F, 0x00])
        self.lora_sel.set_value(1)
        return result[1]
    
    def reset(self):
        self.lora_rst.set_value(0)
        time.sleep(0.01)
        self.lora_rst.set_value(1)
        time.sleep(0.1)
    
    def init(self):
        print("Initializing LoRa receiver...")
        self.reset()
        
        # Check version
        version = self.spi_read(REG_VERSION)
        print(f"  Chip version: 0x{version:02X}")
        
        # Sleep mode
        self.spi_write(REG_OP_MODE, MODE_SLEEP | MODE_LORA)
        time.sleep(0.01)
        
        # Set frequency 433 MHz
        frf = int((433.0 * 1000000.0) / (32000000.0 / 524288.0))
        self.spi_write(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.spi_write(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.spi_write(REG_FRF_LSB, frf & 0xFF)
        
        # SF7, CRC on
        self.spi_write(REG_MODEM_CONFIG_2, (7 << 4) | 0x04)
        
        # BW 125kHz, CR 4/5
        self.spi_write(REG_MODEM_CONFIG_1, 0x70 | 0x02)
        
        # Preamble 8
        self.spi_write(REG_PREAMBLE_MSB, 0x00)
        self.spi_write(REG_PREAMBLE_LSB, 0x08)
        
        # Sync word 0x12
        self.spi_write(REG_SYNC_WORD, 0x12)
        
        # FIFO base
        self.spi_write(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)
        
        # LNA boost
        self.spi_write(REG_LNA, 0x23)
        
        # Standby
        self.spi_write(REG_OP_MODE, MODE_STDBY | MODE_LORA)
        time.sleep(0.01)
        
        # RX continuous
        self.spi_write(REG_OP_MODE, MODE_RX_CONTINUOUS | MODE_LORA)
        
        print("  LoRa RX initialized: 433 MHz, SF7, BW 125kHz, CR 4/5, Sync 0x12")
        print("  Ready to receive packets!\n")
    
    def check_rx(self):
        irq = self.spi_read(REG_IRQ_FLAGS)
        
        if irq & IRQ_RX_DONE:
            # Clear IRQ
            self.spi_write(REG_IRQ_FLAGS, 0xFF)
            
            # Check CRC error
            if irq & IRQ_PAYLOAD_CRC_ERROR:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] CRC ERROR!")
                return None
            
            # Get packet
            current_addr = self.spi_read(REG_FIFO_RX_CURRENT_ADDR)
            nb_bytes = self.spi_read(REG_RX_NB_BYTES)
            rssi = self.spi_read(REG_PKT_RSSI_VALUE)
            
            self.spi_write(REG_FIFO_ADDR_PTR, current_addr)
            
            data = []
            for i in range(nb_bytes):
                data.append(self.spi_read(REG_FIFO))
            
            payload = bytes(data)
            
            # Strip Waveshare header if present [0x00, 0x00, 0x12, 0x11]
            if len(payload) >= 4 and payload[0:4] == bytes([0x00, 0x00, 0x12, 0x11]):
                payload = payload[4:]
            
            return (payload, rssi)
        
        return None
    
    def close(self):
        self.spi.close()
        self.lora_sel.release()
        self.lora_rst.release()

if __name__ == "__main__":
    print("=== Argo LoRa Receiver Test ===\n")
    
    rx = SimpleLoRaRX()
    rx.init()
    
    print("Listening for packets from Waveshare...")
    print("(Press Ctrl+C to stop)\n")
    
    packet_count = 0
    last_status = time.time()
    
    try:
        while True:
            result = rx.check_rx()
            if result:
                data, rssi = result
                packet_count += 1
                rssi_dbm = -157 + rssi
                
                print(f"{'='*60}")
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Packet #{packet_count}")
                print(f"  Length: {len(data)} bytes")
                print(f"  RSSI: {rssi_dbm} dBm")
                print(f"  HEX: {data.hex(' ')}")
                try:
                    text = data.decode('utf-8')
                    print(f"  TEXT: {text}")
                except:
                    print(f"  TEXT: (not UTF-8)")
                print(f"{'='*60}\n")
            
            # Status every 10 seconds
            if time.time() - last_status > 10:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Still listening... ({packet_count} packets received)")
                last_status = time.time()
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {packet_count} packets total.")
    finally:
        rx.close()
