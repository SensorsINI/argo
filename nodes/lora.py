#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
Argo LoRa Communication Node
============================

The LoRa node handles long range communication with Argo. LoRa has maximum bandwidth 
of about 38kb/s so it can only transmit and receive small amounts of data.
It is used to send periodic boat state information to the host side and to receive 
remote commands such as "return home".

Hardware:
- LoRa radio based on SX1276 module (https://aithinker-static.oss-cn-shenzhen.aliyuncs.com/docs/_media_old/sx1276_77_78_79.pdf)
- Onboard versions:
  * Lora sx1278 AI-thinker 433MHz wireless RA-01 DIY kit (https://de.aliexpress.com/item/1005006824333382.html)
  * Ra-01 LoRa SX1278 433MHz (max range ~10km) (https://de.aliexpress.com/item/1005009649770115.html)
- Host side: USB To LoRa Data Transfer Module SX1262 (version USB-TO-LoRa-LF-B, 433MHz) (https://de.aliexpress.com/item/1005005461186227.html)

Connections:
- LoRa radio to Orange Pi: SPI as specified in the pins on Orange Pi
- Orange Pi to host side: USB To LoRa Data Transfer Module SX1262 (version USB-TO-LoRa-LF-B) (https://de.aliexpress.com/item/1005005461186227.html)

Pins on Orange Pi:
  * MISO on pin 21 (PH8/SPI1_MISO); this is the input from the LORA chip
  * MOSI on pin 19 (PH7/SPI1_MOSI); this is the output to the LORA chip
  * SPISCLK on pin 23 (PH6/SPI1_SCLK)
  * LORA_OUT on pin 31 (PI15) - this is interrupt ouptut from the LORA chip
  * !LORA_SEL on pin 27 (PI10) - low to select LORA chip for SPI
  * !LORA_RST on pin 29 (PI0) - low to reset LORA chip
  
The LoRa radio node is connected to the Orange Pi via SPI. On the remote host side (shore side) 
a USB To LoRa Data Transfer Module SX1262 (version USB-TO-LoRa-LF-B) (https://de.aliexpress.com/item/1005005461186227.html) is used.
providing robust, low-bandwidth communication for status reporting, remote commands, 
and emergency fallback.

Host side communication is provided by a remote node "lora_shore.py" which is running on the host side. This node is not running on the Orange Pi.

Published Topics:
- /lora_rx_data (std_msgs/String): Raw data received from LoRa radio
- /lora_remote_command (std_msgs/String): Parsed remote commands (e.g., "return_home")
- /lora_connection_status (std_msgs/Bool): LoRa radio connection health
- /lora_signal_strength (std_msgs/Int32): RSSI signal strength indicator

Subscribed Topics:
- /lora_tx_data (std_msgs/String): Data to transmit via LoRa radio

Services:
- /lora/send_command (std_srvs/srv/Trigger): Send immediate command/status update

Key Features:
- Bandwidth-optimized periodic status transmission
- Remote command reception and parsing
- Signal strength monitoring (RSSI)
- Connection health monitoring with automatic recovery
- Configurable transmission rate to manage bandwidth
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Bool, Int32, Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
import time
import json
import argparse
import argcomplete
import sys
from typing import Optional, Dict, Any
import threading

# SPI and GPIO libraries
try:
    import spidev
    SPI_AVAILABLE = True
except ImportError:
    SPI_AVAILABLE = False
    print("WARNING: spidev not available. Install with: pip3 install spidev")

try:
    import gpiod
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: gpiod library not available. Install with: pip3 install gpiod")


# SX1276 Register definitions (from Semtech datasheet)
class SX1276Registers:
    """SX1276 LoRa module register addresses"""
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
    REG_IRQ_FLAGS_MASK = 0x11
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_PKT_RSSI_VALUE = 0x1A
    REG_PKT_SNR_VALUE = 0x1B
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_PAYLOAD_LENGTH = 0x22
    REG_MODEM_CONFIG_3 = 0x26
    REG_RSSI_WIDEBAND = 0x2C
    REG_DETECTION_OPTIMIZE = 0x31
    REG_DETECTION_THRESHOLD = 0x37
    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40
    REG_DIO_MAPPING_2 = 0x41
    REG_VERSION = 0x42
    REG_PA_DAC = 0x4D

    # Operating modes
    MODE_LONG_RANGE_MODE = 0x80
    MODE_SLEEP = 0x00
    MODE_STDBY = 0x01
    MODE_TX = 0x03
    MODE_RX_CONTINUOUS = 0x05
    MODE_RX_SINGLE = 0x06

    # IRQ flags
    IRQ_TX_DONE = 0x08
    IRQ_RX_DONE = 0x40
    IRQ_PAYLOAD_CRC_ERROR = 0x20

    # PA config
    PA_BOOST = 0x80


# Orange Pi GPIO line numbers for gpiod
# These are the GPIO chip line numbers used by gpiod library
LORA_SEL_LINE = 266   # PI10 (Pin 27) - Chip Select
LORA_RST_LINE = 256   # PI0 (Pin 29) - Reset  
LORA_IRQ_LINE = 271   # PI15 (Pin 31) - Interrupt/DIO0


class LoRaNode(Node):
    """
    ROS2 LoRa communication node for Argo autonomous sailboat.

    Manages low-bandwidth bidirectional communication with ground station via
    LoRa radio module, optimized for the ~38kbps bandwidth constraint.
    """

    def __init__(self, debug_mode=False):
        super().__init__('lora_node')

        # Set logger level to DEBUG if debug mode is enabled
        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().debug('Debug logging enabled')

        self.get_logger().info('Initializing LoRa communication node (SPI mode)...')

        # Check if required libraries are available
        if not SPI_AVAILABLE or not GPIO_AVAILABLE:
            self.get_logger().error("CRITICAL: Required libraries not available")
            self.get_logger().error("Install: pip3 install spidev gpiod")
            sys.exit(1)

        # Declare and get parameters
        self.declare_parameter('spi_bus', 1)  # SPI1 on Orange Pi
        self.declare_parameter('spi_device', 0)  # CS0
        self.declare_parameter('frequency_mhz', 433.0)  # 433 MHz ISM band
        self.declare_parameter('tx_power_dbm', 17)  # TX power (max 20 dBm)
        self.declare_parameter('spreading_factor', 7)  # SF7-SF12
        self.declare_parameter('bandwidth_khz', 125)  # 125, 250, or 500 kHz
        # Status transmission interval
        self.declare_parameter('tx_interval_sec', 10.0)
        # LoRa packet size limit
        self.declare_parameter('max_packet_size', 255)

        self.spi_bus = self.get_parameter(
            'spi_bus').get_parameter_value().integer_value
        self.spi_device = self.get_parameter(
            'spi_device').get_parameter_value().integer_value
        self.frequency_mhz = self.get_parameter(
            'frequency_mhz').get_parameter_value().double_value
        self.tx_power_dbm = self.get_parameter(
            'tx_power_dbm').get_parameter_value().integer_value
        self.spreading_factor = self.get_parameter(
            'spreading_factor').get_parameter_value().integer_value
        self.bandwidth_khz = self.get_parameter(
            'bandwidth_khz').get_parameter_value().integer_value
        self.tx_interval = self.get_parameter(
            'tx_interval_sec').get_parameter_value().double_value
        self.max_packet_size = self.get_parameter(
            'max_packet_size').get_parameter_value().integer_value

        # Publishers
        self.pub_rx_data = self.create_publisher(String, 'lora_rx_data', 10)
        self.pub_remote_command = self.create_publisher(
            String, 'lora_remote_command', 10)
        self.pub_connection_status = self.create_publisher(
            Bool, 'lora_connection_status', 10)
        self.pub_signal_strength = self.create_publisher(
            Int32, 'lora_signal_strength', 10)

        # Subscribers
        self.sub_tx_data = self.create_subscription(
            String, 'lora_tx_data', self.tx_data_callback, 10)

        # Subscribers for boat state data (to build status packets)
        self.sub_gps_sog = self.create_subscription(
            Float64, 'gps_sog', self.gps_sog_callback, 10)
        self.sub_gps_cog = self.create_subscription(
            Float64, 'gps_cog', self.gps_cog_callback, 10)
        self.sub_battery_voltage = self.create_subscription(
            Float64, 'battery_voltage', self.battery_voltage_callback, 10)
        self.sub_human_controlled = self.create_subscription(
            Bool, 'human_controlled', self.human_controlled_callback, 10)
        # Add GPS position and compass heading subscriptions
        self.sub_gps_fix = self.create_subscription(
            NavSatFix, 'fix', self.gps_fix_callback, 10)
        self.sub_compass = self.create_subscription(
            Vector3, 'pose', self.compass_callback, 10)

        # Services
        self.srv_send_command = self.create_service(
            Trigger, '/lora/send_command', self.send_command_callback)

        # State tracking
        self.debug_mode = debug_mode
        self.is_connected = False  # Start disconnected until we receive data
        self.last_rssi = 0
        self.tx_queue = []  # Queue for outgoing messages
        self.rx_lock = threading.Lock()  # Lock for thread-safe operations

        # Boat state for status packets
        self.boat_state = {
            'gps_sog': None,
            'gps_cog': None,
            'battery_voltage': None,
            'human_controlled': None,
            'gps_latitude': None,
            'gps_longitude': None,
            'compass_heading': None,
            'timestamp': None
        }

        # Communication timeout tracking
        self.last_rx_time = time.time()
        self.connection_timeout_sec = 60.0  # Consider disconnected after 60s no rx
        
        # Ping tracking for shore connection monitoring
        self.last_ping_time = time.time()
        self.ping_timeout_sec = 30.0  # Consider disconnected after 30s no pings
        self.rth_on_ping_loss_enabled = False  # Configurable feature
        self.declare_parameter('rth_on_ping_loss', False)
        self.rth_on_ping_loss_enabled = self.get_parameter('rth_on_ping_loss').get_parameter_value().bool_value
        
        # Packet sequence tracking for link quality monitoring
        self.tx_sequence = 0
        self.rx_sequence = 0
        self.packet_loss_count = 0
        self.last_packet_loss_check = time.time()
        
        # Throttled reception logging for startup debugging
        self.first_reception_logged = False
        self.last_reception_log_time = 0.0
        self.reception_log_interval = 60.0  # Log reception success once per minute
        
        # Connection status publishing
        self.last_connection_status_publish = time.time()
        self.connection_status_publish_interval = 120.0  # Publish status at least every 2 minutes

        # Initialize SPI and GPIO
        self.spi = None
        self.gpio_chip = None
        self.lora_sel_line = None
        self.lora_rst_line = None
        self.lora_irq_line = None
        
        try:
            # Initialize GPIO using gpiod (no root required)
            self.gpio_chip = gpiod.Chip("/dev/gpiochip0")
            self.get_logger().debug("GPIO chip opened")
            
            # Configure LORA_SEL (Chip Select) as output
            self.lora_sel_line = self.gpio_chip.get_line(LORA_SEL_LINE)
            self.lora_sel_line.request(consumer="lora_node", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])  # CS high (inactive)
            self.get_logger().debug(f"LORA_SEL (line {LORA_SEL_LINE}) configured as output")
            
            # Configure LORA_RST (Reset) as output
            self.lora_rst_line = self.gpio_chip.get_line(LORA_RST_LINE)
            self.lora_rst_line.request(consumer="lora_node", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])  # Reset high (inactive)
            self.get_logger().debug(f"LORA_RST (line {LORA_RST_LINE}) configured as output")
            
            # Configure LORA_IRQ (Interrupt) as input
            self.lora_irq_line = self.gpio_chip.get_line(LORA_IRQ_LINE)
            self.lora_irq_line.request(consumer="lora_node", type=gpiod.LINE_REQ_DIR_IN)
            self.get_logger().debug(f"LORA_IRQ (line {LORA_IRQ_LINE}) configured as input")

            # Initialize SPI
            self.spi = spidev.SpiDev()
            self.get_logger().debug(f"Opening SPI bus {self.spi_bus}, device {self.spi_device}")
            
            self.spi.open(self.spi_bus, self.spi_device)
            self.get_logger().debug("SPI device opened")
            
            self.spi.max_speed_hz = 500000  # 500 kHz SPI clock
            self.spi.mode = 0  # SPI Mode 0 (CPOL=0, CPHA=0)
            # Note: We manually control CS via GPIO (LORA_SEL on pin 27)
            # Hardware CS on pin 24 is not used

            self.get_logger().info(
                f"SPI initialized on bus {self.spi_bus}, device {self.spi_device}")
            # Note: is_connected remains False until we receive data from shore

        except Exception as e:
            import traceback
            self.get_logger().error(
                f"CRITICAL: Failed to initialize SPI/GPIO: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            self.get_logger().error("CRITICAL: LoRa radio not accessible. Operating without LoRa.")
            self.is_connected = False

        # Initialize LoRa module
        if self.spi:  # Only if SPI is available
            if not self.initialize_lora_module():
                self.get_logger().error("LoRa module initialization failed")

        # Setup GPIO interrupt for packet reception
        # Note: gpiod interrupt handling requires event monitoring in a separate thread
        # For now, we rely on polling in the timer callback
        # TODO: Implement gpiod event monitoring for DIO0 interrupt (line 271)
        if self.spi:
            self.get_logger().info("LoRa using polling mode (interrupt mode not yet implemented with gpiod)")

        # Timers
        self.tx_timer = self.create_timer(
            self.tx_interval, self.transmit_status)  # Periodic status
        self.health_timer = self.create_timer(
            5.0, self.check_connection_health)  # Health check
        self.rx_poll_timer = self.create_timer(
            1.0, self.poll_rx_status)  # Poll RX status every second

        self.get_logger().info("LoRa node ready. Monitoring radio communication...")

        # Publish initial connection status
        self.publish_connection_status()

    def spi_write_register(self, address: int, value: int):
        """Write a single byte to a register"""
        with self.rx_lock:
            self.lora_sel_line.set_value(0)  # CS low (select chip)
            self.spi.xfer2([address | 0x80, value])  # Write bit (MSB=1)
            self.lora_sel_line.set_value(1)  # CS high (deselect chip)

    def spi_read_register(self, address: int) -> int:
        """Read a single byte from a register"""
        with self.rx_lock:
            self.lora_sel_line.set_value(0)  # CS low (select chip)
            result = self.spi.xfer2([address & 0x7F, 0x00])  # Read bit (MSB=0)
            self.lora_sel_line.set_value(1)  # CS high (deselect chip)
            return result[1]

    def spi_write_fifo(self, data: bytes):
        """Write data to FIFO"""
        with self.rx_lock:
            self.lora_sel_line.set_value(0)  # CS low (select chip)
            self.spi.xfer2([SX1276Registers.REG_FIFO | 0x80] + list(data))
            self.lora_sel_line.set_value(1)  # CS high (deselect chip)

    def spi_read_fifo(self, length: int) -> bytes:
        """Read data from FIFO"""
        with self.rx_lock:
            self.lora_sel_line.set_value(0)  # CS low (select chip)
            result = self.spi.xfer2(
                [SX1276Registers.REG_FIFO & 0x7F] + [0x00] * length)
            self.lora_sel_line.set_value(1)  # CS high (deselect chip)
            return bytes(result[1:])

    def reset_module(self):
        """Reset the LoRa module"""
        self.lora_rst_line.set_value(0)  # Reset low (active)
        time.sleep(0.01)  # Hold reset for 10ms
        self.lora_rst_line.set_value(1)  # Reset high (inactive)
        time.sleep(0.1)  # Wait 100ms for crystal oscillator to stabilize

    def set_mode(self, mode: int):
        """Set operating mode"""
        self.spi_write_register(SX1276Registers.REG_OP_MODE,
                                SX1276Registers.MODE_LONG_RANGE_MODE | mode)

    def set_frequency(self, freq_mhz: float):
        """Set carrier frequency in MHz"""
        frf = int((freq_mhz * 1000000.0) / 32000000.0 * 524288.0)
        self.spi_write_register(
            SX1276Registers.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.spi_write_register(SX1276Registers.REG_FRF_MID, (frf >> 8) & 0xFF)
        self.spi_write_register(SX1276Registers.REG_FRF_LSB, frf & 0xFF)

    def initialize_lora_module(self):
        """Initialize LoRa module via SPI"""
        try:
            self.get_logger().info("Initializing SX1276 LoRa module via SPI...")

            # Reset module
            self.reset_module()
            time.sleep(0.1)

            # Check version
            version = self.spi_read_register(SX1276Registers.REG_VERSION)
            self.get_logger().info(f"SX1276 chip version: 0x{version:02X}")
            if version != 0x12:
                self.get_logger().warn(
                    f"Unexpected chip version: 0x{version:02X} (expected 0x12)")

            # Set to sleep mode for configuration
            self.set_mode(SX1276Registers.MODE_SLEEP)
            time.sleep(0.01)

            # Set frequency
            self.set_frequency(self.frequency_mhz)
            self.get_logger().info(
                f"Frequency set to {self.frequency_mhz} MHz")

            # Set spreading factor (SF7-SF12)
            sf_value = (self.spreading_factor << 4) | 0x04  # Enable CRC
            self.spi_write_register(
                SX1276Registers.REG_MODEM_CONFIG_2, sf_value)
            self.get_logger().info(
                f"Spreading factor set to SF{self.spreading_factor}")

            # Set bandwidth
            bw_map = {125: 0x70, 250: 0x80, 500: 0x90}
            bw_value = bw_map.get(self.bandwidth_khz, 0x70)
            self.spi_write_register(
                SX1276Registers.REG_MODEM_CONFIG_1, bw_value | 0x02)  # Coding rate 4/5
            self.get_logger().info(
                f"Bandwidth set to {self.bandwidth_khz} kHz")

            # Set TX power
            if self.tx_power_dbm > 17:
                self.spi_write_register(
                    SX1276Registers.REG_PA_DAC, 0x87)  # High power
                pa_config = SX1276Registers.PA_BOOST | (self.tx_power_dbm - 2)
            else:
                self.spi_write_register(
                    SX1276Registers.REG_PA_DAC, 0x84)  # Normal power
                pa_config = SX1276Registers.PA_BOOST | (self.tx_power_dbm - 2)
            self.spi_write_register(SX1276Registers.REG_PA_CONFIG, pa_config)
            self.get_logger().info(f"TX power set to {self.tx_power_dbm} dBm")

            # Set preamble length
            self.spi_write_register(SX1276Registers.REG_PREAMBLE_MSB, 0x00)
            self.spi_write_register(
                SX1276Registers.REG_PREAMBLE_LSB, 0x08)  # 8 symbols

            # Set sync word (0x12 = private network)
            self.spi_write_register(SX1276Registers.REG_SYNC_WORD, 0x12)

            # Configure DIO0 for RxDone/TxDone interrupt
            self.spi_write_register(SX1276Registers.REG_DIO_MAPPING_1, 0x00)

            # Set FIFO base addresses
            self.spi_write_register(
                SX1276Registers.REG_FIFO_TX_BASE_ADDR, 0x00)
            self.spi_write_register(
                SX1276Registers.REG_FIFO_RX_BASE_ADDR, 0x00)

            # Enter standby mode
            self.set_mode(SX1276Registers.MODE_STDBY)

            # Enter RX continuous mode
            self.set_mode(SX1276Registers.MODE_RX_CONTINUOUS)

            self.get_logger().info("LoRa module initialized successfully, entering RX mode")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize LoRa module: {e}")
            import traceback
            traceback.print_exc()
            return False

    def on_dio0_interrupt(self, channel):
        """GPIO interrupt callback for DIO0 (packet received/transmitted)"""
        try:
            self.get_logger().debug("=== DIO0 INTERRUPT TRIGGERED ===")
            irq_flags = self.spi_read_register(SX1276Registers.REG_IRQ_FLAGS)
            self.get_logger().debug(f"IRQ flags: 0x{irq_flags:02X}")

            # Clear IRQ flags
            self.spi_write_register(SX1276Registers.REG_IRQ_FLAGS, 0xFF)

            if irq_flags & SX1276Registers.IRQ_RX_DONE:
                # Packet received
                self.handle_packet_received()
            elif irq_flags & SX1276Registers.IRQ_TX_DONE:
                # Packet transmitted
                self.get_logger().debug("Packet transmitted successfully")
                # Return to RX mode
                self.set_mode(SX1276Registers.MODE_RX_CONTINUOUS)

        except Exception as e:
            self.get_logger().error(f"Error in DIO0 interrupt handler: {e}")

    def handle_packet_received(self):
        """Handle received packet"""
        try:
            self.get_logger().debug("=== PACKET RECEIVED ===")
            
            # Read packet SNR and RSSI
            snr = self.spi_read_register(SX1276Registers.REG_PKT_SNR_VALUE)
            rssi_raw = self.spi_read_register(
                SX1276Registers.REG_PKT_RSSI_VALUE)

            # Calculate actual RSSI (for HF port < 779 MHz: -157 + rssi)
            if self.frequency_mhz < 779:
                rssi = -157 + rssi_raw
            else:
                rssi = -164 + rssi_raw

            self.last_rssi = rssi

            # Read received bytes count
            rx_nb_bytes = self.spi_read_register(
                SX1276Registers.REG_RX_NB_BYTES)
            self.get_logger().debug(f"RX bytes count: {rx_nb_bytes}")

            # Read current RX address
            fifo_rx_current_addr = self.spi_read_register(
                SX1276Registers.REG_FIFO_RX_CURRENT_ADDR)
            self.get_logger().debug(f"FIFO RX current addr: 0x{fifo_rx_current_addr:02X}")

            # Set FIFO address pointer
            self.spi_write_register(
                SX1276Registers.REG_FIFO_ADDR_PTR, fifo_rx_current_addr)

            # Read payload
            payload = self.spi_read_fifo(rx_nb_bytes)
            self.get_logger().debug(f"Payload raw bytes: {payload.hex()}")

            self.get_logger().debug(
                f"Packet received: {len(payload)} bytes, RSSI: {rssi} dBm, SNR: {snr}")

            # Throttled reception success logging for startup debugging
            current_time = time.time()
            if not self.first_reception_logged:
                self.get_logger().info(f"✅ FIRST PACKET RECEIVED: {len(payload)} bytes, RSSI: {rssi} dBm, SNR: {snr}")
                self.first_reception_logged = True
                self.last_reception_log_time = current_time
            elif current_time - self.last_reception_log_time >= self.reception_log_interval:
                self.get_logger().info(f"📡 LoRa reception active: {len(payload)} bytes, RSSI: {rssi} dBm, SNR: {snr}")
                self.last_reception_log_time = current_time

            # Strip Waveshare stream mode header if present
            # Expected header: [0x00, 0x00, 0x12, 0x11]
            if len(payload) >= 4 and payload[0:4] == bytes([0x00, 0x00, 0x12, 0x11]):
                self.get_logger().debug("Detected Waveshare header, stripping 4 bytes")
                payload = payload[4:]  # Remove header

            # Process received data
            self.last_rx_time = time.time()
            data_str = payload.decode('ascii', errors='ignore').strip()

            if data_str:
                # Publish raw received data
                rx_msg = String()
                rx_msg.data = data_str
                self.pub_rx_data.publish(rx_msg)

                # Publish RSSI
                rssi_msg = Int32()
                rssi_msg.data = rssi
                self.pub_signal_strength.publish(rssi_msg)

                # Parse and handle received data
                self.parse_received_data(data_str)

        except Exception as e:
            self.get_logger().error(f"Error handling received packet: {e}")

    # Callback methods for boat state subscribers
    def gps_sog_callback(self, msg):
        self.boat_state['gps_sog'] = msg.data

    def gps_cog_callback(self, msg):
        self.boat_state['gps_cog'] = msg.data

    def battery_voltage_callback(self, msg):
        self.boat_state['battery_voltage'] = msg.data

    def human_controlled_callback(self, msg):
        self.boat_state['human_controlled'] = msg.data
    
    def gps_fix_callback(self, msg):
        """Receive GPS position from gps node"""
        self.boat_state['gps_latitude'] = msg.latitude if msg.latitude != 0.0 else None
        self.boat_state['gps_longitude'] = msg.longitude if msg.longitude != 0.0 else None
    
    def compass_callback(self, msg):
        """Receive compass heading from pose/compass"""
        self.boat_state['compass_heading'] = msg.z

    def tx_data_callback(self, msg):
        """Queue data for transmission"""
        if len(msg.data) <= self.max_packet_size:
            self.tx_queue.append(msg.data)
            self.get_logger().debug(f"Queued for transmission: {msg.data}")
        else:
            self.get_logger().warn(
                f"Message too large ({len(msg.data)} bytes), max is {self.max_packet_size}")

    def send_command_callback(self, request, response):
        """Service to trigger immediate status transmission"""
        success = self.transmit_status()
        if success:
            response.success = True
            response.message = "Status transmitted successfully"
        else:
            response.success = False
            response.message = "Failed to transmit status"
        return response

    def build_status_packet(self) -> str:
        """Build compact status packet for LoRa transmission with data validation"""
        # Use compact JSON format to minimize bandwidth
        self.boat_state['timestamp'] = int(time.time())
        self.tx_sequence += 1

        # Validate battery voltage (6-9V range)
        battery_voltage = self.boat_state['battery_voltage']
        if battery_voltage is not None and (battery_voltage < 6.0 or battery_voltage > 9.0):
            self.get_logger().warn(f"Battery voltage out of range: {battery_voltage}V (expected 6-9V)")
            battery_voltage = None  # Don't send invalid data

        # Create compact packet (abbreviate keys to save bytes)
        packet = {
            'ts': self.boat_state['timestamp'],  # timestamp
            'seq': self.tx_sequence,  # sequence number for link quality monitoring
            'lat': round(self.boat_state['gps_latitude'], 6) if self.boat_state['gps_latitude'] is not None else None,
            'lon': round(self.boat_state['gps_longitude'], 6) if self.boat_state['gps_longitude'] is not None else None,
            'sog': round(self.boat_state['gps_sog'], 2) if self.boat_state['gps_sog'] is not None else None,
            'cog': round(self.boat_state['gps_cog'], 1) if self.boat_state['gps_cog'] is not None else None,
            'bat': round(battery_voltage, 2) if battery_voltage is not None else None,
            'hum': self.boat_state['human_controlled'],
            'hdg': round(self.boat_state['compass_heading'], 1) if self.boat_state['compass_heading'] is not None else None
        }

        # Remove None values to save bytes
        packet = {k: v for k, v in packet.items() if v is not None}

        return json.dumps(packet, separators=(',', ':'))

    def transmit_packet(self, data: bytes) -> bool:
        """Transmit a packet via LoRa SPI with Waveshare-compatible header"""
        try:
            if not self.spi:
                return False

            if len(data) > self.max_packet_size - 4:  # Reserve 4 bytes for header
                self.get_logger().warn(f"Packet too large: {len(data)} bytes")
                return False

            # Prepend Waveshare stream mode header:
            # Byte 0-1: Address (0x0000 = device address 0)
            # Byte 2: Channel/Network ID (0x12 = 18)
            # Byte 3: Unknown flag (0x11, observed in Waveshare transmissions)
            waveshare_header = bytes([0x00, 0x00, 0x12, 0x11])
            packet_with_header = waveshare_header + data

            # Enter standby mode
            self.set_mode(SX1276Registers.MODE_STDBY)

            # Set FIFO address pointer to TX base
            self.spi_write_register(SX1276Registers.REG_FIFO_ADDR_PTR, 0x00)

            # Write payload to FIFO (with header)
            self.spi_write_fifo(packet_with_header)

            # Set payload length (including header)
            self.spi_write_register(
                SX1276Registers.REG_PAYLOAD_LENGTH, len(packet_with_header))

            # Enter TX mode (DIO0 will trigger interrupt when done)
            self.set_mode(SX1276Registers.MODE_TX)

            self.get_logger().debug(f"Transmitted {len(packet_with_header)} bytes via LoRa (payload: {len(data)}, header: 4)")
            return True

        except Exception as e:
            self.get_logger().error(f"Error transmitting packet: {e}")
            # Try to return to RX mode
            try:
                self.set_mode(SX1276Registers.MODE_RX_CONTINUOUS)
            except:
                pass
            return False

    def transmit_status(self) -> bool:
        """Transmit boat status via LoRa radio with bandwidth monitoring"""
        try:
            # Build status packet
            status_packet = self.build_status_packet()
            packet_bytes = status_packet.encode('ascii')
            
            # Bandwidth check: ensure we don't exceed limits
            packet_size = len(packet_bytes)
            estimated_air_time = self._estimate_air_time(packet_size)
            
            if estimated_air_time > 2.0:  # Warn if packet takes >2 seconds
                self.get_logger().warn(
                    f"LoRa packet large: {packet_size} bytes, ~{estimated_air_time:.1f}s air time")
            
            # Log bandwidth usage
            self.get_logger().debug(
                f"LoRa TX: {packet_size} bytes, air time: ~{estimated_air_time:.2f}s, seq: {self.tx_sequence}")
            
            # Transmit
            success = self.transmit_packet(packet_bytes)
            if success:
                self.get_logger().debug(f"Status transmitted: {status_packet}")

            return success

        except Exception as e:
            self.get_logger().error(f"Error transmitting status: {e}")
            return False
    
    def _estimate_air_time(self, packet_size_bytes: int) -> float:
        """Estimate LoRa air time in seconds based on SF, BW, and packet size"""
        # Simplified formula for SF7, BW=125kHz, CR=4/5
        # Actual air time depends on preamble, header, payload, CRC
        # Rough estimate: ~8-12 ms per byte at SF7, ~40-50 ms per byte at SF9
        sf_multiplier = {7: 0.010, 8: 0.015, 9: 0.040, 10: 0.080, 11: 0.160, 12: 0.320}
        multiplier = sf_multiplier.get(self.spreading_factor, 0.040)
        return packet_size_bytes * multiplier

    def parse_received_data(self, data: str):
        """Parse received LoRa data and extract commands with ping handling"""
        try:
            # Try to parse as JSON command
            try:
                parsed = json.loads(data)
                
                # Handle ping messages
                if parsed.get('cmd') == 'ping':
                    self.last_ping_time = time.time()
                    self.get_logger().debug(f"Received ping #{parsed.get('seq', '?')}")
                    return
                
                # Handle sequence numbers for link quality monitoring
                if 'seq' in parsed:
                    expected_seq = self.rx_sequence + 1
                    received_seq = parsed['seq']
                    if received_seq != expected_seq:
                        self.packet_loss_count += 1
                        self.get_logger().debug(f"Packet loss detected: expected {expected_seq}, got {received_seq}")
                    self.rx_sequence = received_seq
                
                if 'cmd' in parsed:
                    command = parsed['cmd']
                    self.get_logger().info(
                        f"Received remote command: {command}")

                    # Publish command
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.pub_remote_command.publish(cmd_msg)

                if 'rssi' in parsed:
                    # Update signal strength
                    rssi_msg = Int32()
                    rssi_msg.data = int(parsed['rssi'])
                    self.pub_signal_strength.publish(rssi_msg)
                    self.last_rssi = int(parsed['rssi'])

            except json.JSONDecodeError:
                # Not JSON, treat as plain text command
                if data.startswith('CMD:'):
                    command = data[4:].strip()
                    self.get_logger().info(f"Received text command: {command}")

                    cmd_msg = String()
                    cmd_msg.data = command
                    self.pub_remote_command.publish(cmd_msg)

        except Exception as e:
            self.get_logger().debug(f"Could not parse received data: {e}")
        
        # Always publish received data for monitoring (regardless of parsing success)
        rx_msg = String()
        rx_msg.data = data
        self.pub_rx_data.publish(rx_msg)

    def check_connection_health(self):
        """Check LoRa connection health and optionally trigger RTH"""
        current_time = time.time()
        was_connected = self.is_connected
        self.get_logger().debug(f"Health check: was_connected={was_connected}, current_time={current_time}")

        # Check ping timeout for shore connection
        time_since_ping = current_time - self.last_ping_time
        if time_since_ping > self.ping_timeout_sec:
            self.is_connected = False
            if was_connected:
                self.get_logger().warn(
                    f"Shore ping timeout - no pings for {time_since_ping:.0f}s")
                
                # Optional: Trigger RTH on connection loss
                if self.rth_on_ping_loss_enabled:
                    self.get_logger().warn("Triggering RETURN TO HOME due to shore connection loss")
                    cmd_msg = String()
                    cmd_msg.data = 'return_home'
                    self.pub_remote_command.publish(cmd_msg)
        else:
            if not self.is_connected and self.spi:
                self.is_connected = True
                self.get_logger().info("Shore connection established")
        
        # Check if we've received data recently (fallback)
        if current_time - self.last_rx_time > self.connection_timeout_sec:
            if self.is_connected:  # Only warn if we thought we were connected
                self.get_logger().warn(
                    f"LoRa data timeout - no data for {self.connection_timeout_sec}s")
        else:
            if not self.is_connected and self.spi:
                self.is_connected = True
                self.get_logger().info("LoRa connection established")

        # Log packet loss statistics periodically
        if current_time - self.last_packet_loss_check > 60.0:  # Every minute
            if self.tx_sequence > 0:
                loss_rate = (self.packet_loss_count / self.tx_sequence) * 100
                self.get_logger().info(f"Packet loss rate: {loss_rate:.1f}% ({self.packet_loss_count}/{self.tx_sequence})")
                if loss_rate > 10.0:
                    self.get_logger().warn(f"High packet loss detected: {loss_rate:.1f}%")
            self.last_packet_loss_check = current_time

        # Publish connection status if changed or if enough time has passed
        time_since_last_publish = current_time - self.last_connection_status_publish
        self.get_logger().debug(f"Connection status check: time_since_last_publish={time_since_last_publish:.1f}s, interval={self.connection_status_publish_interval}s")
        
        if self.is_connected != was_connected or time_since_last_publish >= self.connection_status_publish_interval:
            self.publish_connection_status()
            self.last_connection_status_publish = current_time
            self.get_logger().info(f"Published connection status: {self.is_connected} (changed: {self.is_connected != was_connected}, periodic: {time_since_last_publish >= self.connection_status_publish_interval})")
        else:
            self.get_logger().debug(f"Connection status not published: no change and {time_since_last_publish:.1f}s < {self.connection_status_publish_interval}s")

    def poll_rx_status(self):
        """Poll RX status for packet reception (since interrupt mode not implemented)"""
        if not self.spi:
            return
            
        try:
            # Check IRQ flags for RX done
            irq_flags = self.spi_read_register(SX1276Registers.REG_IRQ_FLAGS)
            
            if irq_flags & SX1276Registers.IRQ_RX_DONE:
                self.get_logger().debug(f"RX Done flag detected: 0x{irq_flags:02X}")
                # Clear IRQ flags
                self.spi_write_register(SX1276Registers.REG_IRQ_FLAGS, 0xFF)
                # Handle the received packet
                self.handle_packet_received()
            elif irq_flags & SX1276Registers.IRQ_TX_DONE:
                self.get_logger().debug(f"TX Done flag detected: 0x{irq_flags:02X}")
                # Clear IRQ flags
                self.spi_write_register(SX1276Registers.REG_IRQ_FLAGS, 0xFF)
                # Return to RX mode
                self.set_mode(SX1276Registers.MODE_RX_CONTINUOUS)
            elif irq_flags & SX1276Registers.IRQ_PAYLOAD_CRC_ERROR:
                self.get_logger().debug(f"CRC Error flag detected: 0x{irq_flags:02X}")
                # Clear IRQ flags
                self.spi_write_register(SX1276Registers.REG_IRQ_FLAGS, 0xFF)
                # Return to RX mode
                self.set_mode(SX1276Registers.MODE_RX_CONTINUOUS)
            elif irq_flags != 0:
                self.get_logger().debug(f"Other IRQ flags: 0x{irq_flags:02X}")
                # Clear IRQ flags
                self.spi_write_register(SX1276Registers.REG_IRQ_FLAGS, 0xFF)
                
        except Exception as e:
            self.get_logger().error(f"Error polling RX status: {e}")

    def publish_connection_status(self):
        """Publish LoRa connection status"""
        status_msg = Bool()
        status_msg.data = self.is_connected
        self.pub_connection_status.publish(status_msg)

        if self.is_connected:
            self.get_logger().debug("LoRa connection status: CONNECTED")
        else:
            self.get_logger().debug("LoRa connection status: DISCONNECTED")

    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.get_logger().info("Shutting down LoRa node...")

        # Release GPIO lines
        try:
            if self.lora_sel_line:
                self.lora_sel_line.release()
            if self.lora_rst_line:
                self.lora_rst_line.release()
            if self.lora_irq_line:
                self.lora_irq_line.release()
            if self.gpio_chip:
                self.gpio_chip.close()
            self.get_logger().info("GPIO lines released")
        except Exception as e:
            self.get_logger().error(f"Error releasing GPIO: {e}")

        # Close SPI
        if self.spi:
            try:
                self.spi.close()
                self.get_logger().info("SPI closed")
            except Exception as e:
                self.get_logger().error(f"Error closing SPI: {e}")

        super().destroy_node()


def main(args=None):
    """Main function"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='LoRa Communication Node for Argo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node manages LoRa radio communication for the Argo autonomous sailboat:
- Low-bandwidth bidirectional communication via SPI
- Periodic status transmission to ground station
- Remote command reception with interrupt-driven RX
- Signal strength monitoring (RSSI) and SNR
- Connection health monitoring

Topics:
  Publishes:
    /lora_rx_data: String - Raw received data
    /lora_remote_command: String - Parsed commands
    /lora_connection_status: Bool - Connection health
    /lora_signal_strength: Int32 - RSSI signal strength (dBm)
    
  Subscribes:
    /lora_tx_data: String - Data to transmit
    /gps_sog: Float64 - Speed for status packets
    /gps_cog: Float64 - Course for status packets
    /battery_voltage: Float64 - Battery for status packets
    /human_controlled: Bool - Control mode for status packets

Services:
  /lora/send_command: Trigger - Send immediate status update

Parameters:
  spi_bus: SPI bus number (default: 1 for SPI1)
  spi_device: SPI device/CS (default: 0)
  frequency_mhz: Carrier frequency in MHz (default: 433.0)
  tx_power_dbm: TX power in dBm (default: 17, max 20)
  spreading_factor: LoRa SF (default: 7, range 7-12)
  bandwidth_khz: Bandwidth in kHz (default: 125, options: 125/250/500)
  tx_interval_sec: Status transmission interval (default: 10.0)
  max_packet_size: Maximum packet size in bytes (default: 255)

Hardware:
  SX1276/SX1278 LoRa module connected via SPI1
  Pins (Orange Pi Zero 2W):
    - MISO: Pin 21 (PH8/SPI1_MISO)
    - MOSI: Pin 19 (PH7/SPI1_MOSI)
    - SCLK: Pin 23 (PH6/SPI1_SCLK)
    - CS:   Pin 27 (PI10/LORA_SEL)
    - RST:  Pin 29 (PI0/LORA_RST)
    - IRQ:  Pin 31 (PI15/LORA_OUT/DIO0)
        """
    )
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')

    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args(args)

    # Initialize ROS2 with remaining arguments
    rclpy.init(args=unknown_args)
    node = None

    try:
        node = LoRaNode(debug_mode=parsed_args.debug)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("LoRa node interrupted, shutting down gracefully...")
    except ExternalShutdownException:
        if node:
            node.get_logger().info("External shutdown received, exiting gracefully...")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unexpected error: {e}")
        else:
            print(f"Error before node creation: {e}")
    finally:
        # Clean shutdown
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
