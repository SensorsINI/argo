# LoRa: onboard SX1276 ↔ shore Waveshare SX1262

This document describes **hardware**, **link parameters**, and **packet format** for long-range radio between Argo (SPI **SX1276** / RA-02 class module, `nodes/lora.py`) and a shore machine using the **Waveshare USB-TO-LoRa-LF-B** (**SX1262**, USB serial).

**Shore software and ROS2 topics:** see [`../shore/README.md`](../shore/README.md).  
**LoRa behavior and rules:** [argo-lora-communication.mdc](../.cursor/rules/argo-lora-communication.mdc).

---

## Hardware

### Argo (onboard)

| Item | Detail |
|------|--------|
| Module | RA-02 / AI-Thinker 433 MHz class (sx1278-style board) |
| Chip | Semtech **SX1276 / SX1278** |
| Host | Orange Pi Zero 2W, **SPI1** |
| Software | `nodes/lora.py` |

### Shore

| Item | Detail |
|------|--------|
| Module | **Waveshare USB-TO-LoRa-LF-B** |
| Chip | Semtech **SX1262** |
| Connection | USB serial, typically `/dev/ttyACM0`, **115200** baud |
| Firmware | AT-based (e.g. Ver1.2) |
| Software | `shore/lora_shore.py` |

---

## Parameters that must match

User-configurable settings should be aligned on both sides:

| Parameter | SX1276 (Argo) | SX1262 (Waveshare) |
|-----------|----------------|---------------------|
| Frequency | 433 MHz | 433 MHz (LF: channel 23) |
| Spreading factor | SF7 | SF7 |
| Bandwidth | 125 kHz | 125 kHz (AT `BW=0`) |
| Code rate | 4/5 | 4/5 (AT `CR=1`) |
| Network ID / sync word | 18 (0x12) | NETID 18 |
| Mode | — | Stream (`MODE=1`) |
| Encryption | Off | Off (`KEY=0`) |

TX power may differ (e.g. 17 dBm on Argo vs 10 dBm on Waveshare); the receiver does not require matching TX levels.

### Limits of the Waveshare AT interface

These are **not** exposed via AT commands on typical Waveshare firmware; they must be compatible by design:

| Topic | Notes |
|-------|--------|
| CRC mode | Fixed in firmware |
| Header / explicit header mode | Fixed |
| Preamble length | Fixed |

Argo uses settings consistent with common LoRa PHY defaults; adding the stream-mode **header** (below) completed interoperability with Waveshare.

---

## Waveshare stream mode: 4-byte header

Waveshare firmware expects a **4-byte prefix** on data sent **to** the dongle in stream mode:

```
[Address Low] [Address High] [Channel] [Flags]
[   0x00   ]  [   0x00    ] [ 0x12 ]  [0x11]
```

- Bytes 0–1: address `0x0000` (point-to-point usage)  
- Byte 2: network/channel **0x12** (matches sync word / NETID 18)  
- Byte 3: flags **0x11** (as observed on Waveshare→Argo traffic)

`lora.py` prepends this header on transmit and strips the same header from incoming packets when present.

### Payload layout (conceptual)

**Toward Waveshare (Argo TX):**  
`[4-byte header][payload]`

**From Waveshare (serial):**  
Waveshare may strip the header before USB; additional RSSI/status bytes can appear after the payload (see `lora.py` / shore parser for current handling).

---

## Implementation references (Python)

Illustrative pattern (see `nodes/lora.py` for live code):

```python
waveshare_header = bytes([0x00, 0x00, 0x12, 0x11])
packet_with_header = waveshare_header + data
# TX: write FIFO with packet_with_header
# RX: if payload starts with header, strip first 4 bytes
```

---

## AT commands (Waveshare)

Enter / exit AT mode:

```
+++\r\n       # enter AT mode
AT+EXIT\r\n   # return to transparent / stream
```

Common queries (details in vendor documentation):

| Command | Meaning |
|---------|---------|
| `AT+VER?` | Firmware version |
| `AT+MODE?` | Mode (1=stream, …) |
| `AT+SF?` | Spreading factor |
| `AT+BW?` | Bandwidth |
| `AT+CR?` | Coding rate |
| `AT+NETID?` / `AT+NETID=18` | Network ID |
| `AT+TXCH?` / `AT+RXCH?` | Channel (433 MHz LF) |
| `AT+KEY?` | Encryption key (0 = off) |

Example (Python + pyserial):

```python
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(0.5)
ser.write(b"+++\r\n")
time.sleep(1)
ser.read(ser.in_waiting)
ser.write(b"AT+NETID=18\r\n")
time.sleep(0.5)
print(ser.read(ser.in_waiting).decode())
ser.write(b"AT+EXIT\r\n")
ser.close()
```

---

## Validation and tools

- Hardware SPI / GPIO: `nodes/test/test_lora_spi.py` (see `nodes/test/README.md`).
- Onboard node: `nodes/lora.py`.
- Shore node: `shore/lora_shore.py`, launcher `shore/run_lora_shore.sh`.

---

## Document history

Interop and header behavior verified October 2025 (SX1276 ↔ Waveshare SX1262, 433 MHz, indoor short range).
