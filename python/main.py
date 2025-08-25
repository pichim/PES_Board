# pi@rasperry:~/Projects/spi_com_master $ pinout
# Description        : Raspberry Pi 5B rev 1.1
# Revision           : d04171
# SoC                : BCM2712
# RAM                : 8GB
# Storage            : MicroSD
# USB ports          : 4 (of which 2 USB3)
# Ethernet ports     : 1 (1000Mbps max. speed)
# Wi-fi              : True
# Bluetooth          : True
# Camera ports (CSI) : 2
# Display ports (DSI): 2

# ,--------------------------------.
# | oooooooooooooooooooo J8   : +====
# | 1ooooooooooooooooooo      : |USB2
# |  Wi  Pi Model 5B  V1.1  fan +====
# |  Fi     +---+      +---+       |
# |         |RAM|      |RP1|    +====
# ||p       +---+      +---+    |USB3
# ||c      -------              +====
# ||i        SoC      |c|c J14     |
# (        -------  J7|s|s 12 +======
# |  J2 bat   uart   1|i|i oo |   Net
# | pwr\..|hd|...|hd|o|1|0    +======
# `-| |-1o|m0|---|m1|--------------'

# J8:
#    3V3  (1) (2)  5V
#  GPIO2  (3) (4)  5V
#  GPIO3  (5) (6)  GND
#  GPIO4  (7) (8)  GPIO14
#    GND  (9) (10) GPIO15
# GPIO17 (11) (12) GPIO18
# GPIO27 (13) (14) GND
# GPIO22 (15) (16) GPIO23
#    3V3 (17) (18) GPIO24
# GPIO10 (19) (20) GND
#  GPIO9 (21) (22) GPIO25
# GPIO11 (23) (24) GPIO8
#    GND (25) (26) GPIO7
#  GPIO0 (27) (28) GPIO1
#  GPIO5 (29) (30) GND
#  GPIO6 (31) (32) GPIO12
# GPIO13 (33) (34) GND
# GPIO19 (35) (36) GPIO16
# GPIO26 (37) (38) GPIO20
#    GND (39) (40) GPIO21

# J2:
# RUN (1)
# GND (2)

# J7:
# COMPOSITE (1)
#       GND (2)

# J14:
# TR01 TAP (1) (2) TR00 TAP
# TR03 TAP (3) (4) TR02 TAP

# For further information, please refer to https://pinout.xyz/

"""
================================================================================
 SPI Wiring (Raspberry Pi 5 – Master)
 --------------------------------------------------------------------------------
 Raspberry Pi Pin (J8) | Function | Connected to Nucleo F446RE Pin
---------------------------------------------------------------------------------
 GPIO10 (Pin 19)       | MOSI     | PC_3
 GPIO9  (Pin 21)       | MISO     | PC_2
 GPIO11 (Pin 23)       | SCLK     | PB_10
 GPIO8  (Pin 24)       | CS (CE0) | PB_12
 GND    (Pin 25)       | GND      | GND
 3V3    (Pin 1)        | VCC      | 3V3
================================================================================
 Notes:
 - Raspberry Pi 5 is SPI MASTER (bus 0, device 0 → `/dev/spidev0.0`).
 - Uses spidev Python library for full-duplex transfers.
 - Ensure `dtparam=spi=on` is enabled in `/boot/firmware/config.txt`.
 - Protocol: **Double transfer** each 20 ms
     1) 0x56 + zero payload (ARM-ONLY) — lets the Nucleo re-arm/build fresh TX
     2) 0x55 + real payload (PUBLISH)  — Nucleo publishes/updates from this one
================================================================================
"""

# sudo chrt -f 50 python /home/pi/Projects/SPI_COM_Master/python/main.py

import spidev
import struct
import time

# ------------------ CHANGED: protocol constants ------------------
SPI_HEADER_MASTER = 0x55  # Raspberry Pi header: PUBLISH (second transfer)
SPI_HEADER_MASTER_ARM = 0x56  # Raspberry Pi header: ARM-ONLY (first transfer)
SPI_HEADER_SLAVE = 0x45  # Nucleo header

SPI_NUM_FLOATS = 120  # Number of float values in each message
SPI_MSG_SIZE = 1 + SPI_NUM_FLOATS * 4 + 1  # header + floats + checksum

# Main task period (like the C++ example)
main_task_period_us = 10000

# ------------------ CHANGED: always double-transfer ------------------
ARM_GAP_US = 100  # small gap so the slave can re-arm/build fresh TX


def busy_wait_us(us):
    """Busy-wait for 'us' microseconds (sub-ms precision)."""
    t0 = time.perf_counter()
    target = t0 + us / 1_000_000.0
    while time.perf_counter() < target:
        pass


def calculate_crc8(buffer):
    """Calculate CRC-8 with polynomial 0x07"""
    crc = 0x00  # Initial value
    for byte in buffer:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF  # Ensure 8-bit value
    return crc


def verify_checksum(buffer, expected_crc):
    return calculate_crc8(buffer) == expected_crc


class SpiData:
    """Data structure for SPI communication"""

    def __init__(self):
        self.data = [0.0] * SPI_NUM_FLOATS
        self.message_count = 0
        self.failed_count = 0
        self.last_delta_time_us = 0


# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI0.0 (MOSI: GPIO 10, MISO: GPIO 9, SCK: GPIO 11, CS: GPIO 8)
spi.max_speed_hz = 33333333
spi.mode = 0b00  # SPI mode 0

# Data structures
transmitted_data = SpiData()
received_data = SpiData()

# Initialize transmitted data with test values
transmitted_data.data[0] = 42.42
transmitted_data.data[1] = 98.76
transmitted_data.data[2] = 11.11
transmitted_data.data[3] = 55.55
transmitted_data.data[4] = 87.34

# Timing
start_time = time.perf_counter()
previous_time = start_time

while True:
    # Start timer (like main_task_timer.reset() in C++)
    cycle_start_time = time.perf_counter()

    # ---------------- First transfer: ARM-ONLY (0x56 + zeros) ----------------
    tx1 = bytearray(SPI_MSG_SIZE)
    tx1[0] = SPI_HEADER_MASTER_ARM
    # explicit zero payload
    for i in range(SPI_NUM_FLOATS):
        tx1[1 + i * 4 : 1 + (i + 1) * 4] = b"\x00\x00\x00\x00"
    tx1[-1] = calculate_crc8(tx1[:-1])

    rx1 = spi.xfer2(list(tx1))

    # short gap so the slave can process + re-arm with fresh TX
    busy_wait_us(ARM_GAP_US)

    # ---------------- Second transfer: PUBLISH (0x55 + real payload) ---------
    tx2 = bytearray(SPI_MSG_SIZE)
    tx2[0] = SPI_HEADER_MASTER
    for i in range(SPI_NUM_FLOATS):
        float_bytes = struct.pack("<f", transmitted_data.data[i])
        tx2[1 + i * 4 : 1 + (i + 1) * 4] = float_bytes
    tx2[-1] = calculate_crc8(tx2[:-1])

    rx2 = spi.xfer2(list(tx2))

    # Prefer the second reply (fresh data). Do NOT fallback to the first.
    if not (len(rx2) == SPI_MSG_SIZE and verify_checksum(rx2[:-1], rx2[-1]) and rx2[0] == SPI_HEADER_SLAVE):
        received_data.failed_count += 1
        # Optional: log the miss for tuning ARM_GAP_US
        # print("rx2 invalid (no fresh data this cycle)")
        # Skip processing this cycle
        main_task_elapsed_time_us = (time.perf_counter() - cycle_start_time) * 1000000.0
        if main_task_period_us - main_task_elapsed_time_us >= 0:
            time.sleep((main_task_period_us - main_task_elapsed_time_us) / 1000000.0)
        continue

    rx = rx2

    # ---------------- Process selected received message ----------------------
    header_received = rx[0]
    received_checksum = rx[-1]

    # Verify checksum
    if verify_checksum(rx[:-1], received_checksum):
        if header_received == SPI_HEADER_SLAVE:
            # Valid message - extract data
            for i in range(SPI_NUM_FLOATS):
                float_bytes = bytes(rx[1 + i * 4 : 1 + (i + 1) * 4])
                received_data.data[i] = struct.unpack("<f", float_bytes)[0]

            received_data.message_count += 1

            # Measure elapsed time
            current_time = time.perf_counter()
            delta_time_us = int((current_time - previous_time) * 1000000)
            previous_time = current_time
            received_data.last_delta_time_us = delta_time_us

            # Keep transmitted data constant - no incrementing
            # Transmitted data stays: [42.42, 98.76, 11.11]

            transmitted_data.message_count += 1

            # Print results
            print(f"Message: {received_data.message_count} | " f"Delta Time: {delta_time_us} us | " f"Received: [{received_data.data[0]:.2f}, {received_data.data[1]:.2f}, {received_data.data[2]:.2f}, {received_data.data[3]:.2f}, {received_data.data[4]:.2f}] | " f"Header: 0x{header_received:02X} | Failed: {received_data.failed_count}")
        else:
            # Wrong header
            received_data.failed_count += 1
            print(f"Wrong header! Expected: 0x{SPI_HEADER_SLAVE:02X}, Got: 0x{header_received:02X} | Failed: {received_data.failed_count}")
    else:
        received_data.failed_count += 1
        expected_checksum = calculate_crc8(rx[:-1])
        print(f"CRC failed! Expected: 0x{expected_checksum:02X}, Got: 0x{received_checksum:02X} | Failed: {received_data.failed_count}")

    # Read timer and make the main thread sleep for the remaining time span (like C++ example)
    main_task_elapsed_time_us = (time.perf_counter() - cycle_start_time) * 1000000.0
    if main_task_period_us - main_task_elapsed_time_us < 0:
        print("Warning: Main task took longer than main_task_period_ms")
    else:
        time.sleep((main_task_period_us - main_task_elapsed_time_us) / 1000000.0)
