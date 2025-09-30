# command: pinout
#
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

import spidev
import struct
import time
import math

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

# ============================ CRC-8 (poly 0x07) ==============================
CRC8_TAB = [
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
]


def calculate_crc8(buffer):
    """CRC-8 with polynomial 0x07 (table-based, fast)."""
    crc = 0x00
    for b in buffer:
        crc = CRC8_TAB[crc ^ b]
    return crc


def verify_checksum_seq(seq):
    """Verify CRC over all bytes except last, compared to last byte. Works with list/bytes/bytearray."""
    crc = 0x00
    last = len(seq) - 1
    for i in range(last):
        crc = CRC8_TAB[crc ^ seq[i]]
    return crc == seq[last]


class SpiData:
    """Data structure for SPI communication"""

    def __init__(self):
        self.data = [0.0] * SPI_NUM_FLOATS
        self.message_count = 0
        self.failed_count = 0
        self.last_delta_time_us = 0


# --------- NEW: hook to load/update TX payload for each frame (customize later) ---------
def load_tx_frame(tx_list, frame_idx):
    """
    Update tx_list (list of floats) in-place for this frame.
    """
    # Use monotonic time relative to start_time for stable phase
    t = time.perf_counter() - start_time
    tx_list[0] = 0.2333 * math.sin(2.0 * math.pi * 0.25 * t)
    tx_list[1] = 1.5000 * math.sin(2.0 * math.pi * 0.25 * t)
    # k = 1.0
    # tx_list[0] = k * 1.0
    # tx_list[1] = k * 2.0


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

# ---------- OPTIMIZED: prebuild constant tx1 (header 0x56 + zero payload + CRC) ----------
tx1 = bytearray(SPI_MSG_SIZE)
tx1[0] = SPI_HEADER_MASTER_ARM
tx1[1:-1] = b"\x00" * (SPI_NUM_FLOATS * 4)
tx1[-1] = calculate_crc8(memoryview(tx1)[:-1])  # zero-copy view

# ---------- OPTIMIZED: preallocate tx2 buffer (header fixed) ----------
tx2 = bytearray(SPI_MSG_SIZE)
tx2[0] = SPI_HEADER_MASTER

# Timing
start_time = time.perf_counter()
previous_time = start_time

while True:
    # Start timer (like main_task_timer.reset() in C++)
    cycle_start_time = time.perf_counter()

    # ---------------- First transfer: ARM-ONLY (0x56 + zeros) ----------------
    t_xfer1_start = time.perf_counter()
    rx1 = spi.xfer2(tx1)  # pass bytearray directly (no list() copy)
    t_xfer1_end = time.perf_counter()
    xfer1_us = (t_xfer1_end - t_xfer1_start) * 1_000_000.0

    # short gap so the slave can process + re-arm with fresh TX
    # (busy-wait retained to keep "Busy" semantics unchanged)
    t0 = time.perf_counter()
    target = t0 + ARM_GAP_US / 1_000_000.0
    while time.perf_counter() < target:
        pass

    # ---------------- Second transfer: PUBLISH (0x55 + real payload) ---------
    # Load/update TX payload for this frame (no-op by default; customize later)
    load_tx_frame(transmitted_data.data, transmitted_data.message_count)

    # OPTIMIZED: pack all floats in one go
    struct.pack_into("<%df" % SPI_NUM_FLOATS, tx2, 1, *transmitted_data.data)
    tx2[-1] = calculate_crc8(memoryview(tx2)[:-1])  # zero-copy CRC

    t_xfer2_start = time.perf_counter()
    rx2 = spi.xfer2(tx2)  # pass bytearray directly
    t_xfer2_end = time.perf_counter()
    xfer2_us = (t_xfer2_end - t_xfer2_start) * 1_000_000.0

    # Prefer the second reply (fresh data). Do NOT fallback to the first.
    if not (len(rx2) == SPI_MSG_SIZE and verify_checksum_seq(rx2) and rx2[0] == SPI_HEADER_SLAVE):
        received_data.failed_count += 1
        # Skip processing this cycle; compute busy/sleep for print
        main_task_elapsed_time_us = (time.perf_counter() - cycle_start_time) * 1_000_000.0
        remaining_us = main_task_period_us - main_task_elapsed_time_us
        if remaining_us < 0:
            sleep_us = 0.0
            print(f"Warning: Main task took longer than main_task_period_ms | Busy: {int(main_task_elapsed_time_us)} us | Sleep: {int(sleep_us)} us | Xfer1: {int(xfer1_us)} us | Xfer2: {int(xfer2_us)} us | Failed: {received_data.failed_count}")
        else:
            time.sleep(remaining_us / 1_000_000.0)
            sleep_us = remaining_us
            print(f"Busy: {int(main_task_elapsed_time_us)} us | Sleep: {int(sleep_us)} us | Xfer1: {int(xfer1_us)} us | Xfer2: {int(xfer2_us)} us | Failed: {received_data.failed_count}")
        continue

    rx = rx2

    # ---------------- Process selected received message ----------------------
    header_received = rx[0]

    # Verify checksum (already checked above) and extract data
    # OPTIMIZED: bulk unpack; convert list->bytearray once for struct
    rx_ba = bytearray(rx)
    floats_tuple = struct.unpack_from("<%df" % SPI_NUM_FLOATS, rx_ba, 1)
    # keep same list object (avoids reallocation churn)
    received_data.data[:] = floats_tuple

    received_data.message_count += 1

    # Measure elapsed time between valid messages
    current_time = time.perf_counter()
    delta_time_us = int((current_time - previous_time) * 1_000_000)
    previous_time = current_time
    received_data.last_delta_time_us = delta_time_us

    transmitted_data.message_count += 1

    # Read timer and compute sleep before printing
    main_task_elapsed_time_us = (time.perf_counter() - cycle_start_time) * 1_000_000.0
    remaining_us = main_task_period_us - main_task_elapsed_time_us
    if remaining_us < 0:
        sleep_us = 0.0
        print(
            f"Message: {received_data.message_count} | "
            f"Delta Time: {delta_time_us} us | "
            f"Received: [{received_data.data[0]:.4f}, {received_data.data[1]:.4f}, "
            f"{received_data.data[2]:.4f}, {received_data.data[3]:.4f}, {received_data.data[4]:.4f}] | "
            f"Header: 0x{header_received:02X} | Failed: {received_data.failed_count} | "
            f"Busy: {int(main_task_elapsed_time_us)} us | Sleep: {int(sleep_us)} us | "
            f"Xfer1: {int(xfer1_us)} us | Xfer2: {int(xfer2_us)} us"
        )
        print("Warning: Main task took longer than main_task_period_ms")
    else:
        time.sleep(remaining_us / 1_000_000.0)
        sleep_us = remaining_us
        print(
            f"Message: {received_data.message_count} | "
            f"Delta Time: {delta_time_us} us | "
            f"Received: [{received_data.data[0]:.4f}, {received_data.data[1]:.4f}, "
            f"{received_data.data[2]:.4f}, {received_data.data[3]:.4f}, {received_data.data[4]:.4f}] | "
            f"Header: 0x{header_received:02X} | Failed: {received_data.failed_count} | "
            f"Busy: {int(main_task_elapsed_time_us)} us | Sleep: {int(sleep_us)} us | "
            f"Xfer1: {int(xfer1_us)} us | Xfer2: {int(xfer2_us)} us"
        )
