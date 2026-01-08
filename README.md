# SPI COM Master

Python master implementation for a **10 ms, double-transfer SPI protocol** used in a robotics control link between a **Raspberry Pi 5** (master) and an **STM32 Nucleo-F446RE** (slave, Mbed-CE). Raspberry Pi 5 with Raspberry Pi OS or Ubuntu (64-bit).

## Overview

The Python code runs on the Raspberry Pi 5 to exchange **120-float frames** with the STM32 over `/dev/spidev0.0`.
Each loop cycle (default **10 ms**) performs two SPI transfers:

1. **ARM-ONLY frame** – `0x56` + zero payload + CRC
   - lets the slave re-arm and build a fresh reply.
2. **PUBLISH frame** – `0x55` + actual float payload + CRC
   - triggers the slave to send back the fresh data.

This **double-transfer** guarantees that the second reply (`rx2`) always contains an up-to-date data set from the slave.

## Features

- **High-speed SPI** at 33 MHz (mode 0)
- **120 × 32-bit floats** per frame (**482 bytes** total)
- **CRC-8** (polynomial `0x07`, init `0x00`) for error detection
- Accurate **10 ms cycle** using `time.perf_counter()`
- Prints per-cycle **Busy / Sleep / Xfer1 / Xfer2** timings
- Configurable **payload generator** via `load_tx_frame()`
- `chrt -f 50` recommended for real-time priority

## Wiring (Pi J8 → Nucleo-F446RE)

| Raspberry Pi 5 Pin | Function              | Nucleo F446RE Pin |
| ------------------ | --------------------- | ----------------- |
| 5V (PIN 2)         | Optional Power Supply | E5V               |
| GND (PIN 6)        | First GND             | GND below E5V     |
|                    |                       |                   |
| GPIO10 (Pin 19)    | MOSI                  | PC_3              |
| GND    (Pin 20)    | Second GND            | GND below AVDD    |
| GPIO9  (Pin 21)    | MISO                  | PC_2              |
| GPIO11 (Pin 23)    | SCLK                  | PB_10             |
| GPIO8  (Pin 24)    | CS (CE0)              | PB_12             |

It is important to connect two GNDs (pins 6 and 20) to ensure a stable reference.

## Key Parameters

| Variable              | Default | Description                                   |
| --------------------- | ------- | --------------------------------------------- |
| `SPI_NUM_FLOATS`      | 120     | number of float32 values in each frame        |
| `SPI_MSG_SIZE`        | 482     | header (1) + floats (480) + CRC (1)           |
| `main_task_period_us` | 10000   | loop period (µs) – 10 ms                      |
| `ARM_GAP_US`          | 100     | micro-gap between ARM-ONLY and PUBLISH frames |
| `spi.max_speed_hz`    | 33 MHz  | SPI bus speed                                 |

## Tuning

- **ARM_GAP_US** – adjust if you see “Failed” frames (e.g., 150 µs or 200 µs).
- **spi.max_speed_hz** – reduce if CRC errors appear (e.g., 25 MHz).
- **Printing frequency** – to reduce load, print every _N_-th frame for long runs.

## Notes

- Protocol is **master-driven**: the Pi always initiates both transfers.
- Only the **second reply** (`rx2`) is used; `rx1` is ignored.
- CRC failures or wrong headers increment `failed_count` but do not stop the loop.
- Tested on **Raspberry Pi 5 + Nucleo F446RE** with Mbed-CE SPI-DMA slave.

## Run

```bash
sudo chrt -f 50 python3 /home/pi/GIT_repositories/SPI_COM_Master/main.py
```
