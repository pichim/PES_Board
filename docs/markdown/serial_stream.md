# SerialStream

The **SerialStream** class provides a simple interface for transmitting floating-point data from the microcontroller to a host computer (MATLAB or Python) via UART serial communication. It enables real-time data streaming for analysis and visualization. The class supports transmitting up to 30 floating-point values per transmission with a simple synchronization protocol.

This guide provides an overview of the SerialStream functionality, including microcontroller implementation, MATLAB integration, and Python analysis tools.

## Basic Functionality

The `SerialStream` system consists of three components:

### Microcontroller Side (`SerialStream` C++ class):
- Simple buffer for floating-point data (up to 30 float values)
- Transmits data packets via UART at 2 Mbaud baud rate
- Implements start-byte synchronization protocol (byte value 255)
- Automatically sends number of floats once when streaming starts
- Non-buffering, immediate transmission when `send()` is called

### MATLAB Side (`SerialStream.m` class):
- Receives serial data from the microcontroller
- Handles start-byte synchronization
- Collects data and structures it for analysis
- Includes timeout handling and data validation

### Python Side (`SerialStream.py` class):
- Same as MATLAB, but implemented in Python

## Hardware and Pin Configuration

To use the `SerialStream` class, you need an additional Serial to USB cable:

- USB Serial TTL cable

`SerialStream` uses UART communication with the following pin configuration:

```cpp
// Unused UART3
#define PB_UNUSED_UART_TX PB_10
#define PB_UNUSED_UART_RX PC_5
```

### Communication Protocol

`SerialStream` implements a simple protocol:
1. Host sends start byte (255) to trigger data transmission
2. Microcontroller responds by sending number of floats (once)
3. Microcontroller continuously sends float data when `send()` is called
4. Host collects and processes the received data

**Note: The start byte functionality is optional. On the microcontroller side, you can decide whether to use it or not. Its purpose is to trigger the data stream on the microcontroller from the host computer.**

## Example Usage

### How to use Serial Stream

Include the `SerialStream` header file in your `main.cpp`:

```cpp
#include "SerialStream.h"
```

Create a `SerialStream` object alongside with a `Timer` to measure time.

```cpp
// serial stream to send data over uart
SerialStream serialStream(PB_UNUSED_UART_TX, PB_UNUSED_UART_RX);

// additional timer to measure time elapsed since last call
Timer logging_timer;
logging_timer.start();
microseconds time_previous_us{0};
```

Measure Delta Time:

```cpp
// measure delta time
const microseconds time_us = logging_timer.elapsed_time();
const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
time_previous_us = time_us;
```

And send the data to the host computer:

```cpp
if (serialStream.startByteReceived()) {
    // send data over serial stream
    serialStream.write( dtime_us ); //  0 delta time in us
    serialStream.write((float)(1)); //  1 cast 1 to float and log 1.0f
    serialStream.send();
}
```

Maximum used throughput in a real application was sending 30 float values at 1 kHz.

**Important Note: Serial Stream relies on you sending delta time values in microseconds as the first signal. This is crucial for the correct functioning of the Serial Stream implementation on the host computer.**

### Key Methods

```cpp
void write(const float val); // write a float value to the internal buffer
void send();                 // send the buffered data to the host
bool startByteReceived();    // check if the host computer has sent the start byte
void reset();                // reset the SerialStream state and clear buffer
```

### How SerialStream Works

The SerialStream class maintains a simple internal buffer:
- `write()` adds float values to the buffer sequentially
- `send()` transmits all buffered data via UART
- Maximum buffer size is 30 float values (120 bytes)
- If the buffer is full after `write()`, `send()` is called automatically

### Examples 

Log two incrementing counters

- [Example Serial Stream](../solutions/main_serial_stream.cpp)
- [MATLAB Evaluation of Example Serial Stream Data](../matlab/serial_stream_eval.m)
- [Python Evaluation of Example Serial Stream Data](../python/serial_stream_eval.py)
- [Python Evaluation of Example Serial Stream Data](../python/serial_stream_eval.ipynb)
