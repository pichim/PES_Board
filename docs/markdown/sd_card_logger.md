# SD Card Logger

The **SD Card Logger** captures data and saves it in binary format on an SD card. It uses the `SDLogger` class, which internally buffers the measurements and writes them in bursts via a background thread. It only supports writing floating-point data, so when ever you want to log data that is not a float, you need to convert it to a float first, e.g. by using `static_cast<float>(not_a_float_value)` or simply `(float)(not_a_float_value)`.

This guide provides a basic overview of the functionality and two short examples showing how to use `SDLogger` to log data.

**Important Note: Logging with the SD Card Logger only works with the GNU Arm Embedded Toolchain which includes the GNU Compiler (gcc). Therefor it will not work with Mbed Studio which is using Arm Compiler 6 (armclang), you have to use PlatformIO!**

## Basic Functionality

The `SDLogger` class:
- Buffers floating-point data using a ring buffer.
- Writes data to the SD card from a low‐priority thread.
- Automatically flushes the file to disk every 5 seconds to minimize data loss.
- Handles buffer overflow (printing a message if the ring buffer is full).
- Creates a new file with a running number (`/sd/data/001.bin`, `/sd/data/002.bin`, etc.) when it starts.

## Hardware and Pin Configuration

To use the `SDLogger`, you need an **SD card**. Successfully tested sd cards are:

- Transcend 16 GB Micro SD HC1
- ScanDisk Ultra 16 GB Micro SD HC1 A1

The SD card module itself is alread on the PES board. The pin names are defined in `PESBoardPinMap.h`:

```
// SD-Card
#define PB_SD_MOSI PC_12
#define PB_SD_MISO PC_11
#define PB_SD_SCK PC_10
#define PB_SD_CS PD_2
```

## Example Usage

### How to use the SD Card Logger

Include the `SDLogger` header file in your `main.cpp` file:

```
#include "SDLogger.h"
```

Create an `SDLogger` object in the `main()` function:

```
// sd card logger
SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);
```

Now you are already set up to log data. The `SDLogger` class provides the following functions:

```
// write data to the internal buffer of the sd card logger and send it to the sd card
sd_logger.write(1.0f); // log 1.0f
sd_logger.send();
```

You need to call `sd_logger.send()` to write the data to the SD card. The `write()` function writes the data to the internal buffer of the `SDLogger` object. The `send()` function sends the data to the SD card.

Currently the `SDLogger` class supports to log up to 100 float values. To log multiple values, e.g. 3 values you can use

```
// write data to the internal buffer of the sd card logger and send it to the sd card
sd_logger.write(1.0f); // log 1.0f
sd_logger.write(2.0f); // log 2.0f
sd_logger.write(3.0f); // log 3.0f
sd_logger.send();
````

Maximum tested throughput was 22 float values at 500 Hz.

You can cast other data types to float, e.g.:

```
// write data to the internal buffer of the sd card logger and send it to the sd card
sd_logger.write((float)(1)); // cast 1 to float and log 1.0f
sd_logger.send();
```

### Examples 

Log an icrementing counter

- [Example SD-Card 1](../solutions/main_sd_card_logger_e1.cpp)
- [MATLAB Evaluation of Example SD-Card 1 Data](../matlab/sd_card_eval.m)
- [Python Evaluation of Example SD-Card 1 Data](../python/sd_card_eval.py)

Log Time and Data from an IR Distance Sensor

- [Example SD-Card 2](../solutions/main_sd_card_logger_e2.cpp)
- [MATLAB Evaluation of Example SD-Card 2 Data](../matlab/sd_card_with_time_eval.m)
- [Python Evaluation of Example SD-Card 2 Data](../python/sd_card_with_time_eval.py)
