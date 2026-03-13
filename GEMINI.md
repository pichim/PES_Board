# GEMINI.md - PES Board Collector Context

This project is a comprehensive robotics development platform based on **Mbed OS 6**, specifically designed for the **ST Nucleo-F446RE** microcontroller in combination with the custom **PES Board** (developed at ZHAW). It serves as a template and driver repository for various robotics workshops.

## Project Overview

- **Target Hardware:** ST Nucleo-F446RE + PES Board.
- **Framework:** Mbed OS 6 (C++).
- **Core Purpose:** Fast prototyping of robots (DC motors, servos, IMUs, encoders, various sensors).
- **Architecture:** 
  - `src/main.cpp`: Entry point with a standard 50Hz (20ms) real-time loop.
  - `lib/`: Extensive collection of custom hardware drivers and utility libraries.
  - `include/PESBoardPinMap.h`: Centralized pin mapping for the PES Board.
  - `docs/`: Detailed hardware tutorials and workshop instructions.

## Building and Running

The project supports multiple build environments:

### PlatformIO (VS Code)
- **Build:** `pio run`
- **Upload (Flash):** `pio run --target upload`
- **Monitor (Serial):** `pio device monitor` (Baud rate: 115200)
- **Clean:** `pio run --target clean`

### Mbed Studio
- Use the **Hammer** icon to build.
- Use the **Play** icon to build and flash.
- Set target to **NUCLEO-F446RE** and use the **Develop Profile**.

### Manual Flashing
1. Build the project to generate a `.bin` file (found in `BUILD/` or `.pio/build/`).
2. Connect the Nucleo board via USB (appears as `NODE_F446RE` drive).
3. Drag and drop the `.bin` file onto the Nucleo drive.

## Key Files and Directories

- `src/main.cpp`: Skeleton code for user logic. Toggled by the blue **USER BUTTON**.
- `include/PESBoardPinMap.h`: Defines constants like `PB_PWM_M1`, `PB_ENC_A_M1`, etc. Ensure `NEW_PES_BOARD_VERSION` is defined for modern boards.
- `lib/`: Contains critical drivers:
  - `DCMotor`, `EncoderCounter`: For locomotion.
  - `IMU`, `LSM9DS1`, `Mahony`: For orientation and motion sensing.
  - `IRSensor`, `UltrasonicSensor`, `SensorBar`: For environment sensing.
  - `PIDCntrl`, `Motion`, `GPA`: For control and signal processing.
- `platformio.ini`: Configuration for PlatformIO builds.
- `mbed_app.json`: Mbed OS configuration (Baud rate, SD card support, stack size).

## Development Conventions

1. **Loop Timing:** The `main` thread typically runs with a 20ms period (`main_task_period_ms`). User logic should be placed inside the `if (do_execute_main_task)` block.
2. **Safety:** Always turn off the PES board power switch before making hardware changes. The charger is NOT a power supply; batteries must be connected when charging.
3. **Pin Conflicts:** `PB_9` (used for `led1` in some examples) conflicts with the `SensorBar` (Line Follower).
4. **Library Style:** Most drivers follow a class-based approach. Use `PESBoardPinMap.h` for all pin references to ensure portability between board versions.
5. **Coding Style:** 
   - Use `DigitalOut`, `AnalogIn`, `PwmOut` etc. from the Mbed API.
   - Prefer non-blocking `thread_sleep_for()` for timing.
   - Follow the naming conventions established in `main.cpp` (e.g., `do_execute_main_task`).

## Key Commands (Summary)

```powershell
# Build using PlatformIO
pio run

# Upload to Nucleo
pio run --target upload

# Open Serial Monitor
pio device monitor
```
