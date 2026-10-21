<!-- link list -->
[0]: mbed_nucleo_f446re_pin_map.md
[1]: https://forms.cloud.microsoft/e/BFmzMtMYfv

# Workshop 1

The first workshop aims to introduce the Nucleo-F446RE microcontroller. We will use Visual Studio Code with PlatformIO to build and upload the existing Mbed application. The workshop will start by analyzing a given ***main.cpp*** file, its structure, and the implementation of basic commands.

Participants will explore hardware utilization, specifically the IR Sensor and a calibration process. To optimize the calibration parameters we will use either MATLAB or Python.

## Learning outcomes

By the end of this workshop you will be able to:
- Flash and gate a simple loop with the USER button on Nucleo-F446RE.
- Read an analog IR sensor signal and reason about its limits.
- Calibrate the IR sensor to map voltage to distance and use that mapping in code.

## Main file (main.cpp)

If you have already made changes and run tests in `main.cpp`, you can find the original version here: [main.cpp](../solutions/main_base.cpp). It is recommended to start with the original version of `main.cpp` for the workshop.

If you don’t want to lose your changes, save your modified file under a meaningful name in the folder: [temp](../../temp/)

Files stored in the `temp` folder will not be compiled. You can use it to keep different versions of `main.cpp`. For using a supplied application example, see [Working With Examples](../../README.md#working-with-examples).

## Before class (do this first)

**Expected time:** ~110–155 minutes total (you can split this into two shorter sessions).

**Required reading (come prepared so we can spend class time building/testing):**
- [README.md](../../README.md) (overview + safety + board structure).
- [Course Setup](course_setup.md) (installation, GitHub and cloning), followed by the [PlatformIO build guide](build_mbed_platformio.md).
- [Tips](tips.md) (programming hygiene + debugging basics).
- [Main file description](main_description.md) (loop, USER button gating, structure of `main.cpp`).
- [Infrared Distance Sensor](ir_sensor.md) (intro, range table, wiring, calibration overview).
- This document [Workshop 1](ws1.md) (flow + what happens in class).

**Quiz:** complete the short [MS Forms quiz (Workshop 1 Quiz)][1] so we know you can read wiring rules and analog scaling.

<p align="center">
    <img src="../images/ws1_quiz_qr_code.png" alt="Workshop 1 Quiz QR" width="240"/> </br>
    <i>Workshop 1 Quiz</i>
</p>

## Hardware

>Part 1:
> - Nucleo-F446RE board
> - Mini USB cable for flashing and serial communication
> - LED diode
> - 220–500 Ω series resistor for the LED
> - Jumper wires

>Part 2:
> - Nucleo-F446RE board
> - Mini USB cable
> - IR sensor (check which one you have in your kit, the model name is on the side, it will determine the maximum supported range of the measurements)
> - Additional wires to connect the sensor to the Nucleo board
> - Paper tape
> - Length measuring tape

### Assignment

We assume that you already have all necessary accounts set up and software installed to conduct the workshop. If not, go to: [Course Setup](course_setup.md), and follow the instructions there.

Furthermore, ensure that you are familiar with the general structure of the board. If not, refer to: [Nucleo F446RE](../../README.md#nucleo-f446re)

## Part 1

The first task will be to run a test program using an additional LED. Refer to [Main File Description](main_description.md) when tracing the loop and USER-button handling.

1. Follow the [hardware precautions](../../README.md#hardware) and disconnect power before wiring. Connect the additional LED in series with a **220–500 Ω resistor** between **PB_9** and **GND**, with its anode toward PB_9 and its cathode toward GND. See the [Nucleo Board Pinmap][0] and the comments in `src/main.cpp`.
2. After connecting the additional LED, connect the board's ST-LINK USB connector to your computer using the mini USB data cable. Check that its removable drive is available, as described in the [upload instructions](build_mbed_platformio.md#upload).
3. Open the project folder containing `platformio.ini`, save `src/main.cpp`, and select or press **PlatformIO: Upload**. This builds and flashes the program. Use **PlatformIO: Build** when you only want to compile.
4. Follow the upload progress in the terminal and wait for **SUCCESS**. When you need to view printed output, open the **Serial Monitor** tab from Microsoft's extension and start monitoring at **115200 baud**; the default program does not print a startup message. See the [serial-monitor instructions](build_mbed_platformio.md#serial-monitor).
5. The onboard user LED toggles once per main-loop iteration. With the default 20 ms period, that is approximately 50 toggles per second, or 25 complete on/off cycles per second; it may appear almost steady to the eye.
6. Now click the blue **USER** button on the Nucleo board while watching the additional LED.
7. Now click the blue **USER** button on the Nucleo board once again. What happens with the additional LED diode?

After finishing the task, read through the code, read the comments and try to understand it. Discuss the basic program in the group.

**If you have any questions, just ask them : - ), always!**

## Part 2

The second task will be carried out using an IR distance sensor. This task will be to calibrate the sensor so that it can be used to measure distances in centimeters. We want to learn more about how sensors are used, including the different situations in which they might be applied, unusual circumstances, and their limitations. This helps us better understand sensors as a whole system, so we can anticipate potential challenges as engineers.

1. Read the IR sensor manual. There you can find technical information about the sensor and instructions for the connection to the Nucleo board. Perform the calibration according to the hardware tutorial:

    > [Tutorial Infrared Distance Sensor](ir_sensor.md)

## Optional Self-Checks and Discussion

These optional prompts help you review the existing tasks; they add no submission or assessment requirement.

- **Part 1:** compare the extra LED's response to USER presses with the code controlled by `do_execute_main_task`. Identify which code continues to run when that task is disabled.
- **Part 2:** compare the calibrated distance with the reference distance in your measurements, using consistent units and your sensor's valid range. Consider keeping the measurements, fitted constants and plot with your saved code; see [Keeping Useful Notes](tips.md#keeping-useful-notes).

**Discussion prompt:** why can a fitted function agree with its calibration points but give a different error at another distance?

### Troubleshooting Hints

- **Build, upload or monitor problems:** use [PlatformIO Troubleshooting](build_mbed_platformio.md#troubleshooting). The default program has no startup message.
- **Unexpected LED behaviour:** compare your wiring with Part 1 and your program structure with [Main File Description](main_description.md).
- **Unexpected distance values:** compare the raw voltage, units, sensor model, valid range and calibration constants with the [IR Sensor Tutorial](ir_sensor.md#calibration).

For help, use [Asking for Debugging Help](tips.md#asking-for-debugging-help).

## Summary

In this workshop, participants were introduced to the Nucleo-F446RE board, acquiring familiarity with the PlatformIO workflow in VS Code and basic usage of objects in C++. The workshop included understanding of the ***main.cpp*** file, comprehension of its structure, defining and working with objects and the implementation of essential commands. Additionally, exploration of hardware utilization, particularly an IR sensor, and the corresponding calibration using MATLAB were conducted. This first workshop aimed to establish basic steps in using the system, laying the groundwork for further exploration for the development of robotics and mechatronics systems.

## Solutions

- [Workshop 1 Solution: Example Infrared Distance Sensor](../solutions/main_ir_sensor.cpp)
- [Workshop 1 Solution: Example Infrared Distance Sensor using IRSensor class](../solutions/main_ir_sensor_class.cpp)

<p align="center">
    <img src="../images/ir_sensor_eval.png" alt="IR Sensor Evaluation" width="950"/> </br>
    <i>IR Sensor Evaluation</i>
</p>
