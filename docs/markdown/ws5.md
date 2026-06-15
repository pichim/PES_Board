<!-- link list -->
[0]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/
[1]: https://forms.cloud.microsoft/e/rkadzrxAGC

# Workshop 5

In workshop number 5, we will be working with the IMU (Inertial Measurement Unit) and see how the device can be used to estimate the orientation (tilt) in space. This will then be used to control a 2-axis gimbal system.

We will get to know Serial Streaming, which allows us to send data from the microcontroller to the host computer. This is particularly useful for debugging and development. Furthermore, we will implement our own sensor fusion algorithm with real data from the IMU. The goal is to calculate the orientation of the IMU in space (at least for one axis).

## Learning outcomes
By the end of this workshop, you will be able to:
- Understand the functionality of an IMU and how to read data from it
- Use Serial Streaming to send data from the microcontroller to a host computer
- Implement a complementary filter for sensor fusion to estimate orientation
- Implement a 1-D Mahony filter for sensor fusion to estimate orientation
- Integrate the sensor fusion algorithm into your microcontroller code to control a 2-axis gimbal system based on orientation estimates

## Main file (main.cpp)

If you have already made changes and run tests in `main.cpp`, you can find the original version here: [main.cpp](../solutions/main_base.cpp). It is recommended to start with the original version of `main.cpp` for the workshop.

If you don’t want to lose your changes, save your modified file under a meaningful name in the folder: [temp](../../temp/)

Files stored in the `temp` folder will not be compiled. You can use it to keep different versions of `main.cpp`.

## Before class (do this before the workshop)

**Expected time:** ~40-60 minutes total (split into two sessions if needed)

**Required reading (come prepared so we can spend time on the hands-on activities):**
- [README.md](../README.md) (10 minutes)
- This document [Workshop 5: IMU and Sensor Fusion](ws5.md) (20 minutes)
- [IMU Tutorial](imu.md) (20 minutes)

**Quiz:** complete the short [MS Forms quiz (Workshop 5 Quiz)][1] covering IMU functionality, sensor fusion, and gimbal control. (15 minutes)

<p align="center">
    <img src="../images/ws5_quiz_qr_code.png" alt="Workshop 5 Quiz QR" width="240"/> </br>
    <i>Workshop 5 Quiz</i>
</p>

## Hardware

>Part 1:
> - PES board with NUCLEO-F446RE board
> - Mini USB cable
> - 2-axis gimbal system with two servo motors
> - Jumper wires

>Part 2:
> - PES board with NUCLEO-F446RE board
> - Mini USB cable
> - USB Serial TTL cable

### Assignment

- You have successfully completed the previous workshops.

## Part 1

Part 1 of this workshop will focus on understanding the IMU functionality. We will use the information provided in the hardware tutorial: [Tutorial IMU](imu.md).

The goal here is to understand how the IMU works, how to read data from it, and how to use it to control a 2-axis gimbal system.

## Part 2

In the second part, we will learn how to stream data from the microcontroller to a host computer using Serial Streaming. This allows us to capture sensor data and process it on the host, either using MATLAB or Python for analysis and visualization. To get started with Serial Streaming, refer to the tutorial: [Tutorial Serial Stream](serial_stream.md).

The next step is to implement a sensor fusion algorithm that estimates the orientation of the IMU around the X-axis. The algorithm will be applied to the streamed IMU data, and the aim is to achieve results that are approximately equivalent to the built-in ``IMU``orientation estimation (Mahony filter) provided by the IMU class. The main task is to implement a complementary filter that estimates the orientation of the IMU around the X-axis using either MATLAB or Python. Additionally, a 1-D Mahony filter must be implemented to estimate the IMU orientation.

This implementation should then be ported to C++ and integrated into your microcontroller code so that, ultimately, you can control the 2-axis gimbal system based on the orientation estimates from your own algorithm for the roll and pitch angles. Adjusting the algorithm for the pitch estimate should be straightforward.

Start with the basic main file:

- [Main file for Complementary Filter](../solutions/main_comp_filter.cpp)

This is already set up to send all the necessary data to the host computer.

Templates to evaluate the data for either MATLAB or Python can be found here:

- [Template MATLAB Evaluation of IMU Data](../templates/matlab/serial_stream_comp_filter.m)
- [Template Python Evaluation of IMU Data](../templates/python/serial_stream_comp_filter.py)
- [Template Interactive Python Evaluation of IMU Data](../templates/python/serial_stream_comp_filter.ipynb)

For the theoretical background for the 1-D Mahony filter see: [Tutorial 1-D Mahony filter](1d_mahony.md). This will help you understand how to implement the filter first in MATLAB or Python and then port it to C++.

## Summary

In this workshop, we explored the IMU (Inertial Measurement Unit) and its application in estimating orientation in space. We learned how to use Serial Streaming to send data from the microcontroller to a host computer for further processing. The workshop included implementing a sensor fusion algorithm to calculate the orientation of the IMU.

## Solutions

- [Workshop 5 Part 1 Solution: Example Gimbal](../solutions/main_gimbal.cpp)
- [Workshop 5 Part 1 Solution: Example Gimbal using 1-D Mahony Filter](../solutions/main_gimbal_1d_mahony.cpp)
- [Workshop 5 Part 1 Solution: Example Gimbal using 3-D Mahony Filter](../solutions/main_gimbal_3d_mahony.cpp)
- [Workshop 5 Part 2 Solution: MATLAB Evaluation of IMU Data](../solutions/matlab/serial_stream_comp_filter.m)
- [Workshop 5 Part 2 Solution: Python Evaluation of IMU Data](../solutions/python/serial_stream_comp_filter.py)
- [Workshop 5 Part 2 Solution: Interactive Python Evaluation of IMU Data](../solutions/python/serial_stream_comp_filter.ipynb)
