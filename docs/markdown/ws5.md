<!-- link list -->
[0]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/

# Workshop 5

In workshop number 5, we will be working with the IMU (Inertial Measurement Unit) and see how the device can be used to estimate the orientation (tilt) in space. This will then be used to control a 2-axis gimbal system.

We will get to know Serial Streaming, which allows us to send data from the microcontroller to the host computer. This is particularly useful for debugging and development. Furthermore, we will implement our own sensor fusion algorithm with real data from the IMU. The goal is to calculate the orientation of the IMU in space (at least for one axis).

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

In the second part, we'll learn how to stream data from the microcontroller to a host computer using Serial Streaming. This will allow us to capture data and process it on the host. Here we either use MATLAB or Python to process the data and visualize it. To get started with Serial Streaming, refer to the tutorial: [Tutorial Serial Stream](serial_stream.md).

The ultimate goal is to implement a sensor fusion algorithm that estimates the orientation of the IMU around the X-axis. The algorithm will be applied to the data received from the IMU. The goal here is to get roughly the same results as the ``IMU`` class built-in orientation estimation algorithm (Mahony filter). Here the main task is to discretize the needed low-pass filters for the complementary filter and apply these to the appropriate signals.

Start with the basic main file:

- [Main file for Complementary Filter](../solutions/main_comp_filter.cpp)

This is already set up to send all the necessary data to the host computer.

Templates to evaluate the data for either MATLAB or Python can be found here:

- [Template MATLAB Evaluation of IMU Data](../templates/matlab/serial_stream_comp_filter.m)
- [Template Python Evaluation of IMU Data](../templates/python/serial_stream_comp_filter.py)
- [Template Interactive Python Evaluation of IMU Data](../templates/python/serial_stream_comp_filter.ipynb)

## Summary

In this workshop, we explored the IMU (Inertial Measurement Unit) and its application in estimating orientation in space. We learned how to use Serial Streaming to send data from the microcontroller to a host computer for further processing. The workshop included implementing a sensor fusion algorithm to calculate the orientation of the IMU.

## Solutions

- [Workshop 5 Part 1 Solution: Example Gimbal](../solutions/main_gimbal.cpp)
- [Workshop 5 Part 2 Solution: MATLAB Evaluation of IMU Data](../solutions/matlab/serial_stream_comp_filter.m)
- [Workshop 5 Part 2 Solution: Python Evaluation of IMU Data](../solutions/python/serial_stream_comp_filter.py)
- [Workshop 5 Part 2 Solution: Interactive Python Evaluation of IMU Data](../solutions/python/serial_stream_comp_filter.ipynb)
