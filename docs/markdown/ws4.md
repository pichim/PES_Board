<!-- link list -->
[0]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/
[1]: https://forms.cloud.microsoft/e/rkadzrxAGC

# Workshop 4

In the fourth workshop, we will explore differential drive robot kinematics and line following algorithms. Participants will learn how to transform desired robot velocities into individual wheel speeds and implement autonomous line following using an 8-sensor IR array.

## Learning Outcomes

By the end of this workshop, you will be able to:
- Understand and implement differential drive robot kinematics
- Use the SparkFun Line Follower Array to detect and follow lines
- Design control algorithms for line following behavior
- Calibrate robot parameters for improved performance

## Main file (main.cpp)

If you have already made changes and run tests in `main.cpp`, you can find the original version here: [main.cpp](../solutions/main_base.cpp). It is recommended to start with the original version of `main.cpp` for the workshop.

If you don’t want to lose your changes, save your modified file under a meaningful name in the folder: [temp](../../temp/)

Files stored in the `temp` folder will not be compiled. You can use it to keep different versions of `main.cpp`.

## Before class (do this before the workshop)

**Expected time:** ~40-60 minutes total (split into two sessions if needed)

**Required reading (come prepared so we can spend time on the hands-on activities):**
- [README.md](../README.md) (5 minutes)
- This document [Workshop 4: Differential Drive Kinematics and Line Following](ws4.md) (20 minutes)
- [Line Follower](line_follower_array.md) (20 minutes)

**Quiz:** complete the short [MS Forms quiz (Workshop 4 Quiz)][1] covering differential drive kinematics, line follower array functionality, and control algorithm design. (15 minutes)

<p align="center">
    <img src="../images/ws4_quiz_qr_code.png" alt="Workshop 4 Quiz QR" width="240"/> </br>
    <i>Workshop 4 Quiz</i>
</p>

## Hardware

>Part 1 and Part 2::
> - Fully functional differential drive robot

<p align="center">
    <img src="../images/differential_drive_robot.png" alt="Differential Drive Robot" width="650"/> </br>
    <i>Differential Drive Robot</i>
</p>

### Assignment

- You have successfully completed the previous workshops.

## Part 1

In the first part, you will learn about differential drive robot kinematics and implement the mathematical transformations in C++. The mathematical foundation and implementation details are described in the [Tutorial Differential Drive Robot Kinematics](dd_kinematics.md) document.

### Tasks Summary

1. Implement the kinematic transformation matrix using Eigen
2. Test the robot's response to different velocity commands
3. Optional: Calibrate wheel radius and wheelbase parameters

## Part 2

In the second part, you will implement a line following system using the SparkFun Line Follower Array.

The line follower array uses eight IR sensors to detect black lines. The sensor data is processed to calculate the line's angle relative to the robot, which is used to generate steering commands.

### Tasks Summary

1. Connect the line follower array using I2C communication
3. Design a control law combining forward motion with line tracking
4. Test and tune control parameters
5. Optional: Use the ``LineFollower`` driver class

Detailed instructions and examples are provided in the [Tutorial Line Follower](line_follower.md) document.

## Summary

In this workshop, you learned differential drive robot kinematics and line following algorithms. You implemented mathematical transformations to control robot motion and used sensor feedback to create autonomous navigation behavior.

## Solutions

- [Workshop 4 Part 1 Solution: Example Differential Drive Robot Kinematics Calibration](../solutions/main_dd_kinematic_calib.cpp)
- [Workshop 4 Part 2 Solution: Example Line Follower Base](../solutions/main_line_follower_base.cpp)
- [Workshop 4 Part 2 Solution: Example Line Follower](../solutions/main_line_follower.cpp)
