<!-- link list -->
[0]: https://www.sparkfun.com/products/13582
[1]: https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide
[2]: https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi/all
[3]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/

# Line Follower

## Line Follower Array

The sensor incorporates eight diodes for line detection, with each diode's illumination indicating the presence of a line beneath it. The IR brightness control and indicator can be adjusted with the on-board potentiometer. The sensor's I2C interface allows for easy integration with the PES board, and the sensor's low power consumption makes it suitable for battery-powered applications. The sensor's compact size and low weight make it an ideal choice for small robots.

<p align="center">
    <img src="../images/line_follower_sensor.png" alt="Line follower sensor" width="450"/>
</p>

## Technical Specifications

| Sparkfun Line follower sensor array |             |
| ----------------------------------- | ----------- |
| Sensor eyes number                  | 8           |
| Interface                           | I2C         |
| Supply Voltage                      | 5 V         |
| Supply Current                      | 25 - 185 mA |
| Read Cycle Time                     | 3.2 ms      |

## Links

- [Sparkfun Line Follower Sensor Array][0] <br>
- [Sparkfun Line Follower Hookup Guide][1]

## Datasheets

- [Sparkfun Line Follower Sensor Array](../datasheets/line_follower_array.pdf)

## **WARNING 1**

<b>Before attempting to connect the sensor, it is important to carefully review the section [Connection to the PES Board](line_follower.md#connection-to-the-pes-board). This sensor is highly sensitive, and mishandling during the connection will lead to destruction of the sensor. Therefore, caution is necessary to avoid damaging the unit!</b>

### Connection to the PES Board

As the communication protocol, I2C is used. I2C relies on a data pin and a clock pin (more information can be found [here][2]). To power the sensor, a voltage of 5V is required.

Using the following pins is recommended:
- Data Pin **PB_9**
- Clock Pin **PB_8**

[Mbed ST-Nucleo-F446RE][3]

## **WARNING 2 :-)**

<b>As previously emphasized, this sensor is highly sensitive, and improper connections can lead to damage. It's very important to thoroughly examine the provided pictures illustrating the correct way to connect the sensor.</b> <br>

<b>Take note of the pin descriptions on the sensor. Connect a red power cable to the pin labeled 5V and a black ground cable to the pin labeled GND.</b>
<p align="center">
    <img src="../images/line_follower_sensor_look.png" alt="Line follower sensor look from above" width="750"/>
</p>

To plug the power source, you will need to use:

- 2 m/f jumper wires (black and red)
<p align="center">
    <img src="../images/mf_line_follower_array_connection.png" alt="Line follower sensor mf connection" width="750"/>
</p>

- or

<p align="center">
    <img src="../images/ff_line_follower_array_connection.png" alt="Line follower sensor ff connection" width="750"/>
</p>

- and 2 f/f jumper wires (green and yellow) for the I2C communication

<p align="center">
    <img src="../images/SDA_SCL_plugging.png" alt="SDA_SCL" width="750"/>
</p>

## Sensor Bar Driver (in case you want to develop your own line following algorithm)

Include the necessary driver in the ***main.cpp*** file

```cpp
#include "SensorBar.h"
```

and create a variable for the distance from the wheel axis to the LEDs on the sensor bar/array and a ``SensorBar`` object with the pin names for the I2C communication 

- SDA pin - Used for data transfer in I2C standard (``PB_9``)
- SCL pin - Used for clock synchronization (``PB_8``)

```cpp
const float bar_dist = 0.083f; // distance from wheel axis to leds on sensor bar / array in meters
SensorBar sensorBar(PB_9, PB_8, bar_dist);
```

#### Sensor Bar Usage

If you want to read out the angle of the line relative to the robot and only update the variable ``angle`` when the line is detected, you need to define a persistent variable to store the angle. This variable should be updated in the main loop when the line is detected. The angle is calculated in the driver and can be accessed by the user.

```cpp
float angle = 0.0f;
```

The sensor bar driver provides functionality to read the sensor values and calculate the angle of the line relative to the robot's orientation.

```cpp
// only update sensor bar angle if a led is triggered
if (sensorBar.isAnyLedActive())
    angle = sensorBar.getAvgAngleRad();
```

You can now use this angle to control the robot's movement based on the line detection. If you want to use the line follower driver, please refer to the next section.

### Using Eigen Library (Linear Algebra)

You can use the Eigen library for linear algebra operations. The library is used for matrix operations, such as matrix multiplication and inversion, which are essential for the kinematic calculations in the driver.

As usual include the library in the ***main.cpp*** file.

```cpp
#include <Eigen/Dense>

#define M_PIf 3.14159265358979323846f // pi
```

Now you're able to define the mapping from wheel velocity to the robot velocities as a 2x2 matrix using the following code snippet. Check out [Differential Drive Kinematics](kinematics.md) for more information.

```cpp
const float r_wheel = 0.0564f / 2.0f; // wheel radius in meters
const float b_wheel = 0.13f;          // wheelbase, distance from wheel to wheel in meters
// transforms robot to wheel velocities
Eigen::Matrix2f Cwheel2robot;
Cwheel2robot << r_wheel / 2.0f,       r_wheel / 2.0f,
                r_wheel / b_wheel, -r_wheel / b_wheel;
```

Usually, the control law to follow a line is implemented with respect to the robot, so translational forward velocity and angular velocity. Assuming you already have the robot velocities, you can calculate the wheel velocities using the inverse of the matrix defined above.

```cpp
// map robot velocities to wheel velocities in rad/sec
Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;
```

In the above code snippet, the variable ``robot_coord`` is a 2x1 ``Eigen::Vector2f`` vector containing the robot translational forward velocity and the angular velocity. The resulting ``wheel_speed`` vector contains the right and left wheel velocities in radians per second.

To output the wheel velocities as setpoints to the DC motors, you can use the following code snippet.

```cpp
// setpoints for the dc-motors in rps
motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
```

## Examples for Summer School 2024

- [Differential Drive Robot Kinematics Calibration](../solutions/main_calib_kinematic_ss24.cpp)
- [Line follower Base Example](../solutions/main_line_follower_base_ss24.cpp)

## Examples for Ashesi 2025

- [Line follower Base Example](../solutions/main_line_follower_base_ss24.cpp)

## Line Follower Driver (in case you don't want to develop your own line following algorithm)

The ``LineFollower`` driver is designed to drive (control) a differential drive robot with a line follower array attached along a black line on white background.

To start using the ``LineFollower`` driver, the initial step in the ***main.cpp*** file is to create the ``LineFollower`` object and specify the pins to which the object will be assigned.

To set up the module in the main function, it's necessary that you define two DC motor objects. To do so, please see the instructions provided in [DC Motor](dc_motor.md). Code snippets that should be placed in the correct places:

```cpp
#include "DCMotor.h"
```

```cpp
// create object to enable power electronics for the DC motors
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
```

```cpp
const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                 // 6.0f V if you only use one battery pack
const float gear_ratio = 78.125f; 
const float kn = 180.0f / 12.0f;
// motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
```

**NOTE:**
- Follow the instructions [Motor M2 Closed-Loop Velocity Control](dc_motor.md#motor-m2-closed-loop-velocity-control)
- The control algorithm in the ``LineFollower`` driver works best if the motion planner for the DC motors is disabled (should be default).

### Create Line Follower Object

Initially, it's essential to add the suitable driver to our ***main.cpp*** file and then create an object with the following variables defined (in **SI** units):

- SDA pin - Used for data transfer in I2C standard (``PB_9``)
- SCL pin - Used for clock synchronization (``PB_8``)
- bar_dist - Distance from wheel axis to LEDs on sensor bar / array
- d_wheel - Wheel diameter in meters
- b_wheel - Wheelbase, distance from wheel to wheel in meters
- max_motor_vel_rps - Maximum motor speed given in revolutions per second

The remaining values are defined by default, but there is a possibility to change some of the parameters, as described below the description of the internal algorithm.

```cpp
#include "LineFollower.h"
```

```cpp
const float d_wheel = 0.035f;  // wheel diameter in meters
const float b_wheel = 0.1518f; // wheelbase, distance from wheel to wheel in meters
const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar / array in meters
// line follower
LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());
```

<!-- **NOTE:** 
- The velocity values provided as input are originally expressed in revolutions per second. However, within the driver, these values are converted into radians per second for calculation purposes. Once the calculation is completed in the driver, it is then converted back into revolutions per second. This conversion allows for the use of a unit directly compatible with the DC motor object. -->

### Parameters Adjustment

The ``LineFollower`` class provides functionality to dynamically adjust the following key parameters:

1. Proportional Gain (Kp) and Non-linear Gain (Kp_nl):
- Function: ``void setRotationalVelocityGain(float Kp, float Kp_nl)``
- Parameters: ``Kp`` and ``Kp_nl``
- Description: These parameters influence the proportional gain and non-linear gain (squared) in the robot's angular velocity controller, allowing the user to fine-tune the response to deviations from the desired line angle.

2. Maximum Wheel Velocity:
- Function: ``void setMaxWheelVelocityRPS(float wheel_vel_max)``
- Parameter: ``wheel_vel_max``
- Description: This parameter limits the maximum wheel velocity (argument in rotations per second), indirectly affecting the robot's linear and angular velocities. The user can adjust this limit to tune the performance of their system.

### Driver Usage

The mathematical operations carried out within the driver determine the speed values for each wheel: right and left. These speed values are expressed in revolutions per second (RPS), allowing direct control of the motors using these values. Below is the code that should be executed when the **USER** button is pressed.

```cpp
// visual feedback that the main task is executed, setting this once would actually be enough
led1 = 1;
enable_motors = 1;
motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
```

Don't forget to reset the variables when the **USER** button is pressed again.

```cpp
// reset variables and objects
led1 = 0;
enable_motors = 0;
```

**NOTE:** 
- The ``LineFollower`` class assumes that the right motor is M1 and the left motor is M2 (sitting on the robot and looking forward) and that a positive speed setpoint to the motor M1 and M2 will rotate the robot positively around the z-axis (counter-clockwise seen from above).

Below, you'll find an in-depth manual explaining the inner driver functions. While it's not mandatory to use this manual, familiarizing yourself with the content will certainly help. For enhanced comprehension, it's recommended to refer to the [Differential Drive Kinematics](kinematics.md) document, which provides explanations of the mathematical operations involved.

### Thread Algorithm Description

The ``followLine()`` function is a thread task method responsible for controlling the robot to follow a line based on sensor readings.

1. Thread Execution: The ``followLine()`` method runs continuously in a thread loop. It waits for a thread flag to be set before executing, indicating that it should perform the task.

2. Sensor Reading: Inside the ``while()`` loop, the method checks if any LEDs on the sensor bar are active. If any LEDs are active, it calculates the average angle of the detected line segments relative to the robot's orientation. This angle is stored in ``m_angle``.

3. Control Calculation:
- The maximum wheel velocity in radians per second is calculated based on the maximum wheel velocity in rotations per second (``m_wheel_vel_max_rps``).
- The rotational velocity (``m_robot_coord(1)``) is determined using a control ``ang_cntrl_fcn()`` function, which adjusts the robot's orientation to align it with the detected line.
- The translational velocity (``m_robot_coord(0)``) is determined using another control ``vel_cntrl_fcn()`` function, which calculates the robot's forward velocity based on the rotational velocity and geometric parameters.
- The robot's wheel speeds are calculated using the inverse transformation matrix ``m_Cwheel2robot``.

4. Wheel Velocity Conversion: The calculated wheel speeds are converted from radians per second to revolutions per second (``m_wheel_right_velocity_rps`` and ``m_wheel_left_velocity_rps``).

#### Angular Velocity Controller

The ``ang_cntrl_fcn()`` function is responsible for calculating the angular velocity of the robot based on the detected angle of the line relative to the robot's orientation. This function uses proportional and non-linear control to calculate the velocity based on the measured angle.

1. Input Parameters:
- ``Kp``: Proportional gain parameter for angular control.
- ``Kp_nl``: Non-linear gain parameter for angular control.
- ``angle``: The angle of the detected line relative to the robot's orientation.

2. Calculation:
- If the angle is positive (``angle >= 0``), the function calculates the angular velocity using the formula:
    ```cpp
    ang_vel = Kp * angle + Kp_nl * angle * angle
    ```
    This formula applies proportional control (``Kp * angle``) along with a non-linear correction term (``Kp_nl * angle * angle``).
- If the angle is zero or negative (``angle < 0``), the function calculates the angular velocity (``retval``) using a similar formula but with a negative sign for the non-linear term:
    ```cpp
    ang_vel = Kp * angle - Kp_nl * angle * angle
    ```

#### Linear Velocity Controller

The ``vel_cntrl_fcn()`` function calculates the linear velocity of the robot based on its angular velocity and geometric parameters. The function ensures that one of the robot's wheels always turns at the maximum velocity specified by the user while the other adjusts its speed to maintain the desired angular velocity.

1. Input Parameters:
- ``wheel_vel_max``: Maximum wheel speed in radians per second.
- ``rotation_to_wheel_vel``: Geometric parameter related to the distance between the wheels .
- ``robot_ang_vel``: Angular velocity of the robot.
- ``Cwheel2robot``: Transformation matrix from wheel to robot coordinates.

2. Calculation:
- If ``robot_ang_vel`` is positive, it assigns the maximum wheel speed to the first wheel and calculates the speed for the second wheel by subtracting ``2 * b * robot_ang_vel`` from the maximum speed.
- If ``robot_ang_vel`` is negative or zero, it assigns the maximum wheel speed to the second wheel and calculates the speed for the first wheel by adding ``2 * b * robot_ang_vel`` to the maximum speed.
- The function then calculates the robot's coordinate velocities by multiplying the transformation matrix ``Cwheel2robot`` by the wheel speeds.
- Finally, it returns the linear velocity of the robot, which corresponds to the velocity along the x-axis in the robot's coordinate system.

3. Output:
- The function returns the calculated linear velocity of the robot.

## Example

- [Line Follower](../solutions/main_line_follower.cpp)
