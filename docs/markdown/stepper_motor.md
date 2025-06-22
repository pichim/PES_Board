<!-- link list, last updated 11.02.2024 -->
[0]: https://www.pololu.com/product/1209
[1]: https://www.pololu.com/product/2267
[2]: https://www.pololu.com/product/1182
[3]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/
[4]: https://www.youtube.com/watch?v=89BHS9hfSUk&t=1s

# Stepper Motor

A stepper motor is an electromechanical device that converts electrical pulses into precise mechanical movements. Unlike traditional motors, stepper motors move in discrete steps, enabling accurate control of position and speed without requiring external feedback sensors, as their position is inherently known. This makes them ideal for cost-effective applications requiring precise positioning, such as robotics and 3D printing. Stepper motors operate by energizing windings in a specific sequence, generating a magnetic field that forces the rotor to move in fixed angular steps, ensuring controlled and incremental rotation.

<p align="center">
    <img src="../images/stepper_motor.png" alt="Stepper motor" width="400"/> </br>
    <i>Stepper motor</i>
</p>

## Specifications

Below are the specifications of two tested stepper motors as well as the external hardware driver that was used.

| Parameter            | NEMA 14  | NEMA 17    |
| -------------------- | -------- | ---------- |
| Polarity             | Bipolar  | Bipolar    |
| Size                 | 35x36 mm | 42.3x38 mm |
| Current rating       | 1000 mA  | 1680 mA    |
| Voltage rating       | 2.7 V    | 2.8 V      |
| Steps per revolution | 200      | 200        |
| Resistance           | 2.7 Ohm  | 1.65 Ohm   |
| Number of leads      | 4        | 4          |

| Parameter                    | A4988 Stepper Motor Driver Carrier |
| ---------------------------- | ---------------------------------- |
| Minimum operating voltage    | 8 V                                |
| Maximum operating voltage    | 35 V                               |
| Continuous current per phase | 1 A                                |
| Maximum current per phase    | 2 A                                |
| Minimum logic voltage        | 3 V                                |
| Maximum logic voltage        | 5.5 V                              |
| Microstep resolutions        | full, 1/2, 1/4, 1/8, 1/16          |

## Links

- [NEMA 14][0] </br>
- [NEMA 17][1] </br>
- [A4988 Stepper Motor Driver Carrier][2]

## Datasheets

- [A4988 Stepper Motor Driver Carrier](../datasheets/A4988.pdf)

## Practical Tips

The A4988 stepper motor driver allows for adjustable current limiting, which helps prevent motor damage and optimize performance. The current limit is set using a trimmer potentiometer on the board. The method involves calculating the reference voltage $V_{REF}$ using the formula:

$$
I_{limit} = \frac{V_{REF}}{8 \times R_{sense}}
$$

where $R_{sense}$ is the sense resistor value, typically 0.05 Ohm for this board. By adjusting the $V_{REF}$, you can set the desired current limit to match your stepper motor's requirements. It's important to measure $V_{REF}$ directly at the potentiometer with the hardware driver powered but the motor disconnected.

This setup helps ensure your motor operates within safe limits, especially during high-demand scenarios.

For more information check out [A4988 Stepper Motor Driver Carrier][3] and the video [Setting the Current Limit on Pololu Stepper Motor Driver Carriers][4].

## Stepper Motor Example

The `Stepper` class driver controls a stepper motor. The hardware works with specific step and direction pins and steps-per-revolution settings. The class provides methods to retrieve the current step count, rotation, and velocity of the motor. It also includes methods to set the motor's velocity, absolute and relative rotation, and the number of steps. Additionally, the driver manages motor stepping, threading, and timing through its internal methods and variables.

To start working with the motors, it is necessary to wire them correctly and create an object in the ***main.cpp*** file and assign proper pins. In the following example, there will be two motors in use.

### Connection to the Nucleo Board

The motors are directly connected to the hardware driver according to the diagram below. The connection from the Nucleo board to the hardware driver needs two digital outputs, one to indicate direction and the other to indicate steps.

<p align="center">
    <img src="../images/pololu_driver_connection.png" alt="Pololu driver connection" width="700"/> </br>
    <i>Pololu driver connections</i>
</p>

A detailed sketch of the wiring can be found in the file [stepper_motor_connection.pdf](../dev/dev_steppermotor/stepper_motor_connection.pdf)

**Motor 1 Pins**

```cpp
Step PB_9
Direction PB_8
```

**Motor 2 Pins**

```cpp
Step PB_4
Direction PA_7
```

### Create Stepper Object

Initially, it's essential to add the suitable drivers to the ***main.cpp*** file and then create a ``Stepper`` object inside the ``main()`` function with the pin names passed as an argument.

```cpp
#include "Stepper.h"
```

```cpp
// stepper motors
Stepper stepper_M1(PB_9, PB_8);
Stepper stepper_M2(PB_4, PA_7);
```

### Stepper Motor Driver Overview

The driver is designed to work with stepper motors that require direction and step control signals. It supports various operations like setting the motor rotation, controlling the velocity, and handling relative rotations.

**Key Functionalities**

1. **Setting Rotation (`setRotation`)**:
   - **Purpose**: Moves the motor to a specified rotation count. You can set the rotation with or without specifying the velocity.
   - **Usage**:
     ```cpp
     myStepper.setRotation(2.0);      // moves to 2 full rotations at default velocity
     myStepper.setRotation(3.5, 2.0); // moves to 3.5 rotations at 2 rotations per second
     ```

2. **Relative Rotation (`setRotationRelative`)**:
   - **Purpose**: Rotates the motor by a specified relative amount from its current position. Like `setRotation`, you can specify velocity or use the current velocity.
   - **Usage**:
     ```cpp
     myStepper.setRotationRelative(1.0);       // moves 1 full rotation forward from the current position
     myStepper.setRotationRelative(-0.5, 1.5); // moves 0.5 rotations backward at 1.5 rotations per second
     ```

3. **Setting Velocity (`setVelocity`)**:
   - **Purpose**: Sets the motor's velocity directly, either moving it at a constant speed or stopping it if the velocity is zero.
   - **Usage**:
     ```cpp
     myStepper.setVelocity(2.0); // moves continuously at 2 rotations per second
     myStepper.setVelocity(0.0); // stops the motor
     ```

4. **Steps Control (`setSteps`)**:
   - **Purpose**: Moves the motor to a specific step count at a given velocity. It handles the direction automatically.
   - **Usage**:
     ```cpp
     myStepper.setSteps(1600, 2.0); // moves to step 1600 at 2 rotations per second
     ```

A full example can be found in the file [main_stepper_motor.cpp](../solutions/main_stepper_motor.cpp)

### Explanation of the Example

- **Stepper Motor Control**: The `setRotation` and `setRotationRelative` functions are used to control the stepper motors based on the current state of the task. These functions move the motor to a specified rotation with a given velocity.

### Usage Scenarios

- **Continuous Rotation**: If you want the motors to rotate continuously at a set speed, you can use the `setVelocity` function instead of `setRotation`.
- **Precise Positioning**: For applications requiring precise control over the motor's position, `setRotation` and `setRotationRelative` provide accurate movement control.
