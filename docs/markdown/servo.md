<!-- link list, last updated 07.12.2023 -->
[0]: https://www.futaba.ch/?cat=21&tit=Servo%20SBus
[1]: https://www.conrad.ch/de/p/reely-standard-servo-cys-s0090-analog-servo-getriebe-material-metall-stecksystem-jr-2203091.html?refresh=true#productHighlights
[2]: https://theorycircuit.com/servo-motor-driver-circuit/
[3]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/

# Analog Servos

A servo is an electrical motor designed for control over angular or linear position. It typically consists of a motor connected to a sensor for angle or position feedback and a controller that adjusts the motor's movement to track the specified setpoint. Typically used in applications such as robotics, automation, and remote control, servos are available in various types, including digital, continuous rotation and analog that are described in this tutorial.

<p align="center">
    <img src="../images/servo_image.png" alt="servo example" width="350"/> </br>
    <i>Example of analog servo</i>
</p>

## Technical Specifications

|                   | Futaba S3001                 | Reely S-0090                 |
| ----------------- | ---------------------------- | ---------------------------- |
| Speed at 60°      | 0.17 / 0.21 s (6.0 / 4.8 V)  | 0.14 / 0.12 s (6.0 / 7.0 V)  |
| Torque            | 0.29 / 0.24 Nm (6.0 / 4.8 V) | 0.88 / 0.98 Nm (6.0 / 7.0 V) |
| Operating Voltage | 4.8 - 6.0 V                  | 6.0 - 7.0 V                  |
| Control Frequency | 50 - 70 Hz                   | 50 - 70 Hz                   |
| Weight            | 45.1 g                       | 56 g                         |
| Dimensions        | 40.4 x 19.8 x 36 mm          | 41.0 x 20.0 x 38.0 mm        |
| Gear              | Plastic                      | Metal                        |

## Links

- [Futaba S3001][0]</br>
- [Reely S-0090][1]

## Datasheets

- [Futaba S3001](../datasheets/Futaba_Servo_S3001.pdf)

## Absolute Angle / Positioning

The internally used sensor measures the angle of the servo absolutely. This means that the servo knows the absolute position independently of the angle where the system was turned on. Even when power cycling the servo, it will always move to the same position when commanded to do so. This is a very useful feature for applications where the servo is used to control a specific angle or position. Therefore, these actuators do not need to be homed or initialized.

## Practical Tips

- Keep in mind that every servo requires calibration for proper operation. Calibration values can differ not only between different servo models but also among individual units of the same model.
- The plug and pin arrangement provides two connection options, but one is incorrect and can result in servo failure. Pay close attention to matching the GND pin with the GND servo wire.
- Operating servos beyond the minimum and maximum input values may cause audible stuttering in the device. It's advisable to either disable them via software when not used, or even better, to do a proper and precise calibration to avoid this issue.

## Servo Driver

The ``servo`` driver is designed for controlling servos, commanding the angle within a normalized range of 0.0f to 1.0f. Internally there is a motion planner running which can be used to perform smooth movements (acceleration-constrained trajectories).

### Connection to the PES Board

For the PES board, analog servos are associated with specific ports, outlined as follows:

```cpp
PB_D0
PB_D1
PB_D2
PB_D3
```

[PES Board Pinmap](../datasheets/pes_board_peripherals.pdf)

### Create Servo Objects

Add the servo driver ``Servo.h`` to the top of the ***main.cpp*** file:

```cpp
#include "Servo.h"
```

To be able to start to use the ``Servo`` driver, the initial step is to create the servo objects and specify the pins to which the hardware will be connected in the ``main()`` scope. In the following step, two servos are plugged into the pins **D0 - D1** on the PES board.

Create an object with the associated pins passed as an argument:

```cpp
// servo
Servo servo_D0(PB_D0);
Servo servo_D1(PB_D1);
```

### Calibration

In order to properly control the servo, the basic step that should be performed is the calibration.


><b>Why calibration?</b><br>
>Servos are commanded through PWM (Pulse Width Modulation) signals, enabling the adjustment of the servo motor rotation through varying duty cycle PWM pulses. Initially, the specific pulse width corresponding to a particular servo angle is unknown. Hence, a calibration process is necessary to determine the minimum pulse width for the minimum angle and the maximum pulse width associated with the maximum angle.
>
>The figure below is just an example and does not represent the real values for the servos we use nor the real maximum angle the servos can reach.
><p align="center">
>    <img src="../images/servo_figures.png" alt="Example Pulse Width, PWM and Servo Angle" width="850"/> </br>
>    <i>Example Pulse Width, PWM and Servo Angle</i>
></p>
>
>The charts above illustrate a direct dependency between pulse width and servo angle. As the pulse width increases, the servo angle changes proportionally. Keep in mind the minimum angle might not be at zero pulse width. This relationship is further demonstrated in the illustration below.
><p align="center">
>    <img src="../images/servo-motor-pwm-signal-rotation.png" alt="Example Pulse Widths and Corresponding Angles" width="650"/> </br>
>    <i>Example Pulse Widths and Corresponding Angles</i>
></p>
>
>In the second servo illustration, the zero position corresponds to a pulse width of 1 ms, while the maximum angle is achieved at 2 ms. Hence, a calibration process is undertaken to determine these values. These calibration values are specific to the example shown in the figure and do not hold for the servos we use. Also, our servo driver runs at 50 Hz, therefore the servo period response pulse width is 20 ms.
>
> For more information see [here][2].

The calibration process involves sending progressively wider pulses to determine the pulse width corresponding to the minimal angle and the pulse width corresponding to the maximum angle of the servo. Best practice is to start with a very small pulse width and gradually increase it until the servo starts moving. This process is repeated until the servo reaches its maximum angle and stops moving, even though the pulse width is further increased. The pulse width values corresponding to the minimum and maximum positions are used as calibration values. It may be necessary to use a slightly higher than minimal value and a slightly lower than maximum value to ensure that the servo is working as expected (safety margins).

After setting the minimum and maximum pulse width via the servo driver (calibrating the servo) sending 0.0f as a command will move the servo to the minimal angle, while sending 1.0f will move it to the maximum angle. The servo will not move beyond these values, even if the command is smaller than 0.0f or bigger than 1.0f.

>Hardware:
> - PES board with NUCLEO-F446RE board
> - Mini USB cable
> - Servo Futaba S3001/RELY S-0090
> - Additional wires to connect the servo to the board
> - Jumper wires

#### Procedure

Define the necessary variables required for the calibration process.

```cpp
float servo_input = 0.0f;
int servo_counter = 0; // define servo counter, this is an additional variable
                       // used to command the servo
const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));
```

To monitor the ``servo_input`` variable value, it's necessary to include the following printing statement inside the ``while()`` loop to be enforced every iteration regardless of whether the main task has been triggered:

```cpp
// print to the serial terminal
printf("Pulse width: %f \n", servo_input);
```

To activate the servo, use the following command. Place this command to enable the servo after the initiating the program execution with the **USER** button:

```cpp
// enable the servos
if (!servo_D0.isEnabled())
    servo_D0.enable();
if (!servo_D1.isEnabled())
    servo_D1.enable();
```

Next, use the following function and statements. These will enable the incremental adjustment of the servo position every one second. It is important to ensure that the incremental change in the servo position, i.e., the pulse width, is relatively small to obtain precise minimum and maximum values. Try to find a tradeoff between too large and therefore too long execution time to wait for and too small and therefore not long enough values.

```cpp
// command the servos
servo_D0.setPulseWidth(servo_input);
servo_D1.setPulseWidth(servo_input);

// calculate inputs for the servos for the next cycle
if ((servo_input < 1.0f) &&                     // constrain servo_input to be < 1.0f
    (servo_counter % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
    (servo_counter != 0))                       // avoid servo_counter = 0
    servo_input += 0.005f;
servo_counter++;
```

To reset the ``servo_input`` variable to zero and disable the servos without restarting the program, add the following command within the ``else()`` statement into the ``if()`` statement for resetting the variables. This is triggered by pressing the **USER** button while the main task is running (second time you press the button).

```cpp
// reset variables and objects
led1 = 0;
servo_D0.disable();
servo_D1.disable();
servo_input = 0.0f;
```

In the subsequent step, compile the program and flash it. Once complete, click the **USER** button to initiate the execution. This action prompts the ``servo_input`` variable value to display on the serial monitor.

The goal is to monitor the ``servo_input`` variable and the servo. Every one second, this variable increases by the specified value. Record the displayed value on paper after the servo initial movement takes place. Continue monitoring the variable and the servo until increasing the variable no longer results in further rotation. At this point, record the maximum value displayed on the screen.

Now that the values are known, beneath the servo object declaration, define the appropriate variables with the values obtained in the process.

```cpp
// minimal pulse width and maximal pulse width obtained from the servo calibration process
// futuba S3001
float servo_D0_ang_min = 0.0150f; // careful, these values might differ from servo to servo
float servo_D0_ang_max = 0.1150f;
// reely S0090
float servo_D1_ang_min = 0.0325f;
float servo_D1_ang_max = 0.1175f;

// servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
// servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
```

Now the ``servo_input`` variable in the range from 0.0f to 1.0f will be mapped in the driver internally to the pulse width range from value of ``servo_D0_ang_min`` to value of ``servo_D0_ang_max``.

#### Smooth Movement

The class design incorporates the capability to execute smooth movements by adjusting the servo's maximum acceleration. This feature is suitable for movements that need smooth motions, eliminating abrupt movements. As default, the servo will move as fast as possible.

The following function can be used as an example to establish smooth movement and needs to be placed after the ``Servo`` object calibration:

```cpp
// default acceleration of the servo motion profile is 1.0e6f
servo_D0.setMaxAcceleration(0.3f);
```

This function allows the adjustment of the maximum acceleration during the movement. If the argument is omitted, the function defaults to a very large number, resulting in the fastest possible movement. For a smooth motion, you can input any argument greater than zero.

<p align="center">
    <img src="../images/servo_smooth.png" alt="Servo movement with maximum Acceleration of 0.3f" width="650"/> </br>
    <center> <i>Servo movement with maximum Acceleration of 0.3f</i> </center>
</p>

| <center>Default settings</ <enter> | <center>Acceleration limited</nter>|
| - | - |
| <center><i> </i></center> | <center><i>``servo_D0.setMaxAcceleration(0.3f);``</i></center> |
| Without setting the acceleration the servo will move to its commanded position as fast as possible, leading to fast but not very smooth movements. | With a maximum acceleration, the movement becomes smooth, and acceleration values are constrained by the driver. The velocity during the initial stage increases with a constant acceleration and then decreases, maintaining the same acceleration value but with negative sign. This results in a smooth movement. The velocity in- and decreases linearly, for the first increase of the velocity the derivative is approximately 0.1545/(0.95 - 0.435) = 0.3f as set via the class interface. |

## Example

- [Example Servo](../solutions/main_servo.cpp)
