<!-- link list, last updated 07.12.2023 -->
[0]: https://www.pololu.com/product/1136
[1]: https://www.pololu.com/product/136
[2]: https://www.pololu.com/product/2464
[3]: https://robocraze.com/blogs/post/ir-sensor-working
[4]: https://os.mbed.com/platforms/ST-Nucleo-F446RE/

# Infrared Distance Sensor

The analog distance sensor is equipped with an IR diode and uses triangulation to measure distances. Using infrared technology, it calculates distances by measuring angles (indirectly). This device emits infrared light, observes reflections, and provides real-time distance measurements. Valuable for tasks requiring distance determination, it serves as a cheap and simple solution for applications where you need to measure distance.


><b>How does it work?</b><br>
>Very briefly, infrared sensors work on the principle of reflected light waves. Infrared light is reflected from objects or sent from an infrared remote or beacon. Infrared sensors can be used to measure distance or proximity. The reflected light is detected and then an estimate of the distance between the sensor and the object is calculated. The following is a simple representation of the principle of operation:
><p align="center">
>    <img src="../images/how-infrared-sensors-work.png" alt="how_IR_works" width="450"/> </br>
>    <i> Working principle </i>
></p>
>
> More detailed explanation can be found [here][3].

<p align="center">
    <img src="../images/ir-distance-sensor.png" alt="IR_sensor" width="400"/> </br>
    <i> Example of IR distance sensor </i>
</p>

## Technical Specifications

|                                     |        |                       | Sharp GP2D120                      | Sharp GP2Y0A21YK0F                  | Sharp GP2Y0A41SK0F                 |
| ----------------------------------- | ------ | --------------------- | ---------------------------------- | ----------------------------------- | ---------------------------------- |
|                                     | Symbol | Conditions            |                                    |                                     |                                    |
| **Absolute Maximum Ratings**        |        | Ta=25 °C, Vcc = 5 VDC |                                    |                                     |                                    |
| Supply Voltage                      | Vcc    |                       | -0.3 to +7 V                       | -0.3 to +7 V                        | -0.3 to +7 V                       |
| Output Terminal Voltage             | Vo     |                       | -0.3 to (Vcc +0.3) V               | -0.3 to (Vcc +0.3) V                | -0.3 to (Vcc +0.3) V               |
| Operating Temperature               | Topr   |                       | -10 to +60 °C                      | -10 to +60 °C                       | -10 to +60 °C                      |
| **Operating Supply Voltage**        |
| Supply Voltage                      | Vcc    |                       | 4.5 to 5.5 V                       | 4.5 to 5.5 V                        | 4.5 to 5.5 V                       |
| **Electro-optical Characteristics** |        | Ta=25 °C, Vcc = 5 VDC |                                    |                                     |                                    |
| Measuring Distance Range            | ΔL     |                       | (MIN) 4.0 (TYP) ---- (MAX) 30.0 cm | (MIN) 10.0 (TYP) ---- (MAX) 30.0 cm | (MIN) 4.0 (TYP) ---- (MAX) 30.0 cm |

## Links

- [Sharp GP2D120][0] <br>
- [Sharp GP2Y0A21YK0F][1] <br>
- [Sharp GP2Y0A41SK0F][2] <br>

## Datasheets

- [Sharp GP2D120](../datasheets/GP2D120-DATA-SHEET.pdf) <br>
- [Sharp GP2Y0A21YK0F](../datasheets/gp2y0a21yk0f.pdf) <br>
- [Sharp GP2Y0A41SK0F](../datasheets/GP2Y0A41SK0F.pdf)

## Practical Tips

* Remember that reliable measurements can only be made within the measurement range. Be especially careful near the minimum range because of its curve near this point (see distance measuring characteristics chart in the technical documentation of the sensor).
* It is important to note that the underlying principle of this measurement method is based on the reflection of a light beam. Therefore, the measurement is significantly influenced by the surface of the reflecting object. Measuring objects with a surface that reflects light rays poorly will degrade the accuracy.

## Analog Distance Sensor

The ``AnalogIn`` class is a driver provided in the ``mbed-os`` library. The driver maps the input signal received from 0...3.3V to 0.0f...1.0f. This should be kept in mind when using the sensor to interpret the received values correctly.

### Connection to the Nucleo-Board

It is necessary to power the sensors with 5.0V. The sensor has 3 wires: one for signal transmission, one for ground, and one for power. The transmission wire needs to be connected to the Nucleo pin that allows the reception of an analog signal, in our example this is the **PC_2** pin.

[Nucleo Board pinmap][4]

If you are not sure how to connect the sensor, click the following hint.

<details>
<summary>Parts of the Nucleo F446RE Pin Map</summary>
<br>
<p align="center">
    <img src="../images/connection_pin_map.png" alt="Connection pin map" width="700"/> </br>
    <i>Connection Pin map with marked wire's colors</i>
</p>
</details>

### Create Analog Distance Sensor Object

To start working with the sensor, it is necessary to connect it correctly and create an ``AnalogIn`` object in the ***main.cpp*** file.

To be able to use the tooling from the Mbed platform, it is necessary to include the library at the beginning of the ***main.cpp*** file:

```cpp
#include "mbed.h"
```

Create an object with the pin's name passed as an argument and define a variable to store the corresponding reading from the sensor in millivolts:

```cpp
// ir distance sensor
float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in the infrared distance sensor
                             // 0...3.3V are mapped to 0...1
```

### Calibration

The sensor returns distances in normalized volts, which we then scale to millivolts, therefore it is necessary to convert the signal to a unit of length. To do so, it is necessary to determine the function (mapping) that converts the signal from millivolts into a distance in centimeters. This function can be determined by the calibration process. The calibration process is described in the following section.


><b>For what do we need the calibration?</b><br>
>Calibrating the IR distance sensor is essential to establish a precise relationship between the sensor's analog voltage readings and actual distances. In the technical documentation it is possible to find a dependency between voltage readings and distance such as the following:
>
><p align="center">
>   <img src="../images/dist_measure_char.PNG" alt="Distance measuring characteristics" width="550"/> </br>
>   <i>Distance measuring characteristics</i>
></p>
>The above figure shows expected values that can serve as a reference for the measurements to be made. However, all sensors, especially those of lower quality (hobby grade), may be characterized by a slightly altered curve, so a calibration process should be carried out before using such sensors for an application where the distance needs to be measured accurately.

<br>

The first step of the procedure is the simultaneous measurement of the actual distance and the corresponding voltage readings received from the sensor for several distances. Once the values have been measured, it is best to use a program like MATLAB/Python for data processing, where the measurements can be evaluated and further processed. The goal is to approximate the measured characteristics from millivolts to distance in [cm] as a non-linear function (a map) and determine the coefficients of the function that converts the signal from millivolts into a distance. The solution to this problem will be found via nummerical opitimazitaion. A list of hardware and a link to a file that is needed to perform the calibration can be found below:

>Hardware:
> - NUCLEO-F446RE board
> - IR sensor (check which one you have, the model name is on the side, it will determine the range of your measurement)
> - Mini USB cable
> - Additional wires to connect the sensor to the NUCLEO board
> - Paper tape
> - Length measure tape
>
> Software:
> - [Template MATLAB Evaluation of IR sensor Data](../templates/matlab/ir_sensor_eval.m)
> - [Template Python Evaluation of IR sensor Data](../templates/python/ir_sensor_eval.py)

#### Procedure

- Tape the paper tape to the flat surface from the edge of the chosen object and use a tape measure to mark the measurement points on the tape (e.g. 0 to 15 cm every 1 cm, then 17.5 to 30 cm every 2.5 cm and 35 to 75 cm every 5 cm, appropriate measurements may vary depending on the sensor type)

<p align="center">
    <img src="../images/IR_task.png" alt="IR task" width="650"/> </br>
    <i>Performing the exercise</i>
</p>

- To read the values measured by the sensor, it is essential to include a command that will be executed every iteration of the program. Therefore, this command is positioned within the ``while()`` within the ``if()`` statement which indicates that the command will start reading sensor values after starting the program execution with the **USER** button.

```cpp
// read analog input
ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
```

**NOTE:**
- Keep in mind that the signal is mapped from 0...3.3V to a range of 0.0f...1.0f. Consequently, the reading needs to be multiplied by 3.3, representing the maximum range of the sensor, and then by 1000 to convert the signal from volts to millivolts.

- To continuously receive printouts on the serial monitor, place the command within the ``while()`` loop, ensuring constant output regardless of the main task execution:

```cpp
// print to the serial terminal
printf("IR distance mV: %f \n", ir_distance_mV);
```

- To reset the variables to the initial values without restarting the program, add the following command in the ``else()`` statement, triggered by pressing the **USER** button while the program is running.

```cpp
// reset variables and objects
led1 = 0;
ir_distance_mV = 0.0f;
```

- Once the above commands are implemented, the next step is to compile and run the application.
- During the calibration process, position the sensor's edge at the marked points on the tape. The sensor should face the wall to measure the distance from. It's important to align the sensor beam parallel to the ground. Simultaneously, note the distance and the corresponding readout values displayed on the serial monitor after applying it to each designated point.
- After collecting the data points, input them into the MATLAB file [Template MATLAB Evaluation of IR sensor Data](../templates/matlab/ir_sensor_eval.m) under the respective variables dist_cm and dist_mV. This file aids in determining the coefficients for the optimal-fit curve. <b>To achieve accurate results, it's crucial to define a suitable range of values for the curve fitting. Check the name of the sensor and look for its range, only within this range the fitting will be accurate</b>. There is also a corresponding Python evaluation file under [Template Python Evaluation of IR sensor Data](../templates/python/ir_sensor_eval.py).
- Following this, proceed to create a function that converts the sensor readings into a physical length [cm]. While the function definition can be positioned at the end of the ***main.cpp*** file, it must be declared before the ``main()`` function to ensure successful compilation.

Function definition (at the end of the ***main.cpp*** file)

```cpp
float ir_sensor_compensation(float ir_distance_mV)
{
    // insert values that you got from the MATLAB file
    static const float a = 2.574e+04f;
    static const float b = -29.37f;

    // avoid division by zero by adding a small value to the denominator
    if (ir_distance_mV + b == 0.0f)
        ir_distance_mV -= 0.001f;

    return a / (ir_distance_mV + b);
}
```

After inserting the function, take a close look at how it is structured, are there mathematical operations in it, where exceptional values of variables, can cause the execution of forbidden mathematical operations?

Possible situation:

```cpp
(ir_distance_mV + b) == 0.0f
```

In the case of this function, there is the possibility of a situation where a division by zero is performed. This situation is unlikely but theoretically possible, so it is very important to carefully analyze the operations performed by the program. Dividing by zero can lead to a complete failure, which in the case of a simple robot does not necessarily end up as spectacular as in the case of a drone at an altitude of 30 meters.

Function declaration (at the beginning of the ***main.cpp*** file)

```cpp
// function declaration, definition at the end
float ir_sensor_compensation(float ir_distance_mV);
```

- To read the distance in centimeters, declare the variable that will handle this value in the same location where the variable to store the value in millivolts is declared.

```cpp
float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
float ir_distance_cm = 0.0f;
```

Following this, proceed to call the function for evaluation within the ``while()`` loop.

```cpp
ir_distance_cm = ir_sensor_compensation(ir_distance_mV);
```

- To reset the variables to the initial values without restarting the program, add the following command to the ``else()`` statement, triggered by pressing the **USER** button while the program is running.

```cpp
// reset variables and objects
led1 = 0;
ir_distance_mV = 0.0f;
ir_distance_cm = 0.0f;
```

- Finally, add the new variable to the printing command as the last step.

```cpp
// print to the serial terminal
printf("IR distance mV: %f IR distance cm: %f \n", ir_distance_mV, ir_distance_cm);
```

Below are the graphs showing the results from the calibration process.
<p align="center">
    <img src="../images/ir_sensor_eval.png" alt="IR sensor evaluation" width="850"/> </br>
    <i>IR sensor evaluation graph</i>
</p>

The first graph illustrates the non-linear relationship between the sensor's received voltage and its distance from an obstacle. In the second graph two curves are presented: the blue curve representing measured points and the green curve the optimal-fit function. The third graph shows how well the fitted function relates to its optimum (the closer to a linear function, the better).

**NOTES:**
- Keep in mind that the signal is mapped to a range of 0.0f to 1.0f. Consequently, the reading needs to be multiplied by 3.3, representing the maximum range of the sensor, and then by 1000 to convert the signal from volts to millivolts.

- After the calibration, using the sensor is straightforward however, the measured values need to be calibrated using the function acquired during the calibration process. This ensures the result is available as physical distance. Determining this function for each sensor individually is recommended.

## Enhanced Signal Quality

The sensor reading is relatively noisy, which can be improved by filtering the signal. The simplest way to filter the signal is to use a moving average filter. This is implicitly done in the class ``IRSensor``.
With the constructor

```cpp
IRSensor ir_sensor(PC_2);
```

you create an object that reads the sensor signal periodically at 200 Hz and you can obtain the filtered value by calling the method ``read()``. You can apply the calibration with the function

```cpp
ir_sensor.setCalibration(2.574e+04f, -29.37f);
```

To obtain the averaged value you can use

```cpp
float ir_distance_avg = ir_sensor.read();
```

It is important to note that before the ``setCalibration()`` the ``read()`` function returns averaged values in millivolts. After the calibration is set, the ``read()`` function returns the distance in centimeters (if calibrated properly).

Another option is to use the constructor

```cpp
IRSensor ir_sensor(PC_2, 2.574e+04f, -29.37f);
```

where you create an object and calibrate it in one line.

With the commands

```cpp
float ir_distance_mV = ir_sensor.readmV(); // sensor value in millivolts
float ir_distance_cm = ir_sensor.readcm(); // sensor value in centimeters (if calibrated)
```

you can read out the unfiltered values.

## Examples

- [Example Infrared Distance Sensor](../solutions/main_ir_sensor.cpp)
- [Example Infrared Distance Sensor using IRSensor class](../solutions/main_ir_sensor_class.cpp)
