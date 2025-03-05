<!-- link list, last updated 15.01.2023 -->
[0]: https://www.sparkfun.com/products/retired/13284
[1]: https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/
[2]: https://www.youtube.com/watch?v=whSw42XddsU

# IMU

An IMU (Inertial Measurement Unit) sensor is a compact electronic device that measures and reports motion-related data. It typically consists of a 3D gyroscope, a 3D accelerometer, and sometimes of a 3D magnetometer. Gyroscopes measure angular velocity, accelerometers measure linear acceleration, and magnetometers detect magnetic fields. IMU sensors are commonly used in robotics, drones, wearable devices and motion-capture systems to track orientation, angular velocity and linear acceleration. They provide essential data for navigation, stabilization, and motion analysis in various applications.

<p align="center">
    <img src="../images/imu.png" alt="IMU sensor" width="340"/> </br>
    <i>IMU breakout board</i>
</p>

## Technical Specifications

|                     | LSM9DS1 IMU sensor                               |
| ------------------- | ------------------------------------------------ |
| Channels            | 3D gyroscope, 3D acceleration, 3D magnetic field |
| Data output         | 16 bit                                           |
| Interface           | SPI/I2C                                          |
| Supply voltage      | 1.9 V to 3.6V                                    |
| Linear acceleration | ±2/±4/±8/±16 g                                   |
| Gauss magnetic      | ±4/±8/±12/±16                                    |
| Dps angular rate    | ±245/±500/±2000                                  |

## Links

- [SparkFun 9DoF IMU Breakout - LSM9DS1][0] <br>

## Datasheets 

- [LSM9DS1](../datasheets/lsm9ds1.pdf)

## Principle of Operation

An IMU with 9 degrees of freedom is a device that measures and provides information about a body's specific acceleration, how fast it's rotating, and the magnetic field. It does this by using a mix of accelerometers, gyroscopes, and magnetometers.
- Accelerometer - it detects acceleration or changes in velocity along the three axes of 3D space. The sensor operates by employing a small mass connected to fixed capacitors arranged using MEMS (Micro-Electro-Mechanical Systems) technology. When the sensor experiences acceleration or velocity changes in a specific direction, the mass inside the accelerometer moves relative to the fixed capacitors. This alters the distance between the mass and capacitors, thereby changing the capacitance of the system. An electronic circuit detects this capacitance change and converts it into a digital signal. </br>
- Gyroscope - is a sensor designed to gauge the rate of rotation or angular velocity of an object across the three axes of 3D space. Functioning through a minute vibrating component, often a MEMS resonator, it oscillates within a specific frequency range. As the gyro undergoes angular motion, the Coriolis force impacts the resonator, a process detected by sensors positioned around it. These sensors ascertain the rate and direction of angular motion, which is then translated into a digital signal by an electronic circuit. </br>
- Magnetometer - is a sensor crafted to gauge the intensity of the magnetic field surrounding it across the three axes of 3D space. Its mechanism involves a MEMS device housing a minuscule silicon structure designed to respond to magnetic fields. When the sensor encounters a magnetic field, the silicon structure undergoes movement, detected by capacitive or resistive elements within the MEMS device. This alteration in capacitance/resistance is then detected by an electronic circuit, which transforms it into a digital signal. </br>

### Sensors Fusion

To achieve optimal data accuracy, sensor fusion employs specialized algorithms to integrate measurements from multiple sensors of varying types. By merging the strengths of different sensors, this process capitalizes on their respective favorable characteristics, culminating in the attainment of the most precise measurements possible.

A good example to understand the idea of sensor fusion is the measurement of the roll angle using an accelerometer and a gyroscope. As written above, the accelerometer measures the acceleration in each of the three axes and the gyroscope determines the rotational speed around the three axes.
The measurements from the gyroscope and the measurements from the accelerometer can be used separately to estimate the roll angle. However, such an estimation will not be accurate due to the disadvantages of the single sensor measurement rules. The gyroscope is an accurate sensor in the short term, because in the long term, there is an accumulation of error which ultimately leads to an incorrect angle determination. The accelerometer, on the other hand, is a useful tool in the long term, as it measures the acceleration of the earth, which is constant and in a constant direction. It should be remembered that it also measures the acceleration of the object to which it is attached, so in the case of random movement of an object, distortions appear, causing an error in determining the angle. 

Therefore, the two sensors are fused, which combines the precise operation of the gyroscope in the short term and the accelerometer in the long term. This fusion is made with the help of different filters, the simplest of which is a complementary filter, which uses a low pass filter for measurements from an accelerometer and a high pass filter for measurements from a gyroscope, then by manipulating the coefficient you can change the confidence ratio to a particular sensor, remembering that the sum of the confidence factors is always 1.

<p align="center">
    <img src="../images/complementary_filter.png" alt="Complementary filter" width="500"/> </br>
    <i>Complementary filter</i>
</p>

The link to the video, which explains the principle of fusion of these two sensors in a very accurate and accessible way, can be found [here][2]

The ``IMU`` class is using a Mahony filter which is used to accurately estimate the orientation in space by combining data from accelerometer and gyroscope sensors. The Mahony filter can also integrate magnetometer data to further refine orientation estimation by providing a reference to the Earth's magnetic field. This allows for absolute orientation determination relative to magnetic north, enhancing the overall accuracy of the orientation estimation process.

### Magnetometer

In order to improve the accuracy of position determination and to enable determination of yaw, a magnetometer is used. This sensor measures the strength and direction of a magnetic field and in the context of sensor fusion plays a crucial role in providing a reference to the Earth's magnetic field, aiding in orientation estimation alongside accelerometers and gyroscopes.

However, integrating magnetometer data into sensor fusion algorithms presents several challenges, primarily stemming from the need for accurate calibration. Magnetometers are highly sensitive instruments that can be influenced by various sources of magnetic interference, including nearby metallic objects, electrical equipment, and even the Earth's own magnetic anomalies. These external factors can distort the measured magnetic field, leading to inaccuracies in orientation estimation.

**NOTE:**
- IMU by default is not using magnetometer for position evaluation, to enable it is necessary to change defined variable to true in ``IMU.h`` file:

```
#define IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE true
```

- MATLAB file that is used to help with magnetometer calibration can be found [here](../dev/dev_imu/99_fcn_bib/MgnCalibration.m)

## Practical Tips

- Typically, the IMU on the PES board should be mounted in such a way that the coordinate system is arranged as shown in the following illustration. However, it should be remembered that it may be in a different position on the board, so you should be sure of the direction of the axes in the coordinate system before continuing to use the data, and the most convenient way to do this is to print the acceleration indications into the console and then set the board in three positions and think about the position of the coordinate system.

<p align="center">
    <img src="../images/imu_axis_pes.PNG" alt="IMU coordinate system" width="600"/> </br>
    <i>IMU coordinate system</i>
</p>

## IMU Driver

The ``IMU`` class is a tool used to process the results through the fusion of three sensors included in the IMU and calculations used to obtain information about the position of the board in space, as well as about the movement of the board.
The following data can be obtained directly from the driver:
- Gyroscope - values in three XYZ axes
- Accelerometer - values in three XYZ axes
- Magnetometer - values in three XYZ axes
- Board orientation expressed in quaterions obtained by means of the fusion of sesors with the use of a Mahony filter
- Board orientation expressed in Euler angles in the Roll Pitch Yaw convention, seet the visualization [here][1]
- Tilt angle

To start working with the IMU, it is necessary to create an object in the ***main.cpp*** file and assign the correct pins. </br>

There is no need to plug anything as IMU is an integral part of the PES board. Pins that are used to communicate with the IMU:

```
// imu
#define PB_IMU_SDA PC_9
#define PB_IMU_SCL PA_8
```

### Create IMU Object

Add the IMU driver ``IMU.h`` to the top of the ***main.cpp*** file:

```
#include "IMU.h"
```

To be able to start to use the ``IMU`` driver, the initial step is to create the IMU object and specify the pins to which the hardware is connected in the ``main()`` scope.

As mentioned the IMU is using two pins to communicate. Next step is to create an object of the class that will serve as a store of data, sorted in an appropriate way and second object with the associated pins passed as an argument to receive measurments from IMU:

```
// imu
ImuData imu_data;
IMU imu(PB_IMU_SDA, PB_IMU_SCL);   
```

### Read measurments

Once the objects have been declared, it is possible to read data from the sensor. As mentioned, this data is processed inside the class with the appropriate filters and in addition to reading the sensor values themselves, the orientation of the board in space is calculated and expressed in quaternions and Euler angles. As mentioned, the data is collected in a custom data container of the type `ImuData`.

```
// read imu data
imu_data = imu.getImuData();
```

The `ImuData` object is a data structure that is updated as the `IMU` class thread runs, so an object has been created above to collect and access this data. Use the following commands to access the data, bearing in mind that axis-dependent data should be indexed as follows:

- X -> 0
- Y -> 1
- Z -> 2

```
// acceleration around three axis
float acc_X_axis = imu_data.acc(0);
float acc_Y_axis = imu_data.acc(1); 
float acc_Z_axis = imu_data.acc(2);
```

For reading the coefficients of the quaternion, the following commands can be used:

```
// quaterions coefficients
float quatW = imu_data.quat.w();
float quatX = imu_data.quat.x();
float quatY = imu_data.quat.y();
float quatZ = imu_data.quat.z();
```

When reading the angles in the Eulerian convention, one assumes:

- Roll -> 0
- Pitch -> 1
- Yaw -> 2

```
// euler angles
float roll = imu_data.rpy(0);
float pitch = imu_data.rpy(1);
float yaw = imu_data.rpy(2);
```

## Gimbal example

In order to demonstrate how the IMU works, a demonstration example has been prepared in the form of a self-leveling gimbal requiring printing a few parts and assembling the appropriate servos. <br>

<p align="center">
    <img src="../images/gimbal.PNG" alt="gimbal" width="500"/> </br>
    <i>Gimbal</i>
</p>

Prepared parts in `.stl` extension for printing can be found in the folder (docs/cad/gimbal) additionally step files are also placed there in case you wish to modify them. The parts are prepared for two Modelcraft RS2 MG/BB servos.

**NOTE:**
- Remember to calibrate the servos before using them for proper operation of the gimbal.
- The servo for the roll movement `PB_D0` is attached to the first joint (grey)
- The servo for the pitch movement `PB_D1` is attached to the second joint (red)

The gimbal allows stabilization in two axes. Assuming that the horizontal flat surface is defined by the two axes X and Y, in the event of a deflection of the microcontroller in both of these axes, the servos are to compensate for these deflections and cause the mount to which the camera can be attached to be in a perpendicular position with respect to the plane.

### Software

The ***main.cpp*** file has the standard form known from the base file. Add the IMU ``IMU.h`` and  servo ``Servo.h`` driver to the top of the ***main.cpp*** file:

```
#include "IMU.h"
#include "Servo.h"
```

In order to properly program the gimbal, it is necessary to define the ``Servo`` object responsible for roll and the servo responsible for pitch but also the IMU along with the object for collecting data.

```
// servo
Servo servo_roll(PB_D0);
Servo servo_pitch(PB_D1);

// imu
ImuData imu_data;
IMU imu(PB_IMU_SDA, PB_IMU_SCL);    
Eigen::Vector2f rp;
```

The servos must be calibrated before  the use and the appropriate values must be entered into the calibration functions. In addition, angle limits of the servos and the values of the coefficients of the linear functions that allow the signal to be mapped as an angle to the PWM pulse width are also defined.

```
// minimal pulse width and maximal pulse width obtained from the servo calibration process
// modelcraft RS2 MG/BB
float servo_ang_min = 0.0325f;
float servo_ang_max = 0.1250f;

// servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
// servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

// angle limits of the servos
const float angle_range_min = -M_PI/2.0f;
const float angle_range_max = M_PI/2.0f;    

// angle to pulse width coefficients
const float normalised_angle_gain = 1.0f / M_PI;
const float normalised_angle_offset = 0.5f;
```

Next, the variables that will feed the function are defined along with the base position of servos:

```
// pulse width
static float roll_servo_width = 0.5f;
static float pitch_servo_width = 0.5f;

servo_roll.setPulseWidth(roll_servo_width);
servo_pitch.setPulseWidth(pitch_servo_width);
```

To activate the servo, use the following command. Place this command to enable the servo after initiating the program execution with the **USER** button:

```
// enable the servos
if (!servo_roll.isEnabled())
    servo_roll.enable();
if (!servo_pitch.isEnabled())
    servo_pitch.enable();
```

The next step is to place functions that read data from the IMU, perform the appropriate transformation, and map the angles to a PWM signal contained in the range from 0 to 1.

```
// read imu data
imu_data = imu.getImuData();

// roll angle
rp(0) = atan2f(imu_data.quat.x() + imu_data.quat.z(), imu_data.quat.w() - imu_data.quat.y()) - atan2f(imu_data.quat.z() - imu_data.quat.x(), imu_data.quat.y() + imu_data.quat.w());
// pitch angle
rp(1) = acosf((imu_data.quat.w() - imu_data.quat.y()) * (imu_data.quat.w() - imu_data.quat.y()) + (imu_data.quat.x() + imu_data.quat.z()) * (imu_data.quat.x() + imu_data.quat.z()) - 1.0f) - M_PI / 2.0f;
```

The next key element is a command that allows you to control the servo according to the angle of tilt, it is important this command is only enforced if the angle is within a predefined range

```
// map to servo commands
roll_servo_width = -normalised_angle_gain * rp(0) + normalised_angle_offset;
pitch_servo_width = normalised_angle_gain * rp(1) + normalised_angle_offset;
if (rp(0) < angle_range_max && rp(0) > angle_range_min)
    servo_roll.setPulseWidth(roll_servo_width);
if (rp(0) < angle_range_max && rp(0) > angle_range_min)
    servo_pitch.setPulseWidth(pitch_servo_width);
```

Lastly add the following command to the ``else()`` statement, the purpose is to set servo initial position. This is triggered by pressing the **USER** button while the main task is running (second time you press the button).

```
roll_servo_width = 0.5f;
pitch_servo_width = 0.5f;
servo_roll.setPulseWidth(roll_servo_width);
servo_pitch.setPulseWidth(pitch_servo_width);
```

The last thing is the command that allows you to read the width of the roll and pitch pulse.

```
        // print to the serial terminal
        printf("%f, %f \n", roll_servo_width, pitch_servo_width);
```

The full working solution can be found in [Gimbal](../solutions/main_gimbal.cpp)