<!-- link list, last updated 03.01.2024 -->
[0]: https://www.digikey.ch/de/products/detail/seeed-technology-co-ltd/101020010/5482600?s=N4IgTCBcDaIOICUDyA1AogAgKoBkAqCAggMpIByAkgMIZFlxoIYpgB0ADCALoC%2BQA
[1]: https://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/

# Ultrasonic Sensor

An ultrasonic sensor (range finder / detector) is a type of sensor designed to measure the relative distance between the device and an object that reflects sound waves. This sensor operates by emitting ultrasonic pulses and measuring the time it takes for these pulses to bounce back as echoes. By knowing the speed of sound in air, it is possible to calculate the relative distance to the object based on the time elapsed between the emission of the ultrasonic pulse and the reception of its echo.

<p align="center">
    <img src="../images/grove_ultrasonic_ranger_v2.png" alt="Grove-Ultrasonic Ranger V2.0" width="400"/> </br>
    <i>Grove-Ultrasonic Ranger V2.0</i>
</p>

## Specifications

| Parameter           | Value           |
| ------------------- | --------------- |
| Operating voltage   | 3.2 ~ 5.2 V     |
| Measuring range     | 3 - ~200 cm     |
| Size                | 50 x 25 x 16 mm |
| Weight              | 13 g            |
| Measurement angle   | 15 °            |
| Working temperature | -10 ~ 60 °C     |

**NOTE:**
- The measurements showed a relatively high variation above 2 meters, therefore the measurement range is limited to 2 meters (with the current driver implementation).

## Links

- [DigiKey][0] </br>
- [Seeed Studio][1]

## Datasheets

- [Grove-Ultrasonic Ranger V2.0](../datasheets/Grove-Ultrasonic_Ranger_WiKi.pdf)

## Practical Tips

- Do not hot plug the Grove-Ultrasonic Ranger, otherwise it will damage the sensor.
- Before using the sensor for responsible tasks, make sure that the measurements that are read by the sensor are correct and sufficiently accurate (this might need calibration).

## Ultrasonic Sensor Example

The ``UltrasonicSensor`` class provides functionality to measure distances by emitting ultrasonic pulses
and measuring the time taken for the echo to return. It encapsulates the details of interfacing with the
sensor hardware and offers a simple interface for obtaining distance measurements in centimeters.
Maximum measurement distance is approximately 2 meters (measured 198.1 cm) with a measurement period of 12000
microseconds. If no new valid measurement is available, the ``read()`` function returns -1.0f.

To start working with the sensor, it is necessary to connect it correctly and create an object in the ***main.cpp*** file and assign a digital output.

### Connection to the PES Board

The ultrasonic sensor was tested with a 5V power supply and a single pin that transmits the signal. Therefore, it can use the same ports as the servos. You can use the following pins on the PES board:

```cpp
PB_D0
PB_D1
PB_D2
PB_D3
```

[PES Board Pinmap](../datasheets/pes_board_peripherals.pdf)

To establish the connection, use the cable displayed in the following image. It is compatible with the plug and socket on the sensor and with pins on the PES board. When connecting the sensor to the PES board, remember that the yellow wire transmits the bidirectional signal. So, make sure to align the blue plug with the PES board accordingly to ensure a proper connection.

<p align="center">
    <img src="../images/groove_cable_reworked.png" alt="Cables used to connect to the sensor" width="600"/> </br>
    <i>Cable used to connect to the sensor</i>
</p>

### Create Ultrasonic Sensor Object

In the given example, the sensor is plugged into pin **D3 (PB_D3)** on the PES board. Initially, it's essential to add the suitable driver to the ***main.cpp*** file and then create an ``UltrasonicSensor`` object inside ``main()`` function with the pin name passed as an argument along with the variable definition that will be used to store the sensor value.

```cpp
#include "UltrasonicSensor.h"
```

```cpp
// ultra sonic sensor
UltrasonicSensor us_sensor(PB_D3);
float us_distance_cm = 0.0f;
```

<!-- Additional information is greyed out -->
>Details about the driver
>
>By sending a signal to the sensor, we command it to send a pulse with a rising edge followed by a falling edge and a pulse width of 10 microseconds. The device will emit sound with 8 periods of 40 kHz frequency and try to detect the echo (via convolution or similar). The sensor will then send a pulse proportional to the measurement time, which will be detected by the microcontroller. The time between the rising and falling edge is measured and used to calculate the distance on the microcontroller. This process is repeated every 12000 microseconds.

### Read the Distance

The operation is straightforward since all processes are encapsulated within the class including a function that returns the distance in centimeters. This functionality is accessed through the following command:

```cpp
// read us sensor distance, non valid measurements will return -1.0f
us_distance_cm = us_sensor.read();
```

If no new valid measurement is available, the ``read()`` function returns -1.0f. This needs to be handled appropriately in the application.

As an example you can use the following code snippet

```cpp
// read us sensor distance, only valid measurements will update us_distance_cm
const float us_distance_cm_candidate = us_sensor.read();
if (us_distance_cm_candidate > 0.0f)
    us_distance_cm = us_distance_cm_candidate;
```

where you only execute the code within the ``if()`` statement if a valid measurement is available.

**NOTE:**
- Do not readout the sensor faster than every 12000 microseconds, otherwise the sensor will report -1.0f frequently.
- For highly accurate measurements, every sensor unit should be calibrated individually. This depends on your specifications and should be tested.
