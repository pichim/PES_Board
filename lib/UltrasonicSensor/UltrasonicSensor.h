/**
 * @file UltrasonicSensor.h
 * @brief This file defines the UltrasonicSensor class, used for measuring distances using the Ultrasonic Ranger V2.0.
 *
 * The UltrasonicSensor class provides functionality to measure distances by emitting ultrasonic pulses 
 * and measuring the time taken for the echo to return. It encapsulates the details of interfacing with the
 * sensor hardware and offers a simple interface for obtaining distance measurements in centimeters.
 * Maximum measurment distance is approximately 2 meters (measured 198.1 cm) with a mearuement period of 12000
 * microseconds. If no new valid measurement is available, the read() function returns -1.0f.
 *
 * @dependencies
 * This class relies on the following components:
 * - DigitalInOut: For toggling the ultrasonic sensor's pin between output and input.
 * - InterruptIn: For detecting the echo signal.
 * - Timer: For measuring the time interval of the echo.
 * - Timeout: For managing pulse emission timing.
 * - ThreadFlag: For managing threading and synchronization.
 *
 * Usage:
 * To use the UltrasonicSensor class, create an instance with the pin connected to the sensor.
 * Measure the distance using read(). The class also provides an operator float() for direct reading.
 *
 * Example:
 * ```
 * UltrasonicSensor ultrasonicSensor(PIN_NAME);
 * float distance = ultrasonicSensor.read();
 * // or simply
 * float distance = ultrasonicSensor;
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#include "mbed.h"

#include "ThreadFlag.h"

// Time (mus), Distance (cm)
//     10000 ,        164
//     14000 ,        232.5
// ->  12105 ,        200
//     12000 ,        198.1  (measured)

class UltrasonicSensor
{
public:
    /**
     * @brief Construct a new UltrasonicSensor object.
     *
     * @param pin The pin name to which the ultrasonic sensor is connected.
     */
    explicit UltrasonicSensor(PinName pin);

    /**
     * @brief Destroy the UltrasonicSensor object.
     */
    virtual ~UltrasonicSensor();

    /**
     * @brief Read the measured distance in centimeters.
     *
     * Returns the measured distance in centimeters. If no new valid measurement is available, the function returns -1.0f.
     *
     * @return float The positive distance in centimeters, or -1.0f if no new valid measurement is available.
     */
    float read();

private:
    static constexpr int64_t PERIOD_MUS = 12000;

    DigitalInOut m_DigitalInOut;
    InterruptIn m_InteruptIn;
    Timer m_Timer;
    Timeout m_Timeout;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    float m_gain = 0.0170971;
    float m_offset = 1.7451288f;

    float m_us_distance_cm = 0.0f;
    bool m_is_new_value = false;

    void stopPulseAndWaitForRisingEdge();
    void startTimerAndWaitForFallingEdge();
    void measureTimeAndUpdateDistance();

    void threadTask();
    void sendThreadFlag();
};
#endif /* ULTRASONIC_SENSOR_H_ */
