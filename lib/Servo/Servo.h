/**
 * @file Servo.h
 * @brief This file defines the Servo class, which is used for controlling servo motors.
 *
 * The Servo class provides a range of functionalities for smooth and precise control of servo motors. 
 * It includes capabilities for calibrating the servo's pulse width range, setting motion profiles,
 * and enabling or disabling the servo. The class is designed to work with a variety of servo motors 
 * by allowing the user to specify the minimum and maximum pulse widths for calibration.
 *
 * @dependencies
 * This class relies on the following components:
 * - Motion: For handling motion profiles and acceleration control.
 * - ThreadFlag: For managing threading and synchronization.
 *
 * Usage:
 * To use the Servo class, create an instance with the pin connected to the servo motor.
 * Calibrate the servo using calibratePulseMinMax(), set motion profiles using setMaxAcceleration(),
 * and control the servo using enable(), disable(), and setPulseWidth() methods.
 * The state of the servo can be checked with isEnabled().
 *
 * Example:
 * ```
 * Servo servo(PIN_NAME);
 * servo.calibratePulseMinMax(0.0150f, 0.1150f)
 * servo.setMaxAcceleration(3.0f);
 * servo.enable();
 * servo.setPulseWidth(0.5f); // set servo to mid position
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "Motion.h"
#include "ThreadFlag.h"

/**
 * @brief Class for smooth control of a servo motor.
 *
 * This class provides functionalities to control servo motors with
 * features like calibration, setting motion profiles, and enabling/disabling the servo.
 */
class Servo
{
public:
    /**
     * @brief Construct a new Servo object.
     *
     * @param pin The pin name to which the servo is connected.
     */
    explicit Servo(PinName pin);

    /**
     * @brief Destroy the Servo object.
     */
    virtual ~Servo();

    /**
     * @brief Calibrate the minimum and maximum pulse widths for the servo.
     *
     * @param pulse_min The minimum pulse width.
     * @param pulse_max The maximum pulse width.
     */
    void calibratePulseMinMax(float pulse_min = 0.0f, float pulse_max = 1.0f);

    /**
     * @brief Set the motion profile velocity.
     *
     * @param velocity The velocity value for the motion profile.
     */
    void setMaxVelocity(float velocity = 1.0e6f); // 1.0e6f instead of infinity

    /**
     * @brief Set the motion profile acceleration.
     *
     * @param acceleration The acceleration value for the motion profile.
     */
    void setMaxAcceleration(float acceleration = 1.0e6f); // 1.0e6f instead of infinity

    /**
     * @brief Set the normalised pulse width.
     *
     * @param pulse The pulse width to be set, normalised.
     */
    void setPulseWidth(float pulse = 0.0f);

    /**
     * @brief Enable the servo with a specific pulse width.
     *
     * @param pulse The pulse width to set when enabling.
     */
    void enable(float pulse = 0.0f);

    /**
     * @brief Disable the servo.
     */
    void disable();

    /**
     * @brief Check if the servo is enabled.
     *
     * @return true If the servo is enabled.
     * @return false If the servo is disabled.
     */
    bool isEnabled() const;

private:
    static constexpr int64_t PERIOD_MUS = 20000;
    static constexpr float TS = 1.0e-6f * static_cast<float>(PERIOD_MUS);
    static constexpr float PWM_MIN = 0.01f;
    static constexpr float PWM_MAX = 0.99f;

    DigitalOut m_DigitalOut;
    Motion m_Motion;
    Timeout m_Timeout;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    bool m_enabled{false};
    float m_pulse{0.0f};
    float m_pulse_min{0.0f};
    float m_pulse_max{1.0f};

    float calculateNormalisedPulseWidth(float pulse);
    void threadTask();
    void enableDigitalOutput();
    void disableDigitalOutput();
    void sendThreadFlag();
    float constrainPulse(float pulse) const;
};

#endif /* SERVO_H_ */
