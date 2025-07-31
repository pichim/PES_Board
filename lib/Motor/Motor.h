#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_DO_USE_FAST_PWM true

#if MOTOR_DO_USE_FAST_PWM
    #include "FastPWM.h"
#else
    #include "mbed.h"
#endif

class Motor
{
public:
    explicit Motor(PinName pwm, float voltage_max = 12.0f);
    virtual ~Motor() = default;

    float setVoltage(float voltage);

private:
    // TODO: confirm this
    // it's ok to use the range (0.0f, 1.0f) since the driver runs in fast decay mode
    static constexpr float MOTOR_DUTY_CYCLE_MIN_VALUE = 0.0f;
    static constexpr float MOTOR_DUTY_CYCLE_MAX_VALUE = 1.0f;

#if MOTOR_DO_USE_FAST_PWM
    FastPWM m_PWM;
#else
    PwmOut m_PWM;
#endif
    float m_voltage_max;
};
#endif /* MOTOR_H_ */
