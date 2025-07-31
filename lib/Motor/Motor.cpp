#include "Motor.h"

Motor::Motor(PinName pwm, float voltage_max) : m_PWM(pwm)
                                             , m_voltage_max(voltage_max)
{
#if !MOTOR_DO_USE_FAST_PWM
    m_PWM.period_us(50);
    m_PWM.write(0.5f);
#endif
    setVoltage(0.0f);
}

float Motor::setVoltage(float voltage)
{
    // calculate duty cycle
    float duty_cycle = 0.5f * (1.0f + voltage / m_voltage_max);

    // constrain duty cycle to the range [MOTOR_DUTY_CYCLE_MIN_VALUE, MOTOR_DUTY_CYCLE_MAX_VALUE]
    duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                 (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                  duty_cycle;
    
    // write the duty cycle to the PWM output
    m_PWM.write(duty_cycle);

    // return the actual voltage applied to the motor
    return (2.0f * duty_cycle - 1.0f) * m_voltage_max;
}
