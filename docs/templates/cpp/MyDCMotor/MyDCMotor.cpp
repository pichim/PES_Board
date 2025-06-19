#include "MyDCMotor.h"

MyDCMotor::MyDCMotor(PinName pwm_pin,
                     PinName enc_a_pin,
                     PinName enc_b_pin,
                     float gear_ratio,
                     float kn,
                     float voltage_max,
                     float counts_per_turn,
                     uint32_t period_us,
                     osPriority priority,
                     uint32_t stack_size) : RealTimeThread(period_us, priority, stack_size)
                                          , m_Ts(static_cast<float>(period_us) * 1.0e-6f)
                                          , m_Encoder(enc_a_pin, enc_b_pin, gear_ratio * counts_per_turn, ENCODER_FCUT_HZ, m_Ts)
                                          , m_Motor(pwm_pin, voltage_max)
                                          , m_kn(kn > 0.0f ? kn : 1.0f)
                                          , m_voltage_max(voltage_max > 0.0f ? voltage_max : 12.0f)
                                          , m_velocity_max(m_kn / 60.0f * m_voltage_max)
{
    // set integrator limits for the PI controller to the default value
    setVelocityCntrlIntegratorLimitsPercent(VOLTAGE_I_TERM_MAX_PERCENT);
    // enable thread by default
    enable();
}

MyDCMotor::~MyDCMotor()
{
    // ensure motor is stopped safely before thread cleanup
    m_Motor.setVoltage(0.0f);
    // the base class RealTimeThread destructor will handle thread cleanup
}

void MyDCMotor::setVoltage(float voltage)
{
    m_voltage = saturate(voltage, m_voltage_max);
}

void MyDCMotor::setVelocityCntrl(float kp, float ki)
{
    m_kp = kp;
    m_ki = ki;
}

void MyDCMotor::setVelocityCntrlIntegratorLimitsPercent(float percent_of_max)
{
    m_voltage_i_term_max = percent_of_max * 0.01f * m_voltage_max;
}

void MyDCMotor::setVelocity(float velocity)
{   
    m_velocity_setpoint = saturate(velocity, m_velocity_max);
}

void MyDCMotor::setMaxAcceleration(float acceleration)
{
    // ensure acceleration is positive and not zero
    m_acceleration_max = acceleration > 0.0f ? acceleration : ACCELERATION_MAX_DEFAULT;
}

void MyDCMotor::executeTask()
{
    // read encoder signals
    m_encoder_signals = m_Encoder.update();

    // update motor with current target voltage
    m_Motor.setVoltage(m_voltage);
}

float MyDCMotor::saturate(float value, float valueMax) const
{
    return (value > valueMax) ? valueMax : (value < -valueMax) ? -valueMax : value;
}
