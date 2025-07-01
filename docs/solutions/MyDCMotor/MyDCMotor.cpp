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

    // // Be careful: the static objects and variables are shared across all instances of this class.
    // // They are intended for debugging purposes only, assuming a single instance exists.
    // // Do not use this in production code!

    // // static serial stream to send data over uart
    // static SerialStream serialStream(PB_10 /*tx*/, PC_5 /*rx*/);

    // // static timer to measure time elapsed since last call
    // static Timer logging_timer;
    // static microseconds time_previous_us{0};
    // logging_timer.start();
    // const microseconds time_us = logging_timer.elapsed_time();
    // const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
    // time_previous_us = time_us;

    // // static chirp signal
    // static const float f0 = 0.1f;                  // start frequency in Hz
    // static const float f1 = 0.99f / (2.0f * m_Ts); // end frequency in Hz, has to be less than Nyquist frequency
    // static const float t1 = 20.0f;                 // duration in seconds
    // static const float amplitude = 3.0f;
    // static Chirp chirp(f0, f1, t1, m_Ts);

    // // update chirp signal and send signal over serial stream
    // float chirp_exc = 0.0f;
    // if (serialStream.startByteReceived() && chirp.update()) {
    //     // update chirp excitation
    //     chirp_exc = amplitude * chirp.getExc();

    //     // send data over serial stream
    //     serialStream.write( dtime_us );                        //  0 delta time in us
    //     serialStream.write( (float)m_encoder_signals.counts ); //  1 encoder counts
    //     serialStream.write( m_encoder_signals.velocity );      //  2 encoder velocity in rotations per second
    //     serialStream.write( m_encoder_signals.rotations );     //  3 encoder rotations in turns
    //     serialStream.write( m_voltage + chirp_exc );           //  4 voltage in V
    //     serialStream.write( chirp.getSinarg() );               //  5 chirp sinarg in radians
    //     serialStream.send();
    // }

    // acceleration limiting: ramp m_velocity_target towards m_velocity_setpoint
    const float velocity_target_error = m_velocity_setpoint - m_velocity_target;
    const float max_velocity_change = m_acceleration_max * m_Ts;
    if (fabsf(velocity_target_error) <= max_velocity_change) {
        // can reach setpoint in this time step
        m_velocity_target = m_velocity_setpoint;
    } else {
        // need to limit acceleration
        if (velocity_target_error > 0.0f) {
            m_velocity_target += max_velocity_change;
        } else {
            m_velocity_target -= max_velocity_change;
        }
    }

    // // set the target velocity to the setpoint
    // m_velocity_target = m_velocity_setpoint;

    // apply the velocity PI controller
    const float velocity_error = m_velocity_target - m_encoder_signals.velocity;
    // calculate P controller term
    const float voltage_p_term = m_kp * velocity_error;
    // calculate I controller term
    m_voltage_i_term = saturate(m_voltage_i_term + m_ki * velocity_error * m_Ts, m_voltage_i_term_max);
    // // calculate total voltage from PI controller and constrain the voltage to the maximum voltage
    // float voltage = saturate(voltage_p_term + m_voltage_i_term, m_voltage_max);

    // add feedforward term and calculate total voltage from PI controller
    // and constrain the voltage to the maximum voltage
    const float voltage_f_term = (60.0f / m_kn) * m_velocity_target;
    float voltage = saturate(voltage_p_term + m_voltage_i_term + voltage_f_term, m_voltage_max);

    // update votage target
    m_voltage = voltage;

    // if (serialStream.startByteReceived()) {
    //     // send data over serial stream
    //     serialStream.write( dtime_us );                        //  0 delta time in us
    //     serialStream.write( (float)m_encoder_signals.counts ); //  1 encoder counts
    //     serialStream.write( m_encoder_signals.velocity );      //  2 encoder velocity in rotations per second
    //     serialStream.write( m_encoder_signals.rotations );     //  3 encoder rotations in turns
    //     serialStream.write( m_voltage);                        //  4 voltage in V
    //     serialStream.write( m_velocity_setpoint );             //  5 velocity setpoint in rotations per second
    //     serialStream.write( m_velocity_target );               //  6 velocity target in rotations per second
    //     serialStream.send();
    // }

    // update motor with current target voltage
    m_Motor.setVoltage(m_voltage);
}

float MyDCMotor::saturate(float value, float valueMax) const
{
    return (value > valueMax) ? valueMax : (value < -valueMax) ? -valueMax : value;
}
