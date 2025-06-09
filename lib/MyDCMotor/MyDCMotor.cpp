#include "MyDCMotor.h"

MyDCMotor::MyDCMotor(PinName pwm_pin,
                     PinName enc_a_pin,
                     PinName enc_b_pin,
                     float counts_per_turn,
                     PinName tx,
                     PinName rx,
                     float voltage_max,
                     uint32_t period_us,
                     osPriority priority,
                     uint32_t stack_size) : RealTimeThread(period_us, priority, stack_size)
                                          , m_Ts(static_cast<float>(period_us) * 1.0e-6f)
                                          , m_Encoder(enc_a_pin, enc_b_pin, counts_per_turn, FCUT_HZ, D, m_Ts)
                                          , m_Motor(pwm_pin, voltage_max)
                                          , m_Chirp(CHIRP_F0_HZ, 0.99f / (2.0f * m_Ts), CHIRP_TIME_S, m_Ts)
                                          , m_SerialStream(SerialStream::NUM_OF_FLOATS_MAX, tx, rx)
{
    // constructor initializes both base class RealTimeThread and MyDCMotor
    m_encoder_signals = m_Encoder.update();
    // start timer
    m_Timer.start();
    // enable thread by default
    enable();
}

MyDCMotor::~MyDCMotor()
{
    // the base class RealTimeThread destructor will handle thread cleanup
    // but we should ensure motor is stopped
    m_Motor.setVoltage(0.0f);
}

void MyDCMotor::setVoltage(float voltage)
{
    m_target_voltage = voltage;
}


void MyDCMotor::chirpEnable(float amplitude)
{
    m_chirp_amplitude = amplitude;
    m_enable_chirp = true;
}

void MyDCMotor::chirpDisable()
{
    m_enable_chirp = false;
}

void MyDCMotor::executeTask()
{
    // measure delta time
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // read encoder signals
    m_encoder_signals = m_Encoder.update();

    // add chirp excitation to target voltage if enabled
    float chirp_exc = 0.0f;
    static bool do_reset_once = false;
    if (m_enable_chirp) {

        // set reset flag
        do_reset_once = true;

        if (m_SerialStream.startByteReceived() && m_Chirp.update()) {
            // update chirp excitation
            chirp_exc = m_chirp_amplitude * m_Chirp.getExc();

            // send data over serial stream
            m_SerialStream.write( dtime_us );
            m_SerialStream.write( (float)m_encoder_signals.counts );
            m_SerialStream.write( m_encoder_signals.velocity );
            m_SerialStream.write( m_encoder_signals.rotations );
            m_SerialStream.write( m_target_voltage + chirp_exc );
            m_SerialStream.write( m_Chirp.getSinarg() );
            m_SerialStream.send();
        }

    } else {
        if (do_reset_once) {
            do_reset_once = false;

            m_Chirp.reset();
            m_SerialStream.reset();
        }
    }

    // update motor with current target voltage
    m_Motor.setVoltage(m_target_voltage + chirp_exc);
}
