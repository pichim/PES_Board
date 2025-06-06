#include "MyDCMotor.h"

MyDCMotor::MyDCMotor(PinName pwm_pin,
                     PinName enc_a_pin,
                     PinName enc_b_pin,
                     float counts_per_turn,
                     float voltage_max,
                     uint32_t period_us,
                     osPriority priority,
                     uint32_t stack_size) : RealTimeThread(period_us, priority, stack_size)
                                          , m_Ts(static_cast<float>(period_us) * 1.0e-6f)
                                          , m_Encoder(enc_a_pin, enc_b_pin, counts_per_turn, FCUT_HZ, D, m_Ts)
                                          , m_Motor(pwm_pin, voltage_max)
                                          , m_Chirp(CHIRP_F0_HZ, 0.99f / (2.0f * m_Ts), CHIRP_TIME_S, m_Ts)
{
    // constructor initializes both base class RealTimeThread and MyDCMotor
    m_encoder_signals = m_Encoder.update();
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


void MyDCMotor::chirpEnable()
{
    m_enable_chirp = true;
}

void MyDCMotor::chirpDisable()
{
    m_enable_chirp = false;
}

void MyDCMotor::executeTask()
{
    // read encoder signals
    m_encoder_signals = m_Encoder.update();

    // be aware that m_target_voltage is overwritten by chirp if enabled, so 
    // do not use setVoltage() if chirp is enabled
    if (m_enable_chirp) {
        if (m_Chirp.update()) {
            m_target_voltage = CHIRP_AMPLITUDE_V * m_Chirp.getExc() + CHIRP_OFFSET_V;
        } else {
            m_target_voltage = 0.0f; // stop motor after chirp
        }
    }

    // update motor with current target voltage
    m_Motor.setVoltage(m_target_voltage);
}
