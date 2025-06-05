#ifndef MY_DC_MOTOR_H_
#define MY_DC_MOTOR_H_

#include "Chirp.h"
#include "Encoder.h"
#include "Motor.h"
#include "RealTimeThread.h"

using namespace std::chrono;

class MyDCMotor : public RealTimeThread {
public:
    explicit MyDCMotor(PinName pwm_pin,
                       PinName enc_a_pin,
                       PinName enc_b_pin,
                       float counts_per_turn,
                       float voltage_max = 12.0f,
                       uint32_t period_us = 1000,
                       osPriority priority = osPriorityHigh1,
                       uint32_t stack_size = OS_STACK_SIZE);
    virtual ~MyDCMotor();

    float getVelocity() const { return m_encoder_signals.velocity; }
    float getRotation() const { return m_encoder_signals.rotations; }
    long getCounts() const { return m_encoder_signals.counts; }

    void setVoltage(float voltage);
    float getTargetVoltage() const { return m_target_voltage; }

    void chirpEnable();
    void chirpDisable();
    bool isChirpEnabled() const { return m_enable_chirp; };
    
    // disable functions from base class, changing period and priority
    // is not supported as this class is designed for a fixed period
    void setPeriod(uint32_t period_us) = delete;
    void setPriority(osPriority priority) = delete;

protected:
    void executeTask() override;
    
private:
    static constexpr float FCUT_HZ = 15.0f;
    static constexpr float D = 1.0f;
    static constexpr float CHIRP_F0_HZ = 0.1f;
    static constexpr float CHIRP_TIME_S = 10.0f;
    static constexpr float CHIRP_AMPLITUDE_V = 4.0f;
    static constexpr float CHIRP_OFFSET_V = 6.0f;

    float m_Ts;

    Encoder m_Encoder;
    Motor m_Motor;
    Chirp m_Chirp;

    Encoder::encoder_signals_t m_encoder_signals;

    float m_target_voltage = 0.0f;
    bool m_enable_chirp = false;
};

#endif /* MY_DC_MOTOR_H_ */
