#ifndef MY_DC_MOTOR_H_
#define MY_DC_MOTOR_H_

#include "Chirp.h"
#include "Encoder.h"
#include "Motor.h"
#include "RealTimeThread.h"
#include "SerialStream.h"

using namespace std::chrono;

class MyDCMotor : public RealTimeThread {
public:
    /**
     * @brief Construct a new MyDCMotor object.
     * @param pwm_pin The pin name for PWM control of the motor.
     * @param enc_a_pin The first pin name for the encoder.
     * @param enc_b_pin The second pin name for the encoder.
     * @param gear_ratio The gear ratio of the gear box.
     * @param kn The motor constant [rpm/V].
     * @param voltage_max The maximum voltage for the motor (default: 12.0f).
     * @param counts_per_turn The number of encoder counts per turn of the motor (default: 20.0f).
     * @param period_us The period in microseconds for the real-time thread (default: 500).
     * @param priority The thread priority (default: osPriorityHigh1).
     * @param stack_size The stack size in bytes for the thread (default: OS_STACK_SIZE).
     */
    explicit MyDCMotor(PinName pwm_pin,
                       PinName enc_a_pin,
                       PinName enc_b_pin,
                       float gear_ratio,
                       float kn,
                       float voltage_max = 12.0f,
                       float counts_per_turn = 20.0f,
                       uint32_t period_us = 500, // 2 kHz
                       osPriority priority = osPriorityHigh1,
                       uint32_t stack_size = OS_STACK_SIZE);

    virtual ~MyDCMotor();

    float getVelocity() const { return m_encoder_signals.velocity; }
    float getRotation() const { return m_encoder_signals.rotations; }
    long getCounts() const { return m_encoder_signals.counts; }

    void setVoltage(float voltage);
    float getVoltage() const { return m_voltage; }

    void setVelocityCntrl(float kp, float ki);
    void setVelocityCntrlIntegratorLimitsPercent(float percent_of_max);
    void setVelocity(float velocity);
    void setMaxAcceleration(float acceleration);

    float getVelocitySetpoint() const { return m_velocity_setpoint; }
    float getVelocityTarget() const { return m_velocity_target; }
    float getMaxAcceleration() const { return m_acceleration_max; }
    float getMaxVelocity() const { return m_velocity_max; }
    float getMotorConstant() const { return m_kn; }
    float getMaxVoltage() const { return m_voltage_max; }

protected:
    void executeTask() override;
    
private:
    static constexpr float ENCODER_FCUT_HZ = 15.0f;
    static constexpr float VOLTAGE_I_TERM_MAX_PERCENT = 30.0f; // 30% of max voltage by default
    static constexpr float ACCELERATION_MAX_DEFAULT = 1.0e6f;

    float m_Ts;

    Encoder m_Encoder;
    Motor m_Motor;

    Encoder::encoder_signals_t m_encoder_signals;

    float m_kn;
    float m_voltage_max;
    float m_velocity_max;
    float m_voltage_i_term_max;

    float m_voltage = 0.0f;
    float m_velocity_setpoint = 0.0f;
    float m_acceleration_max = ACCELERATION_MAX_DEFAULT;
    float m_velocity_target = 0.0f;
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_voltage_i_term = 0.0f;

    float saturate(float value, float valueMax) const;
};

#endif /* MY_DC_MOTOR_H_ */
