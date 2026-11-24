/**
 * @file Stepper.h
 * @brief PES-Board 2026 STEP/DIR driver: TIM10/TIM11 hardware PWM + optional Motion ramp.
 *
 * The existing Stepper API remains available. Motion planner controls mirror DCMotor.
 * Motion is never called in an interrupt; STEP edges always come from timer hardware.
 * The application owns shared ENABLE (PC5) and RESET (PA5).
 */
#ifndef STEPPER_H_
#define STEPPER_H_

#include "mbed.h"
#include "stm32f4xx_hal.h"
#include "Motion.h"
#include <cstdint>
#include <limits>

class Stepper {
public:
    explicit Stepper(PinName step_pin, PinName dir_pin,
                     uint16_t step_per_rev = 200 * 16);
    ~Stepper();

    Stepper(const Stepper&) = delete;
    Stepper& operator=(const Stepper&) = delete;

    // Existing Stepper API (unchanged signatures and units).
    int getStepsSetpoint() const;
    int getSteps() const;
    float getRotation() const;               // revolutions, based on commanded STEP count
    float getVelocity() const;               // revolutions/s, programmed hardware STEP rate
    void setInternalVelocity(float velocity = 0.0f);
    void setInternalRotation(float rotations = 0.0f);
    void setRotation(float rotations, float velocity);
    void setRotation(float rotations);
    void setRotationRelative(float rotations, float velocity);
    void setRotationRelative(float rotations);
    void setVelocity(float velocity);
    void setSteps(int steps, float velocity);
    bool isRunning() const;

    // Same Motion-facing API and units as DCMotor (no change to Motion itself).
    void setMaxVelocity(float velocity);
    float getMaxVelocity() const;
    float getMaxPhysicalVelocity() const;     // 20 kHz / steps per revolution
    void setMaxAcceleration(float acceleration);
    float getMaxAcceleration() const;
    void enableMotionPlanner();
    void disableMotionPlanner();
    void setMotionPlannerVelocity(float velocity = 0.0f);
    void setMotionPlannerPosition(float position = 0.0f);
    float getVelocityTarget() const;
    float getVelocitySetpoint() const;
    float getRotationTarget() const;
    float getRotationSetpoint() const;

    static void onTim10Update();
    static void onTim11Update();

private:
    static constexpr uint32_t PULSE_US = 10;
    static constexpr uint32_t DIR_SETUP_US = 5;
    static constexpr uint32_t MAX_STEP_HZ = 20000;
    static constexpr int64_t PLANNER_PERIOD_US = 5000; // like DCMotor (5 ms)
    static constexpr uint32_t PLANNER_FLAG = 1u;
    static constexpr int STEPS_MIN = std::numeric_limits<int>::min();
    static constexpr int STEPS_MAX = std::numeric_limits<int>::max();

    enum class Mode : uint8_t { Position, Velocity };

    struct PulseConfig {
        uint32_t prescaler;
        uint32_t period;
        uint32_t compare;
        float actual_hz;
    };

    PwmOut m_Step;
    DigitalOut m_Dir;
    TIM_TypeDef* m_tim;
    IRQn_Type m_irq;
    uint32_t m_timer_clock_hz;
    float m_steps_per_rev;
    volatile int m_steps;
    volatile int m_steps_setpoint;
    volatile float m_velocity;
    float m_default_velocity;
    volatile bool m_running;
    int m_step_sign;

    // Planner data are protected by m_command_mutex (never used in ISR).
    mutable Mutex m_command_mutex;
    Motion m_Motion;
    bool m_enable_motion_planner;
    volatile bool m_planner_active;
    Mode m_mode;
    float m_velocity_max;
    float m_acceleration_max;
    float m_move_velocity_limit;
    float m_velocity_target;
    float m_velocity_setpoint;
    float m_rotation_target;
    float m_rotation_setpoint;

    Thread m_planner_thread;
    Ticker m_planner_ticker;
    volatile bool m_thread_alive;

    static Stepper* s_tim10;
    static Stepper* s_tim11;
    static uint32_t apb2TimerClockHz();

    int rotationsToSteps(float rotations) const;
    bool calculatePulseTrain(float velocity_abs, PulseConfig& config) const;
    void applyPulseTrain(const PulseConfig& config); // short IRQ-masked register writes
    void stopTimer();                               // caller masks IRQ or is in ISR
    void startAtConfig(int sign, const PulseConfig& config);
    void onTimerUpdate();
    void plannerSignal();                           // Ticker ISR: only signals worker
    void plannerTask();                             // normal RTOS thread
    void plannerTick(float dt);                     // normal RTOS thread, holds mutex
};

#endif // STEPPER_H_
