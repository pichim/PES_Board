#include "Stepper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

Stepper* Stepper::s_tim10 = nullptr;
Stepper* Stepper::s_tim11 = nullptr;

extern "C" void PES_Stepper_TIM10_IRQHandler() { Stepper::onTim10Update(); }
extern "C" void PES_Stepper_TIM11_IRQHandler() { Stepper::onTim11Update(); }

uint32_t Stepper::apb2TimerClockHz()
{
    const uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    const uint32_t code = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    const uint32_t apb_div = (code < 4u) ? 1u : (1u << (code - 3u));
#if defined(RCC_DCKCFGR_TIMPRE)
    if (RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE)
        return apb_div <= 4u ? HAL_RCC_GetHCLKFreq() : 4u * pclk;
#endif
    return apb_div == 1u ? pclk : 2u * pclk;
}

Stepper::Stepper(PinName step_pin, PinName dir_pin, uint16_t step_per_rev)
    : m_Step(step_pin)
    , m_Dir(dir_pin, 0)
    , m_tim(nullptr)
    , m_irq(TIM1_UP_TIM10_IRQn)
    , m_timer_clock_hz(apb2TimerClockHz())
    , m_steps_per_rev(static_cast<float>(step_per_rev))
    , m_steps(0)
    , m_steps_setpoint(0)
    , m_velocity(0.0f)
    , m_default_velocity(1.0f)
    , m_running(false)
    , m_step_sign(1)
    , m_enable_motion_planner(false)
    , m_planner_active(false)
    , m_mode(Mode::Position)
    , m_velocity_max(static_cast<float>(MAX_STEP_HZ) / step_per_rev)
    , m_acceleration_max(1.0f)
    , m_move_velocity_limit(1.0f)
    , m_velocity_target(0.0f)
    , m_velocity_setpoint(0.0f)
    , m_rotation_target(0.0f)
    , m_rotation_setpoint(0.0f)
    , m_planner_thread(osPriorityNormal)
    , m_thread_alive(true)
{
    MBED_ASSERT(step_per_rev != 0);

    if (step_pin == PB_8_ALT1) {
        m_tim = TIM10;
        m_irq = TIM1_UP_TIM10_IRQn;
        MBED_ASSERT(s_tim10 == nullptr);
        s_tim10 = this;
    } else if (step_pin == PB_9_ALT1) {
        m_tim = TIM11;
        m_irq = TIM1_TRG_COM_TIM11_IRQn;
        MBED_ASSERT(s_tim11 == nullptr);
        s_tim11 = this;
    } else {
        MBED_ASSERT(false);
    }

    m_Motion.setPosition(0.0);
    m_Motion.setVelocity(0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    m_Motion.setProfileAcceleration(m_acceleration_max);
    m_Motion.setProfileDeceleration(m_acceleration_max);

    // PwmOut supplies Mbed's AF mux; the timer is otherwise exclusive to this object.
    m_Step.write(0.0f);
    m_tim->DIER &= ~TIM_DIER_UIE;
    m_tim->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_OPM);
    m_tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
    // PWM mode 2, 16-bit timer, ARR and CCR preloaded for glitch-free live retuning.
    m_tim->CCMR1 = (m_tim->CCMR1 & ~TIM_CCMR1_OC1M)
                 | TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE;
    m_tim->CR1 |= TIM_CR1_ARPE;
    m_tim->CNT = 0;
    m_tim->ARR = 999;
    m_tim->CCR1 = 1000; // PWM2 remains LOW when CCR > ARR.
    m_tim->EGR = TIM_EGR_UG;
    m_tim->SR &= ~TIM_SR_UIF;
    m_tim->CCER |= TIM_CCER_CC1E;

    const uintptr_t handler = (m_tim == TIM10)
        ? reinterpret_cast<uintptr_t>(&PES_Stepper_TIM10_IRQHandler)
        : reinterpret_cast<uintptr_t>(&PES_Stepper_TIM11_IRQHandler);
    NVIC_SetVector(m_irq, static_cast<uint32_t>(handler));
    NVIC_SetPriority(m_irq, 7); // unchanged from the tested hardware implementation
    NVIC_ClearPendingIRQ(m_irq);
    NVIC_EnableIRQ(m_irq);

    // One dormant worker per Stepper; Ticker is only enabled with Motion planner.
    m_planner_thread.start(callback(this, &Stepper::plannerTask));
}

Stepper::~Stepper()
{
    m_planner_ticker.detach();
    m_thread_alive = false;
    m_planner_thread.flags_set(PLANNER_FLAG);
    m_planner_thread.join();
    {
        CriticalSectionLock lock;
        stopTimer();
        if (m_tim == TIM10) s_tim10 = nullptr;
        if (m_tim == TIM11) s_tim11 = nullptr;
    }
    NVIC_DisableIRQ(m_irq);
}

int Stepper::getStepsSetpoint() const
{
    CriticalSectionLock lock;
    return m_steps_setpoint;
}

int Stepper::getSteps() const
{
    CriticalSectionLock lock;
    return m_steps;
}

float Stepper::getRotation() const
{
    CriticalSectionLock lock;
    return static_cast<float>(m_steps) / m_steps_per_rev;
}

float Stepper::getVelocity() const
{
    CriticalSectionLock lock;
    return m_velocity;
}

bool Stepper::isRunning() const
{
    CriticalSectionLock lock;
    return m_running || m_planner_active;
}

void Stepper::setInternalVelocity(float velocity)
{
    if (!std::isfinite(velocity)) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_default_velocity = std::fabs(velocity);
}

void Stepper::setInternalRotation(float rotations)
{
    if (!std::isfinite(rotations)) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    {
        CriticalSectionLock irq_lock;
        stopTimer();
        m_steps = rotationsToSteps(rotations);
        m_steps_setpoint = m_steps;
        m_planner_active = false;
    }
    const float position = static_cast<float>(m_steps) / m_steps_per_rev;
    m_Motion.setPosition(position);
    m_Motion.setVelocity(0.0f);
    m_rotation_target = position;
    m_rotation_setpoint = position;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
}

int Stepper::rotationsToSteps(float rotations) const
{
    if (!std::isfinite(rotations))
        return rotations > 0 ? STEPS_MAX : (rotations < 0 ? STEPS_MIN : 0);
    const double steps = static_cast<double>(rotations) * m_steps_per_rev;
    if (steps >= STEPS_MAX) return STEPS_MAX;
    if (steps <= STEPS_MIN) return STEPS_MIN;
    return static_cast<int>(steps >= 0 ? steps + 0.5 : steps - 0.5);
}

void Stepper::setRotation(float rotations, float velocity)
{
    setSteps(rotationsToSteps(rotations), velocity);
}

void Stepper::setRotation(float rotations)
{
    float speed;
    {
        std::lock_guard<Mutex> lock(m_command_mutex);
        speed = m_default_velocity;
    }
    setRotation(rotations, speed);
}

void Stepper::setRotationRelative(float rotations, float velocity)
{
    const int delta = rotationsToSteps(rotations);
    int target;
    {
        CriticalSectionLock irq_lock;
        const int64_t sum = static_cast<int64_t>(m_steps) + delta;
        target = sum >= STEPS_MAX ? STEPS_MAX
               : sum <= STEPS_MIN ? STEPS_MIN : static_cast<int>(sum);
    }
    setSteps(target, velocity);
}

void Stepper::setRotationRelative(float rotations)
{
    float speed;
    {
        std::lock_guard<Mutex> lock(m_command_mutex);
        speed = m_default_velocity;
    }
    setRotationRelative(rotations, speed);
}

// Motion calculations and frequency division are deliberately done OUTSIDE
// CriticalSectionLock. At 20 kHz an extended interrupt blackout can create
// an extra physical STEP edge before the timer ISR is allowed to stop PWM.
bool Stepper::calculatePulseTrain(float velocity_abs, PulseConfig& config) const
{
    const double step_hz = static_cast<double>(velocity_abs) * m_steps_per_rev;
    if (!std::isfinite(step_hz) || step_hz <= 0.0 || step_hz > MAX_STEP_HZ)
        return false;

    const double divider = std::ceil(m_timer_clock_hz / (step_hz * 65534.0));
    if (divider > 65536.0) return false;
    const uint32_t psc = divider < 1.0 ? 1u : static_cast<uint32_t>(divider);
    uint32_t period = static_cast<uint32_t>(std::llround(
        m_timer_clock_hz / (static_cast<double>(psc) * step_hz)));
    if (period > 65534u) period = 65534u;

    const uint32_t high = static_cast<uint32_t>(std::llround(
        m_timer_clock_hz * (PULSE_US * 1.0e-6) / psc));
    const uint32_t pulse = std::max<uint32_t>(1u, high);
    if (period < pulse + 2u) return false;

    config.prescaler = psc - 1u;
    config.period = period - 1u;
    config.compare = period - pulse;
    config.actual_hz = static_cast<float>(
        m_timer_clock_hz / (static_cast<double>(psc) * period));
    return true;
}

void Stepper::applyPulseTrain(const PulseConfig& config)
{
    // In running mode PSC, ARR and CCR use preload registers and transfer
    // at the next natural timer update. Never force UG during live retuning.
    m_tim->PSC = config.prescaler;
    m_tim->ARR = config.period;
    m_tim->CCR1 = config.compare;
    if (!m_running) {
        m_tim->CNT = 0;
        m_tim->EGR = TIM_EGR_UG;
        m_tim->SR &= ~TIM_SR_UIF;
    }
    m_velocity = m_step_sign * config.actual_hz / m_steps_per_rev;
}

void Stepper::stopTimer()
{
    m_tim->DIER &= ~TIM_DIER_UIE;
    m_tim->CR1 &= ~TIM_CR1_CEN;
    m_tim->CNT = 0;
    m_tim->CCR1 = m_tim->ARR + 1u;
    m_tim->EGR = TIM_EGR_UG; // flush CCR preload to LOW while stopped
    m_tim->SR &= ~TIM_SR_UIF;
    m_velocity = 0.0f;
    m_running = false;
}

void Stepper::startAtConfig(int sign, const PulseConfig& config)
{
    if (m_running && sign != m_step_sign) stopTimer();
    if (!m_running) {
        m_step_sign = sign;
        m_Dir = sign > 0 ? 1 : 0;
        wait_us(DIR_SETUP_US);
    }
    applyPulseTrain(config);
    if (!m_running) {
        m_running = true;
        NVIC_ClearPendingIRQ(m_irq);
        m_tim->DIER |= TIM_DIER_UIE;
        m_tim->CR1 |= TIM_CR1_CEN;
    }
}

void Stepper::setSteps(int steps, float velocity)
{
    if (!std::isfinite(velocity)) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_mode = Mode::Position;
    m_velocity_target = 0.0f;
    m_default_velocity = std::fabs(velocity);
    m_move_velocity_limit = std::min(std::fabs(velocity), m_velocity_max);
    m_rotation_target = static_cast<float>(steps) / m_steps_per_rev;

    {
        CriticalSectionLock irq_lock;
        m_steps_setpoint = steps;
        if (steps == m_steps || velocity == 0.0f) {
            stopTimer();
            m_planner_active = false;
            m_steps_setpoint = m_steps;
            m_velocity_setpoint = 0.0f;
            return;
        }
    }

    if (m_enable_motion_planner) {
        if (!m_planner_active) {
            const float pos = static_cast<float>(m_steps) / m_steps_per_rev;
            m_Motion.setPosition(pos);
            m_Motion.setVelocity(0.0f);
        }
        m_Motion.setProfileVelocity(m_move_velocity_limit);
        m_planner_active = true; // isRunning() true even during first ramp tick
        return;
    }

    // Legacy mode: immediate constant-speed hardware pulse train.
    PulseConfig config{};
    const bool valid = calculatePulseTrain(std::fabs(velocity), config);
    {
        CriticalSectionLock irq_lock;
        stopTimer();
        if (!valid) {
            m_steps_setpoint = m_steps;
            return;
        }
        m_step_sign = steps > m_steps ? +1 : -1;
        m_Dir = m_step_sign > 0 ? 1 : 0;
        wait_us(DIR_SETUP_US);
        applyPulseTrain(config);
        m_running = true;
        NVIC_ClearPendingIRQ(m_irq);
        m_tim->DIER |= TIM_DIER_UIE;
        m_tim->CR1 |= TIM_CR1_CEN;
        m_velocity_setpoint = m_velocity;
    }
}

void Stepper::setVelocity(float velocity)
{
    if (!std::isfinite(velocity)) return;
    {
        std::lock_guard<Mutex> lock(m_command_mutex);
        if (m_enable_motion_planner) {
            m_mode = Mode::Velocity;
            m_velocity_target = velocity;
            m_Motion.setProfileVelocity(m_velocity_max);
            // Keep finite position-stop logic out of the deceleration path.
            {
                CriticalSectionLock irq_lock;
                if (velocity != 0.0f)
                    m_steps_setpoint = velocity > 0 ? STEPS_MAX : STEPS_MIN;
                else
                    m_steps_setpoint = m_step_sign > 0 ? STEPS_MAX : STEPS_MIN;
                m_planner_active = true;
            }
            return;
        }
    }
    if (velocity == 0.0f) {
        std::lock_guard<Mutex> lock(m_command_mutex);
        CriticalSectionLock irq_lock;
        stopTimer();
        m_steps_setpoint = m_steps;
        m_velocity_target = 0.0f;
        m_velocity_setpoint = 0.0f;
        return;
    }
    setSteps(velocity > 0.0f ? STEPS_MAX : STEPS_MIN, velocity);
}

// Motion-compatible configuration (same interface as DCMotor).
void Stepper::setMaxVelocity(float velocity)
{
    if (!std::isfinite(velocity) || velocity <= 0.0f) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_velocity_max = std::min(velocity, getMaxPhysicalVelocity());
    m_Motion.setProfileVelocity(m_mode == Mode::Position
        ? std::min(m_move_velocity_limit, m_velocity_max) : m_velocity_max);
}

float Stepper::getMaxVelocity() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_velocity_max;
}

float Stepper::getMaxPhysicalVelocity() const
{
    return static_cast<float>(MAX_STEP_HZ) / m_steps_per_rev;
}

void Stepper::setMaxAcceleration(float acceleration)
{
    if (!std::isfinite(acceleration) || acceleration <= 0.0f) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_acceleration_max = acceleration;
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

float Stepper::getMaxAcceleration() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_acceleration_max;
}

void Stepper::enableMotionPlanner()
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    if (m_enable_motion_planner) return;
    {
        CriticalSectionLock irq_lock;
        stopTimer();
        m_steps_setpoint = m_steps;
        m_planner_active = false;
    }
    const float pos = static_cast<float>(m_steps) / m_steps_per_rev;
    m_Motion.set(pos, 0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    m_Motion.setProfileAcceleration(m_acceleration_max);
    m_Motion.setProfileDeceleration(m_acceleration_max);
    m_rotation_target = pos;
    m_rotation_setpoint = pos;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_enable_motion_planner = true;
    m_planner_ticker.attach(callback(this, &Stepper::plannerSignal),
                            std::chrono::microseconds{PLANNER_PERIOD_US});
}

void Stepper::disableMotionPlanner()
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    if (!m_enable_motion_planner) return;
    m_planner_ticker.detach();
    {
        CriticalSectionLock irq_lock;
        stopTimer();
        m_steps_setpoint = m_steps;
        m_planner_active = false;
    }
    const float pos = static_cast<float>(m_steps) / m_steps_per_rev;
    m_Motion.set(pos, 0.0f);
    m_rotation_target = pos;
    m_rotation_setpoint = pos;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_enable_motion_planner = false;
}

void Stepper::setMotionPlannerVelocity(float velocity)
{
    if (!std::isfinite(velocity)) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_Motion.setVelocity(velocity);
}

void Stepper::setMotionPlannerPosition(float position)
{
    if (!std::isfinite(position)) return;
    std::lock_guard<Mutex> lock(m_command_mutex);
    m_Motion.setPosition(position);
}

float Stepper::getVelocityTarget() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_velocity_target;
}

float Stepper::getVelocitySetpoint() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_velocity_setpoint;
}

float Stepper::getRotationTarget() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_rotation_target;
}

float Stepper::getRotationSetpoint() const
{
    std::lock_guard<Mutex> lock(m_command_mutex);
    return m_rotation_setpoint;
}

void Stepper::plannerSignal()
{
    m_planner_thread.flags_set(PLANNER_FLAG); // ISR-safe, no Motion calculations here
}

void Stepper::plannerTask()
{
    Timer tick_timer;
    tick_timer.start();
    while (true) {
        ThisThread::flags_wait_any(PLANNER_FLAG);
        if (!m_thread_alive) return;
        const float dt = std::max(0.001f, std::min(0.025f,
            tick_timer.elapsed_time().count() * 1.0e-6f));
        tick_timer.reset();
        std::lock_guard<Mutex> lock(m_command_mutex);
        if (m_enable_motion_planner && m_planner_active)
            plannerTick(dt);
    }
}

void Stepper::plannerTick(float dt)
{
    // Motion is used unchanged, exactly as in DCMotor. Here it is an open-loop
    // trajectory (there is no encoder and hence no measured position feedback).
    const int actual_steps = m_steps;
    float requested = 0.0f;

    if (m_mode == Mode::Position) {
        m_Motion.incrementToPosition(m_rotation_target, dt);
        requested = m_Motion.getVelocity();
        m_rotation_setpoint = static_cast<float>(m_Motion.getPosition());
    } else {
        m_Motion.incrementToVelocity(m_velocity_target, dt);
        requested = m_Motion.getVelocity();
        m_rotation_setpoint = static_cast<float>(m_Motion.getPosition());
    }

    m_velocity_setpoint = requested;

    if (m_mode == Mode::Position && actual_steps == m_steps_setpoint) {
        CriticalSectionLock irq_lock;
        stopTimer();
        m_planner_active = false;
        m_velocity_setpoint = 0.0f;
        return;
    }

    if (m_mode == Mode::Velocity && m_velocity_target == 0.0f
        && std::fabs(requested) < 0.00001f) {
        CriticalSectionLock irq_lock;
        stopTimer();
        m_steps_setpoint = m_steps;
        m_planner_active = false;
        m_velocity_setpoint = 0.0f;
        return;
    }

    // Position mode: the last 1-2 quantized microsteps may remain after
    // Motion has mathematically reached its floating point destination.
    // Finish them at a low rate rather than leaving an incomplete move.
    if (m_mode == Mode::Position && std::fabs(requested) < 0.0001f) {
        const int64_t error = static_cast<int64_t>(m_steps_setpoint) - actual_steps;
        requested = (error > 0 ? 1.0f : -1.0f)
                  * std::min(0.02f, m_move_velocity_limit);
    }

    if (std::fabs(requested) * m_steps_per_rev < 1.0f) {
        CriticalSectionLock irq_lock;
        stopTimer();
        return; // Planner continues ramping until a useful minimum rate
    }

    requested = std::max(-m_velocity_max, std::min(m_velocity_max, requested));
    PulseConfig config{};
    if (!calculatePulseTrain(std::fabs(requested), config)) {
        CriticalSectionLock irq_lock;
        stopTimer();
        return;
    }
    {
        CriticalSectionLock irq_lock;
        if (!m_planner_active) return; // target may have been reached in timer ISR
        startAtConfig(requested > 0.0f ? 1 : -1, config);
    }
}

void Stepper::onTimerUpdate()
{
    if (!(m_tim->SR & TIM_SR_UIF) || !(m_tim->DIER & TIM_DIER_UIE)) return;
    m_tim->SR &= ~TIM_SR_UIF;
    if (!m_running) return;

    if (m_step_sign > 0) {
        if (m_steps < STEPS_MAX) ++m_steps;
    } else {
        if (m_steps > STEPS_MIN) --m_steps;
    }

    if (m_steps == m_steps_setpoint) {
        stopTimer();
        m_planner_active = false; // marks finished even if the next 5 ms tick is late
    }
}

void Stepper::onTim10Update() { if (s_tim10) s_tim10->onTimerUpdate(); }
void Stepper::onTim11Update() { if (s_tim11) s_tim11->onTimerUpdate(); }
