#include "Stepper.h"

#include <cmath>

Stepper::Stepper(PinName step_pin, PinName dir_pin, uint16_t step_per_rev)
    : m_Step(step_pin)
    , m_Dir(dir_pin)
    , m_Thread(osPriorityHigh2)
{
    m_steps_per_rev = static_cast<float>(step_per_rev);
    m_time_step_const = 1.0e6f / m_steps_per_rev; // [us] per step @ 1 rps

    m_steps_setpoint = 0;
    m_steps = 0;
    m_velocity = 1.0f;

    m_period_mus = 0;
    m_exit_thread = false;

    // define initial direction to avoid spurious first pulse
    m_Dir.write(0);

    // start thread
    m_Thread.start(callback(this, &Stepper::threadTask));
}

Stepper::~Stepper()
{
    // stop ticker first to prevent new tasks
    m_Ticker.detach();

    // signal thread to exit gracefully
    m_exit_thread = true;
    m_Thread.flags_set(m_ThreadFlag);

    // wait until worker thread terminates
    m_Thread.join();

    // ensure no trailing pulses
    m_Timeout.detach();
    disableDigitalOutput();
}

int Stepper::rotationsToSteps(float rotations) const
{
    const float max_rot = static_cast<float>(STEPS_MAX) / m_steps_per_rev;
    const float min_rot = static_cast<float>(STEPS_MIN) / m_steps_per_rev;

    // reject non-finite inputs explicitly
    if (!std::isfinite(rotations))
        return (rotations > 0.0f) ? STEPS_MAX : (rotations < 0.0f ? STEPS_MIN : 0);

    if (rotations >= max_rot)
        return STEPS_MAX;
    if (rotations <= min_rot)
        return STEPS_MIN;

    const float steps_f = rotations * m_steps_per_rev;
    return (steps_f >= 0.0f) ? static_cast<int>(steps_f + 0.5f) : static_cast<int>(steps_f - 0.5f);
}

void Stepper::setRotation(float rotations, float velocity)
{
    const float max_rotations = static_cast<float>(STEPS_MAX) / m_steps_per_rev;
    const float min_rotations = static_cast<float>(STEPS_MIN) / m_steps_per_rev;

    if (rotations > max_rotations)
        rotations = max_rotations;
    else if (rotations < min_rotations)
        rotations = min_rotations;

    setSteps(rotationsToSteps(rotations), velocity);
}

void Stepper::setRotation(float rotations) { setRotation(rotations, m_velocity); }

void Stepper::setRotationRelative(float rotations, float velocity)
{
    const int current_steps = m_steps;
    const int delta_steps = rotationsToSteps(rotations);

    // Saturating add in 64-bit to avoid any overflow UB
    const int64_t tmp = static_cast<int64_t>(current_steps) + static_cast<int64_t>(delta_steps);

    int target_steps;
    if (tmp > STEPS_MAX)
        target_steps = STEPS_MAX;
    else if (tmp < STEPS_MIN)
        target_steps = STEPS_MIN;
    else
        target_steps = static_cast<int>(tmp);

    setSteps(target_steps, velocity);
}

void Stepper::setRotationRelative(float rotations) { setRotationRelative(rotations, m_velocity); }

void Stepper::setVelocity(float velocity)
{
    if (velocity == 0.0f) {
        // stop: lock setpoint to current position and detach ticker
        m_velocity = 0.0f;
        m_period_mus = 0;
        m_steps_setpoint = m_steps;
        m_Ticker.detach();
        return;
    } else if (velocity > 0.0f) {
        setSteps(STEPS_MAX, velocity);
    } else {
        setSteps(STEPS_MIN, velocity);
    }
}

void Stepper::setSteps(int steps, float velocity)
{
    // clamp setpoint to representable range to guarantee eventual stop
    if (steps > STEPS_MAX)
        steps = STEPS_MAX;
    else if (steps < STEPS_MIN)
        steps = STEPS_MIN;

    // update setpoint
    m_steps_setpoint = steps;

    // return if setpoint is equal to current position or velocity is zero
    if (m_steps_setpoint == m_steps || velocity == 0.0f) {
        m_velocity = 0.0f;
        m_period_mus = 0;
        m_steps_setpoint = m_steps; // keep setpoint aligned when stopping
        m_Ticker.detach();
        return;
    }

    // write direction
    if (m_steps_setpoint > m_steps)
        m_Dir.write(1);
    else
        m_Dir.write(0);

    // calculate period for ticker in microseconds
    int period_mus = static_cast<int>(m_time_step_const / std::fabs(velocity) + 0.5f);

    // enforce minimum period so STEP can actually go low between pulses
    if (period_mus < MIN_PERIOD_US)
        period_mus = MIN_PERIOD_US;

    // reconfigure ticker only if the period changed
    if (m_period_mus != period_mus) {
        m_period_mus = period_mus;

        // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically,
        // which signals the thread to execute
        m_Ticker.attach(callback(this, &Stepper::sendThreadFlag), std::chrono::microseconds{period_mus});
    }

    // always keep m_velocity consistent with the active period and requested sign
    if (m_period_mus > 0) {
        m_velocity = std::copysign(m_time_step_const / static_cast<float>(m_period_mus), velocity);
    } else {
        m_velocity = 0.0f;
    }
}

void Stepper::step()
{
    // send one step via timeout
    m_Timeout.detach(); // ensure no previous pulse is still pending
    enableDigitalOutput();
    m_Timeout.attach(callback(this, &Stepper::disableDigitalOutput), std::chrono::microseconds{PULSE_MUS});

    // increment steps with saturation
    if (m_Dir.read() == 0) {
        if (m_steps > STEPS_MIN)
            --m_steps;
    } else {
        if (m_steps < STEPS_MAX)
            ++m_steps;
    }
}

void Stepper::enableDigitalOutput()
{
    m_Step = 1; // set the digital output to high
}

void Stepper::disableDigitalOutput()
{
    m_Step = 0; // set the digital output to low
}

void Stepper::threadTask()
{
    while (true) {
        // wait for a tick
        ThisThread::flags_wait_any(m_ThreadFlag);

        if (m_exit_thread)
            break;

        // if we are at the setpoint, stop the ticker and skip stepping
        if (m_steps_setpoint == m_steps) {
            m_period_mus = 0;
            m_Ticker.detach();
            continue;
        }

        step();
    }
}

void Stepper::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
