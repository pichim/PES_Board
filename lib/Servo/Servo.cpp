#include "Servo.h"

Servo::Servo(PinName pin, float pulse_min, float pulse_max) : m_DigitalOut(pin), m_Thread(osPriorityAboveNormal1)
{
    // calibrate the servo with the given minimum and maximum pulse widths
    calibratePulseMinMax(pulse_min, pulse_max);

    // set default motion profile
    setMaxVelocity();
    setMaxAcceleration();

    // start thread
    m_Thread.start(callback(this, &Servo::threadTask));
}

Servo::~Servo()
{
    m_Ticker.detach();
    m_Timeout.detach();
    m_Thread.terminate();
}

void Servo::calibratePulseMinMax(float pulse_min, float pulse_max)
{
    // set minimal and maximal pulse width
    m_pulse_min = pulse_min;
    m_pulse_max = pulse_max;
}

void Servo::setMaxVelocity(float velocity)
{
    // convert velocity from calibrated normalised pulse width to normalised pulse width
    velocity *= (m_pulse_max - m_pulse_min);
    m_Motion.setProfileVelocity(velocity);
}

void Servo::setMaxAcceleration(float acceleration)
{
    // convert acceleration from calibrated normalised pulse width to normalised pulse width
    acceleration *= (m_pulse_max - m_pulse_min);
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

void Servo::setPulseWidth(float pulse)
{
    // after calibrating the mapping from setPulseWidth() is (0, 1) -> (pulse_min, pulse_max)
    m_pulse = calculateNormalisedPulseWidth(pulse);
}

void Servo::enable(float pulse)
{
    m_enabled = true;

    // set pulse width when enabled
    m_pulse = calculateNormalisedPulseWidth(pulse);
    m_Motion.setPosition(m_pulse);

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &Servo::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

void Servo::disable()
{
    m_enabled = false;

    // detach ticker and timeout
    m_Ticker.detach();
    m_Timeout.detach();
}

bool Servo::isEnabled() const
{
    return m_enabled;
}

float Servo::calculateNormalisedPulseWidth(float pulse)
{
    // it is assumed that after the calibration m_pulse_min != 0.0f and if so
    // we constrain the pulse to the range (0.0f, 1.0f)
    if (m_pulse_min != 0.0f)
        pulse = (pulse > 1.0f) ? 1.0f : (pulse < 0.0f) ? 0.0f : pulse;
    return constrainPulse((m_pulse_max - m_pulse_min) * pulse + m_pulse_min);
}

void Servo::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        if (isEnabled()) {
            // increment to position
            m_Motion.incrementToPosition(m_pulse, TS);

            // convert to pulse width
            const uint16_t pulse_mus = static_cast<uint16_t>(m_Motion.getPosition() * static_cast<float>(PERIOD_MUS));

            // enable digital output and attach disableDigitalOutput() to timeout for soft PWM
            enableDigitalOutput();
            m_Timeout.attach(callback(this, &Servo::disableDigitalOutput), std::chrono::microseconds{pulse_mus});
        }
    }
}

void Servo::enableDigitalOutput()
{
    // set the digital output to high
    m_DigitalOut = 1;
}

void Servo::disableDigitalOutput()
{
    // set the digital output to low
    m_DigitalOut = 0;
}

void Servo::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}

float Servo::constrainPulse(float pulse) const
{
    // constrain pulse to range (PWM_MIN, PWM_MAX)
    return (pulse > PWM_MAX) ? PWM_MAX :
           (pulse < PWM_MIN) ? PWM_MIN :
            pulse;
}
