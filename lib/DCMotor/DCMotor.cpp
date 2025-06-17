#include "DCMotor.h"

DCMotor::DCMotor(PinName pwm_pin,
                 PinName enc_a_pin,
                 PinName enc_b_pin,
                 float gear_ratio,
                 float kn,
                 float voltage_max,
                 float counts_per_turn) : m_FastPWM(pwm_pin),
                                          m_EncoderCounter(enc_a_pin, enc_b_pin),
                                          m_Thread(osPriorityHigh1)
#if PERFORM_CHIRP_MEAS
                                          , m_BufferedSerial(USBTX, USBRX)
#endif
{
    // motor parameters
    m_counts_per_turn = gear_ratio * counts_per_turn;
    m_voltage_max = voltage_max;
    m_velocity_physical_max = kn / 60.0f * voltage_max;
    m_velocity_max = m_velocity_physical_max;

    // default controller parameters, parameters adapted from gear ratio 78:1 tune
    const float k_gear = gear_ratio / 78.125f;
    setVelocityCntrl(DCMotor::KP * k_gear, DCMotor::KI * k_gear, DCMotor::KD * k_gear);
    if (kn > 0.0f)
        m_PIDCntrl_velocity.setParamF(60.0f / kn);
    setRotationCntrlGain();

    // iir filter
    m_IIR_Filter_velocity.lowPass2Init(15.0f, 1.0f, TS);

    // initialise control signals
    m_count = m_count_previous = m_EncoderCounter.read();
    m_rotation_initial = static_cast<float>(m_count) / m_counts_per_turn;
    m_rotation_target = m_rotation_initial;
    m_rotation_setpoint = m_rotation_initial;
    m_rotation = m_rotation_initial;
    m_velocity_target = 0.0f;
    m_velocity_setpoint = 0.0f;
    m_velocity = 0.0f;
    m_voltage = 0.0f;
    m_pwm = 0.0f;

    // initilise motion planner, parameters adapted from gear ratio 78:1 tune
    m_enable_motion_planner = false;
    m_Motion.setPosition(0.0f);
    m_Motion.setProfileVelocity(m_velocity_max);
    m_acceleration_max = 400.0f / gear_ratio;
    setMaxAcceleration(m_acceleration_max);

#if PERFORM_GPA_MEAS
    // closed-loop measurement
    const float fMin = 1.0f;
    const float fMax = 0.99f/2.0f/TS;
    const uint16_t NfexcDes = 80;
    const float Aexc0 = 0.4f * m_velocity_max;
    const float Aexc1 = 0.5f * 0.4f * m_velocity_max; // Aexc0/fMax;
    const int   NperMin = 3;
    const float TmeasMin = 0.5f;
    const int   NmeasMin = (int)ceilf(TmeasMin/TS);
    const float Tstart = 1.0f;
    const int   Nstart = (int)ceilf(Tstart/TS);
    const float Tsweep = 0.3f;
    const int   Nsweep = (int)ceilf(Tsweep/TS);
    m_GPA.init(fMin, fMax, NfexcDes, NperMin, NmeasMin, TS, Aexc0, Aexc1, Nstart, Nsweep, true, true);
#endif

#if PERFORM_CHIRP_MEAS
    const float f0 = 0.1f;
    const float f1 = 0.99f/2.0f/TS;
    const float t1 = 60.0f;
    m_chirp.init(f0, f1, t1, TS);
    m_BufferedSerial.set_baud(2000000);
    m_BufferedSerial.set_blocking(false);
    m_timer.start();
#endif

    // start thread
    m_Thread.start(callback(this, &DCMotor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &DCMotor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

DCMotor::~DCMotor()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void DCMotor::setVelocity(float velocity)
{
    m_cntrlMode = CntrlMode::Velocity;
    m_velocity_target = velocity;
}

void DCMotor::setRotation(float rotation)
{
    m_cntrlMode = CntrlMode::Rotation;
    m_rotation_target = m_rotation_initial + rotation;
}

void DCMotor::setRotationRelative(float rotation_relative)
{
    m_cntrlMode = CntrlMode::Rotation;
    m_rotation_target = getRotation() + rotation_relative;
}

float DCMotor::getRotationTarget() const
{
    return m_rotation_target;
}

float DCMotor::getRotationSetpoint() const
{
    return m_rotation_setpoint;
}

float DCMotor::getRotation() const
{
    return m_rotation - m_rotation_initial;
}

float DCMotor::getVelocityTarget() const
{
    return m_velocity_target;
}

float DCMotor::getVelocitySetpoint() const
{
    return m_velocity_setpoint;
}

float DCMotor::getVelocity() const
{
    return m_velocity;
}

float DCMotor::getVoltage() const
{
    return m_voltage;
}

float DCMotor::getPWM() const
{
    return m_pwm;
}

void DCMotor::setVelocityCntrl(float kp, float ki, float kd)
{
    const float tau_f = 1.0f / (2.0f * M_PIf * 30.0f);
    const float tau_ro = 1.0f / (2.0f * M_PIf * 0.5f / (2.0f * TS));
    m_PIDCntrl_velocity.setup(kp,
                              ki,
                              kd,
                              tau_f,
                              tau_ro,
                              TS,
                              m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                              m_voltage_max * (2.0f * PWM_MAX - 1.0f));
    // avoid students melting their motors
    setVelocityCntrlIntegratorLimitsPercent();
    // // print for debugging
    // printf("DCMotor: velocity controller parameters: kp = %.4f, ki = %.4f, kd = %.4f\n", kp, ki, kd);
}

void DCMotor::setVelocityCntrlIntegratorLimitsPercent(float percent_of_max)
{
    percent_of_max = percent_of_max * 0.01f;
    m_PIDCntrl_velocity.setIntegratorLimits(percent_of_max * m_voltage_max * (2.0f * PWM_MIN - 1.0f),
                                            percent_of_max * m_voltage_max * (2.0f * PWM_MAX - 1.0f));
}

void DCMotor::setRotationCntrlGain(float p)
{
    m_p = p;
}

void DCMotor::setMaxVelocity(float velocity)
{
    m_velocity_max = (velocity > m_velocity_physical_max) ? m_velocity_physical_max : velocity;
    m_Motion.setProfileVelocity(m_velocity_max);
}

float DCMotor::getMaxVelocity() const
{
    return m_velocity_max;
}

float DCMotor::getMaxPhysicalVelocity() const
{
    return m_velocity_physical_max;
}

void DCMotor::setMaxAcceleration(float acceleration)
{
    m_Motion.setProfileAcceleration(acceleration);
    m_Motion.setProfileDeceleration(acceleration);
}

float DCMotor::getMaxAcceleration() const
{
    return m_acceleration_max;
}

void DCMotor::enableMotionPlanner()
{
    m_enable_motion_planner = true;
}

void DCMotor::disableMotionPlanner()
{
    m_enable_motion_planner = false;
}

long DCMotor::getEncoderCount() const
{
    return m_count;
}

void DCMotor::setMotionPlanerVelocity(float velocity) {
    m_Motion.setVelocity(velocity);
}

void DCMotor::setMotionPlanerPosition(float position) {
    m_Motion.setPosition(position);
}

void DCMotor::setFastPWMPeriod_mus(int period_mus)
{
    // set the period of the PWM signal in microseconds
    m_FastPWM.period_mus(period_mus);
}

#if PERFORM_GPA_MEAS
void DCMotor::startGPA()
{
    if (!m_start_gpa) {
        m_start_gpa = true;
    }
}
#endif

#if PERFORM_CHIRP_MEAS
void DCMotor::startChrip()
{
    if (!m_start_chirp) {
        m_start_chirp = true;
        m_timer.reset();
    }
}
#endif

void DCMotor::threadTask()
{
#if PERFORM_GPA_MEAS
    // print some gpa info
    m_GPA.printGPAmeasPara();
#endif

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // update counts (avoid overflow)
        const short count_actual = m_EncoderCounter.read();
        const short count_delta = count_actual - m_count_previous; // avoid overflow
        m_count_previous = count_actual;

        // update rotation
        m_count += count_delta;
        m_rotation = static_cast<float>(m_count) / m_counts_per_turn;

        // update velocity
        const float rotation_increment = static_cast<float>(count_delta) / m_counts_per_turn;
        m_velocity = m_IIR_Filter_velocity.apply(rotation_increment / TS);

        float velocity_setpoint = 0.0f;

        switch (m_cntrlMode) {

            case CntrlMode::Rotation:
                if (m_enable_motion_planner) {
                    // use motion planner
                    m_Motion.incrementToPosition(m_rotation_target, TS);
                    m_rotation_setpoint = m_Motion.getPosition();
                    if ((fabs(m_rotation_setpoint - m_rotation) > ROTATION_ERROR_MAX) || (fabs(m_Motion.getVelocity()) > 0.0f))
                        velocity_setpoint = m_p * (m_rotation_setpoint - m_rotation) + m_Motion.getVelocity();
                } else {
                    m_rotation_setpoint = m_rotation_target;
                    if (fabs(m_rotation_setpoint - m_rotation) > ROTATION_ERROR_MAX)
                        velocity_setpoint = m_p * (m_rotation_setpoint - m_rotation);
                }

                break;

            case CntrlMode::Velocity:
                if (m_enable_motion_planner) {
                    // use motion planner
                    m_Motion.incrementToVelocity(m_velocity_target, TS);
                    velocity_setpoint = m_Motion.getVelocity();
                } else {
                    velocity_setpoint = m_velocity_target;
                }

                break;

            default:

                break; // should not happen
        }

        // constrain velocity to (-m_velocity_max, m_velocity_max)
        velocity_setpoint = (velocity_setpoint >  m_velocity_max) ?  m_velocity_max :
                            (velocity_setpoint < -m_velocity_max) ? -m_velocity_max :
                             velocity_setpoint;

#if PERFORM_GPA_MEAS
        static float exc = 0.0f;
        // closed-loop measurement
        const float voltage = m_PIDCntrl_velocity.update(0.6f * m_velocity_max - m_velocity + exc);
        if (m_start_gpa) {
            exc = m_GPA.update(voltage, m_velocity);
        }
#elif PERFORM_CHIRP_MEAS
        const float magnitude = 4.0f;
        const float offset = 5.0f;
        float voltage = offset;
        if (m_start_chirp && m_chirp.update()) {
            const float time_ms = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(m_timer.elapsed_time()).count()) * 1.0e-3f;
            m_timer.reset();
            const float exc = m_chirp.getExc();
            const float fchirp = m_chirp.getFreq();
            const float sinarg = m_chirp.getSinarg();
            voltage = magnitude * exc + offset;
            if (m_BufferedSerial.writable()) {
                memcpy(&m_buffer[0 ], &time_ms, 4);
                memcpy(&m_buffer[4 ], &voltage, 4);
                memcpy(&m_buffer[8 ], &fchirp, 4);
                memcpy(&m_buffer[12], &sinarg, 4);
                memcpy(&m_buffer[16], &m_rotation, 4);
                m_BufferedSerial.write(m_buffer, 20);
            }
        }
#else
        const float voltage = m_PIDCntrl_velocity.update(velocity_setpoint,       // w
                                                         m_velocity,              // y_p
                                                         rotation_increment / TS, // y_i
                                                         m_velocity);             // y_d
#endif

        // calculate pwm and write output
        const float pwm = 0.5f + 0.5f * voltage / m_voltage_max;
        m_FastPWM.write(pwm);

        // update signals
        m_velocity_setpoint = velocity_setpoint;
        m_voltage = voltage;
        m_pwm = pwm;
    }
}

void DCMotor::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
