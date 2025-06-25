#include "IRSensor.h"

IRSensor::IRSensor(PinName pin) : m_AnalogIn(pin),
                                  m_AvgFilter(N),
                                  m_Thread(osPriorityNormal)
{
    // start thread
    m_Thread.start(callback(this, &IRSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &IRSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

IRSensor::IRSensor(PinName pin, float a, float b) : m_AnalogIn(pin),
                                                    m_AvgFilter(N),
                                                    m_Thread(osPriorityNormal)
{
    // calibrate the sensor
    setCalibration(a, b);

    // start thread
    m_Thread.start(callback(this, &IRSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &IRSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

IRSensor::~IRSensor()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

float IRSensor::reset()
{
    // readout in millivolts
    m_distance_mV = m_AnalogIn.read() * 3300.0f;

    // apply calibration to cm (if calibrated)
    float distance_cm = 0.0f;
    if (m_is_calibrated) {
        distance_cm = applyCalibration(m_distance_mV, m_a, m_b);
        // constrain distance to [IR_SENSOR_DISTANCE_MIN, IR_SENSOR_DISTANCE_MAX]
        distance_cm = (distance_cm > IR_SENSOR_DISTANCE_MAX) ? IR_SENSOR_DISTANCE_MAX :
                      (distance_cm < IR_SENSOR_DISTANCE_MIN) ? IR_SENSOR_DISTANCE_MIN :
                       distance_cm;
    } else {
        distance_cm = m_distance_mV;
    }
    m_distance_cm = distance_cm;

    // reset the filter to the current readout
    m_distance_avg = m_AvgFilter.reset(m_distance_cm);

    return m_distance_avg;
}

void IRSensor::setCalibration(float a, float b)
{
    m_a = a;
    m_b = b;
    m_is_calibrated = true;
}

void IRSensor::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // readout in millivolts
        m_distance_mV = m_AnalogIn.read() * 3300.0f;

        // apply calibration to cm (if calibrated)
        float distance_cm = 0.0f;
        if (m_is_calibrated) {
            distance_cm = applyCalibration(m_distance_mV, m_a, m_b);
            // constrain distance to [IR_SENSOR_DISTANCE_MIN, IR_SENSOR_DISTANCE_MAX]
            distance_cm = (distance_cm > IR_SENSOR_DISTANCE_MAX) ? IR_SENSOR_DISTANCE_MAX :
                          (distance_cm < IR_SENSOR_DISTANCE_MIN) ? IR_SENSOR_DISTANCE_MIN :
                           distance_cm;
        } else {
            distance_cm = m_distance_mV;
        }
        m_distance_cm = distance_cm;

        // average filtered distance
        static bool is_first_run = true;
        if (is_first_run) {
            is_first_run = false;
            m_distance_avg = m_AvgFilter.reset(m_distance_cm);
        } else
            m_distance_avg = m_AvgFilter.apply(m_distance_cm);
    }
}

float IRSensor::applyCalibration(float ir_distance_mV, float a, float b)
{
    // // insert values that you got from the MATLAB or Python file
    // static const float a = 2.574e+04f;
    // static const float b = -29.37f;

    // avoid division by zero by adding a small value to the denominator
    if (ir_distance_mV + b == 0.0f)
        ir_distance_mV -= 0.001f;

    return a / (ir_distance_mV + b);
}

void IRSensor::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
