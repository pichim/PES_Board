/**
 * @file IRSensor.h
 * @brief This file defines the IRSensor class.
 * @author M. Peter / pmic / pichim
 */

#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include "mbed.h"

#include "ThreadFlag.h"
#include "AvgFilter.h"

#define IR_SENSOR_DISTANCE_MIN 0.0f
#define IR_SENSOR_DISTANCE_MAX 200.0f

class IRSensor
{
public:
    explicit IRSensor(PinName pin);
    explicit IRSensor(PinName pin, float a, float b);
    virtual ~IRSensor();

    // resets the filter to the current readout
    float reset();

    // returns the current average
    float read() const { return m_distance_avg; }
    float readmV() const { return m_distance_mV; }
    float readcm() const { return m_distance_cm; } // equal to m_distance_mV if not calibrated
    void setCalibration(float a, float b);

private:
    static constexpr int64_t PERIOD_MUS = 2000;
    static constexpr uint8_t N = 31;

    AnalogIn m_AnalogIn;
    AvgFilter m_AvgFilter;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    bool m_is_calibrated{false};
    float m_distance_mV{0.0f};
    float m_distance_cm{0.0f};
    float m_distance_avg{0.0f}; // average of m_distance_cm if calibrated, m_distance_mV otherwise
    float m_a{0.0f};
    float m_b{0.0f};

    float applyCalibration(float ir_distance_mV, float a, float b);

    void threadTask();
    void sendThreadFlag();
};
#endif /* IR_SENSOR_H_ */
