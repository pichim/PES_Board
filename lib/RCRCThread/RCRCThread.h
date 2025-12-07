/**
 * @file RCRCThread.h
 * @brief Real-time thread for RCRC frequency response measurement.
 *
 * This class inherits from RealTimeThread and periodically performs
 * analog I/O operations, generates a chirp excitation, and streams
 * measured data via SerialStream for logging (e.g. OpenLager or MATLAB).
 *
 * - The thread toggles execution using a button or a start byte from host.
 * - While executing, it applies a chirp to an analog output and logs
 *   excitation and measurement voltages.
 * - When idle, it keeps the output at DC offset and resets when toggled off.
 *
 * The internal timing (ticker period) and thread priority are inherited
 * from RealTimeThread.
 *
 * @note The class mirrors the behavior of the original working RealTimeThread
 *       implementation, but with a cleaner separation of concerns.
 *
 * @author
 *   M. Peter / pmic / pichim
 */

#ifndef RCRC_THREAD_H_
#define RCRC_THREAD_H_

#include "config.h"

#include "Chirp.h"
#include "DebounceIn.h"
#include "RealTimeThread.h"
#include "SerialStream.h"
// #include "SerialStreamThread.h"

using namespace std::chrono;

/**
 * @class RCRCThread
 * @brief Real-time task for RCRC (frequency response) measurement.
 */
class RCRCThread : public RealTimeThread {
public:
    /**
     * @brief Construct a new RCRCThread.
     *
     * @param period_us  Period in microseconds (default: RTT_PERIOD_US)
     * @param priority   Thread priority (default: osPriorityHigh)
     * @param stack_size Stack size in bytes (default: 4096)
     */
    explicit RCRCThread(uint32_t period_us = RTT_PERIOD_US,
                        osPriority priority = osPriorityHigh,
                        uint32_t stack_size = 4096);

    /** @brief Destructor — puts output in safe state. */
    ~RCRCThread() override;

protected:
    /**
     * @brief Periodic task executed by the RealTimeThread base class.
     *
     * This function is called every period_us when the ticker triggers.
     * It handles the measurement, excitation, and serial transmission.
     */
    void executeTask() override;

private:
    // Sampling time
    const float m_Ts;

    // I/O
    DebounceIn m_Button;  ///< Start/stop button
    DigitalOut m_Led;     ///< Status LED (on during measurement)
    AnalogIn   m_Ain1;    ///< Analog input channel 1
    AnalogIn   m_Ain2;    ///< Analog input channel 2
    AnalogOut  m_Aout;    ///< Analog output (chirp excitation)

    // Serial communication
    SerialStream m_SerialStream; ///< Stream for data logging
    // SerialStreamThread m_SerialStream; ///< Stream for data logging

    // Timing
    Timer        m_Timer;             ///< Measures elapsed time
    microseconds m_time_previous_us{0}; ///< Last timestamp

    // Chirp generator
    static constexpr float T1_SEC    = 3.0f;
    static constexpr float F0_HZ     = 1.0f / T1_SEC;
    static constexpr float AMP_V     = 0.9f * (3.3f / 2.0f);
    static constexpr float OFFSET_V  = 3.3f / 2.0f;
    Chirp m_Chirp;
    float m_sinarg{0.0f};

    // State
    bool m_do_execute{false};
    bool m_do_reset{false};
    bool m_is_start_byte_received{false};

    // Button ISR (no heavy work)
    void toggleDoExecute();
};

#endif /* RCRC_THREAD_H_ */
