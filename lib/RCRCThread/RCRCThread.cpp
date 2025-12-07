#include "RCRCThread.h"

/**
 * @brief Constructor: initializes all peripherals and starts the periodic thread.
 *
 * The ticker is attached immediately (enable()), matching the original
 * monolithic RealTimeThread timing. The button ISR only toggles state flags.
 */
RCRCThread::RCRCThread(uint32_t period_us,
                       osPriority priority,
                       uint32_t stack_size)
    : RealTimeThread(period_us, priority, stack_size)
    , m_Ts(static_cast<float>(period_us) * 1.0e-6f)
    , m_Button(RTT_BUTTON, PullUp)
    , m_Led(RTT_LED, 0)
    , m_Ain1(RTT_AIN1)
    , m_Ain2(RTT_AIN2)
    , m_Aout(RTT_AOUT1)
    , m_SerialStream(RTT_TX, RTT_RX,
                     /*num_of_floats*/ RTT_NUM_OF_FLOATS,
                     /*baud*/ RTT_BAUDRATE)
    // , m_SerialStream(RTT_TX, RTT_RX,
    //                  /*num_of_floats*/ RTT_NUM_OF_FLOATS,
    //                  /*baud*/ RTT_BAUDRATE,
    //                  /*period_us (drain)*/ 1000,
    //                  /*prio*/ osPriorityAboveNormal)
    , m_Chirp(F0_HZ, (1.0f / 2.0f) / m_Ts, T1_SEC, m_Ts) // f1 = 1/(2*Ts)
{
    // Button ISR — toggles execution flag
    m_Button.fall(callback(this, &RCRCThread::toggleDoExecute));

    // Start ticker (RealTimeThread handles periodic flag signaling)
    enable();
    // m_SerialStream.enable();
}

/**
 * @brief Destructor: disables output for safety.
 */
RCRCThread::~RCRCThread()
{
    m_Aout.write(OFFSET_V / 3.3f);
}

/**
 * @brief ISR: toggles measurement execution flag.
 */
void RCRCThread::toggleDoExecute()
{
    m_do_execute = !m_do_execute;
    if (m_do_execute)
        m_do_reset = true;
}

/**
 * @brief Main periodic routine executed every RealTimeThread tick.
 *
 * Implements the same measurement cycle as the original working version:
 * - Measures analog inputs
 * - Generates a chirp excitation when active
 * - Sends data to host
 * - Toggles LED for visual feedback
 */
void RCRCThread::executeTask()
{
    // Perform one-time initialization inside the real-time thread.
    static bool boot = false;
    if (!boot) {
        m_Timer.start();
        m_time_previous_us = m_Timer.elapsed_time();
        ThisThread::sleep_for(1000ms);  // Wait for OpenLager / host to start
        boot = true;
    }

    // Check for start byte from host (MATLAB/OpenLager)
    if (!m_is_start_byte_received && m_SerialStream.startByteReceived()) {
        m_is_start_byte_received = true;
        toggleDoExecute();
    }

    // Measure Δt since last cycle
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // Read analog inputs
    float uc_1 = m_Ain1.read() * 3.3f;
    float uc_2 = m_Ain2.read() * 3.3f;
    float u_e  = OFFSET_V;

    if (m_do_execute) {
        // Generate chirp excitation
        if (m_Chirp.update()) {
            const float exc = m_Chirp.getExc();
            m_sinarg = m_Chirp.getSinarg();
            u_e = AMP_V * exc + OFFSET_V;
        } else {
            m_do_execute = false; // finished sweep
        }

        // Write analog output
        m_Aout.write(u_e / 3.3f);

        // Transmit data frame (same layout as original)
        // m_SerialStream.write(dtime_us); // 0: dt [µs]
        m_SerialStream.write(u_e);      // 1: excitation [V]
        m_SerialStream.write(uc_1);     // 2: Ain1 [V]
        m_SerialStream.write(uc_2);     // 3: Ain2 [V]
        m_SerialStream.write(m_sinarg); // 4: chirp argument [rad]
        m_SerialStream.send();

        // LED on while executing
        m_Led = 1;

    } else {
        // Idle: keep output constant
        m_Aout.write(u_e / 3.3f);

        if (m_do_reset) {
            m_do_reset = false;

            m_SerialStream.reset();
            m_Chirp.reset();
            m_is_start_byte_received = false;

            // LED off when idle
            m_Led = 0;
        }
    }
}
