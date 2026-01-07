/**
 * @file RealTimeThread.h
 * @brief Simple real-time thread template for periodic tasks
 *
 * The RealTimeThread class provides a simple framework for executing periodic tasks
 * at real-time intervals. It uses a ticker to signal a dedicated thread at regular
 * intervals, making it suitable for control loops, sensor reading, and other
 * time-critical operations.
 *
 * @dependencies
 * This class relies on the following components:
 * - **ThreadFlag**: For thread synchronization and signaling
 * - **Thread**: For dedicated task execution
 * - **Ticker**: For precise timing control
 * - **Mutex**: For thread-safe enable/disable operations
 *
 * @usage
 * 1. Create a RealTimeThread instance with fixed period configuration
 * 2. Override the virtual executeTask() method
 * 3. Use enable()/disable() to control execution
 *
 * @example
 * ```cpp
 * class MyRealTimeThread : public RealTimeThread {
 *     // for every constructor that exists in RealTimeThread, add a corresponding constructor to the
 *     // overload set of MyRealTimeThread that simply forwards its arguments to the base‐class constructor
 *     using RealTimeThread::RealTimeThread;
 * protected:
 *     void executeTask() override {
 *         static uint32_t run_cntr = 0;
 *         // avoid printf in real-time threads by default, this is just an example!
 *         printf("MyRealTimeThread is enabled and runs for the %lu time\n", run_cntr++);
 *     }
 *  };
 *
 * MyRealTimeThread my_real_time_thread(2000000);
 * my_real_time_thread.enable();
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef REAL_TIME_THREAD_H_
#define REAL_TIME_THREAD_H_

#include "ThreadFlag.h"

class RealTimeThread
{
public:
    /**
     * @brief Construct a RealTimeThread with fixed configuration
     * @param period_us Period in microseconds (default: 1000 us = 1 ms for real-time control)
     * @param priority Thread priority (default: osPriorityNormal)
     * @param stack_size Stack size in bytes (default: OS_STACK_SIZE = 4096)
     */
    explicit RealTimeThread(uint32_t period_us = 1000,
                            osPriority priority = osPriorityNormal,
                            uint32_t stack_size = OS_STACK_SIZE);

    virtual ~RealTimeThread();

    // Control methods
    void enable();
    void disable();
    bool isEnabled() const;

protected:
    /**
     * @brief Override this method to implement your real-time task
     * This method will be called periodically when the thread is enabled
     */
    virtual void executeTask();

private:
    // Immutable configuration
    const uint32_t m_period_us;

    // Threading components
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    mutable Mutex m_Mutex; // protects m_enabled and m_running

    // States
    bool m_enabled{false}; // Ticker state
    bool m_running{false}; // Thread state

    void threadTask();
    void sendThreadFlag();
};

#endif /* REAL_TIME_THREAD_H_ */
