/**
 * @file RealTimeThread.h
 * @brief Defines a configurable real-time thread template for periodic tasks.
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
 * 1. Create a RealTimeThread instance with optional period customization
 * 2. Override the virtual executeTask() method or use callback functionality
 * 3. Use enable()/disable() to control execution
 * 4. Monitor execution statistics with diagnostic functions
 *
 * @example
 * ```cpp
 * class MyRealTimeTask : public RealTimeThread {
 *     // for every constructor that exists in RealTimeThread, add a corresponding constructor to the 
 *     // overload set of MyRealTimeTask that simply forwards its arguments to the base‐class constructor
 *     using RealTimeThread::RealTimeThread;
 * protected:
 *     void executeTask() override {
 *         static uint32_t run_cntr = 0;
 *         // avoid printf in real-time threads by default, this is just an example!
 *         printf("MyRealTimeTask is enabled and runs for the %lu-th time\n", run_cntr++);
 *     }
 * };
 *
 * MyRealTimeTask real_time_thread(1000000);
 * real_time_thread.enable();
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef REAL_TIME_THREAD_H_
#define REAL_TIME_THREAD_H_

#include "ThreadFlag.h"

using namespace std::chrono;

class RealTimeThread
{
public:
    /**
     * @brief Construct a RealTimeThread with configurable period
     * @param period_us Period in microseconds (default: 1000 us = 1 ms for real-time control)
     * @param priority Thread priority (default: osPriorityNormal)
     * @param stack_size Stack size in bytes (default: OS_STACK_SIZE = 4096)
     */
    explicit RealTimeThread(uint32_t period_us = 1000,
                            osPriority priority = osPriorityNormal,
                            uint32_t stack_size = OS_STACK_SIZE);

    virtual ~RealTimeThread();

    // Check
    bool isRunning() const { return m_running; }

    // Control methods
    void enable();
    void disable();
    bool isEnabled() const { return m_enabled; }

    // Configuration methods
    void setPeriod(uint32_t period_us);
    uint32_t getPeriod() const { return m_period_us; }
    void setPriority(osPriority priority);
    osPriority getPriority() const { return m_priority; }

    // Diagnostics
    uint32_t getExecutionCount() const { return m_execution_count; }
    uint32_t getMaxExecutionTimeUs() const { return m_max_execution_time_us; }
    uint32_t getLastExecutionTimeUs() const { return m_last_execution_time_us; }
    uint32_t getMinExecutionTimeUs() const;
    float getAverageExecutionTimeUs() const;
    void resetDiagnostics();

protected:
    /**
     * @brief Override this method to implement your real-time task
     * This method will be called periodically when the thread is enabled
     */
    virtual void executeTask();

private:
    // Initialization methods
    bool init();

    // Configuration
    uint32_t m_period_us;
    osPriority m_priority;
    uint32_t m_stack_size;

    // Threading components
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    mutable Mutex m_Mutex;  // mutable to allow locking in const methods

    // States
    bool m_enabled{false};
    bool m_running{false};

    // Diagnostics
    mutable uint32_t m_execution_count{0};
    mutable uint32_t m_max_execution_time_us{0};
    mutable uint32_t m_min_execution_time_us{UINT32_MAX};
    mutable uint32_t m_last_execution_time_us{0};
    mutable uint64_t m_total_execution_time_us{0};
    void updateExecutionStats(uint32_t execution_time_us) const;

    void threadTask();
    void sendThreadFlag();
};

#endif /* REAL_TIME_THREAD_H_ */
