#include "RealTimeThread.h"

RealTimeThread::RealTimeThread(uint32_t period_us,
                               osPriority priority,
                               uint32_t stack_size) : m_period_us(period_us)
                                                    , m_priority(priority)
                                                    , m_stack_size(stack_size)
                                                    , m_Thread(priority, stack_size)
{
    const bool init_success = init();
    if (!init_success) {
        printf("RealTimeThread: CRITICAL ERROR - Failed to initialize, object is not functional\n");
    }
    // we enable the base class per default, so that the thread starts running
    enable();
}

RealTimeThread::~RealTimeThread()
{
    // graceful shutdown - disable ticker first
    disable();

    // signal shutdown with atomic operation
    m_Mutex.lock();
    m_running = false;
    m_Mutex.unlock();

    // wake up thread for final exit check
    m_Thread.flags_set(m_ThreadFlag);

    // wait for thread to finish with proper timeout
    Timer shutdown_timer;
    shutdown_timer.start();
    while (m_Thread.get_state() != Thread::Deleted &&
           duration_cast<milliseconds>(shutdown_timer.elapsed_time()).count() < 100) {
        ThisThread::sleep_for(1ms);
    }

    // only terminate if thread hasn't exited cleanly
    if (m_Thread.get_state() != Thread::Deleted) {
        printf("RealTimeThread: WARNING - Thread did not exit cleanly, forcing termination\r\n");
        m_Thread.terminate();
    }
}

void RealTimeThread::enable()
{
    if (!m_running) {
        printf("RealTimeThread: ERROR - Cannot enable, object not properly initialized\n");
        return;
    }

    m_Mutex.lock();
    if (!m_enabled) {
        // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
        m_Ticker.attach(callback(this, &RealTimeThread::sendThreadFlag), microseconds{m_period_us});
        m_enabled = true;
    }
    m_Mutex.unlock();
}

void RealTimeThread::disable()
{
    m_Mutex.lock();
    if (m_enabled) {
        m_Ticker.detach();
        m_enabled = false;
    }
    m_Mutex.unlock();
}

void RealTimeThread::setPeriod(uint32_t period_us)
{
    // input validation with proper default handling
    if (period_us == 0) {
        printf("RealTimeThread: WARNING - Zero period specified, using 1 ms default\r\n");
        period_us = 1000;  // set to 1ms default
    }

    // thread-safe update of period and enabled state
    m_Mutex.lock();
    m_period_us = period_us;
    const bool currently_enabled = m_enabled;
    m_Mutex.unlock();

    if (currently_enabled) {
        // restart ticker with new period (outside of mutex to avoid deadlock)
        m_Ticker.detach();
        m_Ticker.attach(callback(this, &RealTimeThread::sendThreadFlag), std::chrono::microseconds{period_us});
    }
}

void RealTimeThread::setPriority(osPriority priority)
{
    m_Mutex.lock();
    m_priority = priority;
    m_Thread.set_priority(priority);
    m_Mutex.unlock();
}

uint32_t RealTimeThread::getMinExecutionTimeUs() const
{
    m_Mutex.lock();
    const uint32_t min_execution_time_us = (m_execution_count > 0) ? m_min_execution_time_us : 0;
    m_Mutex.unlock();
    return min_execution_time_us;
}

float RealTimeThread::getAverageExecutionTimeUs() const
{
    m_Mutex.lock();
    const float average_execution_time_us = (m_execution_count > 0) ? (float)m_total_execution_time_us / m_execution_count : 0.0f;
    m_Mutex.unlock();
    return average_execution_time_us;
}

void RealTimeThread::resetDiagnostics()
{
    m_Mutex.lock();
    m_execution_count = 0;
    m_max_execution_time_us = 0;
    m_min_execution_time_us = UINT32_MAX;
    m_last_execution_time_us = 0;
    m_total_execution_time_us = 0;
    m_Mutex.unlock();
}

bool RealTimeThread::init()
{
    // check if already initialized
    if (m_running) {
        printf("RealTimeThread: WARNING - init() called more than once\r\n");
        return true;
    }

    // check if thread flag was allocated successfully
    if (!m_ThreadFlag.isValid()) {
        printf("RealTimeThread: ERROR - Failed to allocate thread flag, cannot start thread\r\n");
        return false; // cannot proceed without a valid thread flag
    }

    // input validation
    if (m_period_us == 0) {
        m_period_us = 1000;
        printf("RealTimeThread: WARNING - Zero period specified, using 1 ms default\r\n");
    }

    // start thread
    m_Thread.start(callback(this, &RealTimeThread::threadTask));

    // mark thread as running
    m_running = true;

    return true;
}

void RealTimeThread::executeTask()
{
    // default implementation - can be overridden
    static uint32_t run_cntr = 0;
    // avoid printf in real-time threads by default, this is just an example!
    printf("RealTimeThread is enabled and runs for the %lu time\n", run_cntr++);
}

void RealTimeThread::updateExecutionStats(uint32_t execution_time_us) const
{
    m_Mutex.lock();
    m_execution_count++;
    m_last_execution_time_us = execution_time_us;
    m_total_execution_time_us += execution_time_us;
    if (execution_time_us > m_max_execution_time_us) {
        m_max_execution_time_us = execution_time_us;
    }
    if (execution_time_us < m_min_execution_time_us) {
        m_min_execution_time_us = execution_time_us;
    }
    m_Mutex.unlock();
}

void RealTimeThread::threadTask()
{
    Timer execution_timer;
    execution_timer.start();

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // exit thread on shutdown
        if (!m_running) {
            break;
        }

        // skip execution if disabled, but keep thread alive
        if (m_enabled) {
            // reset the execution timer
            execution_timer.reset();

            // execute the user-defined task
            executeTask();

            // measure execution time and update statistics
            const microseconds elapsed_time = execution_timer.elapsed_time();
            const uint32_t execution_time_us = duration_cast<microseconds>(elapsed_time).count();

            // // check for timing overruns (critical for real-time systems)
            // if (execution_time_us > (m_period_us * 80) / 100) {  // 80% threshold
            //     printf("RealTimeThread: WARNING - Task overrun: %lu μs > 80%% of %lu μs period\r\n", 
            //            execution_time_us, m_period_us);
            // }

            updateExecutionStats(execution_time_us);
        }
    }
}

void RealTimeThread::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
