#include "RealTimeThread.h"

using MutexLock = rtos::ScopedMutexLock;
using namespace std::chrono;

RealTimeThread::RealTimeThread(uint32_t period_us, osPriority priority, uint32_t stack_size)
    : m_period_us(period_us > 0 ? period_us : 1000)
    , m_Thread(priority, stack_size > 0 ? stack_size : OS_STACK_SIZE)
{
    // Start thread
    m_Thread.start(callback(this, &RealTimeThread::threadTask));
    m_running = true;
}

RealTimeThread::~RealTimeThread()
{
    // Stop periodic callbacks first
    disable();

    // Mark thread as no longer running under the same mutex used elsewhere
    {
        MutexLock lock(m_Mutex);
        m_running = false;
    }

    // Wake up thread for final exit check
    m_Thread.flags_set(m_ThreadFlag);

    // Wait for thread to finish with simple timeout
    ThisThread::sleep_for(10ms);

    // Force termination if thread didn't exit cleanly
    if (m_Thread.get_state() != Thread::Deleted) {
        m_Thread.terminate();
    }
}

void RealTimeThread::enable()
{
    MutexLock lock(m_Mutex);
    if (!m_enabled && m_running) {
        // Attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically,
        // which signals the thread to execute
        m_Ticker.attach(callback(this, &RealTimeThread::sendThreadFlag), microseconds{m_period_us});
        m_enabled = true;
    }
}

void RealTimeThread::disable()
{
    MutexLock lock(m_Mutex);
    if (m_enabled) {
        m_Ticker.detach();
        m_enabled = false;
    }
}

bool RealTimeThread::isEnabled() const
{
    MutexLock lock(m_Mutex);
    return m_enabled;
}

void RealTimeThread::executeTask()
{
    // Default implementation - can be overridden
    static uint32_t run_cntr = 0;
    // Avoid printf in real-time threads by default, this is just an example!
    printf("RealTimeThread is enabled and runs for the %lu time\n", run_cntr++);
}

void RealTimeThread::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        bool running;
        bool enabled;
        {
            MutexLock lock(m_Mutex);
            running = m_running;
            enabled = m_enabled;
        }

        // exit thread on shutdown
        if (!running) {
            break;
        }

        // skip execution if disabled, but keep thread alive
        if (enabled) {
            // execute the user-defined task
            executeTask();
        }
    }
}

void RealTimeThread::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
