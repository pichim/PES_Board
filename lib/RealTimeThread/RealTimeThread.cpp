#include "RealTimeThread.h"

RealTimeThread::RealTimeThread(uint32_t period_us,
                               osPriority priority,
                               uint32_t stack_size) : m_period_us(period_us > 0 ? period_us : 1000)
                                                    , m_Thread(priority, stack_size > 0 ? stack_size : OS_STACK_SIZE)
{
    // start thread
    m_Thread.start(callback(this, &RealTimeThread::threadTask));
    m_running = true;
}

RealTimeThread::~RealTimeThread()
{
    // graceful shutdown - disable ticker first
    disable();

    // signal thread to exit
    m_running = false;

    // wake up thread for final exit check
    m_Thread.flags_set(m_ThreadFlag);

    // wait for thread to finish with simple timeout
    ThisThread::sleep_for(10ms);
    
    // force termination if thread didn't exit cleanly
    if (m_Thread.get_state() != Thread::Deleted) {
        m_Thread.terminate();
    }
}

void RealTimeThread::enable()
{
    m_Mutex.lock();
    if (!m_enabled && m_running) {
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

void RealTimeThread::executeTask()
{
    // default implementation - can be overridden
    static uint32_t run_cntr = 0;
    // avoid printf in real-time threads by default, this is just an example!
    printf("RealTimeThread is enabled and runs for the %lu time\n", run_cntr++);
}

void RealTimeThread::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        
        // exit thread on shutdown
        if (!m_running) {
            break;
        }

        // skip execution if disabled, but keep thread alive
        if (m_enabled) {
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
