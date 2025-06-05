#include "SDLogger.h"

SDLogger::SDLogger(PinName mosi,
                   PinName miso,
                   PinName sck,
                   PinName cs,
                   uint8_t num_of_floats) : m_SDWriter(mosi, miso, sck, cs),
                                            m_Thread(osPriorityLow),
                                            m_num_of_floats(num_of_floats)
{
    // validate input parameters
    if (num_of_floats == 0 || num_of_floats > SD_LOGGER_NUM_OF_FLOATS_MAX) {
        printf("SDLogger: Invalid num_of_floats (%d), using default (%d)\n",
            num_of_floats,
            SD_LOGGER_NUM_OF_FLOATS_MAX);
        m_num_of_floats = SD_LOGGER_NUM_OF_FLOATS_MAX;
    }

    // start thread
    m_Thread.start(callback(this, &SDLogger::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &SDLogger::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});

    // open the file
    if (!openFile()) {
        printf("WARNING: Could not open SD file for logging.\n");
    }
}

SDLogger::~SDLogger()
{
    closeFile();
    m_Ticker.detach();
    m_Thread.terminate();
}

void SDLogger::write(const float val)
{
    // add val to the buffer
    m_data[m_float_cntr++] = val;

    // send the data if the buffer is full immediately
    if (m_float_cntr == m_num_of_floats) {
        send();
    }
}

void SDLogger::send()
{
    // return if there is no data to send
    if (m_float_cntr == 0)
        return;

    static bool send_num_of_floats_once = false;
    if (!send_num_of_floats_once) {
        send_num_of_floats_once = true;
        // write current count
        if (!m_SDWriter.writeByte(m_float_cntr)) {
            printf("SDLogger: writing num_of_floats byte failed\n");
        }
    }

    // write the data
    logFloats(m_data, m_float_cntr);
    m_float_cntr = 0;
}

void SDLogger::logFloats(const float* data)
{
    logFloats(data, m_num_of_floats);
}

void SDLogger::logFloats(const float* data, size_t count)
{
    if (!m_file_open) {
        printf("SDLogger: File not open—discarding data.\n");
        return;
    }

    // note: i leave this as an example
    // core_util_critical_section_enter(); 
    // bool ok = pushFloats(data, count);
    // core_util_critical_section_exit();

    m_Mutex.lock();
    bool ok = pushFloats(data, count);
    m_Mutex.unlock();

    if (!ok) {
        // buffer is full
        printf("SDLogger: Buffer overflow, lost data!\n");
    }
}

bool SDLogger::openFile()
{
    // close any existing file first
    if (m_file_open) {
        closeFile();
    }
    // attempt to mount & open
    if (!m_SDWriter.mount()) {
        printf("SDLogger: mount failed\n");
        return false;
    }
    if (!m_SDWriter.openNextFile()) {
        printf("SDLogger: openNextFile failed\n");
        m_SDWriter.unmount(); // cleanup on failure
        return false;
    }
    // note: i leave this in case it is needed in the future
    // // write the "m_num_of_floats" as the first byte once
    // if (!m_SDWriter.writeByte(m_num_of_floats)) {
    //     printf("SDLogger: writing num_of_floats byte failed\n");
    //     return false;
    // }
    m_file_open = true;
    printf("SDLogger: File opened successfully\n");
    return true;
}

void SDLogger::closeFile()
{
    if (m_file_open) {
        m_SDWriter.closeFile();
        m_SDWriter.unmount();
        m_file_open = false;
    }
}

void SDLogger::flushBuffer()
{
    if (!m_file_open) {
        return;
    }

    // drain in chunks
    static constexpr size_t CHUNK_SIZE = 512; // increased from 256 for better throughput
    float tmp[CHUNK_SIZE];

    while (!m_CircularBuffer.empty()) {
        size_t count_to_pop = 0;

        // note: i leave this as an example
        // core_util_critical_section_enter();
        // while ((count_to_pop < CHUNK_SIZE) && (!m_CircularBuffer.empty())) {
        //     m_CircularBuffer.pop(tmp[count_to_pop]);
        //     count_to_pop++;
        // }
        // core_util_critical_section_exit();

        m_Mutex.lock();
        while ((count_to_pop < CHUNK_SIZE) && (!m_CircularBuffer.empty())) {
            m_CircularBuffer.pop(tmp[count_to_pop]);
            count_to_pop++;
        }
        m_Mutex.unlock();

        if (count_to_pop > 0) {
            // write that chunk
            if (!m_SDWriter.writeFloats(tmp, count_to_pop)) {
                printf("SDLogger: writeFloats failed\n");
                // break to avoid infinite loop on persistent errors
                break;
            }
        }
    }
}

bool SDLogger::pushFloats(const float* data, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (m_CircularBuffer.full()) {
            m_overflow_count++;
            return false;
        }
        m_CircularBuffer.push(data[i]);
    }

    // update max buffer usage for diagnostics
    size_t current_usage = m_CircularBuffer.size();
    if (current_usage > m_max_buffer_usage) {
        m_max_buffer_usage = current_usage;
    }
    
    return true;
}

void SDLogger::threadTask()
{
    Timer flush_timer;
    flush_timer.start();

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // write any pending data
        flushBuffer();

        // flush the file so data is physically on sd card
        if (flush_timer.elapsed_time() >= 5s) {
            flush_timer.reset();

            if (m_file_open) {
                bool ok = m_SDWriter.flush();
                if (!ok) {
                    printf("SDLogger: fflush failed\n");
                }
            }
        }
    }
}

void SDLogger::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
