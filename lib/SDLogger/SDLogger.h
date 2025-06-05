/**
 * @file SDLogger.h
 * @brief Defines the SDLogger class for logging floating-point data to an SD card.
 *
 * The SDLogger class buffers floating-point samples using a ring buffer and writes them 
 * to an SD card in bursts from a low-priority thread. It manages thread creation, 
 * periodic flushing, and synchronization, providing a simple and efficient logging interface.
 *
 * Maximum throughput depends on SD card speed and buffer size. If the buffer fills up, 
 * additional data is discarded. By default, data is flushed to disk every 5 seconds 
 * to reduce data loss in case of power failure.
 *
 * @dependencies
 * This class relies on:
 * - **SDWriter**: Handles SD card mounting, file creation, and binary writes.
 * - **CircularBuffer<float>**: Buffers incoming float data.
 * - **ThreadFlag** and **Ticker**: Schedule periodic buffer flushing.
 * - **Mutex**: Ensures thread-safe access to the buffer.
 *
 * @usage
 * 1. Create an `SDLogger` instance by specifying SPI pins and the number of floats per record.
 * 2. Use `write(float val)` to queue individual float values.
 * 3. When the required number of floats per record is reached, the data is automatically sent.
 * 4. The `send()` method can also be called manually to force immediate writing.
 *
 * @example
 * ```
 * SDLogger sd_logger(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN);
 *
 * // write data to sd card
 * for (int i = 0; i < 50; i++) {
 *     float value = static_cast<float>(i);
 *     sd_logger.write(value);
 * }
 *
 * // send data to sd card
 * sd_logger.send();
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef SD_LOGGER_H_
#define SD_LOGGER_H_

#include "mbed.h"

#include "SDWriter.h"
#include "ThreadFlag.h"

#define SD_LOGGER_NUM_OF_FLOATS_MAX 100 // tested 22 floats at 500 Hz

/**
 * A minimal thread-based SD logger that:
 * - Uses a larger CircularBuffer<float, BUFFER_SIZE>.
 * - Logs data in bursts from a low-priority thread.
 * - Flushes once per second so data is physically written.
 * - Prints a message if the buffer is full or if an SD write fails.
 * - Writes a "m_num_of_floats" byte at the file start (like a header).
 */
class SDLogger
{
public:
/**
 * @param mosi          SPI MOSI pin
 * @param miso          SPI MISO pin
 * @param sck           SPI SCK pin
 * @param cs            SPI CS pin
 * @param num_of_floats Number of floats per record to write to SD card.
 *                      Must be <= SD_LOGGER_NUM_OF_FLOATS_MAX
 */
    explicit SDLogger(PinName mosi,
                      PinName miso,
                      PinName sck,
                      PinName cs,
                      uint8_t num_of_floats = SD_LOGGER_NUM_OF_FLOATS_MAX);
    virtual ~SDLogger();

    // write float values one by one (appends to the ring buffer automatically, but you need to write m_num_of_floats floats)
    void write(const float val);
    // send the data immediately, this will be triggered automatically if you hav writte num_of_floats floats already
    void send();

private:
    // Increase buffer size for higher throughput / less overflow
    // static const size_t BUFFER_SIZE = 8192; // 8k floats = 32kB
    // static const size_t BUFFER_SIZE = 4096; // 4k floats = 16kB
    static const size_t BUFFER_SIZE = 2048; // 2k floats = 8kB
    // static const size_t BUFFER_SIZE = 1024; // e.g. 4kB
    static constexpr int64_t PERIOD_MUS = 20000; // 20 ms period

    Mutex m_Mutex; // mutex to protect the ring buffer
    CircularBuffer<float, BUFFER_SIZE> m_CircularBuffer;
    SDWriter m_SDWriter;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    uint8_t m_num_of_floats;
    uint8_t m_float_cntr{0};
    bool m_file_open{false};
    float m_data[SD_LOGGER_NUM_OF_FLOATS_MAX];

    // buffer monitoring
    size_t m_max_buffer_usage{0};
    uint32_t m_overflow_count{0};

    // log some float data (appends to ring buffer)
    void logFloats(const float* data);
    // log some float data (appends to ring buffer)
    void logFloats(const float* data, size_t count);
    // opens a new file on the SD card, writes the "m_num_of_floats" as a header byte
    bool openFile();
    // closes the file
    void closeFile();
    // helper to drain the buffer in chunks
    void flushBuffer();
    // helper to push floats into the ring buffer
    bool pushFloats(const float* data, size_t count);

    void threadTask();
    void sendThreadFlag();

    // diagnostics
    size_t getBufferUsage() const { return m_CircularBuffer.size(); }
    size_t getMaxBufferUsage() const { return m_max_buffer_usage; }
    uint32_t getOverflowCount() const { return m_overflow_count; }
    void resetDiagnostics() { m_max_buffer_usage = 0; m_overflow_count = 0; }
};
#endif /* SD_LOGGER_H_ */
