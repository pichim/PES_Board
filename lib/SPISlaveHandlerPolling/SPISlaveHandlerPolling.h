#ifndef SPI_SLAVE_HANDLER_POLLING_H_
#define SPI_SLAVE_HANDLER_POLLING_H_

#include "mbed.h"
#include <cstring>

#include "ThreadFlag.h"

using namespace std::chrono;

#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE  0x45
#define SPI_NUM_FLOATS    3
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// SPIData structure with processing time (names kept as in your new code)
class SPIData
{
public:
    SPIData() { init(); };
    ~SPIData() = default;

    float data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    uint32_t readout_time_us;

    void init()
    {
        for (int i = 0; i < SPI_NUM_FLOATS; i++) data[i] = 0.0f;
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    };
};

class SPISlaveHandlerPolling
{
public:
    // SPI2: PC_3 (MOSI), PC_2 (MISO), PB_10 (SCK), PB_12 (NSS/CS)
    SPISlaveHandlerPolling(PinName mosi = PC_3,
                           PinName miso = PC_2,
                           PinName sck  = PB_10,
                           PinName nss  = PB_12,
                           PinName led_pin = LED1);

    // Processes at most one received byte per call.
    // Returns true exactly when a full frame has finished (valid or invalid).
    bool poll();

    bool has_new_data() const { return new_data_available; }

    void get_received_data(SPIData &out)
    {
        out = received_data;
        new_data_available = false;
    }

    // Access to failed_count for printing in main
    uint32_t get_failed_count() const { return received_data.failed_count; }

private:
    // Replace your calculate_crc8 with a table version (poly 0x07)
    static const uint8_t CRC8_TAB[256];

    static inline uint8_t calculate_crc8(const uint8_t *buffer, size_t length)
    {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < length; ++i) crc = CRC8_TAB[crc ^ buffer[i]];
        return crc;
    }

    static inline bool verify_checksum(const uint8_t *buffer, size_t length, uint8_t expected_crc)
    {
        return calculate_crc8(buffer, length) == expected_crc;
    }

private:
    static constexpr int64_t PERIOD_MUS = 20;

    SPISlave spi;
    DigitalOut led;

    Timer timer;
    microseconds time_previous;

    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];

    SPIData received_data;
    SPIData reply_data;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    int byte_index;
    bool message_started;

    volatile bool new_data_available;

    void threadTask();
    void sendThreadFlag();
};

#endif // SPI_SLAVE_HANDLER_POLLING_H_
