/*
================================================================================
 SPI Wiring (Nucleo F446RE – Slave)
 --------------------------------------------------------------------------------
 Nucleo Pin   | Function | Connected to Raspberry Pi 5 (J8 Header Pin)
---------------------------------------------------------------------------------
  PC_3        | MOSI     | GPIO10 (Pin 19)
  PC_2        | MISO     | GPIO9  (Pin 21)
  PB_10       | SCLK     | GPIO11 (Pin 23)
  PB_4        | CS (NSS) | GPIO8  (Pin 24)
  GND         | GND      | Pin 25 (Ground)
  3V3         | VCC      | Pin 1  (3.3V logic reference)
================================================================================
 Notes:
 - Nucleo runs as SPI SLAVE.
 - Raspberry Pi 5 runs as SPI MASTER.
 - Logic levels: Pi = 3.3V, Nucleo = 3.3V → direct connection is safe.
================================================================================
*/

#ifndef SPI_SLAVE_HANDLER_H_
#define SPI_SLAVE_HANDLER_H_

#include "mbed.h"
// #include <cstring>
// #include <chrono>

#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE 0x45
#define SPI_NUM_FLOATS 3
#define SPI_MSG_SIZE (1 + SPI_NUM_FLOATS * 4 + 1)

using namespace std::chrono;

// SPIData structure with processing time
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
        for (int i = 0; i < SPI_NUM_FLOATS; i++) {
            data[i] = 0.0f;
        }
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    };
};

class SPISlaveHandler
{
public:
    SPISlaveHandler(PinName mosi, PinName miso, PinName sclk, PinName cs_pin, PinName led_pin = LED1);

    bool has_new_data() const;

    void get_received_data(SPIData &out);

private:
    SPISlave spi;
    InterruptIn cs;
    DigitalOut led;
    Timer timer;
    microseconds time_previous;

    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];

    SPIData received_data;
    SPIData reply_data;

    volatile bool new_data_available = false;

    void on_cs_fall();
    void prepare_reply();
    uint8_t calculate_crc8(const uint8_t *buffer, size_t length);
    bool verify_checksum(const uint8_t *buffer, size_t length, uint8_t expected_crc);
};

#endif // SPI_SLAVE_HANDLER_H_
