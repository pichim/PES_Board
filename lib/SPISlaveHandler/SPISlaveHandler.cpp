#include "SPISlaveHandler.h"

using namespace std::chrono;

SPISlaveHandler::SPISlaveHandler(PinName mosi,
                                 PinName miso,
                                 PinName sclk,
                                 PinName cs_pin,
                                 PinName led_pin) : spi(mosi, miso, sclk, cs_pin)
                                                  , cs(cs_pin)
                                                  , led(led_pin)
{
    spi.format(8, 0);
    cs.mode(PullUp);
    cs.fall(callback(this, &SPISlaveHandler::on_cs_fall));

    reply_data.data[0] = 11.11f;
    reply_data.data[1] = 22.22f;
    reply_data.data[2] = 33.33f;

    prepare_reply();
    timer.start();
    time_previous = timer.elapsed_time();
    new_data_available = false;
}

bool SPISlaveHandler::has_new_data() const
{
    return new_data_available;
}

void SPISlaveHandler::get_received_data(SPIData &out)
{
    core_util_critical_section_enter();
    out = received_data;
    new_data_available = false;
    core_util_critical_section_exit();
}

void SPISlaveHandler::on_cs_fall()
{
    // Start processing time measurement
    const auto start_time = timer.elapsed_time();

    for (int i = 1; i < SPI_MSG_SIZE; ++i) {
        while (!spi.receive()) {
            // Wait for data to be available
        }
        buffer_rx[i - 1] = spi.read();
        spi.reply(buffer_tx[i]);
    }

    while (!spi.receive()) {
        // Wait for the last byte to be available
    }
    buffer_rx[SPI_MSG_SIZE - 1] = spi.read();

    const uint8_t received_checksum = buffer_rx[SPI_MSG_SIZE - 1];

    if (verify_checksum(buffer_rx, SPI_MSG_SIZE - 1, received_checksum)) {
        memcpy(received_data.data, &buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
        received_data.message_count++;

        const auto now = timer.elapsed_time();
        const auto delta_us = duration_cast<microseconds>(now - time_previous).count();
        received_data.last_delta_time_us = delta_us;
        time_previous = now;

        prepare_reply(); // now included in readout time

        const auto end = timer.elapsed_time();
        received_data.readout_time_us = duration_cast<microseconds>(end - start_time).count();

        led = !led;
        new_data_available = true;
    } else {
        received_data.failed_count++;
    }
}

void SPISlaveHandler::prepare_reply()
{
    buffer_tx[0] = SPI_HEADER_SLAVE;
    memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);
    spi.reply(buffer_tx[0]);
}

uint8_t SPISlaveHandler::calculate_crc8(const uint8_t *buffer, size_t length)
{
    uint8_t crc = 0x00; // Initial value
    for (size_t i = 0; i < length; ++i) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // Polynomial 0x07
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool SPISlaveHandler::verify_checksum(const uint8_t *buffer, size_t length, uint8_t expected_crc)
{
    return calculate_crc8(buffer, length) == expected_crc;
}
