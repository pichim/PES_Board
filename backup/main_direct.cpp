// Direct SPISlave Implementation - Copy to src/main.cpp for debugging/testing
// This version uses direct SPISlave for real-time byte-by-byte communication
// Compatible with Python spidev xfer2() calls

#include "mbed.h"
#include "SPISlaveCom/SPISlaveCom.h"
#include <cstring>

using namespace std::chrono;

// Simple checksum functions
uint8_t calculate_checksum(const uint8_t* buffer, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= buffer[i];
    }
    return checksum;
}

bool verify_checksum(const uint8_t* buffer, size_t length, uint8_t expected_checksum) {
    return calculate_checksum(buffer, length) == expected_checksum;
}

int main() {
    // Initialize direct SPI slave (real-time communication)
    SPISlave spi(PC_3  /*MOSI*/,
                 PC_2  /*MISO*/,
                 PB_10 /*SCK*/,
                 PB_4  /*CS*/);
    spi.format(8, 0); // 8 bits, mode 0

    // Status LED
    DigitalOut led(LED1);

    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];
    int byte_index = 0;
    bool message_started = false;

    // Data structures
    SPIData received_data;
    SPIData reply_data;
    
    // Initialize reply data with test values
    reply_data.data[0] = 33.93f;
    reply_data.data[1] = 12.34f;
    reply_data.data[2] = 56.78f;

    uint8_t header_received = 0x00;

    Timer timer;
    microseconds time_previous{0};

    timer.start();
    time_previous = timer.elapsed_time();

    printf("SPI Communication Test Started (Direct SPISlave)\r\n");
    printf("Waiting for data from Raspberry Pi...\r\n");

    // Prepare initial reply message
    buffer_tx[0] = SPI_HEADER_SLAVE;
    memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    buffer_tx[SPI_MSG_SIZE - 1] = calculate_checksum(buffer_tx, SPI_MSG_SIZE - 1);
    spi.reply(buffer_tx[0]); // preload first byte

    while (true) {
        if (spi.receive()) {
            uint8_t byte = spi.read();

            if (!message_started) {
                // Look for header
                if (byte == SPI_HEADER_MASTER) {
                    message_started = true;
                    buffer_rx[0] = byte;
                    byte_index = 1;
                    // Prepare reply for next byte
                    if (byte_index < SPI_MSG_SIZE) {
                        spi.reply(buffer_tx[byte_index]);
                    }
                } else {
                    // Invalid header, ignore and prepare for next potential header
                    spi.reply(buffer_tx[0]);
                }
            } else {
                // Collecting message bytes
                buffer_rx[byte_index] = byte;
                byte_index++;

                // Prepare reply for NEXT byte
                if (byte_index < SPI_MSG_SIZE) {
                    spi.reply(buffer_tx[byte_index]);
                }

                // If complete message received
                if (byte_index == SPI_MSG_SIZE) {
                    header_received = buffer_rx[0];
                    uint8_t received_checksum = buffer_rx[SPI_MSG_SIZE - 1];

                    // Verify checksum
                    if (verify_checksum(buffer_rx, SPI_MSG_SIZE - 1, received_checksum)) {
                        // Valid message - extract data
                        memcpy(received_data.data, &buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                        received_data.message_count++;

                        // Measure elapsed time
                        const microseconds time = timer.elapsed_time();
                        const uint32_t delta_time = duration_cast<microseconds>(time - time_previous).count();
                        time_previous = time;
                        received_data.last_delta_time_us = delta_time;

                        // Keep reply data constant - no incrementing
                        // Reply data stays: [33.93, 12.34, 56.78]

                        // Prepare next reply message (same data each time)
                        buffer_tx[0] = SPI_HEADER_SLAVE;
                        memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
                        buffer_tx[SPI_MSG_SIZE - 1] = calculate_checksum(buffer_tx, SPI_MSG_SIZE - 1);

                        // Print results (Python-compatible format)
                        printf("Message: %lu | ", received_data.message_count);
                        printf("Delta Time: %lu us | ", delta_time);
                        printf("Received: [%.2f, %.2f, %.2f] | ", 
                               received_data.data[0], received_data.data[1], received_data.data[2]);
                        printf("Header: 0x%02X | Failed: %lu\r\n", 
                               header_received, received_data.failed_count);

                        // Blink LED to indicate communication
                        led = !led;
                    } else {
                        // Invalid checksum
                        received_data.failed_count++;
                        printf("Checksum failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                               calculate_checksum(buffer_rx, SPI_MSG_SIZE - 1), received_checksum, received_data.failed_count);
                    }

                    // Reset for next message
                    message_started = false;
                    byte_index = 0;
                    spi.reply(buffer_tx[0]); // preload first byte for next message
                }
            }
        }
    }
}
