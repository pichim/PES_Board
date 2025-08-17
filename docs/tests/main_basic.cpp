// Basic SPISlave Implementation - Copy to src/main.cpp for simple testing
// This is the original simple version from today's development
// Everything self-contained in one file with basic 3-float communication

#include "mbed.h"
#include <cstring>

using namespace std::chrono;

// Protocol constants
#define SPI_HEADER_MASTER 0x55  // Raspberry Pi header
#define SPI_HEADER_SLAVE  0x45  // Nucleo header
#define SPI_NUM_FLOATS    3     // Number of float values in each message
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1)  // header + floats + checksum

// Simple data structure
struct SPIData {
    float data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    
    SPIData() : message_count(0), failed_count(0), last_delta_time_us(0) {
        for (int i = 0; i < SPI_NUM_FLOATS; i++) {
            data[i] = 0.0f;
        }
    }
};

// CRC-8 (polynomial 0x07, initial value 0x00)
uint8_t calculate_crc8(const uint8_t* buffer, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF; // ensure 8-bit result
        }
    }
    return crc;
}

bool verify_checksum(const uint8_t* buffer, size_t length, uint8_t expected_crc) {
    return calculate_crc8(buffer, length) == expected_crc;
}

int main() {
    // Initialize SPI slave
    SPISlave spi(PC_3  /*MOSI*/,
                 PC_2  /*MISO*/,
                 PB_10 /*SCK*/,
                 PB_4  /*CS*/);
    spi.format(8, 0); // 8 bits, mode 0

    // Status LED
    DigitalOut led(LED1);

    // Communication buffers
    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];
    int byte_index = 0;
    bool message_started = false;

    // Data structures
    SPIData received_data;
    SPIData reply_data;
    
    // Initialize reply data with simple test values
    reply_data.data[0] = 11.11f;
    reply_data.data[1] = 22.22f;
    reply_data.data[2] = 33.33f;

    // Timing
    Timer timer;
    microseconds time_previous{0};
    timer.start();
    time_previous = timer.elapsed_time();

    printf("Basic SPI Communication Started\r\n");
    printf("Using 3 floats, header validation, and checksum verification\r\n");
    printf("Reply data: [%.2f, %.2f, %.2f]\r\n", 
           reply_data.data[0], reply_data.data[1], reply_data.data[2]);

    // Prepare initial reply message
    buffer_tx[0] = SPI_HEADER_SLAVE;
    memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);
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
                    uint8_t header_received = buffer_rx[0];
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

                        // Simple output
                        printf("Message: %lu | Time: %lu us | "
                               "Received: [%.2f, %.2f, %.2f] | "
                               "Header: 0x%02X | Failed: %lu\r\n",
                               received_data.message_count, delta_time,
                               received_data.data[0], received_data.data[1], received_data.data[2],
                               header_received, received_data.failed_count);

                        // Blink LED to indicate communication
                        led = !led;
                    } else {
                        // Invalid checksum
                        received_data.failed_count++;
                        printf("Checksum failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                               calculate_crc8(buffer_rx, SPI_MSG_SIZE - 1), received_checksum, received_data.failed_count);
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
