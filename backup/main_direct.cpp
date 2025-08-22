#include "mbed.h"
#include <cstring>

using namespace std::chrono;

#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE  0x45
#define SPI_NUM_FLOATS    3
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// Replace your calculate_crc8 with a table version (poly 0x07)
static const uint8_t CRC8_TAB[256] = {
    // precomputed for poly 0x07, init 0x00
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

static inline uint8_t calculate_crc8(const uint8_t *buffer, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) crc = CRC8_TAB[crc ^ buffer[i]];
    return crc;
}

// // CRC-8 (poly 0x07, init 0x00)
// static uint8_t calculate_crc8(const uint8_t *buffer, size_t length)
// {
//     uint8_t crc = 0x00;
//     for (size_t i = 0; i < length; ++i) {
//         crc ^= buffer[i];
//         for (uint8_t j = 0; j < 8; ++j) {
//             if (crc & 0x80) crc = (crc << 1) ^ 0x07;
//             else            crc <<= 1;
//         }
//     }
//     return crc;
// }

static bool verify_checksum(const uint8_t *buffer, size_t length, uint8_t expected_crc)
{
    return calculate_crc8(buffer, length) == expected_crc;
}

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

int main()
{
    // SPI2: PC_3 (MOSI), PC_2 (MISO), PB_10 (SCK), PB_12 (NSS/CS)
    SPISlave spi(PC_3, PC_2, PB_10, PB_12);
    spi.format(8, 0); // 8-bit, mode 0

    DigitalOut led(LED1);

    uint8_t buffer_rx[SPI_MSG_SIZE];
    uint8_t buffer_tx[SPI_MSG_SIZE];

    SPIData received_data;
    SPIData reply_data;

    // Match Python test values (or keep any constants you prefer)
    reply_data.data[0] = 11.11f;
    reply_data.data[1] = 22.22f;
    reply_data.data[2] = 33.33f;

    Timer timer;
    microseconds time_previous{0};
    timer.start();
    time_previous = timer.elapsed_time();

    // Prepare initial reply and preload first byte (as in your newer code)
    buffer_tx[0] = SPI_HEADER_SLAVE;
    std::memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);
    spi.reply(buffer_tx[0]); // preload first byte

    int byte_index = 0;
    bool message_started = false;

    printf("SPI Communication (polling) started. Waiting for master...\r\n");

    while (true) {
        if (!spi.receive()) continue;

        const uint8_t byte = spi.read();

        if (!message_started) {
            // Expect exact header from Raspberry Pi
            if (byte == SPI_HEADER_MASTER) {
                message_started = true;
                buffer_rx[0] = byte;
                byte_index = 1;

                // Prime reply for next byte in this same frame
                if (byte_index < SPI_MSG_SIZE) spi.reply(buffer_tx[byte_index]);
            } else {
                // Not our header — keep first reply byte primed
                spi.reply(buffer_tx[0]);
            }
            continue;
        }

        // Collect message bytes
        buffer_rx[byte_index] = byte;
        byte_index++;

        // Prepare reply for NEXT incoming byte
        if (byte_index < SPI_MSG_SIZE) {
            spi.reply(buffer_tx[byte_index]);
        }

        // Full message received
        if (byte_index == SPI_MSG_SIZE) {
            const uint8_t received_checksum = buffer_rx[SPI_MSG_SIZE - 1];

            if (verify_checksum(buffer_rx, SPI_MSG_SIZE - 1, received_checksum)) {
                // Extract three floats (little-endian on STM32 matches Python "<f")
                std::memcpy(received_data.data, &buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                received_data.message_count++;

                const auto now = timer.elapsed_time();
                received_data.last_delta_time_us = duration_cast<microseconds>(now - time_previous).count();
                time_previous = now;

                // Prepare next reply (keep constants or update here if you wish)
                buffer_tx[0] = SPI_HEADER_SLAVE;
                std::memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
                buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);

                // Blink + print
                led = !led;
                printf("Message: %lu | Delta Time: %lu us | "
                       "Received: [%.2f, %.2f, %.2f] | "
                       "Header: 0x%02X | Failed: %lu\r\n",
                       received_data.message_count,
                       received_data.last_delta_time_us,
                       received_data.data[0], received_data.data[1], received_data.data[2],
                       SPI_HEADER_SLAVE, // what we send back
                       received_data.failed_count);
            } else {
                received_data.failed_count++;
                const uint8_t expected = calculate_crc8(buffer_rx, SPI_MSG_SIZE - 1);
                printf("CRC failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                       expected, received_checksum, received_data.failed_count);
            }

            // Reset for next frame; always preload first reply byte again
            message_started = false;
            byte_index = 0;
            spi.reply(buffer_tx[0]);
        }
    }
}
