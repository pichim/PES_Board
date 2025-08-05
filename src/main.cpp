// {
//     "target_overrides": {
//         "*": {
//             "target.features_add": ["STORAGE"],
//             "target.components_add": ["SD"],
//             "target.printf_lib": "std",
//             "platform.stdio-baud-rate": 115200,
//             "sd.INIT_FREQUENCY": 100000
//         }
//     },
//     "config": {
//         "main-stack-size": {
//             "value": 16384
//         }
//     }
// }

// cmake --build build/NUCLEO_F446RE-Develop --target flash-PES_Board
// cmake --build build/NUCLEO_F446RE-Develop --target clean
// cmake --build build/NUCLEO_F446RE-Develop

// | Function | NUCLEO Pin | Pi GPIO | Pi Header Pin |
// | -------- | ---------- | ------- | ------------- |
// | MOSI     | `PC_3`     | GPIO10  | Pin 19        |
// | MISO     | `PC_2`     | GPIO9   | Pin 21        |
// | SCK      | `PB_10`    | GPIO11  | Pin 23        |
// | CS       | `PB_4`     | GPIO8   | Pin 24        |
// | GND      | GND        | GND     | Pin 6         |

#include "mbed.h"
#include <cstring>

// SPI2 pins: mosi, miso, sclk, ssel (CS)
SPISlave spi(PC_3, PC_2, PB_10, PB_4);

float received_value = 0.0f;
float reply_value = 33.93f;

uint8_t rx_buffer[4];
uint8_t tx_buffer[4];
int byte_index = 0;

Timer cs_timer;

int main() {
    spi.format(8, 0);        // 8 bits, mode 0
    // spi.frequency(500000);   // Match Pi

    printf("SPISlave ready...\r\n");

    // Prepare response bytes
    memcpy(tx_buffer, &reply_value, sizeof(reply_value));
    spi.reply(tx_buffer[0]);  // preload first byte

    cs_timer.start();
    uint32_t last_time_us = cs_timer.read_us();

    while (true) {
        if (spi.receive()) {
            // Read one byte sent by Pi
            uint8_t byte = spi.read();
            rx_buffer[byte_index] = byte;

            // Prepare reply for NEXT byte
            byte_index++;
            if (byte_index < 4) {
                spi.reply(tx_buffer[byte_index]);
            }

            // If all 4 bytes received, decode and reset
            if (byte_index == 4) {
                memcpy(&received_value, rx_buffer, sizeof(received_value));

                // Prepare next reply if desired
                // e.g., reply_value += 1.0f;
                memcpy(tx_buffer, &reply_value, sizeof(reply_value));
                byte_index = 0;
                spi.reply(tx_buffer[0]);  // restart

                uint32_t now_us = cs_timer.read_us();
                uint32_t delta_us = now_us - last_time_us;
                last_time_us = now_us;
                printf("dt: %lu us | Bytes: ", delta_us);
                for (int i = 0; i < 4; ++i) {
                    printf("%02X ", rx_buffer[i]);
                }
                printf("| Float: %.2f\r\n", received_value);
            }
        }
    }
}


// #include "mbed.h"
// #include <cstring>

// // SPI2 pins: MOSI, MISO, SCLK, SSEL (CS)
// SPISlave spi(PC_3, PC_2, PB_10, PB_4);

// float received_value = 0.0f;
// float reply_value = 33.93f;

// uint8_t rx_buffer[4];
// uint8_t tx_buffer[4];
// int byte_index = 0;

// Timer cs_timer;

// int main() {
//     spi.format(8, 0);        // 8 bits per transfer, SPI mode 0
//     spi.frequency(500000);   // Match Raspberry Pi's speed

//     printf("SPISlave ready...\r\n");

//     // Prepare reply value
//     memcpy(tx_buffer, &reply_value, sizeof(reply_value));
//     spi.reply(tx_buffer[0]);  // Preload first byte

//     cs_timer.start();
//     uint32_t last_time_us = cs_timer.read_us();

//     while (true) {
//         if (spi.receive()) {
//             uint32_t now_us = cs_timer.read_us();
//             uint32_t delta_us = now_us - last_time_us;
//             last_time_us = now_us;

//             // Read byte and store
//             uint8_t byte = spi.read();
//             rx_buffer[byte_index] = byte;

//             // Prepare next byte for reply
//             byte_index++;
//             if (byte_index < 4) {
//                 spi.reply(tx_buffer[byte_index]);
//             }

//             // Full float received
//             if (byte_index == 4) {
//                 memcpy(&received_value, rx_buffer, sizeof(received_value));

//                 printf("Δt: %lu us | Bytes: ", delta_us);
//                 for (int i = 0; i < 4; ++i) {
//                     printf("%02X ", rx_buffer[i]);
//                 }
//                 printf("| Float: %.2f\r\n", received_value);

//                 // Optionally update reply_value here
//                 // reply_value += 1.0f;

//                 memcpy(tx_buffer, &reply_value, sizeof(reply_value));
//                 byte_index = 0;
//                 spi.reply(tx_buffer[0]);  // Restart reply cycle
//             }
//         }
//     }
// }


// #include "mbed.h"
// #include <cstring>

// // SPI2 in hardware slave mode: mosi, miso, sclk, ssel
// SPISlave spi(PC_3, PC_2, PB_10, PB_4);  // MOSI, MISO, SCLK, CS

// float received_value = 0.0f;
// const float response_value = 12.45f;
// uint8_t rx_buffer[4];
// uint8_t tx_buffer[4];
// int byte_index = 0;

// Timer cs_timer;

// int main() {
//     spi.format(8, 0);        // 8-bit, mode 0
//     spi.frequency(500000);   // Match Raspberry Pi

//     printf("SPISlave ready...\r\n");

//     cs_timer.start();
//     uint32_t last_cs_us = cs_timer.read_us();

//     // Pre-load the first response byte (very important!)
//     std::memcpy(tx_buffer, &response_value, sizeof(response_value));
//     spi.reply(tx_buffer[0]);  // First byte of response

//     while (true) {
//         if (spi.receive()) {
//             uint32_t now_us = cs_timer.read_us();
//             uint32_t delta_us = now_us - last_cs_us;
//             last_cs_us = now_us;

//             uint8_t byte = spi.read();
//             rx_buffer[byte_index] = byte;

//             // Load the *next* byte for the next clock cycle
//             byte_index++;
//             if (byte_index < 4) {
//                 spi.reply(tx_buffer[byte_index]);
//             }

//             if (byte_index == 4) {
//                 // Process the full float after 4 bytes
//                 memcpy(&received_value, rx_buffer, sizeof(received_value));
//                 printf("Δt: %lu us | RX: ", delta_us);
//                 for (int i = 0; i < 4; ++i) {
//                     printf("%02X ", rx_buffer[i]);
//                 }
//                 printf("| Float: %.2f\r\n", received_value);

//                 // Prepare next response
//                 memcpy(tx_buffer, &response_value, sizeof(response_value));
//                 byte_index = 0;
//                 spi.reply(tx_buffer[0]);  // Start again
//             }
//         }
//     }
// }


// #include "mbed.h"
// #include <cstring>

// // Use SPI2 hardware slave mode: MOSI, MISO, SCK, SSEL (CS)
// SPISlave spi(PC_3, PC_2, PB_10, PB_4);  // mosi, miso, sclk, ssel

// float received_value = 0.0f;
// uint8_t rx_buffer[4];

// Timer cs_timer;

// int main() {
//     spi.format(8, 0);        // 8-bit, mode 0
//     spi.frequency(500000);   // Match Raspberry Pi

//     printf("SPISlave ready...\r\n");

//     cs_timer.start();
//     uint32_t last_cs_us = cs_timer.read_us();
//     int byte_index = 0;

//     while (true) {
//         if (spi.receive()) {
//             uint32_t now_us = cs_timer.read_us();
//             uint32_t delta_us = now_us - last_cs_us;
//             last_cs_us = now_us;

//             uint8_t byte = spi.read();
//             rx_buffer[byte_index++] = byte;

//             if (byte_index == 4) {
//                 printf("Time since last CS: %lu us\r\n", delta_us);

//                 printf("Bytes received: ");
//                 for (int i = 0; i < 4; ++i) {
//                     printf("%02X ", rx_buffer[i]);
//                 }
//                 printf("\r\n");

//                 memcpy(&received_value, rx_buffer, sizeof(received_value));
//                 printf("Decoded float: %.2f\r\n", received_value);

//                 byte_index = 0;  // Reset for next transaction
//             }
//         }
//     }
// }
