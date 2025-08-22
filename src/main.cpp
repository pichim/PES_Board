// #include "mbed.h"
// #include "SPISlaveHandler.h"

// int main() {
//     SPISlaveHandler handler(
//         PC_3,   // MOSI  (SPI2_MOSI)
//         PC_2,   // MISO  (SPI2_MISO)
//         PB_10,  // SCLK  (SPI2_SCK)
//         PB_12   // CS/NSS (SPI2_NSS)  <-- hardware CS
//     );

//     while (true) {
//         SPIData data;

//         if (handler.has_new_data()) {
//             handler.get_received_data(data);

//             printf("Message: %lu | Delta Time: %lu us | "
//                    "Received: [%.2f, %.2f, %.2f] | "
//                    "Header: 0x%02X | Failed: %lu | "
//                    "Readout Time: %lu us\r\n",
//                    data.message_count, data.last_delta_time_us,
//                    data.data[0], data.data[1], data.data[2],
//                    SPI_HEADER_SLAVE, data.failed_count,
//                    data.readout_time_us);
//         }

//         ThisThread::sleep_for(1ms);
//     }
// }



// #include "mbed.h"
// #include "SPISlaveHandlerPolling.h"

// int main()
// {
//     SPISlaveHandlerPolling handler(PC_3, PC_2, PB_10, PB_12, LED1);

//     printf("SPI Communication (polling) started. Waiting for master...\r\n");

//     while (true) {
//         // const bool frame_done = handler.poll(); // processes at most one byte

//         // if (frame_done && handler.has_new_data()) {
//         if (handler.has_new_data()) {
//             SPIData d;
//             handler.get_received_data(d);

//             printf("Message: %lu | Delta Time: %lu us | "
//                    "Received: [%.2f, %.2f, %.2f] | "
//                    "Header: 0x%02X | Failed: %lu\r\n",
//                    d.message_count,
//                    d.last_delta_time_us,
//                    d.data[0], d.data[1], d.data[2],
//                    SPI_HEADER_SLAVE, // what we send back
//                    handler.get_failed_count());
//         }
//         // no sleep — keep polling tight
//     }
// }



#include "mbed.h"
#include "SPISlaveDMA.h"

int main()
{
    printf("SPI Communication started. Waiting for master...\n");

    // Create driver (SPI2 default pins on NUCLEO_F446RE)
    SPIData spiData;
    SpiSlaveDMA spiSlaveDMA;

    // Explicitly start DMA + worker thread/ticker; bail out if it fails
    if (!spiSlaveDMA.start()) {
        printf("[SPI] start() failed — check wiring, pin mapping, or DMA state.\r\n");
        while (true) {
            ThisThread::sleep_for(500ms);
        }
    }

    while (true) {
        if (spiSlaveDMA.has_new_data()) {
            spiSlaveDMA.get_last(spiData);

            printf("Message: %lu | Delta Time: %lu us | "
                   "Received: [%.2f, %.2f, %.2f] | "
                   "Header: 0x%02X | Failed: %lu | "
                   "Readout Time: %lu us\r\n",
                   spiData.message_count, spiData.last_delta_time_us,
                   spiData.data[0], spiData.data[1], spiData.data[2],
                   SPI_HEADER_SLAVE, spiData.failed_count,
                   spiData.readout_time_us);
        }

        // Example: you can update the reply payload any time
        // spiSlaveDMA.set_reply_data(11.11f, 22.22f, 33.33f);

        ThisThread::sleep_for(5ms);
    }
}

