#include "SPISlaveHandler.h"

int main() {
    SPISlaveHandler handler(
        PC_3,   // MOSI
        PC_2,   // MISO
        PB_10,  // SCLK
        PB_4    // CS
    );

    while (true) {
        SPIData data;

        if (handler.has_new_data()) {
            handler.get_received_data(data);

            printf("Message: %lu | Delta Time: %lu us | "
                   "Received: [%.2f, %.2f, %.2f] | "
                   "Header: 0x%02X | Failed: %lu | "
                   "Readout Time: %lu us\r\n",
                   data.message_count, data.last_delta_time_us,
                   data.data[0], data.data[1], data.data[2],
                   SPI_HEADER_SLAVE, data.failed_count,
                   data.readout_time_us);
        }

        ThisThread::sleep_for(1ms);
    }
}
