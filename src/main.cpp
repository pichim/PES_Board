/**
 * TODO
 * - Make setReplyData usefull
 * - Check thread priority
 * - SPI pins are configurable via the SpiSlaveDMA constructor.
 */

#include "mbed.h"
#include "SPISlaveDMA.h"

int main()
{
    printf("SPI Communication started. Waiting for master...\n");

    // Create driver (explicit SPI2 pins).
    // Valid SPI2 (AF5) options on NUCLEO_F446RE:
    //   SCK  : PB_10, PB_13
    //   MISO : PB_14, PC_2
    //   MOSI : PB_15, PC_3
    //   NSS  : PB_9,  PB_12
    //
    // Example wiring used here: MOSI=PC_3, MISO=PC_2, SCK=PB_10, NSS=PB_12
    // MOSI, MISO, SCK, NSS
    SpiSlaveDMA spiSlaveDMA(PC_3, PC_2, PB_10, PB_12);

    // Explicitly start DMA + worker thread; bail out if it fails
    if (!spiSlaveDMA.start()) {
        printf("[SPI] start() failed — check wiring, pin mapping, or DMA state.\n");
        while (true) {
            ThisThread::sleep_for(500ms);
        }
    }

    while (true) {
        if (spiSlaveDMA.hasNewData()) {
            SpiData spiData = spiSlaveDMA.getSPIData();

            printf("Message: %lu | Delta Time: %lu us | "
                   "Received: [%.2f, %.2f, %.2f, %.2f, %.2f] | "
                   "Header: 0x%02X | Failed: %lu | "
                   "Readout Time: %lu us\n",
                   spiData.message_count, spiData.last_delta_time_us,
                   spiData.data[0], spiData.data[1], spiData.data[2], spiData.data[3], spiData.data[4],
                   SPI_HEADER_SLAVE, spiData.failed_count,
                   spiData.readout_time_us);
        }

        // Example: you can update the reply payload any time (copied atomically into next TX)
        // spiSlaveDMA.setReplyData(11.11f, 22.22f, 33.33f);

        ThisThread::sleep_for(5ms);
    }
}
