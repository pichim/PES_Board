#include "mbed.h"
#include "PESBoardPinMap.h"

int main()
{
    BufferedSerial serial_port(USBTX, USBRX, 115200);
    I2C i2c(PB_IMU_SDA, PB_IMU_SCL);
    i2c.frequency(100000);

    printf("Scanning I2C bus on SDA=%d SCL=%d\n", PB_IMU_SDA, PB_IMU_SCL);

    bool found_device = false;

    for (int addr_7bit = 0x08; addr_7bit <= 0x77; addr_7bit++) {
        char dummy = 0;
        int result = i2c.write(addr_7bit << 1, &dummy, 0);

        if (result == 0) {
            found_device = true;
            printf("Found I2C device at 0x%02X\n", addr_7bit);
        }
    }

    if (!found_device) {
        printf("No I2C device responded.\n");
    }

    while (true) {
        thread_sleep_for(1000);
    }
}
