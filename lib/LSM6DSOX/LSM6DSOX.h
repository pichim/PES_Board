/**
 * @file LSM6DSOX.h
 * @brief SPI driver for LSM6DSOX accelerometer + gyroscope (Mbed OS).
 *
 * Sensor-frame axes; acceleration in m/s^2, angular velocity in rad/s.
 * No orientation transformation or calibration is applied here.
 */
#pragma once

#include "mbed.h"
#include <cstdint>

class LSM6DSOX {
public:
    struct Sample {
        float accX = 0.0f;
        float accY = 0.0f;
        float accZ = 0.0f;
        float gyroX = 0.0f;
        float gyroY = 0.0f;
        float gyroZ = 0.0f;
    };

    /** The SPI and chip-select objects must outlive the driver. */
    LSM6DSOX(SPI& spi, DigitalOut& cs);

    /** Verify WHO_AM_I (0x6C), configure BDU + auto-increment, 104 Hz, +/-2g, +/-500 dps. */
    bool begin();

    /** Returns the silicon ID, without changing configuration. */
    uint8_t whoAmI();

    /** Read six axes in one 12-byte burst. False if begin() has not succeeded. */
    bool readSample(Sample& sample);

private:
    static constexpr uint8_t REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t REG_CTRL1_XL = 0x10;
    static constexpr uint8_t REG_CTRL2_G  = 0x11;
    static constexpr uint8_t REG_CTRL3_C  = 0x12;
    static constexpr uint8_t REG_OUTX_L_G = 0x22;
    static constexpr uint8_t EXPECTED_WHO_AM_I = 0x6C;

    SPI& m_spi;
    DigitalOut& m_cs;
    bool m_initialized = false;

    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void readRegisters(uint8_t startAddress, uint8_t* dest, uint8_t count);
    static int16_t asInt16(uint8_t low, uint8_t high);
};
