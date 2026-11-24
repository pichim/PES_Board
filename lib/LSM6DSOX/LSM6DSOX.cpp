/**
 * @file LSM6DSOX.cpp
 * @brief Minimal LSM6DSOX SPI driver, based on the previously verified HW test.
 */
#include "LSM6DSOX.h"

namespace {
    // +/-2 g: 0.061 mg/LSB, +/-500 dps: 17.50 mdps/LSB.
    constexpr float ACC_FACTOR = 0.000061f * 9.80665f;
    constexpr float GYRO_FACTOR = 0.01750f * 3.14159265358979323846f / 180.0f;
}

LSM6DSOX::LSM6DSOX(SPI& spi, DigitalOut& cs)
    : m_spi(spi), m_cs(cs)
{
    m_cs = 1;
}

bool LSM6DSOX::begin()
{
    m_initialized = false;
    m_spi.format(8, 3);       // LSM6DSOX SPI mode 3
    m_spi.frequency(1000000); // Conservative, verified 1 MHz
    m_cs = 1;

    if (whoAmI() != EXPECTED_WHO_AM_I) {
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x44); // BDU=1, IF_INC=1
    writeRegister(REG_CTRL1_XL, 0x40); // 104 Hz, +/-2g
    writeRegister(REG_CTRL2_G,  0x44); // 104 Hz, +/-500 dps

    m_initialized =
        readRegister(REG_CTRL3_C) == 0x44 &&
        readRegister(REG_CTRL1_XL) == 0x40 &&
        readRegister(REG_CTRL2_G) == 0x44;

    return m_initialized;
}

uint8_t LSM6DSOX::whoAmI()
{
    return readRegister(REG_WHO_AM_I);
}

void LSM6DSOX::writeRegister(uint8_t address, uint8_t value)
{
    m_spi.lock();
    m_cs = 0;
    m_spi.write(address & 0x7F);
    m_spi.write(value);
    m_cs = 1;
    m_spi.unlock();
}

uint8_t LSM6DSOX::readRegister(uint8_t address)
{
    m_spi.lock();
    m_cs = 0;
    m_spi.write(address | 0x80);
    const uint8_t result = static_cast<uint8_t>(m_spi.write(0x00));
    m_cs = 1;
    m_spi.unlock();
    return result;
}

void LSM6DSOX::readRegisters(uint8_t startAddress, uint8_t* dest, uint8_t count)
{
    m_spi.lock();
    m_cs = 0;
    m_spi.write(startAddress | 0x80);
    for (uint8_t i = 0; i < count; ++i) {
        dest[i] = static_cast<uint8_t>(m_spi.write(0x00));
    }
    m_cs = 1;
    m_spi.unlock();
}

int16_t LSM6DSOX::asInt16(uint8_t low, uint8_t high)
{
    const uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
    return static_cast<int16_t>(raw);
}

bool LSM6DSOX::readSample(Sample& sample)
{
    if (!m_initialized) {
        return false;
    }

    uint8_t bytes[12] = {};
    readRegisters(REG_OUTX_L_G, bytes, sizeof(bytes));

    sample.gyroX = asInt16(bytes[0], bytes[1]) * GYRO_FACTOR;
    sample.gyroY = asInt16(bytes[2], bytes[3]) * GYRO_FACTOR;
    sample.gyroZ = asInt16(bytes[4], bytes[5]) * GYRO_FACTOR;

    sample.accX = asInt16(bytes[6], bytes[7]) * ACC_FACTOR;
    sample.accY = asInt16(bytes[8], bytes[9]) * ACC_FACTOR;
    sample.accZ = asInt16(bytes[10], bytes[11]) * ACC_FACTOR;
    return true;
}
