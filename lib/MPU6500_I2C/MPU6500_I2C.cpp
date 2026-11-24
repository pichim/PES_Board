#include "MPU6500_I2C.h"

MPU6500_I2C::MPU6500_I2C(I2C &i2c) : m_i2c(i2c) {}

bool MPU6500_I2C::init()
{
    m_i2c.frequency(400000);
    ThisThread::sleep_for(chrono::milliseconds(150));

    // Chip Reset Sequence - P.42 in RM-MPU-6500A-00  R2.1
    writeRegister(PWR_MGMT_1, 0x80);
    ThisThread::sleep_for(chrono::milliseconds(150));
    writeRegister(SIGNAL_PATH_RESET, 0x07);
    ThisThread::sleep_for(chrono::milliseconds(150));

    // Default to internal 20MHz clock source until hyro oscillator stabilizes
    writeRegister(PWR_MGMT_1, 0x00);
    ThisThread::sleep_for(chrono::milliseconds(15));
    // Set Gyro oscillator as the clock source. This is done for more accuracy when calculating angles. Datasheet section 4.12 Clocking
    writeRegister(PWR_MGMT_1, 0x01);
    ThisThread::sleep_for(chrono::milliseconds(15));

    // Set Gyro scale to +-250 dps
    writeRegister(GYRO_CONFIG, 0x00);
    ThisThread::sleep_for(chrono::milliseconds(15));
    // Set Gyro scale to +-2g dps
    writeRegister(ACCEL_CONFIG, 0x00);
    ThisThread::sleep_for(chrono::milliseconds(15));

    // Set Gyro Digital Low Pass Filter (DLPF) Bandwidth to 41 Hz (5.9 ms) and the Temperature BW to 42 Hz.
    // Ensure the FCHOICE_B bits in the MPUREG_GYRO_CONFIG register are set to 00 for this to work, specially when configuring the gyro range later on.
    writeRegister(CONFIG, 0x03);
    ThisThread::sleep_for(chrono::milliseconds(15));
    // Set Accelerometer Digital Low Pass Filter (DLPF) Bandwidth to 184 Hz (5.80 ms). Used to be at 41 Hz, changed it bc of the delay time.
    // Ensure the ACCEL_FCHOICE_B bit in the MPUREG_ACCEL_CONFIG_2 register are set to 0 for this to work.
    writeRegister(ACCEL_CONFIG_2, 0x01);
    ThisThread::sleep_for(chrono::milliseconds(15));

    // Sets the Sampler rate divider to 0, which would result in a sample rate of 1Hz for the gyro when DLPF is enabled.
    writeRegister(SMPLRT_DIV, 0x00);
    ThisThread::sleep_for(chrono::milliseconds(15));

    // Interrupt Configurator
    writeRegister(INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    ThisThread::sleep_for(chrono::milliseconds(15));
    writeRegister(INT_ENABLE, 1 << 0);
    ThisThread::sleep_for(chrono::milliseconds(15));

    return true;
}

bool MPU6500_I2C::configuration(void)
{
    // set gyro and acc lpf to highest common setting
    // gyro: BW 184 Hz (delay 2.9 ms) at Fs 1 kHz
    // acc : BW 184 Hz (delay 5.8 ms) at Fs 1 kHz
    writeRegister(CONFIG, DLPF_CFG_184HZ_188HZ_NA);
    ThisThread::sleep_for(chrono::milliseconds(20));
    writeRegister(ACCEL_CONFIG_2, DLPF_CFG_184HZ_188HZ_NA);
    ThisThread::sleep_for(chrono::milliseconds(20));

    // set gyro scale and acc range
    set_gyro_scale(FS_SEL_2000DPS);
    ThisThread::sleep_for(chrono::milliseconds(20));
    set_acc_scale(AFS_SEL_2G);
    ThisThread::sleep_for(chrono::milliseconds(20));

    return true;
}

uint16_t MPU6500_I2C::getACCRawX()
{
    return m_ACCrawX;
}

uint16_t MPU6500_I2C::getACCRawY()
{
    return m_ACCrawY;
}

uint16_t MPU6500_I2C::getACCRawZ()
{
    return m_ACCrawZ;
}

uint16_t MPU6500_I2C::getGYRRawX()
{
    return m_GYRrawX;
}

uint16_t MPU6500_I2C::getGYRRawY()
{
    return m_GYRrawY;
}

uint16_t MPU6500_I2C::getGYRRawZ()
{
    return m_GYRrawZ;
}

uint16_t MPU6500_I2C::getTempRaw()
{
    return m_Tempraw;
}

float MPU6500_I2C::getAccX()
{
    return (float)((int16_t)m_ACCrawX) * m_ACCfsr;
}

float MPU6500_I2C::getAccY()
{
    return (float)((int16_t)m_ACCrawY) * m_ACCfsr;
}

float MPU6500_I2C::getAccZ()
{
    return (float)((int16_t)m_ACCrawZ) * m_ACCfsr;
}

float MPU6500_I2C::getGyroX()
{
    return (float)((int16_t)m_GYRrawX) * m_GYRfsr;
}

float MPU6500_I2C::getGyroY()
{
    return (float)((int16_t)m_GYRrawY) * m_GYRfsr;
}

float MPU6500_I2C::getGyroZ()
{
    return (float)((int16_t)m_GYRrawZ) * m_GYRfsr;
}

float MPU6500_I2C::getTemp()
{
    return (float)((int16_t)m_Tempraw) / 340.0f + 36.53f; // Temperature in Celsius, on page 31
}

void MPU6500_I2C::readACCX()
{
    readRegister(ACCEL_XOUT_H, 2);
    m_ACCrawX = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readACCY()
{
    readRegister(ACCEL_YOUT_H, 2);
    m_ACCrawY = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readACCZ()
{
    readRegister(ACCEL_ZOUT_H, 2);
    m_ACCrawZ = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readAccAll()
{
    readRegister(ACCEL_XOUT_H, 6);
    m_ACCrawX = (((uint16_t)m_buffer[0] << 8) | (uint16_t)m_buffer[1]);
    m_ACCrawY = (((uint16_t)m_buffer[2] << 8) | (uint16_t)m_buffer[3]);
    m_ACCrawZ = (((uint16_t)m_buffer[4] << 8) | (uint16_t)m_buffer[5]);
    return;
}

void MPU6500_I2C::readGYRX()
{
    readRegister(GYRO_XOUT_H, 2);
    m_GYRrawX = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readGYRY()
{
    readRegister(GYRO_YOUT_H, 2);
    m_GYRrawY = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readGYRZ()
{
    readRegister(GYRO_ZOUT_H, 2);
    m_GYRrawZ = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readGyroAll()
{
    readRegister(GYRO_XOUT_H, 6);
    m_GYRrawX = (((uint16_t)m_buffer[0] << 8) | (uint16_t)m_buffer[1]);
    m_GYRrawY = (((uint16_t)m_buffer[2] << 8) | (uint16_t)m_buffer[3]);
    m_GYRrawZ = (((uint16_t)m_buffer[4] << 8) | (uint16_t)m_buffer[5]);
    return;
}

void MPU6500_I2C::readTemp()
{
    readRegister(TEMP_OUT_H, 2);
    m_Tempraw = (uint16_t)(m_buffer[0] << 8 | m_buffer[1]);
    return;
}

void MPU6500_I2C::readAll()
{
    readRegister(ACCEL_XOUT_H, 14);
    m_ACCrawX = (((uint16_t)m_buffer[0] << 8) | (uint16_t)m_buffer[1]);
    m_ACCrawY = (((uint16_t)m_buffer[2] << 8) | (uint16_t)m_buffer[3]);
    m_ACCrawZ = (((uint16_t)m_buffer[4] << 8) | (uint16_t)m_buffer[5]);
    m_Tempraw = (((uint16_t)m_buffer[6] << 8) | (uint16_t)m_buffer[7]);
    m_GYRrawX = (((uint16_t)m_buffer[8] << 8) | (uint16_t)m_buffer[9]);
    m_GYRrawY = (((uint16_t)m_buffer[10] << 8) | (uint16_t)m_buffer[11]);
    m_GYRrawZ = (((uint16_t)m_buffer[12] << 8) | (uint16_t)m_buffer[13]);
    return;
}

void MPU6500_I2C::set_gyro_scale(uint8_t config)
{
    readRegister(GYRO_CONFIG, 1);
    char msg = (m_buffer[0] & ~FS_SEL_MASK) | config;
    writeRegister(GYRO_CONFIG, msg);

    // Confirm the change
    readRegister(GYRO_CONFIG, 1);
    if (m_buffer[0] == FS_SEL_250DPS) {
        m_GYRfsr = (M_PIf / 180.0f) * (250.0f / 32768.0f);
    } else if (m_buffer[0] == FS_SEL_500DPS) {
        m_GYRfsr = (M_PIf / 180.0f) * (500.0f / 32768.0f);
    } else if (m_buffer[0] == FS_SEL_1000DPS) {
        m_GYRfsr = (M_PIf / 180.0f) * (1000.0f / 32768.0f);
    } else if (m_buffer[0] == FS_SEL_2000DPS) {
        m_GYRfsr = (M_PIf / 180.0f) * (2000.0f / 32768.0f);
    }
}

void MPU6500_I2C::set_acc_scale(uint8_t config)
{
    readRegister(ACCEL_CONFIG, 1);
    char msg = (m_buffer[0] & ~AFS_SEL_MASK) | config;
    writeRegister(ACCEL_CONFIG, msg);

    // Confirm the change
    readRegister(ACCEL_CONFIG, 1);
    if (m_buffer[0] == AFS_SEL_2G) {
        m_ACCfsr = (9.81f * 2.0f) / 32768.0f;
    } else if (m_buffer[0] == AFS_SEL_4G) {
        m_ACCfsr = (9.81f * 4.0f) / 32768.0f;
    } else if (m_buffer[0] == AFS_SEL_8G) {
        m_ACCfsr = (9.81f * 8.0f) / 32768.0f;
    } else if (m_buffer[0] == AFS_SEL_16G) {
        m_ACCfsr = (9.81f * 16.0f) / 32768.0f;
    }
}

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU6500_I2C::testConnection()
{
    printf("MPU6500 connected...\n");
    uint8_t deviceId = whoami();
    printf("Device ID: %d\n", deviceId);

    return deviceId == 0x70;
}

unsigned int MPU6500_I2C::whoami()
{
    unsigned int response;
    m_cmd[0] = WHO_AM_I;
    m_i2c.write(MPU6500_I2C_ADDRESS, m_cmd, 1);
    m_i2c.read(MPU6500_I2C_ADDRESS, m_buffer, 1);

    response = m_buffer[0];
    return response;
}

void MPU6500_I2C::readRegister(uint8_t addr, uint8_t nBytes) {
    m_cmd[0] = addr;
    m_i2c.write(MPU6500_I2C_ADDRESS, m_cmd, 1);
    m_i2c.read(MPU6500_I2C_ADDRESS, m_buffer, nBytes);

    return;
}

void MPU6500_I2C::writeRegister(uint8_t addr, uint8_t msg) {
    m_cmd[0] = addr;
    m_cmd[1] = msg;
    m_i2c.write(MPU6500_I2C_ADDRESS, m_cmd, 2);

    return;
}

void MPU6500_I2C::writeRegister(uint8_t addr, uint8_t* msg, char nBytes) {
    m_cmd[0] = addr;
    for (int i(1); i<nBytes+1; i++) m_cmd[i] = msg[i-1];
    m_i2c.write(MPU6500_I2C_ADDRESS, m_cmd, nBytes+1);

    return;
}