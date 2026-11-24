#ifndef MPU6500_I2C_H_
#define MPU6500_I2C_H_

#include <cstdint>
#include <stdint.h>
#include "mbed.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

#define MPU6500_I2C_ADDRESS     0x68U<<1   // MPU6500 I2C address

#define SELF_TEST_X             0x0DU   /* R/W */
#define SELF_TEST_Y             0x0EU   /* R/W */
#define SELF_TEST_Z             0x0FU   /* R/W */
#define SELF_TEST_A             0x10U   /* R/W */

#define SMPLRT_DIV              0x19U   /* R/W */
#define CONFIG                  0x1AU   /* R/W */
#define GYRO_CONFIG             0x1BU   /* R/W */
#define ACCEL_CONFIG            0x1CU   /* R/W */
#define ACCEL_CONFIG_2          0x1DU   /* R/W */
#define LP_ACCEL_ODR            0x1EU   /* R/W */
#define MOT_THR                 0x1FU   /* R/W */
#define FIFO_EN                 0x23U   /* R/W */
#define I2C_MST_CTRL            0x24U   /* R/W */
#define I2C_SLV0_ADDR           0x25U   /* R/W */
#define I2C_SLV0_REG            0x26U   /* R/W */
#define I2C_SLV0_CTRL           0x27U   /* R/W */
#define I2C_SLV1_ADDR           0x28U   /* R/W */
#define I2C_SLV1_REG            0x29U   /* R/W */
#define I2C_SLV1_CTRL           0x2AU   /* R/W */
#define I2C_SLV2_ADDR           0x2BU   /* R/W */
#define I2C_SLV2_REG            0x2CU   /* R/W */
#define I2C_SLV2_CTRL           0x2DU   /* R/W */
#define I2C_SLV3_ADDR           0x2EU   /* R/W */
#define I2C_SLV3_REG            0x2FU   /* R/W */
#define I2C_SLV3_CTRL           0x30U   /* R/W */
#define I2C_SLV4_ADDR           0x31U   /* R/W */
#define I2C_SLV4_REG            0x32U   /* R/W */
#define I2C_SLV4_DO             0x33U   /* R/W */
#define I2C_SLV4_CTRL           0x34U   /* R/W */
#define I2C_SLV4_DI             0x35U   /* R */
#define I2C_MST_STATUS          0x36U   /* R */
#define INT_PIN_CFG             0x37U   /* R/W */
#define INT_ENABLE              0x38U   /* R/W */
#define INT_STATUS              0x3AU   /* R */

#define ACCEL_XOUT_H            0x3BU   /* R */
#define ACCEL_XOUT_L            0x3CU   /* R */
#define ACCEL_YOUT_H            0x3DU   /* R */
#define ACCEL_YOUT_L            0x3EU   /* R */
#define ACCEL_ZOUT_H            0x3FU   /* R */
#define ACCEL_ZOUT_L            0x40U   /* R */
#define TEMP_OUT_H              0x41U   /* R */
#define TEMP_OUT_L              0x42U   /* R */
#define GYRO_XOUT_H             0x43U   /* R */
#define GYRO_XOUT_L             0x44U   /* R */
#define GYRO_YOUT_H             0x45U   /* R */
#define GYRO_YOUT_L             0x46U   /* R */
#define GYRO_ZOUT_H             0x47U   /* R */
#define GYRO_ZOUT_L             0x48U   /* R */

#define EXT_SENS_DATA_00        0x49U   /* R */
#define EXT_SENS_DATA_01        0x4AU   /* R */
#define EXT_SENS_DATA_02        0x4BU   /* R */
#define EXT_SENS_DATA_03        0x4CU   /* R */
#define EXT_SENS_DATA_04        0x4DU   /* R */
#define EXT_SENS_DATA_05        0x4EU   /* R */
#define EXT_SENS_DATA_06        0x4FU   /* R */
#define EXT_SENS_DATA_07        0x50U   /* R */
#define EXT_SENS_DATA_08        0x51U   /* R */
#define EXT_SENS_DATA_09        0x52U   /* R */
#define EXT_SENS_DATA_10        0x53U   /* R */
#define EXT_SENS_DATA_11        0x54U   /* R */
#define EXT_SENS_DATA_12        0x55U   /* R */
#define EXT_SENS_DATA_13        0x56U   /* R */
#define EXT_SENS_DATA_14        0x57U   /* R */
#define EXT_SENS_DATA_15        0x58U   /* R */
#define EXT_SENS_DATA_16        0x59U   /* R */
#define EXT_SENS_DATA_17        0x5AU   /* R */
#define EXT_SENS_DATA_18        0x5BU   /* R */
#define EXT_SENS_DATA_19        0x5CU   /* R */
#define EXT_SENS_DATA_20        0x5DU   /* R */
#define EXT_SENS_DATA_21        0x5EU   /* R */
#define EXT_SENS_DATA_22        0x5FU   /* R */
#define EXT_SENS_DATA_23        0x60U   /* R */
#define I2C_SLV0_DO             0x63U   /* R/W */
#define I2C_SLV1_DO             0x64U   /* R/W */
#define I2C_SLV2_DO             0x65U   /* R/W */
#define I2C_SLV3_DO             0x66U   /* R/W */
#define I2C_MST_DELAY_CTRL      0x67U   /* R/W */
#define SIGNAL_PATH_RESET       0x68U   /* R/W */
#define MOT_DETECT_CTRL         0x69U   /* R/W */
#define USER_CTRL               0x6AU   /* R/W */
#define PWR_MGMT_1              0x6BU   /* R/W */
#define PWR_MGMT_2              0x6CU   /* R/W */
#define FIFO_COUNTH             0x72U   /* R/W */
#define FIFO_COUNTL             0x73U   /* R/W */
#define FIFO_R_W                0x74U   /* R/W */
#define WHO_AM_I                0x75U   /* R */


// CONFIGURATION BITS
// CONFIG
#define DLPF_CFG_260HZ_256HZ_NA 0x00U<<0
#define DLPF_CFG_184HZ_188HZ_NA 0x01U<<0
#define DLPF_CFG_94HZ_98HZ_NA   0x02U<<0
#define DLPF_CFG_44HZ_42HZ_NA   0x03U<<0
#define DLPF_CFG_21HZ_20HZ_NA   0x04U<<0
#define DLPF_CFG_10HZ_10HZ_NA   0x05U<<0
#define DLPF_CFG_5HZ_5HZ_NA     0x06U<<0
#define DLPF_CFG_MASK           0x07U<<0

// GYRO_CONFIG
#define FS_SEL_250DPS           0x00U<<3
#define FS_SEL_500DPS           0x01U<<3
#define FS_SEL_1000DPS          0x02U<<3
#define FS_SEL_2000DPS          0x03U<<3
#define FS_SEL_MASK             0x03U<<3

// ACCEL_CONFIG
#define AFS_SEL_2G              0x00U<<3
#define AFS_SEL_4G              0x01U<<3
#define AFS_SEL_8G              0x02U<<3
#define AFS_SEL_16G             0x03U<<3
#define AFS_SEL_MASK            0x03U<<3


// #define I2C_MASTER_TX_BUF_DISABLE 0      // I2C master doesn't need buffer
// #define I2C_MASTER_RX_BUF_DISABLE 0      // I2C master doesn't need buffer

class MPU6500_I2C{
    public:
        MPU6500_I2C();
        MPU6500_I2C(I2C &i2c);
        MPU6500_I2C(PinName pin_sda, PinName pin_scl);

        bool init();
        bool configuration();

        virtual ~MPU6500_I2C(){};

        uint16_t getACCRawX(), getACCRawY(), getACCRawZ();
        uint16_t getGYRRawX(), getGYRRawY(), getGYRRawZ();
        uint16_t getTempRaw();

        // interfaces for the user, acc in (m^2/sec), gyro in rad/sec
        float getAccX(), getAccY(), getAccZ();
        float getGyroX(), getGyroY(), getGyroZ();

        float getTemp();

        void readACCX();
        void readACCY();
        void readACCZ();

        // update the acc data
        void readAccAll();

        void readGYRX();
        void readGYRY();
        void readGYRZ();

        // update the gyro data
        void readGyroAll();

        void readTemp();
        void readAll();

        void set_gyro_scale(uint8_t config);
        void set_acc_scale(uint8_t config);

        bool testConnection();
        unsigned int whoami();

    private:

        void writeRegister(uint8_t addr, uint8_t msg);
        void writeRegister(uint8_t addr, uint8_t* msg, char nBytes);
        void readRegister(uint8_t addr, uint8_t nBytes);

        I2C &m_i2c;
        char m_cmd[2] = {0};
        char m_buffer[14] = {0};
        uint16_t m_ACCrawX, m_ACCrawY, m_ACCrawZ = 0;
        uint16_t m_GYRrawX, m_GYRrawY, m_GYRrawZ = 0;
        float m_Tempraw = 0.0f;
        float m_ACCfsr, m_GYRfsr = 0.0f;
};

#endif // MPU6500_I2C_H_