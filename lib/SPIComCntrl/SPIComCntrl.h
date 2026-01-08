#ifndef SPI_COM_CNTRL_H_
#define SPI_COM_CNTRL_H_

#include "mbed.h"

#include "PESBoardPinMap.h"

#include "DCMotor.h"
#include "IMU.h"
#include "SPISlaveDMA.h"
#include "ThreadFlag.h"

#include <Eigen/Dense>

class SPIComCntrl
{
public:
    explicit SPIComCntrl();
    virtual ~SPIComCntrl();

private:
    static constexpr int64_t PERIOD_MUS = 1000;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    SpiData m_spiData;
    SpiSlaveDMA m_SpiSlaveDMA;

    ImuData m_ImuData;
    IMU m_Imu;

    DigitalOut m_enable_motors;
    DCMotor m_Motor_M1;
    DCMotor m_Motor_M2;

    Eigen::Matrix2f Cwheel2robot;
    Eigen::Vector2f robot_speed_setpoint;
    Eigen::Vector2f wheel_speed_setpoint;
    Eigen::Vector2f robot_speed;
    Eigen::Vector2f wheel_speed;

    float m_reply_data[SPI_NUM_FLOATS];

    void threadTask();
    void sendThreadFlag();
};
#endif /* SPI_COM_CNTRL_H_ */
