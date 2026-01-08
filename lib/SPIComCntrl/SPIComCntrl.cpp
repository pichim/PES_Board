#include "SPIComCntrl.h"

SPIComCntrl::SPIComCntrl() : m_Thread(osPriorityNormal)
                           , m_SpiSlaveDMA(PC_3,
                                           PC_2,
                                           PB_10,
                                           PB_12)
                           , m_Imu(PB_IMU_SDA,
                                   PB_IMU_SCL)
                           , m_enable_motors(PB_ENABLE_DCMOTORS)
                           , m_Motor_M1(PB_PWM_M1,
                                        PB_ENC_A_M1,
                                        PB_ENC_B_M1,
                                        31.25f, // 156.25f,
                                        450.0f / 12.0f) // 89.0f / 12.0f)
                           , m_Motor_M2(PB_PWM_M2,
                                        PB_ENC_A_M2,
                                        PB_ENC_B_M2,
                                        156.25f,
                                        89.0f / 12.0f)
{
    // Differential drive kinematics
    const float r_wheel = 0.0822f / 2.0f; // Wheel radius in meters
    const float b_wheel = 0.1435f;        // Wheelbase, distance from wheel to wheel in meters
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / b_wheel, -r_wheel / b_wheel;

    // Robot speed setpoints contains v and w (robot translational and rotational velocity in m/sec and rad/sec)
    robot_speed_setpoint.setZero();
    robot_speed.setZero();

    // Wheel speed setpoints contains w1 and w2 (wheel speed in rad/sec)
    wheel_speed_setpoint.setZero();
    wheel_speed.setZero();

    // Start thread
    m_Thread.start(callback(this, &SPIComCntrl::threadTask));
}

SPIComCntrl::~SPIComCntrl()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void SPIComCntrl::threadTask()
{
    // Start SPI communication
    if (!m_SpiSlaveDMA.start()) {
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        while (true) {
            ThisThread::sleep_for(500ms);
        }
    } else {
        printf("SPI Communication started. Waiting for master...\n");
    }

    // Enable the motors
    m_enable_motors = 1;

    // Start periodic ticker to trigger thread
    m_Ticker.attach(callback(this, &SPIComCntrl::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});

    // Main thread loop
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // Check for new SPI data from master
        if (m_SpiSlaveDMA.hasNewData()) {
            m_spiData = m_SpiSlaveDMA.getSPIData();

            printf("Message: %lu | Delta Time: %lu us | "
                   "Received: [%.2f, %.2f, %.2f, %.2f, %.2f] | "
                   "Header: 0x%02X | Failed: %lu | "
                   "Readout Time: %lu us\n",
                   m_spiData.message_count, m_spiData.last_delta_time_us,
                   m_spiData.data[0], m_spiData.data[1], m_spiData.data[2], m_spiData.data[3], m_spiData.data[4],
                   SPI_HEADER_SLAVE, m_spiData.failed_count,
                   m_spiData.readout_time_us);
        }

        // Read wheel speeds from motors in rad/sec
        wheel_speed = { m_Motor_M1.getVelocity() * (2.0f * M_PIf),
                        m_Motor_M2.getVelocity() * (2.0f * M_PIf) };

        // Map wheel speeds to robot velocities in m/sec and rad/sec
        robot_speed = Cwheel2robot * wheel_speed;

        // Read setpoints from SPI data
        robot_speed_setpoint = { m_spiData.data[0],
                                 m_spiData.data[1] };
        // Map robot velocity setpoints to wheel speed setpoints in rad/sec
        wheel_speed_setpoint = Cwheel2robot.inverse() * robot_speed_setpoint;

        // Set wheel speeds to motors in RPS
        m_Motor_M1.setVelocity(wheel_speed_setpoint(0) / (2.0f * M_PIf));
        m_Motor_M2.setVelocity(wheel_speed_setpoint(1) / (2.0f * M_PIf));

        // Read IMU data
        m_ImuData = m_Imu.getImuData();

        // Prepare next reply
        m_reply_data[ 0]  = robot_speed(0);     // Robot translational velocity
        m_reply_data[ 1]  = robot_speed(1);     // Robot rotational velocity
        m_reply_data[ 2]  = m_ImuData.gyro.x(); // Gyro X in rad/sec
        m_reply_data[ 3]  = m_ImuData.gyro.y(); // Gyro Y in rad/sec
        m_reply_data[ 4]  = m_ImuData.gyro.z(); // Gyro Z in rad/sec
        m_reply_data[ 5]  = m_ImuData.acc.x();  // Acc X in m/sec^2
        m_reply_data[ 6]  = m_ImuData.acc.y();  // Acc Y in m/sec^2
        m_reply_data[ 7]  = m_ImuData.acc.z();  // Acc Z in m/sec^2
        m_reply_data[ 8]  = m_ImuData.mag.x();  // Mag X in whatever
        m_reply_data[ 9]  = m_ImuData.mag.y();  // Mag Y in whatever
        m_reply_data[10]  = m_ImuData.mag.z();  // Mag Z in whatever
        m_SpiSlaveDMA.setReplyData(m_reply_data, 11);
    }
}

void SPIComCntrl::sendThreadFlag()
{
    // Set thread flag to trigger next iteration
    m_Thread.flags_set(m_ThreadFlag);
}
