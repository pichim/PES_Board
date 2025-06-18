#include "IMU.h"

IMU::IMU(PinName pin_sda, PinName pin_scl) : m_ImuLSM9DS1(pin_sda, pin_scl),
                                             m_Mahony(Parameters::kp, Parameters::ki, TS),
                                             m_Thread(osPriorityHigh)
{
#if (IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE && IMU_DO_USE_STATIC_MAG_CALIBRATION)
    m_magCalib.setCalibrationParameter(Parameters::A_mag, Parameters::b_mag);
#endif
    
    // start thread
    m_Thread.start(callback(this, &IMU::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &IMU::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

IMU::~IMU()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

ImuData IMU::getImuData() const
{
    return m_ImuData;
}

void IMU::threadTask()
{
    static const uint16_t Navg = static_cast<uint16_t>(1.0f / TS);
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset;
    static Eigen::Vector3f acc_offset;
    gyro_offset.setZero();
    acc_offset.setZero();
    static Timer timer;
    timer.start();

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        m_ImuLSM9DS1.updateGyro();
        m_ImuLSM9DS1.updateAcc();
        Eigen::Vector3f gyro(m_ImuLSM9DS1.readGyroX(), m_ImuLSM9DS1.readGyroY(), m_ImuLSM9DS1.readGyroZ());
        Eigen::Vector3f acc(m_ImuLSM9DS1.readAccX(), m_ImuLSM9DS1.readAccY(), m_ImuLSM9DS1.readAccZ());

#if IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE
        m_ImuLSM9DS1.updateMag();
        Eigen::Vector3f mag(m_ImuLSM9DS1.readMagX(), m_ImuLSM9DS1.readMagY(), m_ImuLSM9DS1.readMagZ());
#else
        static Eigen::Vector3f mag = Eigen::Vector3f::Zero();
#endif

        if (!imu_is_calibrated) {
            gyro_offset += gyro;
            acc_offset += acc;
            avg_cntr++;
            if (avg_cntr == Navg) {
                imu_is_calibrated = true;
                gyro_offset /= avg_cntr;
                acc_offset /= avg_cntr;
                // we have to keep gravity in acc z direction
                acc_offset(2) = 0.0f;
#if IMU_DO_USE_STATIC_ACC_CALIBRATION
                acc_offset = Parameters::b_acc;
#else
                printf("Averaged acc offset: %.7ff, %.7ff, %.7f\n", acc_offset(0), acc_offset(1), acc_offset(2));
#endif
            }
        }

        if (imu_is_calibrated) {
            gyro -= gyro_offset;
            acc -= acc_offset;

#if IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE
            mag = m_magCalib.applyCalibration(mag);
            m_Mahony.update(gyro, acc, mag);
#else
            m_Mahony.update(gyro, acc);
#endif

            // update data object
            m_ImuData.gyro = gyro;
            m_ImuData.acc = acc;
            m_ImuData.mag = mag;
            m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
            m_ImuData.rpy = m_Mahony.getOrientationAsRPYAngles();
            m_ImuData.pry = m_Mahony.getOrientationAsPRYAngles();
            m_ImuData.tilt = m_Mahony.getTiltAngle();
        }

#if IMU_DO_PRINTF
        static float time_ms_past = 0.0f;
        float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() * 1.0e-3f;
        const float dtime_ms = time_ms - time_ms_past;
        time_ms_past = time_ms;
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, ", m_ImuData.gyro(0), m_ImuData.gyro(1), m_ImuData.gyro(2),
               m_ImuData.acc(0), m_ImuData.acc(1), m_ImuData.acc(2),
               m_ImuData.mag(0), m_ImuData.mag(1), m_ImuData.mag(2), time_ms);
        printf("%.6f, %.6f, %.6f, %.6f, ", m_ImuData.quat.w(), m_ImuData.quat.x(), m_ImuData.quat.y(), m_ImuData.quat.z());
        printf("%.6f, %.6f, %.6f, ", m_ImuData.rpy(0), m_ImuData.rpy(1), m_ImuData.rpy(2));
        printf("%.6f, %.6f, %.6f, ", m_ImuData.pry(0), m_ImuData.pry(1), m_ImuData.pry(2));
        printf("%.6f\n", m_ImuData.tilt);
#endif
    }
}

void IMU::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
