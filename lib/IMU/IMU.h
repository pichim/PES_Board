/**
 * @file IMU.h
 * @brief This file defines the IMU class.
 * @author M. Peter / pmic / pichim
 */

#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include <Eigen/Dense>

#include "LSM9DS1.h"
#include "LinearCharacteristics3.h"
#include "Mahony.h"
#include "ThreadFlag.h"

#define IMU_DO_PRINTF false
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true  // if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_MAG_CALIBRATION false // if this is false then no mag calibration gets applied, e.g. A_mag = I, b_mag = 0
#define IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE false

namespace Parameters
{
#if IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE
    // % bessel (D = sqrt(3)/2)
    // w0 = 3;
    // kp = w0 / ( sqrt(3)/3 )
    // ki = kp^2 / 3
    static const float kp = 3.0f / (sqrtf(3.0f) / 3.0f);
    static const float ki = kp * kp / 3.0f;
#else
    // % real pole, no integrator, use this if you dont use the mag
    // w0 = 3;
    // kp = w0;
    // ki = 0;
    static const float kp = 3.0f;
    static const float ki = 0.0f;
#endif

    // mag_calibrated = A_mag * ( mag - b_mag )
    static const Eigen::Matrix3f A_mag = (Eigen::Matrix3f() << 1.0000000f, 0.0000000f, 0.0000000f,
                                                               0.0000000f, 1.0000000f, 0.0000000f,
                                                               0.0000000f, 0.0000000f, 1.0000000f).finished();
    static const Eigen::Vector3f b_mag = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
    static const Eigen::Vector3f b_acc = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
}

class ImuData
{
public:
    ImuData() {
        init();
    };
    ~ImuData() = default;

    Eigen::Vector3f gyro, acc, mag;
    Eigen::Quaternionf quat;
    Eigen::Vector3f rpy; // roll, pitch, yaw according to Tait-Bryan angles ZYX
                         // where R = Rz(yaw) * Ry(pitch) * Rx(roll) for ZYX sequence
                         // singularity at pitch = +/-pi/2 radians (+/- 90 deg)
    Eigen::Vector3f pry; // pitch, roll, yaw according to Tait-Bryan angles ZXY
                         // where R = Rz(yaw) * Rx(roll) * Ry(pitch) for ZXY sequence
                         // singularity at roll = +/-pi/2 radians (+/- 90 deg)
    float tilt = 0.0f;

    void init() {
        gyro.setZero();
        acc.setZero();
        mag.setZero();
        quat.setIdentity();
        rpy.setZero();
        pry.setZero();
    };
};

class IMU
{
public:
    explicit IMU(PinName pin_sda, PinName pin_scl);
    virtual ~IMU();

    ImuData getImuData() const;

private:
    static constexpr int64_t PERIOD_MUS = 20000;
    static constexpr float TS = 1.0e-6f * static_cast<float>(PERIOD_MUS);

    ImuData m_ImuData;
    LSM9DS1 m_ImuLSM9DS1;
    LinearCharacteristics3 m_magCalib;
    Mahony m_Mahony;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    void threadTask();
    void sendThreadFlag();
};

#endif /* IMU_H_ */
