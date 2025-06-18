/**
 * @file Mahony.h
 * @brief This file defines the Mahony class.
 * @author M. Peter / pmic / pichim
 */

#ifndef MAHONY_H_
#define MAHONY_H_

#include <Eigen/Dense>

class Mahony
{
public:
    explicit Mahony();
    explicit Mahony(float kp, float ki, float Ts);
    virtual ~Mahony() = default;

    void setup(float kp, float ki, float Ts);
    void setGains(float kp, float ki);
    void setSamplingTime(float Ts);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag);
    Eigen::Quaternionf getOrientationAsQuaternion() const;
    Eigen::Vector3f getOrientationAsRPYAngles() const;
    Eigen::Vector3f getOrientationAsPRYAngles() const;
    float getTiltAngle() const;

private:
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_Ts = 1.0f;
    Eigen::Quaternionf m_quat;
    Eigen::Vector3f m_bias;
    Eigen::Vector3f m_rpy;
    Eigen::Vector3f m_pry;
    float m_tilt = 0.0f;

    void initialise();
    void updateOrientation(Eigen::Vector3f gyro, Eigen::Vector3f e);
    Eigen::Vector3f quat2rpy(Eigen::Quaternionf q);
    Eigen::Vector3f quat2pry(Eigen::Quaternionf q);
    Eigen::Vector3f calcRotationError(Eigen::Vector3f v1, Eigen::Vector3f v2);
};

#endif /* MAHONY_H_ */
