/**
 * notes:
 * - roll, pitch and therefor tilt can be estimated without magnetometer
 * - yaw can only be estimated with magnetometer
 * - magnetic north is not the same as true north, taking magnetic declination into account and implement it would be necessary for true north reference
 * - proper calibration of the magnetometer is crucial for yaw estimation
  */

#include "Mahony.h"

Mahony::Mahony()
{
    initialise();
}

Mahony::Mahony(float kp, float ki, float Ts)
{
    initialise();
    setup(kp, ki, Ts);
}

void Mahony::update(Eigen::Vector3f gyro, Eigen::Vector3f acc)
{
    Eigen::Vector3f g_n(                                          2.0f * ( m_quat.x()*m_quat.z() - m_quat.w()*m_quat.y() ),
                                                                  2.0f * ( m_quat.y()*m_quat.z() + m_quat.w()*m_quat.x() ),
                         ( m_quat.w()*m_quat.w() - m_quat.x()*m_quat.x() - m_quat.y()*m_quat.y() + m_quat.z()*m_quat.z() )  );
    // Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );
    Eigen::Vector3f e = calcRotationError(acc, g_n);

    updateOrientation(gyro, e);
}

void Mahony::update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag)
{
    Eigen::Matrix3f R = m_quat.toRotationMatrix();

    Eigen::Vector3f g_n = R.block<1,3>(2,0).transpose();
    // Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );
    Eigen::Vector3f e = calcRotationError(acc, g_n);

    Eigen::Vector3f h = R * mag.normalized();
    h(2) = 0.0f;
    Eigen::Vector3f b(h.norm(), 0.0f, 0.0f);
    // e += R.transpose() * h.cross(b);
    e += R.transpose() * calcRotationError(h, b) * h.norm();

    updateOrientation(gyro, e);
}

Eigen::Quaternionf Mahony::getOrientationAsQuaternion() const
{
    return m_quat;
}

Eigen::Vector3f Mahony::getOrientationAsRPYAngles() const
{
    return m_rpy;
}

Eigen::Vector3f Mahony::getOrientationAsPRYAngles() const
{
    return m_pry;
}

float Mahony::getTiltAngle() const
{
    return m_tilt;
}

void Mahony::setup(float kp, float ki, float Ts)
{
    setGains(kp, ki);
    setSamplingTime(Ts);
}

void Mahony::setGains(float kp, float ki)
{
    m_kp = kp;
    m_ki = ki;
}

void Mahony::setSamplingTime(float Ts)
{
    m_Ts = Ts;
}

void Mahony::initialise()
{
    m_quat.setIdentity();
    m_bias.setZero();
    m_rpy.setZero();
    m_pry.setZero();
}

Eigen::Vector3f Mahony::quat2rpy(Eigen::Quaternionf q)
{
    // roll, pitch, yaw according to Tait-Bryan angles ZYX
    // where R = Rz(yaw) * Ry(pitch) * Rx(roll) for ZYX sequence
    // singularity at pitch = +/-pi/2 radians (+/- 90 deg)

    // Quaternion components
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // Compute intermediate terms
    float sinP = -2.0f * (qx * qz - qw * qy);            // sin(pitch)
    float sinR_cosP = 2.0f * (qw * qx + qy * qz);        // sin(roll ) * cos(pitch)
    float cosR_cosP = 1.0f - 2.0f * (qx * qx + qy * qy); // cos(roll ) * cos(pitch)
    float sinY_cosP = 2.0f * (qw * qz + qx * qy);        // sin(yaw  ) * cos(pitch)
    float cosY_cosP = 1.0f - 2.0f * (qy * qy + qz * qz); // cos(yaw  ) * cos(pitch)

    // Compute pitch (Y-axis rotation)
    if      (sinP >  1.0f) sinP =  1.0f; // clamp to [-1,1] to avoid NaN
    else if (sinP < -1.0f) sinP = -1.0f;
    float pitch = asinf(sinP);

    // Compute roll (X-axis) and yaw (Z-axis)
    float roll = atan2f(sinR_cosP, cosR_cosP);
    float yaw  = atan2f(sinY_cosP, cosY_cosP);

    // // Compute roll (X-axis) and yaw (Z-axis)
    // float roll, yaw;
    // float cosP = cosf(pitch);
    // if (fabsf(cosP) < 1e-6f) {
    //     // Singularity: cos(pitch) ~ 0 (pitch ≈ ±90°).
    //     // Yaw and roll are coupled; assign all rotation to yaw (or roll).
    //     roll = 0.0f;
    //     // yaw + roll combined in this case
    //     float R10 = 2.0f * (qx * qy + qw * qz);        // matrix element R[1][0]
    //     float R00 = 1.0f - 2.0f * (qy * qy + qz * qz); // matrix element R[0][0]
    //     yaw = atan2f(R10, R00);
    // } else {
    //     roll = atan2f(sinR_cosP, cosR_cosP);
    //     yaw  = atan2f(sinY_cosP, cosY_cosP);
    // }

    return Eigen::Vector3f(roll, pitch, yaw);

    // Eigen::Vector3f angles = q.toRotationMatrix().eulerAngles(2, 1, 0); // [yaw , pitch, roll]
    // return Eigen::Vector3f(angles[2], angles[1], angles[0]);            // [roll, pitch, yaw ]
}

Eigen::Vector3f Mahony::quat2pry(Eigen::Quaternionf q)
{
    // pitch, roll, yaw according to Tait-Bryan angles ZXY
    // where R = Rz(yaw) * Rx(roll) * Ry(pitch)
    // singularity at roll = +/-pi/2

    // Quaternion components
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // Compute intermediate terms
    float sinR = 2.0f * (qw * qx + qy * qz);             // sin(roll )
    float sinP_cosR = 2.0f * (qw * qy - qx * qz);        // sin(pitch) * cos(roll)
    float cosP_cosR = 1.0f - 2.0f * (qx * qx + qy * qy); // cos(pitch) * cos(roll)
    float sinY_cosR = 2.0f * (qw * qz - qx * qy);        // sin(yaw  ) * cos(roll)
    float cosY_cosR = 1.0f - 2.0f * (qx * qx + qz * qz); // cos(yaw  ) * cos(roll)

    // Compute roll (X-axis rotation)
    if      (sinR >  1.0f) sinR =  1.0f; // clamp to [-1,1] to avoid NaN
    else if (sinR < -1.0f) sinR = -1.0f;
    float roll = asinf(sinR);

    // Compute pitch (Y-axis) and yaw (Z-axis)
    float pitch = atan2f(sinP_cosR, cosP_cosR);
    float yaw   = atan2f(sinY_cosR, cosY_cosR);

    // // Compute pitch (Y-axis) and yaw (Z-axis)
    // float pitch, yaw;
    // float cosR = cosf(roll);
    // if (fabsf(cosR) < 1e-6f) {
    //     // Singularity: cos(roll) ~ 0 (roll ≈ ±90°). 
    //     // Yaw and pitch are coupled; assign all rotation to yaw (or pitch).
    //     pitch = 0.0f;
    //     // yaw + pitch combined in this case
    //     float R10 = 2.0f * (qx * qy + qw * qz);        // matrix element R[1][0]
    //     float R00 = 1.0f - 2.0f * (qy * qy + qz * qz); // matrix element R[0][0]
    //     yaw   = atan2f(R10, R00);
    // } else {
    //     pitch = atan2f(sinP_cosR, cosP_cosR);
    //     yaw   = atan2f(sinY_cosR, cosY_cosR);
    // }

    return Eigen::Vector3f(pitch, roll, yaw);

    // Eigen::Vector3f angles = q.toRotationMatrix().eulerAngles(2, 0, 1); // [yaw  , roll, pitch]
    // return Eigen::Vector3f(angles[2], angles[1], angles[0]);            // [pitch, roll, yaw  ]
}

Eigen::Vector3f Mahony::calcRotationError(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    // https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    Eigen::Vector3f vn = v1.cross(v2);
    float vn_norm = vn.norm();
    if (vn_norm != 0.0f) {
        vn /= vn_norm;
    }
    float ang = atan2f(v1.cross(v2).dot(vn), v1.dot(v2));
    return ang * vn;
}

void Mahony::updateOrientation(Eigen::Vector3f gyro, Eigen::Vector3f e)
{
    m_bias += m_ki * e * m_Ts;
    Eigen::Matrix<float, 4, 3> Q;
    Q << -m_quat.x(), -m_quat.y(), -m_quat.z(),
          m_quat.w(), -m_quat.z(),  m_quat.y(),
          m_quat.z(),  m_quat.w(), -m_quat.x(),
         -m_quat.y(),  m_quat.x(),  m_quat.w();

    // carefull here, Eigen Quaternions have the internal storage order [x y z w] but you inilialise them with quat(w, x, y, z)
    // so I rather type the following explicitly
    Eigen::Vector4f dquat = m_Ts * 0.5f * Q * ( gyro + m_bias + m_kp * e );
    m_quat.w() += dquat(0);
    m_quat.x() += dquat(1);
    m_quat.y() += dquat(2);
    m_quat.z() += dquat(3);
    m_quat.normalize();

    m_rpy = quat2rpy(m_quat);

    m_pry = quat2pry(m_quat);

    m_tilt = acosf( m_quat.w() * m_quat.w() - m_quat.x() * m_quat.x() - m_quat.y() * m_quat.y() + m_quat.z() * m_quat.z() );
}
