#include "LineFollower.h"

// Constructor
LineFollower::LineFollower(PinName sda_pin,
                           PinName scl_pin,
                           float bar_dist,
                           float d_wheel,
                           float b_wheel,
                           float max_motor_vel_rps) : m_SensorBar(sda_pin, scl_pin, bar_dist, false),
                                                      m_Thread(osPriorityAboveNormal2)
{
    // set default gains of the controllers
    setRotationalVelocityControllerGains();

    // transforms wheel to robot velocities
    float r_wheel = d_wheel / 2.0f;
    m_Cwheel2robot << r_wheel / 2.0f,       r_wheel / 2.0f,
                      r_wheel / b_wheel, -r_wheel / b_wheel;

    m_robot_coord.setZero();

    if (r_wheel != 0.0f)
        m_rotation_to_wheel_vel = b_wheel / (2.0f * r_wheel);
    m_motor_vel_max_rps = max_motor_vel_rps;
    m_wheel_vel_max_rps = m_motor_vel_max_rps;

    // start thread
    m_Thread.start(callback(this, &LineFollower::followLine));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &LineFollower::sendThreadFlag), std::chrono::microseconds{m_SensorBar.PERIOD_MUS});
}

// Deconstructor
LineFollower::~LineFollower()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

void LineFollower::setRotationalVelocityControllerGains(float Kp, float Kp_nl)
{
    m_Kp = Kp;
    m_Kp_nl = Kp_nl;
}

void LineFollower::setMaxWheelVelocity(float wheel_vel_max)
{
    if (m_motor_vel_max_rps < wheel_vel_max) {
        m_wheel_vel_max_rps = m_motor_vel_max_rps;
    } else if (wheel_vel_max < 0.0f) {
        m_wheel_vel_max_rps = 0.0f;
    } else {
        m_wheel_vel_max_rps = wheel_vel_max;
    }
}

float LineFollower::getAngleRadians() const
{
    return m_angle;
}

float LineFollower::getAngleDegrees() const
{
    return m_angle * 180.0f / M_PIf;
}

float LineFollower::getRotationalVelocity() const
{
    return m_robot_coord(1);
}

float LineFollower::getTranslationalVelocity() const
{
    return m_robot_coord(0);
}

float LineFollower::getRightWheelVelocity() const
{
    return m_wheel_right_velocity_rps;
}

float LineFollower::getLeftWheelVelocity() const
{
    return m_wheel_left_velocity_rps;
}

bool LineFollower::isLedActive() const
{
    return is_any_led_active;
}

// Thread task
void LineFollower::followLine()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // update sensor bar readings
        m_SensorBar.update();

        // only update sensor bar angle if an led is triggered
        is_any_led_active = m_SensorBar.isAnyLedActive();
        if (is_any_led_active) {
            m_angle = m_SensorBar.getAvgAngleRad();
        }

        // control algorithm for robot velocities
        m_robot_coord(1) = ang_cntrl_fcn(m_Kp, m_Kp_nl, m_angle);
        m_robot_coord(0) = vel_cntrl_fcn(m_wheel_vel_max_rps * 2 * M_PIf,
                                         m_rotation_to_wheel_vel,
                                         m_robot_coord(1),
                                         m_Cwheel2robot);

        // map robot velocities to wheel velocities in rad/sec
        Eigen::Vector2f wheel_speed = m_Cwheel2robot.inverse() * m_robot_coord;

        // setpoints for the dc motors in rps
        m_wheel_right_velocity_rps = wheel_speed(0) / (2.0f * M_PIf);
        m_wheel_left_velocity_rps = wheel_speed(1) / (2.0f * M_PIf);
    }
}

float LineFollower::ang_cntrl_fcn(float Kp, float Kp_nl, float angle)
{
    return Kp * angle + Kp_nl * angle * fabsf(angle);
}

float LineFollower::vel_cntrl_fcn(float wheel_vel_max,
                                  float rotation_to_wheel_vel,
                                  float robot_ang_vel,
                                  Eigen::Matrix2f Cwheel2robot)
{
    Eigen::Matrix<float, 2, 1> wheel_speed;
    if (robot_ang_vel > 0.0f) {
        wheel_speed(0) = wheel_vel_max;
        wheel_speed(1) = wheel_vel_max - 2.0f * rotation_to_wheel_vel * robot_ang_vel;
    } else {
        wheel_speed(0) = wheel_vel_max + 2.0f * rotation_to_wheel_vel * robot_ang_vel;
        wheel_speed(1) = wheel_vel_max;
    }
    Eigen::Matrix<float, 2, 1> robot_coord = Cwheel2robot * wheel_speed;

    return robot_coord(0);
}

void LineFollower::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
