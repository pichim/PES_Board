/**
 * @file LineFollower.h
 * @brief This file defines the LineFollower class.
 * @author M. Peter / pmic / pichim
 */

#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include "mbed.h"

#include <math.h>

#include "SensorBar.h"

#include <Eigen/Dense>

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f // pi
#endif

class LineFollower
{
public:
    /**
     * @brief Construct a new Line Follower object.
     *
     * @param sda_pin I2C data line pin
     * @param scl_pin I2C clock line pin
     * @param bar_dist Distance between sensor bar and wheelbase in meters.
     * @param d_wheel Diameter of the wheels in meters.
     * @param b_wheel Wheelbase (distance between the wheels) in meters.
     * @param max_motor_vel_rps Maximum motor velocity in rotations per second.
     */
    explicit LineFollower(PinName sda_pin,
                          PinName scl_pin,
                          float bar_dist,
                          float d_wheel,
                          float b_wheel,
                          float max_motor_vel_rps);

    /**
     * @brief Destroy the Line Follower object.
     */
    virtual ~LineFollower();

    /**
     * @brief Set the gains for the rotational velocity controller.
     *
     * @param Kp Proportional gain.
     * @param Kp_nl Non-linear proportional gain.
     */
    void setRotationalVelocityControllerGains(float Kp = 2.0f, float Kp_nl = 17.0f);

    /**
     * @brief Set the maximum wheel velocity.
     *
     * @param wheel_vel_max Maximum wheel velocity.
     */
    void setMaxWheelVelocity(float wheel_vel_max);

    /**
     * @brief Get the angle in radians.
     *
     * @return float Angle in radians.
     */
    float getAngleRadians() const;

    /**
     * @brief Get the angle in degrees.
     *
     * @return float Angle in degrees.
     */
    float getAngleDegrees() const;

    /**
     * @brief Get the rotational velocity of robot.
     *
     * @return float Rotational velocity.
     */
    float getRotationalVelocity() const;

    /**
     * @brief Get the translational velocity of robot.
     *
     * @return float Translational velocity.
     */
    float getTranslationalVelocity() const;

    /**
     * @brief Get the right wheel velocity in rotations per second.
     *
     * @return float Right wheel velocity in rotations per second.
     */
    float getRightWheelVelocity() const;

    /**
     * @brief Get the left wheel velocity in rotations per second.
     *
     * @return float Left wheel velocity in rotations per second.
     */
    float getLeftWheelVelocity() const;

    /**
     * @brief Check if the LED is active.
     *
     * @return true If any LED is active.
     * @return false If no LED is active.
     */
    bool isLedActive() const;


private:
    // rotational velocity controller
    float m_Kp;
    float m_Kp_nl;

    // TODO: proper comments
    float m_rotation_to_wheel_vel;
    float m_motor_vel_max_rps;
    float m_wheel_vel_max_rps;

    // wheels velocities
    float m_wheel_left_velocity_rps{0.0f};
    float m_wheel_right_velocity_rps{0.0f};

    // angle line sensor
    float m_angle{0.0};
    bool is_any_led_active{false};

    SensorBar m_SensorBar;

    Eigen::Matrix2f m_Cwheel2robot; // transforms robot to wheel coordinates
    Eigen::Vector2f m_robot_coord;  // contains w and v (robot rot. and trans. velocities)

    // thread objects
    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    // velocity controller functions
    float ang_cntrl_fcn(float Kp, float Kp_nl, float angle);
    float vel_cntrl_fcn(float wheel_vel_max,
                        float rotation_to_wheel_vel,
                        float robot_ang_vel,
                        Eigen::Matrix2f Cwheel2robot);

    // thread functions
    void followLine();
    void sendThreadFlag();
};

#endif /* LINE_FOLLOWER_H_ */
