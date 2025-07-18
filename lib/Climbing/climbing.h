#ifndef CLIMBING_H
#define CLIMBING_H

#include <vector>
#include "mbed.h"
#include <Eigen/Dense>
#include "Servo.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include "IMU.h"

/**
 * @enum RobotState
 * @brief Enumerates the possible states of the robot during climbing operation.
 */
enum class RobotState {
    INITIAL,    ///< Initialization state, setup hardware
    EXECUTION,  ///< Main operation: line following and balancing
    CLIMB       ///< Climbing state
};

/**
 * @brief Initializes all robot hardware components.
 * 
 * Enables motors and servos, and configures motor accelerations.
 * 
 * @param enable_motors DigitalOut reference to motor power enable pin
 * @param servo_roll Servo controlling roll axis
 * @param servo_pitch Servo controlling pitch axis
 * @param servo_D0 Servo controlling lifting mechanism
 * @param motor_M1 First drive motor
 * @param motor_M2 Second drive motor
 * @param motor_M3 Third motor used during climbing
 */
void initClimbing(DigitalOut& enable_motors,
                  Servo& servo_roll,
                  Servo& servo_pitch,
                  Servo& servo_D0,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  DCMotor& motor_M3);

/**
 * @brief Reads IMU data and applies 1-D Mahony filter to estimate roll and pitch.
 * 
 * @param imu Reference to IMU object for sensor reading
 * @param imu_data Reference to ImuData struct for storing latest IMU values
 * @param rp Reference to Eigen vector containing roll and pitch estimates (output)
 * @param roll_estimate Reference to scalar roll angle estimate (updated in function)
 * @param pitch_estimate Reference to scalar pitch angle estimate (updated in function)
 * @param Ts Sample time in seconds (e.g. main loop period)
 * @param kp Proportional gain for Mahony filter correction
 */
void updateIMUEstimate(IMU& imu,
                      ImuData& imu_data,
                      Eigen::Vector2f& rp,
                      float& roll_estimate,
                      float& pitch_estimate,
                      float Ts,
                      float kp);

/**
 * @brief Performs initialization routine in INITIAL state.
 * 
 * Enables motors and servos. Transitions robot state to EXECUTION.
 * 
 * @param robot_state Reference to robot state variable (modified)
 * @param enable_motors DigitalOut controlling motor power
 * @param servo_roll Servo controlling roll axis
 * @param servo_pitch Servo controlling pitch axis
 * @param servo_D0 Servo controlling lifting mechanism
 */
void runInitial(RobotState& robot_state,
                DigitalOut& enable_motors,
                Servo& servo_roll,
                Servo& servo_pitch,
                Servo& servo_D0);

/**
 * @brief Executes main operation in EXECUTION state.
 * 
 * Performs line following using sensor bar, controls motors for steering and speed,
 * and maps IMU roll and pitch estimates to servo commands for balancing.
 * Checks mechanical button to transition to CLIMB state.
 * 
 * @param robot_state Reference to robot state variable (may be changed)
 * @param sensor_bar Reference to SensorBar for line detection
 * @param motor_M1 Reference to first drive motor
 * @param motor_M2 Reference to second drive motor
 * @param servo_roll Servo controlling roll axis
 * @param servo_pitch Servo controlling pitch axis
 * @param servo_D0 Servo controlling lifting mechanism (may be used/updated)
 * @param mechanical_button DigitalIn for obstacle detection / state change trigger
 * @param robot_coord Reference to 2D vector holding velocity (v) and angular velocity (ω)
 * @param wheel_speed Reference to 2D vector holding wheel velocities (rad/s)
 * @param Kp Proportional gain for steering control
 * @param v_max Maximum forward velocity
 * @param gamma Velocity reduction factor depending on steering angle
 * @param v_min Minimum forward velocity to avoid stall
 * @param angle_exp Exponent applied to steering angle (nonlinear control)
 * @param roll_servo_width Reference to roll servo pulse width (updated)
 * @param pitch_servo_width Reference to pitch servo pulse width (updated)
 * @param angle_range_min Minimum valid servo angle (radians)
 * @param angle_range_max Maximum valid servo angle (radians)
 * @param normalised_angle_gain Gain factor for angle-to-pulse width mapping
 * @param normalised_angle_offset Offset for angle-to-pulse width mapping
 * @param rp Reference to Eigen vector containing current roll and pitch estimates
 */
void runExecution(RobotState& robot_state,
                  SensorBar& sensor_bar,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  Servo& servo_roll,
                  Servo& servo_pitch,
                  Servo& servo_D0,
                  DigitalIn& mechanical_button,
                  Eigen::Vector2f& robot_coord,
                  Eigen::Vector2f& wheel_speed,
                  float Kp,
                  float v_max,
                  float gamma,
                  float v_min,
                  float angle_exp,
                  float& roll_servo_width,
                  float& pitch_servo_width,
                  const float angle_range_min,
                  const float angle_range_max,
                  float normalised_angle_gain,
                  float normalised_angle_offset,
                  Eigen::Vector2f& rp);

/**
 * @brief Executes the climbing sequence in CLIMB state.
 * 
 * Controls motor velocities and servos through climbing steps.
 * Manages counters and step transitions internally.
 * 
 * @param robot_state Reference to robot state variable (may be changed back to EXECUTION)
 * @param motor_M1 Reference to first drive motor
 * @param motor_M2 Reference to second drive motor
 * @param motor_M3 Reference to climbing motor
 * @param servo_pitch Servo controlling pitch axis (may be disabled during climbing)
 * @param servo_D0 Servo controlling lifting mechanism
 * @param climb_step Reference to climb step counter (updated internally)
 * @param emergency_stop Reference to emergency stop flag (set internally)
 * @param lifting_counter Reference to lifting counter (incremented)
 * @param dropping_counter Reference to dropping counter (incremented)
 * @param forwarding_count Reference to forward motion counter (incremented)
 * @param backmotion_count Reference to backward motion counter (incremented)
 */
void runClimb(RobotState& robot_state,
              DCMotor& motor_M1,
              DCMotor& motor_M2,
              DCMotor& motor_M3,
              Servo& servo_pitch,
              Servo& servo_D0,
              int& climb_step,
              int& emergency_stop,
              int& lifting_counter,
              int& dropping_counter,
              int& forwarding_count,
              int& backmotion_count);

#endif // CLIMBING_H
