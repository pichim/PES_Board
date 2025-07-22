#include "climbing.h"
#include <cstdio>
#include <cmath>


#define M_PIf 3.14159265358979323846f

void initClimbing(DigitalOut& enable_motors,
                  Servo& servo_roll,
                  Servo& servo_pitch,
                  Servo& servo_D0,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  DCMotor& motor_M3)
{
    // Enable motors and configure acceleration limits
    enable_motors = 1;
    motor_M1.setMaxAcceleration(0.3f);
    motor_M2.setMaxAcceleration(0.3f);
    motor_M3.setMaxAcceleration(0.5f);

    // Enable servos if not already enabled
    if (!servo_roll.isEnabled()) servo_roll.enable();
    if (!servo_pitch.isEnabled()) servo_pitch.enable();
    if (!servo_D0.isEnabled()) servo_D0.enable(0.0f);

    printf("Initialization complete\n");
}

void updateIMUEstimate(IMU& imu,
                      ImuData& imu_data,
                      Eigen::Vector2f& rp,
                      float& roll_estimate,
                      float& pitch_estimate,
                      float Ts,
                      float kp)
{
    // Read latest IMU data
    imu_data = imu.getImuData();

    // Compute roll and pitch from accelerometer data
    float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2));
    float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2));

    // Update estimates using 1-D Mahony filter formula
    roll_estimate += Ts * (imu_data.gyro(0) + kp * (roll_acc - roll_estimate));
    pitch_estimate += Ts * (imu_data.gyro(1) + kp * (pitch_acc - pitch_estimate));

    // Update output vector
    rp(0) = roll_estimate;
    rp(1) = pitch_estimate;
}

void runInitial(RobotState& robot_state,
                DigitalOut& enable_motors,
                Servo& servo_roll,
                Servo& servo_pitch,
                Servo& servo_D0)
{
    // Enable motors and servos
    enable_motors = 1;
    if (!servo_roll.isEnabled()) servo_roll.enable();
    if (!servo_pitch.isEnabled()) servo_pitch.enable();
    if (!servo_D0.isEnabled()) servo_D0.enable(0.0f);

    // Transition to EXECUTION state
    robot_state = RobotState::EXECUTION;
    printf("Switched to EXECUTION state\n");
}

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
                  Eigen::Vector2f& rp)
{
    // Ensure servos are enabled
    if (!servo_roll.isEnabled()) servo_roll.enable();
    if (!servo_pitch.isEnabled()) servo_pitch.enable();

    // Read line sensor angle, if active
    float angle = 0.0f;
    if (sensor_bar.isAnyLedActive())
        angle = sensor_bar.getAvgAngleRad();

    // Calculate steering angular velocity (omega)
    float omega = -(Kp * std::pow(angle, angle_exp));

    // Compute forward velocity reduced by steering angle
    float v = v_max / (1.0f + gamma * std::fabs(angle));
    v = std::max(v, v_min);

    // Create robot velocity vector (v, omega)
    robot_coord = {v, omega};

    // (Assuming transformation applied externally to convert to wheel speeds)
    wheel_speed = robot_coord;

    // Send commands to motors (convert rad/s to revolutions/s)
    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

    // Map IMU roll and pitch to servo pulse widths for balancing
    roll_servo_width = -normalised_angle_gain * rp(0) + normalised_angle_offset;
    pitch_servo_width = normalised_angle_gain * rp(1) + normalised_angle_offset;

    // Only update servos if within angle limits
    if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
        servo_roll.setPulseWidth(roll_servo_width);

    if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
        servo_pitch.setPulseWidth(pitch_servo_width);

    // Transition to climbing if mechanical button is pressed
    if (mechanical_button.read())
    {
        robot_state = RobotState::CLIMB;
        printf("Switching to CLIMB state\n");
    }
}

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
              int& backmotion_count)
{
    // Disable pitch servo during climbing
    if (servo_pitch.isEnabled())
        servo_pitch.disable();

    // Step through climbing sequence
    switch (climb_step)
    {
        case 0:
            if (emergency_stop == 0)
            {
                motor_M1.setVelocity(0);
                motor_M2.setVelocity(0);
                emergency_stop = 1;
                printf("Emergency stop activated\n");
            }
            climb_step++;
            break;

        case 1:
            motor_M1.setVelocity(-0.09f);
            motor_M2.setVelocity(-0.09f);
            climb_step++;
            break;

        case 2:
            if (backmotion_count > 75)
            {
                motor_M1.setVelocity(0);
                motor_M2.setVelocity(0);
                climb_step++;
            }
            backmotion_count++;
            break;

        case 3:
            servo_D0.setPulseWidth(1.0f);
            climb_step++;
            break;

        case 4:
            if (lifting_counter > 200)
                climb_step++;
            lifting_counter++;
            break;

        case 5:
            motor_M1.setVelocity(0.25f);
            motor_M2.setVelocity(0.25f);
            motor_M3.setVelocity(-0.412f);
            climb_step++;
            break;

        case 6:
            if (forwarding_count > 120 && forwarding_count < 400)
            {
                motor_M1.setVelocity(0.09f);
                motor_M2.setVelocity(0.09f);
                motor_M3.setVelocity(-0.15f);
            }
            if (forwarding_count > 400)
            {
                motor_M1.setVelocity(0.0001f);
                motor_M2.setVelocity(0.0001f);
                motor_M3.setVelocity(-0.0001f);
                climb_step++;
            }
            forwarding_count++;
            break;

        case 7:
            servo_D0.setPulseWidth(0.0f);
            climb_step++;
            break;

        case 8:
            if (dropping_counter > 200)
                climb_step++;
            dropping_counter++;
            break;

        default:
            robot_state = RobotState::EXECUTION;
            printf("Climb finished, returning to EXECUTION state\n");
            break;
    }
}
