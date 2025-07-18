#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"
#include "Servo.h"
#include "IMU.h"
#include "climbing.h"
#include <chrono>

using namespace std::chrono_literals;

// Global flags controlled by user button interrupt
volatile bool do_execute_main_task = false;  // Flag to control main task execution
volatile bool do_reset_all_once = false;     // Flag to trigger one-time reset

// Debounced user button instance on BUTTON1 pin
DebounceIn user_button(BUTTON1);

/**
 * @brief Interrupt callback toggling main task execution flag.
 *        Sets reset flag on activation to reset system state.
 */
void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task)
        do_reset_all_once = true;
}

int main()
{
    // Attach falling edge interrupt for user button press
    user_button.fall(&toggle_do_execute_main_fcn);

    // Digital output pins for enabling motors and status LEDs
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);

    // Servo objects for roll, pitch control and lifting mechanism
    Servo servo_roll(PB_D2);
    Servo servo_pitch(PB_D1);
    Servo servo_D0(PB_D0);

    // IMU instance on I2C pins, with data structure for readings
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    ImuData imu_data;
    Eigen::Vector2f rp(0.0f, 0.0f);  // Vector storing roll and pitch estimates
    float roll_estimate = 0.0f;       // Roll angle estimate
    float pitch_estimate = 0.0f;      // Pitch angle estimate

    // DC motor instances with encoders and parameters
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, 100.0f, 140.0f/12.0f, 12.0f);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, 100.0f, 140.0f/12.0f, 12.0f);
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, 391.0f, 3.0f, 12.0f);

    // Sensor bar instance for line following
    SensorBar sensor_bar(PB_9, PB_8, 0.114f);

    // Digital input for mechanical button, configured with pull-up resistor
    DigitalIn mechanical_button(PB_14);
    mechanical_button.mode(PullUp);

    // Vectors for robot velocity commands and wheel speeds
    Eigen::Vector2f robot_coord(0.0f, 0.0f);
    Eigen::Vector2f wheel_speed(0.0f, 0.0f);

    // Timing variables and timer for periodic main loop execution
    const int main_task_period_ms = 20;
    Timer main_task_timer;
    main_task_timer.start();

    // Servo calibration parameters for lifting servo (servo_D0)
    float servo_D0_ang_min = 0.0335f;
    float servo_D0_ang_max = 0.1165f;
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D0.setMaxAcceleration(0.2f);

    // Servo roll and pitch pulse width parameters and angle limits
    float roll_servo_width = 0.5f;  // Initial pulse width (neutral position)
    float pitch_servo_width = 0.5f;

    float servo_ang_min = 0.035f;
    float servo_ang_max = 0.130f;
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    const float angle_range_min = -M_PIf/2.0f;  // Minimum servo angle limit (radians)
    const float angle_range_max = M_PIf/2.0f;   // Maximum servo angle limit (radians)

    const float normalised_angle_gain = 1.0f / M_PIf;   // Gain to normalize angles to servo pulse widths
    const float normalised_angle_offset = 0.5f;         // Offset for servo pulse width calibration

    // Control parameters for motor velocity and steering
    const float Kp = 4.8f;  // Steering proportional gain
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();  // Max wheel velocity (rad/s)
    const float v_max = 0.15f * wheel_vel_max * (0.090f / 2.0f);  // Max forward velocity (m/s)
    const float gamma = 3.0f;  // Velocity reduction factor proportional to steering angle
    const float v_min = 0.02f; // Minimum forward velocity to prevent stall
    const float angle_exp = 1.0f; // Exponent for nonlinear steering effect

    RobotState robot_state = RobotState::INITIAL;  // Initial robot state on startup

    // Variables used for climbing state machine step counts and flags
    int climb_step = 0;
    int emergency_stop = 0;
    int lifting_counter = 0;
    int dropping_counter = 0;
    int forwarding_count = 0;
    int backmotion_count = 0;

    // Main program loop, runs forever
    while (true)
    {
        main_task_timer.reset();  // Reset timer to track elapsed time per loop iteration

        // Read IMU and update roll/pitch estimates each iteration using Mahony filter
        updateIMUEstimate(imu, imu_data, rp, roll_estimate, pitch_estimate, main_task_period_ms * 1e-3f, 3.5f);

        // If execution flag is set, run the appropriate state machine logic
        if (do_execute_main_task)
        {
            switch (robot_state)
            {
                case RobotState::INITIAL:
                    // Initialization: enable motors and servos, transition state
                    runInitial(robot_state, enable_motors, servo_roll, servo_pitch, servo_D0);
                    break;

                case RobotState::EXECUTION:
                    // Main execution: line follow and balance
                    runExecution(robot_state, sensor_bar, motor_M1, motor_M2,
                                 servo_roll, servo_pitch, servo_D0,
                                 mechanical_button,
                                 robot_coord, wheel_speed,
                                 Kp, v_max, gamma, v_min, angle_exp,
                                 roll_servo_width, pitch_servo_width,
                                 angle_range_min, angle_range_max,
                                 normalised_angle_gain, normalised_angle_offset,
                                 rp);
                    break;

                case RobotState::CLIMB:
                    // Climbing behavior state machine
                    runClimb(robot_state, motor_M1, motor_M2, motor_M3, servo_pitch, servo_D0,
                             climb_step, emergency_stop, lifting_counter,
                             dropping_counter, forwarding_count, backmotion_count);
                    break;
            }
        }
        // If reset flag is set, reset robot and peripherals to default states
        else if (do_reset_all_once)
        {
            do_reset_all_once = false;
            led1 = 0;
            enable_motors = 0;
            // Reset servo positions and robot state machine
            roll_servo_width = 0.5f;
            pitch_servo_width = 0.5f;
            servo_roll.setPulseWidth(roll_servo_width);
            servo_pitch.setPulseWidth(pitch_servo_width);
            servo_D0.setPulseWidth(0.0f);
            robot_state = RobotState::INITIAL;
        }

        // Blink user LED every loop iteration to indicate alive status
        user_led = !user_led;

        // Calculate elapsed time in milliseconds since start of this loop iteration
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();

        // Sleep for the remainder of main task period if ahead of schedule, otherwise warn
        if (main_task_period_ms - elapsed_ms > 0)
            thread_sleep_for(main_task_period_ms - elapsed_ms);
        else
            printf("Warning: Main task took longer than main_task_period_ms\n");
    }
}
