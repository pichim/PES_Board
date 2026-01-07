#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include <Eigen/Dense>

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio = 78.125f;
    const float kn = 180.0f / 12.0f;
    // motor M1 and M2, do NOT enable motion planner, disabled per default
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // const float r1_wheel = 0.0357f / 2.0f; // right wheel radius in meters
    // const float r2_wheel = 0.0357f / 2.0f; // left  wheel radius in meters
    // const float b_wheel = 0.1520f;  // wheelbase, distance from wheel to wheel in meters
    const float r1_wheel = 0.01783f; // right wheel radius in meters
    const float r2_wheel = 0.01787f; // left  wheel radius in meters
    const float b_wheel  = 0.15572f; // wheelbase, distance from wheel to wheel in meters
    const float radius_ratio = r1_wheel / r2_wheel;

    // additional dc motor settings (don't use them for the line follower)
    motor_M1.enableMotionPlanner();
    motor_M1.setMaxVelocity(motor_M1.getMaxVelocity() * 0.6f);
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.2f);
    motor_M1.setVelocityCntrlIntegratorLimitsPercent(100.0f);
    motor_M2.enableMotionPlanner();
    motor_M2.setMaxVelocity(motor_M2.getMaxVelocity() * 0.6f * radius_ratio);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.2f * radius_ratio);
    motor_M2.setVelocityCntrlIntegratorLimitsPercent(100.0f);

    // transforms robot to wheel velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    const float L_square = 1.4f;      // forward distance in meters
    const float turn = -M_PIf / 2.0f; // rotation angle in radians
                                      // -90 deg for right turn (CW), +90 deg for left turn (CCW)

    // calculate pure forward and pure turn movement as wheel angles
    Eigen::Vector2f robot_coord_forward = {L_square, 0.0f};
    Eigen::Vector2f robot_coord_turn = {0.0, turn};
    Eigen::Vector2f wheel_angle_forward = Cwheel2robot.inverse() * robot_coord_forward;
    Eigen::Vector2f wheel_angle_turn = Cwheel2robot.inverse() * robot_coord_turn;

    // set up states for state machine
    enum RobotState {
        FORWARD,
        TURN,
        FORWARD_OR_RESET,
        RESET
    } robot_state = RobotState::FORWARD;

    const float angle_threshold = 0.005f;
    int turn_cntr = 0;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;

            // state machine
            switch (robot_state) {
                case RobotState::FORWARD: {
                    motor_M1.setRotationRelative(wheel_angle_forward(0) / (2.0f * M_PIf));
                    motor_M2.setRotationRelative(wheel_angle_forward(1) / (2.0f * M_PIf));
                    robot_state = RobotState::TURN;

                    break;
                }
                case RobotState::TURN: {
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                        (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        motor_M1.setRotationRelative(wheel_angle_turn(0) / (2.0f * M_PIf));
                        motor_M2.setRotationRelative(wheel_angle_turn(1) / (2.0f * M_PIf));
                        robot_state = RobotState::FORWARD_OR_RESET;
                    }

                    break;
                }
                case RobotState::FORWARD_OR_RESET: {
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                        (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        turn_cntr++;
                        if (turn_cntr == 4) {
                            robot_state = RobotState::RESET;
                        } else {
                            robot_state = RobotState::FORWARD;
                        }
                    }

                    break;
                }
                case RobotState::RESET: {
                    toggle_do_execute_main_fcn();
                    turn_cntr = 0;
                    robot_state = RobotState::FORWARD;

                    break;
                }
                default: {

                    break; // do nothing
                }
            }

            // debugging
            // printf("RS: %d, M1 SP: %.3f, M2 SP: %.3f, M1: %.3f, M2: %.3f, TC: %d\n", robot_state
            //                                                                        , motor_M1.getRotationTarget()
            //                                                                        , motor_M2.getRotationTarget()
            //                                                                        , motor_M1.getRotation()
            //                                                                        , motor_M2.getRotation()
            //                                                                        , turn_cntr);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                // enable_motors = 0; // do not disable motors for this specific example
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
