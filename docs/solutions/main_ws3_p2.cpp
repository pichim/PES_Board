#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"

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
    // set up states for state machine
    enum RobotState {
        INITIAL,
        SLEEP,
        FORWARD,
        BACKWARD,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

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

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                       // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                       // is a defined potential

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    // motor M3
    const float gear_ratio_M3 = 78.125f; // gear ratio
    const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);

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

            // read us sensor distance, only valid measurements will update us_distance_cm
            const float us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f)
                us_distance_cm = us_distance_cm_candidate;

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::SLEEP: {
                    // wait for the signal from the user, so to run the process
                    // that is triggered by clicking mechanical button
                    // then go the the FORWARD state
                    if (mechanical_button.read())
                        robot_state = RobotState::FORWARD;

                    break;
                }
                case RobotState::FORWARD: {
                    // press is moving forward until it reaches 2.9f rotations,
                    // when reaching the value go to BACKWARD
                    motor_M3.setRotation(2.9f); // setting this once would actually be enough
                    // if the distance from the sensor is less than 4.5f cm,
                    // we transition to the EMERGENCY state
                    if (us_distance_cm < 4.5f)
                        robot_state = RobotState::EMERGENCY;
                    // switching condition is slightly smaller for robustness
                    if (motor_M3.getRotation() > 2.89f)
                        robot_state = RobotState::BACKWARD;

                    break;
                }
                case RobotState::BACKWARD: {
                    // move backwards to the initial position
                    // and go to the SLEEP state if reached
                    motor_M3.setRotation(0.0f);
                    // switching condition is slightly bigger for robustness
                    if (motor_M3.getRotation() < 0.01f)
                        robot_state = RobotState::SLEEP;

                    break;
                }
                case RobotState::EMERGENCY: {
                    // disable the motion planner and
                    // move to the initial position asap
                    // then reset the system
                    motor_M3.disableMotionPlanner();
                    motor_M3.setRotation(0.0f);
                    if (motor_M3.getRotation() < 0.01f)
                        toggle_do_execute_main_fcn();

                    break;
                }
                default: {

                    break; // do nothing
                }
            }
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                us_distance_cm = 0.0f;
                motor_M3.setMotionPlanerPosition(0.0f);
                motor_M3.setMotionPlanerVelocity(0.0f);
                motor_M3.enableMotionPlanner();
                robot_state = RobotState::INITIAL;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        printf("US Sensor in cm: %f, DC Motor Rotations: %f\n", us_distance_cm, motor_M3.getRotation());

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
