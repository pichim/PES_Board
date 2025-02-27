#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "LineFollower.h"

#define USE_MOTOR_GEAR_RATIO_78 false

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    #if USE_MOTOR_GEAR_RATIO_78                                    
    const float gear_ratio = 78.125f; 
    const float kn = 180.0f / 12.0f;
    const float d_wheel = 0.035f;  // wheel diameter in meters
    const float b_wheel = 0.1518f; // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar / array in meters
    const float Kp = 2.0f;
    const float Kp_nl = 17.0f;
    #else // GEAR_RATIO 100
    const float gear_ratio = 78.125f; 
    const float kn = 180.0f / 12.0f;
    const float d_wheel = 0.0372f;  // wheel diameter in meters
    const float b_wheel = 0.156f; // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.114f; // distance from wheel axis to leds on sensor bar / array in meters
    const float Kp = 1.6f * 2.0f;
    const float Kp_nl = 1.6f * 17.0f;
    #endif

    // motor M2 and M3, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio, kn, voltage_max);

    // line follower
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M3.getMaxPhysicalVelocity());
    lineFollower.setRotationalVelocityGain(Kp, Kp_nl);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            enable_motors = 1;
            motor_M2.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M2
            motor_M3.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M3
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

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
