#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end
float toggle_dir = 1.0f;
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
    // DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // // motor M1
    // FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1
    // pwm_M1.period_mus(50);    // set PWM period to 1000 us, therefore the PWM frequency is 20 kHz
    // pwm_M1.write(0.0f);        // set PWM duty cycle to 0.0f, therefore the motor is not moving

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    // Motor M1
    const float gear_ratio_M1 = 156.0f; // gear ratio
    const float kn_M1 = 180.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_DIR_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f); // limit max. velocity to half physical possible velocity
    motor_M1.enableMotionPlanner(); // enable the motion planner for smooth movements
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f); // limit max. acceleration to half of the default acceleration

    // // motor M2
    const float gear_ratio_M2 = 31.0f; // gear ratio
    const float kn_M2 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // // it is assumed that only one motor is available, therefore
    // // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M2(PB_PWM_M2, PB_DIR_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

    // motor M3
    const float gear_ratio_M3 = 78.125f; // gear ratio
    const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M3(PB_PWM_M3, PB_DIR_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner();
    // limit max. velocity to half physical possible velocity
    motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);

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

            // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1;

            // pwm_M1.write(0.75f); // apply 6V to the motor
            // motor_M1.setVelocity(toggle_dir*motor_M1.getMaxVelocity());
            motor_M2.setVelocity(toggle_dir*motor_M2.getMaxVelocity());
            // motor_M3.setVelocity(toggle_dir*motor_M3.getMaxVelocity());
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                toggle_dir = -1.0f*toggle_dir; // change direction for next time
            }
        }

        // print to the serial terminal
        printf("Motor M1 velocity: %f\t Target: %f, Motor M2 velocity: %f\t Target: %f\t Motor M3 velocity: %f\t Target: %f \n",
                motor_M1.getVelocity(), motor_M1.getVelocityTarget(),
                motor_M2.getVelocity(), motor_M2.getVelocityTarget(),
                motor_M3.getVelocity(), motor_M3.getVelocityTarget());

        // toggling the user led
        // user_led = !user_led;

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
