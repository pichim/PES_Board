// ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
// picocom /dev/ttyACM0 -b 115200

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
// #include "DCMotor.h"
#include "DebounceIn.h"
#include "MyDCMotor.h"

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

    // https://www.pololu.com/product/3475/specs
    const float counts_per_turn_at_gear_end = 31.25f * 20.0f;
    const float kn_M1 = 450.0f / 12.0f; // motor constant [rpm/V]
    // const float voltage_for_one_rotation_per_second = 60.0f / kn_M1;
    MyDCMotor motor_M1(PB_PWM_M1,
                       PB_ENC_A_M1,
                       PB_ENC_B_M1,
                       counts_per_turn_at_gear_end,
                       PB_UART3_TX,
                       PB_UART3_RX,
                       voltage_max);

    // // https://www.pololu.com/product/3475/specs
    // const float gear_ratio_M1 = 31.25f; // gear ratio
    // const float kn_M1 = 450.0f / 12.0f; // motor constant [rpm/V]
    // DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // enable the motors if they are not enabled yet
            if (!enable_motors) {
                enable_motors = 1;
                motor_M1.setVoltage(6.0f);
            }

            // enable chirp signal excitation
            if (!motor_M1.isChirpEnabled()) {
                // enable the chirp function to test the motor
                motor_M1.chirpEnable(4.0f);
            }

            // static int cntr = 0;
            // if (cntr++ > 50*5)
            //     motor_M1.startGPA();

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                enable_motors = 0;
                motor_M1.setVoltage(0.0f);

                motor_M1.chirpDisable();

                // reset variables and objects
                led1 = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // print to the serial terminal
        printf("%ld, %f, %f\n", motor_M1.getCounts(),
                                motor_M1.getVelocity(),
                                motor_M1.getRotation());

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
