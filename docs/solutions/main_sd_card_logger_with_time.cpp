#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IRSensor.h"
#include "SDLogger.h"

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

    // ir distance sensor with average filter and implicit calibration
    float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
    float ir_distance_cm = 0.0f;
    float ir_distance_avg = 0.0f;
    IRSensor ir_sensor(PC_2);                      // before the calibration the read function will return the averaged mV value
    ir_sensor.setCalibration(2.574e+04f, -29.37f); // after the calibration the read function will return the calibrated value

    // sd card logger
    SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);

    // additional timer to measure time elapsed since last call
    Timer logging_timer;
    microseconds time_previous_us{0};

    // start timer
    main_task_timer.start();
    logging_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        // measure delta time
        const microseconds time_us = logging_timer.elapsed_time();
        const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;

            // read analog input
            ir_distance_mV = ir_sensor.readmV(); // sensor value in millivolts
            ir_distance_cm = ir_sensor.readcm(); // sensor value in centimeters (if calibrated)
            ir_distance_avg = ir_sensor.read();  // after the calibration you actually only need this function call
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
                ir_distance_mV = 0.0f;
                ir_distance_cm = 0.0f;
                ir_distance_avg = 0.0f;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        printf("IR distance mV: %f IR distance cm: %f IR distance cm averaged: %f \n", ir_distance_mV, ir_distance_cm, ir_distance_avg);

        // write data to the internal buffer of the sd card logger and send it to the sd card
        sd_logger.write( dtime_us );        //  0 delta time in us
        sd_logger.write( ir_distance_mV );  //  1 ir distance in mV
        sd_logger.write( ir_distance_cm );  //  2 ir distance in cm
        sd_logger.write( ir_distance_avg ); //  3 ir distance averaged in cm
        sd_logger.send();

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
