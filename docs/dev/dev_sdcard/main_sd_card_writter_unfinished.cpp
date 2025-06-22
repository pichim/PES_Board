/**
 * NOTE:
 * - Somehow i only read garbage when the PES board is powered with 2 battery packs
 * - 
 * 
 * TODO SD-Card:
 * - Check what writing frequency is best to send data to the SD card
 * - Check what parameters for the SDWriter works best
 * - Adapt the writing functionallity according to SerialStream
 * - 
 * 
 * TODO General:
 * - fix .gitignore
 * - fid .mbed
 */

 #include "mbed.h"

 // pes board pin map
 #include "PESBoardPinMap.h"
 
 // drivers
 #include "DebounceIn.h"
 #include "MedianFilter3.h"
 #include "SDWriter.h"
 #include "SDLogger.h"
 // #include "SerialStream.h"
 // #include "AvgFilter.h"
 #include "IRSensor.h"
 
 bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                    // decides whether to execute the main task or not
 bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                    // shows how you can run a code segment only once
 
 // objects for user button (blue button) handling on nucleo board
 DebounceIn user_button(BUTTON1);     // create DebounceIn to evaluate the user button
                                      // falling and rising edge
 void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                      // button gets pressed, definition below
 
 // // function declarations, definitions at the end
 // float ir_sensor_compensation(float ir_distance_mV);
 
 // main runs as an own thread
 int main()
 {
     // attach button fall function address to user button object
     user_button.fall(&toggle_do_execute_main_fcn);
 
     // while loop gets executed every main_task_period_ms milliseconds, this is a
     // simple approach to repeatedly execute main
     const int main_task_period_ms = 2; // define main task period time in ms e.g. 20 ms, there for
                                         // the main task will run 50 times per second
     Timer main_task_timer;              // create Timer object which we use to run the main task
                                         // every main_task_period_ms
 
     // led on nucleo board
     DigitalOut user_led(LED1);
 
     // additional led
     // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
     // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
     DigitalOut led1(PB_9);
 
     // // ir distance sensor
     // float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
     // float ir_distance_cm = 0.0f;
     // AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in the infrared distance sensor
     //                              // 0...3.3V are mapped to 0...1
     IRSensor ir_sensor(PC_2);
     ir_sensor.setCalibration(2.574e+04f, -29.37f);
 
     // sd card logger
     const uint8_t num_of_floats = 1 + 21; // first sample is time in seconds    
     SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);
 
     // additional timer to measure time
     Timer logging_timer;
     logging_timer.start();
 
     // // median filter for ir_distance_cm_median
     // float ir_distance_cm_median = 0.0f;
     // MedianFilter3 ir_distance_cm_median_filter;
 
     // float ir_distance_cm_avg = 0.0f;
     // AvgFilter ir_distance_cm_avg_filter(31);
 
     // // serial stream either to matlab or to the openlager
     // SerialStream serialStream(15,
     //                           PA_2,
     //                           PA_3,
     //                           2000000);
 
     // start timer
     main_task_timer.start();
 
     // TODO: remove this
     // printf("--- Starting main loop ---\n");
 
     // this loop will run forever
     while (true) {
         main_task_timer.reset();
 
         // measure delta time
         static microseconds time_previous_us{0}; // static variables are only initialized once
         const microseconds time_us = logging_timer.elapsed_time();
         const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
         time_previous_us = time_us;
 
         // // read analog input
         // ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
         // ir_distance_cm = ir_sensor_compensation(ir_distance_mV);
 
         // // median filtered distance
         // static bool is_first_run = true;
         // if (is_first_run) {
         //     ir_distance_cm_avg_filter.reset(ir_distance_cm);
         //     is_first_run = false;
         // } else
         //     ir_distance_cm_avg = ir_distance_cm_avg_filter.apply(ir_distance_cm);
 
         if (do_execute_main_task) {
 
             // visual feedback that the main task is executed, setting this once would actually be enough
             led1 = 1;
 
             // store detla time in microseconds
             sd_logger.write(dtime_us);
 
             // store values in the array, we store the same values num_of_floats/3 times as an example
             for (int i = 1; i < num_of_floats; i += 3) {
                 sd_logger.write(ir_sensor.readmV());
                 sd_logger.write(ir_sensor.readcm());
                 sd_logger.write(ir_sensor.read());
             }
             sd_logger.send();
 
             // // serial stream to matlab
             // serialStream.write(dtime_us);
             // serialStream.write(ir_distance_mV);
             // serialStream.write(ir_distance_cm);
             // serialStream.write(ir_distance_cm_median);
             // serialStream.send();
 
         } else {
             // the following code block gets executed only once
             if (do_reset_all_once) {
                 do_reset_all_once = false;
 
                 // reset variables and objects
                 led1 = 0;
                 // ir_distance_mV = 0.0f;
                 // ir_distance_cm = 0.0f;
             }
         }
 
         // toggling the user led
         user_led = !user_led;
 
         // print to the serial terminal
         // printf("IR distance mV: %f IR distance cm: %f IR distance cm median: %f \n", ir_distance_mV, ir_distance_cm, ir_distance_cm_median);
 
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
 
 // float ir_sensor_compensation(float ir_distance_mV)
 // {
     // // insert values that you got from the MATLAB file
     // static const float a = 2.574e+04f;
     // static const float b = -29.37f;
 
     // // avoid division by zero by adding a small value to the denominator
     // if (ir_distance_mV + b == 0.0f)
     //     ir_distance_mV -= 0.001f;
 
     // return a / (ir_distance_mV + b);
 // }
 