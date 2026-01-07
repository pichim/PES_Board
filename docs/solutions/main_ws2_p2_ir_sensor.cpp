#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IRSensor.h"
#include "Servo.h"
// #include "UltrasonicSensor.h"

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
        EXECUTION,
        SLEEP,
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

    // // ultra sonic sensor
    // UltrasonicSensor us_sensor(PB_D3);
    // ir distance sensor instead of ultra sonic sensor
    IRSensor ir_sensor(PB_A0);
    ir_sensor.setCalibration(11801.3246f, -11.2132f); // set calibration values
    float us_distance_cm = 0.0f;

    // min and max ultra sonic sensor reading
    float us_distance_min = 6.0f;
    float us_distance_max = 40.0f;

    // servo
    Servo servo_D0(PB_D0);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // // futuba S3001
    // float servo_D0_ang_min = 0.0150f;
    // float servo_D0_ang_max = 0.1150f;
    // // reely S0090
    // float servo_D0_ang_min = 0.0325f;
    // float servo_D0_ang_max = 0.1175f;
    // modelcraft RS2 MG/BB
    float servo_D0_ang_min = 0.0325f;
    float servo_D0_ang_max = 0.1250f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);

    // variables to move the servo, this is just an example
    float servo_input = 0.0f;

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

            // // read us sensor distance, only valid measurements will update us_distance_cm
            // const float us_distance_cm_candidate = us_sensor.read();
            // if (us_distance_cm_candidate > 0.0f)
            //     us_distance_cm = us_distance_cm_candidate;
            // ir distance sensor instead of ultra sonic sensor
            us_distance_cm = ir_sensor.read();

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    // enable the servo
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    robot_state = RobotState::EXECUTION;

                    break;
                }
                case RobotState::EXECUTION: {
                    printf("EXECUTION\n");
                    // function to map the distance to the servo movement (us_distance_min, us_distance_max) -> (0.0f, 1.0f)
                    servo_input = (us_distance_cm - us_distance_min) / (us_distance_max - us_distance_min);
                    // values smaller than 0.0f or bigger than 1.0f are constrained to the range (0.0f, 1.0f) in setPulseWidth
                    servo_D0.setPulseWidth(servo_input);

                    // if the measurement is outside the min or max limit go to SLEEP
                    if ((us_distance_cm < us_distance_min) || (us_distance_cm > us_distance_max))
                        robot_state = RobotState::SLEEP;

                    // if the mechanical button is pressed go to EMERGENCY
                    if (mechanical_button.read())
                        robot_state = RobotState::EMERGENCY;

                    break;
                }
                case RobotState::SLEEP: {
                    printf("SLEEP\n");
                    // if the measurement is within the min and max limits go to EXECUTION
                    if ((us_distance_cm > us_distance_min) && (us_distance_cm < us_distance_max))
                        robot_state = RobotState::EXECUTION;

                    // if the mechanical button is pressed go to EMERGENCY
                    if (mechanical_button.read())
                        robot_state = RobotState::EMERGENCY;

                    break;
                }
                case RobotState::EMERGENCY: {
                    printf("EMERGENCY\n");
                    // the transition to the emergency state causes the execution of the commands contained
                    // in the outer else statement scope, and since do_reset_all_once is true the system undergoes a reset
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
                servo_D0.disable();
                us_distance_cm = 0.0f;
                robot_state = RobotState::INITIAL;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        printf("US distance cm: %f \n", us_distance_cm);

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
