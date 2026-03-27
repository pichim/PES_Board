#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include <Eigen/Dense>

#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"

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

    // --- adding variables and objects and applying functions starts here ---

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    constexpr float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    constexpr float b_wheel = 0.128f;  // wheelbase, distance from wheel to wheel in meters
    constexpr float bar_dist = 0.066f; // distance from wheel axis to leds on sensor bar / array in meters
    constexpr float d_wheel = 0.0596f; // wheel diameter in meters
    constexpr float r_wheel = d_wheel / 2.0f; // wheel radius in meters

    // https://www.pololu.com/product/3490/specs
    constexpr float gear_ratio = 100.00f;
    constexpr float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f, r_wheel / b_wheel, -r_wheel / b_wheel;

    float sensor_direction = 1.0f;

    // Both sensor bars, one in front, one at the back
    SensorBar sensor_bar_front(PB_9, PB_8, bar_dist);
    SensorBar sensor_bar_back(PB_3, PB_10, bar_dist);

    // Pointer to the sensor bar actually being used
    SensorBar * active_sensor_bar = &sensor_bar_front;

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    // const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // start timer
    main_task_timer.start();

    // Counter to assess how long the program has been running
    int counter = 0;

    // this loop will run forever
    while (true) {

        // This condition is only for demonstration purposes.
        // Switching the direction based on time does not make sense
        if (counter % 500 == 0) {
            sensor_direction = -sensor_direction;

            if (active_sensor_bar == &sensor_bar_front) {
                active_sensor_bar = &sensor_bar_back;
            } else {
                active_sensor_bar = &sensor_bar_front;
            }
        }
        counter++;

        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {
            constexpr float Kp{5.0f};

            // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            enable_motors = 1;

            // only update sensor bar angle if an led is triggered
            if (active_sensor_bar->isAnyLedActive()) {
                // The measured angle needs to be multiplied with -1 if the robot is driving backwards
                angle = sensor_direction * active_sensor_bar->getAvgAngleRad();
            }

            // printf("angle: %f\n", angle);

            // control algorithm for robot velocities
            Eigen::Vector2f robot_coord = {0.25f * wheel_vel_max * r_wheel, // half of the max. forward velocity
                                           -Kp * angle};                     // simple proportional angle controller

            // map robot velocities to wheel velocities in rad/sec
            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

            // The motors need to be multiplied with -1 if the robot is driving backwards
            // Motor 2 is multiplied by -1 because of the way it is mounted in the chassis
            motor_M1.setVelocity((wheel_speed(0) / (2.0f * M_PIf)) * sensor_direction);
            motor_M2.setVelocity(((wheel_speed(1) / (2.0f * M_PIf)) * -1) * sensor_direction);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        const long long main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
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