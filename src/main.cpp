#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"
#include "Servo.h"
#include "MazeSolver.h"
#include <chrono>

using namespace std::chrono_literals;

// Global flags for controlling execution state
volatile bool do_execute_main_task = false;
volatile bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);

// Interrupt function to toggle running state
void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task)
        do_reset_all_once = true; // Request reset on start
}

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);  // Attach interrupt on button press

    // Setup peripherals
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);

    Servo servo_D0(PB_D0);
    servo_D0.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_D0.setMaxAcceleration(1.0f);

    const float voltage_max = 12.0f;
    const float gear_ratio = 100.0f;
    const float kn = 140.0f / 12.0f;
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    SensorBar sensor_bar(PB_9, PB_8, 0.114f);
    DigitalIn mechanical_button(PB_14);
    mechanical_button.mode(PullUp);

    // Variables to track maze state
    std::vector<Turn> global_path;
    std::vector<Junction> junctions;
    int junction_counter = 0;
    int turn_count = 0;
    size_t optimal_run_count = 0;
    int current_state = 0; // 0=INIT, 1=EXPLORING, 2=OPTIMAL_RUN

    const int main_task_period_ms = 20;
    Timer main_task_timer;
    main_task_timer.start();


    while (true)
    {
        main_task_timer.reset();

        if (do_execute_main_task)
        {
            switch (current_state)
            {
                case 0:  // INIT state
                    initMaze(enable_motors, servo_D0, motor_M1, motor_M2);
                    current_state = 1; // Switch to EXPLORING state
                    break;

                case 1: // EXPLORING state
                    runExploring(sensor_bar, motor_M1, motor_M2,
                                 global_path, junctions,
                                 junction_counter, turn_count,
                                 current_state);
                    break;

                case 2: // OPTIMAL_RUN state
                    runOptimalRun(sensor_bar, motor_M1, motor_M2,
                                  global_path, optimal_run_count,
                                  mechanical_button, current_state,
                                  do_execute_main_task);
                    break;

                default:
                    // Unknown state, reset to INIT
                    current_state = 0;
                    break;
            }
        }
        else if (do_reset_all_once)
        {
            do_reset_all_once = false;
            resetAll(enable_motors, motor_M1, motor_M2,
                     turn_count, optimal_run_count,
                     junction_counter, global_path,
                     junctions, current_state,
                     do_execute_main_task);
        }

        user_led = !user_led; // Blink LED to show alive

        // Maintain loop timing
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - elapsed < 0)
            printf("Warning: Main task took longer than expected\n");
        else
            thread_sleep_for(main_task_period_ms - elapsed);
    }
}