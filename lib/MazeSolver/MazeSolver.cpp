#include "MazeSolver.h"
#include <cstdio>
#include <algorithm>
#include <cmath>


// Define your constants here (pi, sensor patterns)
#define M_PIf 3.14159265358979323846f
#define DEAD_END 0b00000000
#define JUNCTION 0b11111111
#define LEFTLINE 0b0011111
#define RIGHTLINE 0b11111000

// Control constants
constexpr float Kp = 4.8f;
constexpr float v_min = 0.05f;
constexpr float gamma_factor = 3.0f;
constexpr float angle_exp = 1.0f;

// Helper function: move forward with given speed for given duration (ms)
static void moveForward(DCMotor& m1, DCMotor& m2, float speed, int duration_ms)
{
    m1.setVelocity(speed);
    m2.setVelocity(speed);
    thread_sleep_for(duration_ms);
    m1.setVelocity(0.0f);
    m2.setVelocity(0.0f);
}

// Helper function: turn left for given duration (ms)
static void turnLeft(DCMotor& m1, DCMotor& m2, int duration_ms)
{
    m1.setVelocity(0.4f);
    m2.setVelocity(-0.4f);
    thread_sleep_for(duration_ms);
    m1.setVelocity(0.0f);
    m2.setVelocity(0.0f);
}

// Helper function: turn right for given duration (ms)
static void turnRight(DCMotor& m1, DCMotor& m2, int duration_ms)
{
    m1.setVelocity(-0.4f);
    m2.setVelocity(0.4f);
    thread_sleep_for(duration_ms);
    m1.setVelocity(0.0f);
    m2.setVelocity(0.0f);
}

void initMaze(DigitalOut& enable_motors,
              Servo& servo_D0,
              DCMotor& motor_M1,
              DCMotor& motor_M2)
{
    enable_motors = 1; // Enable motor power
    motor_M1.setMaxAcceleration(0.3); // Limit acceleration for smooth start
    motor_M2.setMaxAcceleration(0.3);

    if (!servo_D0.isEnabled())
        servo_D0.enable(0.0f); // Enable servo with zero speed

    printf("Init\n");
}

void runExploring(SensorBar& sensor_bar,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  std::vector<Turn>& global_path,
                  std::vector<Junction>& junctions,
                  int& junction_counter,
                  int& turn_count,
                  int& current_state)
{
    // Read raw sensor bits to detect line pattern
    uint8_t bits = sensor_bar.getRaw();
    float left_average = sensor_bar.getMeanThreeAvgBitsLeft(); //  incase reflectance pulsates
    float right_average = sensor_bar.getMeanThreeAvgBitsRight();// incase reflectance pulsates
    if (bits == LEFTLINE || bits == RIGHTLINE || (left_average > 0.6f) || (right_average> 0.6f) )
     {
        // Move forward a bit to confirm junction or corner
        printf("Forwarding\n");
        moveForward(motor_M1, motor_M2, 0.1f, 1500);
        thread_sleep_for(1000);

        uint8_t forward_check = sensor_bar.getRaw();

        if (forward_check == JUNCTION || sensor_bar.isAnyLedActive()) {
            // Check if goal reached by specific bit pattern
            if ((forward_check & 0b00111100) == 0b00111100) {
                motor_M1.setVelocity(0.0f);
                motor_M2.setVelocity(0.0f);
                printf("Goal found!\n");
                printPath(global_path);
                current_state = 2; // OPTIMAL_RUN
            } else {
                // At a junction, decide turn based on sensor bits
                turn_count = 0;
                Junction j;
                j.id = junction_counter++;
                j.path_index = global_path.size();

                if (bits == LEFTLINE || ((bits & 0b00000001) == 0b00000001) || ((left_average >0.6f )&& (right_average < 0.6f))) {
                    j.first_turn = LEFT;
                    j.tried_turns.push_back(LEFT);
                    j.last_exit = LEFT;
                    junctions.push_back(j);
                    turnLeft(motor_M1, motor_M2, 1200);
                    printf("Left turn at junction\n");
                    global_path.push_back(LEFT);
                }
                else if (bits == RIGHTLINE || ((bits & 0b10000000) == 0b10000000) || ((left_average <0.6f )&& (right_average > 0.6f))) {
                    j.first_turn = RIGHT;
                    j.tried_turns.push_back(RIGHT);
                    j.last_exit = RIGHT;
                    junctions.push_back(j);
                    turnRight(motor_M1, motor_M2, 1200);
                    printf("Right turn at junction\n");
                    global_path.push_back(RIGHT);
                }
            }
        } else {
            // No junction detected, it's a corner: turn accordingly
            if (bits == LEFTLINE || ((left_average >0.6f )&& (right_average < 0.6f))) {
                if (!junctions.empty()) turn_count++;
                turnLeft(motor_M1, motor_M2, 1200);
            }
            else if (bits == RIGHTLINE || ((left_average <0.6f )&& (right_average > 0.6f))) {
                if (!junctions.empty()) turn_count++;
                turnRight(motor_M1, motor_M2, 1200);
            }
        }
    }
    else if (bits == DEAD_END) {
        // Dead end detected, backtracking logic could go here
        printf("Dead end detected - backtracking not implemented in this simplified version\n");
    }
    else if (bits == JUNCTION) {
        // Default junction handling - turn left and add to junction list
        printf("Junction detected - turning left\n");
        Junction j;
        j.id = junction_counter++;
        j.path_index = global_path.size();
        j.first_turn = LEFT;
        j.tried_turns.push_back(LEFT);
        j.last_exit = LEFT;
        junctions.push_back(j);
        turnLeft(motor_M1, motor_M2, 1200);
        global_path.push_back(LEFT);
    }
    else {
        // Normal line following - just move forward slowly
        printf("Line following\n");
        moveForward(motor_M1, motor_M2, 0.2f, 100);
    }
}

void runOptimalRun(SensorBar& sensor_bar,
                   DCMotor& motor_M1,
                   DCMotor& motor_M2,
                   const std::vector<Turn>& global_path,
                   size_t& optimal_run_count,
                   DigitalIn& mechanical_button,
                   int& current_state,
                   volatile bool& do_execute_main_task)
{
    // Check if mechanical button pressed to start optimal run
    if (mechanical_button.read() == 1) {
        printf("Optimal run started\n");
        while (optimal_run_count < global_path.size()) {
            Turn next_turn = global_path[optimal_run_count];

            // Execute each turn in the optimal path
            switch (next_turn) {
                case LEFT:
                    turnLeft(motor_M1, motor_M2, 1200);
                    break;
                case RIGHT:
                    turnRight(motor_M1, motor_M2, 1200);
                    break;
                case FORWARD:
                    moveForward(motor_M1, motor_M2, 0.25f, 1200);
                    break;
                default:
                    printf("Unknown turn command '%c'\n", (char)next_turn);
                    break;
            }
            optimal_run_count++;
        }
        printf("Optimal run complete\n");
        // Stop execution after optimal run finishes
        do_execute_main_task = false;
        current_state = 0; // Reset state to INIT
    }
}

void resetAll(DigitalOut& enable_motors,
              DCMotor& motor_M1,
              DCMotor& motor_M2,
              int& turn_count,
              size_t& optimal_run_count,
              int& junction_counter,
              std::vector<Turn>& global_path,
              std::vector<Junction>& junctions,
              int& current_state,
              volatile bool& do_execute_main_task)
{
    enable_motors = 0;  // Disable motors
    motor_M1.setVelocity(0.0f);
    motor_M2.setVelocity(0.0f);
    turn_count = 0;
    optimal_run_count = 0;
    junction_counter = 0;
    global_path.clear();
    junctions.clear();
    current_state = 0; // Reset to INIT state
    do_execute_main_task = false;
    printf("Reset all variables\n");
}

void printPath(const std::vector<Turn>& path)
{
    printf("\n===== Optimal path (%d moves) =====\n", (int)path.size());
    for (Turn t : path) {
        putchar(static_cast<char>(t));
    }
    putchar('\n');
    printf("===================================\n");
}
