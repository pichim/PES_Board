#ifndef MAZESOLVER_H
#define MAZESOLVER_H

#include <vector>
#include "mbed.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include "Servo.h"


/****Our Motor_M1 is our right wheeled motor, MOTOR_m2 is our left wheeled motor *****/

// Turn directions for maze solving
enum Turn { FORWARD = 'F', LEFT = 'L', RIGHT = 'R', UTURN = 'U' };

// Structure to hold information about junctions encountered
struct Junction {
    int id;                     // Unique id for the junction
    int path_index;             // Index in global path when junction was found
    std::vector<Turn> tried_turns;  // Which turns tried at this junction
    Turn first_turn;            // First turn attempted
    Turn last_exit;             // Last turn used to leave this junction
};

// Initialize robot hardware and state
void initMaze(DigitalOut& enable_motors,
              Servo& servo_D0,
              DCMotor& motor_M1,
              DCMotor& motor_M2);

// Run the exploring state logic (called when do_execute_main_task is true and state is EXPLORING)
void runExploring(SensorBar& sensor_bar,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  std::vector<Turn>& global_path,
                  std::vector<Junction>& junctions,
                  int& junction_counter,
                  int& turn_count,
                  int& current_state);

// Run the optimal run state logic (called when do_execute_main_task is true and state is OPTIMAL_RUN)
void runOptimalRun(SensorBar& sensor_bar,
                   DCMotor& motor_M1,
                   DCMotor& motor_M2,
                   const std::vector<Turn>& global_path,
                   size_t& optimal_run_count,
                   DigitalIn& mechanical_button,
                   int& current_state,
                   volatile bool& do_execute_main_task);

// Reset all variables and stop motors (called when do_reset_all_once is true)
void resetAll(DigitalOut& enable_motors,
              DCMotor& motor_M1,
              DCMotor& motor_M2,
              int& turn_count,
              size_t& optimal_run_count,
              int& junction_counter,
              std::vector<Turn>& global_path,
              std::vector<Junction>& junctions,
              int& current_state,
              volatile bool& do_execute_main_task);

// Utility to print the path
void printPath(const std::vector<Turn>& path);

#endif // MAZESOLVER_H
