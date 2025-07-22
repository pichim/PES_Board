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

/**
 * @brief Initializes maze-solving robot hardware components.
 * 
 * Enables motors and prepares the lifting servo and drive motors.
 * 
 * @param enable_motors DigitalOut reference to motor enable pin
 * @param servo_D0 Servo controlling auxiliary mechanism (e.g. lifting)
 * @param motor_M1 Right wheel drive motor
 * @param motor_M2 Left wheel drive motor
 */
void initMaze(DigitalOut& enable_motors,
              Servo& servo_D0,
              DCMotor& motor_M1,
              DCMotor& motor_M2);

/**
 * @brief Executes exploration logic during maze-solving.
 * 
 * Follows the line, detects and logs junctions, makes decisions,
 * and stores turn choices in global path and junction list.
 * 
 * @param sensor_bar Line sensor used for path detection
 * @param motor_M1 Right wheel drive motor
 * @param motor_M2 Left wheel drive motor
 * @param global_path Vector storing sequence of turns taken
 * @param junctions Vector storing junction metadata
 * @param junction_counter Counter tracking number of junctions found
 * @param turn_count Counter tracking number of turns made
 * @param current_state Reference to robot's current state (modified)
 */
void runExploring(SensorBar& sensor_bar,
                  DCMotor& motor_M1,
                  DCMotor& motor_M2,
                  std::vector<Turn>& global_path,
                  std::vector<Junction>& junctions,
                  int& junction_counter,
                  int& turn_count,
                  int& current_state);

/**
 * @brief Executes the optimal run through the maze using the learned path.
 * 
 * Follows the stored global path from exploration to reach the goal directly.
 * 
 * @param sensor_bar Line sensor used for navigation
 * @param motor_M1 Right wheel drive motor
 * @param motor_M2 Left wheel drive motor
 * @param global_path Vector of previously stored turn instructions
 * @param optimal_run_count Counter tracking steps in optimal path
 * @param mechanical_button Button used for triggering or stopping execution
 * @param current_state Reference to robot's current state (may change)
 * @param do_execute_main_task Reference flag to control execution
 */
void runOptimalRun(SensorBar& sensor_bar,
                   DCMotor& motor_M1,
                   DCMotor& motor_M2,
                   const std::vector<Turn>& global_path,
                   size_t& optimal_run_count,
                   DigitalIn& mechanical_button,
                   int& current_state,
                   volatile bool& do_execute_main_task);

/**
 * @brief Resets all robot states, counters, and stops the motors.
 * 
 * Useful before restarting exploration or optimal run phases.
 * 
 * @param enable_motors DigitalOut to disable motor power
 * @param motor_M1 Right wheel drive motor
 * @param motor_M2 Left wheel drive motor
 * @param turn_count Reference to turn counter (reset)
 * @param optimal_run_count Reference to optimal path counter (reset)
 * @param junction_counter Reference to junction count (reset)
 * @param global_path Reference to path vector (cleared)
 * @param junctions Reference to junction vector (cleared)
 * @param current_state Reference to robot's current state (reset)
 * @param do_execute_main_task Reference flag to control execution (reset)
 */
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

/**
 * @brief Utility function to print the global path of turns.
 * 
 * Displays the stored sequence of turns for debugging or review.
 * 
 * @param path Vector of turn directions to print
 */
void printPath(const std::vector<Turn>& path);

#endif // MAZESOLVER_H
