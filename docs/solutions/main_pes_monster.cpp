#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "IMU.h"
#include "IRSensor.h"
#include "RealTimeThread.h"
#include "SDLogger.h"
#include "Servo.h"
#include "UltrasonicSensor.h"

// unused, i just want to see if it compiles
#include "AvgFilter.h"
#include "Chirp.h"
#include <Eigen/Dense>
#include "EncoderCounter.h"
#include "FastPWM.h"
#include "GPA.h"
#include "IIRFilter.h"
#include "LinearCharacteristics3.h"
#include "LineFollower.h"
#include "LSM9DS1.h"
#include "Mahony.h"
#include "MedianFilter3.h"
#include "Motion.h"
#include "PIDCntrl.h"
#include "SDWriter.h"
#include "SensorBar.h"
#include "serial_pipe.h"
#include "SerialStream.h"
#include "Stepper.h"
#include "ThreadFlag.h"

#define M_PIf 3.14159265358979323846f // pi

#define RUN_REALTIME_THREAD_EXAMPLE false

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
        WAIT,
        FORWARD,
        BACKWARD
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
    DigitalOut led2(PB_8);

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                       // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                       // is a defined potential

    // ir distance sensor with average filter and implicit calibration
    float ir_distance_avg = 0.0f;
    IRSensor ir_sensor(PC_2);                      // before the calibration the read function will return the averaged mV value
    ir_sensor.setCalibration(2.574e+04f, -29.37f); // after the calibration the read function will return the calibrated value

    // real time thread template
    RealTimeThread real_time_thread(1000000);

    class MyRealTimeThread : public RealTimeThread {
        // for every constructor that exists in RealTimeThread, add a corresponding constructor to the
        // overload set of MyRealTimeThread that simply forwards its arguments to the base‐class constructor
        using RealTimeThread::RealTimeThread;
    protected:
        void executeTask() override {
            static uint32_t run_cntr = 0;
            // avoid printf in real-time threads by default, this is just an example!
            printf("MyRealTimeThread is enabled and runs for the %lu time\n", run_cntr++);
        }
    };
    MyRealTimeThread my_real_time_thread(2000000);

    // TODO RealTimeThread:
    // - check diagnostics of the real-time thread, e.g. execution time, max. execution time, etc.
    // - alter it so that i can use inheritance and also use relevant objects for mahony, dc motor control and line follower
    // - ... many things ...

    // servo
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);
    Servo servo_D2(PB_D2);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // futuba S3001
    float servo_D0_ang_min = 0.0150f;
    float servo_D0_ang_max = 0.1150f;
    // modelcraft RS2 MG/BB
    float servo_D1_ang_min = 0.0325f;
    float servo_D1_ang_max = 0.1250f;
    // reely S0090
    float servo_D2_ang_min = 0.0325f;
    float servo_D2_ang_max = 0.1175f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);

    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(1.0f);
    servo_D1.setMaxAcceleration(1.0f);
    servo_D2.setMaxAcceleration(1.0f);

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack

    // https://www.pololu.com/product/3475/specs
    const float gear_ratio_M1 = 31.25f; // gear ratio
    const float kn_M1 = 450.0f / 12.0f; // motor constant [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // enable the motion planner for smooth movement
    motor_M1.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);
    motor_M1.setMaxVelocity(motor_M1.getMaxVelocity() * 0.5f);
    // const float velocity_max_M1 = kn_M1 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M1.setMaxVelocity(velocity_max_M1 * 0.5f);           // set maximum velocity to 50% of maximum velocity

    // https://www.pololu.com/product/3485/specs
    const float gear_ratio_M2 = 488.28125f; // gear ratio
    const float kn_M2 = 28.0f / 12.0f;      // motor constant [rpm/V]
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // enable the motion planner for smooth movement
    motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);
    motor_M2.setMaxVelocity(motor_M2.getMaxVelocity() * 0.5f);
    // const float velocity_max_M2 = kn_M2 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M2.setMaxVelocity(velocity_max_M2 * 0.5f);           // set maximum velocity to 50% of maximum velocity

    // https://www.pololu.com/product/3477/specs
    const float gear_ratio_M3 = 78.125f; // gear ratio
    const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);
    motor_M3.setMaxVelocity(motor_M3.getMaxVelocity() * 0.5f);
    // const float velocity_max_M3 = kn_M3 / 60.0f * voltage_max; // maximum velocity in rotations per second
    // motor_M3.setMaxVelocity(velocity_max_M3 * 0.5f);           // set maximum velocity to 50% of maximum velocity

    // // adjust fast pwm frequency
    // int period_mus = 2000; // 500 Hz
    // motor_M1.setFastPWMPeriod_mus(period_mus);
    // motor_M2.setFastPWMPeriod_mus(period_mus);
    // motor_M3.setFastPWMPeriod_mus(period_mus);

    const float motor_setpoint_M1 = 300.0f / gear_ratio_M1;
    const float motor_setpoint_M2 = 300.0f / gear_ratio_M2;
    const float motor_setpoint_M3 = 300.0f / gear_ratio_M3;

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);

    // sd card logger
    SDLogger sd_logger(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK, PB_SD_CS);

    // additional timer to measure time
    Timer logging_timer;
    logging_timer.start();

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

            // read analog input
            ir_distance_avg = ir_sensor.read();

            // enable real time threads
#if RUN_REALTIME_THREAD_EXAMPLE
            real_time_thread.enable();
            my_real_time_thread.enable();
#endif

            // read us sensor distance, only valid measurements will update us_distance_cm
            const float us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f)
                us_distance_cm = us_distance_cm_candidate;

            // read imu data
            imu_data = imu.getImuData();

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    // enable the servos
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    if (!servo_D1.isEnabled())
                        servo_D1.enable();
                    if (!servo_D2.isEnabled())
                        servo_D2.enable();

                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    motor_M1.setRotation(0.0f);
                    motor_M2.setRotation(0.0f);
                    motor_M3.setRotation(0.0f);

                    robot_state = RobotState::WAIT;

                    break;
                }
                case RobotState::WAIT: {
                    if (mechanical_button.read()) {
                        led2 = 1;

                        servo_D0.setPulseWidth(1.0f);
                        servo_D1.setPulseWidth(1.0f);
                        servo_D2.setPulseWidth(1.0f);

                        motor_M1.setRotation(motor_setpoint_M1);
                        motor_M2.setRotation(motor_setpoint_M2);
                        motor_M3.setRotation(motor_setpoint_M3);

                        robot_state = RobotState::FORWARD;
                    }

                    break;
                }
                case RobotState::FORWARD: {
                    if (motor_M3.getRotation() >= 0.95f * motor_setpoint_M3) {

                        servo_D0.setPulseWidth(0.5f);
                        servo_D1.setPulseWidth(0.5f);
                        servo_D2.setPulseWidth(0.5f);

                        motor_M1.setRotation(0.5f * motor_setpoint_M1);
                        motor_M2.setRotation(0.5f * motor_setpoint_M2);
                        motor_M3.setRotation(0.5f * motor_setpoint_M3);

                        robot_state = RobotState::BACKWARD;
                    }

                    break;
                }
                case RobotState::BACKWARD: {
                    if (motor_M3.getRotation() <= 0.55f * motor_setpoint_M3) {

                        servo_D0.setPulseWidth(0.0f);
                        servo_D1.setPulseWidth(0.0f);
                        servo_D2.setPulseWidth(0.0f);

                        motor_M1.setRotation(0.0f);
                        motor_M2.setRotation(0.0f);
                        motor_M3.setRotation(0.0f);

                        led2 = 0;

                        robot_state = RobotState::WAIT;
                    }

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
                led1 = led2 = 0;
                ir_distance_avg = 0.0f;
                real_time_thread.disable();
                my_real_time_thread.disable();
                us_distance_cm = 0.0f;
                imu_data.init();
                servo_D0.setPulseWidth(0.0f);
                servo_D1.setPulseWidth(0.0f);
                servo_D2.setPulseWidth(0.0f);
                // enable_motors = 0; // do not disable motors for this specific example
                motor_M1.setRotation(0.0f);
                motor_M2.setRotation(0.0f);
                motor_M3.setRotation(0.0f);
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        printf("IR cm: %6.2f, US cm: %6.2f, R deg: %6.2f, P deg: %6.2f, Y deg: %6.2f, M1 rot: %6.2f, %6.2f, M2 rot: %6.2f, %6.2f, M3 rot: %6.2f, %6.2f \n",
            ir_distance_avg,
            us_distance_cm,
            imu_data.rpy(0) * (180.0f / M_PIf),
            imu_data.rpy(1) * (180.0f / M_PIf),
            imu_data.rpy(2) * (180.0f / M_PIf),
            motor_M1.getRotationTarget(),
            motor_M1.getRotation(),
            motor_M2.getRotationTarget(),
            motor_M2.getRotation(),
            motor_M3.getRotationTarget(),
            motor_M3.getRotation());

        // measure delta time
        static microseconds time_previous_us{0}; // static variables are only initialized once
        const microseconds time_us = logging_timer.elapsed_time();
        const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // write data to the internal buffer of the sd card logger and send it to the sd card
        sd_logger.write(dtime_us);
        sd_logger.write(ir_distance_avg);
        sd_logger.write(us_distance_cm);
        sd_logger.write(imu_data.rpy(0) * (180.0f / M_PIf));
        sd_logger.write(imu_data.rpy(1) * (180.0f / M_PIf));
        sd_logger.write(imu_data.rpy(2) * (180.0f / M_PIf));
        sd_logger.write(motor_M1.getRotationTarget());
        sd_logger.write(motor_M1.getRotation());
        sd_logger.write(motor_M2.getRotationTarget());
        sd_logger.write(motor_M2.getRotation());
        sd_logger.write(motor_M3.getRotationTarget());
        sd_logger.write(motor_M3.getRotation());
        sd_logger.send();

        // for the evaluation see: PES_Board\docs\dev\dev_sdcard\sdcard_eval_pes_monster.m

        // // debugging old (not updated but might be useful one day)
        // int time_ms = duration_cast<milliseconds>(logging_timer.elapsed_time()).count();
        // DCMotor* DCMotor_ptr = &motor_M3;
        // printf("%d, %d, %f, %f, %f, %f, %f, %f, %f, %f\n",
        //         time_ms,
        //         robot_state,
        //         DCMotor_ptr->getRotationTarget(),
        //         DCMotor_ptr->getRotationSetpoint(),
        //         DCMotor_ptr->getRotation(),
        //         DCMotor_ptr->getVelocityTarget(),
        //         DCMotor_ptr->getVelocitySetpoint(),
        //         DCMotor_ptr->getVelocity(),
        //         DCMotor_ptr->getVoltage(),
        //         DCMotor_ptr->getPWM());

        // // debugging old (not updated but might be useful one day)
        // float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(logging_timer.elapsed_time()).count() * 1.0e-3f;
        // printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, ", imu_data.gyro(0), imu_data.gyro(1), imu_data.gyro(2),
        //                                                                        imu_data.acc(0) , imu_data.acc(1) , imu_data.acc(2) ,
        //                                                                        imu_data.mag(0) , imu_data.mag(1) , imu_data.mag(2) , time_ms );
        // printf("%.6f, %.6f, %.6f, %.6f, ", imu_data.quat.w(), imu_data.quat.x(), imu_data.quat.y(), imu_data.quat.z() );
        // printf("%.6f, %.6f, %.6f, ", imu_data.rpy(0), imu_data.rpy(1), imu_data.rpy(2) );
        // printf("%.6f\n", imu_data.tilt);

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
