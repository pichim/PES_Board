#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include <Eigen/Dense>
#include "IMU.h"
#include "Servo.h"

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

    // servo
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // modelcraft RS2 MG/BB
    float servo_ang_min = 0.035f;
    float servo_ang_max = 0.130f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    // angle limits of the servos
    const float angle_range_min = -M_PIf / 2.0f;
    const float angle_range_max =  M_PIf / 2.0f;

    // angle to pulse width coefficients
    const float normalised_angle_gain = 1.0f / M_PIf;
    const float normalised_angle_offset = 0.5f;

    // pulse width
    static float roll_servo_width = 0.5f;
    static float pitch_servo_width = 0.5f;

    servo_roll.setPulseWidth(roll_servo_width);
    servo_pitch.setPulseWidth(pitch_servo_width);

    // linear 1-D mahony filter
    const float Ts = static_cast<float>(main_task_period_ms) * 1.0e-3f; // sample time in seconds
    const float kp = 3.0f; // time constant of the filter is 1/kp, kp > 0.0f
    float roll_estimate = 0.0f;
    float pitch_estimate = 0.0f;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        // read imu data
        imu_data = imu.getImuData();

        // 3-D mahony filter
        // // roll, pitch, yaw according to Tait-Bryan angles ZYX
        // const float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2)); // roll angle from accelerometer
        // const float pitch_acc = atan2f(-imu_data.acc(0), sqrtf(imu_data.acc(1) * imu_data.acc(1) + imu_data.acc(2) * imu_data.acc(2))); // pitch angle from accelerometer
        // // [dphi; dtheta; dpsi] = OmegaInv * [wx; wy; wz]
        // // OmegaInv = [[ 1, sin(phi)*tan(theta),  cos(phi)*tan(theta)],
        // //             [ 0,           cos(phi),           -sin(phi)],
        // //             [ 0, sin(phi)/cos(theta), cos(phi)/cos(theta)]]
        // Eigen::Matrix3f OmegaInv;
        // OmegaInv << 1.0f, sinf(roll_estimate) * tanf(pitch_estimate),  cosf(roll_estimate) * tanf(pitch_estimate),
        //             0.0f,                       cosf(roll_estimate ),                       -sinf(roll_estimate ),
        //             0.0f, sinf(roll_estimate) / cosf(pitch_estimate),  cosf(roll_estimate) / cosf(pitch_estimate);
        // Eigen::Vector3f omega;
        // omega << imu_data.gyro(0), imu_data.gyro(1), imu_data.gyro(2); // [wx, wy, wz]
        // Eigen::Vector3f dang = OmegaInv * omega; // [dphi; dtheta; dpsi]
        // roll_estimate  += Ts * (dang(0) + kp * (roll_acc  - roll_estimate ));
        // pitch_estimate += Ts * (dang(1) + kp * (pitch_acc - pitch_estimate));

        // pitch, roll, yaw according to Tait-Bryan angles ZXY
        const float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2)); // pitch angle from accelerometer
        const float roll_acc = atan2f(imu_data.acc(1), sqrtf(imu_data.acc(0) * imu_data.acc(0) + imu_data.acc(2) * imu_data.acc(2))); // roll angle from accelerometer
        // [dtheta; dphi; dpsi] = OmegaInv * [wy; wx; wz]
        // OmegaInv = [[1,  tan(phi)*sin(theta), -cos(theta)*tan(phi)],
        //             [0,          cos(theta),           sin(theta)],
        //             [0, -sin(theta)/cos(phi),  cos(theta)/cos(phi)]]
        Eigen::Matrix3f OmegaInv;
        OmegaInv << 1.0f,  tanf(roll_estimate ) * sinf(pitch_estimate), -cosf(pitch_estimate) * tanf(roll_estimate ),
                    0.0f,                         cosf(pitch_estimate),                         sinf(pitch_estimate),
                    0.0f, -sinf(pitch_estimate) / cosf(roll_estimate ),  cosf(pitch_estimate) / cosf(roll_estimate );
        Eigen::Vector3f omega;
        omega << imu_data.gyro(1), imu_data.gyro(0), imu_data.gyro(2); // [wy, wx, wz]
        Eigen::Vector3f dang = OmegaInv * omega; // [dtheta; dphi; dpsi]
        pitch_estimate += Ts * (dang(0) + kp * (pitch_acc - pitch_estimate));
        roll_estimate  += Ts * (dang(1) + kp * (roll_acc  - roll_estimate ));

        // update angles for the servos
        rp(0) = roll_estimate; // roll angle
        rp(1) = pitch_estimate; // pitch angle

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // enable the servos
            if (!servo_roll.isEnabled())
                servo_roll.enable();
            if (!servo_pitch.isEnabled())
                servo_pitch.enable();

            // map to servo commands
            roll_servo_width  = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            pitch_servo_width =  normalised_angle_gain * rp(1) + normalised_angle_offset;
            if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                servo_roll.setPulseWidth(roll_servo_width);
            if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                servo_pitch.setPulseWidth(pitch_servo_width);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                roll_servo_width = 0.5f;
                pitch_servo_width = 0.5f;
                servo_roll.setPulseWidth(roll_servo_width);
                servo_pitch.setPulseWidth(pitch_servo_width);
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        printf("%6.2f, %6.2f \n", roll_servo_width, pitch_servo_width);
        // printf("%6.2f, %6.2f ", imu_data.rpy(0) * 180.0f / M_PIf, imu_data.rpy(1) * 180.0f / M_PIf);
        // printf("%6.2f, %6.2f \n", roll_estimate * 180.0f / M_PIf, pitch_estimate * 180.0f / M_PIf);
        // printf("%6.2f, %6.2f ", imu_data.pry(1) * 180.0f / M_PIf, imu_data.pry(0) * 180.0f / M_PIf);
        // printf("%6.2f, %6.2f \n", roll_estimate * 180.0f / M_PIf, pitch_estimate * 180.0f / M_PIf);

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
