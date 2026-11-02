
#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"
#include "DCMotor.h"

// -----------------------------------------
// Test parameters
// -----------------------------------------
constexpr int PWM_PERIOD_US = 50;       // 20 kHz
constexpr int LOG_PERIOD_MS = 250;
constexpr int PHASE_DURATION_MS = 2500;

constexpr float M1_VELOCITY = 0.20f;   // rev/s
constexpr float M2_VELOCITY = 0.50f;   // rev/s
constexpr float M3_POSITION = 0.125f;  // rev = 45 degrees

constexpr float VOLTAGE_MAX = 12.0f;
constexpr float KN = 180.0f / 12.0f;
constexpr float ENCODER_CPR = 20.0f;

// -----------------------------------------
// Button interrupt
// -----------------------------------------
volatile bool start_test = false;

void trigger_test()
{
    start_test = true;
}

// -----------------------------------------
// Logging function
// -----------------------------------------
bool logPhase(const char* phase,
              DCMotor& m1,
              DCMotor& m2,
              DCMotor& m3,
              DigitalIn& fault,
              float m3_target_deg)
{
    printf("\n========== %s ==========\n", phase);

    for (int t = LOG_PERIOD_MS;
         t <= PHASE_DURATION_MS;
         t += LOG_PERIOD_MS) {

        thread_sleep_for(LOG_PERIOD_MS);

        if (fault.read() == 0) {
            printf("FAULT detected at %d ms!\n", t);
            return false;
        }

        printf("t=%4d ms\n", t);

        // M1: Velocity without planner
        printf("  M1: target=%+.3f actual=%+.3f pwm=%.3f\n",
               m1.getVelocityTarget(),
               m1.getVelocity(),
               m1.getPWM());

        // M2: Velocity with planner
        printf("  M2: target=%+.3f setpoint=%+.3f "
               "actual=%+.3f pwm=%.3f\n",
               m2.getVelocityTarget(),
               m2.getVelocitySetpoint(),
               m2.getVelocity(),
               m2.getPWM());

        // M3: Position with planner
        printf("  M3: target=%+.1f deg pos=%+.1f deg "
               "vel=%+.3f pwm=%.3f\n",
               m3_target_deg,
               m3.getRotation() * 360.0f,
               m3.getVelocity(),
               m3.getPWM());
    }

    printf("Encoder counts: M1=%ld M2=%ld M3=%ld\n",
           m1.getEncoderCount(),
           m2.getEncoderCount(),
           m3.getEncoderCount());

    return true;
}

// -----------------------------------------
// Main
// -----------------------------------------
int main()
{
    // Shared hardware
    DigitalOut enable(PB_ENABLE_DCMOTORS,
                      DISABLE_DCMOTORS);
    DigitalIn fault(PB_DC_MOT_STATE, PullUp);

    // M1: Gearbox 156:1
    DCMotor motor_M1(
        PB_PWM_M1, PB_DIR_M1,
        PB_ENC_A_M1, PB_ENC_B_M1,
        156.0f, KN, VOLTAGE_MAX, ENCODER_CPR
    );

    // M2: Gearbox 31:1
    DCMotor motor_M2(
        PB_PWM_M2, PB_DIR_M2,
        PB_ENC_A_M2, PB_ENC_B_M2,
        31.0f, KN, VOLTAGE_MAX, ENCODER_CPR
    );

    // M3: Gearbox 78.125:1
    DCMotor motor_M3(
        PB_PWM_M3, PB_DIR_M3,
        PB_ENC_A_M3, PB_ENC_B_M3,
        78.125f, KN, VOLTAGE_MAX, ENCODER_CPR
    );

    // -------------------------------------
    // Configure motors
    // -------------------------------------

    // Common PWM frequency
    motor_M1.setFastPWMPeriod_mus(PWM_PERIOD_US);
    motor_M2.setFastPWMPeriod_mus(PWM_PERIOD_US);
    motor_M3.setFastPWMPeriod_mus(PWM_PERIOD_US);

    // M1: Velocity, no motion planner
    motor_M1.disableMotionPlanner();
    motor_M1.setMaxVelocity(0.40f);
    motor_M1.setVelocity(0.0f);

    // M2: Velocity with motion planner
    motor_M2.enableMotionPlanner();
    motor_M2.setMaxVelocity(0.80f);
    motor_M2.setMaxAcceleration(2.0f);
    motor_M2.setVelocity(0.0f);

    // M3: Position with motion planner
    motor_M3.enableMotionPlanner();
    motor_M3.setMaxVelocity(0.40f);
    motor_M3.setMaxAcceleration(1.0f);
    motor_M3.setRotation(0.0f);

    // User button
    DebounceIn button(BUTTON1);
    button.fall(&trigger_test);

    printf("\n====================================\n");
    printf("PES Board 2026 - 3 Motor Control Test\n");
    printf("====================================\n");
    printf("M1: Velocity without planner\n");
    printf("M2: Velocity with planner\n");
    printf("M3: Position with planner\n");
    printf("PWM: 20 kHz\n");
    printf("Press blue button to start.\n\n");

    // -------------------------------------
    // Wait for button
    // -------------------------------------
    while (!start_test) {
        thread_sleep_for(20);
    }

    // One test cycle per board reset
    printf("Starting test...\n");

    enable = ENABLE_DCMOTORS;
    thread_sleep_for(20);

    bool ok = (fault.read() != 0);

    if (!ok) {
        printf("FAULT on enable!\n");
    }

    // -------------------------------------
    // Phase 1: Forward / positive position
    // -------------------------------------
    if (ok) {
        motor_M1.setVelocity(+M1_VELOCITY);
        motor_M2.setVelocity(+M2_VELOCITY);
        motor_M3.setRotation(+M3_POSITION);

        ok = logPhase(
            "PHASE 1 - FORWARD",
            motor_M1, motor_M2, motor_M3,
            fault, +45.0f
        );
    }

    // -------------------------------------
    // Phase 2: Reverse / negative position
    // -------------------------------------
    if (ok) {
        motor_M1.setVelocity(-M1_VELOCITY);
        motor_M2.setVelocity(-M2_VELOCITY);
        motor_M3.setRotation(-M3_POSITION);

        ok = logPhase(
            "PHASE 2 - REVERSE",
            motor_M1, motor_M2, motor_M3,
            fault, -45.0f
        );
    }

    // -------------------------------------
    // Phase 3: Return to zero
    // -------------------------------------
    if (ok) {
        motor_M1.setVelocity(0.0f);
        motor_M2.setVelocity(0.0f);
        motor_M3.setRotation(0.0f);

        ok = logPhase(
            "PHASE 3 - STOP / HOME",
            motor_M1, motor_M2, motor_M3,
            fault, 0.0f
        );
    }

    // -------------------------------------
    // Shutdown
    // -------------------------------------

    // Disable power stage immediately.
    enable = DISABLE_DCMOTORS;

    // Leave safe targets for inactive drivers.
    motor_M1.setVelocity(0.0f);
    motor_M2.setVelocity(0.0f);
    motor_M3.setRotation(motor_M3.getRotation());

    if (ok) {
        printf("\nTest completed successfully.\n");
    } else {
        printf("\nTest aborted due to FAULT.\n");
    }

    printf("Motor drivers disabled.\n");
    printf("Reset board before running again.\n");

    while (true) {
        thread_sleep_for(1000);
    }
}
