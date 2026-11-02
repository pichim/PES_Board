#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"
#include "DCMotor.h"
#include <cstdint>
#include <cmath>

// PES-Board 2026: Combined resource and peripheral test
// Requires PB_FTDI_UART_TX = PC_10, PB_FTDI_UART_RX = PC_11.
// STEP loopbacks: PB9 -> PA4; PB8 -> PA5 (PA5 NOT stepper RESET here).
// SPI loopbacks: PB5 -> PB4; PC3 -> PB14.
// UART loopback: PC10 -> PC11.
// ADC: A0..A2 -> GND; A3 -> 3.3 V.

constexpr int DC_PWM_PERIOD_US = 50;        // 20 kHz
constexpr int SERVO_PERIOD_US = 20000;      // 50 Hz
constexpr int STEP1_PERIOD_US = 20000;      // 50 Hz, TIM11
constexpr int STEP2_PERIOD_US = 12500;      // 80 Hz, TIM10
constexpr int SAMPLE_DELAY_MS = 50;
constexpr int SAMPLES_PER_PHASE = 20;
constexpr int SPI_REPEATS_PER_SAMPLE = 10;
constexpr int UART_BAUD = 460800;
constexpr int UART_TIMEOUT_US = 20000;
constexpr int SPI_BAUD = 4000000;
constexpr float VREF = 3.3f;
constexpr float KN = 180.0f / 12.0f;
constexpr float VOLTAGE_MAX = 12.0f;
constexpr float ENCODER_CPR = 20.0f;
constexpr float STEP_TOLERANCE_HZ = 2.0f;

volatile uint32_t step1_edges = 0;
volatile uint32_t step2_edges = 0;
volatile bool start_test = false;

void onStep1Rise() { ++step1_edges; }
void onStep2Rise() { ++step2_edges; }
void triggerTest() { start_test = true; }

struct ServoPhase {
    const char* name;
    int d0, d1, d2, d3;
};

const ServoPhase phases[] = {
    {"Phase A", 1250, 1400, 1600, 1750},
    {"Phase B", 1750, 1600, 1400, 1250},
    {"Phase C", 1500, 1500, 1500, 1500}
};

const uint8_t patterns[] = {
    0x00, 0xFF, 0x55, 0xAA, 0xA5,
    0x5A, 0x96, 0x69, 0x3C, 0xC3
};
constexpr int PATTERN_COUNT = sizeof(patterns) / sizeof(patterns[0]);
constexpr uint32_t EXPECTED_TRANSFERS =
    3 * SAMPLES_PER_PHASE * SPI_REPEATS_PER_SAMPLE * PATTERN_COUNT;

// For non-blocking UnbufferedSerial, send and receive one byte per transfer.
// Reading the echoed byte before sending another avoids RX FIFO overrun.
bool uartLoopbackByte(UnbufferedSerial& uart, uint8_t tx, uint8_t& rx)
{
    Timer timeout;
    timeout.start();

    while (uart.write(&tx, 1) != 1) {
        if (timeout.elapsed_time().count() >= UART_TIMEOUT_US)
            return false;
        wait_us(5);
    }

    timeout.reset();
    while (uart.read(&rx, 1) != 1) {
        if (timeout.elapsed_time().count() >= UART_TIMEOUT_US)
            return false;
        wait_us(5);
    }
    return true;
}

int main()
{
    // Shared motor enable and fault
    DigitalOut dc_enable(PB_ENABLE_DCMOTORS, DISABLE_DCMOTORS);
    DigitalOut step_enable(PB_ENABLE_STEPPERMOTORS, DISABLE_STEPPERMOTORS);
    DigitalIn fault(PB_DC_MOT_STATE, PullUp);

    // Three independent DC motor controllers
    DCMotor m1(PB_PWM_M1, PB_DIR_M1, PB_ENC_A_M1, PB_ENC_B_M1,
               156.0f, KN, VOLTAGE_MAX, ENCODER_CPR);
    DCMotor m2(PB_PWM_M2, PB_DIR_M2, PB_ENC_A_M2, PB_ENC_B_M2,
               31.0f, KN, VOLTAGE_MAX, ENCODER_CPR);
    DCMotor m3(PB_PWM_M3, PB_DIR_M3, PB_ENC_A_M3, PB_ENC_B_M3,
               78.125f, KN, VOLTAGE_MAX, ENCODER_CPR);

    m1.setFastPWMPeriod_mus(DC_PWM_PERIOD_US);
    m2.setFastPWMPeriod_mus(DC_PWM_PERIOD_US);
    m3.setFastPWMPeriod_mus(DC_PWM_PERIOD_US);

    m1.disableMotionPlanner();
    m1.setMaxVelocity(0.40f);
    m1.setVelocity(0.0f);

    m2.enableMotionPlanner();
    m2.setMaxVelocity(0.80f);
    m2.setMaxAcceleration(2.0f);
    m2.setVelocity(0.0f);

    m3.enableMotionPlanner();
    m3.setMaxVelocity(0.40f);
    m3.setMaxAcceleration(1.0f);
    m3.setRotation(0.0f);

    // Servo outputs: TIM8
    PwmOut d0(PB_D0), d1(PB_D1), d2(PB_D2), d3(PB_D3);
    d0.period_us(SERVO_PERIOD_US);
    d1.period_us(SERVO_PERIOD_US);
    d2.period_us(SERVO_PERIOD_US);
    d3.period_us(SERVO_PERIOD_US);
    d0.pulsewidth_us(1500);
    d1.pulsewidth_us(1500);
    d2.pulsewidth_us(1500);
    d3.pulsewidth_us(1500);

    // Four ADC inputs
    AnalogIn a0(PB_A0), a1(PB_A1), a2(PB_A2), a3(PB_A3);
    AnalogIn* adc[4] = {&a0, &a1, &a2, &a3};
    float adc_min[4] = {1, 1, 1, 1};
    float adc_max[4] = {0, 0, 0, 0};
    int adc_failures[4] = {0, 0, 0, 0};
    uint32_t adc_samples = 0;

    // Independent SPI buses at 4 MHz
    DigitalOut sd_cs(PB_SD_CS, 1), imu_cs(PB_IMU_CS, 1);
    SPI spi1(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK);
    SPI spi2(PB_IMU_MOSI, PB_IMU_MISO, PB_IMU_SCK);
    spi1.format(8, 0);
    spi2.format(8, 0);
    spi1.frequency(SPI_BAUD);
    spi2.frequency(SPI_BAUD);
    uint32_t spi1_errors = 0, spi2_errors = 0, spi_transfers = 0;

    // TIM11 and TIM10: step pulses, without stepper drivers
    PwmOut step1(PB_STEP_STEPPERMOTOR1);
    PwmOut step2(PB_STEP_STEPPERMOTOR2);
    step1.write(0.0f);
    step2.write(0.0f);
    step1.period_us(STEP1_PERIOD_US);
    step2.period_us(STEP2_PERIOD_US);
    DigitalOut dir1(PB_DIR_STEPPERMOTOR1, 0);
    DigitalOut dir2(PB_DIR_STEPPERMOTOR2, 1);

    // Loopback interrupt inputs. PA5 cannot also be STEP RESET.
    InterruptIn irq1(PA_4);
    InterruptIn irq2(PA_5);
    irq1.mode(PullDown);
    irq2.mode(PullDown);
    irq1.rise(&onStep1Rise);
    irq2.rise(&onStep2Rise);

    // USART3 (PC10 -> PC11) independent from ST-LINK USART2 (printf)
    UnbufferedSerial ftdi(PB_FTDI_UART_TX, PB_FTDI_UART_RX, UART_BAUD);
    ftdi.format(8, SerialBase::None, 1);
    ftdi.set_blocking(false);
    uint32_t uart_errors = 0, uart_transfers = 0;

    DebounceIn button(BUTTON1);
    button.fall(&triggerTest);

    printf("\n====================================\n");
    printf("PES Board 2026 - FULL HW + UART TEST\n");
    printf("====================================\n");
    printf("DC: TIM1 PWM 20kHz, encoder TIM2/3/4\n");
    printf("Servos: TIM8, 50 Hz\n");
    printf("STEP: TIM11 50 Hz, TIM10 80 Hz\n");
    printf("ADC: 4 channels\n");
    printf("SPI1 + SPI2: 4 MHz, loopback\n");
    printf("USART3: 460800 baud, loopback\n");
    printf("Press blue button to start.\n");

    while (!start_test)
        thread_sleep_for(20);

    bool ok = true;
    dc_enable = ENABLE_DCMOTORS;
    thread_sleep_for(20);
    if (fault.read() == 0) {
        printf("FAULT on enable!\n");
        ok = false;
    }

    if (ok) {
        m1.setVelocity(0.20f);
        m2.setVelocity(0.50f);
        m3.setRotation(0.125f); // 45 degrees
    }

    Timer step_timer;
    if (ok) {
        step1_edges = 0;
        step2_edges = 0;
        step_timer.start();
        step1.write(0.5f);
        step2.write(0.5f);
    }

    if (ok) {
        for (int p = 0; p < 3 && ok; p++) {
            const ServoPhase& phase = phases[p];
            d0.pulsewidth_us(phase.d0);
            d1.pulsewidth_us(phase.d1);
            d2.pulsewidth_us(phase.d2);
            d3.pulsewidth_us(phase.d3);

            printf("\n========== %s ==========\n", phase.name);
            printf("Servo: %d %d %d %d us\n",
                   phase.d0, phase.d1, phase.d2, phase.d3);

            for (int i = 0; i < SAMPLES_PER_PHASE && ok; i++) {
                float value[4];
                for (int ch = 0; ch < 4; ch++) {
                    value[ch] = adc[ch]->read();
                    if (value[ch] < adc_min[ch]) adc_min[ch] = value[ch];
                    if (value[ch] > adc_max[ch]) adc_max[ch] = value[ch];
                    const bool valid = (ch == 3)
                        ? value[ch] > 0.95f
                        : value[ch] < 0.05f;
                    if (!valid) adc_failures[ch]++;
                }
                adc_samples++;

                // 100 bytes per sample on each SPI bus and USART3
                for (int n = 0; n < SPI_REPEATS_PER_SAMPLE && ok; n++) {
                    for (uint8_t tx : patterns) {
                        sd_cs = 0;
                        uint8_t rx1 = static_cast<uint8_t>(spi1.write(tx));
                        sd_cs = 1;
                        if (rx1 != tx) spi1_errors++;

                        uint8_t tx2 = static_cast<uint8_t>(~tx);
                        imu_cs = 0;
                        uint8_t rx2 = static_cast<uint8_t>(spi2.write(tx2));
                        imu_cs = 1;
                        if (rx2 != tx2) spi2_errors++;
                        spi_transfers++;

                        uint8_t uart_rx = 0;
                        const bool received = uartLoopbackByte(ftdi, tx, uart_rx);
                        uart_transfers++;
                        if (!received || uart_rx != tx) {
                            uart_errors++;
                            if (uart_errors <= 5) {
                                printf("UART ERROR: TX=%02X RX=%02X timeout=%d\n",
                                       (unsigned)tx, (unsigned)uart_rx,
                                       received ? 0 : 1);
                            }
                            // Avoid a long sequence of timeout waits with bad wiring.
                            if (uart_errors >= 10) {
                                printf("UART: too many errors, aborting.\n");
                                ok = false;
                                break;
                            }
                        }
                    }
                }

                if (fault.read() == 0) {
                    printf("DC MOTOR FAULT!\n");
                    ok = false;
                }

                // Same loose-M3-cable check as in previous combined test
                if (p == 0 && i == 9 &&
                    fabsf(m3.getRotation()) < 2.0f / 360.0f) {
                    printf("ERROR: M3 did not move!\n");
                    ok = false;
                }

                if ((i + 1) % 5 == 0) {
                    printf(
                        "t~%4d | ADC %.3f %.3f %.3f %.3f V | "
                        "M1=%+.3f M2=%+.3f M3=%+.1f deg | STEP=%lu/%lu\n",
                        (i + 1) * SAMPLE_DELAY_MS,
                        value[0] * VREF, value[1] * VREF,
                        value[2] * VREF, value[3] * VREF,
                        m1.getVelocity(), m2.getVelocity(),
                        m3.getRotation() * 360.0f,
                        (unsigned long)step1_edges,
                        (unsigned long)step2_edges);
                }

                if (ok) thread_sleep_for(SAMPLE_DELAY_MS);
            }
        }
    }

    // Stop STEP before time/count evaluation (avoid printf time skew).
    step1.write(0.0f);
    step2.write(0.0f);
    step_timer.stop();
    const float step_elapsed_s =
        step_timer.elapsed_time().count() / 1000000.0f;
    const uint32_t count1 = step1_edges;
    const uint32_t count2 = step2_edges;

    // Capture steady-state motor values before shutdown.
    const float final_m1 = m1.getVelocity();
    const float final_m2 = m2.getVelocity();
    const float final_m3_deg = m3.getRotation() * 360.0f;
    const bool motors_passed = ok &&
        fabsf(final_m1 - 0.20f) < 0.05f &&
        fabsf(final_m2 - 0.50f) < 0.10f &&
        fabsf(final_m3_deg - 45.0f) < 5.0f;

    // Disable power stage before printing the long summaries.
    dc_enable = DISABLE_DCMOTORS;
    step_enable = DISABLE_STEPPERMOTORS;
    m1.setVelocity(0.0f);
    m2.setVelocity(0.0f);
    m3.setRotation(m3.getRotation());
    d0.pulsewidth_us(0);
    d1.pulsewidth_us(0);
    d2.pulsewidth_us(0);
    d3.pulsewidth_us(0);
    sd_cs = 1;
    imu_cs = 1;

    printf("\n========== MOTOR SUMMARY ==========\n");
    printf("M1: %.3f rev/s (target 0.200)\n", final_m1);
    printf("M2: %.3f rev/s (target 0.500)\n", final_m2);
    printf("M3: %.2f deg (target 45.00)\n", final_m3_deg);
    printf("Motors: %s\n", motors_passed ? "PASSED" : "FAILED");

    int total_adc_failures = 0;
    printf("\n========== ADC SUMMARY ==========\n");
    for (int ch = 0; ch < 4; ch++) {
        total_adc_failures += adc_failures[ch];
        printf("A%d: min=%.4f V max=%.4f V errors=%d\n",
               ch, adc_min[ch] * VREF, adc_max[ch] * VREF,
               adc_failures[ch]);
    }
    const bool adc_passed =
        adc_samples == 60 && total_adc_failures == 0;
    printf("ADC: %s\n", adc_passed ? "PASSED" : "FAILED");

    printf("\n========== SPI SUMMARY ==========\n");
    printf("SPI1: %lu transfers, %lu errors\n",
           (unsigned long)spi_transfers, (unsigned long)spi1_errors);
    printf("SPI2: %lu transfers, %lu errors\n",
           (unsigned long)spi_transfers, (unsigned long)spi2_errors);
    const bool spi_passed =
        spi_transfers == EXPECTED_TRANSFERS &&
        spi1_errors == 0 && spi2_errors == 0;
    printf("SPI: %s\n", spi_passed ? "PASSED" : "FAILED");

    printf("\n========== UART SUMMARY ==========\n");
    printf("USART3: %lu transfers, %lu errors @ %d baud\n",
           (unsigned long)uart_transfers,
           (unsigned long)uart_errors, UART_BAUD);
    const bool uart_passed =
        uart_transfers == EXPECTED_TRANSFERS && uart_errors == 0;
    printf("UART: %s\n", uart_passed ? "PASSED" : "FAILED");

    const float step_freq1 = step_elapsed_s > 0.0f
        ? count1 / step_elapsed_s : 0.0f;
    const float step_freq2 = step_elapsed_s > 0.0f
        ? count2 / step_elapsed_s : 0.0f;
    const bool step_passed = step_elapsed_s > 0.0f &&
        fabsf(step_freq1 - 50.0f) <= STEP_TOLERANCE_HZ &&
        fabsf(step_freq2 - 80.0f) <= STEP_TOLERANCE_HZ;

    printf("\n========== STEPPER SUMMARY ==========\n");
    printf("Duration: %.3f s\n", step_elapsed_s);
    printf("STEP1 TIM11: %lu edges, %.2f Hz\n",
           (unsigned long)count1, step_freq1);
    printf("STEP2 TIM10: %lu edges, %.2f Hz\n",
           (unsigned long)count2, step_freq2);
    printf("STEP: %s\n", step_passed ? "PASSED" : "FAILED");

    const bool all_passed = motors_passed && adc_passed &&
                            spi_passed && uart_passed && step_passed;
    printf("\n====================================\n");
    printf("FULL HARDWARE + UART TEST: %s\n",
           all_passed ? "PASSED" : "FAILED");
    printf("====================================\n");
    printf("Motor drivers disabled; servo and STEP pulses disabled.\n");
    printf("Reset board to repeat.\n");

    while (true) thread_sleep_for(1000);
}
