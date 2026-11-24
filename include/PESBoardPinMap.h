#ifndef PES_BOARD_PIN_MAP_
#define PES_BOARD_PIN_MAP_

/**
 * @file PESBoardPinMap.h
 * @brief PES-Board Nucleo-F446RE pin mapping for V2019/V2023, V2025 and V2026.
 *
 * Select exactly one hardware version, either here or with a compiler -D flag:
 *   PESBOARD_V2019_2023   (shared mapping of V2019 and V2023)
 *   PESBOARD_V2025
 *   PESBOARD_V2026
 *
 * If no version is selected, V2026 is used for compatibility with the
 * previous PESBoardPinMap.h. Explicit -D selection overrides this default.
 *
 * Pin names refer to Mbed PinName symbols (e.g. PA_11, PC_7_ALT0).
 * Do not use two alternative functions of the same MCU pin simultaneously.
 */

// -----------------------------------------------------------------------------
// 1. Hardware-version selection
// -----------------------------------------------------------------------------
#if !defined(PESBOARD_V2019_2023) && !defined(PESBOARD_V2025) && !defined(PESBOARD_V2026)
    #define PESBOARD_V2019_2023
#endif

#if (defined(PESBOARD_V2019_2023) + defined(PESBOARD_V2025) + defined(PESBOARD_V2026)) != 1
    #error "Select exactly one PES-Board version: PESBOARD_V2019_2023, PESBOARD_V2025 or PESBOARD_V2026."
#endif

// -----------------------------------------------------------------------------
// 2. Hardware capabilities and DC-motor control semantics
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    // Bipolar PWM driver: duty=0.0 -> -VMOT, duty=0.5 -> 0 V,
    // duty=1.0 -> +VMOT. The middle value is 50%, not 0.5%.
    #define PB_DC_CONTROL_BIPOLAR 1
    #define PB_DC_CONTROL_PWM_DIR 0
    #define PB_HAS_STEPPERS 0    // Stepper connector signals intentionally ignored.
    #define PB_STEPPER_TIM10_TIM11 0
    #define PB_HAS_IMU_SPI 0
    #define PB_HAS_SDCARD 1
#else
    // Separate PWM magnitude + DIR signal. Preserve DCMotor's existing
    // #ifdef NEW_PES_BOARD_VERSION behavior until its planned refactoring.
    #define PB_DC_CONTROL_BIPOLAR 0
    #define PB_DC_CONTROL_PWM_DIR 1
    #define NEW_PES_BOARD_VERSION
    #define PB_HAS_STEPPERS 1
    #if defined(PESBOARD_V2026)
        #define PB_STEPPER_TIM10_TIM11 1
        #define PB_HAS_IMU_SPI 1
        #define PB_HAS_SDCARD 1
    #else
        #define PB_STEPPER_TIM10_TIM11 0
        #define PB_HAS_IMU_SPI 0
        #define PB_HAS_SDCARD 0
    #endif
#endif

// -----------------------------------------------------------------------------
// 3. Analog inputs
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    #define PB_A0 PC_2
    #define PB_A1 PC_3
    #define PB_A2 PC_5
    #define PB_A3 PB_1
#elif defined(PESBOARD_V2025)
    #define PB_A0 PC_3
    #define PB_A1 PC_0
    #define PB_A2 PC_2
    #define PB_A3 PC_1
#else // PESBOARD_V2026
    #define PB_A0 PC_1
    #define PB_A1 PC_0
    #define PB_A2 PC_2
    #define PB_A3 PB_0
#endif

// -----------------------------------------------------------------------------
// 4. Servos / general digital outputs
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    #define PB_D0 PB_2
    #define PB_D1 PC_8
    #define PB_D2 PC_6
    #define PB_D3 PB_12
#elif defined(PESBOARD_V2025)
    #define PB_D0 PA_4
    #define PB_D1 PB_0
    #define PB_D2 PA_15
    #define PB_D3 PC_13  // Shares the Nucleo user-button pin.
#else // PESBOARD_V2026
    #define PB_D0 PC_7_ALT0
    #define PB_D1 PC_6_ALT0
    #define PB_D2 PC_8_ALT0
    #define PB_D3 PC_9_ALT0
#endif

// -----------------------------------------------------------------------------
// 5. DC motors: PWM, direction, encoders, enable, feedback, fault
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    #define PB_PWM_M1 PB_13
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PA_10

    #define PB_ENC_A_M1 PA_6
    #define PB_ENC_B_M1 PC_7
    #define PB_ENC_A_M2 PB_6
    #define PB_ENC_B_M2 PB_7
    #define PB_ENC_A_M3 PA_0
    #define PB_ENC_B_M3 PA_1

    #define PB_ENABLE_DCMOTORS PB_15
    // No independent DIR pins for bipolar PWM.

#elif defined(PESBOARD_V2025)
    #define PB_PWM_M1 PB_15
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PB_13

    #define PB_DIR_M1 PB_14
    #define PB_DIR_M2 PB_10
    #define PB_DIR_M3 PC_4

    #define PB_ENC_A_M1 PA_6
    #define PB_ENC_B_M1 PC_7
    #define PB_ENC_A_M2 PB_6
    #define PB_ENC_B_M2 PB_7
    #define PB_ENC_A_M3 PA_0
    #define PB_ENC_B_M3 PA_1

    #define PB_ENABLE_DCMOTORS PB_9
    #define PB_FB_M1 PA_5
    #define PB_FB_M2 PA_7
    #define PB_FB_M3 PB_1
    #define PB_DC_MOT_STATE PH_1

#else // PESBOARD_V2026
    #define PB_PWM_M1 PA_11  // SJ8/SJ7 hardware selection: PWM_M1 vs CAN_RX.
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PA_10

    #define PB_DIR_M1 PD_2
    #define PB_DIR_M2 PB_12
    #define PB_DIR_M3 PA_8

    #define PB_ENC_A_M1 PA_6
    #define PB_ENC_B_M1 PA_7
    #define PB_ENC_A_M2 PB_6
    #define PB_ENC_B_M2 PB_7
    #define PB_ENC_A_M3 PA_0
    #define PB_ENC_B_M3 PA_1

    #define PB_ENABLE_DCMOTORS PA_15  // MOT_SLEEP, asserted high for operation.
    // V2026 has no DC-motor current measurement. Do not define PB_FB_M1/M2/M3.
    #define PB_DC_MOT_STATE PA_12     // MOT1_FAULT (SJ9/SJ10) vs CAN_TX.
#endif

// Existing active-high enable convention retained for all board revisions;
// electrical polarity of earlier versions must be checked on hardware.
#define ENABLE_DCMOTORS true
#define DISABLE_DCMOTORS false

// -----------------------------------------------------------------------------
// 6. Stepper drivers (V2025 / V2026 only)
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2025)
    #define PB_STEP_STEPPERMOTOR1 PB_12
    #define PB_DIR_STEPPERMOTOR1 PB_2
    #define PB_STEP_STEPPERMOTOR2 PC_6
    #define PB_DIR_STEPPERMOTOR2 PC_5
    #define PB_ENABLE_STEPPERMOTORS PC_8
    #define PB_RESET_STEPPERMOTORS PB_8
#elif defined(PESBOARD_V2026)
    #define PB_STEP_STEPPERMOTOR1 PB_9_ALT1  // TIM11_CH1
    #define PB_DIR_STEPPERMOTOR1 PB_2
    #define PB_STEP_STEPPERMOTOR2 PB_8_ALT1  // TIM10_CH1
    #define PB_DIR_STEPPERMOTOR2 PB_15
    #define PB_ENABLE_STEPPERMOTORS PC_5
    #define PB_RESET_STEPPERMOTORS PA_5
#endif

#if PB_HAS_STEPPERS
    // Shared A4988-style active-low enable/reset signals.
    #define ENABLE_STEPPERMOTORS false
    #define DISABLE_STEPPERMOTORS true
    #define RESET_STEPPERMOTORS false
#endif

// -----------------------------------------------------------------------------
// 7. Communication interfaces and connected peripherals
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    // Shared SPI bus from original connector schematic.
    #define PB_SPI_SCK PC_10
    #define PB_SPI_MISO PC_11
    #define PB_SPI_MOSI PC_12
    #define PB_SPI_CS PD_2

    #define PB_I2C_SDA PC_9
    #define PB_I2C_SCL PA_8
    #define PB_CAN_TD PA_12
    #define PB_CAN_RD PA_11
    // Keep legacy I2C IMU names from the original header.
    #define PB_IMU_SDA PB_I2C_SDA
    #define PB_IMU_SCL PB_I2C_SCL

    // Legacy PB_SD_* aliases actually refer to the generic SPI signals.
    #define PB_SD_SCK PB_SPI_SCK
    #define PB_SD_MISO PB_SPI_MISO
    #define PB_SD_MOSI PB_SPI_MOSI
    #define PB_SD_CS PB_SPI_CS

    #define PB_UNUSED_UART_TX PB_10
    #define PB_UNUSED_UART_RX PC_5

#elif defined(PESBOARD_V2025)
    // External SPI and I2C as shown in V2025 pin schematic.
    #define PB_SPI_SCK PB_3
    #define PB_SPI_MISO PB_4
    #define PB_SPI_MOSI PB_5
    #define PB_SPI_CS PA_10

    // Legacy SPI pin names used by existing PES-Board examples
    #define PB_SD_SCK  PB_SPI_SCK
    #define PB_SD_MISO PB_SPI_MISO
    #define PB_SD_MOSI PB_SPI_MOSI
    #define PB_SD_CS   PB_SPI_CS

    #define PB_I2C_SDA PC_9
    #define PB_I2C_SCL PA_8
    #define PB_CAN_TD PA_12
    #define PB_CAN_RD PA_11
    // Keep legacy I2C IMU names from the original header.
    #define PB_IMU_SDA PB_I2C_SDA
    #define PB_IMU_SCL PB_I2C_SCL

    #define PB_UART3_TX PC_10
    #define PB_UART3_RX PC_11
    #define PB_UART5_TX PC_12
    #define PB_UART5_RX PD_2

    // Legacy UART pin names
    #define PB_UNUSED_UART_TX PB_UART3_TX
    #define PB_UNUSED_UART_RX PB_UART3_RX
    // UART3 and UART5 are external interfaces on the V2025 schematic.

#else // PESBOARD_V2026
    // Dedicated SPI to IMU (SPI2 signals distributed across port groups).
    #define PB_IMU_MISO PB_14
    #define PB_IMU_MOSI PC_3
    #define PB_IMU_SCK PB_10
    #define PB_IMU_CS PA_4  // Corrected from old PC12: schematic says PA4.

    // SD card SPI bus.
    #define PB_SD_MOSI PB_5
    #define PB_SD_MISO PB_4
    #define PB_SD_SCK PB_3
    #define PB_SD_CS PB_13

    // External SPI3 is an ALTERNATIVE to UART3 on PC10/PC11.
    #define PB_SPI_SCK PC_10
    #define PB_SPI_MISO PC_11
    #define PB_SPI_MOSI PC_12

    // Hardware links select CAN or the DC-motor signals on PA11/PA12.
    #define PB_CAN_TD PA_12
    #define PB_CAN_RD PA_11

    #define PB_FTDI_UART_TX PC_10
    #define PB_FTDI_UART_RX PC_11
    #define PB_UART3_TX PC_10
    #define PB_UART3_RX PC_11
#endif

// -----------------------------------------------------------------------------
// 8. User button and optional external LEDs
// -----------------------------------------------------------------------------
#if defined(PESBOARD_V2019_2023)
    #define PB_MECH_BUTTON PB_10   // User Button in connector drawing.
    #define PB_STATUS_LED1 PB_9    // Original header: optional external LED.
    #define PB_STATUS_LED2 PB_8    // Original header: optional external LED.
#elif defined(PESBOARD_V2025)
    #define PB_MECH_BUTTON PC_11   // Requires oscillator / solder-bridge check!

    // Legacy status LED pins
    #define PB_STATUS_LED1 PC_10
    #define PB_STATUS_LED2 PC_12

    #else // PESBOARD_V2026
    #define PB_MECH_BUTTON PC_13   // Nucleo blue user button.
    #define PB_STATUS_LED1 PC_10   // Legacy alias; conflicts with FTDI TX.
#endif

#endif // PES_BOARD_PIN_MAP_
