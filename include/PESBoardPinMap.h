#ifndef PES_BOARD_PIN_MAP_
#define PES_BOARD_PIN_MAP_

// PES-Board Pin Names
//#define NEW_PES_BOARD_VERSION
#ifdef NEW_PES_BOARD_VERSION
    // Analog inputs 
    #define PB_A0 PC_3
    #define PB_A1 PC_0
    #define PB_A2 PC_2
    #define PB_A3 PC_1

    // Servos and Ultrasonic Sensor
    #define PB_D0 PA_4
    #define PB_D1 PB_0
    #define PB_D2 PA_15
    #define PB_D3 PC_13

    // DC-Motors
    #define PB_PWM_M1 PB_15
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PB_13

    #define PB_ENC_A_M1 PA_6
    #define PB_ENC_B_M1 PC_7
    #define PB_ENC_A_M2 PB_6
    #define PB_ENC_B_M2 PB_7
    #define PB_ENC_A_M3 PA_0
    #define PB_ENC_B_M3 PA_1

    #define PB_ENABLE_DCMOTORS PB_9
    #define ENABLE_DCMOTORS true
    #define DISABLE_DCMOTORS false

    #define PB_DIR_M1 PB_14
    #define PB_DIR_M2 PB_10
    #define PB_DIR_M3 PC_4

    #define PB_FB_M1 PA_5
    #define PB_FB_M2 PA_7
    #define PB_FB_M3 PB_1

    #define PB_DC_MOT_STATE PH_1

    // IMU
    #define PB_IMU_SDA PC_9
    #define PB_IMU_SCL PA_8

    // SD-Card
    #define PB_SD_MOSI PB_5
    #define PB_SD_MISO PB_4
    #define PB_SD_SCK PB_3
    #define PB_SD_CS PA_10

    // Unused UART
    #define PB_UNUSED_UART_TX PC_10
    #define PB_UNUSED_UART_RX PC_11

    #define PB_UART5_TX PC_12
    #define PB_UART5_RX PD_2

    // Stepper Motors
    #define PB_STEP_STEPPERMOTOR1 PB_12
    #define PB_DIR_STEPPERMOTOR1 PB_2

    #define PB_STEP_STEPPERMOTOR2 PC_6
    #define PB_DIR_STEPPERMOTOR2 PC_5

    #define PB_ENABLE_STEPPERMOTORS PC_8
    #define ENABLE_STEPPERMOTORS false
    #define DISABLE_STEPPERMOTORS true

    #define PB_RESET_STEPPERMOTORS PB_8
    #define RESET_STEPPERMOTORS false

    // CAN  
    #define PB_CAN_TD PA_12
    #define PB_CAN_RD PA_11

    // User button
    //#define PB_USER_BUTTON PH_0  // Do not use: PH0 is the HSE oscillator input
    #define PB_MECH_BUTTON PC_11
#else
    // Analog Inputs
    #define PB_A0 PC_2
    #define PB_A1 PC_3
    #define PB_A2 PC_5
    #define PB_A3 PB_1

    // Servos and Ultrasonic Sensor
    #define PB_D0 PB_2
    #define PB_D1 PC_8
    #define PB_D2 PC_6
    #define PB_D3 PB_12

    // DC-Motors
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

    // IMU
    #define PB_IMU_SDA PC_9
    #define PB_IMU_SCL PA_8

    // SD-Card
    #define PB_SD_MOSI PC_12
    #define PB_SD_MISO PC_11
    #define PB_SD_SCK PC_10
    #define PB_SD_CS PD_2

    #define PB_UNUSED_UART_TX PB_10
    #define PB_UNUSED_UART_RX PC_5

    // User button
    //#define PB_USER_BUTTON PH_0 // Do not use: PH_0 is the HSE oscillator input
    #define PB_MECH_BUTTON PB_10
   
#endif

#endif /* PES_BOARD_PIN_MAP_ */
