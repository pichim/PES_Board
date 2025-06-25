#ifndef PES_BOARD_PIN_MAP_
#define PES_BOARD_PIN_MAP_

// PES-Board Pin Names
#define NEW_PES_BOARD_VERSION
#ifdef NEW_PES_BOARD_VERSION

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

    // Unused UART3
    #define PB_UNUSED_UART_TX PB_10
    #define PB_UNUSED_UART_RX PC_5

#else

    #define PB_D0 PC_9  // ???
    #define PB_D1 PC_8  // ???
    #define PB_D2 PC_6  // ???
    #define PB_D3 PB_12 // ???

    #define PB_PWM_M1 PA_8
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PA_13 // ???

    #define PB_ENC_A_M1 PB_6
    #define PB_ENC_B_M1 PB_7
    #define PB_ENC_A_M2 PA_6
    #define PB_ENC_A_M2 PC_7
    #define PB_ENC_A_M3 PA_1 // ???
    #define PB_ENC_B_M3 PA_0 // ???

    #define PB_ENABLE_DCMOTORS PB_2 // PB_13 ???

    #define PB_IMU_SDA // ???
    #define PB_IMU_SCL // ???
    
#endif

#endif /* PES_BOARD_PIN_MAP_ */
