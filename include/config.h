#ifndef CONFIG_H_
#define CONFIG_H_

// define the target board (only use one at a time)
#define USE_NUCLEO_L432KC
// #define USE_NUCLEO_F446RE
// #define USE_NUCLEO_H743ZI2

#if defined(USE_NUCLEO_L432KC)

    // task period
    // #define RTT_PERIOD_US 2500   //   400 Hz
    // #define RTT_PERIOD_US 2000   //   500 Hz
    // #define RTT_PERIOD_US 1000   //  1000 Hz
    #define RTT_PERIOD_US 500    //  2000 Hz
    // #define RTT_PERIOD_US 333    //  3000 Hz
    // #define RTT_PERIOD_US 200    //  5000 Hz
    // #define RTT_PERIOD_US 150    //  6666 Hz
    // #define RTT_PERIOD_US 133    //  7500 Hz
    // #define RTT_PERIOD_US 100    // 10000 Hz
    // #define RTT_PERIOD_US  50    // 20000 Hz

    // streaming device, openlager or laptop / pc
    // #define RTT_TX USBTX // usb to computer
    // #define RTT_RX USBRX
    #define RTT_TX PB_6 // usb 2.0-cable TTL serial 6 pin to computer or openlager
    #define RTT_RX PB_7
    #define RTT_BAUDRATE 2000000
    #define RTT_NUM_OF_FLOATS 30

    // additonal button
    #define RTT_BUTTON PB_0

    // additional led
    #define RTT_LED PA_12

    // analog inputs
    #define RTT_AIN1 PA_0
    #define RTT_AIN2 PA_1

    // analog output
    #define RTT_AOUT1 PA_4

#elif defined(USE_NUCLEO_F446RE)

    // task period
    // #define RTT_PERIOD_US 2500   //   400 Hz
    // #define RTT_PERIOD_US 2000   //   500 Hz
    // #define RTT_PERIOD_US 1000   //  1000 Hz
    // #define RTT_PERIOD_US 500    //  2000 Hz
    // #define RTT_PERIOD_US 333    //  3000 Hz
    // #define RTT_PERIOD_US 200    //  5000 Hz
    // #define RTT_PERIOD_US 150    //  6666 Hz
    #define RTT_PERIOD_US 133    //  7500 Hz
    // #define RTT_PERIOD_US 100    // 10000 Hz
    // #define RTT_PERIOD_US  50    // 20000 Hz

    // streaming device, openlager or laptop / pc
    #define RTT_TX USBTX // usb to computer
    #define RTT_RX USBRX
    // #define RTT_TX PA_0 // usb 2.0-cable TTL serial 6 pin to computer or openlager
    // #define RTT_RX PA_1
    #define RTT_BAUDRATE 2000000
    #define RTT_NUM_OF_FLOATS 30

    // additonal button
    #define RTT_BUTTON BUTTON1 // blue button

    // additional led
    #define RTT_LED PB_9

    // analog inputs
    #define RTT_AIN1 PB_0
    #define RTT_AIN2 PC_1

    // analog output
    #define RTT_AOUT1 PA_4

#elif defined(USE_NUCLEO_H743ZI2)

    // task period
    // #define RTT_PERIOD_US 2500   //   400 Hz
    // #define RTT_PERIOD_US 2000   //   500 Hz
    // #define RTT_PERIOD_US 1000   //  1000 Hz
    // #define RTT_PERIOD_US 500    //  2000 Hz
    // #define RTT_PERIOD_US 333    //  3000 Hz
    // #define RTT_PERIOD_US 200    //  5000 Hz
    #define RTT_PERIOD_US 150    //  6666 Hz
    // #define RTT_PERIOD_US 133    //  7500 Hz
    // #define RTT_PERIOD_US 100    // 10000 Hz
    // #define RTT_PERIOD_US  50    // 20000 Hz

    // streaming device, openlager or laptop / pc
    #define RTT_TX USBTX // usb to computer
    #define RTT_RX USBRX
    // #define RTT_TX PA_0 // usb 2.0-cable TTL serial 6 pin to computer or openlager
    // #define RTT_RX PA_1
    #define RTT_BAUDRATE 2000000
    #define RTT_NUM_OF_FLOATS 30

    // additonal button
    #define RTT_BUTTON BUTTON1 // blue button

    // additional led
    #define RTT_LED PB_9

    // analog inputs
    #define RTT_AIN1 PF_4
    #define RTT_AIN2 PF_5

    // analog output
    #define RTT_AOUT1 PA_4

#endif

#endif /* CONFIG_H_ */
