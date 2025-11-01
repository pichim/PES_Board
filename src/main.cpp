#include "mbed.h"

#include "RCRCThread.h"

// main thread is just blinking the led on the nucleo
int main()
{
    // spawn a new thread for the real-time task
    RCRCThread rcrcThread;

    printf("RCRC Thread started.\r\n");

    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}
