#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(PinName pin)
    : m_DigitalInOut(pin),
      m_InteruptIn(pin),
      m_Thread(osPriorityAboveNormal)
{
    m_Timer.start();

    // start thread
    m_Thread.start(callback(this, &UltrasonicSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &UltrasonicSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

UltrasonicSensor::~UltrasonicSensor()
{
    m_Timeout.detach();
    m_Ticker.detach();
    m_Thread.terminate();
}

float UltrasonicSensor::read()
{
    if (m_is_new_value) {
        m_is_new_value = false;
        return m_us_distance_cm;
    } else {
        return -1.0f;
    }
}

void UltrasonicSensor::stopPulseAndWaitForRisingEdge()
{
    // set the digital output to low and change the pin to input mode
    m_DigitalInOut = 0;
    m_DigitalInOut.input();

    // enable interrupt and attach functions to rising and falling edge
    m_InteruptIn.enable_irq();
    m_InteruptIn.rise(callback(this, &UltrasonicSensor::startTimerAndWaitForFallingEdge));
}

void UltrasonicSensor::startTimerAndWaitForFallingEdge()
{
    // start timer and attach function to falling edge
    m_Timer.reset();
    m_InteruptIn.fall(callback(this, &UltrasonicSensor::measureTimeAndUpdateDistance));
}

void UltrasonicSensor::measureTimeAndUpdateDistance()
{
    // measure time and update distance
    const int pulse_time = std::chrono::duration_cast<std::chrono::microseconds>(m_Timer.elapsed_time()).count();
    // m_us_distance_cm = static_cast<float>(pulse_time);
    m_us_distance_cm = m_gain * static_cast<float>(pulse_time) + m_offset;
    m_is_new_value = true;
}

void UltrasonicSensor::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        // for a successful measurement the ultrasonic sensor needs to respond:
        // the ultrasonic sensor needs to send a pulse with a rising edge followed by a falling edge
        // the time length between the rising and falling edge is proportional to the distance
        // 1. threadTask() is called periodically by the ticker triggers a measurement via pulse
        // 2. stopPulseAndWaitForRisingEdge()
        // 3. startTimerAndWaitForFallingEdge()
        // 4. measureTimeAndUpdateDistance()

        // detach interrupt
        m_InteruptIn.disable_irq();
        m_InteruptIn.rise(NULL);
        m_InteruptIn.fall(NULL);

        // change the pin to output mode and set the digital output to high
        // this will generate a pulse of length m_pulsetime and trigger the ultrasonic sensor
        // to perform a measurement
        m_DigitalInOut.output();
        m_DigitalInOut = 1;
        m_Timeout.attach(callback(this, &UltrasonicSensor::stopPulseAndWaitForRisingEdge), std::chrono::microseconds{10});
    }
}

void UltrasonicSensor::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
