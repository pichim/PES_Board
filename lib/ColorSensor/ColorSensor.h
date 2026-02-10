/**
 * @file ColorSensor.h
 * @brief This file defines the ColorSensor class.
 * @author C. Huber / hurc / hurcIMS
 */

#ifndef COLOR_SENSOR_H_
#define COLOR_SENSOR_H_

#include "mbed.h"

#include "ThreadFlag.h"
#include "AvgFilter.h"
#include "PwmIn.h"

// Default pin mapping for the board/target.
// If your wiring differs, use the 6-pin constructor to override these.
#define COLOR_SENSOR_LED PB_4
#define COLOR_SENSOR_S0 PA_4
#define COLOR_SENSOR_S1 PB_0
#define COLOR_SENSOR_S2 PC_0
#define COLOR_SENSOR_S3 PC_1

#define COLOR_DEBUG 0

#define N_FILTER 5

/**
 * @brief Calibration/reference values for each optical channel.
 *
 * Units are sensor output frequency (Hz) measured with the same:
 * - output scaling (S0/S1)
 * - LED state
 * - distance/optics
 */
typedef struct {
    float red;
    float green;
    float blue;
    float white;
} colorcalibration_t;


/**
 * @brief Photodiode filter selection.
 *
 * Numeric values intentionally encode the sensor select bits:
 * - bit0 -> S3
 * - bit1 -> S2
 *
 * This matches typical TCS3200/TCS230-style sensors.
 */
typedef enum {RED = 0, BLUE = 1, WHITE = 2, GREEN = 3} colorfilter_t;
/**
 * @brief Output frequency scaling selection.
 *
 * Numeric values intentionally encode the sensor select bits:
 * - bit0 -> S1
 * - bit1 -> S0
 */
typedef enum {FREQ_OFF = 0, FREQ_002, FREQ_20, FREQ_100} frequency_t;

/** @brief LED control (if present / wired). */
typedef enum {OFF = 0, ON} ledstate_t;


/**
 * @brief Frequency-output color sensor driver.
 *
 * The sensor is read periodically by a worker thread that is woken by a Ticker.
 * Public read*() accessors return pointers to internal arrays updated by that thread.
 * If you need a coherent snapshot, copy the arrays quickly (or add a mutex).
 */
class ColorSensor
{
public:
    /**
     * @brief Construct using default board pin mapping for LED/S0..S3.
     * @param pin Frequency output pin from the sensor.
     */
    explicit ColorSensor(PinName pin);
    /**
     * @brief Construct with explicit wiring.
     * @param pin Frequency output pin from the sensor.
     * @param led LED enable pin.
     * @param s0 Frequency scaling select pin (S0).
     * @param s1 Frequency scaling select pin (S1).
     * @param s2 Color filter select pin (S2).
     * @param s3 Color filter select pin (S3).
     */
    explicit ColorSensor(PinName pin, PinName led, PinName s0, PinName s1, PinName s2, PinName s3);
    /**
     * @brief Destructor.
     *
     * Detaches the ticker and terminates the worker thread (fast shutdown).
     * Note: terminate() is abrupt; keep this in mind if you later add shared resources.
     */
    virtual ~ColorSensor();

    /**
     * @brief Reset internal moving-average filters to the current raw readings.
     *
     * Useful after startup or after changing scaling/LED to avoid long settling.
     */
    void reset();

    /** @return Raw (unfiltered) channel frequencies in Hz: [R,G,B,Clear]. */
    const float* readRawColor() const { return m_color_Hz; }
    /** @return Filtered (moving-average) channel frequencies in Hz: [R,G,B,Clear]. */
    const float* readColor() const { return m_color_Avg_Hz; }
    /** @return Calibrated chromatic ratios (white-balanced), unitless. */
    const float* readColorCalib() const { return m_color_cal; }
    /** @return Normalized RGB (max(R,G,B) = 1), unitless. */
    const float* readColorNorm() const { return m_color_norm; }

    int getColor();
    static const char* getColorString(int color);
    
    /** @brief Select which photodiode filter is routed to the output (S2/S3). */
    void setColor(colorfilter_t color);
    /** @brief Select output frequency scaling (S0/S1). */
    void setFrequency(frequency_t frequency);
    /**
     * @brief Load hard-coded reference calibration values.
     *
     * These references are installation-specific (sensor, LED, distance, surface).
     * Update them if you change any of those conditions.
     */
    void setCalibration();
    /** @brief Turn the illumination LED on/off (if wired). */
    void switchLed(ledstate_t led_state);

private:
    // Thread wake-up period in microseconds (100,000us = 100ms ~ 10Hz update).
    static constexpr int64_t PERIOD_MUS = 150000;

    PwmIn m_PwmIn;
    DigitalOut m_S0;
    DigitalOut m_S1;
    DigitalOut m_S2;
    DigitalOut m_S3;

    DigitalOut m_Led;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    bool m_is_calibrated{false};
    colorfilter_t m_color[4] = {RED, GREEN, BLUE, WHITE};
    colorcalibration_t m_reference_color;
    colorcalibration_t m_reference_black;
    colorcalibration_t m_reference_white;
    float m_calib_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float m_calib_black[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float m_calib_white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float m_ratio[4]       = {1.0f, 1.0f, 1.0f, 1.0f};

    float m_color_Hz[4]     = {0.0f, 0.0f, 0.0f, 0.0f};
    float m_color_Avg_Hz[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float m_color_cal[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
    float m_color_norm[4]   = {0.0f, 0.0f, 0.0f, 0.0f};


    AvgFilter m_AvgFilter[4];
    bool m_is_first_run[4]{true};

    
    /**
     * @brief Convert raw channel frequencies into calibrated / normalized RGB.
     * @param color_Hz Pointer to [R,G,B,Clear] channel frequencies in Hz.
     */
    void applyCalibration(const float *color_Hz);

    void threadTask();
    void sendThreadFlag();
};
#endif /* COLOR_SENSOR_H_ */
