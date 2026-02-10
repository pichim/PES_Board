#include "ColorSensor.h"
#include <cmath>

// Implementation notes:
// - The sensor produces a square-wave output whose frequency is proportional to light intensity.
// - S2/S3 select the photodiode filter (R/G/B/Clear) and S0/S1 select output scaling.
// - A Ticker periodically wakes a worker thread that sequences through the filters and updates
//   m_color_* arrays.

/**
 * @brief Construct using default pin mapping for LED/S0..S3.
 *
 * Sets a default filter and scaling, loads calibration references, then starts the worker thread.
 */
ColorSensor::ColorSensor(PinName pin) : m_PwmIn(pin),
                                        m_Led(COLOR_SENSOR_LED, 0),
                                        m_S0(COLOR_SENSOR_S0, 0),
                                        m_S1(COLOR_SENSOR_S1, 0),
                                        m_S2(COLOR_SENSOR_S2, 0),
                                        m_S3(COLOR_SENSOR_S3, 0),
                                        m_Thread(osPriorityLow),
                                        m_AvgFilter{AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER)}
{
    setColor(WHITE);
    setFrequency(FREQ_002);
    setCalibration();

    // start thread
    m_Thread.start(callback(this, &ColorSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ColorSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

/**
 * @brief Construct with explicit wiring.
 *
 * Uses a shorter averaging window by default (faster response, less smoothing).
 */
ColorSensor::ColorSensor(PinName pin, PinName led, PinName s0, PinName s1, PinName s2, PinName s3) : m_PwmIn(pin),
                                                                                        m_Led(led, 0),
                                                                                        m_S0(s0, 0),
                                                                                        m_S1(s1, 0),
                                                                                        m_S2(s2, 0),
                                                                                        m_S3(s3, 0),
                                                                                        m_Thread(osPriorityLow),
                                                                                        m_AvgFilter{AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER), AvgFilter(N_FILTER)}
{
    setColor(WHITE);
    setFrequency(FREQ_002);
    setCalibration();

    // start thread
    m_Thread.start(callback(this, &ColorSensor::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &ColorSensor::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

/**
 * @brief Destructor.
 *
 * Important ordering:
 * 1) Detach the ticker first so no further wakeups occur.
 * 2) Terminate the worker thread.
 *
 * Note: terminate() is an abrupt stop. It's acceptable here because the thread only touches
 * this object, but if you later add shared resources (I2C, heap allocations, mutexes), consider
 * a graceful shutdown.
 */
ColorSensor::~ColorSensor()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

/**
 * @brief Reset the moving-average filters to the current raw readings.
 *
 * This forces the filtered output to match the latest raw sample immediately.
 */
void ColorSensor::reset()
{
    for (int i = 0; i < 4; i++) {
        m_AvgFilter[i].reset(m_color_Hz[i]);
        m_is_first_run[i] = true;
    }
}

/**
 * @brief Load hard-coded calibration references.
 *
 * The reference values below are frequency measurements (Hz) for:
 * - an example "color" surface (m_reference_color) [currently unused in math]
 * - a "black" surface (offset / dark level)
 * - a "white" surface (normalization / white balance)
 *
 * These numbers depend on:
 * - sensor module tolerance
 * - selected scaling (S0/S1)
 * - LED state and brightness
 * - distance/angle and ambient light
 *
 * If you change any of those, remeasure and update the constants.
 */
void ColorSensor::setCalibration()
{
    m_reference_color.red = 265.5f;
    m_reference_color.green = 192.0f;
    m_reference_color.blue = 234.0f;
    m_reference_color.white = 1220.0f;

    m_reference_black.red = 70.0f;
    m_reference_black.green = 74.0f;
    m_reference_black.blue = 91.0f;
    m_reference_black.white = 228.0f;

    m_reference_white.red = 362.0f;
    m_reference_white.green = 397.0f;
    m_reference_white.blue = 491.0f;
    m_reference_white.white = 1220.0f;

    m_calib_color[0] = m_reference_color.red;
    m_calib_color[1] = m_reference_color.green;
    m_calib_color[2] = m_reference_color.blue;
    m_calib_color[3] = m_reference_color.white;

    m_calib_black[0] = m_reference_black.red;
    m_calib_black[1] = m_reference_black.green;
    m_calib_black[2] = m_reference_black.blue;
    m_calib_black[3] = m_reference_black.white;

    m_calib_white[0] = m_reference_white.red;
    m_calib_white[1] = m_reference_white.green;
    m_calib_white[2] = m_reference_white.blue;
    m_calib_white[3] = m_reference_white.white;

    const float eps = 1e-3f;
    float denom = (m_calib_white[3] - m_calib_black[3]);
    if (denom < eps) {
        // invalid calibration
        return;
    }

    for (int i = 0; i < 3; i++) {
        m_ratio[i] = (m_calib_white[i] - m_calib_black[i]) / denom;
        if (m_ratio[i] < eps) m_ratio[i] = eps; // avoid divide by zero later
    }
    
}

/**
 * @brief Select the photodiode filter via S2/S3.
 *
 * colorfilter_t values encode the required bit pattern (see header).
 */
void ColorSensor::setColor(colorfilter_t color)
{
    m_S3 = (color >> 0) & 1; // LSB
    m_S2 = (color >> 1) & 1; // MSB
}

/**
 * @brief Select output scaling via S0/S1.
 *
 * frequency_t values encode the required bit pattern (see header).
 */
void ColorSensor::setFrequency(frequency_t frequency)
{
    m_S1 = (frequency >> 0) & 1; // LSB
    m_S0 = (frequency >> 1) & 1; // MSB
}


/**
 * @brief Turn illumination LED on/off.
 */
void ColorSensor::switchLed(ledstate_t led_state)
{
    m_Led = led_state;
}

/**
 * @brief Worker thread: periodically samples all 4 channels.
 *
 * Woken by a Ticker via sendThreadFlag(). For each channel, we select the filter,
 * wait briefly for the sensor output to settle, then measure the output period.
 */
void ColorSensor::threadTask()
{
    while (true) {
        // Block until the ticker signals us.
        ThisThread::flags_wait_any(m_ThreadFlag);

        // Sequence through channels in the order defined by m_color[].
        for (int i = 0; i < 4; i++)
        {
            setColor(m_color[i]);
            // Allow filter switching and output frequency to stabilize.
            ThisThread::sleep_for(20ms);

            // PwmIn::period() returns the measured signal period (typically in seconds).
            float period = m_PwmIn.period();

            // catch division by zero
            if (period > 1e-6f) {
                m_color_Hz[i] = 1.0f/period;

                // average filtered color
                if (m_is_first_run[i]) {
                    m_is_first_run[i] = false;
                    m_color_Avg_Hz[i] = m_AvgFilter[i].reset(m_color_Hz[i]);
                } else {
                    m_color_Avg_Hz[i] = m_AvgFilter[i].apply(m_color_Hz[i]);
                }
            }
        }    
        applyCalibration(m_color_Avg_Hz);    
    }
}

/**
 * @brief Apply black/white calibration and compute normalized RGB.
 *
 * Steps:
 * 1) Subtract black reference (dark offset).
 * 2) Convert RGB to chromatic ratios by dividing by Clear.
 * 3) White-balance by dividing by precomputed m_ratio[] (derived from white reference).
 * 4) Normalize RGB so max(R,G,B)=1 (useful for classification/thresholding).
 */
void ColorSensor::applyCalibration(const float *color)
{
    float color0[4];

    // Subtract black reference and clamp
    for (int i = 0; i < 4; i++) {
        color0[i] = color[i] - m_calib_black[i];
        color0[i] = fmax(color0[i], 0.0f);
    }

    const float eps = 1e-3f;
    
    // Clear channel after black subtraction is the denominator
    if (color0[3] < eps) {
        return; // too dark / invalid
    }

    m_color_cal[3] = color0[3];
    
    // Chromatic ratios (relative to clear)
    float ratio[3];
    for (int i = 0; i < 3; i++) {
        ratio[i] = color0[i] / color0[3];
    }

    // White-balance
    for (int i = 0; i < 3; i++) {
        if (m_ratio[i] > eps) {
            m_color_cal[i] = ratio[i] / m_ratio[i];
        }
    }

    // Normalize RGB
    const float max_rgb = fmax(m_color_cal[0], fmax(m_color_cal[1], m_color_cal[2]));
    if (max_rgb < eps) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        m_color_norm[i] = m_color_cal[i] / max_rgb;
    }

}

int ColorSensor::getColor()
{
    // Brightness after black subtraction (Hz)
    const float c0 = std::max(0.0f, m_color_cal[3]);

    // ---- Tunables ----
    const float C0_BLACK_MAX = 80.0f;     // below => BLACK
    const float C0_WHITE_MIN = 650.0f;    // above + neutral => WHITE

    const float SATP_GRAY_MAX  = 0.08f;   // below => neutral
    const float SATP_COLOR_MIN = 0.12f;   // below => treat as unknown/neutral

    // Hue boundaries
    const float H_RED_MAX_1   = 20.0f;
    const float H_YELLOW_MAX  = 65.0f;
    const float H_GREEN_MAX   = 170.0f;
    const float H_CYAN_MAX    = 200.0f;
    const float H_BLUE_MAX    = 240.0f;
    const float H_MAGENTA_MIN = 345.0f;

    // Magenta override thresholds (tuned from your logs)
    const float MAG_RG_MIN = 2.5f;  // magenta r0/g0 ~4.5, blue r0/g0 ~0.5
    const float MAG_BG_MIN = 1.2f;  // ensure blue is present (magenta b0/g0 ~1.9)
    const float MAG_RB_MAX = 3.8f;  // keep pure red out (red r0/b0 ~5.6, magenta ~2.4)

    const int STABLE_COUNT = 3;

    // ---- Use calibrated (white-balanced) channels ----
    float r0 = std::max(0.0f, m_color_cal[0]);
    float g0 = std::max(0.0f, m_color_cal[1]);
    float b0 = std::max(0.0f, m_color_cal[2]);

    const float eps = 1e-6f;

    const float sum = r0 + g0 + b0;
    const float mx0 = std::max(r0, std::max(g0, b0));
    const float mn0 = std::min(r0, std::min(g0, b0));
    const float satp = (mx0 - mn0) / std::max(sum, eps);

    int candidate = 0;

    // ---- Brightness gates ----
    if (c0 < C0_BLACK_MAX) {
        candidate = 1; // BLACK
    }
    else if (satp < SATP_GRAY_MAX && c0 > C0_WHITE_MIN) {
        candidate = 2; // WHITE
    }
    else if (satp < SATP_COLOR_MIN) {
        candidate = 0; // UNKNOWN
    }
    else {
        // ---- MAGENTA OVERRIDE (before HSV hue) ----
        // Use ratios in calibrated space; these are stable and separate your clusters well.
        const float rg = r0 / std::max(g0, eps);
        const float bg = b0 / std::max(g0, eps);
        const float rb = r0 / std::max(b0, eps);

        // Magenta: strong red, meaningful blue, low green; but not "pure red" (rb too large).
        if (rg > MAG_RG_MIN && bg > MAG_BG_MIN && rb < MAG_RB_MAX) {
            candidate = 8; // MAGENTA
        } else {
            // Safe normalization for hue
            float r = r0 / std::max(mx0, eps);
            float g = g0 / std::max(mx0, eps);
            float b = b0 / std::max(mx0, eps);

            // HSV hue
            const float cmax = std::max(r, std::max(g, b));
            const float cmin = std::min(r, std::min(g, b));
            const float delta = cmax - cmin;

            float h = 0.0f;
            if (delta < eps) {
                candidate = 0;
            } else {
                if (cmax == r)      h = 60.0f * std::fmod(((g - b) / delta), 6.0f);
                else if (cmax == g) h = 60.0f * (((b - r) / delta) + 2.0f);
                else                h = 60.0f * (((r - g) / delta) + 4.0f);
                if (h < 0.0f) h += 360.0f;

                if (h <= H_RED_MAX_1 || h >= H_MAGENTA_MIN) candidate = 3; // RED
                else if (h <= H_YELLOW_MAX)                 candidate = 4; // YELLOW
                else if (h <= H_GREEN_MAX)                  candidate = 5; // GREEN
                else if (h <= H_CYAN_MAX)                   candidate = 6; // CYAN
                else if (h <= H_BLUE_MAX)                   candidate = 7; // BLUE
                else                                        candidate = 8; // MAGENTA
            }
        }

#if COLOR_DEBUG
        printf("c0=%.1f  r0=%.3f g0=%.3f b0=%.3f  satp=%.3f  out=%d\n",
               c0, r0, g0, b0, satp, candidate);
#endif
    }

    // ---- Hysteresis ----
    static int last = 0;
    static int cnt  = 0;

    if (candidate == last) cnt = std::min(cnt + 1, STABLE_COUNT);
    else { last = candidate; cnt = 1; }

    return (cnt >= STABLE_COUNT) ? candidate : last;
}

const char* ColorSensor::getColorString(int color)
{
    switch (color) {
        case 0: return "UNKNOWN";
        case 1: return "BLACK";
        case 2: return "WHITE";
        case 3: return "RED";
        case 4: return "YELLOW";
        case 5: return "GREEN";
        case 6: return "CYAN";
        case 7: return "BLUE";
        case 8: return "MAGENTA";
        default: return "INVALID";
    }
}

/**
 * @brief Wake the worker thread.
 *
 * Called by the ticker at PERIOD_MUS cadence.
 */
void ColorSensor::sendThreadFlag()
{
    m_Thread.flags_set(m_ThreadFlag);
}