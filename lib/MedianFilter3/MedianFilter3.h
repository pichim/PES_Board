/**
 * @file MedianFilter3.h
 * @brief This file defines the MedianFilter3 class.
 * @author M. Peter / pmic / pichim
 */

#ifndef MEDIAN_FILTER_3_H_
#define MEDIAN_FILTER_3_H_

#include "mbed.h"

/**
 * Median filter for exactly 3 samples.
 */
class MedianFilter3
{
public:
    // constructors
    explicit MedianFilter3();
    virtual ~MedianFilter3() = default;

    // optionally call this if you want to re-init the filter later.
    void init();

    // resets the filter to 'val' (all 3 samples become 'val').
    float reset(float val);

    // resets the filter to 0.0f.
    float reset();

    // feed one new sample 'inp'. Returns the median of the last 3 samples.
    float apply(float inp);

    // returns the current median from the last apply() call.
    float read() const { return m_val; }

private:
    float m_window[3]; // ring buffer storage
    uint8_t m_idx;     // current index in the ring buffer [0..2]
    float m_val;       // the latest computed median
};

#endif /* MEDIAN_FILTER_3_H_ */
