/**
 * @file AvgFilter.h
 * @brief This file defines the Average Filter class.
 * @author M. Peter / pmic / pichim
 */

#ifndef AVG_FILTER_H_
#define AVG_FILTER_H_

#include "mbed.h"

/**
 * Average filter class.
 */
class AvgFilter
{
public:
    // default constructor (does nothing by itself)
    explicit AvgFilter() {};

    // constructor that initializes the filter with N samples
    explicit AvgFilter(uint8_t N);

    // destructor
    virtual ~AvgFilter();

    // initializes/allocates the ring buffer for N samples
    void init(uint8_t N);

    // resets the filter to 'val' (all samples become 'val/N')
    float reset(float val);

    // resets the filter to 0.0f
    float reset();

    // applies the filter to a new input 'inp' and returns the updated average
    float apply(float inp);

    // returns the current average
    float read() const { return m_val; }

private:
    float   m_val;         // rolling average (actually the sum of scaled samples)
    uint8_t m_N;           // number of samples in the filter
    uint8_t m_idx;         // current index for the ring buffer
    float*  m_ring_buffer; // dynamically allocated array storing scaled samples
};

#endif /* AVG_FILTER_H_ */
