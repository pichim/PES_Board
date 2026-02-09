/* mbed PwmIn Library
 * Copyright (c) 2008-2010, sford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_PWMIN_H
#define MBED_PWMIN_H

#include "mbed.h"

/** PwmIn class to read PWM inputs
 * 
 * Uses InterruptIn to measure the changes on the input
 * and record the time they occur
 *
 * @note uses InterruptIn, so not available on p19/p20
 */
class PwmIn {
public:
    /** Create a PwmIn
     *
     * @param p The pwm input pin (must support InterruptIn)
     */ 
    PwmIn(PinName p);
    
    /** Read the current period
     *
     * @returns the period in seconds
     */
    float period();
    
    /** Read the current pulsewidth
     *
     * @returns the pulsewidth in seconds
     */
    float pulsewidth();
    
    /** Read the current dutycycle
     *
     * @returns the dutycycle as a percentage, represented between 0.0-1.0
     */
    float dutycycle();

protected:        
    void rise();
    void fall();
    
    InterruptIn _p;
    Timer _t;
    float _pulsewidth, _period;
};

#endif
