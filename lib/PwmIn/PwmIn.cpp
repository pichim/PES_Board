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

#include "PwmIn.h"

PwmIn::PwmIn(PinName p) : _p(p) {
    _p.rise(callback(this, &PwmIn::rise));
    _p.fall(callback(this, &PwmIn::fall));
    _period = 0.0;
    _pulsewidth = 0.0;
    _t.start();
}

float PwmIn::period() {
    return _period;
}

float PwmIn::pulsewidth() {
    return _pulsewidth;
}

float PwmIn::dutycycle() {
    return _pulsewidth / _period;
}

void PwmIn::rise() {
    _period = _t.read();
    _t.reset();
}

void PwmIn::fall() {
    _pulsewidth = _t.read();
}
