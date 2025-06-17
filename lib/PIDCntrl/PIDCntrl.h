/**
 * @file PIDCntrl.h
 * @brief This file defines the PIDCntrl class.
 * @author M. Peter / pmic / pichim
 */

#ifndef PID_CNTRL_H_
#define PID_CNTRL_H_

#include <math.h>

#ifndef M_PI
    #define M_PI 3.141592653589793238462643383279502884 // pi
#endif

class PIDCntrl
{
public:
    explicit PIDCntrl(float I, float Ts, float uMin, float uMax);
    explicit PIDCntrl(float P, float I, float Ts, float uMin, float uMax);
    explicit PIDCntrl(float P, float I, float D, float Ts, float uMin, float uMax);
    explicit PIDCntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    explicit PIDCntrl(float P, float I, float D, float tau_f, float tau_ro, float Ts, float uMin, float uMax);

    explicit PIDCntrl(){};

    virtual ~PIDCntrl() = default;

    void reset(float initValue = 0.0f);

    void setup(float I, float Ts, float uMin, float uMax);
    void setup(float P, float I, float Ts, float uMin, float uMax);
    void setup(float P, float I, float D, float Ts, float uMin, float uMax);
    void setup(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax);
    void setup(float P, float I, float D, float tau_f, float tau_ro, float Ts, float uMin, float uMax);

    void setParamP(float P);
    void setParamI(float I);
    void setParamD(float D);
    void setParamF(float F);

    void scalePIDT2params(float scale);

    float update(float e);
    float update(float e, float y);
    float update(float w, float y_p, float y_i, float y_d);

    void setLimits(float uMin, float uMax);
    void setIntegratorLimits(float uIMin, float uIMax);

    float prewarp(float T, float Ts);

    float get_ulimit();
    float get_P_gain();
    float get_bd();
    float get_ad();
    float getCurrentOutput();

private:
    float IPart, Dpart, d_old, u_old, uf;
    float P, I, D, tau_f, tau_ro, Ts, uMin, uMax, uIMin, uIMax;
    float bi, bd, ad, bf, af;
    float P_init, I_init, D_init;
    float F{0.0f};

    void setCoefficients(float P, float I, float D, float tau_f, float tau_ro, float Ts);

    void updateParamI(float I, float Ts);
    void updateParamD(float D, float Ts, float tau_f);
    void updateParamRO(float Ts, float tau_ro);

    float saturate(float u, float uMin, float uMax);
};

#endif /* PID_CNTRL_H_ */
