/**
 * @file IIRFilter.h
 * @brief This file defines the IIRFilter class.
 * @author M. Peter / pmic / pichim
 */

#ifndef IIR_FILTER_H_
#define IIR_FILTER_H_

class IIRFilter {
public:
    explicit IIRFilter() {};
    virtual ~IIRFilter() = default;

    void integratorInit(const float Ts);
    void integratorUpdate(const float Ts);

    void differentiatorInit(const float Ts);
    void differentiatorUpdate(const float Ts);

    void lowPass1Init(const float fcut, const float Ts);
    void lowPass1Update(const float fcut, const float Ts);

    void differentiatingLowPass1Init(const float fcut, const float Ts);
    void differentiatingLowPass1Update(const float fcut, const float Ts);

    void leadLag1Init(const float fZero, const float fPole, const float Ts);
    void leadLag1Update(const float fZero, const float fPole, const float Ts);
    void phaseComp1Init(const float fCenter, const float phaseLift, const float Ts);
    void phaseComp1Update(const float fCenter, const float phaseLift, const float Ts);

    void notchInit(const float fcut, const float D, const float Ts);
    void notchUpdate(const float fcut, const float D, const float Ts);

    void lowPass2Init(const float fcut, const float D, const float Ts);
    void lowPass2Update(const float fcut, const float D, const float Ts);

    void leadLag2Init(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts);
    void leadLag2Update(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts);

    void reset(const float output);
    void resetDifferentingFilterToZero(const float output);
    float apply(const float input);
    float applyConstrained(const float input, const float yMin, const float yMax);

private:
    struct IIRFilterParams{
        unsigned order;
        float A[2]; // [a1, a2], a0 is always 1.0
        float B[3]; // [b0, b1, b2]
        float w[2]; // [w1, w2]
    } filter;

    void applyFilterUpdate(const float input, const float output);
};

#endif /* IIR_FILTER_H_ */
