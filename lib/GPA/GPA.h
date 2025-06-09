/**
 * @file GPA.h
 * @brief This file defines the GPA class.
 * @author M. Peter / pmic / pichim
 */

#ifndef GPA_H_
#define GPA_H_

#include "mbed.h"

#include "math.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f // pi
#endif

#ifndef M_PI
    #define M_PI 3.141592653589793238462643383279502884 // pi
#endif

#define GPA_EXC_VIA_FILTER false
#define BUFFER_LENGTH 120

using namespace std;

class GPA
{
public:

    typedef struct gpadata_s {
        float fexc;
        float Ureal;
        float Uimag;
        float Yreal;
        float Yimag;
        float Rreal;
        float Rimag;
        bool  MeasPointFinished;
        bool  MeasFinished;
        int   ind;
    } gpadata_t;

    GPA();
    GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts);
    GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep);
    GPA(float f0, float f1, float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep);
    GPA(float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep);
    GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep, bool doPrint, bool doPrecalcParam);

    float operator()(float inp, float out)
    {
        return update(inp, out);
    }

    ~GPA() = default;

    void    init(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep, bool doPrint, bool doPrecalcParam);
    void    reset();
    float   update(float inp, float out);

    void    printGPAfexcDes();
    void    printGPAmeasPara();
    void    printGPAmeasTime();
    void    printNfexcDes();
    void    printPrecalcParam();

    gpadata_t getGPAdata();

private:
    BufferedSerial m_BufferedSerial;
    char m_buffer[BUFFER_LENGTH];
    void setUpBufferedSerial();
    uint16_t m_print_cntr;

    int     NfexcDes;
    int     NperMin;
    int     NmeasMin;
    float   Ts;
    float  *fexcDes;
    float   aAexcDes;
    float   bAexcDes;

    float   fnyq;
    double  pi2;
    float   pi4;
    double  pi2Ts;
    float   piDiv2;
    float   rad2deg;
    float   div12pi;
    float   div812pi;

    int     Nmeas;
    int     Nper;
    double  dfexc;
    double  fexc;
    float   fexcPast;
    float   dfexcj;
    int     i;
    int     j;
    double  scaleG;
    double  cr;
    double  ci;
    double *sU;
    double *sY;
#if GPA_EXC_VIA_FILTER
    double *sR;
#endif
    float   exc;
    double  sinarg;
    float   sinargR;
    int     NmeasTotal;
    float   Aexc;
    float   AexcPast;
    double  pi2Tsfexc;
    int     Nstart;
    int     Nsweep;
    int     Nsweep_i;
    float   AexcOut;

    /* storage to precalculate parameters */
    int    *Nper_vec;
    int    *Nmeas_vec;
    double *fexc_vec;
    float  *Aexc_vec;
    bool    doPrecalcParam;
    int     NfexcAct;

    gpadata_t gpaData;
    bool    doPrint;

    void    assignParameters(int NfexcDes, int NperMin, int NmeasMin, float Ts, int Nstart, int Nsweep);
    void    calculateDecreasingAmplitudeCoefficients(float Aexc0, float Aexc1);
    void    initializeConstants(float Ts);
    void    assignFilterStorage();
    void    assignAndResetParamStorage();
    void    fexcDesLogspace(float fMin, float fMax, int NfexcDes);
    void    calcGPAmeasPara(float fexcDes_i);
    void    precalcParam();
    float   wrapAngle(float angle);

    // Timer timer;

};

#endif
