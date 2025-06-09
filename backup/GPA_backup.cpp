/*
    GPA: Frequency point wise gain and phase analyser to measure a frequency respone function (FRF) of a dynamical system

         Hint:        If the plant has a pole at zero, is unstable or weakly damped the measurement has to be perfomed
                      in closed loop (this is NOT tfestimate, the algorithm is based on the one point DFT).
         Assumption:  The system is and remains at the desired steady state when the measurment starts

         Note:        The amplitude drops with 1/fexc, if you're using Axc1 = Aexc0/fMax then d/dt exc = const.,
                      this is recommended if your controller does not have a rolloff. If a desired frequency point
                      is not measured (could not be reached) try to increase Nmeas.


    Instantiate option 0: ("Not a Jedi yet" users, for logarithmic equaly spaced frequency points)

        GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts)

            fMin:       Minimal desired frequency that should be measured in Hz
            fMax:       Maximal desired frequency that should be measured in Hz
            NfexcDes:   Number of logarithmic equaly spaced frequency points between fMin and fMax
            Aexc0:      Excitation amplitude at fMin
            Aexc1:      Excitation amplitude at fMax
            Ts:         Sampling time in sec

            Default values are as follows:
            int NperMin  = 3;
            int NmeasMin = (int)ceil(1.0f/Ts);
            int Nstart   = (int)ceil(3.0f/Ts);
            int Nsweep   = (int)ceil(0.0f/Ts);

    Instantiate option 1: ("Jedi or Sith Lord", for logarithmic equaly spaced frequency points)

        GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep)

            fMin:       Minimal desired frequency that should be measured in Hz
            fMax:       Maximal desired frequency that should be measured in Hz
            NfexcDes:   Number of logarithmic equaly spaced frequency points
            NperMin:    Minimal number of periods that are used for each frequency point
            NmeasMin:   Minimal number of samples that are used for each frequency point
            Ts:         Sampling time in sec
            Aexc0:      Excitation amplitude at fMin
            Aexc1:      Excitation amplitude at fMax
            Nstart:     Minimal number of samples to sweep to the first frequency point (can be equal 0)
            Nsweep:     Minimal number of samples to sweep from freq. point to freq. point (can be equal 0)


    Instantiate option 2: (for a second, refined frequency grid measurement)

        GPA(float f0, float f1, float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep)

            f0:         Frequency point for the calculation of Aexc0 in Hz (should be chosen like in the first measurement)
            f1:         Frequency point for the calculation of Aexc1 in Hz (should be chosen like in the first measurement)
            *fexcDes:   Sorted frequency point array in Hz
            NfexcDes:   Length of fexcDes

            For the other parameters see above.

    Instantiate option 3: (for an arbitary but sorted frequency grid measurement)

        GPA(float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep)

            *fexcDes:   Sorted frequency point array in Hz
            Aexc0:      Excitation amplitude at fexcDes[0]
            Aexc1:      Excitation amplitude at fexcDes[NfexcDes-1]
            NfexcDes:   Length of fexcDes

            For the other parameters see above.


    Block diagram:

                w (const.)    exc(2)                C: controller
                |              |                    P: plant
                v   e          v
     exc(1) --> o   ->| C |--->o------->| P |----------> out (y)
                ^ -                |             |
                |                   --> inp (u)  |  exc (R): excitation signal
                |                                |  inp (U): input plant
                 --------------------------------   out (Y): output plant


    Pseudo code for an open loop measurement:

        - Measuring the plant P = Gyu = Gyr:

            u = w + exc;
            ... write output u here! it follows exc(k+1) ...
            exc = Wobble(exc, y);

            Closed loop FRF calculus with a stabilizing controller C:
                S  = 1/(1 + C*P);  % ( exc1 -> e ,   1/(1 + C*P) ) sensitivity, contr. error rejection, robustness (1/max|S|)
                T  = 1 - S;        % ( w -> y    , C*P/(1 + C*P) ) compl. sensitivity, tracking performance
                CS = C*S;          % ( exc1 -> u ,   C/(1 + C*P) ) control effort, disturbance plant output on plant input
                PS = P*S;          % ( exc2 -> y ,   P/(1 + C*P) ) compliance, disturbance plant input on plant output


    Pseudo code for a closed loop measurement with stabilizing controller C:

        Excitation at excitation input (1):

        - Measuring the plant P = Gyu and the closed loop tf T = PC/(1 + PC) = Gyr:

            u = C(w - y + exc);
            ... write output u here! it follows exc(k+1) ...
            exc = gpa(u, y);

            Closed loop FRF calculus:
                S  = 1 - T;
                PS = P*S;
                CS = T/P;
                C  = C/S;

        Excitation at excitation input (2):

        - Measuring the plant P = Gyu and the closed loop tf PS = P/(1 + PC) = Gyr:

            u = C(w - y) + exc;
            ... write output u here! it follows exc(k+1) ...
            exc = gpa(u, y);

            Closed loop FRF calculus:
                S  = PS/P;
                T  = 1 - S;
                CS = T/P;
                C  = C/S;

    Usage:
        exc(k+1) = gpa(inp(k), out(k)) does update the internal states of the
        gpa at the timestep k and returns the excitation signal for the timestep
        k+1. The FRF data are plotted to a terminal (Putty) over a serial
        connection and look as follows:

    In MATLAB you can use:
        U = data(:,2) + 1i*data(:,3);
        Y = data(:,4) + 1i*data(:,5);
        R = data(:,6) + 1i*data(:,7);
        P = frd(Y./U, data(:,1), Ts, 'Units', 'Hz');
        T = frd(Y./R, data(:,1), Ts, 'Units', 'Hz');
        S = 1 - T;
        C = T/S/P;
        SC = S*C;
        SP = S*P;

    If you're evaluating more than one measurement which contain equal frequency points use:
        data = [data1; data2];
        [~, ind] = unique(data(:,1), 'stable');
        data = data(ind,:);


    Autor and Copyrigth: 2018-2021 / M.E. Peter

*/

// -----------------------------------------------------------------------------
//      instantiate
// -----------------------------------------------------------------------------

#include "GPA.h"

GPA::GPA() : m_BufferedSerial(USBTX, USBRX) {
    setUpBufferedSerial();
}

void GPA::setUpBufferedSerial()
{
    m_BufferedSerial.set_baud(2000000);
    m_BufferedSerial.set_blocking(false);
}

GPA::GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts) : m_BufferedSerial(USBTX, USBRX)
{
    setUpBufferedSerial();
    doPrint = false;
    doPrecalcParam = true;
    int NperMin = 3;
    int NmeasMin = (int)ceil(1.0f/Ts);
    int Nstart = (int)ceil(3.0f/Ts);
    int Nsweep = (int)ceil(0.0f/Ts);
    init(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint, doPrecalcParam);
}

GPA::GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep) : m_BufferedSerial(USBTX, USBRX)
{
    setUpBufferedSerial();
    doPrint = false;
    doPrecalcParam = true;
    init(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint, doPrecalcParam);
}

GPA::GPA(float f0, float f1, float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep) : m_BufferedSerial(USBTX, USBRX)
{
    setUpBufferedSerial();
    doPrint = false;
    doPrecalcParam = true;
    
    // Validate input pointer
    if (fexcDes == nullptr) {
        printf("GPA: ERROR - Frequency array pointer is null\r\n");
        return;
    }
    
    // Validate using f0 and f1 as frequency range
    if (!validateParameters(f0, f1, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep)) {
        printf("GPA: ERROR - Invalid parameters in constructor\r\n");
        return;
    }
    
    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);// convert fexcDes from float to float, it is assumed that it is sorted
    this->fexcDes = (float*)malloc(NfexcDes*sizeof(float));
    if (this->fexcDes == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for fexcDes\r\n");
        return; // Early return on allocation failure
    }
    for(int i = 0; i < NfexcDes; i++) {
        this->fexcDes[i] = (float)fexcDes[i];
    }

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
    if(doPrecalcParam) {
        assignAndResetParamStorage();
        precalcParam();
    }
}

GPA::GPA(float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep) : m_BufferedSerial(USBTX, USBRX)
{
    setUpBufferedSerial();
    doPrint = false;
    doPrecalcParam = true;
    
    // Validate input pointer
    if (fexcDes == nullptr) {
        printf("GPA: ERROR - Frequency array pointer is null\r\n");
        return;
    }
    
    // Find min and max frequencies for validation
    float fMin = fexcDes[0];
    float fMax = fexcDes[0];
    for (int i = 1; i < NfexcDes; i++) {
        if (fexcDes[i] < fMin) fMin = fexcDes[i];
        if (fexcDes[i] > fMax) fMax = fexcDes[i];
    }
    
    if (!validateParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep)) {
        printf("GPA: ERROR - Invalid parameters in constructor\r\n");
        return;
    }
    
    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);// convert fexcDes from float to float, it is assumed that it is sorted
    this->fexcDes = (float*)malloc(NfexcDes*sizeof(float));
    if (this->fexcDes == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for fexcDes\r\n");
        return; // Early return on allocation failure
    }
    for(int i = 0; i < NfexcDes; i++) {
        this->fexcDes[i] = fexcDes[i];
    }

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
    if(doPrecalcParam) {
        assignAndResetParamStorage();
        precalcParam();
    }
}

GPA::GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep, bool doPrint, bool doPrecalcParam) : m_BufferedSerial(USBTX, USBRX)
{
    setUpBufferedSerial();
    init(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint, doPrecalcParam);
}

void GPA::init(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep, bool doPrint, bool doPrecalcParam)
{
    // Validate input parameters first
    if (!validateParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep)) {
        printf("GPA: ERROR - Invalid parameters, initialization failed\r\n");
        return;
    }
    
    this->doPrint = doPrint;
    this->doPrecalcParam = doPrecalcParam;
    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);// calculate logarithmic spaced frequency points
    fexcDes = (float*)malloc(NfexcDes*sizeof(float));
    if (fexcDes == nullptr) {
        // Handle memory allocation failure
        printf("GPA: ERROR - Failed to allocate memory for fexcDes\r\n");
        return; // Early return on allocation failure
    }
    fexcDesLogspace(fMin, fMax, NfexcDes);

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
    if(doPrecalcParam) {
        assignAndResetParamStorage();
        precalcParam();
    }
}

void GPA::reset()
{
    m_print_cntr = 0;

    memset(&gpaData, 0, sizeof(gpaData));

    Nmeas = 0;
    Nper = 0;
    dfexc = 0.0;
    fexc = 0.0;
    fexcPast = 0.0f;
    dfexcj = 0.0f;
    i = 1; // iterating through desired frequency points
    j = 1; // iterating through measurement points w.r.t. reachable frequency
    scaleG = 0.0;
    cr = 0.0;
    ci = 0.0;
    for(int i = 0; i < 3; i++) {
        sU[i] = 0.0;
        sY[i] = 0.0;
#if GPA_EXC_VIA_FILTER
        sR[i] = 0.0;
#endif
    }
    exc = 0.0f;
    sinarg = 0.0;
    sinargR = 0.0f;
    NmeasTotal = 0;
    Aexc = 0.0f;
    pi2Tsfexc = 0.0;
    Nsweep_i = Nstart;
    AexcOut = 0.0f;
    gpaData.fexc  = 0.0f;
    gpaData.Ureal = 0.0f;
    gpaData.Uimag = 0.0f;
    gpaData.Yreal = 0.0f;
    gpaData.Yimag = 0.0f;
    gpaData.Rreal = 0.0f;
    gpaData.Rimag = 0.0f;
    gpaData.MeasPointFinished = false;
    gpaData.MeasFinished = false;
    gpaData.ind = -1;
}

// -----------------------------------------------------------------------------
//      update (operator)
// -----------------------------------------------------------------------------

float GPA::update(float inp, float out)
{
    static bool do_reset_timer{true};
    static Timer timer;

    // a new frequency point has been reached
    if(j == 1) {
        // user info
        if(i == 1 && doPrint) {
            printf("   fexc[Hz]       Ureal         Uimag         Yreal         Yimag         Rreal         Rimag     T avg. mus   Cntr\n");
        }
        // get a new unique frequency point
        while(fexc == fexcPast) {
            if(doPrecalcParam) {
                // measurement finished
                if(i > NfexcAct) {
                    gpaData.MeasPointFinished = false;
                    gpaData.MeasFinished = true;
                    return 0.0f;
                }
                Nper  = Nper_vec[i-1];
                Nmeas = Nmeas_vec[i-1];
                fexc  = fexc_vec[i-1];
                Aexc  = Aexc_vec[i-1];
                pi2Tsfexc = pi2Ts*fexc;
            } else {
                // measurement finished
                if(i > NfexcDes) {
                    gpaData.MeasPointFinished = false;
                    gpaData.MeasFinished = true;
                    return 0.0f;
                }
                calcGPAmeasPara(fexcDes[i - 1]);
                // secure fexc is not higher or equal to nyquist frequency
                if(fexc >= fnyq) {
                    fexc = fexcPast;
                }
                // no frequency found
                if(fexc == fexcPast) {
                    i += 1;
                } else {
                    Aexc = aAexcDes/fexc + bAexcDes;
                    pi2Tsfexc = pi2Ts*fexc;
                }
            }
        }
        // filter scaling
        scaleG = 1.0/sqrt((double)Nmeas);
        // filter coefficients
        cr = cos(pi2Tsfexc);
        ci = sin(pi2Tsfexc);
        // set filter storage zero
        for(int i = 0; i < 3; i++) {
            sU[i] = 0.0;
            sY[i] = 0.0;
#if GPA_EXC_VIA_FILTER
            sR[i] = 0.0;
#endif
        }
        gpaData.MeasPointFinished = false;
    }
    // perfomre the sweep or measure
    if(j <= Nsweep_i) {
        /* this calculation takes 6 mus, 29.06.2021 */
        // timer.start();
        // timer.reset();
        dfexcj = ((float)j - 1.0f)/((float)Nsweep_i - 1.0f);
        dfexcj = div12pi*sinf(pi4*dfexcj) - div812pi*sinf((float)pi2*dfexcj) + dfexcj;
        dfexc  = fexcPast + (fexc - fexcPast)*dfexcj;
        AexcOut = AexcPast + (Aexc - AexcPast)*dfexcj;
        // float dt = timer.read()*1000000.0f;
        // printf("%6i", (int)dt);
    } else {
        dfexc = fexc;
        AexcOut = Aexc;
        // one point DFT filter step for signal su
        sU[0] = scaleG*(double)(inp) + 2.0*cr*sU[1] - sU[2];
        sU[2] = sU[1];
        sU[1] = sU[0];
        // one point DFT filter step for signal sy
        sY[0] = scaleG*(double)(out) + 2.0*cr*sY[1] - sY[2];
        sY[2] = sY[1];
        sY[1] = sY[0];
#if GPA_EXC_VIA_FILTER
        // one point DFT filter step for signal sr
        sR[0] = scaleG*(double)(exc) + 2.0*cr*sR[1] - sR[2];
        sR[2] = sR[1];
        sR[1] = sR[0];
#endif
        if (do_reset_timer) {
            do_reset_timer = false;
            timer.start();
            timer.reset();
        }
    }
    // copy starting value for angle(R)
    if(j == 1 || j == Nsweep_i + 1)
        sinargR = sinarg;
    // measurement of frequencypoint is finished
    if(j == Nmeas + Nsweep_i) {
        const uint32_t meas_time = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
        fexcPast = fexc;
        AexcPast = Aexc;
        Nsweep_i = Nsweep;
        // calculate real and imaginary pars of the signal spectras
        gpaData.fexc  = (float)fexc;
        gpaData.Ureal = (float)(2.0*scaleG*(cr*sU[1] - sU[2]));
        gpaData.Uimag = (float)(2.0*scaleG*ci*sU[1]);
        gpaData.Yreal = (float)(2.0*scaleG*(cr*sY[1] - sY[2]));
        gpaData.Yimag = (float)(2.0*scaleG*ci*sY[1]);
#if GPA_EXC_VIA_FILTER
        gpaData.Rreal = (float)(2.0*scaleG*(cr*sR[1] - sR[2]));
        gpaData.Rimag = (float)(2.0*scaleG*ci*sR[1]);
#else
        gpaData.Rreal = Aexc*cosf(sinargR - piDiv2);
        gpaData.Rimag = Aexc*sinf(sinargR - piDiv2);
#endif
        gpaData.MeasPointFinished = true;
        gpaData.ind++;
        // user info
        if(doPrint) {
            const float avg_time = static_cast<float>(meas_time) / static_cast<float>(Nmeas - 1);           
            const int buffer_length = snprintf(m_buffer, BUFFER_LENGTH,
                                               "%12.5e %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e %11.4e %4d\r\n",
                                               gpaData.fexc,
                                               gpaData.Ureal, gpaData.Uimag,
                                               gpaData.Yreal, gpaData.Yimag,
                                               gpaData.Rreal, gpaData.Rimag,
                                               avg_time, m_print_cntr);
            if (buffer_length >= 0 && buffer_length < BUFFER_LENGTH) {
                if (m_BufferedSerial.writable()) {
                    m_BufferedSerial.write(m_buffer, buffer_length);
                    m_print_cntr++;
                }
            }
        }
        i += 1;
        j = 1;
        do_reset_timer = true;
    } else {
        j += 1;
    }
    // calculate the excitation
    sinarg = fmod(sinarg + pi2Ts*dfexc, pi2);
    NmeasTotal += 1;
    exc = AexcOut*sinf(sinarg);
    return exc;
}

// -----------------------------------------------------------------------------
//      private functions
// -----------------------------------------------------------------------------

void GPA::assignParameters(int NfexcDes, int NperMin, int NmeasMin, float Ts, int Nstart, int Nsweep)
{
    // Basic validation for parameters that don't require frequency range
    if (NfexcDes <= 0 || NperMin <= 0 || NmeasMin <= 0 || Ts <= 0.0f || Nstart < 0 || Nsweep < 0) {
        printf("GPA: ERROR - Invalid parameters in assignParameters\r\n");
        return;
    }
    
    this->NfexcDes = NfexcDes;
    this->NperMin = NperMin;
    this->NmeasMin = NmeasMin;
    this->Ts = Ts;
    this->Nstart = Nstart;
    this->Nsweep = Nsweep;
}

void GPA::calculateDecreasingAmplitudeCoefficients(float Aexc0, float Aexc1)
{
    // calculate coefficients for decreasing amplitude (1/fexc)
    this->aAexcDes = (Aexc1 - Aexc0)/(1.0f/fexcDes[NfexcDes-1] - 1.0f/fexcDes[0]);
    this->bAexcDes = Aexc0 - aAexcDes/fexcDes[0];
}

void GPA::initializeConstants(float Ts)
{
    this->fnyq     = 1.0f / (2.0f * Ts);
    this->pi2      = 2.0 * M_PI;
    this->pi4      = 4.0f * M_PIf;
    this->pi2Ts    = 2.0 * M_PI * (double)Ts;
    this->piDiv2   = M_PIf / 2.0f;
    this->rad2deg  = 180.0f / M_PIf;
    this->div12pi  = 1.0f / (12.0f * M_PIf);
    this->div812pi = 8.0f / (12.0f * M_PIf);
}

void GPA::assignFilterStorage()
{
    sU = (double*)malloc(3*sizeof(double));
    if (sU == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for sU\r\n");
        return;
    }
    
    sY = (double*)malloc(3*sizeof(double));
    if (sY == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for sY\r\n");
        free(sU); // Clean up previously allocated memory
        sU = nullptr;
        return;
    }
    
#if GPA_EXC_VIA_FILTER
    sR = (double*)malloc(3*sizeof(double));
    if (sR == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for sR\r\n");
        free(sU);
        free(sY);
        sU = nullptr;
        sY = nullptr;
        return;
    }
#endif
}

void GPA::assignAndResetParamStorage()
{
    Nper_vec = (int*)malloc(NfexcDes*sizeof(int));
    if (Nper_vec == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for Nper_vec\r\n");
        return;
    }
    
    Nmeas_vec = (int*)malloc(NfexcDes*sizeof(int));
    if (Nmeas_vec == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for Nmeas_vec\r\n");
        free(Nper_vec);
        Nper_vec = nullptr;
        return;
    }
    
    fexc_vec = (double*)malloc(NfexcDes*sizeof(double));
    if (fexc_vec == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for fexc_vec\r\n");
        free(Nper_vec);
        free(Nmeas_vec);
        Nper_vec = nullptr;
        Nmeas_vec = nullptr;
        return;
    }
    
    Aexc_vec = (float*)malloc(NfexcDes*sizeof(float));
    if (Aexc_vec == nullptr) {
        printf("GPA: ERROR - Failed to allocate memory for Aexc_vec\r\n");
        free(Nper_vec);
        free(Nmeas_vec);
        free(fexc_vec);
        Nper_vec = nullptr;
        Nmeas_vec = nullptr;
        fexc_vec = nullptr;
        return;
    }
    
    for(int i = 0; i < NfexcDes; i++) {
        Nper_vec[i]  = 0;
        Nmeas_vec[i] = 0;
        fexc_vec[i]  = 0.0;
        Aexc_vec[i]  = 0.0f;
    }
    NfexcAct = 0;
}

void GPA::fexcDesLogspace(float fMin, float fMax, int NfexcDes)
{
    // calculate logarithmic spaced frequency points
    float Gain = log10f(fMax/fMin)/((float)NfexcDes - 1.0f);
    float expon = 0.0;
    for(int i = 0; i < NfexcDes; i++) {
        fexcDes[i] = fMin*powf(10.0f, expon);
        expon += Gain;
    }
}

void GPA::calcGPAmeasPara(float fexcDes_i)
{
    // Nmeas has to be an integer
    Nper = NperMin;
    Nmeas = (int)floor((float)Nper/fexcDes_i/Ts + 0.5f);
    //  secure that the minimal number of measurements is fullfilled
    int Ndelta = NmeasMin - Nmeas;
    if(Ndelta > 0) {
        Nper = (int)ceil((float)NmeasMin*fexcDes_i*Ts);
        Nmeas = (int)floor((float)Nper/fexcDes_i/Ts + 0.5f);
    }
    // evaluating reachable frequency
    fexc = (float)((double)Nper/(double)Nmeas/(double)Ts);
}

void GPA::precalcParam()
{
    for(int i = 0; i < NfexcDes; i++) {
        calcGPAmeasPara(fexcDes[i]);
        if(fexc != fexcPast && fexc < fnyq) {
            Aexc = aAexcDes/fexc + bAexcDes;
            fexcPast = fexc;
            AexcPast = Aexc;
            // this is a reachable frequency point
            Nper_vec[i]  = Nper;
            Nmeas_vec[i] = Nmeas;
            fexc_vec[i]  = fexc;
            Aexc_vec[i]  = Aexc;
            NfexcAct++;
        }
    }
    reset();
}

float GPA::wrapAngle(float angle)
{
    // wrap angle from (-2pi,2pi) into (-pi,pi)
    if(fabsf(angle) > M_PIf)
        angle -= copysignf(-(float)pi2, angle); // -1*sign(angle)*2*pi + angle;
    return angle;
}

// -----------------------------------------------------------------------------
//      public functions (mainly for debugging)
// -----------------------------------------------------------------------------

void GPA::printGPAfexcDes()
{
    for(int i = 0; i < NfexcDes; i++) {
        printf("%9.4f\n", fexcDes[i]);
    }
}

void GPA::printGPAmeasPara()
{
    printf(" fexcDes[Hz]   fexc[Hz]     Aexc      Nmeas   Nper  Nsweep_i\n");
    wait_us(10000);
    for(int i = 0; i < NfexcDes; i++) {
        calcGPAmeasPara(fexcDes[i]);
        if(fexc == fexcPast || fexc >= fnyq) {
            fexc = 0.0;
            Aexc = 0.0f;
            Nmeas = 0;
            Nper = 0;
            Nsweep_i = 0;
        } else {
            Aexc = aAexcDes/fexc + bAexcDes;
            fexcPast = fexc;
            AexcPast = Aexc;
        }
        NmeasTotal += Nmeas;
        NmeasTotal += Nsweep_i;
        printf("%11.4e %12.4e %10.3e %7i %6i %7i\n", fexcDes[i], (float)fexc, Aexc, Nmeas, Nper, Nsweep_i);
        // wait_us(100 * 1000);
        Nsweep_i = Nsweep;
    }
    printGPAmeasTime();
    reset();
}

void GPA::printGPAmeasTime()
{
    printf(" Number of data points :  %11i\n", NmeasTotal);
    printf(" Measurment time in sec: %12.2f\n", (float)NmeasTotal*Ts);
}

void GPA::printNfexcDes()
{
    printf(" Number of frequancy points:   %3i\n", NfexcDes);
}

void GPA::printPrecalcParam()
{
    printf(" Number of actual frequancy points:   %3i\n", NfexcAct);
    for(int i = 0; i < NfexcAct; i++) printf(" %6i %6i %9.3e %9.3e\n", Nper_vec[i], Nmeas_vec[i], fexc_vec[i], Aexc_vec[i]);
}

GPA::gpadata_t GPA::getGPAdata()
{
    return gpaData;
}

GPA::~GPA()
{
    // Clean up dynamically allocated memory to prevent memory leaks
    if (fexcDes != nullptr) {
        free(fexcDes);
        fexcDes = nullptr;
    }
    
    if (sU != nullptr) {
        free(sU);
        sU = nullptr;
    }
    
    if (sY != nullptr) {
        free(sY);
        sY = nullptr;
    }
    
#if GPA_EXC_VIA_FILTER
    if (sR != nullptr) {
        free(sR);
        sR = nullptr;
    }
#endif
    
    // Clean up parameter storage arrays
    if (Nper_vec != nullptr) {
        free(Nper_vec);
        Nper_vec = nullptr;
    }
    
    if (Nmeas_vec != nullptr) {
        free(Nmeas_vec);
        Nmeas_vec = nullptr;
    }
    
    if (fexc_vec != nullptr) {
        free(fexc_vec);
        fexc_vec = nullptr;
    }
    
    if (Aexc_vec != nullptr) {
        free(Aexc_vec);
        Aexc_vec = nullptr;
    }
}

// -----------------------------------------------------------------------------
//      validation functions
// -----------------------------------------------------------------------------

bool GPA::validateParameters(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep)
{
    // Validate frequency parameters
    if (fMin <= 0.0f || fMax <= 0.0f) {
        printf("GPA: ERROR - Frequencies must be positive (fMin=%f, fMax=%f)\r\n", fMin, fMax);
        return false;
    }
    
    if (fMin >= fMax) {
        printf("GPA: ERROR - fMin must be less than fMax (fMin=%f, fMax=%f)\r\n", fMin, fMax);
        return false;
    }
    
    // Validate sampling time
    if (Ts <= 0.0f) {
        printf("GPA: ERROR - Sampling time must be positive (Ts=%f)\r\n", Ts);
        return false;
    }
    
    // Check Nyquist frequency constraint
    float fnyq = 1.0f / (2.0f * Ts);
    if (fMax >= fnyq) {
        printf("GPA: ERROR - fMax (%f Hz) must be less than Nyquist frequency (%f Hz)\r\n", fMax, fnyq);
        return false;
    }
    
    // Validate integer parameters
    if (NfexcDes <= 0) {
        printf("GPA: ERROR - Number of frequency points must be positive (NfexcDes=%d)\r\n", NfexcDes);
        return false;
    }
    
    if (NperMin <= 0) {
        printf("GPA: ERROR - Minimum periods must be positive (NperMin=%d)\r\n", NperMin);
        return false;
    }
    
    if (NmeasMin <= 0) {
        printf("GPA: ERROR - Minimum measurements must be positive (NmeasMin=%d)\r\n", NmeasMin);
        return false;
    }
    
    if (Nstart < 0 || Nsweep < 0) {
        printf("GPA: ERROR - Start and sweep samples must be non-negative (Nstart=%d, Nsweep=%d)\r\n", Nstart, Nsweep);
        return false;
    }
    
    // Validate amplitude parameters
    if (Aexc0 < 0.0f || Aexc1 < 0.0f) {
        printf("GPA: ERROR - Excitation amplitudes must be non-negative (Aexc0=%f, Aexc1=%f)\r\n", Aexc0, Aexc1);
        return false;
    }
    
    return true;
}
