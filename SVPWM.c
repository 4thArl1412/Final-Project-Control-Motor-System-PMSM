#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME SVPWM
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S){
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 5); /* [Vdc, Fs, Va, Vb, Vc] */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3); /* [PWMa, PWMb, PWMc] */
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    // Kita gunakan Continuous agar bisa membaca variabel waktu 't' dengan presisi
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T t = ssGetT(S);

    // PERBAIKAN: Vdc dimasukkan ke real_T
    real_T Fs, Voffset, Vmax, Vmin, Va, Vb, Vc, carrier, Vmod, Vdc;
    int i; // i tetap integer untuk indexing loop

    // 1. Assign variable
    Vdc = U(0) / 2.0; // Gunakan 2.0 agar terbaca sebagai float division
    Fs = U(1);
    Va = U(2);
    Vb = U(3);
    Vc = U(4);

    // 2. Buat sinyal Carrier Segitiga (0.0 sampai 1.0)
    // Gunakan 1.0/Fs untuk memastikan presisi koma
    carrier = Fs * fmod(t, 1.0/Fs);

    // 3. Setting the Vmax and Vmin for Zero-Sequence Injection (SVPWM)
    Vmax = Va;
    if (Vb > Vmax) Vmax = Vb;
    if (Vc > Vmax) Vmax = Vc;

    Vmin = Va;
    if (Vb < Vmin) Vmin = Vb;
    if (Vc < Vmin) Vmin = Vc;

    Voffset = - (Vmax + Vmin) / 2.0;

    // 4. SVPWM Logic Modulation
    for (i = 0; i < 3; i++) {
        // Tegangan referensi ditambah offset
        Vmod = U(i+2) + Voffset;

        // Bandingkan rasio tegangan dengan carrier segitiga
        if ((fabs(Vmod) / Vdc) >= carrier){
            Y[i] = Vdc; // Sakelar ON
        }
        else {
            Y[i] = 0.0; // Sakelar OFF
        }

        // Kembalikan polaritas arah tegangan
        if (Vmod < 0.0) {
            Y[i] = -Y[i]; // Sakelar kutub negatif ON
        }
    }
}

static void mdlTerminate(SimStruct *S) { }

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif