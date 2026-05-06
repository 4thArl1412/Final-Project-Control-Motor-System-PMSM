#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME SVPWM
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/


static void mdlInitializeSizes(SimStruct *S){
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 5);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); }

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0); }

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T t = ssGetT(S);
    real_T Fs, Voffset, Vmax, Vmin, Va, Vb, Vc, carrier, Vmod;
    int i, Vdc;


    //assign variable
    Vdc = U(0)/sqrt(3);
    Fs = U(1);
    Va = U(2);
    Vb = U(3);
    Vc = U(4);
    carrier = Fs * fmod(t, 1/Fs);

    //setting the vmax and vmin for offset
    Vmax = Va;
    if (Vmax < Vb) Vmax = Vb;
    if (Vmax < Vc) Vmax = Vc;

    Vmin = Va;
    if (Vmin > Vb) Vmin = Vb;
    if (Vmin > Vc) Vmin = Vc;

    Voffset = - (Vmax + Vmin)/2.0;

    //svpwm logic
    for (i = 0; i < 3; i++) {
        Vmod = U(i+2) + Voffset;

        if ((fabs(Vmod))/ Vdc  >= carrier){
            Y[i] = Vdc;
        }
        else {
            Y[i] = 0.0;
        }
        if (Vmod < 0.0) {
            Y[i] = -Y[i];
        }
    }




}


static void mdlTerminate(SimStruct *S)
{ } /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif