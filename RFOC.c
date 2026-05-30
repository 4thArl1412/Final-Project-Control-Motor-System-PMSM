#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME RFOC
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S){
    ssSetNumDiscStates(S, 7);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE
    | SS_OPTION_DISCRETE_VALUED_OUTPUT));}

static void mdlInitializeSampleTimes(SimStruct *S){
    ssSetSampleTime(S, 0, 1e-4);
    ssSetOffsetTime(S, 0, 0.0);}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S){
    real_T *X0 = ssGetRealDiscStates(S);
    int_T nXStates = ssGetNumDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    int_T i;

    /* initialize the states to 0.0 */
    for (i=0; i < nXStates; i++) {
        X0[i] = 0.0; } }

static void mdlOutputs(SimStruct *S, int_T tid){
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    real_T *X = ssGetRealDiscStates(S);


    Y[0] = X[2];
    Y[1] = X[3];
    Y[2] = X[4];

}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T KpId, KiId, KpIq, KiIq,  errorId, errorIq, integral_old_Id, integral_old_Iq, integrator_Id, integrator_Iq, outputVd, outputVq;
    real_T wm, theta_m, Ld, Lq, Rs, refId, actualId, refIq, actualIq, Tid, Tiq, Np;
    real_T phi_f, Vmax;
    real_T Valph, Vbeta, Va, Vb, Vc, theta_e, wc, alpha_filter, actualId_filtered, actualIq_filtered;
    real_T dt = 1e-4;
    real_T prevId_filt, prevIq_filt;

    //initalize input and state
    refId = U(0);
    refIq = U(1);
    actualId = U(2);
    actualIq = U(3);
    wm = U(4);
    theta_m = U(5);
    integral_old_Id = X[0];
    integral_old_Iq = X[1];
    prevId_filt = X[5];
    prevIq_filt = X[6];

    //initialize contant value
    Np = 4;
    phi_f = 0.062;
    Rs = 0.025;
    Ld = 0.0002;
    Lq = 0.0004;
    Vmax = 99.7;
    wc = 250.0;
    alpha_filter = 0.05;
    Tid = Ld/Rs;
    Tiq = Lq/Rs;
    theta_e = Np * theta_m;

    //set gain propotional and integrator for each current
    KpId = Ld * wc;
    KpIq = Lq * wc;
    KiId = Rs* wc;
    KiIq = Rs* wc;


    //PI by the paper reference
    /*
    KpId = 40;
    KiId = 60;
    KpIq = 20;
    KiIq = 60;
    */

    //set low pass filter to reduce noise
    actualId_filtered = (alpha_filter * actualId) + (1.0 - alpha_filter) * prevId_filt;
    actualIq_filtered = (alpha_filter * actualIq) + (1.0 - alpha_filter) * prevIq_filt;


    //set error for each kp and ki
    errorId = refId - actualId_filtered;
    errorIq = refIq - actualIq_filtered;
    integrator_Id = integral_old_Id + KiId * dt * errorId;
    integrator_Iq = integral_old_Iq + KiIq * dt * errorIq;

    //anti windup system
    if (integrator_Id > Vmax) integrator_Id = Vmax;
    if (integrator_Id < -Vmax) integrator_Id = -Vmax;
    if (integrator_Iq > Vmax) integrator_Iq = Vmax;
    if (integrator_Iq < -Vmax) integrator_Iq = -Vmax;

    //set output
    outputVd = (KpId * errorId) + (integrator_Id) - (Np * wm * Lq * refIq) ;
    outputVq = (KpIq * errorIq) + (integrator_Iq) + (Np * wm * ((Ld * refId) + phi_f));

    //output clamp so it will not bypass the maximum voltage
    if (outputVd > Vmax) outputVd = Vmax;
    if (outputVd < -Vmax) outputVd = -Vmax;
    if (outputVq > Vmax) outputVq = Vmax;
    if (outputVq < -Vmax) outputVq = -Vmax;


    //voltage d-q axis into alpha beta
    Valph = outputVd * cos(theta_e) - outputVq * sin(theta_e);
    Vbeta = outputVd * sin(theta_e) + outputVq * cos(theta_e);

    //voltage alpha beta to abc
    Va = sqrt(2.0/3.0) * Valph;
    Vb = sqrt(2.0 / 3.0) * (-0.5 *Valph + sqrt (3.0/4.0) * Vbeta);
    Vc = sqrt (2.0/3.0) * (-0.5 * Valph - sqrt(3.0/4.0) * Vbeta);

    //set update state
    X[0] = integrator_Id;
    X[1] = integrator_Iq;
    X[2] = Va;
    X[3] = Vb;
    X[4] = Vc;
    X[5] = actualId_filtered;
    X[6] = actualIq_filtered;



}

static void mdlTerminate(SimStruct *S)
{ } /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif

