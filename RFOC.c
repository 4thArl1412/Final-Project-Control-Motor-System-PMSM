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

    real_T Valph, Vbeta, Va, Vb, Vc, theta_m, theta_e, Np;

    //initialize variable
    Np = 4;
    theta_e = Np * X[6];

    //voltage d-q axis into alpha beta
    Valph = X[4] * cos(theta_e) - X[5] * sin(theta_e);
    Vbeta = X[4] * sin(theta_e) + X[5] * cos(theta_e);

    //voltage alpha beta to abc
    Va = sqrt(2.0/3.0) * Valph;
    Vb = sqrt(2.0 / 3.0) * (-0.5 *Valph + sqrt (3.0/4) * Vbeta);
    Vc = sqrt (2.0/3.0) * (-0.5 * Valph - sqrt(3.0/4) * Vbeta);

    Y[0] = Va;
    Y[1] = Vb;
    Y[2] = Vc;

}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T KpId, KiId, KpIq, KiIq,  errorId, errorIq, integral_old_Id, integral_old_Iq, integrator_Id, integrator_Iq, outputVd, outputVq;
    real_T wm, theta_m, Ld, Lq, Rs, refId, actualId, refIq, actualIq, Ts, L, Tid, Tiq, errorId_old, errorIq_old, Np;
    real_T phi_f, Vmax;
    real_T dt = 1e-4;

    //initalize input and state
    refId = U(0);
    refIq = U(1);
    actualId = U(2);
    actualIq = U(3);
    wm = U(4);
    theta_m = U(5);
    errorId_old = X[0];
    errorIq_old = X[1];
    integral_old_Id = X[2];
    integral_old_Iq = X[3];

    //initialize contant value
    Np = 4;
    phi_f = 0.062;
    Rs = 0.025;
    Ld = 0.0002;
    Lq = 0.0004;
    Vmax = 283;
    Ts = 0.001; //Time Sampling
    L = 0.5 * Ts;
    Tid = (L/0.3);
    Tiq = (L/0.3);

    //set gain propotional and integrator for each current
    KpId = Ld/Tid;
    KpIq = Lq/Tiq;
    KiId = Rs/Tid;
    KiIq = Rs/Tiq;

    //set error for each kp and ki
    errorId = refId - actualId;
    errorIq = refIq - actualIq;
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

    //set update state
    X[0] = errorId;
    X[1] = errorIq;
    X[2] = integrator_Id;
    X[3] = integrator_Iq;
    X[4] = outputVd;
    X[5] = outputVq;
    X[6] = theta_m;



}

static void mdlTerminate(SimStruct *S)
{ } /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif

