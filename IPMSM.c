/* ==SOURCE file list of “templateStrukC.c” with Structure C == */ 
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  IPMSM                                                    
#include "simstruc.h"                                                                           
#include <math.h>                                                                                
                                                                                                                
#define U(element) (*uPtrs[element])  /*Pointer to Input Port0*/  
                                                                                                                
static void mdlInitializeSizes(SimStruct *S){                                    
ssSetNumContStates(S, 4);
if (!ssSetNumInputPorts(S, 1)) return;                                             
ssSetInputPortWidth(S, 0, 4);                                                           
ssSetInputPortDirectFeedThrough(S, 0, 1);                                    
ssSetInputPortOverWritable(S, 0, 1);                                              
if (!ssSetNumOutputPorts(S, 1)) return;                                          
ssSetOutputPortWidth(S, 0, 8);                                                         
ssSetNumSampleTimes(S, 1);                                                            
                                                                                                              
ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); }    
                                                                                                              
static void mdlInitializeSampleTimes(SimStruct *S) {                   
ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);         
ssSetOffsetTime(S, 0, 0.0); }


// Torque ElectroMechanical Function
typedef struct {
  real_T Te;
}Torque;

void EMT (Torque *T, real_T Np, real_T Ff, real_T id, real_T  iq, real_T Ld, real_T Lq) {
  T->Te = (3.0/2.0) * Np *(Ff * iq + (Ld - Lq) * id * iq);
}
                                                                                                             
#define MDL_INITIALIZE_CONDITIONS                                  
static void mdlInitializeConditions(SimStruct *S) {                      
                                                                                                            
  real_T 	*X0 = ssGetContStates(S);                                       
  int_T 	nStates = ssGetNumContStates(S);                         
  int_T 	i;                                                                                  

/* initialize the states to 0.0 */                                                            
  for (i=0; i < nStates; i++) {X0[i] = 0.0;} }                                       
                                                                                                             
static void mdlOutputs(SimStruct *S, int_T tid) {                          
  real_T            *Y = ssGetOutputPortRealSignal(S,0);                  
  real_T            *X = ssGetContStates(S);                                         
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

  real_T Np, theta_e, ialph, ibeta, ia, ib, ic, Te, Ff, Ld, Lq;
  Torque T;


  //initialize variable
  Np = 4;
  Ld = 0.0002;
  Lq = 0.0004;
  Ff = 0.062;
  EMT(&T, Np, Ff, X[0], X[1], Ld, Lq);
  Te = T.Te;
  theta_e = Np * X[3];

  //current d-q axis into alpha-beta
  ialph = X[0] * cos(theta_e) - X[1] * sin(theta_e);
  ibeta = X[0] * sin(theta_e) + X[1] * cos(theta_e) ;

  //current alpha-beta into abc
  ia = sqrt(2.0/3.0) * ialph;
  ib = sqrt(2.0/3.0) * (-0.5 *ialph + sqrt (3.0/4) * ibeta);
  ic = sqrt (2.0/3.0) * (-0.5 * ialph - sqrt(3.0/4) * ibeta);

  Y[0] = ia;
  Y[1] = ib;
  Y[2] = ic;
  Y[3] = X[0];
  Y[4] = X[1];
  Y[5] = Te;
  Y[6] = X[2] * (60/(2 * 3.14));
  Y[7] = X[3];
                                                                                                              
   }                                                                                    

#define MDL_DERIVATIVES                                                          
static void mdlDerivatives(SimStruct *S) {                                      
  real_T  *dX = ssGetdX(S);                                                               
  real_T  *X = ssGetContStates(S);                                                    
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);  

  real_T Vpwma, Vpwmb, Vpwmc,Valph, Vbeta, Vd, Vq, Te, Td, Ld, Lq, j, Rs, we, B, phi_f, Np, id, iq, wm, theta_m, theta_e, id_dot, iq_dot, wm_dot, theta_m_dot;
  Torque T;

  //Initialize state
  id = X[0];
  iq = X[1];
  wm = X[2];
  theta_m = X[3];

  //Initialize input
  Vpwma = U(0);
  Vpwmb = U(1);
  Vpwmc = U(2);
  Td = U(3);

  //Initialize value and equation
  Rs = 0.025;
  Ld = 0.0002;
  Lq = 0.0004;
  j = 0.01;
  phi_f = 0.062;
  Np = 4;
  B = 0.001;
  we = Np * wm;
  EMT(&T, Np, phi_f, id, iq, Ld, Lq);
  Te = T.Te;
  theta_e = Np * theta_m;


  //PWM to alhpa-beta axis voltage
  Valph = sqrt(2.0/3.0) * (Vpwma  - (0.5 * Vpwmb) - ( 0.5 * Vpwmc));
  Vbeta = sqrt(2.0/3.0) * (sqrt(3.0/4) * Vpwmb - (sqrt(3.0/4) * Vpwmc));

  //alpha-beta to d-q axis voltage
  Vd = Valph * cos(theta_e) + Vbeta * sin(theta_e);
  Vq = -Valph * sin(theta_e) + Vbeta * cos(theta_e);
  
  //D-Q Voltage to D-Q Current
  id_dot = 1/Ld * (Vd - Rs * id + (we * Lq) * iq);
  iq_dot = 1/Lq * (Vq - (we * Ld) * id - (Rs * iq) - we * phi_f);
  wm_dot = 1/j * ((Te - Td) - B * wm);
  theta_m_dot = wm;

  dX[0] = id_dot;
  dX[1] = iq_dot;
  dX[2] = wm_dot;
  dX[3] = theta_m_dot;
                                                                                                              
   }                                                                     
                                                                                                              
static void mdlTerminate(SimStruct *S)                                         
{} /*Keep this function empty since no memory is allocated*/    
                                                                                                            
#ifdef  MATLAB_MEX_FILE                                                        
/* Is this file being compiled as a MEX-file? */                             
#include "simulink.c"   /* MEX-file interface mechanism */      
#else                                                                                                   
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif                                                                                               
