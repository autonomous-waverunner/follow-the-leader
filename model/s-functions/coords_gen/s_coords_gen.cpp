#include "coords_gen.h"

extern "C" {

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  s_coords_gen

#include "simstruc.h"

  /* Function: mdlInitializeSizes ===============================================
   * Abstract:
   *    The sizes information is used by Simulink to determine the S-function
   *    block's characteristics (number of inputs, outputs, states, etc.).
   */
  static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 2);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
      return;
    }
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    if (!ssSetNumInputPorts(S, 0))
      return;
    
    if (!ssSetNumOutputPorts(S, 1))
      return;
    
    ssSetOutputPortVectorDimension(S, 0, 2);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    ssSetOptions(S, 0);
  }
  
  /* Function: mdlInitializeSampleTimes =========================================
   * Abstract:
   *    This function is used to specify the sample time(s) for your
   *    S-function. You must register the same number of sample times as
   *    specified in ssSetNumSampleTimes.
   */
  static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
  }
  

  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  #define MDL_START
  static void mdlStart(SimStruct *S) {
    size_t strlen = mxGetN(ssGetSFcnParam(S,1)) + 1;
    char fname[strlen];
    mxGetString(ssGetSFcnParam(S,1), fname, strlen);
    try {
      ssGetPWork(S)[0] = (void *) new coords_gen (fname);
    } catch (int e) {
      if (e == EX_FILE_NOT_FOUND) {
	ssSetErrorStatus(S, "File doesn't exist");
      } else if (e == EX_PARSE) {
	ssSetErrorStatus(S, "Malformated input file");
      }
      return;
    }
  }

  /* Function: mdlOutputs =======================================================
   * Abstract:
   *    In this function, you compute the outputs of your S-function
   *    block. Generally outputs are placed in the output vector, ssGetY(S).
   */
  static void mdlOutputs(SimStruct *S, int_T tid) {
    coords_gen *c = (coords_gen *) ssGetPWork(S)[0];
    real_T  *y = ssGetOutputPortRealSignal(S,0);
    auto t = c->get();
    y[0] = t.first;
    y[1] = t.second;
  }
  
  /* Function: mdlTerminate =====================================================
   * Abstract:
   *    In this function, you should perform any actions that are necessary
   *    at the termination of a simulation.  For example, if memory was
   *    allocated in mdlStart, this is the place to free it.
   */
  static void mdlTerminate(SimStruct *S) {
    coords_gen *c = (coords_gen *) ssGetPWork(S)[0];
    delete c;
  }
  
#ifdef  MATLAB_MEX_FILE
  #include "simulink.c"
#else
  #include "cg_sfun.h"
#endif

}
