#include "path_alg.h"

extern "C" {

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  s_path_alg

#define DT 0.1

#include "simstruc.h"

  /* Function: mdlInitializeSizes ===============================================
   * Abstract:
   *    The sizes information is used by Simulink to determine the S-function
   *    block's characteristics (number of inputs, outputs, states, etc.).
   */
#define MDL_INITIAL_SIZES
  static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 13);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
      return;
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    // Configure inputs    
    if (!ssSetNumInputPorts(S, 4))
      return;
    
    ssSetInputPortVectorDimension(S, 0, 2); // LC coordinates [lat long]
    ssSetInputPortVectorDimension(S, 1, 2); // WR position    [lat long]
    ssSetInputPortWidth(S, 2, 1);           // WR angle       [radians]
    ssSetInputPortWidth(S, 3, 1);           // WR velocity    [m/s]

    // Configure each input port as being used directly in the output
    // Guess this is the case?
    for (int i = 0; i < 4; i++)
      ssSetInputPortDirectFeedThrough(S, i, 1);
    
    // Configure outputs    
    if (!ssSetNumOutputPorts(S, 3))
      return;
    
    ssSetOutputPortWidth(S, 0, 1);          // Gas            [scalar]      [0, 100]
    ssSetOutputPortWidth(S, 1, 1);          // Nozzle angle   [radians] ??? [-6, 6] / 180 * pi 
    ssSetOutputPortVectorDimension(S, 2, 7);// Test vector containing:
                                            //   dist_to_path(A, B, WR).dist_a_error
                                            //   state.desired_rotation
                                            //   state.distance_to_lc
                                            //   state.target.x, state.target.y
                                            //   state.pred_loc.x, state.pred_loc.y

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
    ssSetSampleTime(S, 0, DT);
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
    double ps[13];
    for (int i = 0; i < 13; i++)
      ps[i] = mxGetScalar(ssGetSFcnParam(S,i));
    //double deltaT = mxGetScalar(ssGetSFcnParam(S,0));
    const double deltaT = ps[0];
    const double dist_ref = 55;
    const double SCALING = 1.0;
    const pid_v throttle = { ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], 1.0, SCALING};
    const pid_v nozzle = { ps[7], ps[8], ps[9], ps[10], ps[11], ps[12], 3.0, SCALING};
    ssGetPWork(S)[0] = (void *) new path_track (deltaT, dist_ref,
						throttle, nozzle);
  }
  
  #define MDL_UPDATE
  static void mdlUpdate(SimStruct *S, int_T tid) {
    static Eigen::Vector2d last_coord(0, 0);
    path_track *p = (path_track *) ssGetPWork(S)[0];
    InputRealPtrsType u = ssGetInputPortRealSignalPtrs(S, 0);

    const Eigen::Vector2d new_coord(*u[0], *u[1]);
    if (last_coord != new_coord) {
      p->push_coord(new_coord);
      last_coord = new_coord;
    }
  }

  /* Function: mdlOutputs =======================================================
   * Abstract:
   *    In this function, you compute the outputs of your S-function
   *    block. Generally outputs are placed in the output vector, ssGetY(S).
   */
  static void mdlOutputs(SimStruct *S, int_T tid) {
    // Get inputs
    InputRealPtrsType u = ssGetInputPortRealSignalPtrs(S, 1);
    InputRealPtrsType v = ssGetInputPortRealSignalPtrs(S, 2);
    InputRealPtrsType w = ssGetInputPortRealSignalPtrs(S, 3);
    const Eigen::Vector2d wr_position(*u[0], *u[1]);
    const double wr_angle = *v[0];
    const double wr_velocity  = *w[0];

    ssPrintf("\n[%f] ", ssGetT(S));

    // Do work
    path_track *p = (path_track *) ssGetPWork(S)[0];
    ssPrintf("[coords_size = %d] ", p->coords_queue_size());
    p->tick(wr_position, wr_angle, wr_velocity, DT);

    state_params state = p->get_state();
    /*
    ssPrintf("target: [%f %f]  wr_position: [%f %f]  wr_velocity: %f\n",
	     state.target[0], state.target[1],
	     state.wr_position[0], state.wr_position[1]), state.wr_velocity;
    ssPrintf("desired_rotation: %f  distance_to_lc: %f",
	     state.desired_rotation, state.distance_to_lc);
    */
    real_T  *test = ssGetOutputPortRealSignal(S,2);
    test[0] = state.dist_a_error;
    test[1] = state.desired_rotation;
    test[2] = state.distance_to_lc;
    test[3] = state.target[0];
    test[4] = state.target[1];
    test[5] = state.pred_loc[0];
    test[6] = state.pred_loc[1];
    result res = p->get_results();
    // Provide outputs
    real_T  *gas = ssGetOutputPortRealSignal(S,0);
    real_T  *nozzle_angle = ssGetOutputPortRealSignal(S,1);
    gas[0] = res.gas;
    nozzle_angle[0] = p->res.nozzle_angle;
    //	ssPrintf("done");
  }
  
  /* Function: mdlTerminate =====================================================
   * Abstract:
   *    In this function, you should perform any actions that are necessary
   *    at the termination of a simulation.  For example, if memory was
   *    allocated in mdlStart, this is the place to free it.
   */
  static void mdlTerminate(SimStruct *S) {
    path_track *p = (path_track *) ssGetPWork(S)[0];
    delete p;
  }
  
#ifdef  MATLAB_MEX_FILE
  #include "simulink.c"
#else
  #include "cg_sfun.h"
#endif

}
