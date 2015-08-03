#include "integrator_system.h"
#include <teb_package.h>

#ifdef __cplusplus
extern "C" { // use the C fcn-call standard for all functions  
#endif       // defined within this scope   

#define S_FUNCTION_NAME integrator_system_sfun /* Defines and Includes */
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"


#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{

	// Reserve elements in the pointer work vector to store C++ objects
	// (see mdlStart function for object instantiation)
	ssSetNumPWork(S, 3); // Assume 3 objects (controller, solver, system)

	// Check inputs and outputs
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }

    if (!ssSetNumInputPorts(S, 2)) return;
    // input 1: xf
    ssSetInputPortWidth(S, 0, 2);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    // input 2: x0
    ssSetInputPortWidth(S, 1, 2);
    ssSetInputPortDirectFeedThrough(S, 1, 1);  

    // output: u
    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 1);
	
    ssSetNumSampleTimes(S, 1); // TODO

    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    }

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME); // TODO
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
	teb::Config* cfg = new teb::Config;         
        cfg->teb.teb_iter = 1;
  
        // Instantiate persistent objects (delete objects in mdlTerminate)
	teb::BaseSolver* solver = new teb::SolverLevenbergMarquardtEigenSparse;
	teb::TebController<2, 1>* ctrl = new teb::TebController<2,1>(cfg, solver);
	IntegratorSystem<2>* int_sys = new IntegratorSystem<2>;
	
	// Store void pointers in simulinks work vector for persistent variables.
	ssGetPWork(S)[0] = (void*) solver; // solver
	ssGetPWork(S)[1] = (void*) ctrl; // controller
	ssGetPWork(S)[2] = (void*) int_sys; // system

	// setup controller
	ctrl->setSystemDynamics(int_sys);
	ctrl->activateObjectiveTimeOptimal();
	ctrl->activateControlBounds(0, -1.0, 1.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
	// reset trajectroy
	teb::TebController<2, 1>* ctrl = (teb::TebController<2, 1>*) ssGetPWork(S)[1];
	ctrl->resetController();
}

#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
	// Retrieve persistent controller object
	teb::TebController<2, 1>* ctrl = (teb::TebController<2, 1>*) ssGetPWork(S)[1];

	// Get input and output parameters
    InputRealPtrsType xfPtrs = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType x0Ptrs = ssGetInputPortRealSignalPtrs(S,1);
    real_T *uPtr = ssGetOutputPortRealSignal(S,0);
    
    // Perform MPC step
    ctrl->step(*x0Ptrs, *xfPtrs, uPtr);
    
}

static void mdlTerminate(SimStruct *S)
{
	// Destroy persistent objects
	teb::BaseSolver* solver = (teb::BaseSolver*) ssGetPWork(S)[0];
	teb::TebController<2, 1>* ctrl = (teb::TebController<2, 1>*) ssGetPWork(S)[1];
	IntegratorSystem<2>* int_sys = (IntegratorSystem<2>*) ssGetPWork(S)[2];
	delete solver;
	delete ctrl;
	delete int_sys;
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif

#ifdef __cplusplus
} // end of extern "C" scope
#endif