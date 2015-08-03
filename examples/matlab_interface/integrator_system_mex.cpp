#include "mex.h" 
#include <iostream>

#include "integrator_system.h"

// support helper class to define a pointer suited for matlabs handle object
// @see teb::matlab::matlabClassHandle
using namespace teb::matlab; 

// Create typedef for our controller, solver and system type
typedef teb::TebController<2, 1> ControllerType;
typedef teb::SolverLevenbergMarquardtEigenSparse SolverType;
typedef IntegratorSystem<2> SystemType;

// Create global variables
std::unique_ptr<SolverType> solver; // Solver
std::unique_ptr<SystemType> dynamics; // System dynamics

void mexFunction(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])
{
	// Get command string
	char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string with less than 64 characters.");


	// Create new obejct if command equals "new"
	if (!strcmp("new", cmd))
	{
		// Check parameters
		if (nlhs != 1)
			mexErrMsgTxt("New: One output expected.");


		// Instantiate new objects
		solver = std::unique_ptr<SolverType>(new SolverType);
		dynamics = std::unique_ptr<SystemType>(new SystemType);
		ControllerType* ctrl = new ControllerType(solver.get());

		// Setup controller
		ctrl->setSystemDynamics(dynamics.get()); // Satisfy system dynamics

		ctrl->activateObjectiveTimeOptimal(); // Minimum time control
		ctrl->activateControlBounds(0, -1.0, 1.0); // Control input bounds

		// Return handle to the new controller instance
		plhs[0] = convertPtr2Mat< ControllerType >(ctrl);

		return;
	}


    // For further work, we need a second input (class instance handle)
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete object
	if (!strcmp("delete", cmd))
	{
		destroyObject<ControllerType>(prhs[1]);
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// For further work, retrieve the class instance pointer from the second input
	ControllerType* ctrl = convertMat2Ptr<ControllerType>(prhs[1]);

	// Now start calling desired class method

	// Call teb::BaseController::step()
	if (!strcmp("step", cmd))
	{
		// Check inputs and outputs
		if (nrhs != 4)  mexErrMsgIdAndTxt("teb_package:step:nrhs", "Four inputs required: cmd, obj, x0 and xf [2x1].");
		if (nlhs != 1)  mexErrMsgIdAndTxt("teb_package:step:nlhs", "One output required: u [1x1]//"); // teb [3 x n_max], dt [1x1].");

		double* x0 = mxGetPr(prhs[2]); // start state
		double* xf = mxGetPr(prhs[3]); // goal state
		double u = 0; // store control

		// Perform open-loop planning
		ctrl->step(x0, xf, &u);

		// create output (control input u)
		plhs[0] = mxCreateDoubleScalar(u);
		return;
	}


	// Return TEB matrix teb::BaseController::getStateCtrlInfoMat()
	if (!strcmp("tebmatrix", cmd))
	{
		// Check inputs and outputs
		if (nrhs != 2) mexErrMsgTxt("No more input parameter allowed.");
		if (nlhs != 1) mexErrMsgTxt("One output for the teb matrix required.");
		
		// Size of the mat: [p+q+1 x n] // +1: add time vector
		int m = ctrl->NoStates + ctrl->NoControls + 1;
		int n = ctrl->getN();
		plhs[0] = mxCreateNumericMatrix(m, n, mxDOUBLE_CLASS, mxREAL);
		
		// Get values
		// Wrap mxArray into an Eigen::Matrix
		Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::ColMajor>> mat_map(mxGetPr(plhs[0]), m, n); // Matlab uses column major representation
		mat_map.topRows(1) = ctrl->getAbsoluteTimeVec().transpose(); 
		mat_map.bottomRows(m-1) = ctrl->getStateCtrlInfoMat();
		return;
	}


    // If this line is reached, the command is not recognized
	mexErrMsgTxt("Command noch recognized.");
} 
