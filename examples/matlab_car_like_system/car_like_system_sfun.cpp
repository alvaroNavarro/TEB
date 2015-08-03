
#include <teb_package.h>
#include "controller_car_like.h"

#ifdef __cplusplus
extern "C" { // use the C fcn-call standard for all functions  
#endif       // defined within this scope   

#define S_FUNCTION_NAME car_like_system_sfun /* Defines and Includes */
#define S_FUNCTION_LEVEL 2
  
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

const int n_states   = 4;
const int n_controls = 2;

const double tebStepSize = 0.01;

/* Define horizon length */
const double T = 2.0;

double runtime_max = 0.0;

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

    if (!ssSetNumInputPorts(S, 4)) return;
    
    //! input 1: x0
    ssSetInputPortWidth(S, 0, n_states);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    //! input 2: xf
    ssSetInputPortWidth(S, 1, n_states);
    ssSetInputPortDirectFeedThrough(S, 1, 1);  
	    
    //! input 3: weight_Obstacle
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    //! input 4: weight_YawAngle
    ssSetInputPortWidth(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
	
    //! number of TEB points n = T / dt + 1
    unsigned int n = T / tebStepSize + 1;
    // output: u

    if (!ssSetNumOutputPorts(S, 1 + n_states + n_controls)) return;
    
    ssSetOutputPortWidth(S, 0, n_controls);	    
    ssSetOutputPortWidth(S, 1, n);   // x    
    ssSetOutputPortWidth(S, 2, n);   // y    
    ssSetOutputPortWidth(S, 3, n);   // theta    
    ssSetOutputPortWidth(S, 4, n);   // phi
    ssSetOutputPortWidth(S, 5, n);   // v
    ssSetOutputPortWidth(S, 6, n);   // omega
            
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
  
  // optimization settings   -   Default values but the weight factor change during the simulation..
  cfg->optim.solver.lsq.weight_equalities = 200;			// 200
  cfg->optim.solver.lsq.weight_inequalities = 20;			// 20
  cfg->optim.solver.lsq.weight_adaptation_factor = 2;			// 2
  cfg->optim.solver.solver_iter = 3;					// default: 5
  
  // trajectory settings
  cfg->teb.n_pre = 50;
  cfg->teb.dt_ref = 0.1;
  cfg->teb.dt_hyst = cfg->teb.dt_ref/10;       
  cfg->teb.n_max = 250;
  cfg->teb.n_min = 2;
  cfg->teb.teb_iter = 5;   
  
  InputRealPtrsType weight_Obstacle = ssGetInputPortRealSignalPtrs(S, 2);	
  InputRealPtrsType weight_YawAngle = ssGetInputPortRealSignalPtrs(S, 3);
      
  // Instantiate persistent objects (delete objects in mdlTerminate)
  teb::BaseSolver* solver = new teb::SolverLevenbergMarquardtEigenSparse;
  teb::RobControllerCarLike<n_states, n_controls>* ctrl = new teb::RobControllerCarLike<n_states, n_controls>(cfg, solver);
  MobileRobotCarLikeSystem<n_states, n_controls>* rob_sys = new MobileRobotCarLikeSystem<n_states, n_controls>;
  
  //! Setup Mobile
  rob_sys->setTypeDriving(teb::TypeDriving::REAR_WHEEL);
  rob_sys->setRobotDimesion(0.9,0.35);
  
  //! Setup controller
  ctrl->activateObjectiveTimeOptimal();        
  ctrl->setSystemDynamics(rob_sys);    
  ctrl->activateControlBounds({-0.1,-0.5},{1,0.5});
  ctrl->setSystem(rob_sys);

  //! Insert Obstacles in the environment 
  ctrl->addObstacle(new teb::Obstacle(-1, 4,  2));     //! X, Y, Radius  
  ctrl->addObstacle(new teb::Obstacle(-1, -1, 2));

 
  //! Set edge weights
  //ctrl->setWeightObstacle(weight_Obstacle[0][0]);
  ctrl->setWeightYawAngle(weight_YawAngle[0][0]);

  // Store void pointers in simulinks work vector for persistent variables.
  ssGetPWork(S)[0] = (void*) solver; // solver
  ssGetPWork(S)[1] = (void*) ctrl; // controller
  ssGetPWork(S)[2] = (void*) rob_sys; // system
  
  // Setup receding horizon T = (n - 1) * dt
  // Common setting: unfixed goal and fixed resolution
  //ctrl->setupHorizon(false, true, T / tebStepSize, tebStepSize);


  // add callback	
  // using namespace std::placeholders;
  // _callback_step_post = std::bind(&teb::RobController<nStates, nControls>::saveVelocityAfterStep, ctrl, _1, _2, _3);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
  // reset trajectroy
  teb::RobControllerCarLike<n_states, n_controls>* ctrl = (teb::RobControllerCarLike<n_states, n_controls>*) ssGetPWork(S)[1];
  ctrl->resetController();
}

#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
	
  using StateVector       = Eigen::Matrix<double, n_states, 1>;	
  using ControlVector     = Eigen::Matrix<double, n_controls, 1>;	
  using StateSequence     = std::deque< teb::StateVertex<n_states>, Eigen::aligned_allocator<teb::StateVertex<n_states>> >;	
  using ControlSequence   = std::deque< teb::ControlVertex<n_controls>, Eigen::aligned_allocator<teb::ControlVertex<n_controls>> >;
  using ObstacleContainer = std::vector<teb::Obstacle*>;

  // Retrieve persistent controller object
  teb::RobControllerCarLike<n_states, n_controls>* ctrl   = (teb::RobControllerCarLike<n_states, n_controls>*) ssGetPWork(S)[1];
  MobileRobotCarLikeSystem<n_states, n_controls>* rob_sys = (MobileRobotCarLikeSystem<n_states, n_controls>*)  ssGetPWork(S)[2];

  // Get input and output parameters
  InputRealPtrsType x0Ptrs = ssGetInputPortRealSignalPtrs(S, 0);
  InputRealPtrsType xfPtrs = ssGetInputPortRealSignalPtrs(S, 1);

  // setup output for controls
  real_T *uPtr = ssGetOutputPortRealSignal(S, 0);
 
  Eigen::Vector2d distanceToObstacle;
  double proj_dist;
  double proj_dist_min = -1;
  
  const ObstacleContainer* obstacles = &ctrl->getObstacles();


#ifndef RTW
  // send some values to MATLAB console
  std::ostringstream strs;
  // start timer
  auto start = std::chrono::high_resolution_clock::now();
#endif
  
  
  ctrl->step(*x0Ptrs, *xfPtrs, uPtr);
	  
  //! Time to collision calculation.

  for (unsigned int i = 0; i < obstacles->size(); ++i)
  {

     distanceToObstacle = { x0Ptrs[0][0] - obstacles->at(i)->getX(), x0Ptrs[0][1] - obstacles->at(i)->getY() };

     proj_dist = (distanceToObstacle.norm() - rob_sys->getRobotLength() - obstacles->at(i)->getRadiusObstacle());
     if (proj_dist_min == -1 || proj_dist_min > proj_dist)
     {
        proj_dist_min = proj_dist;
     }
   }

#ifndef RTW
  strs << "distance: " << proj_dist_min << "\n";
  // mexPrintf(strs.str().c_str());

  // clear stringstream
  strs.str(std::string()); 
  strs.clear();
#endif
	
  StateSequence state_seq  = ctrl->stateSequence();
  ControlSequence ctrl_seq = ctrl->controlSequence();

#ifndef RTW			

   double runtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();

   if (runtime > runtime_max)   
      runtime_max = runtime;   
    
   strs << "Runtime: " << runtime << " ms / max: " << runtime_max << " ms\n";
   mexPrintf(strs.str().c_str());

   // clear stringstream
   strs.str(std::string());
   strs.clear();

#endif

   // setup output for teb 
   real_T *teb_x     = ssGetOutputPortRealSignal(S, 1);
   real_T *teb_y     = ssGetOutputPortRealSignal(S, 2);   
   real_T *teb_v     = ssGetOutputPortRealSignal(S, 5);
   real_T *teb_omega = ssGetOutputPortRealSignal(S, 6);

   // read teb   
   for (unsigned int i = 0; i < state_seq.size(); ++i)
   {
      teb::StateVertex<n_states>* sample_i          = static_cast<teb::StateVertex<n_states>*>(& state_seq.at(i));
      teb::ControlVertex<n_controls>* ctrl_sample_i = static_cast<teb::ControlVertex<n_controls>*>(&ctrl_seq.at(i));

      teb_x[i]     = sample_i->states()[0];
      teb_y[i]     = sample_i->states()[1];      
      teb_v[i]     = ctrl_sample_i->controls()[0];
      teb_omega[i] = ctrl_sample_i->controls()[1];

   }

   // Call user-defined function if desired
   // if (_callback_step_post) _callback_step_post(ctrl, int_sys, nullptr);    
}

static void mdlTerminate(SimStruct *S)
{
   // Destroy persistent objects
   teb::BaseSolver* solver = (teb::BaseSolver*) ssGetPWork(S)[0];
   teb::RobControllerCarLike<n_states, n_controls>* ctrl   = (teb::RobControllerCarLike<n_states, n_controls>*) ssGetPWork(S)[1];
   MobileRobotCarLikeSystem<n_states, n_controls>* rob_sys = (MobileRobotCarLikeSystem<n_states, n_controls>*) ssGetPWork(S)[2];
   delete solver;
   delete ctrl;
   delete rob_sys;
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif

#ifdef __cplusplus
} // end of extern "C" scope
#endif