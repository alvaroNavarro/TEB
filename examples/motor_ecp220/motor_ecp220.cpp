#include "motor_ecp220.h"


int main()
{
    teb::Config cfg;
    cfg.teb.teb_iter = 10;
    cfg.optim.solver.solver_iter = 10;
    //cfg.teb.n_pre = 30;
    
    //! Define start and goal
    double x0[] = {0,0,0}; //! start state
    double xf[] = {6*PI,0,0}; //! goal state
    bool goal_fixed[] = {true,true,false}; //! current is not fixed
    double u; //! store control
    
    //! Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<3,1> teb(&cfg,&solver);
    
    //! System specific settings
    MotorECP220 system;
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -0.4, 0.4);
    teb.setGoalStatesFixedOrUnfixed(goal_fixed);
    
    //! Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf, &u);
    STOP_TIMER("TEB optimization loop");
    
    //! Print determined control for the current sampling interval
	PRINT_INFO("Control u: " << u);
    
    //! Plot states and control input
    teb::TebPlotter plotter;
    plotter.plotTEB(teb);
    
    return 0;
}
