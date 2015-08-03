#include "van_der_pol_system.h"


int main()
{
  
    const int p = 2; // number of states
    
    
    // Define start and goal
    double x0[] = {0,0}; // start state
    double xf[] = {1,0}; // goal state
    double u; //! store control
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&solver);
    
    // System specific settings
    VanDerPolSystem system;
    system.setParameters(1.0);
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1.0, 1.0);
    
    // Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf, &u);
    STOP_TIMER("TEB optimization loop");
    
    // Print determined control for the current sampling interval
	PRINT_INFO("Control u: " << u);
    
    // Plot states and control input
    teb::TebPlotter plotter;
    plotter.plotTEB(teb);
    
    return 0;
}
