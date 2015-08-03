#include "linear_system_ode.h"


int main()
{
  
    const int p = 2; // number of states
    // ode: a_2 * xddot + a_1 * xdot + a_0 * x = b * u;
    const double coeff_a[] {0.8,0.9,1}; // a_2, a_1, a_0
    const double coeff_b  = 1; // no derivatives of the input supported atm.
    
    double x0[] = {0,0}; // start state
    double xf[] = {1,0}; // goal state
    double u; // store control
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&solver);
    
    // System specific settings
    LinearSystemODE<p> system;
    system.setODECoefficients(coeff_a, coeff_b);
    
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
