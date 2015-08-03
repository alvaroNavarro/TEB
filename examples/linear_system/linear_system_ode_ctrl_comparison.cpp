#include "linear_system_ode.h"


int main()
{
  
    const int p = 2; // number of states
    // ode: a_2 * xddot + a_1 * xdot + a_0 * x = b * u;
    const double coeff_a[] {1, -0.1, 1}; // a_2, a_1, a_0
    const double coeff_b  = 1; // no derivatives of the input supported atm.
    
    double x0[] = {0, 0}; // start state
    double xf[] = {1, 0}; // goal state
    
    // System specific settings
    LinearSystemODE<p> system;
    system.setODECoefficients(coeff_a, coeff_b);
    
    
    // Setup controller 1
    teb::SolverLevenbergMarquardtEigenSparse solver1;
    teb::TebController<p,1> teb(&solver1);
    
    teb.setSystemDynamics(&system);
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1.0, 1.0);
    
    
    
    // Setup controller 2
    teb::Config mpc_cfg;
    mpc_cfg.optim.solver.lsq.weight_equalities = 4;
    teb::SolverLevenbergMarquardtEigenSparse solver2;
    teb::TebController<p,1> mpc(&mpc_cfg, &solver2);
    
    mpc.setSystemDynamics(&system);
    
    // Setup receding horizon T = n * dt  = 10 * 0.1
    // Common setting: unfixed goal and fixed resolution
    mpc.setupHorizon(false, true, 10, 0.1);
    
    mpc.activateObjectiveQuadraticForm(1, 0.1, 100); // Parameters: Q, R, Qf
    mpc.activateControlBounds(0, -1.0, 1.0);

    
    
    
    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(system,&plotter);
    // Set simulation sample time
    sim.setSampleTime(0.05);
    
    //sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
 
    // Create container storing the controllers that should be compared
    std::vector<std::pair<teb::BaseController*, std::string>> controllers;
    controllers.emplace_back(&teb, "TEB - Time Optimal");
    controllers.emplace_back(&mpc, "MPC - Quadratic Form");

    // Closed-loop sim
    sim.simClosedLoop(controllers, x0, xf, 8);
    
    // Open-loop sim
    // create new figure with id 1
    plotter.switchWindow(1,true);
    sim.simOpenLoop(controllers, x0, xf);
    
    return 0;
}
