#include "rocket_system.h"


int main()
{
    const int p = 3; // number of states
    
    // Change default config, since default soft constraint weights are too low.
    teb::Config cfg;
    cfg.optim.solver.lsq.weight_inequalities = 50;
    cfg.optim.solver.lsq.weight_adaptation_factor = 1.1;
    cfg.optim.solver.lsq.soft_constr_epsilon = 0; //! TODO: nothing happens after changing...
    
    // Define start and goal
    double x0[] = {0,0,1}; // start state
    double xf[] = {10,0,0.5}; // goal state
    bool goal_fixed[] = {true,true,false}; // mass is not fixed
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&cfg, &solver);
    
    // System specific settings
    FreeSpaceRocketSystem system;
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1.1, 1.1);
    teb.activateStateBounds(1,-0.5,1.7);
    teb.setGoalStatesFixedOrUnfixed(goal_fixed);
    
    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(&teb,system,&plotter);

    // Set simulation sample time
    sim.setSampleTime(-1); // Inherit from TEB (asynchronous control)
    
    // Simulate 9 seconds
    sim.simOpenAndClosedLoop(x0,xf,9);
    
    
    return 0;
}
