#include "integrator_system.h"


int main()
{
  
    const int p = 2; // number of states
    double x0[] = {0,0}; // start state
    double xf[] = {1,0}; // goal state
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&solver);
    
    // System specific settings
    IntegratorSystem<p> system;
    teb.setSystemDynamics(&system);
    
    // Optional: smooth control input trajectory
    // with small weighted quadratic cost
    //teb.activateObjectiveQuadraticForm(0,0.1,0);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1.0, 1.0);
    
    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(&teb,system,&plotter);
    // Set simulation sample time
    sim.setSampleTime(0.1);
    // Simulate 3 seconds
    sim.simOpenAndClosedLoop(x0,xf,3);
    
    return 0;
}


