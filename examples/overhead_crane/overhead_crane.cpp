#include "overhead_crane.h"


int main()
{
  
    const int p = 4; // number of states
    
    teb::Config cfg;
     cfg.optim.solver.lsq.weight_equalities = 50;
     cfg.optim.solver.lsq.weight_inequalities = 5;
     cfg.optim.solver.lsq.weight_adaptation_factor = 2;
    
    // Define start and goal
    double x0[] = {0,0,0,0}; // start state
    double xf[] = {0.5,0,0,0}; // goal state
    double u_min = -2;
    double u_max = 2;
    //double u; //! store control
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&cfg,&solver);
    
    // System specific settings
    OverheadCrane<false,true> system;
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, u_min, u_max);
    //teb.activateStateBounds(0,-1,1);
    
    // Perform open-loop planning
    //START_TIMER;
    //teb.step(x0, xf, &u);
    //STOP_TIMER("TEB optimization loop");
    
    // Print determined control for the current sampling interval
    //std::cout << "Control u: " << u[0] << "," << u[1] << std::endl;
    
    // Plot states and control input
    //teb::TebPlotter plotter;
    //plotter.plotTEB(teb);

    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(&teb,system,&plotter);
    // Set simulation sample time
    //sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
    sim.setSampleTime(0.05);
        
    sim.setControlInputSaturation(&u_min, &u_max);
    
    // Simulate
    sim.simOpenAndClosedLoop(x0,xf,2);
    
    return 0;
}
