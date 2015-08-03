#include "integrator_system.h"

int main()
{
    const int p = 4; //! number of states
    double x0[] = {0,0,0,0}; //! start state
    double xf[] = {1,0,0,0}; //! goal state
    double u = 0; //! store control

    // Setup solver and controller
    //teb::SolverLevenbergMarquardtEigenSparse solver;
    
    
    // test SQP solver
    teb::Config cfg;
    cfg.teb.teb_iter = 1;
    /*
    cfg.optim.solver.nonlin_prog.hessian.hessian_method = teb::HessianMethod::FULL_BFGS;
    cfg.optim.solver.nonlin_prog.hessian.hessian_init = teb::HessianInit::IDENTITY;
    cfg.optim.solver.nonlin_prog.hessian.hessian_init_identity_scale = 0.01;
     */
    teb::SolverSQPLocalDense solver;
    
    
    teb::TebController<p,1> teb(&cfg, &solver);
    
    // System specific settings
    IntegratorSystem<p> system;
    system.setTimeConstant(1.0);
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
	teb.activateControlBounds({-1}, {1});

    // Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf, &u);
    STOP_TIMER("TEB optimization loop");
    
    // Print determined control for the current sampling interval
    PRINT_INFO("Control u: " << u);
    
    // Plot states and control input
    teb::TebPlotter plotter;
    // optional: plotter.setOutputToFile("teb_opt_int",teb::TebPlotter::FileFormat::PNG);
    plotter.plotTEB(teb);
    
    return 0;
}
