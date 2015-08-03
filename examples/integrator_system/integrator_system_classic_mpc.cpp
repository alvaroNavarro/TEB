#include "integrator_system.h"

int main()
{
    const int p = 3; //! number of states
    double x0[] = {0,0,0}; //! start state
    double xf[] = {1,0,0}; //! goal state
    //double u; //! store control

    // Setup soft constraint weights
    teb::Config cfg;
    cfg.optim.solver.lsq.weight_equalities = 2;
    cfg.optim.solver.lsq.weight_inequalities = 2;
    cfg.optim.solver.lsq.weight_adaptation_factor = 5;

    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&cfg, &solver);
    
    // Setup receding horizon T = n * dt = 10 * 0.1
    // Common setting: unfixed goal and fixed resolution
    teb.setupHorizon(false, true, 10, 0.1);
    
    // Setup optimization problem
    teb.activateObjectiveQuadraticForm(0.5,0.1,50); // parameters: Q, R, Qf
    teb.activateControlBounds(0, -1.0, 1.0);
    
    IntegratorSystem<p> system;
    system.setTimeConstant(1.0);
    
    teb.setSystemDynamics(&system);

    
    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(&teb, system, &plotter);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);

    // Simulate 12 seconds and save image to file (look in the folder of the binary)
    plotter.setOutputToFile("mpc_control_integrator", teb::TebPlotter::FileFormat::PNG);

	PRINT_INFO("Simulation running...");
    sim.simOpenAndClosedLoop(x0, xf, 12, false, false);
	PRINT_INFO("Simulation finished. Find the output file in your current working folder.");
    
    return 0;
}
