#include "van_der_pol_system.h"


int main()
{
  
    const int p = 2; // number of states
        
	VanDerPolSystem system;
	system.setParameters(1.0);

	/*

    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&cfg, &solver);
    
    // System specific settings
    
 
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
	teb.activateControlBounds({ -1 }, { 1 });
    
    // Create simulator
    teb::TebPlotter plotter;
    teb::Simulator<p,1> sim(&teb,system,&plotter);

    // Set simulation sample time
    sim.setSampleTime(0.05);
	sim.simOpenAndClosedLoop({ 0, 0 }, { 1, 0 }, 3, false, true, { 1 }); // Add step response for u_ref = 1 to the results (last argument)

	*/


	// Setup controller 1
	// Setup config
	teb::Config cfg1;
	cfg1.optim.solver.lsq.weight_inequalities = 4;
	cfg1.teb.dt_ref = 0.05; // 0.01
	teb::SolverLevenbergMarquardtEigenSparse solver1;
	teb::TebController<p, 1> teb(&cfg1, &solver1);

	teb.setSystemDynamics(&system);
	teb.activateObjectiveTimeOptimal();
	teb.activateControlBounds({ -1 }, { 1 });



	// Setup controller 2
	teb::Config mpc_cfg;
	mpc_cfg.optim.solver.lsq.weight_equalities = 4;
	teb::SolverLevenbergMarquardtEigenSparse solver2;
	teb::TebController<p, 1> mpc(&mpc_cfg, &solver2);

	mpc.setSystemDynamics(&system);

	// Setup receding horizon T = n * dt  = 10 * 0.1
	// Common setting: unfixed goal and fixed resolution
	//mpc.setupHorizon(false, true, 100, 0.01);
	mpc.setupHorizon(false, true, 50, 0.05);

	mpc.activateObjectiveQuadraticForm(1, 0.1, 100); // Parameters: Q, R, Qf
	mpc.activateControlBounds({ -1 }, { 1 });


	// Create simulator
	teb::TebPlotter plotter;
	teb::Simulator<p, 1> sim(system, &plotter);
	// Set simulation sample time
	sim.setSampleTime(0.05); // 0.01

	// Create container storing the controllers that should be compared
	std::vector<std::pair<teb::BaseController*, std::string>> controllers;
	controllers.emplace_back(&teb, "TEB - Time Optimal");
	controllers.emplace_back(&mpc, "MPC - Quadratic Form (RH, Q=1, R=0.1, Qf=100)");

	// Closed-loop sim
	sim.simClosedLoop(controllers, { 0, 0 }, { 1, 0 }, 8, true, true, { 1 });

	// Open-loop sim
	// create new figure with id 1
	//plotter.switchWindow(1, true);
	//sim.simOpenLoop(controllers, x0, xf);

    return 0;
}
