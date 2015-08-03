#include "gtest/gtest.h"
#include "integrator_system.h"

using namespace teb;

class IntegratorSystemControl : public testing::Test {
protected: 

	using SolverPtr = std::unique_ptr<teb::BaseSolver>;

	using SimResultsPtr = std::unique_ptr<SimResults>;

	// virtual void SetUp() will be called before each test is run.  You
	// should define it if you need to initialize the varaibles.
	// Otherwise, this can be skipped.
	virtual void SetUp()
	{
		cfg.teb.teb_iter = 5;
		cfg.optim.solver.solver_iter = 5;
		cfg.optim.solver.lsq.weight_adaptation_factor = 2;
		cfg.optim.solver.lsq.weight_equalities = 2;
		cfg.optim.solver.lsq.weight_inequalities = 2;

		// add multiple solvers to test
		solvers.push_back(SolverPtr(new SolverLevenbergMarquardtEigenDense()));
		solvers.push_back(SolverPtr(new SolverLevenbergMarquardtEigenSparse()));

		// add a controller for each solver
		for (SolverPtr& solver : solvers)
		{
			controllers.emplace_back(&cfg, solver.get());
		}

		// set Integrator system time constant
		system.setTimeConstant(1.0);

		// apply Integrator system dynamics to the controllers
		for (TebController<p, 1>& ctrl : controllers)
		{
			ctrl.setSystemDynamics(&system);

			// set bounds to: -1 <= u_k <= 1
			ctrl.activateControlBounds(0, -1.0, 1.0);
		}

	}

	// virtual void TearDown() will be called after each test is run.
	// You should define it if there is cleanup work to do.  Otherwise,
	// you don't have to provide it.
	virtual void TearDown() 
	{
		// reset controller
		for (TebController<p, 1>& ctrl : controllers)
		{
			ctrl.resetController();
		}
 		
	}

	// Declares the variables your tests want to use.
	static const int p = 2; //! number of states

	Eigen::Vector2d x0 = Eigen::Vector2d(0, 0);
	Eigen::Vector2d xf = Eigen::Vector2d(1, 0);
	teb::Config cfg;

	std::vector<TebController<p, 1>> controllers;
	std::vector<SolverPtr> solvers;

	IntegratorSystem<p> system;
};


// Now we add tests using the trajectory_fixture from above


TEST_F(IntegratorSystemControl, time_optimal_control_opt_only)
{
	// Solve time optimal control problem with fixed endpoint
	for (TebController<p, 1>& ctrl : controllers)
	{
		// fixed goal, unfixed dt, 20 initial states, dt_init = 0.1
		ctrl.setupHorizon(true, false, 20, 0.1);

		// set time optimal control objective
		ctrl.activateObjectiveTimeOptimal(true);

		// solve optimal control problem
		Eigen::Matrix<double, 1, 1> u;
		ctrl.step(x0, xf, u);

		// check first control input
		EXPECT_NEAR(1, u[0], 0.1);

		// check transition time
		EXPECT_NEAR(1.78, (ctrl.getN() - 1)*ctrl.dt().dt(), 0.1);
	}

	// Repeat simulation for negative goal xf
	// Solve time optimal control problem with fixed endpoint
	for (TebController<p, 1>& ctrl : controllers)
	{
		// reset controller
		ctrl.resetController();

		// fixed goal, unfixed dt, 20 initial states, dt_init = 0.1
		ctrl.setupHorizon(true, false, 20, 0.1);

		// set time optimal control objective
		ctrl.activateObjectiveTimeOptimal(true);

		// solve optimal control problem
		Eigen::Matrix<double, 1, 1> u;
		ctrl.step(x0, -xf, u);

		// check first control input
		EXPECT_NEAR(-1, u[0], 0.1) << "Control input determined is not correct";

		// check transition time
		EXPECT_NEAR(1.78, (ctrl.getN() - 1)*ctrl.getDt(), 0.1) << "Transition time required is not correct";
	}
}

TEST_F(IntegratorSystemControl, time_optimal_control_open_loop_sim)
{
	// Solve time optimal control problem with fixed endpoint
	for (TebController<p, 1>& ctrl : controllers)
	{
		// fixed goal, unfixed dt, 20 initial states, dt_init = 0.1
		ctrl.setupHorizon(true, false, 20, 0.1);

		// set time optimal control objective
		ctrl.activateObjectiveTimeOptimal(true);

		// solve optimal control problem
		
		teb::Simulator<p, 1> sim(&ctrl, system);
		sim.setSampleTime(0.1);
		SimResultsPtr results = sim.simOpenLoop(x0.data(), xf.data());
		// check transition time
		EXPECT_NEAR(1.78, results->series.back().time.tail(1)[0], 0.1);
	}
}

TEST_F(IntegratorSystemControl, time_optimal_control_closed_loop_sim)
{
	// Solve time optimal control problem with fixed endpoint
	for (TebController<p, 1>& ctrl : controllers)
	{
		// fixed goal, unfixed dt, 20 initial states, dt_init = 0.1
		ctrl.setupHorizon(true, false, 20, 0.1);

		// set time optimal control objective
		ctrl.activateObjectiveTimeOptimal(true);

		// sim closed loop controller
		teb::Simulator<p, 1> sim(&ctrl, system);
		sim.setSampleTime(0.1);
		SimResultsPtr results = sim.simClosedLoop(x0.data(), xf.data(),2.5); // sim 2 sec
		// check if final state reached
		EXPECT_NEAR(xf[0], results->series.back().states.rightCols(1)(0), 0.1);
		EXPECT_NEAR(xf[1], results->series.back().states.rightCols(1)(1), 0.1);
	}
}

TEST_F(IntegratorSystemControl, quadraticform_mpc_horizon_closed_loop_sim)
{
	// Solve time optimal control problem with fixed endpoint
	for (TebController<p, 1>& ctrl : controllers)
	{
		// unfixed goal, fixed dt dt, 10 initial states, dt_init = 0.1
		ctrl.setupHorizon(false, true, 10, 0.1);

		// set quadratic form objective
		ctrl.activateObjectiveQuadraticForm(1,0.5,100);

		// sim closed loop controller
		teb::Simulator<p, 1> sim(&ctrl, system);
		sim.setSampleTime(0.1);
		SimResultsPtr results = sim.simClosedLoop(x0.data(), xf.data(), 5); // sim 5 sec
		// check if final state reached
		EXPECT_NEAR(xf[0], results->series.back().states.rightCols(1)(0), 0.1);
		EXPECT_NEAR(xf[1], results->series.back().states.rightCols(1)(1), 0.1);
	}
}

