#include "gtest/gtest.h"
#include <teb_package/base/teb_controller.h>
#include <teb_package/base/common_teb_edges.h>
#include "integrator_system.h"


class TEBGraphTestFixture : public testing::Test {
protected: 
	// virtual void SetUp() will be called before each test is run.  You
	// should define it if you need to initialize the varaibles.
	// Otherwise, this can be skipped.
	virtual void SetUp()
	{
		// Init trajectory
		teb.initTrajectory(x0, xf, 10);
	}

	// virtual void TearDown() will be called after each test is run.
	// You should define it if there is cleanup work to do.  Otherwise,
	// you don't have to provide it.
	virtual void TearDown() 
	{
 		teb.resetController();
		teb.initTrajectory(x0, xf, 10);
	}

	// Declares the variables your tests want to use.
	static const int p = 2; //! number of states
	teb::TebController<p, 1> teb;
	Eigen::Vector2d x0 = Eigen::Vector2d(0, 0);
	Eigen::Vector2d xf = Eigen::Vector2d(1, -1);
};


// Now we add tests using the fixture from above

// test fixing vertices
TEST_F(TEBGraphTestFixture, graph_fixed_vertices)
{
	// check timediff
	teb.setFixedDt(false);
	EXPECT_FALSE(teb.dt().isFixedAll());
	teb.setFixedDt(true);
	EXPECT_TRUE(teb.dt().isFixedAll());

	// check goal state

	teb.setGoalStatesFixedOrUnfixed(true); 	// goal states are fixed within initOptimization();
	teb.initOptimization();
	EXPECT_TRUE(teb.stateSequence().back().isFixedAll());

	teb.setGoalStatesFixedOrUnfixed(false); // goal states are fixed within initOptimization();
	teb.initOptimization();
	EXPECT_FALSE(teb.stateSequence().back().isFixedAll());

	Eigen::Matrix<bool, p, 1> g_fixed;
	g_fixed[0] = false;
	g_fixed[1] = true;
	teb.setGoalStatesFixedOrUnfixed(g_fixed); // goal states are fixed within initOptimization();
	teb.initOptimization();
	EXPECT_TRUE(teb.stateSequence().back().isFixedAny());
	EXPECT_FALSE(teb.stateSequence().back().isFixedAll());
	EXPECT_FALSE(teb.stateSequence().back().isFixedComp(0));
	EXPECT_TRUE(teb.stateSequence().back().isFixedComp(1));
}


// test fixing vertices after resampling
TEST_F(TEBGraphTestFixture, graph_fixed_vertices_resample)
{
	// first partially fix current goal

	Eigen::Matrix<bool, p, 1> g_fixed;
	g_fixed[0] = false;
	g_fixed[1] = true;
	teb.setGoalStatesFixedOrUnfixed(g_fixed);
	teb.initOptimization(); // goal states are fixed within initOptimization();

	// now resample and check
	teb.resampleTrajectory(15);
	for (unsigned int i = 10; i < 14; ++i)
	{
		EXPECT_FALSE(teb.stateSequence().at(i).isFixedAny());
	}
	EXPECT_EQ(g_fixed, teb.stateSequence().at(14).fixed_states());

	// resample again
	teb.resampleTrajectory(5);
	EXPECT_EQ(g_fixed, teb.stateSequence().back().fixed_states());
}

// test setup horizon function
TEST_F(TEBGraphTestFixture, graph_setup_horizon)
{
	// First assume that the trajectory was initialized before
	teb.setupHorizon(true, true, 15, 0.5);
	teb.initOptimization(); // goal states are fixed within initOptimization();
	EXPECT_TRUE(teb.stateSequence().front().isFixedAll());
	EXPECT_TRUE(teb.stateSequence().back().isFixedAll());
	EXPECT_TRUE(teb.dt().isFixedAll());
	EXPECT_EQ(15, teb.getN());
	EXPECT_EQ(15, teb.getM());
	EXPECT_DOUBLE_EQ(0.5, teb.dt().dt());

	teb.setupHorizon(true, false, 5, -1);
	teb.initOptimization(); // goal states are fixed within initOptimization();
	EXPECT_TRUE(teb.stateSequence().front().isFixedAll());
	EXPECT_TRUE(teb.stateSequence().back().isFixedAll());
	EXPECT_FALSE(teb.dt().isFixedAny());
	EXPECT_EQ(5, teb.getN());
	EXPECT_EQ(5, teb.getM());
	EXPECT_DOUBLE_EQ(teb.cfg->teb.dt_ref, teb.dt().dt());

	teb.setupHorizon(false, false, -1, -1);
	teb.initOptimization(); // goal states are fixed within initOptimization();
	EXPECT_TRUE(teb.stateSequence().front().isFixedAll());
	EXPECT_FALSE(teb.stateSequence().back().isFixedAny());
	EXPECT_FALSE(teb.dt().isFixedAny());
	EXPECT_EQ(5, teb.getN()) << "Since no_sample argument is -1, no change should be made to the number of samples";
	EXPECT_EQ(5, teb.getM()) << "Since no_sample argument is -1, no change should be made to the number of samples";
	EXPECT_DOUBLE_EQ(teb.cfg->teb.dt_ref, teb.dt().dt());

	// Now the trajectory is not initialized before!
	teb.resetController();
	teb.setupHorizon(false, false, -1, -1);
	teb.updateStart(x0);
	teb.updateGoal(xf); // these to functions should initialized the state and control input sequences internally
	teb.initOptimization(); // goal states are fixed within initOptimization();
	EXPECT_TRUE(teb.stateSequence().front().isFixedAll());
	EXPECT_FALSE(teb.stateSequence().back().isFixedAny());
	EXPECT_FALSE(teb.dt().isFixedAny());
	EXPECT_EQ(teb.cfg->teb.n_pre, teb.getN());
	EXPECT_EQ(teb.cfg->teb.n_pre, teb.getM());
	EXPECT_DOUBLE_EQ(teb.cfg->teb.dt_ref, teb.dt().dt());
}


// test search for active vertices
TEST_F(TEBGraphTestFixture, graph_collect_active_vertices)
{
	// start with fixed goal
	teb.setGoalStatesFixedOrUnfixed(true); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	unsigned int expected_no_av = teb.getN() + teb.getM() - 2 + 1 - 1; // fixed start and fixed goal but unfixed dt - fixed last control
	EXPECT_EQ(expected_no_av, teb.graph().activeVertices().size());

	// unfixed goal
	teb.setGoalStatesFixedOrUnfixed(false);
	teb.initOptimization();
	expected_no_av = teb.getN() + teb.getM() - 1 + 1 - 1; // fixed start, unfixed goal but unfixed dt - fixed last control
	EXPECT_EQ(expected_no_av, teb.graph().activeVertices().size());

	// partially unfixed goal
	Eigen::Matrix<bool, p, 1> g_fixed;
	g_fixed[0] = false;
	g_fixed[1] = true;
	teb.setGoalStatesFixedOrUnfixed(g_fixed);
	teb.initOptimization();
	expected_no_av = teb.getN() + teb.getM() - 1 + 1 - 1; // fixed start, partially fixed goal but unfixed dt - fixed last control
	EXPECT_EQ(expected_no_av, teb.graph().activeVertices().size());
}

// test hessian index
TEST_F(TEBGraphTestFixture, graph_active_vertices_hessian_indices)
{
	teb.resampleTrajectory(4);

	// start with fixed goal
	teb.setGoalStatesFixedOrUnfixed(true); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	
	// first active vertex is completely unfixed (start is not included in the set of active vertices)
	// it is the first control input vector that is an unfixed scalar value
	EXPECT_EQ(0, teb.graph().activeVertices().at(0)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(0)->dimensionFree());
	// now second state (completely unfixed - 2 components)
	EXPECT_EQ(1, teb.graph().activeVertices().at(1)->getOptVecIdx());
	EXPECT_EQ(2, teb.graph().activeVertices().at(1)->dimensionFree());
	// now second control input (completly unfixed)
	EXPECT_EQ(3, teb.graph().activeVertices().at(2)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(2)->dimensionFree());
	// now third state (completely unfixed)
	EXPECT_EQ(4, teb.graph().activeVertices().at(3)->getOptVecIdx());
	EXPECT_EQ(2, teb.graph().activeVertices().at(3)->dimensionFree());
	// now third control input (completely unfixed)
	EXPECT_EQ(6, teb.graph().activeVertices().at(4)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(4)->dimensionFree());
	// now there should be the timediff left (since last state and control are fixed)
	EXPECT_TRUE(dynamic_cast<teb::TimeDiff*>(teb.graph().activeVertices().at(5)) != nullptr);
	EXPECT_EQ(7, teb.graph().activeVertices().at(5)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(5)->dimensionFree());
	EXPECT_EQ(6, teb.graph().activeVertices().size());

	// unfixed goal
	teb.setGoalStatesFixedOrUnfixed(false); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	// continue at vertex 5 (that was the timediff before), should be a state now
	EXPECT_EQ(7, teb.graph().activeVertices().at(5)->getOptVecIdx());
	EXPECT_EQ(2, teb.graph().activeVertices().at(5)->dimensionFree());
	// now there should be the timediff left
	EXPECT_TRUE(dynamic_cast<teb::TimeDiff*>(teb.graph().activeVertices().at(6)) != nullptr);
	EXPECT_EQ(9, teb.graph().activeVertices().at(6)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(6)->dimensionFree());
	EXPECT_EQ(7, teb.graph().activeVertices().size());

	// partially fixed goal
	Eigen::Matrix<bool, p, 1> g_fixed;
	g_fixed[0] = false;
	g_fixed[1] = true;
	teb.setGoalStatesFixedOrUnfixed(g_fixed); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	// continue at vertex 5
	EXPECT_EQ(7, teb.graph().activeVertices().at(5)->getOptVecIdx());
	EXPECT_EQ(1, teb.graph().activeVertices().at(5)->dimensionFree()); // a component is fixed now

	// check fixed time diff
	teb.setFixedDt(true);
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	EXPECT_EQ(6, teb.graph().activeVertices().size());

}

// check if EdgeMinimizeTime and EdgePositiveTime can be added
TEST_F(TEBGraphTestFixture, graph_add_time_optimal_edge)
{
	teb.resampleTrajectory(4);

	teb.activateObjectiveTimeOptimal(true);

	teb.setGoalStatesFixedOrUnfixed(true); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().objectives())
	{
		if (dynamic_cast<const teb::EdgeMinimizeTime*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(1, found_edges) << "We expect only a single EdgeMinimizeTime (objective)";

	found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().inequalities())
	{
		if (dynamic_cast<const teb::EdgePositiveTime*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(1, found_edges) << "We expect only a single EdgePositiveTime (inequality constraint)";
}

// check if EdgeQuadraticForm can be added
TEST_F(TEBGraphTestFixture, graph_add_quadratic_form_edge)
{
	teb.resampleTrajectory(4);
	teb.activateObjectiveQuadraticForm(1,1,1,true);

	teb.setGoalStatesFixedOrUnfixed(true); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().objectives())
	{
		if (dynamic_cast<const teb::EdgeQuadraticForm<2,1>*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(3, found_edges) << "We expect 3 EdgeQuadraticForm<2,1> (objective), since the last state is fixed (the first as well, but the first control is free)";

	teb.setGoalStatesFixedOrUnfixed(false); 	// goal states are fixed within initOptimization();
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	found_edges = 0;
	unsigned int found_edges_qf_zero_q = 0;
	for (const teb::EdgeType* edge : teb.graph().objectives())
	{
		if (dynamic_cast<const teb::EdgeQuadraticForm<2, 1>*>(edge) != nullptr) ++found_edges;
		if (dynamic_cast<const teb::EdgeQuadraticForm<2, 0>*>(edge) != nullptr) ++found_edges_qf_zero_q;
	}
	EXPECT_TRUE(found_edges == 3 || found_edges == 4); // depends on whether the last state edge is added with q=0 or not
	EXPECT_TRUE(found_edges_qf_zero_q == 0 || found_edges_qf_zero_q == 1);
}

// check if EdgeControlBounds can be added
TEST_F(TEBGraphTestFixture, graph_add_control_bounds_edge)
{
	teb.resampleTrajectory(4);
	teb.activateControlBounds(0, -1, 2);

	teb.setGoalStatesFixedOrUnfixed(false); 
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().inequalities())
	{
		if (dynamic_cast<const teb::EdgeControlBounds<1>*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(3, found_edges) << "We expect 3 EdgeControlBounds<1> (inequality_constraints)";
}

// check if EdgeStateBounds can be added
TEST_F(TEBGraphTestFixture, graph_add_state_bounds_edge)
{
	teb.resampleTrajectory(4);
	teb.activateStateBounds(Eigen::Vector2d(-1,-1),Eigen::Vector2d(1,1));

	teb.setGoalStatesFixedOrUnfixed(false);
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().inequalities())
	{
		if (dynamic_cast<const teb::EdgeStateBounds<2>*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(3, found_edges) << "We expect 4 EdgeStateBounds<2> (inequality_constraints)";
}

// check if EdgeSystemDynamics can be added
TEST_F(TEBGraphTestFixture, graph_add_system_dynamics_edge)
{
	teb.resampleTrajectory(4);

	IntegratorSystem<2> system;
	teb.setSystemDynamics(&system);

	teb.setGoalStatesFixedOrUnfixed(false);
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int found_edges = 0;
	for (const teb::EdgeType* edge : teb.graph().equalities())
	{
		if (dynamic_cast<const teb::EdgeSystemDynamics<2,1>*>(edge) != nullptr) ++found_edges;
	}
	EXPECT_EQ(3, found_edges) << "We expect 3 EdgeSystemDynamics<2,1> (equality_constraints)";
}

// check if graph is marked as modified after changes
TEST_F(TEBGraphTestFixture, graph_modified_flag_check)
{

	// should be true after initialization
	EXPECT_TRUE(teb.graph().isGraphModified());
	// reset flag
	teb.graph().notifyGraphModified(false);
	EXPECT_FALSE(teb.graph().isGraphModified());

	// check update start
	teb.updateStart(x0);
	EXPECT_TRUE(teb.graph().isGraphModified());
	teb.graph().notifyGraphModified(false);

	// check update goal
	teb.updateGoal(xf);
	EXPECT_TRUE(teb.graph().isGraphModified());
	teb.graph().notifyGraphModified(false);

	// check resampling
	teb.resampleTrajectory(5);
	EXPECT_TRUE(teb.graph().isGraphModified());
	teb.graph().notifyGraphModified(false);

	// check reset
	teb.resetController();
	EXPECT_TRUE(teb.graph().isGraphModified());
	teb.graph().notifyGraphModified(false);
}

// test graph reset
TEST_F(TEBGraphTestFixture, graph_reset)
{
	teb.activateObjectiveTimeOptimal();
	teb.activateControlBounds(0, -1, 1);
	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	EXPECT_FALSE(teb.graph().activeVertices().empty());
	EXPECT_FALSE(teb.graph().objectives().empty());
	EXPECT_FALSE(teb.graph().inequalities().empty());

	teb.graph().clearEdges();
	EXPECT_FALSE(teb.graph().activeVertices().empty());
	EXPECT_TRUE(teb.graph().objectives().empty());
	EXPECT_TRUE(teb.graph().inequalities().empty());

	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	teb.graph().clearActiveVertices();
	EXPECT_TRUE(teb.graph().activeVertices().empty());
	EXPECT_FALSE(teb.graph().objectives().empty());
	EXPECT_FALSE(teb.graph().inequalities().empty());

	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges
	teb.resetController();
	EXPECT_TRUE(teb.graph().activeVertices().empty());
	EXPECT_TRUE(teb.graph().objectives().empty());
	EXPECT_TRUE(teb.graph().inequalities().empty());
}

// test what happens if the teb optim is called multiple times in a row
TEST_F(TEBGraphTestFixture, graph_multiple_init)
{
	teb.resampleTrajectory(5);
	teb.activateObjectiveTimeOptimal();
	teb.activateControlBounds(0, -1, 1);

	IntegratorSystem<2> system;
	teb.setSystemDynamics(&system);

	teb.initOptimization(); // this includes building the hyper-graph, fixing states and collecting active edges

	unsigned int no_active_vertices = (unsigned int) teb.graph().activeVertices().size();
	unsigned int no_objectives = (unsigned int) teb.graph().objectives().size();
	unsigned int no_equalities = (unsigned int) teb.graph().equalities().size();
	unsigned int no_inequalities = (unsigned int) teb.graph().inequalities().size();

	// now init again
	teb.initOptimization();
	EXPECT_EQ(no_active_vertices, teb.graph().activeVertices().size());
	EXPECT_EQ(no_objectives, teb.graph().objectives().size());
	EXPECT_EQ(no_equalities, teb.graph().equalities().size());
	EXPECT_EQ(no_inequalities, teb.graph().inequalities().size());
}