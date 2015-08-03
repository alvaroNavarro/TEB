#include "gtest/gtest.h"
#include <teb_package/base/teb_controller.h>


class TrajectoryTestFixture : public testing::Test {
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


// Now we add tests using the trajectory_fixture from above

// check if initialized cleaning up of the controller is working
TEST_F(TrajectoryTestFixture, reset_controller)
{
	teb.resetController();
	EXPECT_TRUE(teb.stateSequence().empty()); // sate sequence length should be zero now
	EXPECT_TRUE(teb.controlSequence().empty()); // control input sequence length should be zero now
}

// check if the initialized trajectory has the correct number of states
TEST_F(TrajectoryTestFixture, init_trajectory_size)
{
	EXPECT_EQ(10, teb.getN()); // sate sequence length should be zero now
	EXPECT_EQ(10, teb.getM()); // control input sequence length should be zero now
}

// test insertion of new states
TEST_F(TrajectoryTestFixture, trajectory_insert_remove)
{
	// start with zero states and controls
	teb.resetController();

	Eigen::Vector2d x(0.1, 0.2);
	Eigen::Matrix<double, 1, 1> u;
	u[0] = 1.5;

	// now add a state and control input pair at the back.
	teb.pushBackStateControlInputPair(x, u);

	// check new seq lengths
	EXPECT_EQ(1, teb.getN());
	EXPECT_EQ(1, teb.getM());

	// check values
	EXPECT_EQ(x, teb.stateSequence().front().states());
	EXPECT_EQ(u, teb.controlSequence().front().controls());

	// now add a new state and control input pair at the front
	teb.pushFrontStateControlInputPair(2 * x, 2 * u);

	// check new seq lengths
	EXPECT_EQ(2, teb.getN());
	EXPECT_EQ(2, teb.getM());

	// check values
	EXPECT_EQ(2 * x, teb.stateSequence().at(0).states());
	EXPECT_EQ(2 * u, teb.controlSequence().at(0).controls());

	// now add a new state and control input pair to the middle (index 1)
	teb.insertStateControlInputPair(1, 3 * x, 3 * u);

	// check new seq lengths
	EXPECT_EQ(3, teb.getN());
	EXPECT_EQ(3, teb.getM());

	// check values
	EXPECT_EQ(2 * x, teb.stateSequence().at(0).states()) << "State vector values do not match the original ones";
	EXPECT_EQ(2 * u, teb.controlSequence().at(0).controls()) << "Control input vector values do not match the original ones";
	EXPECT_EQ(3 * x, teb.stateSequence().at(1).states()) << "State vector values do not match the original ones";
	EXPECT_EQ(3 * u, teb.controlSequence().at(1).controls()) << "Control input vector values do not match the original ones";
	EXPECT_EQ(x, teb.stateSequence().at(2).states()) << "State vector values do not match the original ones";
	EXPECT_EQ(u, teb.controlSequence().at(2).controls()) << "Control input vector values do not match the original ones";

	// now add a new state and control input pair to the front
	teb.insertStateControlInputPair(0, 4 * x, 4 * u);

	EXPECT_EQ(4 * x, teb.stateSequence().at(0).states());
	EXPECT_EQ(4 * u, teb.controlSequence().at(0).controls());

	// check new seq lengths
	EXPECT_EQ(4, teb.getN());
	EXPECT_EQ(4, teb.getM());

	// check values
	EXPECT_EQ(4 * x, teb.stateSequence().at(0).states());
	EXPECT_EQ(4 * u, teb.controlSequence().at(0).controls());

	// now add a new state and control input pair to the back
	teb.insertStateControlInputPair(teb.getN(), 5 * x, 5 * u);

	EXPECT_EQ(5 * x, teb.stateSequence().back().states());
	EXPECT_EQ(5 * u, teb.controlSequence().back().controls());

	// check new seq lengths
	EXPECT_EQ(5, teb.getN());
	EXPECT_EQ(5, teb.getM());

	// now remove the last state and control input
	teb.removeStateControlInputPair(4);

	// check new seq lengths
	EXPECT_EQ(4, teb.getN());
	EXPECT_EQ(4, teb.getM());

	// check values
	EXPECT_EQ(x, teb.stateSequence().back().states());
	EXPECT_EQ(u, teb.controlSequence().back().controls());

	// now remove the first state and control input
	teb.removeStateControlInputPair(0);

	// check new seq lengths
	EXPECT_EQ(3, teb.getN());
	EXPECT_EQ(3, teb.getM());

	// check values
	EXPECT_EQ(2 * x, teb.stateSequence().front().states());
	EXPECT_EQ(2 * u, teb.controlSequence().front().controls());

	// now remove the middle state and control input
	teb.removeStateControlInputPair(1);

	// check new seq lengths
	EXPECT_EQ(teb.getN(), 2);
	EXPECT_EQ(teb.getM(), 2);

	// check values
	EXPECT_EQ(2 * x, teb.stateSequence().at(0).states());
	EXPECT_EQ(2 * u, teb.controlSequence().at(0).controls());
	EXPECT_EQ(x, teb.stateSequence().at(1).states());
	EXPECT_EQ(u, teb.controlSequence().at(1).controls());
}


// test resampling of the trajectory
TEST_F(TrajectoryTestFixture, trajectory_resampling)
{
	// the current trajectory has 10 samples between {x_s=[0,0], u_s = 0} and {x_f[1, -1], u_f = 0} with an equidistant seperation
	
	// Now resample to 5 states
	teb.resampleTrajectory(5);

	// Check new seq lengths
	EXPECT_EQ(teb.getN(), 5);
	EXPECT_EQ(teb.getM(), 5);

	// Check values
	unsigned int i = 0;
	for (const teb::StateVertex<p>& state : teb.stateSequence())
	{
		EXPECT_DOUBLE_EQ(x0[0] + i*(xf[0] - x0[0]) / 4,  state.states()[0]);
		EXPECT_DOUBLE_EQ(x0[1] + i*(xf[1] - x0[1]) / 4, state.states()[1]);
		++i;
	}
	for (const teb::ControlVertex<1>& control : teb.controlSequence())
	{
		EXPECT_DOUBLE_EQ(0, control.controls()[0]);
	}

	// Now resample to 15 states
	teb.resampleTrajectory(15);

	// Check new seq lengths
	EXPECT_EQ(teb.getN(), 15);
	EXPECT_EQ(teb.getM(), 15);

	// Check values
	i = 0;
	for (const teb::StateVertex<2>& state : teb.stateSequence())
	{
		EXPECT_DOUBLE_EQ(x0[0] + i*(xf[0] - x0[0]) / 14, state.states()[0]);
		EXPECT_DOUBLE_EQ(x0[1] + i*(xf[1] - x0[1]) / 14, state.states()[1]);
		++i;
	}
	for (const teb::ControlVertex<1>& control : teb.controlSequence())
	{
		EXPECT_DOUBLE_EQ(0, control.controls()[0]);
	}


}


