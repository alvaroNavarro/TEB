#include "gtest/gtest.h"
#include <teb_package/base/teb_vertices.h>
#include <teb_package/base/common_teb_edges.h>
#include <teb_package/solver/solver_levenbergmarquardt_eigen_sparse.h>
#include <teb_package/solver/solver_levenbergmarquardt_eigen_dense.h>
#include <teb_package/solver/solver_sqp_dense.h>

using namespace teb;

class SolverTestFixture : public testing::Test {
protected:

	using SolverPtr = std::unique_ptr<teb::BaseSolver>;

	// virtual void SetUp() will be called before each test is run.  You
	// should define it if you need to initialize the varaibles.
	// Otherwise, this can be skipped.
	virtual void SetUp()
	{
		cfg.optim.solver.solver_iter = 5;
		cfg.optim.solver.lsq.weight_adaptation_factor = 2;
		cfg.optim.solver.lsq.weight_equalities = 2;
		cfg.optim.solver.lsq.weight_inequalities = 2;

		// add multiple solvers to test
		solvers.push_back( SolverPtr(new SolverLevenbergMarquardtEigenDense()));
		solvers.push_back( SolverPtr(new SolverLevenbergMarquardtEigenSparse()));

		// TODO: ADD MORE SOLVERS: E.G. SolverSQP, but it is not ready atm

		// set config
		for (SolverPtr& solver : solvers) solver->setConfig(&cfg);

	}

	// virtual void TearDown() will be called after each test is run.
	// You should define it if there is cleanup work to do.  Otherwise,
	// you don't have to provide it.
	virtual void TearDown()
	{
		graph.clearGraph(); // the memory of all edges is freed as well
	}

	Config cfg;
	HyperGraph graph;

	std::vector<SolverPtr> solvers;
};


TEST_F(SolverTestFixture, solve_simple_unconstr_problem)
{

	// Add a single variable
	StateVertex<1> x;
    x.setOptVecIdx(0); // set index of the overall optimization vector

	// Create edge for minimizing x^2
    using MyEdgeT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR_SQUARED, StateVertex<1>>; // LINEAR_SQUARED functions will by squared by all solvers
    auto fun = [] (MyEdgeT::VertexContainer& vertices) {return vertices.at(0)->getData(0);}; // Note, we define only x
    MyEdgeT* my_edge = new MyEdgeT(fun, x);
    
	// add everything to the graph
	graph.addActiveVertex(&x);
	graph.addEdgeObjective(my_edge);

	// now solve the problem with different solvers
	unsigned int solver_no = 0;
	for (SolverPtr& solver : solvers)
	{
		// set initial values
		x.setStates(10); // intitialize value
		solver->solve(&graph);
		EXPECT_NEAR(0, x.states()[0], 1e-5) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
		++solver_no;
	}

}

TEST_F(SolverTestFixture, solve_simple_unconstr_problem_offset)
{
	// Add a single variable
	StateVertex<1> x;
    x.setOptVecIdx(0); // set index of the overall optimization vector

	// Create edge for minimizing (x+5)^2
    using MyEdgeT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR_SQUARED, StateVertex<1>>; // LINEAR_SQUARED functions will by squared by all solvers
    auto fun = [] (MyEdgeT::VertexContainer& vertices) {return vertices.at(0)->getData(0) + 5;}; // Note, we define only (x+5)
    MyEdgeT* my_edge = new MyEdgeT(fun, x);
	
	// add everything to the graph
	graph.addActiveVertex(&x);
	graph.addEdgeObjective(my_edge);

	// now solve the problem with different solvers
	unsigned int solver_no = 0;
	for (SolverPtr& solver : solvers)
	{
		x.setStates(10); // intitialize value
		solver->solve(&graph);
		EXPECT_NEAR(-5, x.states()[0], 1e-5) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
		++solver_no;
	}
}

TEST_F(SolverTestFixture, solve_rosenbrock_unconstr)
{
    // Add variables
    StateVertex<1> x1;
    StateVertex<1> x2;
    x1.setOptVecIdx(0); // set index of the overall optimization vector
    x2.setOptVecIdx(1); // set index of the overall optimization vector
    
    // Create edges for minimizing 0.5 * (100*(x2-x1^2)^2 + (1-x1)^2 ) => f1^2 + f2^2
    
    // start with f1
    using EdgeFun1T = EdgeGenericScalarFun<FUNCT_TYPE::NONLINEAR_SQUARED, StateVertex<1>, StateVertex<1>>; // xxx_SQUARED functions will by squared by all solvers
    auto fun1 = [] (EdgeFun1T::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(1)->getData(0);
        return sqrt(100)*(x2-x1*x1);
    };
    EdgeFun1T* edge_fun1 = new EdgeFun1T(fun1, x1, x2);
    
    // now create f2
    using EdgeFun2T = EdgeGenericScalarFun<FUNCT_TYPE::NONLINEAR_SQUARED, StateVertex<1>>;
    auto fun2 = [] (EdgeFun2T::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        return 1-x1;
    };
    EdgeFun2T* edge_fun2 = new EdgeFun2T(fun2, x1);

    // add everything to the graph
    graph.addActiveVertex(&x1);
    graph.addActiveVertex(&x2);
    graph.addEdgeObjective(edge_fun1);
    graph.addEdgeObjective(edge_fun2);
    
    // now solve the problem with different solvers
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x1.setStates(5); // intitialize value x1
        x2.setStates(-5); // intitialize value x2
        solver->solve(&graph);
        EXPECT_NEAR(1, x1.states()[0], 1e-3) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(1, x2.states()[0], 1e-3) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
}



TEST_F(SolverTestFixture, solve_betts_fun_constr)
{
    // Add variables
    StateVertex<1> x1;
    StateVertex<1> x2;
    x1.setOptVecIdx(0); // set index of the overall optimization vector
    x2.setOptVecIdx(1); // set index of the overall optimization vector
    
    // Create edges for minimizing 0.01 * x1^2 + x2^2 - 100 = f1^2 + f^2  s.t. 2<=x1<=50, -50<=x2<=50, 10*x1-x2>=10
    
    // start with f1
    using EdgeFunT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR_SQUARED, StateVertex<1>>; // xxx_SQUARED functions will by squared by all solvers
    auto fun1 = [] (EdgeFunT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        return sqrt(0.01)*x1;
    };
    EdgeFunT* edge_fun1 = new EdgeFunT(fun1, x1);
    
    // now create f2
    auto fun2 = [] (EdgeFunT::VertexContainer& vertices)
    {
        const double& x2 = vertices.at(0)->getData(0);
        return x2;
    };
    EdgeFunT* edge_fun2 = new EdgeFunT(fun2, x2);
    
    // create bound on x1 and x2
    using EdgeBoundT = BoundConstraint<BOUND_TYPE::LOWERUPPER, StateVertex<1>, BOUND_VARS::SINGLE,0>;
    EdgeBoundT* edge_bound1 = new EdgeBoundT(x1);
    edge_bound1->setBounds(2, 50);
    EdgeBoundT* edge_bound2 = new EdgeBoundT(x2);
    edge_bound2->setBounds(-50,50);
    
    // create linear inequality constraint 10*x1-x2>=10 -> 10*x1-x2-10>=0 -> x2-10*x1+10<=0
    using EdgeLinIneqT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR, StateVertex<1>, StateVertex<1>>;
    auto linIneqFun = [] (EdgeLinIneqT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(1)->getData(0);
        return x2 - 10*x1 + 10; // c(x)<=0 convention
    };
    EdgeLinIneqT* edge_lin_ineq = new EdgeLinIneqT(linIneqFun,x1,x2);
    
    // add everything to the graph
    graph.addActiveVertex(&x1);
    graph.addActiveVertex(&x2);
    graph.addEdgeObjective(edge_fun1);
    graph.addEdgeObjective(edge_fun2);
    graph.addEdgeInequality(edge_bound1);
    graph.addEdgeInequality(edge_bound2);
    graph.addEdgeInequality(edge_lin_ineq);
    
    // now solve the problem with different solvers (starting from an infeasible point)
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x1.setStates(-1); // intitialize value x1 (infeasible point)
        x2.setStates(-1); // intitialize value x2 (infeasible point)
        solver->solve(&graph);
        EXPECT_NEAR(2, x1.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(0, x2.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
    
    // now solve the problem with different solvers (starting from an feasible point)
    solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x1.setStates(5); // intitialize value x1
        x2.setStates(-5); // intitialize value x2
        solver->solve(&graph);
        // I reduced the precision for the check since the number of solver iterations is fixed
        EXPECT_NEAR(2, x1.states()[0], 1e-1) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(0, x2.states()[0], 1e-1) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
}


TEST_F(SolverTestFixture, solve_betts_fun_constr_single_vec)
{
    // Add variables
    StateVertex<2> x;
    x.setOptVecIdx(0); // set index of the overall optimization vector
    
    // Create edges for minimizing 0.01 * x1^2 + x2^2 - 100 = f1^2 + f^2  s.t. 2<=x1<=50, -50<=x2<=50, 10*x1-x2>=10
    
    // start with objective
    using EdgeFunT = EdgeGenericVectorFun<2, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<2>>; // xxx_SQUARED functions will by squared by all solvers
    auto fun = [] (EdgeFunT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(0)->getData(1);
        return Eigen::Vector2d( sqrt(0.01)*x1, x2 );
    };
    EdgeFunT* edge_fun = new EdgeFunT(fun, x);
    
    // create bound on x
    using EdgeBoundT = BoundConstraint<BOUND_TYPE::LOWERUPPER, StateVertex<2>, BOUND_VARS::ALL>;
    EdgeBoundT* edge_bound = new EdgeBoundT(x);
    Eigen::Vector2d lb(2,-50);
    Eigen::Vector2d ub(50,50);
    edge_bound->setBounds(lb,ub);
    //edge_bound->setBounds({2,-50}, {50,50});

    // create linear inequality constraint 10*x1-x2>=10 -> 10*x1-x2-10>=0 -> x2-10*x1+10<=0
    using EdgeLinIneqT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR, StateVertex<2>>;
    auto linIneqFun = [] (EdgeLinIneqT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(0)->getData(1);
        return x2 - 10*x1 + 10; // c(x)<=0 convention
    };
    EdgeLinIneqT* edge_lin_ineq = new EdgeLinIneqT(linIneqFun,x);
    
    // add everything to the graph
    graph.addActiveVertex(&x);
    graph.addEdgeObjective(edge_fun);
    graph.addEdgeInequality(edge_bound);
    graph.addEdgeInequality(edge_lin_ineq);
    
    Eigen::Vector2d x0(-1,-1);
    
    // now solve the problem with different solvers (starting from an infeasible point)
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x.setStates(x0); // intitialize vector x (infeasible point)
        solver->solve(&graph);
        EXPECT_NEAR(2, x.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(0, x.states()[1], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
    
    x0[0] = 5;
    x0[1] = -5;
    
    // now solve the problem with different solvers (starting from an feasible point)
    solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x.setStates(x0); // intitialize vector x
        solver->solve(&graph);
        // I reduced the precision for the check since the number of solver iterations is fixed
        EXPECT_NEAR(2, x.states()[0], 1e-1) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(0, x.states()[1], 1e-1) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
}


TEST_F(SolverTestFixture, solve_vec_fun_constr_eq_ineq)
{
    // We need to increase the weights in order to get a precision up to 10e-2
    cfg.optim.solver.lsq.weight_equalities = 500;
    cfg.optim.solver.lsq.weight_inequalities = 500;
    
    // Add variables
    StateVertex<3> x;
    x.setOptVecIdx(0); // set index of the overall optimization vector
    
    // Create edges for minimizing x1^2 + x2^2 + x3^2 s.t. x1>2, x3>-1 ,x2*x1=2
    
    // start with objective
    using EdgeFunT = EdgeGenericVectorFun<3, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<3>>; // xxx_SQUARED functions will by squared by all solvers
    auto fun = [] (EdgeFunT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(0)->getData(1);
        const double& x3 = vertices.at(0)->getData(2);
        return Eigen::Vector3d( x1, x2, x3 );
    };
    EdgeFunT* edge_fun = new EdgeFunT(fun, x);
    
    // create bound on x
    using EdgeBoundT = BoundConstraint<BOUND_TYPE::LOWER, StateVertex<3>, BOUND_VARS::ALL>;
    EdgeBoundT* edge_bound = new EdgeBoundT(x);
    Eigen::Vector3d lb(2,-1,-1);
    edge_bound->setBounds(lb);
    
    // create equality constraint
    using EdgeEqT = EdgeGenericScalarFun<FUNCT_TYPE::NONLINEAR_ONCE_DIFF, StateVertex<3>>;
    auto edgeEqFun = [] (EdgeEqT::VertexContainer& vertices)
    {
        const double& x1 = vertices.at(0)->getData(0);
        const double& x2 = vertices.at(0)->getData(1);
        return x2*x1 - 2; // c(x)=0 convention
    };
    EdgeEqT* edge_lin_eq = new EdgeEqT(edgeEqFun,x);
    
    // add everything to the graph
    graph.addActiveVertex(&x);
    graph.addEdgeObjective(edge_fun);
    graph.addEdgeInequality(edge_bound);
    graph.addEdgeEquality(edge_lin_eq);
    
    Eigen::Vector3d x0(1, 1.5, 5);
    
    // now solve the problem with different solvers (starting from an infeasible point w.r.t. inequalities)
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x.setStates(x0);
        solver->solve(&graph);
        EXPECT_NEAR(2, x.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(1, x.states()[1], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(0, x.states()[2], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
}

TEST_F(SolverTestFixture, solve_with_fixed_states)
{
    // We need to increase the weights in order to get a precision up to 10e-2
    cfg.optim.solver.lsq.weight_equalities = 500;
    cfg.optim.solver.lsq.weight_inequalities = 500;
    
    // Add variables
    StateVertex<2> x1;
    StateVertex<2> x2;
    
    // fix components
    x1.setFixedState(0, true);
    x2.setFixedState(1, true);
    
    x1.setOptVecIdx(0); // set index of the overall optimization vector
    x2.setOptVecIdx(1); // set index of the overall optimization vector (note, x1 has only 1 free var)
    
    // Create edges for minimizing x1_1^2 + x1_2^2 + x2_1^2 + x2_2^2 s.t. x1(1)>1, x1(2)>2, x2(1)=-1 and x1(1) fixed at x01(1), x2(2) fixed x02(2)
    
    // start with f1
    using EdgeFunT = EdgeGenericVectorFun<4, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<2>, StateVertex<2>>; // xxx_SQUARED functions will by squared by all solvers
    auto fun = [] (EdgeFunT::VertexContainer& vertices)
    {
        const double& x1_1 = vertices.at(0)->getData(0);
        const double& x1_2 = vertices.at(0)->getData(1);
        const double& x2_1 = vertices.at(1)->getData(0);
        const double& x2_2 = vertices.at(1)->getData(1);
        return Eigen::Vector4d( x1_1, x1_2, x2_1, x2_2 );
    };
    EdgeFunT* edge_fun = new EdgeFunT(fun, x1, x2);
    
    // create bound on x
    using EdgeBound_T = BoundConstraint<BOUND_TYPE::LOWER, StateVertex<2>, BOUND_VARS::ALL>;
    EdgeBound_T* edge_bound = new EdgeBound_T(x1);
    Eigen::Vector2d lb(1,2);
    edge_bound->setBounds(lb);
    
    // create equality constraint
    using EdgeEqT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR, StateVertex<2>>;
    auto edgeEqFun = [] (EdgeEqT::VertexContainer& vertices)
    {
        const double& x2_1 = vertices.at(0)->getData(0);
        return x2_1 + 1; // c(x)=0 convention
    };
    EdgeEqT* edge_lin_eq = new EdgeEqT(edgeEqFun,x2);
    
    // add everything to the graph
    graph.addActiveVertex(&x1);
    graph.addActiveVertex(&x2);
    graph.addEdgeObjective(edge_fun);
    graph.addEdgeInequality(edge_bound);
    graph.addEdgeEquality(edge_lin_eq);
    
    Eigen::Vector2d x01(3, 5);
    Eigen::Vector2d x02(1, 2);
    
    // now solve the problem with different solvers (starting from an infeasible point w.r.t. inequalities)
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        x1.setStates(x01); // intitialize vector x1
        x2.setStates(x02); // intitialize vector x2
        solver->solve(&graph);
        EXPECT_NEAR(3, x1.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(2, x1.states()[1], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(-1, x2.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        EXPECT_NEAR(2, x2.states()[1], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
}



TEST_F(SolverTestFixture, solve_obj_edge_set_vertices_at_runtime)
{
    // We check, if the dynamic version of BaseEdge works with vertices added at runtime.
    
    // Let us create a problem with 25 vertices (1D states)
    std::vector<StateVertex<1>> states;
    states.resize(25);
    
    // The goal is to minimize the square of the sum of all values ((sum(vertices) - 1)^2 = (x1+x2+x3+...+xn - 1)^2
    // NOTE: this problem is illposed, since we have infinite degrees of freedom to solve the problem.
    //       The levenberg marquardt algorithm regularizes the problem and in addition we get the same gradient for all values, therefore we converge against xi = 1/25 (equidistant)
    //       But to make the problem solvable for all solvers, we add bounds xi>1/25 -> in that case we obtain a unique solution
    
    // Now we create an edge without specifying the actual vertices
    using EdgeDynFunT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR_SQUARED>; // xxx_SQUARED functions will by squared by all solvers
    // The objective function creates the sum over all vertices
    auto fun = [] (EdgeDynFunT::VertexContainer& vertices)
    {
        double sum = 0;
        for (const VertexType* vertex : vertices)
        {
            sum += vertex->getData(0);
        }
        return sum - 1;
    };
    EdgeDynFunT* edge = new EdgeDynFunT(fun);
    
    // Now we can add vertices at runtime, but do not forget to resize first
    edge->resizeVertexContainer(25);
    unsigned int id=0;
    for (StateVertex<1>& state : states)
    {
        edge->setVertex(id, &state);
        
        // Add the states to the hyper graph and set the indices inside the overall optimizaiton vector b=[x1,x2,...,xn]^T
        graph.addActiveVertex(&state);
        state.setOptVecIdx(id); // we increase the index by one since each state vertex has the dimension 1 and is unfixed (we exploit using the ID here, but never use the id in arbitrary cases)
        ++id;
    }
    
    // Now add the single edge.
    // Note, the optimizaiton problem is now completely dense!
    // If one create an edge, that requires a large mount of vertices in comparison to the total number of vertices involved, the problem is dense.
    // Here this edge depends on all vertices!
    graph.addEdgeObjective(edge);
    
    // add bounds to force the problem towards a unique solution
    // create bound on x
    using EdgeBound_T = BoundConstraint<BOUND_TYPE::LOWER, StateVertex<1>, BOUND_VARS::SINGLE,0>;
    for (StateVertex<1>& state : states)
    {
        EdgeBound_T* edge_bound = new EdgeBound_T(state);
        edge_bound->setBounds(1./25.);
        graph.addEdgeInequality(edge_bound);
    }
    
    // now solve the problem with different solvers (starting from an infeasible point w.r.t. inequalities)
    unsigned int solver_no = 0;
    for (SolverPtr& solver : solvers)
    {
        // Initialize the states (set the initial values)
        for (StateVertex<1>& state : states)
            state.setStates(5);
        
        solver->solve(&graph);
        for (StateVertex<1>& state : states)
            EXPECT_NEAR(1./25., state.states()[0], 1e-2) << "Solver with index " << solver_no << " failed. Check vector inside fixture to see the name of the solver";
        ++solver_no;
    }
    
}


