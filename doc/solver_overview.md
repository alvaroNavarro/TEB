Solver Overview			{#solver_overview}
================

The following list contains descriptions and small guidances about the solver backends
implemented or interfaced within this package.


Least-Squares Solver
---------------------

Least-Square solvers minimize only quadratic functions without constraints 
\f$ V^*(\mathcal{B}) = \min f^2(\mathcal{B}) \f$.

Advantages of the least-squares solver are the amazing performance, the robustness (Levenberg-Marquardt) and even after a few iterations,
a suitable solution is found often.
A drawback is the lack of hard constraints.
This implementation transforms equality and inequality constraints automatically into soft constraints (refer to the class description).

Due to the prequisite that only squared function can be minimized, we need to keep the following important definitions/conventions in mind:

- The objective functions are always defined without squaring the values: \f$ f(\mathcal{B}) \f$ rather than \f$ f^2(\mathcal{B}) \f$.
- The solver will square the values internally (this definition is much more efficient than specifying the squared values).
- Constraints are squared internally as well using the soft-constraint approximation.
- To make the MPC optimization problem simultaneously compatible to Non-Linear Program Solvers (see below), the designer of an objective function (given as an edge type for the hyper-graph, see the mobile robot example) can specify some information about the function type (see Enumeration teb::FUNCT_TYPE). This can be helpful for the hessian calculation as well, since a linear function has a zero hessian.
The function types are defined as follow:
	- *Nonlinear*: The function is nonlinear (it will be squared by LeastSquares solver, but not by other solvers)
	- *Nonlinear_Squared*: The function is nonlinear (it will be squared by all solvers)
	- *Nonlinear_Once_Diff*: The hessian of this function is always zero (and it will be squared only by LeastSquare solvers)
	- *Quadratic*: The function will be squared only by LeastSquares solvers (-> Power of 4 !!!) 
	- *Linear*: The function will be squared only by LeastSquares solvers (zero hessian)
	- *LinearSquared*: The function will be squared by all solvers (zero hessian)


**Levenberg-Marquardt Dense** (teb::SolverLevenbergMarquardtEigenDense)

This solver constitutes a robust least-squares optimization algorithm.
The Levenberg-Marquardt algorithm is similar to the one used in \cite kummerle2011.
The underlying linear system is solved using the Eigen framework.

**Levenberg-Marquardt Sparse** (teb::SolverLevenbergMarquardtEigenSparse)

This solver implements the sparse version of the Levenberg-Marquardt algorithm mentioned above.
Jacobians and Hessians are internally treaten as sparse matrices, that are effienctly constructed using the hyper-graph formulation
of the underlying optimization problem.
The resulting sparse linear system is solved using the Eigen framework (sparse modules). If CMake is able to find SuiteSparse/Cholmod libraries (see \ref install),
the cholmod solver is utilized for solving the sparse linear system as long as <code>!defined(FORCE_EIGEN_SOLVER)</code>.


Nonlinear Program Solver
---------------------

**Sequential-Quadratic-Programming (SQP) Dense with qpOASES** (teb::SolverSQPDense)

The solver implementation is still in progress, documantation will be added in the future.


**NLOPT Solver Interface** (teb::SolverNloptPackage)

The solver implementation is still in progress, documantation will be added in the future.



