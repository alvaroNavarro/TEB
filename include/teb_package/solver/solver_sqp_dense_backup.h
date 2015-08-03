#ifndef __teb_package__solver_sqp_dense__
#define __teb_package__solver_sqp_dense__

#include <teb_package/solver/base_solver_nonlinear_program_dense.h>

#include <qpOASES.hpp>



namespace teb
{


class SolverSQPDense : public BaseSolverNonlinearProgramDense
{
    
public:
   
    SolverSQPDense() : BaseSolverNonlinearProgramDense() {} //!< Empty Constructor.
        

protected:

    virtual void initSolverWorkspace()
    {
       _A.resize(_equalities_dim + _inequalities_dim, _opt_vec_dim);
       _lb.resize(_A.rows());
       _ub.resize(_A.rows());
       _delta.resize(_opt_vec_dim);
       _qdual.resize(_opt_vec_dim + _A.rows()); // qpOASES uses a dual vector that first stores all state bounds (that are not used here) and afterwards according to matrix A the equality and inequality constraints)
       
       new (&_dmultiplier_eq) Eigen::Map<Eigen::VectorXd>(_qdual.data()+_opt_vec_dim+1, _equalities_dim);  // map to equality lagrange multipliers in the dual solution
       new (&_dmultiplier_ineq) Eigen::Map<Eigen::VectorXd>(_qdual.bottomRows(_inequalities_dim).data(), _inequalities_dim); // map to inequality lagrange multipliers in the dual solution
       
       _equality_values.resize(_equalities_dim);
       _inequality_values.resize(_inequalities_dim);
       
       _merit_grad.resize(_opt_vec_dim);
       
       // Map data
       
       // -ceq <= A*b <= -ceq, since equality constraint: lb = ub here
       // WARNING: we must call ceq(x)*=-1 before passing to the quadratic solver
       //new (&_equality_values) Eigen::Map<Eigen::VectorXd>( _lb.data(), _equalities_dim ); 
       //new (&_equality_values) Eigen::Map<Eigen::VectorXd>( _ub.data(), _equalities_dim );
       
       // c(x)<0 -> c(x_k) + Jc * b < 0  ->  -inf < Jc * b < -c(x_k)   0 <= A*b <= -c, therefore: lb=0, ub = -c
       // WARNING: we must call c(x)*=-1 before passing to the quadratic solver
       //new (&_inequality_values) Eigen::Map<Eigen::VectorXd>( _ub.tail(_inequalities_dim).data(), _inequalities_dim );
       
#ifdef TEST
       _ub.tail(_inequalities_dim).setConstant(INF); // TEST CHANGE HERE
#else
       _ub.tail(_inequalities_dim).setConstant(INF); // TEST CHANGE HERE
#endif
       
       new (&_equality_jacobian) MatMapRowMajor( _A.data(), _equalities_dim, _opt_vec_dim );
       new (&_inequality_jacobian) MatMapRowMajor( _A.bottomRows(_inequalities_dim).data(), _inequalities_dim, _opt_vec_dim );
	
    }
    
    // Actual solve implementation - See BaseSolver.
    virtual bool solveImpl()
    {
      
      initSolverWorkspace(); // TODO: call only if graph is modified, but be careful with multiplier initialization etc.
            
      // initialize lagrange multiplier for equality and inequality constraints
      initializeLagrangeMultiplier(); // TODO: call everytime?
      
      
      
      // Calculate objective and constraint values for the current working state
      buildObjectiveValue();
      buildEqualityConstraintValueVector();
      buildInequalityConstraintValueVector();
#ifdef TEST
      _inequality_values *= -1; // TEST DEBUG
#endif
      
      qpOASES::QProblem qsolver(_opt_vec_dim, _A.rows());
      qsolver.setPrintLevel(qpOASES::PL_NONE);
      
      // linesearch parameter -> cfg
      double alpha = 0.01; // should be in range [0,0.5]
      double beta = 0.5; // should be in range [0,1]
            
      
      for (unsigned int iter=0; iter < 5; ++iter)
      {
	  
	    // Now calculate the gradient, jacobians and hessians
	    // We calculate it here, since the objective and constraint values are calculated 
	    // at the end of the loop and in the initialization above
	    buildObjectiveGradient();
	    buildEqualityConstraintJacobian();
	    buildInequalityConstraintJacobian();
	    if (iter==0)
	    {
	      //calculateLagrangianGradient();
	      //calculateLagrangianHessian();
	      _lagrangian_hessian.fill(0);
	    }
    
	    // solve quadratic program with qpOASES (approx. of the original problem)
	    //int nWSR = 5 * (_opt_vec_dim + _A.rows());
	    int nWSR = 5 * _A.rows();
	    
	    // We need to do some copies here atm.
	    _lb.head(_equalities_dim) = -_equality_values;
	    _ub.head(_equalities_dim) = _lb.head(_equalities_dim);
#ifdef TEST
	    _lb.tail(_inequalities_dim) = -_inequality_values;
#else
	    _lb.tail(_inequalities_dim) = _inequality_values;
#endif
	    qsolver.init( _lagrangian_hessian.data(), _objective_gradient.data(), _A.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
	
	    // get solution of the QPOASES optimization
	    qsolver.getPrimalSolution( _delta.data() );
	    qsolver.getDualSolution( _qdual.data() );
	    
	    std::cout << _delta.sum() << std::endl; // TEST
	    
	    // check convergence
	    //if (_delta.norm() < 1e-3) break;
	    
	    // perform line search to get a delta that actually reduces the merit function
	    
	    // get prameter for the merit function based on old constraint values
#ifdef TEST
	    double rho = 1/(std::max(-(_inequality_values.minCoeff()), _equality_values.lpNorm<Eigen::Infinity>() + 0.01));
#else
	    double rho = 1/(std::max(_inequality_values.maxCoeff(), _equality_values.lpNorm<Eigen::Infinity>() + 0.01)); // TEST
#endif
	    
	    // get merit value for the working state a-priori to solving the quadratic problem (we didn't update anything until now)
	    double merit_derivative; // the derivative of the merit function with respect to the new delta!
	    double merit_old = calculateMerit(rho, &merit_derivative);

	    double step = 1; // start with step width 1, that means we would accept the complete delta of the qproblem and add it to the original values.
	    
	    // backup current solution
	    backupVertices();
	    
	    // apply the new increment step*delta to the vertices
	    applyIncrement(step*_delta);
      
	    double norm_eq = _equality_values.norm(); // TODO: norm calculation redundant, used for the workaround before
	    
	    // Calculate new objective and constraint values and afterwards recalculate merit (with same rho as before)
	    buildObjectiveValue();
	    buildEqualityConstraintValueVector();
	    buildInequalityConstraintValueVector();
#ifdef TEST
		_inequality_values *= -1; // TEST DEBUG
#endif
	    double merit_new = calculateMerit(rho);
	    
	    // Workaround: sometimes the first step is not accepted, if obejctive costs are too high
	    if (_equality_values.norm()/norm_eq < 0.5) merit_new = 0; // TODO: redundant as well, maybe better workaround or store norm
	    
	    // do actual line search now (backtracking strategy)
	    while (merit_new > merit_old + alpha*step*merit_derivative)
	    {
		// decrease step width by factor beta (<1)
	        step *= beta;
		if (step<1e-4)
		{
		  PRINT_DEBUG("SolverSQPDense(): Trying to perform linesearch, but the step is still decreasing without reducing merit. Maybe still converged?");
		  break;
		}
		
		// Apply new increment to the vertices, but restore old values before // TODO: Can we achieve the same more efficiently?
		restoreVerticesButKeepBackup();
		applyIncrement(step*_delta);
		
		// recalculate new objective, cosntraint values and merit
		buildObjectiveValue();
		buildEqualityConstraintValueVector();
		buildInequalityConstraintValueVector();
#ifdef TEST
 		_inequality_values *= -1; // TEST DEBUG
#endif
		merit_new = calculateMerit(rho);
	    }
	    
	    // update multipliers
	    _multiplier_eq += step * _dmultiplier_eq;
	    _multiplier_ineq += step * _dmultiplier_ineq;
	    
	    // discard backup
	    discardBackupVertices();
	    
      }
      
      // TODO: discard backups if for-loop was leaved before finishing the loop
      
      return true;
    }
    
    
    
        
    double calculateMerit(double rho, double* merit_derivative_out = nullptr) const
    {
	assert(rho!=0);
	// exact penalty merit function (M = f + 1/rho ||ceq|| + 1/rho ||max(0,c)||)
#ifdef TEST
	double ineq_part = 1/rho * (-_inequality_values).cwiseMax(0).lpNorm<1>(); // TEST
#else
	double ineq_part = 1/rho *_inequality_values.cwiseMax(0).lpNorm<1>(); 
#endif
	double eq_part = 1/rho * _equality_values.lpNorm<1>();
	if (merit_derivative_out) *merit_derivative_out = _objective_gradient.dot(_delta) - eq_part - ineq_part;
        return _objective_value + eq_part + ineq_part;
    }
    
    
    
    
    Eigen::Matrix<double,-1,-1,Eigen::RowMajor> _A;
    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;
    
    Eigen::VectorXd _delta;
    Eigen::VectorXd _qdual;
    Eigen::Map<Eigen::VectorXd> _dmultiplier_eq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    Eigen::Map<Eigen::VectorXd> _dmultiplier_ineq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    Eigen::VectorXd _merit_grad;
    
};

    
} // end namespace teb



#endif /* defined(__teb_package__base_solver_nonlinear_program_dense__) */
