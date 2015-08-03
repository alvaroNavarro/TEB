#ifndef __teb_package__solver_sqp_dense__
#define __teb_package__solver_sqp_dense__

#ifdef QPOASES

#include <teb_package/solver/base_solver_nonlinear_program_dense.h>

#include <qpOASES.hpp>



namespace teb
{


/**
 * @brief Dense sequential quadratic programming (SQP) solver for nonlinear programs.
 *
 * @ingroup solver
 *
 * @todo The documentation of this class, after the SQP sovler passed a couple of remaining (robustness) tests
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */  
class SolverSQPDense : public BaseSolverNonlinearProgramDense
{
    
public:
   
    SolverSQPDense() : BaseSolverNonlinearProgramDense()
    {
        _qsolver.setPrintLevel(qpOASES::PL_NONE);
    } //!< Solver Constructor.
        

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
       
       _merit_alpha = cfg->optim.solver.nonlin_prog.linesearch.alpha_init;
        
       // Map data
       
       // -ceq <= A*b <= -ceq, since equality constraint: lb = ub here
       // WARNING: we must call ceq(x)*=-1 before passing to the quadratic solver
       //new (&_equality_values) Eigen::Map<Eigen::VectorXd>( _lb.data(), _equalities_dim ); 
       //new (&_equality_values) Eigen::Map<Eigen::VectorXd>( _ub.data(), _equalities_dim );
       
       // c(x)<0 -> c(x_k) + Jc * b < 0  ->  -inf < Jc * b < -c(x_k)   0 <= A*b <= -c, therefore: lb=0, ub = -c
       // WARNING: we must call c(x)*=-1 before passing to the quadratic solver
       //new (&_inequality_values) Eigen::Map<Eigen::VectorXd>( _ub.tail(_inequalities_dim).data(), _inequalities_dim );
       

       _ub.tail(_inequalities_dim).setConstant(INF); // TEST CHANGE HERE

       
       new (&_equality_jacobian) MatMapRowMajor( _A.data(), _equalities_dim, _opt_vec_dim );
       new (&_inequality_jacobian) MatMapRowMajor( _A.bottomRows(_inequalities_dim).data(), _inequalities_dim, _opt_vec_dim );
        
        // we need to recreate the solver class since it only accepts new dimensions while constructing
        if (cfg->optim.solver.nonlin_prog.hessian.hessian_method == HessianMethod::ZERO_HESSIAN)
        {
            _qsolver = qpOASES::SQProblem(_opt_vec_dim, (int) _A.rows(),qpOASES::HST_ZERO);
        }
        else
        {
			_qsolver = qpOASES::SQProblem(_opt_vec_dim, (int)_A.rows());
        }
        _qsolver.setPrintLevel(qpOASES::PL_NONE);
    }
    
    // Actual solve implementation - See BaseSolver.
    virtual bool solveImpl()
    {
      
      if (_graph_structure_modified)
      {
          initSolverWorkspace(); // TODO: call only if graph is modified, but be careful with multiplier initialization etc.
      }
        
      // initialize lagrange multiplier for equality and inequality constraints
      initializeLagrangeMultiplier(); // TODO: call everytime?
      
      
      
      // Calculate objective and constraint values for the current working state
      buildObjectiveValue();
      buildEqualityConstraintValueVector();
      buildInequalityConstraintValueVector();

        
      // linesearch parameter -> cfg
      
      for (unsigned int iter=0; iter < 5; ++iter) //cfg->optim.solver.solver_iter
      {
	  
	    // Now calculate the gradient, jacobians and hessians
	    // We calculate it here, since the objective and constraint values are calculated 
	    // at the end of the loop and in the initialization above
	    buildObjectiveGradient();
	    buildEqualityConstraintJacobian();
	    buildInequalityConstraintJacobian();
        calculateLagrangianGradient();
        if (iter==0 && _graph_structure_modified && (cfg->optim.solver.nonlin_prog.hessian.hessian_method == HessianMethod::BLOCK_BFGS || cfg->optim.solver.nonlin_prog.hessian.hessian_method == HessianMethod::FULL_BFGS || cfg->optim.solver.nonlin_prog.hessian.hessian_method == HessianMethod::FULL_BFGS_WITH_STRUCTURE_FILTER) )
	    {
            initHessianBFGS();

	    }
        else
        {
            calculateLagrangianHessian(&_increment);
        }
        
        // Check convergence
        if (checkConvergence())
        {
            PRINT_DEBUG_ONCE("SQP Solver converged. This debug message is printed once.");
            break;
        }
    
	    // solve quadratic program with qpOASES (approx. of the original problem)
	    //int nWSR = 5 * (_opt_vec_dim + _A.rows());
	    int nWSR = 5 * (int) _A.rows();
	    
	    // We need to do some copies here atm.
	    _lb.head(_equalities_dim) = -_equality_values;
	    _ub.head(_equalities_dim) = _lb.head(_equalities_dim);
	    _lb.tail(_inequalities_dim) = _inequality_values;

        if (iter==0 || _graph_structure_modified)
        {
            _qsolver.init( _lagrangian_hessian.data(), _objective_gradient.data(), _A.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
        }
        else
        {
            _qsolver.hotstart( _lagrangian_hessian.data(), _objective_gradient.data(), _A.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
        }
          
	    // get solution of the QPOASES optimization
	    _qsolver.getPrimalSolution( _delta.data() );
	    _qsolver.getDualSolution( _qdual.data() );
        
	    
	    // check convergence 2
	    if (_delta.norm() < 1e-3) break;
	    
	    // perform line search to get a delta that actually reduces the merit function
	    
	    // get prameter for the merit function based on old constraint values // TODO: Check that the sequence of _merit_alpha is bounded.
        _merit_alpha = std::max(_merit_alpha, std::max(_inequality_values.maxCoeff(), _equality_values.lpNorm<Eigen::Infinity>()) + 0.01 );
          
	    // get merit value for the working state a-priori to solving the quadratic problem (we didn't update anything until now)
	    // the derivative of the merit function with respect to the new delta!
        double merit_derivative = calculateMeritDerivative(_merit_alpha);
	    double merit_old = calculateMerit(_merit_alpha);
	    double step = 0.99; // start with step width 1, that means we would accept the complete delta of the qproblem and add it to the original values.
	    
	    // backup current solution
	    backupVertices();
	    
	    // apply the new increment step*delta to the vertices
        _increment = step*_delta;
	    applyIncrement(_increment);
    

	    //double norm_eq = _equality_values.norm(); // TODO: norm calculation redundant, used for the workaround before
	    
	    // Calculate new objective and constraint values and afterwards recalculate merit (with same rho as before)
	    buildObjectiveValue();
	    buildEqualityConstraintValueVector();
	    buildInequalityConstraintValueVector();

	    double merit_new = calculateMerit(_merit_alpha);
	    
	    // Workaround: sometimes the first step is not accepted, if obejctive costs are too high
	    //if (_equality_values.norm()/norm_eq < 0.5) merit_new = 0; // TODO: redundant as well, maybe better workaround or store norm
	    
	    // do actual line search now (backtracking strategy)
	    while (merit_new > merit_old + cfg->optim.solver.nonlin_prog.linesearch.sigma*step*merit_derivative)
	    {
            // decrease step width by factor beta (<1)
	        step *= cfg->optim.solver.nonlin_prog.linesearch.beta;
            if (step<1e-4)
            {
              PRINT_DEBUG("SolverSQPDense(): Trying to perform linesearch, but the step is still decreasing without reducing merit. Maybe still converged?");
              break;
            }
            
            // Apply new increment to the vertices, but restore old values before // TODO: Can we achieve the same more efficiently?
            restoreVerticesButKeepBackup();
            _increment = step*_delta;
            applyIncrement(_increment);
            
            // recalculate new objective, cosntraint values and merit
            buildObjectiveValue();
            buildEqualityConstraintValueVector();
            buildInequalityConstraintValueVector();
            merit_new = calculateMerit(_merit_alpha);
	    }
	    
	    // update multipliers (qp oases uses multiplier for the negative lagrangian convention) // TODO: adapt to negative as well for the complete solver, but test everything first!
	    _multiplier_eq =   -_dmultiplier_eq; //TODO: minus or plus or weighted with step?
	    _multiplier_ineq =  -_dmultiplier_ineq;
	    
	    // discard backup
	    discardBackupVertices();
         
	    
      }
        
      return true;
    }
    
    
    bool checkConvergence() // calc gradient of lagrangian and equality values before
    {
        return ( sqrt(_lagrangian_gradient.squaredNorm() + _equality_values.squaredNorm()) < 1e-2 ) ? true : false;
    }
    
        
    double calculateMerit(double alpha) const
    {
	assert(alpha!=0);
	// exact penalty merit function (M = f + 1/rho ||ceq|| + 1/rho ||max(0,c)||)
	double ineq_part = _inequality_values.cwiseMax(0).sum();
	double eq_part = _equality_values.lpNorm<1>();
    return _objective_value + alpha*(eq_part + ineq_part);
    }
    
    double calculateMeritDerivative(double alpha) const
    {
        assert(alpha!=0);

        double ineq_part = 0;
		for (unsigned int i = 0; i < (unsigned int) _inequalities_dim; ++i) // maybe implement with eigen and without for-loop
        {
            if (_inequality_values.coeffRef(i)>0)
            {
               ineq_part += _inequality_jacobian.row(i) * _delta;
            }
            else if (_inequality_values.coeffRef(i)==0)
            {
                ineq_part += std::max( double(_inequality_jacobian.row(i) * _delta), 0.0);
            }
            
        }
        double eq_part = 0;
		for (unsigned int i = 0; i < (unsigned int) _equalities_dim; ++i) // maybe implement with eigen and without for-loop
        {
            if (_equality_values.coeffRef(i) > 0 )
            {
                eq_part += _equality_jacobian.row(i) * _delta;
            }
            else if (_equality_values.coeffRef(i) < 0)
            {
                eq_part -= _equality_jacobian.row(i) * _delta;
            }
            else
            {
                eq_part += fabs(_equality_jacobian.row(i) * _delta);
            }
        }
        return _objective_gradient.dot(_delta) + alpha*(eq_part + ineq_part);
    }
    
    

    
	qpOASES::SQProblem _qsolver;
    
    Eigen::Matrix<double,-1,-1,Eigen::RowMajor> _A;
    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;
    
    Eigen::VectorXd _delta;
    Eigen::VectorXd _qdual;
    Eigen::Map<Eigen::VectorXd> _dmultiplier_eq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    Eigen::Map<Eigen::VectorXd> _dmultiplier_ineq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    Eigen::VectorXd _merit_grad;
    
    Eigen::VectorXd _increment;
    
    double _merit_alpha = 0;
    
};

    
} // end namespace teb


#endif /* defined(QPOASES) */


#endif /* defined(__teb_package__base_solver_nonlinear_program_dense__) */
