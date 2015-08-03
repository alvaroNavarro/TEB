#ifndef __teb_package__solver_sqp_local_dense__
#define __teb_package__solver_sqp_local_dense__

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
class SolverSQPLocalDense : public BaseSolverNonlinearProgramDense
{
    
public:
   
    SolverSQPLocalDense() : BaseSolverNonlinearProgramDense()
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
      
        
      for (unsigned int iter=0; iter < 1; ++iter) //cfg->optim.solver.solver_iter
      {
	  
        // Calculate objective and constraint values for the current working state
        buildObjectiveValue();
        buildEqualityConstraintValueVector();
        buildInequalityConstraintValueVector();
          
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
        if (checkConvergence() || checkKKTPoint())
        {
            PRINT_DEBUG_ONCE("SQP Solver converged. This debug message is printed once.");
            break;
        }
          
          PRINT_DEBUG("NORM: " << _lagrangian_gradient.norm() << " EQ: " << _equality_values.norm() << " c<0: " << (int) (_inequality_values.array() <= 0).all() << " lambda>0: " << (_multiplier_ineq.array() >= 0).all());
    
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
	    
        // apply the new increment delta to the vertices
	    applyIncrement(_delta);
    
	    // update multipliers (qp oases uses multiplier for the negative lagrangian convention) // TODO: adapt to negative as well for the complete solver, but test everything first!
	    _multiplier_eq =   -_dmultiplier_eq; //TODO: minus or plus or weighted with step?
	    _multiplier_ineq =  -_dmultiplier_ineq;
	    
      }
        
      return true;
    }
    
    
    bool checkConvergence() // calc gradient of lagrangian and equality values before
    {
        return ( sqrt(_lagrangian_gradient.squaredNorm() + _equality_values.squaredNorm()) < 1e-2 ) ? true : false;
    }

    bool checkKKTPoint()
    {
        return _lagrangian_gradient.norm() < 1e-2 && _equality_values.norm() < 1e-2 && (_inequality_values.array() <= 0).all(); // && (_multiplier_ineq >=).all();
    }
    
    
	qpOASES::SQProblem _qsolver;
    
    Eigen::Matrix<double,-1,-1,Eigen::RowMajor> _A;
    Eigen::VectorXd _lb;
    Eigen::VectorXd _ub;
    
    Eigen::VectorXd _delta;
    Eigen::VectorXd _qdual;
    Eigen::Map<Eigen::VectorXd> _dmultiplier_eq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    Eigen::Map<Eigen::VectorXd> _dmultiplier_ineq = Eigen::Map<Eigen::VectorXd>(nullptr,0);
    
    Eigen::VectorXd _increment;

    
};

    
} // end namespace teb


#endif /* defined(QPOASES) */


#endif /* defined(__teb_package__solver_sqp_local_dense__) */
