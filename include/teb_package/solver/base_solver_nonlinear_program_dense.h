#ifndef __teb_package__base_solver_nonlinear_program_dense__
#define __teb_package__base_solver_nonlinear_program_dense__

#include <teb_package/base/base_solver.h>

#include <Eigen/Dense>


// #define TEST

namespace teb
{

    
/**
 * @brief Extended base solver class for nonlinear programs (dense version).
 *
 * @ingroup solver
 *
 * This abstract class extends the BaseSolver class by routines and methods
 * required by dedicated nonlinear program solvers. \n
 * Inequality and equality constraints are treaten as hard constraints.
 * This class provides methods to calculate objective and constraint gradients/jacobians
 * and as well as the Hessian of the Lagrangian. \n
 * The actual solveImpl() implementation has to be done in a seperate subclass, e.g. depending 
 * if the underlying nonlinear-program solver is a newton-type solver, interior-point, barrier, sqp, ... solver.
 *
 * @todo The documentation of this class, after this class passed a couple of more tests
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class BaseSolverNonlinearProgramDense : public BaseSolver
{
    
public:
   
    BaseSolverNonlinearProgramDense() : BaseSolver() {} //!< Empty Constructor.
        

protected:
  
    // Actual solve implementation - See BaseSolver.
    virtual bool solveImpl() = 0;
    
    // Initialize edge workspaces - See BaseSolver.
    virtual void initWorkspaces()
    {
      // init workspaces and allocate memory only if graph structure was changed
      if (!_graph_structure_modified) return;
      
      for (EdgeType* edge : *_objectives) edge->allocateMemory(false); 
      for (EdgeType* edge : *_equalities) edge->allocateMemory(false); 
      for (EdgeType* edge : *_inequalities) edge->allocateMemory(false);
      
             
       // Check edge function types in debug mode and print warning
       // if constraints are marked to be squared by the solver, warn that this is only supported for objectives atm.
#ifndef NDEBUG
        auto checkSquared = [] (const EdgeType* edge) {return edge->functType() == FUNCT_TYPE::LINEAR_SQUARED || edge->functType() ==  FUNCT_TYPE::NONLINEAR_SQUARED;};
        PRINT_DEBUG_COND_ONCE(std::find_if(_equalities->begin(), _equalities->end(), checkSquared) != _equalities->end()
                  || std::find_if(_inequalities->begin(), _inequalities->end(), checkSquared) != _inequalities->end(),
                  "BaseSolverNonlinearProgramDense(): Constraint edge detected with function type FUNCT_TYPE::LINEAR_SQUARED or FUNCT_TYPE::NONLINEAR_SQUARED. Letting the solver take the square of edges is currently only supported for objective edges rhather than for constraint edges.");
#endif
    }
    
    // Make sure to init all space with zeros and resize correctly!
    virtual void initSolverWorkspace() = 0;

    
    void initializeLagrangeMultiplier()
    {
      _multiplier_eq.resize(_equalities_dim);
      _multiplier_eq.fill(1);
      
      _multiplier_ineq.resize(_inequalities_dim);
#ifdef TEST
      _multiplier_ineq.fill(1);
#else 
      _multiplier_ineq.fill(1);
#endif
    }
    
    //! Get the sum of all objective values since we solve \f$ f = \min \sum f_k \f$.
    void buildObjectiveValue();
    
    //! Build value / cost vector including all equality constraints
    void buildEqualityConstraintValueVector();
    
    //! Build value / cost vector including all inequality constraints
    void buildInequalityConstraintValueVector();
    
    void buildObjectiveGradient();

    void buildEqualityConstraintJacobian();

    void buildInequalityConstraintJacobian();
    
    void calculateLagrangianGradient() // TODO: distinguish between negative and positive lagrangian convention
    {
      _lagrangian_gradient = _objective_gradient + _equality_jacobian.transpose() * _multiplier_eq + _inequality_jacobian.transpose() * _multiplier_ineq;
    }
    
    void calculateLagrangianHessian(Eigen::VectorXd* increment = nullptr);
    
    
    void calculateLagrangianHessianNumerically();
    void initHessianBFGS();
    void calculateLagrangianHessianFullBFGS(Eigen::VectorXd* increment);
    
    using MatMapRowMajor = Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>>;
    
    double _objective_value; //!< Store the sum of all values computed in buildObjectiveValueVector() since we solve \f$ f = \min \sum f_k \f$.
    Eigen::VectorXd _equality_values; //!< Store the value vector computed in buildEqualityConstraintValueVector().    
    Eigen::VectorXd _inequality_values; //!< Store the value vector computed in buildInequalityConstraintValueVector().
    
    Eigen::VectorXd _objective_gradient; //!< Store the gradient of the objective value computed in buildObjectiveGradient().
    MatMapRowMajor _equality_jacobian = MatMapRowMajor(nullptr,0,0); //!< Store the jacobian matrix of the equality constraint computed in buildEqualityConstraintJacobian().
    MatMapRowMajor _inequality_jacobian = MatMapRowMajor(nullptr,0,0); //!< Store the jacobian matrix of the inequality constraint computed in buildInequalityConstraintJacobian().
    
    Eigen::VectorXd _lagrangian_gradient;
    Eigen::VectorXd _lagrangian_gradient_backup; //!< The gradient from the last step has to be stored for BFGS.
    Eigen::Matrix<double,-1,-1,Eigen::RowMajor> _lagrangian_hessian; //!< Store the hessian of the lagrangian \f$ \nabla^2 L = \nabla^2 (f - \boldsymbol{\mu}^T \mathbf{ceq} - \boldsymbol{\lambda}^T \mathbf{c}) \f$
    
    Eigen::VectorXd _multiplier_ineq;
    Eigen::VectorXd _multiplier_eq;

};

    


} // end namespace teb



#endif /* defined(__teb_package__base_solver_nonlinear_program_dense__) */
