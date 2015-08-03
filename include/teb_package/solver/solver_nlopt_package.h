#ifndef __teb_package__solver_nlopt_package__
#define __teb_package__solver_nlopt_package__

#include <teb_package/base/base_solver.h>

#ifdef NLOPT
#include <nlopt.hpp>
#endif

namespace teb
{

    
/**
 * @brief This class wraps the optimization problem to allow solving by NLOPT.
 *
 * @ingroup solver
 * 
 * Wrapper function for the NLopt optimization library (http://ab-initio.mit.edu/wiki/index.php/NLopt)
 * The purpose of this solver is not to perform an efficient and fast real-time optimization, rather than
 * having alternative constraint solvers for debugging and testing.
 * 
 * Refer to Config::Optim::Solver::Nlopt for configuration parameters.
 * 
 * @test This class needs more testing (different solvers etc.)
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SolverNloptPackage : public BaseSolver
{
    
public:
   
    SolverNloptPackage() : BaseSolver() {}  //!< Empty Constructor.
    
    //! Objective function wrapper that iterates all objective edges.
    static double objectiveFunction(unsigned n, const double *x, double *grad, void *solver_object_this);
    
    //! Equality constraint function wrapper that iterates all equality constraint edges.
    static void equalityConstraintFunction(unsigned m, double *result, unsigned n, const double* x, double* grad, void* solver_object_this);
    
    //! Inequality constraint function wrapper that iterates all inequality constraint edges (excluding bound constraints).
    static void inequalityConstraintFunction(unsigned m, double *result, unsigned n, const double* x, double* grad, void* solver_object_this);

    // Initialize edge workspaces - See BaseSolver.
    virtual void initWorkspaces()
    {
      for (EdgeType* edge : *_objectives) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
      for (EdgeType* edge : *_equalities) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
      for (EdgeType* edge : *_inequalities) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
    }

    
    EdgeContainer* objectives() {return _objectives;} //!< Return pointer to the edges containing objectives
    EdgeContainer* constraintsEq() {return _equalities;} //!< Return pointer to the edges containing equalities
    EdgeContainer* constraintsInEq() {return _inequalities;} //!< Return pointer to the edges containing inequalities
    int getOptVecDimFromStorage() const {return _opt_vec_dim;} //!< Return dimension of the optimization vector (call getOptVecDimension() before)
    int getEqConstrDimFromStorage() const {return _equalities_dim;} //!< Return dimension of the equality constraint vector
//     int getInEqConstrDimFromStorage() const {return _inequalities_dim;} //!< Return dimension of the inequality constraint vector
   
    void seperateInequalitiesAndBounds(); //!< Seperate constraintsInEq to bound constraints and general nonlinear inequality constraints and store results internally.
    int getBoundConstrDimFromStorage() const {return _bound_constraints_dim;} //!< Return dimension of the equality constraint vector
    int getInEqConstrDimFromStorage() const {return _inequalities_without_bounds_dim;} //!< Return dimension of the inequality constraint vector
    
protected:
  
    // Implements the BaseSolverLeastSquares::solveImpl() method.
    virtual bool solveImpl();
    
    EdgeContainer _bound_constraints; //!< Store bound constraints using seperateInequalitiesAndBounds().
    EdgeContainer _inequalities_without_bounds; //!< Store general nonlinear constraints using seperateInequalitiesAndBounds().
    int _bound_constraints_dim = 0; //!< Store dimension of bound constraints using seperateInequalitiesAndBounds().
    int _inequalities_without_bounds_dim = 0; //!< Store dimension ofgeneral nonlinear constraints using seperateInequalitiesAndBounds().
    
    Eigen::VectorXd _lower_bounds; //!< Store lower bounds according to the optimization vector
    Eigen::VectorXd _upper_bounds; //!< Store upper bounds according to the optimization vector
    Eigen::VectorXd _ctol_eq; //!< Store tolerances for the equality constraints (stopping criteria)
    Eigen::VectorXd _ctol_ineq; //!< Store tolerances for the inequality constraints (stopping criteria)
    
#ifdef NLOPT
    nlopt_opt _nlopt; //!< Store NLOPT solver object.
#endif
};

    
} // end namespace teb

#endif /* defined(__teb_package__solver_nlopt_package__) */
