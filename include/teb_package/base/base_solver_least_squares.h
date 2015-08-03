#ifndef __teb_package__base_solver_least_squares__
#define __teb_package__base_solver_least_squares__

#include <teb_package/base/base_solver.h>

#include <Eigen/Dense>

namespace teb
{

    
/**
 * @brief Extended base solver class for least squares optimizations.
 *
 * @ingroup solver
 *
 * This abstract class extends the BaseSolver class by routines and methods
 * required by dedicated least squares solvers. \n
 * It transforms inequality and equality constraints into soft constraints 
 * that are added to the objective functions.
 * objective functions are squared. \n
 * The jacobian calculation is based on the new transformed objective function
 * and used for quasi-newton hessian approximation \f$ \mathbf{H} = \mathbf{J}^T \mathbf{J} \f$.
 * However, the actual hessian calculation using the computeHessian() functions of each edge
 * is ommitted.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class BaseSolverLeastSquares : public BaseSolver
{
    
public:
   
    BaseSolverLeastSquares() : BaseSolver() {} //!< Empty Constructor.
        

protected:
  
    // Actual solve implementation - See BaseSolver.
    virtual bool solveImpl() = 0;
    
    // Initialize edge workspaces - See BaseSolver.
    virtual void initWorkspaces()
    {
      // init workspaces and allocate memory only if graph structure was changed
      if (!_graph_structure_modified) return;
      
      for (EdgeType* edge : *_objectives) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
      for (EdgeType* edge : *_equalities) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
      for (EdgeType* edge : *_inequalities) edge->allocateMemory(true); // skip hessian memory alloc, since it is not required
        
        
        // Check edge function types in debug mode and print warning
        // if quadratic functions are found, since this solver takes the square itself
#ifndef NDEBUG
        auto checkQuadratic = [] (const EdgeType* edge) {return edge->functType() == FUNCT_TYPE::QUADRATIC;};
        PRINT_DEBUG_COND_ONCE(std::find_if(_objectives->begin(), _objectives->end(), checkQuadratic) != _objectives->end()
                  || std::find_if(_equalities->begin(), _equalities->end(), checkQuadratic) != _equalities->end()
                  || std::find_if(_inequalities->begin(), _inequalities->end(), checkQuadratic) != _inequalities->end(),
                  "BaseSolverLeastSquares(): Edge detected with function type FUNCT_TYPE::QUADRATIC. It is not recommend to use this edge with least-squares solvers since lsq-solvers square edge functions/values itself (since it is more efficient). The resulting edge has now power of 4. Think about using FUNCT_TYPE::LINEAR_SQUARED or FUNCT_TYPE::NONLINEAR_SQUARED to force all supported/implemented solver interfaces to take the square the values itself.");
#endif
    }
    
    //! Build value / cost vector including all objectives and constraints.
    void buildValueVector();
    
    //! Get dimension of the composed value/cost vector (including objectives and constraints).
    int getValueDimension() const;
    

    /**
     * @brief Calculate the \f$ \chi^2 \f$ error of the optimization problem.
     *
     * The \f$ \chi^2 \f$ error is the scalar cost value of the least-squares optimization problem since 
     * the total cost function is composed of weighted terms:
     * \f[ f(\mathcal{B}) = \mathbf{f}(\mathcal{B})^T \mathbf{f}(\mathcal{B}) \f]
     * Weights are included in \f$ \mathbf{f}(\mathcal{B}) \f$.
     * In this case it is \f$ \chi^2 = f(\mathcal{B}) \f$.
     * @todo Maybe switch to the formulation \f$ f(\mathcal{B}) = \mathbf{f}(\mathcal{B})^T \, \boldsymbol{\Omega} \, \mathbf{f}(\mathcal{B}) \f$ similar to g2o and Teb-Matlab.
     *
     */
    double getChi2() const
    {
        return _values.squaredNorm();
    }
    
    //! Automatic weight adaptation that increases soft constraint weights after each outer teb iteration.
    void adaptWeights();
    
    Eigen::VectorXd _values; //!< Store the value vector computed in buildValueVector().
        
    int _weight_adapt_count = 0; //!< Store current state of the weight adaptation method adaptWeights().
    double _weight_equalities = 1; //!< Store current weight for equality soft-constraints.
    double _weight_inequalities = 1; //!< Store current weight for inequality soft-constraints.
   
    int _val_dim = -1; //!< Store dimension of the value vector here (see getValueDimension()).
};

    
} // end namespace teb

#endif /* defined(__teb_package__base_solver_least_squares__) */
