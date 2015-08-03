#ifndef __teb_package__solver_levenbergmarquardt_eigen_dense__
#define __teb_package__solver_levenbergmarquardt_eigen_dense__

#include <teb_package/base/base_solver_least_squares.h>


namespace teb
{

    
/**
 * @brief Levenberg-Marquardt Solver that uses Eigen to solve the resulting linear system (Dense matrices version).
 *
 * @ingroup solver
 *
 * This solver implements the Levenberg-Marquardt algorithm also implemented in \cite kummerle2011.
 * The underlying linear system is solved using Eigens Dense Matrix algebra 
 * (http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html).
 *
 * The jacobian required for the optimization routine is stored in a dense matrix.
 * Therefore no exploitation of the sparse structure for solving the linear system is taken into account.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SolverLevenbergMarquardtEigenDense : public BaseSolverLeastSquares
{
    
public:
   
    SolverLevenbergMarquardtEigenDense() : BaseSolverLeastSquares() {}  //!< Empty Constructor.
    
protected:
  
    // Implements the BaseSolverLeastSquares::solveImpl() method.
    virtual bool solveImpl();
        
    //! Construct the jacobian of the complete least-squares optimziation problem.
    void buildJacobian();
    
    Eigen::MatrixXd _jacobian; //!< Store the jacobian matrix of the complete least-squares optimziation problem.
};

    
} // end namespace teb

#endif /* defined(__teb_package__solver_levenbergmarquardt_eigen_dense__) */
