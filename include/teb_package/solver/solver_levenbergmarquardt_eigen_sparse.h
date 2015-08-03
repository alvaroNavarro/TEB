#ifndef __teb_package__solver_levenbergmarquardt_eigen_sparse__
#define __teb_package__solver_levenbergmarquardt_eigen_sparse__

#include <teb_package/base/base_solver_least_squares.h>

#include <Eigen/Sparse>

#ifdef CHOLMOD
  #include <Eigen/CholmodSupport>
#endif

namespace teb
{

    
/**
 * @brief Levenberg-Marquardt Solver that uses Eigen to solve the resulting linear system (Sparse matrices version).
 *
 * @ingroup solver
 *
 * This solver implements the Levenberg-Marquardt algorithm also implemented in \cite kummerle2011.
 * The underlying linear system is solved using Eigens Sparse Matrix algebra
 * ( http://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html ).
 *
 * The jacobian required for the optimization routine is stored in a sparse matrix.
 *
 * @todo Consider only the upper diagonal part of the hessian?
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SolverLevenbergMarquardtEigenSparse : public BaseSolverLeastSquares
{
    
public:
   
    SolverLevenbergMarquardtEigenSparse() : BaseSolverLeastSquares() {}  //!< Empty Constructor.
        
protected:
  
    // Implements the BaseSolverLeastSquares::solveImpl() method.
    virtual bool solveImpl();
        
    //! Allocate sparse jacobian
    void allocateSparseJacobian()
    {
      _jacobian.resize(_val_dim, _opt_vec_dim);
      countJacobianColNNZ();
      _jacobian.reserve(_nnz_per_col);
    }
    
    //! Construct the jacobian of the complete least-squares optimziation problem.
    void buildJacobian();
    
    /**
     * @brief Count number of non-zeros per column in order to intialize jacobain sparse matrix.
     * The results are stored to the class member variable SolverLevenbergMarquardtEigenSparse::_nnz_per_col.
     * @remarks: Make sure that BaseSolverLeastSquares::_opt_vec_dim is valid!
     */
    void countJacobianColNNZ();
    
    Eigen::SparseMatrix<double> _jacobian; //!< Store the jacobian matrix of the complete least-squares optimziation problem.
    
    Eigen::VectorXi _nnz_per_col; //!< Store number of non-zeros per column of the jacobian matrix.
    
    /**
     * @brief Eigens sparse solver wrapper.
     * Check http://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html for further information and different solvers.
     * The second template parameter specifies, whether the upper or lower triangular part should be used.
     * If CHOLMOD was found by CMake, the cholmod supernodal llt version is used rathar than Eigens simplicial ldlt.
     * Define \c FORCE_EIGEN_SOLVER if the default LDLT should sill be used even if CHOLMOD is found.
     */
#if defined(CHOLMOD) && !defined(FORCE_EIGEN_SOLVER)
    Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>, Eigen::Upper> _sparse_solver;
#else
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Upper> _sparse_solver;
#endif
    
};

    
} // end namespace teb

#endif /* defined(__teb_package__solver_levenbergmarquardt_eigen_sparse__) */
