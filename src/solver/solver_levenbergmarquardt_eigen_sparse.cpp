#include <teb_package/solver/solver_levenbergmarquardt_eigen_sparse.h>
#include <iostream>

namespace teb
{

    
bool SolverLevenbergMarquardtEigenSparse::solveImpl()
{    
  
     // get dimension for the value/cost vector
    _val_dim = getValueDimension();
      
    // adapt weights
    adaptWeights();
    
    // build sparse jacobian
    buildJacobian(); //TODO: maybe calcualte value vector before and use result inside jacobian to calculate chain-rule for inequalities
    
    // construct quasi-newton hessian approximation
    Eigen::SparseMatrix<double> hessian = _jacobian.transpose() * _jacobian;
    
    // build complete value vector
    buildValueVector();

    // construct right-hand-side of the approxiamted linear system
    Eigen::MatrixXd rhs = -_values.transpose() * _jacobian;
    rhs.transposeInPlace();

    // define delta vector (contains increments for all vertices)
    Eigen::VectorXd delta(_opt_vec_dim);
    
    // convergence consts
    const double eps1 = 1e-15;
    const double eps2 = 1e-15;
    const double eps3 = 1e-15;
    const double eps4 = 0;
    
    // lm variables
    unsigned int v = 2;
    unsigned int k = 0;
    double tau = 1e-5;
    
    const double goodStepUpperScale = 2./3.;
    const double goodStepLowerScale = 1./3.;
    
    bool stop = ( rhs.lpNorm<Eigen::Infinity>() <= eps1) ;
    
    double mu = tau * hessian.diagonal().maxCoeff();
    if (mu < 0) mu = 0;
    
    double rho = 0;
    
    // get old chi2
    double chi2_old = getChi2(); // _values has to be valid (call createValueVector() before

    
    // start levenberg marquardt optimization loop
    while (!stop && k < cfg->optim.solver.solver_iter)
    {
        k++;
        do
        {
            // augment hessian diagonal with damping factor
	    // iterate diagonal elements
	    for (int i=0; i < _opt_vec_dim ; ++i)
	    {
	      hessian.coeffRef(i,i) += mu;
	    }
            
            // solve linear system
			//delta = _sparse_solver.compute(hessian).solve(rhs);
			
			if (_graph_structure_modified)
			{
				// calc structural (non-zero) pattern only if graph (structure) was changed
				_sparse_solver.analyzePattern(hessian);
			}
			_sparse_solver.factorize(hessian); // calculate factors with actual values
			delta = _sparse_solver.solve(rhs); // solve linear system
			
            
            //if (delta.norm() <= eps2 * (x.norm() + eps2) stop = true; // x -> values of vertices, not constructed until now. modify check
            if (delta.norm() <= eps2)
                stop = true;
            else
            {
                // backup current vertex values
                backupVertices();
                
                // apply new delta/increment to vertices
                applyIncrement(delta);
                
                // calculate new costs/values
                buildValueVector();
                
                // get new chi2
                double chi2_new = getChi2();
                
                rho = (chi2_old - chi2_new) / (delta.transpose() * (mu * delta + rhs));
                
                if (rho > 0 && !isnan(chi2_new) && !isinf(chi2_new)) // adaptation of mu to find a sufficient descent step
                {
                    stop = ( sqrt(chi2_old) - sqrt(chi2_new) < eps4 * sqrt(chi2_old) );
                    
                    // accept update and discard backup
                    discardBackupVertices();
                    
                    // calculate new jacobian
                    buildJacobian();
                    
                    // get new hessian
                    hessian = _jacobian.transpose() * _jacobian;
                    
                    // get new cost/value vector
                    buildValueVector();
                    
                    // calculate new right hand side
                    rhs = -_values.transpose() * _jacobian;
                    rhs.transposeInPlace();
                    
                    stop = stop || ( rhs.lpNorm<Eigen::Infinity>() <= eps1);
                    
                    double alpha = std::min ( goodStepUpperScale, 1 - pow((2*rho-1),3) );
                    double scaleFactor = std::max(goodStepLowerScale, alpha);
                    mu *= scaleFactor;
                    v = 2;
                    
                    chi2_old = chi2_new;
                }
                else
                {
                    // restore from backup
                    restoreVertices();
                    //hessian.diagonal().array() -= mu; // this should be doen here, but it works in the matlab version without for some time now.
                    
                    mu = mu*v;
                    v = 2*v;
                }
            }
            
        } while ( rho <= 0 && !stop);
        stop = (_values.norm() <= eps3);
    }
    
    return true;
};
    
   
/**
 * This method constructs the jacobian of the complete optimziation problem.
 * The size of the jacobian is [getValueDimension() x getOptVecDimension()].
 * This method queries the EdgeType::computeJacobian() of each edge.
 * The column index for each entry is determined using TebVertex::getOptVecIdx and the number of free dimensions for each vertex.
 *
 * Since this solver relies on soft-constraints for equality and inequality constraints, the chain-rule for calculating derivatives is utilized.
 * Equality constraints are treaten as usual objective, therefore the jacobian does not need to be changed.\n
 * For inequality constraints, the derivative of \f$ \max{(c(x),0)} \f$ is
 * \f[
 *     \begin{cases}
 *        \frac{\partial}{\partial x}  c_i(x) & \text{if} \ c_i(x)>0 \\
 *        0 & \text{otherwise}
 *     \end{cases} = \mathbf{J}_{ineq}(:,j=1:n) .* \frac{1}{\mathbf{c}(x)} .* \max{(\mathbf{c}(x),\mathbf{0}})
 * \f]
 * The max operator is applicated component-wise. '\f$ .* \f$' corresponds to Matlab's component-wise multiplication.
 *
 * The resulting jacobian matrix is stored to SolverLevenbergMarquardtEigenSparse::_jacobian;
 *
 * @todo Check if implemented ordering of column and row iterations fits Eigen's column major representation best.
 */
void SolverLevenbergMarquardtEigenSparse::buildJacobian()
{
    // Allocate memory for the sparse jacobian matrix
    allocateSparseJacobian();
    
    // Now fill sparse jacobian with block-jacobians of each edge.
    unsigned int jac_row_idx = 0;
    
    // start with objectives
    for (EdgeType* edge : *_objectives)
    {
        edge->computeJacobian();
        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_free_idx = vertex->getOptVecIdx();
                for (int block_col = 0; block_col < vertex->dimension(); ++block_col)
                {
                    if (!vertex->isFixedComp(block_col))
                    {
                        for (int block_row = 0; block_row < edge->dimension(); ++block_row)
                        {
                            _jacobian.coeffRef(jac_row_idx+block_row, vert_free_idx) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col);
                        }
                        ++vert_free_idx;
                    }
                }
            }
            else
            {
                // Copy complete jacobian block
                for (int block_col = 0; block_col < edge->jacobians().getWorkspace(i).cols(); ++block_col)
                {
                    for (int block_row = 0; block_row < edge->dimension(); ++block_row)
                    {
                        // we call coeffRef on the sparse jacobian instead of insert. For the first insertion it is slower, because a binary search is performed.
                        // But we do not need to track, if we have a first insertion or an accumulation.
                        _jacobian.coeffRef(jac_row_idx+block_row, vertex->getOptVecIdx()+block_col) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col);
                    }
                }
            }
        }
        jac_row_idx += edge->dimension();
    }
    
    // continue with equality constraints
    for (EdgeType* edge : *_equalities)
    {
        edge->computeJacobian();
        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_free_idx = vertex->getOptVecIdx();
                for (int block_col = 0; block_col < vertex->dimension(); ++block_col)
                {
                    if (!vertex->isFixedComp(block_col))
                    {
                        for (int block_row = 0; block_row < edge->dimension(); ++block_row)
                        {
                            _jacobian.coeffRef(jac_row_idx+block_row, vert_free_idx) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col) * sqrt(_weight_equalities); // sqrt(weight) only to make it comparable to matlab version
                        }
                        ++vert_free_idx;
                    }
                }
            }
            else
            {
                // Copy complete jacobian block
                for (int block_col = 0; block_col < edge->jacobians().getWorkspace(i).cols(); ++block_col)
                {
                    for (int block_row = 0; block_row < edge->dimension(); ++block_row)
                    {
                        // we call coeffRef on the sparse jacobian instead of insert. For the first insertion it is slower, because a binary search is performed.
                        // But we do not need to track, if we have a first insertion or an accumulation.
                        _jacobian.coeffRef(jac_row_idx+block_row, vertex->getOptVecIdx()+block_col) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col) * sqrt(_weight_equalities);  // sqrt(weight) only to make it comparable to matlab version
                    }
                }
            }
        }
        jac_row_idx += edge->dimension();
    }

    // continue with inequality constraints
    
    // In case of inequalities, that are not support by conventional least-squares algorithms, we choose soft constraints.
    // Costs are squared. In case of inequalities, we have only costs/values for c(x)>0, since we want the optimizer to satisfy c(x)<=0.
    // We choose soft constraints: values = max(c(x),0). Thus, we have zero cost for c(x)<=0 and otherwise negative cost.
    // Within the least squares framework, we actually calculate values.^2 which leads to positive costs for c(x)>0.
    // In addition, the squared problem is twice differentiable if c(x) is once diffenentiable (and c(x) should intersect with the zero axis).
    // For the following jacobian, we can use chane-rule to compute the derivative of max(c(x),0), that is d/dx c(x) * { 1 if c(x)>0; otherwise 0} = Jc * 1/c(x)*max(c(x),0);
    for (EdgeType* edge : *_inequalities)
    {
        edge->computeJacobian();
        edge->computeValues(); // we need the values for the jacobian chain-rule. (Here we do not assume, that computeValues was called before....)
        //Eigen::VectorXd soft_constr_values = edge->valuesMap().array().max(0.0);
        //Eigen::VectorXd soft_constr_values = (edge->valuesMap().array() > 0).select(1.0,0.0); // -> 1 if c(x)>0, otherwise 0 // Not working with select(scalar, scalar)
        //switch to simple for loop:
        Eigen::VectorXd soft_constr_values = edge->valuesMap();
        double epsilon = cfg->optim.solver.lsq.soft_constr_epsilon;
        for (int i=0; i<edge->dimension(); ++i)
        {
            if (soft_constr_values[i]<= -epsilon) soft_constr_values[i] = 0.0; // consider soft constraint epsilon: value = max(c(x)-epsilon,0) = max(c(x),-epsilon)
            else soft_constr_values[i] = 1.0;
        }

        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_free_idx = vertex->getOptVecIdx();
                for (int block_col = 0; block_col < vertex->dimension(); ++block_col)
                {
                    if (!vertex->isFixedComp(block_col))
                    {
			for (int block_row = 0; block_row < edge->dimension(); ++block_row)
			{
			  _jacobian.coeffRef(jac_row_idx+block_row, vert_free_idx) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col) * soft_constr_values.coeffRef(block_row) * sqrt(_weight_inequalities); // sqrt(weight) only to make it comparable to matlab version
			}
			++vert_free_idx;
                    }
                }
            }
            else
            {
                // Copy complete jacobian block 
               	for (int block_col = 0; block_col < edge->jacobians().getWorkspace(i).cols(); ++block_col)
                {
                    for (int block_row = 0; block_row < edge->dimension(); ++block_row)
                    {
                        // we call coeffRef on the sparse jacobian instead of insert. For the first insertion it is slower, because a binary search is performed.
                        // But we do not need to track, if we have a first insertion or an accumulation.
                        _jacobian.coeffRef(jac_row_idx+block_row, vertex->getOptVecIdx()+block_col) += edge->jacobians().getWorkspace(i).coeffRef(block_row,block_col) * soft_constr_values.coeffRef(block_row)  * sqrt(_weight_inequalities);  // sqrt(weight) only to make it comparable to matlab version
                    }
                }
            }
        }
        jac_row_idx += edge->dimension();
    }
    
    
}
    
    
    
void SolverLevenbergMarquardtEigenSparse::countJacobianColNNZ()
{
    _nnz_per_col.resize(_opt_vec_dim);
    _nnz_per_col.setZero();
    
    // Iterate objectives
    for (const EdgeType* edge : *_objectives)
    {
        // Iterate all attached vertices
        for (unsigned int vert_idx=0; vert_idx < edge->noVertices(); ++vert_idx)
	{
	  const VertexType* vertex = edge->getVertex(vert_idx);
	  unsigned int vertex_oidx = (unsigned int) vertex->getOptVecIdx();
	  
	  // Iterate all free variables
	  unsigned int idx_free = 0;
	  for (int i=0; i < vertex->dimension(); ++i)
	  {
	    if (!vertex->isFixedComp(i))
	    {
	      // If the complete block jacobian for this edge and vertex is non-zero, 
	      // the number of nnz per col equals the edge-dimension.
	      _nnz_per_col[vertex_oidx+idx_free] += edge->dimension();
	      ++idx_free;
	    }
	  }
	}
    }
    
    // Iterate equality constraints
    for (const EdgeType* edge : *_equalities)
    {
        // Iterate all attached vertices
        for (unsigned int vert_idx=0; vert_idx < edge->noVertices(); ++vert_idx)
	{
	  const VertexType* vertex = edge->getVertex(vert_idx);
	  unsigned int vertex_oidx = (unsigned int) vertex->getOptVecIdx();
	  
	  // Iterate all free variables
	  unsigned int idx_free = 0;
	  for (int i=0; i < vertex->dimension(); ++i)
	  {
	    if (!vertex->isFixedComp(i))
	    {
	      // If the complete block jacobian for this edge and vertex is non-zero, 
	      // the number of nnz per col equals the edge-dimension.
	      _nnz_per_col[vertex_oidx+idx_free] += edge->dimension();
	      ++idx_free;
	    }
	  }
	}
    }
    
    // Iterate inequality constraints
    for (const EdgeType* edge : *_inequalities)
    {
        // Iterate all attached vertices
        for (unsigned int vert_idx=0; vert_idx < edge->noVertices(); ++vert_idx)
	{
	  const VertexType* vertex = edge->getVertex(vert_idx);
	  unsigned int vertex_oidx = (unsigned int) vertex->getOptVecIdx();
	  
	  // Iterate all free variables
	  unsigned int idx_free = 0;
	  for (int i=0; i < vertex->dimension(); ++i)
	  {
	    if (!vertex->isFixedComp(i))
	    {
	      // If the complete block jacobian for this edge and vertex is non-zero, 
	      // the number of nnz per col equals the edge-dimension.
	      _nnz_per_col[vertex_oidx+idx_free] += edge->dimension();
	      ++idx_free;
	    }
	  }
	}
    }
}
    
    
       
} // end namespace teb

