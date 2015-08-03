#ifdef NLOPT

#include <teb_package/solver/solver_nlopt_package.h>
#include <iostream>
#include <memory>
#include <teb_package/base/common_teb_edges.h>
namespace teb
{

bool SolverNloptPackage::solveImpl()
{    
  
  // create optimization problem  
  nlopt_algorithm algorithm;
  switch (cfg->optim.solver.nlopt.algorithm)
  {
    case NloptAlgorithms::SLSQP:
      algorithm = NLOPT_LD_SLSQP;
    break;
   
  }
  
  _nlopt = nlopt_create( algorithm, (unsigned int) _opt_vec_dim); /* algorithm and dimensionality */  // NLOPT_LD_MMA, NLOPT_LN_COBYLA, NLOPT_LD_SLSQP,  NLOPT_LD_LBFGS, NLOPT_AUGLAG, NLOPT_GN_ISRES
  
  // set callback for objective function
  nlopt_set_min_objective(_nlopt, SolverNloptPackage::objectiveFunction, this);
  
  // set callback for constraints
  seperateInequalitiesAndBounds();
  
  
  // stopping criteria for constraints
  _ctol_eq.setConstant(getEqConstrDimFromStorage(), cfg->optim.solver.nlopt.tolerance_equalities);
  _ctol_ineq.setConstant(getInEqConstrDimFromStorage(), cfg->optim.solver.nlopt.tolerance_inequalities);
   


   if (getEqConstrDimFromStorage()>0)
   {
      nlopt_result retval = nlopt_add_equality_mconstraint(_nlopt, (unsigned) getEqConstrDimFromStorage(), SolverNloptPackage::equalityConstraintFunction, this, _ctol_eq.data());
      if (retval < 0)
      {
	PRINT_DEBUG("Adding equality constraints failed. Maybe switch solver type in NLOPT");
	return false;
      }
   }

   if (getBoundConstrDimFromStorage()>0)
   {

      _lower_bounds.setConstant(getOptVecDimFromStorage(), -INF);

      _upper_bounds.setConstant(getOptVecDimFromStorage(), INF);

      for (EdgeType* edge : _bound_constraints)
      {
	CustomBoundData* bound_data = static_cast<CustomBoundData*>(edge->getCustomData());
	if (!bound_data) continue;
	assert(edge->noVertices()==1 && bound_data);
	const VertexType* vertex = edge->getVertex(0);
	int vert_idx = vertex->getOptVecIdx();
	int no_free_vars = vertex->dimensionFree();
	
	// Get lower bounds
	if (bound_data->no_lower>0)
	{
	  assert(bound_data->lower.rows()==bound_data->no_lower && bound_data->no_lower == no_free_vars);
	  _lower_bounds.segment(vert_idx,no_free_vars) = (bound_data->lower.array() > _lower_bounds.segment(vert_idx,no_free_vars).array()).select(bound_data->lower,_lower_bounds.segment(vert_idx,no_free_vars));	
	}
	
	// Create upper bounds
	if (bound_data->no_upper>0)
	{
	  assert(bound_data->upper.rows()==bound_data->no_upper && bound_data->no_upper == no_free_vars);
 	  _upper_bounds.segment(vert_idx,no_free_vars) = (bound_data->upper.array() < _upper_bounds.segment(vert_idx,no_free_vars).array()).select(bound_data->upper,_upper_bounds.segment(vert_idx,no_free_vars));		 
	}
	delete bound_data;
      }
      nlopt_set_lower_bounds(_nlopt, _lower_bounds.data());
      nlopt_set_upper_bounds(_nlopt, _upper_bounds.data());
   }


   if (getInEqConstrDimFromStorage()>0)
   {
      nlopt_result retval = nlopt_add_inequality_mconstraint(_nlopt, (unsigned) getInEqConstrDimFromStorage(), SolverNloptPackage::inequalityConstraintFunction, this, _ctol_ineq.data());
      if (retval < 0)
      {
	PRINT_DEBUG("Adding inequality constraints failed. Maybe switch solver type in NLOPT");
	return false;
      }
   }
   
  // set tolerances for stopping criterias
  nlopt_set_ftol_rel(_nlopt, cfg->optim.solver.nlopt.stopping_criteria_ftol_rel);
  nlopt_set_ftol_abs(_nlopt, cfg->optim.solver.nlopt.stopping_criteria_ftol_abs);
  nlopt_set_xtol_rel(_nlopt, cfg->optim.solver.nlopt.stopping_criteria_xtol_rel);
  nlopt_set_xtol_abs1(_nlopt, cfg->optim.solver.nlopt.stopping_criteria_xtol_abs);
  nlopt_set_maxtime(_nlopt, cfg->optim.solver.nlopt.max_optimization_time);
  
  // initial values
  Eigen::VectorXd opt_vec_init = getOptVecCopy();

  double minf; /* the minimum objective value, upon return */
  
  // optimize
  nlopt_result opt_flag = nlopt_optimize(_nlopt, opt_vec_init.data(), &minf);
  if ( opt_flag < 0)
  {
	PRINT_DEBUG("nlopt failed!");
	nlopt_destroy(_nlopt);  
	return false;
  }
  
  
  // Copy optimization results to the TEB
  applyOptVec(opt_vec_init);
  
  nlopt_destroy(_nlopt);  
  return true;
};
    


/**
 * Calculates the sum of all objective functions with respect to the input optimization vector \c x.
 * @param n Size of the optimization vector (getOptVecDimFromStorage())
 * @param x Pointer to the optimization vector stored as double array [n x 1]
 * @param grad Pointer to the gradient double array [n x 1]
 * @param solver_object_this Pointer to the SolverNloptPackage object
 * @returns scalar objective function value / cost
 */
double SolverNloptPackage::objectiveFunction(unsigned n, const double *x, double *grad, void *solver_object_this)
{
    // Get solver object to update states and get edges storing objective functions
    SolverNloptPackage* solver = static_cast<SolverNloptPackage*>(solver_object_this);
    
    
    // update states
    assert( (int) n == solver->getOptVecDimFromStorage());
    Eigen::Map<const Eigen::VectorXd> opt_vec(x,n);
    solver->applyOptVec(opt_vec);
    
    double objective_scalar = 0; // store total objective value (sum over all objectives)
    
    // Iterate edges and get cost value
    for (EdgeType* objective : *solver->objectives())
    {
	objective->computeValues();
	for (int i=0; i< objective->dimension(); ++i)
	{
	   objective_scalar += objective->valuesData()[i];
	}
    }
    
    // Build gradient
    if (grad)
    {
      
      Eigen::Map<Eigen::VectorXd> grad_vec(grad,n);
      grad_vec.fill(0.0);
      
      for (EdgeType* edge : *solver->objectives())
      {
	  edge->computeJacobian();
	  for (unsigned int i=0; i < edge->noVertices(); ++i)
	  {
	    VertexType* vertex = edge->getVertex(i);	
	    int opt_vec_idx = vertex->getOptVecIdx();  
	    if (vertex->isFixedAny())
            {
                for (int j = 0; j < vertex->dimension(); ++j)
                {
                    if (!vertex->isFixedComp(j))
                    {
                        grad_vec.coeffRef(opt_vec_idx++) +=  edge->jacobians().getWorkspace(i).col(j).sum();
                    }
                }
            }
            else
            {
                // Copy complete gradient
                grad_vec.segment(opt_vec_idx, edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i).colwise().sum();
            }
	    
	  }
      }
    }
    return objective_scalar;
}

    
 /**
  * Calculates the equality constraint vector ceq(x)=0 with respect to the input optimization vector \c x.
  * @param m Number of equality constraints
  * @param result Equality constraint values ceq(x) [m x 1]
  * @param n Size of the optimization vector (getOptVecDimFromStorage())
  * @param x Pointer to the optimization vector stored as double array [n x 1]
  * @param grad Pointer to the gradient double matrix [n*m x 1] (Jacobian in RowMajor format: grad[i*n + j])
  * @param solver_object_this Pointer to the SolverNloptPackage object
  */   
void SolverNloptPackage::equalityConstraintFunction(unsigned m, double *result, unsigned n, const double* x, double* grad, void* solver_object_this)
{
    // Get solver object to update states and get edges storing objective functions
    SolverNloptPackage* solver = static_cast<SolverNloptPackage*>(solver_object_this);
    
    assert( (int) n == solver->getOptVecDimFromStorage());
    assert( (int) m == solver->getEqConstrDimFromStorage());
    
    // update states
    Eigen::Map<const Eigen::VectorXd> opt_vec(x,n);
    solver->applyOptVec(opt_vec);
        
    // Iterate edges and get cost value
    unsigned int idx_curr = 0;
    for (EdgeType* cval : *solver->constraintsEq())
    {
	cval->computeValues();
	std::memcpy(result+idx_curr, cval->valuesData(), cval->dimension() );
	idx_curr += cval->dimension();
    }

    // build constraint gradient matrix (jacobian transpose)
    if (grad)
    {
      // fore row-major representation since NLOPT stores the gradient matrix as follows:
      // grad[i*n + j]. Where i denotes the constraint index and j is the optimization variable index.
      // With row-major we can directly use the jacobian from the original hyper-graph
      // Refer to http://eigen.tuxfamily.org/dox-devel/group__TopicStorageOrders.html.
      Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>> jacobian(grad,m,n);
      jacobian.fill(0.0);

      unsigned int jac_row_idx = 0;
      
      for (EdgeType* edge : *solver->constraintsEq())
      {
	  edge->computeJacobian();
	  for (unsigned int i=0; i < edge->noVertices(); ++i)
	  {
	    VertexType* vertex = edge->getVertex(i);	
	    if (vertex->isFixedAny())
            {
                int vert_idx = vertex->getOptVecIdx();
                for (int j = 0; j < vertex->dimension(); ++j)
                {
                    if (!vertex->isFixedComp(j))
                    {
                        jacobian.block(jac_row_idx, vert_idx++, edge->dimension(), 1) +=  edge->jacobians().getWorkspace(i).col(j);
                    }
                }
            }
            else
            {
                // Copy complete jacobian block
                jacobian.block(jac_row_idx, vertex->getOptVecIdx(), edge->dimension(), edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i); 
            }
	  }
	  jac_row_idx += edge->dimension();
      }
    }
}    
    

    
 /**
  * Calculates the inequality constraint vector c(x)<0 with respect to the input optimization vector \c x.
  * @param m Number of inequality constraints
  * @param result Inequality constraint values c(x) [m x 1]
  * @param n Size of the optimization vector (getOptVecDimFromStorage())
  * @param x Pointer to the optimization vector stored as double array [n x 1]
  * @param grad Pointer to the gradient double matrix [n*m x 1] (Jacobian in RowMajor format: grad[i*n + j])
  * @param solver_object_this Pointer to the SolverNloptPackage object
  */     
void SolverNloptPackage::inequalityConstraintFunction(unsigned m, double *result, unsigned n, const double* x, double* grad, void* solver_object_this)
{
    // Get solver object to update states and get edges storing objective functions
    SolverNloptPackage* solver = static_cast<SolverNloptPackage*>(solver_object_this);
    
    assert( (int) n == solver->getOptVecDimFromStorage());
    assert( (int) m == solver->getInEqConstrDimFromStorage());
    
    // update states
    Eigen::Map<const Eigen::VectorXd> opt_vec(x,n);
    solver->applyOptVec(opt_vec);
        
    // Iterate edges and get cost value
    unsigned int idx_curr = 0;
    for (EdgeType* cval : *solver->constraintsInEq())
    {
	cval->computeValues();
	std::memcpy(result+idx_curr, cval->valuesData(), cval->dimension() );
	idx_curr += cval->dimension();
    }

    // build constraint gradient matrix (jacobian transpose)
    if (grad)
    {
      // fore row-major representation since NLOPT stores the gradient matrix as follows:
      // grad[i*n + j]. Where i denotes the constraint index and j is the optimization variable index.
      // With row-major we can directly use the jacobian from the original hyper-graph
      // Refer to http://eigen.tuxfamily.org/dox-devel/group__TopicStorageOrders.html.
      Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor>> jacobian(grad,m,n);
      jacobian.fill(0.0);
      
      unsigned int jac_row_idx = 0;
      
      for (EdgeType* edge : *solver->constraintsInEq())
      {
	  edge->computeJacobian();
	  for (unsigned int i=0; i < edge->noVertices(); ++i)
	  {
	    VertexType* vertex = edge->getVertex(i);	
	    if (vertex->isFixedAny())
            {
                int vert_idx = vertex->getOptVecIdx();
                for (int j = 0; j < vertex->dimension(); ++j)
                {
                    if (!vertex->isFixedComp(j))
                    {
                        jacobian.block(jac_row_idx, vert_idx++, edge->dimension(), 1) +=  edge->jacobians().getWorkspace(i).col(j);
                    }
                }
            }
            else
            {
                // Copy complete jacobian block
                jacobian.block(jac_row_idx, vertex->getOptVecIdx(), edge->dimension(), edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i); 
            }
	  }
	  jac_row_idx += edge->dimension();
      }
    }
}       
    
    
 
/**
 * Seperate the inequality constraints stored in SolverNloptPackage::_inequalities
 * into bound constraints (SolverNloptPackage::_bound_constraints) and 
 * actual nonlinear inequality constraints (SolverNloptPackage::_inequalities_without_bounds).
 * 
 * This method iterates all edges and checks the edge type obtained by
 * EdgeType::edgeType(), if it matches EDGE_TYPE::BOUNDCONSTRAINT.
 */ 
void SolverNloptPackage::seperateInequalitiesAndBounds()
{
  _bound_constraints.clear();
  _inequalities_without_bounds.clear();
  _bound_constraints_dim = 0;
  _inequalities_without_bounds_dim = 0;
  _bound_constraints.reserve(_inequalities_dim); // reserve worst case memory
  _inequalities_without_bounds.reserve(_inequalities_dim);
  for (EdgeType* edge : *_inequalities)
  {
    if (edge->isBoundConstraint())
    {
      _bound_constraints.push_back(edge);
      _bound_constraints_dim += edge->dimension();
    }
    else 
    {
      _inequalities_without_bounds.push_back(edge);
      _inequalities_without_bounds_dim += edge->dimension();
    }
  }
  
}
    
       
} // end namespace teb

#else // ifdef NLOPT
#pragma message( "solver_nlopt_package.cpp: You've linked against this package, but NLOPT is not defined or installed, skipping.")
#endif // ifdef NLOPT