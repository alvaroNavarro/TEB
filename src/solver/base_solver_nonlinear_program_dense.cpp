#include <teb_package/solver/base_solver_nonlinear_program_dense.h>


namespace teb
{

void BaseSolverNonlinearProgramDense::buildObjectiveValue()
{
    _objective_value = 0;
    
    int idx = 0;
    for (EdgeType* edge : *_objectives)
    {
        edge->computeValues();
	
	// Take the square of each value if desired by the creator of the current edge
	if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
	{
	  _objective_value += edge->valuesMap().squaredNorm();
	}
	else
	{
	  _objective_value += edge->valuesMap().sum();
	}
        idx += edge->dimension();
    };
}


void BaseSolverNonlinearProgramDense::buildEqualityConstraintValueVector()
{
    // resize rhs to current problem size
    //_equality_values.resize(_equalities_dim);
    assert (_equalities_dim == _equality_values.rows());  
    
    int idx = 0;
    for (EdgeType* edge : *_equalities)
    {
        edge->computeValues();
	
	// Take the square of each value if desired by the creator of the current edge
	//if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
	//{
	//  _equality_values.segment(idx,edge->dimension()) = edge->valuesMap().array().square();
	//}
	//else
	//{
	  _equality_values.segment(idx,edge->dimension()) = edge->valuesMap();
	//}
        idx += edge->dimension();
    };
}

void BaseSolverNonlinearProgramDense::buildInequalityConstraintValueVector()
{
    // resize rhs to current problem size
    //_inequality_values.resize(_inequalities_dim);
    assert (_inequalities_dim == _inequality_values.rows());  
    
    int idx = 0;
    for (EdgeType* edge : *_inequalities)
    {
        edge->computeValues();
	
	// Take the square of each value if desired by the creator of the current edge
	//if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
	//{
	//  _inequality_values.segment(idx,edge->dimension()) = edge->valuesMap().array().square();
	//}
	//else
	//{
	  _inequality_values.segment(idx,edge->dimension()) = edge->valuesMap();
	//}
        idx += edge->dimension();
    };
}
    
    
    
    
    
void BaseSolverNonlinearProgramDense::buildObjectiveGradient()
{
    // resize gradient vector to current problem size and fill with zeros
    _objective_gradient.resize(_opt_vec_dim);
    
    // fill with zeros and add block_jacobian of each edge
    _objective_gradient.fill(0.0);
    
    // iterate objective edges
    for (EdgeType* edge : *_objectives)
    {
	// compute edge jacobian (the dimension can be greater than 1, therefore jacobian. Later we sum up all rows to get the actual gradient)
        edge->computeJacobian();
	
        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_idx = vertex->getOptVecIdx();
		
		// Take the square of each value if desired by the creator of the current edge
		if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
	        {
		    // TODO: maybe calculate complete value vector before to use it here (it was definitely calulated before, but immediately summed up))
		    edge->computeValues();
		    Eigen::VectorXd edge_values = edge->valuesMap();
		    for (int j = 0; j < vertex->dimension(); ++j)
		    {
			if (!vertex->isFixedComp(j))
			{
			    _objective_gradient.coeffRef(vert_idx++) += 2 * edge_values.dot(edge->jacobians().getWorkspace(i).col(j)); // Chain-rule: Jfsquare = 2 * f^T * Jf 
			}
		    }
		}
		else 
		{
		  
		    for (int j = 0; j < vertex->dimension(); ++j)
		    {
			if (!vertex->isFixedComp(j))
			{
			    _objective_gradient.coeffRef(vert_idx++) += edge->jacobians().getWorkspace(i).col(j).sum();
			}
		    }	  
		  
		}
            }
            else
            {
	      	// Take the square of each value if desired by the creator of the current edge
		if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
	        {
		  // TODO: maybe calculate complete value vector before to use it here (it was definitely calulated before, but immediately summed up))
		  edge->computeValues();
		  Eigen::VectorXd edge_values = edge->valuesMap();
		  // Copy complete jacobian block
		  _objective_gradient.segment(vertex->getOptVecIdx(), edge->jacobians().getWorkspace(i).cols()) += 2 * edge_values.transpose() * edge->jacobians().getWorkspace(i); // transpose is not necessary here, eigen knows how to assign
		}
		else
		{
		  // Copy complete jacobian block
		  _objective_gradient.segment(vertex->getOptVecIdx(), edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i).colwise().sum(); // transpose is not necessary here, eigen knows how to assign
		}
            }
        }
    }
    
  
}    
    
    
void BaseSolverNonlinearProgramDense::buildEqualityConstraintJacobian()
{
  
    // resize jacobian to current problem size and fill with zeros
   // _equality_jacobian.resize(_equalities_dim, _opt_vec_dim);
    
    // fill with zeros and add block_jacobian of each edge
    _equality_jacobian.fill(0.0);
    
    assert (_equalities_dim == _equality_values.rows() &&  _equalities_dim == _equality_jacobian.rows() && _opt_vec_dim == _equality_jacobian.cols());  
  
    unsigned int jac_row_idx = 0;
    
    
    for (EdgeType* edge : *_equalities)
    {
        // compute edge jacobian
        edge->computeJacobian();
	
        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_idx = vertex->getOptVecIdx();
                for (int j = 0; j < vertex->dimension(); ++j)
                {
                    if (!vertex->isFixedComp(j))
                    {
                        _equality_jacobian.block(jac_row_idx, vert_idx++, edge->dimension(), 1) +=  edge->jacobians().getWorkspace(i).col(j);
                    }
                }
            }
            else
            {

		// Copy complete jacobian block
		_equality_jacobian.block(jac_row_idx, vertex->getOptVecIdx(), edge->dimension(), edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i);
            }
        }
        jac_row_idx += edge->dimension();
    }
    
    
}    

void BaseSolverNonlinearProgramDense::buildInequalityConstraintJacobian()
{
 
  
    // resize jacobian to current problem size and fill with zeros
    //_inequality_jacobian.resize(_inequalities_dim, _opt_vec_dim);
    
    // fill with zeros and add block_jacobian of each edge
    _inequality_jacobian.fill(0.0);
  
    assert (_inequalities_dim == _inequality_values.rows() &&  _inequalities_dim == _inequality_jacobian.rows() && _opt_vec_dim == _inequality_jacobian.cols());  
    
    unsigned int jac_row_idx = 0;
    

    for (EdgeType* edge : *_inequalities)
    {
        // compute edge jacobian
        edge->computeJacobian();
	
        for (unsigned int i=0; i< edge->noVertices(); ++i)
        {
            VertexType* vertex = edge->getVertex(i);
            if (vertex->isFixedAny())
            {
                int vert_idx = vertex->getOptVecIdx();
                for (int j = 0; j < vertex->dimension(); ++j)
                {
                    if (!vertex->isFixedComp(j))
                    {
                        _inequality_jacobian.block(jac_row_idx, vert_idx++, edge->dimension(), 1) +=  edge->jacobians().getWorkspace(i).col(j);
                    }
                }
            }
            else
            {
                // Copy complete jacobian block
                _inequality_jacobian.block(jac_row_idx, vertex->getOptVecIdx(), edge->dimension(), edge->jacobians().getWorkspace(i).cols()) += edge->jacobians().getWorkspace(i);
            }
        }
        jac_row_idx += edge->dimension();
    }    
}    


void BaseSolverNonlinearProgramDense::calculateLagrangianHessian(Eigen::VectorXd* increment)
{
    switch (cfg->optim.solver.nonlin_prog.hessian.hessian_method)
    {
        case HessianMethod::NUMERIC:
            calculateLagrangianHessianNumerically();
            break;
            
        case HessianMethod::BLOCK_BFGS:
            PRINT_INFO("BLOCK_BFGS not implemented yet.");
            break;
            
        case HessianMethod::FULL_BFGS_WITH_STRUCTURE_FILTER:
            PRINT_INFO("FULL_BFGS_WITH_STRUCTURE_FILTER not implemented yet.");
        case HessianMethod::FULL_BFGS:
            calculateLagrangianHessianFullBFGS(increment); // TODO: add structure filter internally
            break;
            
        case HessianMethod::ZERO_HESSIAN: // do nothing but make sure to init hessian using initHessianBFGS zero
            break;
            
        case HessianMethod::GAUSS_NEWTON: // Use it only for pure quadratic objective functions (XXX_SQUARED function types)
            _lagrangian_hessian = _lagrangian_gradient.transpose() * _lagrangian_gradient;
            break;
            
        default:
            PRINT_ERROR("calculateLagrangianHessian() - Unknown method for hessian calculation selected.")
            break;
    }
}
    

void BaseSolverNonlinearProgramDense::calculateLagrangianHessianNumerically()
{
    // set current size
    _lagrangian_hessian.resize(_opt_vec_dim, _opt_vec_dim);
    // and fill with zeros
    _lagrangian_hessian.fill(0); // TODO: inefficient
    
    // === OBJECTIVE HESSIAN ===
    for (EdgeType* edge : *_objectives)
    {
	// skip if hessian must be zero by definition
        if (edge->functType() != FUNCT_TYPE::LINEAR && edge->functType() != FUNCT_TYPE::NONLINEAR_ONCE_DIFF )
	{
	      // compute edge jacobian
	      //edge->computeJacobian(); // WARNING assume all jacobians are called before calling calculateLagrangianHessian
	      // compute edge hessian
	      edge->computeHessian();
	      
	      // iterate all vertices attached to this edge
	      for (unsigned int edge_vert_idx=0; edge_vert_idx< edge->noVertices(); ++edge_vert_idx)
	      {
		  VertexType* vertex = edge->getVertex(edge_vert_idx);
		  
		  
		  if (vertex->isFixedAny())
		  {
		          		    
		      int vert_idx = vertex->getOptVecIdx();
		      // Iterate through all rows and columns in order to detect what is fixed...
		      for (int vert_row_idx = 0; vert_row_idx < vertex->dimension(); ++vert_row_idx)
		      {
			if (!vertex->isFixedComp(vert_row_idx))
			{
			    for (int vert_col_idx = 0; vert_col_idx < vertex->dimension(); ++vert_col_idx)
			    {
				if (!vertex->isFixedComp(vert_col_idx))
				{
				  
				  // Take the square of each value if desired by the creator of the current edge -> Use chain rule to calculate new derivative
				  if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
				  {
				      // WARNING We use gauß-newton approximation here for simplicity ... : H = 2 * J^T * J 
				      _lagrangian_hessian.coeffRef(vert_idx+vert_row_idx,vert_idx+vert_col_idx) += 2 * edge->jacobians().getWorkspace(edge_vert_idx).col(vert_row_idx).transpose() * edge->jacobians().getWorkspace(edge_vert_idx).col(vert_col_idx); // TEST indices in Jacobian
				  }
				  else
				  {
					// now iterate through all edge values, to add all values for the found vertex row-col pair at once
					for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
					{
					    _lagrangian_hessian.coeffRef(vert_idx+vert_row_idx,vert_idx+vert_col_idx) += edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx).coeffRef(vert_row_idx,vert_col_idx);
					}  
				  }
				}
			    }
			}
		      }		     
		  }
		  else
		  {
		        // iterate all edge value dimensions
			int vert_idx = vertex->getOptVecIdx();
			int vert_dim = vertex->dimension();
			
			// Take the square of each value if desired by the creator of the current edge -> Use chain rule to calculate new derivative
			if (edge->functType() == FUNCT_TYPE::NONLINEAR_SQUARED || edge->functType() == FUNCT_TYPE::LINEAR_SQUARED)
			{
			    // WARNING We use gauß-newton approximation here for simplicity ...
			    //edge->computeJacobian(); // assume jacobian is called before this complete function
			    //edge->computeValues(); // call after computeJacobian, maybe the actual value is not restored inside computeJacobian();
			    
			    _lagrangian_hessian.block(vert_idx, vert_idx, vert_dim, vert_dim) += 2 * edge->jacobians().getWorkspace(edge_vert_idx).transpose() * edge->jacobians().getWorkspace(edge_vert_idx);
			}
			else
			{
			    for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
			    {
			      // Copy complete hessian block
			      _lagrangian_hessian.block(vert_idx, vert_idx, vert_dim, vert_dim) += edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx);
			    }
			}
		  }
	      }
	  }   
    }
        
    
    
    // === EQUALITY CONSTRAINT HESSIAN WEIGHTED WITH LAGRANGE MULTIPLIERS ===
    
    assert (_equalities_dim == _equality_values.rows() &&  _equalities_dim == _multiplier_eq.rows());    
    unsigned int value_row_idx = 0;
    
    for (EdgeType* edge : *_equalities)
    {
	// skip if hessian must be zero by definition
        if (edge->functType() != FUNCT_TYPE::LINEAR && edge->functType() != FUNCT_TYPE::NONLINEAR_ONCE_DIFF )
	{
	      // compute edge jacobian
	      //edge->computeJacobian(); // WARNING assume all jacobians are called before calling calculateLagrangianHessian
	      // compute edge hessian
	      edge->computeHessian();
	      
	      // iterate all vertices attached to this edge
	      for (unsigned int edge_vert_idx=0; edge_vert_idx< edge->noVertices(); ++edge_vert_idx)
	      {
		  VertexType* vertex = edge->getVertex(edge_vert_idx);
		  
		  
		  if (vertex->isFixedAny())
		  {
		          		    
		      int vert_idx = vertex->getOptVecIdx();
		      // Iterate through all rows and columns in order to detect what is fixed...
		      for (int vert_row_idx = 0; vert_row_idx < vertex->dimension(); ++vert_row_idx)
		      {
			if (!vertex->isFixedComp(vert_row_idx))
			{
			    for (int vert_col_idx = 0; vert_col_idx < vertex->dimension(); ++vert_col_idx)
			    {
				if (!vertex->isFixedComp(vert_col_idx))
				{
				   // now iterate through all edge values, to add all values for the found vertex row-col pair at once
				   for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
				   {
				      _lagrangian_hessian.coeffRef(vert_idx+vert_row_idx,vert_idx+vert_col_idx) += _multiplier_eq.coeffRef(value_row_idx+edge_val_idx) * edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx).coeffRef(vert_row_idx,vert_col_idx);
				   }  
				}
			    }
			}
		      }		     
		  }
		  else
		  {
		        // iterate all edge value dimensions
			int vert_idx = vertex->getOptVecIdx();
			int vert_dim = vertex->dimension();
			for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
			{
			  // Copy complete hessian block and scale each row with the correlating multiplier
			  _lagrangian_hessian.block(vert_idx, vert_idx, vert_dim, vert_dim) += _multiplier_eq.coeffRef(value_row_idx+edge_val_idx) * edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx);
			}
		  }
	      }
	  }   
	  
	  value_row_idx += edge->dimension();
    }    
    
    
    
    
// === INEQUALITY CONSTRAINT HESSIAN WEIGHTED WITH LAGRANGE MULTIPLIERS ===
    
    assert (_inequalities_dim == _inequality_values.rows() &&  _inequalities_dim == _multiplier_ineq.rows());  
    value_row_idx = 0;
    
    for (EdgeType* edge : *_inequalities)
    {
	// skip if hessian must be zero by definition
        if (edge->functType() != FUNCT_TYPE::LINEAR && edge->functType() != FUNCT_TYPE::NONLINEAR_ONCE_DIFF )
	{
	      // compute edge jacobian
	      //edge->computeJacobian(); // WARNING assume all jacobians are called before calling calculateLagrangianHessian
	      // compute edge hessian
	      edge->computeHessian();
	      
	      // iterate all vertices attached to this edge
	      for (unsigned int edge_vert_idx=0; edge_vert_idx< edge->noVertices(); ++edge_vert_idx)
	      {
		  VertexType* vertex = edge->getVertex(edge_vert_idx);
		  
		  
		  if (vertex->isFixedAny())
		  {
		          		    
		      int vert_idx = vertex->getOptVecIdx();
		      // Iterate through all rows and columns in order to detect what is fixed...
		      for (int vert_row_idx = 0; vert_row_idx < vertex->dimension(); ++vert_row_idx)
		      {
			if (!vertex->isFixedComp(vert_row_idx))
			{
			    for (int vert_col_idx = 0; vert_col_idx < vertex->dimension(); ++vert_col_idx)
			    {
				if (!vertex->isFixedComp(vert_col_idx))
				{
				   // now iterate through all edge values, to add all values for the found vertex row-col pair at once
				   for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
				   {
				      _lagrangian_hessian.coeffRef(vert_idx+vert_row_idx,vert_idx+vert_col_idx) += _multiplier_ineq.coeffRef(value_row_idx+edge_val_idx) * edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx).coeffRef(vert_row_idx,vert_col_idx);
				   }  
				}
			    }
			}
		      }		     
		  }
		  else
		  {
		        // iterate all edge value dimensions
			int vert_idx = vertex->getOptVecIdx();
			int vert_dim = vertex->dimension();
			for (int edge_val_idx = 0; edge_val_idx < edge->dimension(); ++edge_val_idx)
			{
			  // Copy complete hessian block and scale each row with the correlating multiplier
			  _lagrangian_hessian.block(vert_idx, vert_idx, vert_dim, vert_dim) += _multiplier_ineq.coeffRef(value_row_idx+edge_val_idx) * edge->hessians().getWorkspace(edge_vert_idx, edge_val_idx);
			}
		  }
	      }
	  }   
	  
	  value_row_idx += edge->dimension();
    }        
    
}

    
 // WARNING: calculateLagrangianGradient has to be called before
void BaseSolverNonlinearProgramDense::initHessianBFGS()
{
    // set current size
    _lagrangian_hessian.resize(_opt_vec_dim, _opt_vec_dim);
    
    switch (cfg->optim.solver.nonlin_prog.hessian.hessian_init)
    {
        case HessianInit::NUMERIC:
            calculateLagrangianHessianNumerically();
            break;
            
        case HessianInit::ZERO:
            // fill with zeros
            _lagrangian_hessian.fill(0); // TODO: inefficient
            break;
            
        case HessianInit::IDENTITY:
            // set diagonal matrix with elemens specified by Config::Optim::Solver::NonlinearProgramm::hessian_init_identity_scale.
            _lagrangian_hessian.setIdentity().diagonal().setConstant(cfg->optim.solver.nonlin_prog.hessian.hessian_init_identity_scale);
            break;
            
        default:
            PRINT_ERROR("initHessianBFGS() - Unknown method for hessian initialization selected.")
            break;
    }
    
    // Store current gradient
    _lagrangian_gradient_backup = _lagrangian_gradient; // TODO: maybe more efficient without the extra copy.
}
    

void BaseSolverNonlinearProgramDense::calculateLagrangianHessianFullBFGS(Eigen::VectorXd* increment)
{
    assert(increment!=nullptr);
    
    // calcute update
    Eigen::VectorXd gradient_diff = _lagrangian_gradient - _lagrangian_gradient_backup;
    
    double aux_denum1 = gradient_diff.dot(*increment); // denumerator of summand 1 (see BFGS update equation)
    
    if (fabs(aux_denum1)<1e-5)
    {
        PRINT_DEBUG("calculateLagrangianHessianFullBFGS() - skipping update.");
    }
    else
    {

        double aux_denum2 = increment->transpose() * _lagrangian_hessian * (*increment);
        if (cfg->optim.solver.nonlin_prog.hessian.bfgs_damped_mode)
        {
            // see Nocedal, Jorge; Wright, Stephen J. (2006), Numerical Optimization (2nd ed.), Berlin, New York: Springer-Verlag, ISBN 978-0-387-30303-1
            double theta = 1;
            bool ndamped = aux_denum1 >= 0.2*aux_denum2;
            if (!ndamped) theta = 0.8*aux_denum2/(aux_denum2-aux_denum1);
            gradient_diff = theta*gradient_diff + (1-theta)*_lagrangian_hessian* (*increment); // linear interpolation that ensures positive definiteness
        }
        else
        {
            // see http://www.mathworks.de/de/help/optim/ug/constrained-nonlinear-optimization-algorithms.html?refresh=true
            // mabye incorporate modifications presented in http://ccom.ucsd.edu/~optimizers/papers/siam_44609.pdf (but not implemented yet)
            
            // check if aux_denum1 is positive to ensure the positive definiteness of H
            if (aux_denum1<0)
            {
                PRINT_DEBUG("calculateLagrangianHessianFullBFGS() - hessian indefinite, try to fix");
                // first approach:
                for (unsigned int i=0; i<100; ++i) // hard coded number of max tries
                {
                  // find minimum value of y.*increment and correct line   // TODO/TEST: corret? or min(y)
                  Eigen::VectorXd::Index min_idx;
                  gradient_diff.cwiseProduct(*increment).minCoeff(&min_idx);
                  // now modify this value and hope that it will make the matrix positive definite
                  gradient_diff.coeffRef(min_idx) /= 2;
                  aux_denum1 = gradient_diff.dot(*increment);
                  if (aux_denum1>0) break; // should be positive definite now
                }
                
                // if still indefinite, try to add small offset
                if (aux_denum1<0)
                {
                    PRINT_DEBUG("calculateLagrangianHessianFullBFGS() - first try to make hessian positive definite failed, trying different approach.");
                    double w = 0.0001;
                    for (unsigned int i=0; i<3; ++i) // hard coded number of max tries
                    {
                        gradient_diff.array() += w; // this is actually not what we want, since we only want to correct single values of the gradient_diff.
                                                    // But matlab's approach didn't worked for me, so we try this first. //TODO fix in the future
                        aux_denum1 = gradient_diff.dot(*increment);
                        if (aux_denum1>0) break; // should be positive definite now
                        w*=10; // hard-coded factor
                    }
                }
                
                if (aux_denum1<0)
                {
                    PRINT_DEBUG("calculateLagrangianHessianFullBFGS() - Failed to force hessian to be positive definite.");
                }
            }
        }
        
        // Now update hessian
       _lagrangian_hessian += ( gradient_diff * gradient_diff.transpose() ) / aux_denum1;
       _lagrangian_hessian += (_lagrangian_hessian * (*increment) * increment->transpose() * _lagrangian_hessian) / aux_denum2;
        
        // TODO: maybe filter values that are near zero?

        
    }
    
    
    // Backup current gradient for the subsequent iteration
    _lagrangian_gradient_backup = _lagrangian_gradient; // TODO: maybe more efficient without the extra copy.
    
}
    
    
       
} // end namespace teb

