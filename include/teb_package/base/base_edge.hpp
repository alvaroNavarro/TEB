#include <teb_package/base/base_edge.h>

namespace teb
{


/**
 * This implementation calculates the Block-Jacobians for all
 * statically allocated vertices numerically using central differences.
 * For more information about the Jacobian structure
 * refer to JacobianWorkspace.
 *
 * The results are stored to the class member BaseEdgeBinary::_jacobians.
 *
 * The implementation is similar to \cite kummerle2011.
 *
 * Feel free to reimplement this function in child classes
 * in order to provide analytical Jacobians.
 *
 *
 * @sa computeHessian(), JacobianWorkspace
 */
template <int D, FUNCT_TYPE FuncType, class... Vertices>
void BaseEdge<D, FuncType, Vertices...>::computeJacobian()
{
  
  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);

  ValueVector values1;
  ValueVector valueBeforeNumeric = _values;
  
  for (unsigned int i=0; i < _vertices.size(); ++i)
  {
      VertexType* sample_i = static_cast<VertexType*>(_vertices[i]);

      if (sample_i->isFixedAll())
          continue;
              
  #ifdef _MSC_VER
      double* add_sample_i = new double [sample_i->dimension()];
  #else
      double add_sample_i[sample_i->dimension()];
  #endif
      std::fill(add_sample_i, add_sample_i + sample_i->dimension(), 0.0);
  
      // add small step along the unit vector in each dimension
      for (int d=0; d < sample_i->dimension(); ++d)
      {
          // TODO: check which strategy is more efficient:
          // either skip fixed states here (or just calculate it) -> partially fixed states are rare
          // or perform as follows without checking if(!sample_i->isFixed)
          sample_i->push();
          add_sample_i[d] = delta;
          sample_i->plus(add_sample_i);
          computeValues();
          values1 = _values;
          sample_i->pop();
          sample_i->push();
          add_sample_i[d] = -delta;
          sample_i->plus(add_sample_i);
          computeValues();
          sample_i->pop();
          add_sample_i[d] = 0.0;
          
          _jacobians.getWorkspace(i).col(d) = scalar * (values1 - _values);
      }
      #ifdef _MSC_VER
    delete[] add_sample_i;
  #endif
  }
}


/**
 * This implementation calculates the Block-Jacobians for all
 * dynamically allocated vertices numerically using central differences.
 * For more information about the Jacobian structure
 * refer to JacobianWorkspace.
 *
 * The results are stored to the class member BaseEdgeBinary::_jacobians.
 *
 * The implementation is similar to \cite kummerle2011.
 *
 * Feel free to reimplement this function in child classes
 * in order to provide analytical Jacobians.
 *
 *
 * @sa computeHessian(), JacobianWorkspace
 */
template <int D, FUNCT_TYPE FuncType>
void BaseEdge<D, FuncType>::computeJacobian()
{

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);

  ValueVector values1;
  ValueVector valueBeforeNumeric = _values;
  
  for (unsigned int i=0; i < _vertices.size(); ++i)
  {
      VertexType* sample_i = static_cast<VertexType*>(_vertices[i]);

      if (sample_i->isFixedAll())
          continue;
              
  #ifdef _MSC_VER
      double* add_sample_i = new double [sample_i->dimension()];
  #else
      double add_sample_i[sample_i->dimension()];
  #endif
      std::fill(add_sample_i, add_sample_i + sample_i->dimension(), 0.0);
  
      // add small step along the unit vector in each dimension
      for (int d=0; d < sample_i->dimension(); ++d)
      {
          // TODO: check which strategy is more efficient:
          // either skip fixed states here (or just calculate it) -> partially fixed states are rare
          // or perform as follows without checking if(!sample_i->isFixed)
          sample_i->push();
          add_sample_i[d] = delta;
          sample_i->plus(add_sample_i);
          computeValues();
          values1 = _values;
          sample_i->pop();
          sample_i->push();
          add_sample_i[d] = -delta;
          sample_i->plus(add_sample_i);
          computeValues();
          sample_i->pop();
          add_sample_i[d] = 0.0;
          
          _jacobians.getWorkspace(i).col(d) = scalar * (values1 - _values);
      }
      #ifdef _MSC_VER
    delete[] add_sample_i;
  #endif
  }
}
  
  
/**
 * This implementation calculates the Block-Hessians for all
 * statically allocated vertices and cost/constraint values numerically
 * using forward differences.
 * For more information about the Hessian structure
 * refer to HessianWorkspace.
 *
 * If the FuncType template parameter is set to FUNCT_TYPE::LINEAR or
 * FuncType == FUNCT_TYPE::NONLINEAR_ONCE_DIFF the hessian is set to zero.
 *
 * The results are stored to the class member BaseEdgeBinary::_hessians.
 *
 * Feel free to reimplement this function in child classes
 * in order to provide analytical Hessians.
 *
 * @remarks This function assumes that computeJacobian() was called before!
 * 	      It is used as reference for calculating the forward-differences.
 *
 * @sa computeJacobian(), HessianWorkspace
 */

template <int D, FUNCT_TYPE FuncType, class... Vertices>
void BaseEdge<D, FuncType, Vertices...>::computeHessian()
{
  
    // Special handling of zero hessian function types
    if (FuncType == FUNCT_TYPE::LINEAR || FuncType == FUNCT_TYPE::NONLINEAR_ONCE_DIFF || FuncType == FUNCT_TYPE::LINEAR_SQUARED)
    {
      for (unsigned int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
      {
	for (int val_idx=0; val_idx< D; ++val_idx)
	{
	      _hessians.getWorkspace(sample_idx,val_idx).setZero();
	}
      }
      return;
    }
    /* We do not know the weight...
    else if (FuncType == FUNCT_TYPE::QUADRATIC)
    {
      for (int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
      {
	for (int val_idx=0; val_idx< D; ++val_idx)
	{
	      _hessians.getWorkspace(sample_idx,val_idx).setIdentity().diagonal().setConstant(2);
	}
      }
      return;    
    }
    */
    // Otherwise calculate numerically
  
  
    // backup jacobians (allows to use calculateJacobian in order to calculate hessian via finite differences)
    // assume calcJacobian was calculated before in, therefore use backup_jacobian as current estimate. TODO: ALWAYS TRUE?
    JacobianWorkspace jacobians_backup = _jacobians;
    
    
    // parameter
    const double delta = 1e-2;
    const double scalar = 1.0 / delta;
            
    // approximate by forward differences:
    for (unsigned int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
    {
        VertexType* sample_i = static_cast<VertexType*>(_vertices[sample_idx]);
        
        if (sample_i->isFixedAll())
            continue;
    
    #ifdef _MSC_VER
    double* add_sample_i = new double [sample_i->dimension()];
    #else
    double add_sample_i[sample_i->dimension()];
    #endif
    std::fill(add_sample_i, add_sample_i + sample_i->dimension(), 0.0);
        
        // add small step along the unit vector in each dimension
        for (int d=0; d < sample_i->dimension(); ++d)
        {
            sample_i->push();
            add_sample_i[d] = delta;
            sample_i->plus(add_sample_i);
            computeJacobian();
            sample_i->pop();
            add_sample_i[d] = 0.0;
            
            // get column for each hessian for each value
            for (int val_idx=0; val_idx<D; ++val_idx)
            {
                _hessians.getWorkspace(sample_idx,val_idx).col(d) = scalar *
                (_jacobians.getWorkspace(sample_idx).row(val_idx) - jacobians_backup.getWorkspace(sample_idx).row(val_idx)).transpose();
            }
            
        }
        
        // force symmetry: 0.5*(H+H') // TODO: improve performance!!!
        for (int val_idx=0; val_idx< D; ++val_idx)
        {
            _hessians.getWorkspace(sample_idx,val_idx) = ( ( _hessians.getWorkspace(sample_idx,val_idx) + _hessians.getWorkspace(sample_idx,val_idx).transpose() )/2 ).eval(); // force eigen to avoid lazy evaluation using eval() -> http://eigen.tuxfamily.org/dox/TopicLazyEvaluation.html, but implies internal copy!
        }
        
      #ifdef _MSC_VER
    delete[] add_sample_i;
  #endif     
    }
    
    // restore original jacobians
    _jacobians = jacobians_backup;
}    
 
  
  
/**
 * This implementation calculates the Block-Hessians for all
 * dynamically allocated vertices and cost/constraint values numerically
 * using forward differences.
 * For more information about the Hessian structure
 * refer to HessianWorkspace.
 * 
 * If the FuncType template parameter is set to FUNCT_TYPE::LINEAR or
 * FuncType == FUNCT_TYPE::NONLINEAR_ONCE_DIFF the hessian is set to zero.
 *
 * The results are stored to the class member BaseEdgeBinary::_hessians.
 *
 * Feel free to reimplement this function in child classes
 * in order to provide analytical Hessians.
 *
 * @remarks This function assumes that computeJacobian() was called before!
 * 	      It is used as reference for calculating the forward-differences.
 *
 * @sa computeJacobian(), HessianWorkspace
 */
template <int D, FUNCT_TYPE FuncType>
void BaseEdge<D, FuncType>::computeHessian()
{
  
    // Special handling of zero hessian function types
    if (FuncType == FUNCT_TYPE::LINEAR || FuncType == FUNCT_TYPE::NONLINEAR_ONCE_DIFF || FuncType == FUNCT_TYPE::LINEAR_SQUARED)
    {
      for (int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
      {
	for (int val_idx=0; val_idx< D; ++val_idx)
	{
	      _hessians.getWorkspace(sample_idx,val_idx).setZero();
	}
      }
      return;
    }
    /* We do not know the weight...
    else if (FuncType == FUNCT_TYPE::QUADRATIC)
    {
      for (int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
      {
	for (int val_idx=0; val_idx< D; ++val_idx)
	{
	      _hessians.getWorkspace(sample_idx,val_idx).setIdentity().diagonal().setConstant(2);
	}
      }
      return;    
    }
    */
  
    // Otherwise calculate numerically
  
  
    // backup jacobians (allows to use calculateJacobian in order to calculate hessian via finite differences)
    // assume calcJacobian was calculated before in, therefore use backup_jacobian as current estimate. TODO: ALWAYS TRUE?
    JacobianWorkspace jacobians_backup = _jacobians;
    
    
    // parameter
    const double delta = 1e-2;
    const double scalar = 1.0 / delta;
            
    // approximate by forward differences:
    for (int sample_idx=0; sample_idx<_vertices.size(); ++sample_idx)
    {
        VertexType* sample_i = static_cast<VertexType*>(_vertices[sample_idx]);
        
        if (sample_i->isFixedAll())
            continue;
    
    #ifdef _MSC_VER
    double* add_sample_i = new double [sample_i->dimension()];
    #else
    double add_sample_i[sample_i->dimension()];
    #endif
    std::fill(add_sample_i, add_sample_i + sample_i->dimension(), 0.0);
        
        // add small step along the unit vector in each dimension
        for (int d=0; d < sample_i->dimension(); ++d)
        {
            sample_i->push();
            add_sample_i[d] = delta;
            sample_i->plus(add_sample_i);
            computeJacobian();
            sample_i->pop();
            add_sample_i[d] = 0.0;
            
            // get column for each hessian for each value
            for (int val_idx=0; val_idx<D; ++val_idx)
            {
                _hessians.getWorkspace(sample_idx,val_idx).col(d) = scalar *
                (_jacobians.getWorkspace(sample_idx).row(val_idx) - jacobians_backup.getWorkspace(sample_idx).row(val_idx)).transpose();
            }
            
        }
        
        // force symmetry: 0.5*(H+H') // TODO: improve performance!!!
        for (int val_idx=0; val_idx< D; ++val_idx)
        {
            _hessians.getWorkspace(sample_idx,val_idx) = ( ( _hessians.getWorkspace(sample_idx,val_idx) + _hessians.getWorkspace(sample_idx,val_idx).transpose() )/2 ).eval(); // force eigen to avoid lazy evaluation using eval() -> http://eigen.tuxfamily.org/dox/TopicLazyEvaluation.html, but implies internal copy!
        }
        
      #ifdef _MSC_VER
    delete[] add_sample_i;
  #endif     
    }
    
    // restore original jacobians
    _jacobians = jacobians_backup;
}  
  

} // end namespace teb



