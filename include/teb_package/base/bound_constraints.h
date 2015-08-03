#ifndef __teb_package__constraint_edges__
#define __teb_package__constraint_edges__

#include <teb_package/base/base_edge.h>

#include <type_traits>
#include <tuple>

namespace teb
{


//! Enumerator class that stores all types of bounds for consideration in BoundConstraint class
enum class BOUND_TYPE {LOWER, UPPER, LOWERUPPER};
//! Enumerator class that stores all types of variables that can be bounded using BoundConstraint class
enum class BOUND_VARS {ALL, SINGLE};

//! \internal Helper function for BoundConstraint: Calculate number of bounds depending on the template arguments (DOES NOT WORK WITH VISUAL STUDIO 2013 WITHOUT PATCHES
//constexpr int noBounds(BOUND_TYPE Bound_type, int Index, int var_dim) { return Index >= 0 ? (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 : 1) : (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 * var_dim : var_dim); }

    
//! Datastruct for custom bound data that can be queried from the BoundConstraint class.
struct CustomBoundData
{
    int index = -1; //!< Index for the bound (-1 for multiple components)
    int no_lower = 0; //!< Number of lower bounds
    Eigen::VectorXd lower; //!< [no_lower x 1] lower bounds vector
    int no_upper = 0; //!< Number of upper bounds
    Eigen::VectorXd upper; //!< [no_upper x 1] upper bounds vector
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
    
    
/**
 * @brief This class captures general bound constraints on optimization variables / vertices.
 *
 * @ingroup controller
 *
 * Use this class to create bound constraints \f$ \mathbf{x}_{min} \le \mathbf{x} \le \mathbf{x}_{max} \f$
 *
 * @remarks Call allocateMemory() before using the edge for optimization.
 * @todo Maybe use a special representation or zero Hessians to avoid processing a lot of zeros (same for other edges)
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @sa ConstraintEdgeUnary, ConstraintEdgeBinary, ConstraintEdgeMulti, ObjectiveEdgeUnary, ObjectiveEdgeBinary, ObjectiveEdgeMulti
 *
 * @tparam Bound_type Lower or upper bound corresponding to BOUND_TYPE enum
 * @tparam Vertex Type of the attached vertex.
 * @tparam Bound_vars Specify whether a single state or all components of the VertexType should be bounded (See BOUND_VARS enum).
 * @tparam Index If \c Bound_vars==BOUND_VARS::SINGLE then this Index specifies the element/component, otherwise set \c Index to -1.
 */
template <BOUND_TYPE Bound_type, typename Vertex, BOUND_VARS Bound_vars, int Index = -1>
class BoundConstraint : public BaseEdge< (Index >= 0 ? (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 : 1) : (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 * Vertex::Dimension : Vertex::Dimension)), FUNCT_TYPE::LINEAR, Vertex>
{
 
  // Check template parameters
  static_assert( (Index>=0 && Bound_vars==BOUND_VARS::SINGLE) || (Index<0 && Bound_vars!=BOUND_VARS::SINGLE), "Index must be specified if Bound_vars == BOUND_VARS::SINGLE");
  
public:



  static const int NoBounds = Index >= 0 ? (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 : 1) : (Bound_type == BOUND_TYPE::LOWERUPPER ? 2 * Vertex::Dimension : Vertex::Dimension); //!< Store number of bounds.
  static const BOUND_TYPE BoundType = Bound_type; //!< Store type of bound (lower, upper, or both);
  
  
  //! Typedef to represent the static Eigen::Vector of cost/constraint values.
  using ValueVector = typename BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>::ValueVector;
  
  /**
   * @brief Construct edge and attach the corresponding vertex.
   * @param vertex Reference to the vertex that should be bounded
   */
  BoundConstraint(Vertex& vertex) : BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>(vertex)
  {
     // Template parameter checks
     //static_assert(std::is_base_of<VertexType, Vertex>::value,"Template parameter 'Vertex' must be derived from VertexType.");
  }
  
   //! Override this method to specify this edge as a bound constraint since it can be handled seperately by some solvers to speed up optimization
   virtual const bool isBoundConstraint() const {return true;}
  
  virtual void computeValues() // See EdgeType::computeValues()
  {
      const Vertex* vertex = static_cast<const Vertex*>(_vertices[0]);
      
      
      if (Bound_type == BOUND_TYPE::LOWERUPPER)
      {
	  if (Bound_vars == BOUND_VARS::SINGLE)
	  {
	      _values[0] = -vertex->getData(Index) + _bounds.coeffRef(0); // x > x_min -> -x + x_min < 0	    
	      _values[1] =  vertex->getData(Index) - _bounds.coeffRef(1); // x < x_max -> x - x_max < 0
	  }
	  else
	  {
	    for (unsigned int i=0; i<Vertex::Dimension; ++i)
	    {
	      	_values[2*i]   =  -vertex->getData(i) + _bounds.coeffRef(i); // u > u_min -> -u + u_min < 0
            _values[2*i+1] =   vertex->getData(i) - _bounds.coeffRef(i + Vertex::Dimension); // u < u_max -> u - u_max < 0
	    }
	  }
	}
	else // LOWER or UPPER bound only
	{
	  if (Bound_vars == BOUND_VARS::SINGLE)
	  {
	    _values[0] =  (vertex->getData(Index) - _bounds.coeffRef(0)) * ( Bound_type == BOUND_TYPE::LOWER ? -1 : 1 );
	  }
	  else
	  {
	    for (unsigned int i=0; i<Vertex::Dimension; ++i)
	    {
	      _values[i] =  (vertex->getData(i) - _bounds.coeffRef(i)) * ( Bound_type == BOUND_TYPE::LOWER ? -1 : 1 );
	    }	    
	  }
	}
      
            
  };

  //! Specification of the analytic block jacobian
  virtual void computeJacobian()
  {
    
      _jacobians.getWorkspace(0).fill(0.0);
      
      if (Bound_type == BOUND_TYPE::LOWERUPPER)
      {
          if (Bound_vars == BOUND_VARS::SINGLE)
          {
                _jacobians.getWorkspace(0).coeffRef(0,Index) = -1;
                _jacobians.getWorkspace(0).coeffRef(1,Index) = 1;
          }
          else
          {
            for (unsigned int i=0; i<Vertex::Dimension; ++i)
            {
                _jacobians.getWorkspace(0).coeffRef(2*i,i) = -1;
                _jacobians.getWorkspace(0).coeffRef(2*i+1,i) = 1;
            }
          }
	}
	else // LOWER or UPPER bound only
	{
	  if (Bound_vars == BOUND_VARS::SINGLE)
	  {
	    _jacobians.getWorkspace(0).coeffRef(0,Index) = ( Bound_type == BOUND_TYPE::LOWER ? -1 : 1 ); 
	  }
	  else
	  {
	    for (unsigned int i=0; i<Vertex::Dimension; ++i)
	    {
	      _jacobians.getWorkspace(0).coeffRef(i,i) = ( Bound_type == BOUND_TYPE::LOWER ? -1 : 1 );
	    }	    
	  }
	}      
  };

  //! Specification of the analytic block hessian
  virtual void computeHessian()
  {
    
    // zero hessian for all values
    for (unsigned int i=0; i<Vertex::Dimension; ++i)
    {
        _hessians.getWorkspace(0,i).fill(0.0);
    }
    
  }
  
  /**
   * @brief Set bounds for either BOUND_TYPE::LOWER or BOUND_TYPE::UPPER.
   * Set an array of the bounds. The length depends on the class' template arguments
   * BOUND_TYPE \c Bound_type and BOUND_VARS \c Bound_vars :
   * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::SINGLE : 1 
   * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::ALL : Vertex::Dimension 
   * @param bounds pointer to the bound-array.
   */
  void setBounds(const double* bounds)
  {
    static_assert(Bound_type!=BOUND_TYPE::LOWERUPPER,"Please use setBounds(lb,ub) to specify bounds for BOUND_TYPE==LOWERUPPER");
    _bounds = Eigen::Matrix<double, NoBounds, 1>(bounds);
  }
    
   /**
    * @brief Set bounds for either BOUND_TYPE::LOWER or BOUND_TYPE::UPPER.
    * Set an array of the bounds. The length depends on the class' template arguments
    * BOUND_TYPE \c Bound_type and BOUND_VARS \c Bound_vars :
    * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::SINGLE : 1
    * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::ALL : Vertex::Dimension
    * @param bounds Eigen Vector containing the bound.
    */
    void setBounds(const Eigen::Ref<const Eigen::Matrix<double, NoBounds, 1>>& bounds)
    {
        static_assert(Bound_type!=BOUND_TYPE::LOWERUPPER,"Please use setBounds(lb,ub) to specify bounds for BOUND_TYPE==LOWERUPPER");
        _bounds = bounds;
    }
  
  /**
   * @brief Set single bound for either BOUND_TYPE::LOWER or BOUND_TYPE::UPPER.
   * Set a single bound. BOUND_VARS::SINGLE and \c Index>=0 template parameter is required!
   * @param bound bound-value
   */
  void setBounds(double bound)
  {
    static_assert(Bound_type!=BOUND_TYPE::LOWERUPPER && Bound_vars==BOUND_VARS::SINGLE,"Please use setBounds(bounds) to specify bounds for BOUND_TYPE==LOWERUPPER and Bound_type==BOUND_VARS::SINGLE");
      _bounds[0] = bound;
  }
  
  /**
   * @brief Set single bound forBOUND_TYPE::LOWERUPPER.
   * Set a single bound. BOUND_VARS::SINGLE and \c Index>=0 template parameter are required!
   * @param lb lower bound-value
   * @param ub upper bound-value
   */
  void setBounds(double lb, double ub)
  {
      static_assert(Bound_type==BOUND_TYPE::LOWERUPPER && Bound_vars==BOUND_VARS::SINGLE,"Please use setBounds(bounds) to specify bounds for BOUND_TYPE!=LOWERUPPER and Bound_type==BOUND_VARS::SINGLE");
      _bounds[0] = lb;
      _bounds[1] = ub;
  }
  
  /**
   * @brief Set lower and upper bounds for BOUND_TYPE::LOWERUPPER.
   * Set an array of the bounds. The length depends on the class' template arguments
   * BOUND_TYPE \c Bound_type and BOUND_VARS \c Bound_vars :
   * 	- BOUND_TYPE::LOWERUPPER && BOUND_VARS::SINGLE : 2 
   * 	- BOUND_TYPE::LOWERUPPER && BOUND_VARS::ALL : 2*Vertex::Dimension 
   * @param lb pointer to the lower bound-array.
   * @param ub pointer to the upper bound-array.
   */
  void setBounds(const double* lb, const double* ub)
  {
    static_assert(Bound_type==BOUND_TYPE::LOWERUPPER,"Please use setBounds(bounds) to specify bounds for BOUND_TYPE!=LOWERUPPER");
    if (Bound_vars == BOUND_VARS::SINGLE)
    {
      _bounds[0] = *lb;
      _bounds[1] = *ub;
    }
   else
    {
      for (unsigned int i=0; i<Vertex::Dimension; ++i)
      {
	      _bounds[i] = *(lb+i);
	      _bounds[i + Vertex::Dimension] = *(ub+i);
      }	    
    }  
  }
  
    /**
     * @brief Set lower and upper bounds for BOUND_TYPE::LOWERUPPER.
     * Set an array of the bounds. The length depends on the class' template arguments
     * BOUND_TYPE \c Bound_type and BOUND_VARS \c Bound_vars :
     * 	- BOUND_TYPE::LOWERUPPER && BOUND_VARS::SINGLE : 2
     * 	- BOUND_TYPE::LOWERUPPER && BOUND_VARS::ALL : 2*Vertex::Dimension
     * @param lb Eigen::Vector [NoBounds x 1] for the lower bound.
     * @param ub Eigen::Vector [NoBounds x 1] for the upper bound.
     */
    void setBounds(const Eigen::Ref<const Eigen::Matrix<double, NoBounds/2, 1>>& lb, const Eigen::Ref<const Eigen::Matrix<double, NoBounds/2, 1>>& ub)
    {
        static_assert(Bound_type==BOUND_TYPE::LOWERUPPER,"Please use setBounds(bounds) to specify bounds for BOUND_TYPE!=LOWERUPPER");
        _bounds.head((int) NoBounds/2) = lb;
        _bounds.tail((int) NoBounds/2) = ub;
    }
    
  /**
   * @brief get a constant pointer to bound data.
   * See setBounds() overloads for more information about the length.
   * In summary:
   * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::SINGLE : 1 
   * 	- BOUND_TYPE::LOWER / BOUND_TYPE::UPPER && BOUND_VARS::ALL : Vertex::Dimension 
   * 
   *    - BOUND_TYPE::LOWERUPPER && BOUND_VARS::SINGLE : 2 
   * 	- BOUND_TYPE::LOWERUPPER && BOUND_VARS::ALL : 2*Vertex::Dimension 
   * 
   * In case of BOUND_TYPE::LOWERUPPER first all lower bounds are stored and then all upper ones.
   * 
   * @return constant pointer to bound data
   */
  const double* getBounds() const
  {
    return _bounds.data();
  }
    
  
  /**
   * @brief Return bound information.
   * This function overwrites the default interface to share custom data with edges.
   * It avoids dynamic casting with known (!!!) template parameters.
   * Cast the output to a CustomBoundData struct:
   * @code
   * 	EdgeType* edge = new BoundConstaint<>(); // Do not forget to set bounds afterwards ...
   * 	CustomBoundData* bound_data = static_cast<CustomBoundData*>(edge->getCustomData());
   *    // Do something with bound_data.
   *    delete bound_data;
   * @endcode
   * Do not forget to delete the \c bound_data object later
   * 
   * The bound data returned shares always the same size like \c vertex->dimensionFree. \n
   * All Bounds that are not considered in this class (like for Index>0) are set to
   * -INF or INF depending on ther type.
   * 
   * @returns Void-pointer to CustomBoundData object on the heap.
   */
  virtual void* getCustomData()
  {
     assert(_vertices.size() == 1);
     CustomBoundData* bound_data = new CustomBoundData;
     bound_data->index = Index;
     bound_data->lower.setConstant(_vertices.front()->dimensionFree(), -INF );
     bound_data->upper.setConstant(_vertices.front()->dimensionFree(), INF );
      switch (Bound_type)
      {
	case BOUND_TYPE::LOWER:
	  bound_data->no_lower = _vertices.front()->dimensionFree();
	  if (Index>=0 && !_vertices.front()->isFixedComp(Index))
	  {
 	     bound_data->lower.coeffRef(Index) = _bounds[0];
	  }
	  else if (Index<0)
	  {
	      unsigned int idx_free = 0;
		  for (unsigned int i = 0; i < (unsigned int) _vertices.front()->dimension(); ++i)
	      {
 		if (!_vertices.front()->isFixedComp(i)) { bound_data->lower.coeffRef(idx_free++) = _bounds[i];};
	      }
	  }
	  break;
	case BOUND_TYPE::UPPER:
	  bound_data->no_upper =  _vertices.front()->dimensionFree();
	  if (Index>=0 && !_vertices.front()->isFixedComp(Index))
	  {
 	      bound_data->upper.coeffRef(Index) = _bounds[0];
	  }
	  else if (Index<0)
	  {
	      unsigned int idx_free = 0;
		  for (unsigned int i = 0; i < (unsigned int) _vertices.front()->dimension(); ++i)
	      {
 		if (!_vertices.front()->isFixedComp(i)) { bound_data->upper.coeffRef(idx_free++) = _bounds[i];};
	      }
	  }
	  break;
	case BOUND_TYPE::LOWERUPPER:
              static_assert((Bound_type==BOUND_TYPE::LOWERUPPER && NoBounds%2==0) || (Bound_type!=BOUND_TYPE::LOWERUPPER && (NoBounds%2!=0 || Vertex::Dimension%2==0)),"This should not appear...");
	  bound_data->no_lower =  _vertices.front()->dimensionFree();
	  bound_data->no_upper =  _vertices.front()->dimensionFree();
	  if (Index>=0 && !_vertices.front()->isFixedComp(Index))
	  {
 	      bound_data->lower.coeffRef(Index) = _bounds[0];
 	      bound_data->upper.coeffRef(Index) = _bounds[1];
	  }
	  else if (Index<0)
	  {
	      unsigned int idx_free = 0;
		  for (unsigned int i = 0; i < (unsigned int) _vertices.front()->dimension(); ++i)
	      {
		if (!_vertices.front()->isFixedComp(i))
		{
 		  bound_data->lower.coeffRef(idx_free) = _bounds[i];
 		  bound_data->upper.coeffRef(idx_free) = _bounds[i+_vertices.front()->dimension()];
		  ++idx_free;
		};
	      }
	  }
	  break;
      }
    return bound_data;
  }
    
    
protected:

  Eigen::Matrix<double, NoBounds, 1> _bounds; //!< Store bounds: first lower and then upper bound, depending on Bound_type parameter.

  // Use members from the parent template class.
  using BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>::_vertices;
  using BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>::_values;
  using BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>::_jacobians;
  using BaseEdge<NoBounds, FUNCT_TYPE::LINEAR, Vertex>::_hessians;
      
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};




    
} // end namespace teb


#endif /* defined(__teb_package__constraint_edges__) */
