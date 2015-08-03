#ifndef __teb_package__base_edge__
#define __teb_package__base_edge__

#include <teb_package/base/graph.h>
#include <teb_package/base/teb_vertices.h>
#include <teb_package/utilities/misc.h>
#include <array>


// Disable warning for multiple default constractors in Visual Studio 2013
// The warning appears only when implementing the default constructor even if it is deleted.
// But the warning doesn't change anything, the behavior is correct.
#ifdef WIN32
#pragma warning(disable: 4520)
#endif


namespace teb
{

/**
 * @brief Templated base edge class that stores an arbitary number of values.
 * 
 * @ingroup controller
 *  
 * This class defines the basis for nearly all edges. The dimension
 * has to be known at compile time and is set by the template parameter
 * \c D.
 * 
 * @remarks Call allocateMemory() before using the edge for optimization.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa EdgeType
 *
 * @tparam D Dimension of the edge (value vector).
 * @tparam FuncType Function type of the underlying cost function (according to FUNC_TYPE enum)
 * @tparam Vertices... An arbitary number of vertex types that are attached to this edge
 */    
template <int D, FUNCT_TYPE FuncType, class... Vertices>
class BaseEdge : public EdgeType
{
public:
 
#ifdef WIN32
  static const int NoVertices = sizeof...(Vertices); //< Get number of statically attached vertices
#else
  static constexpr const int NoVertices = sizeof...(Vertices); //< Get number of statically attached vertices
#endif
  static const int Dimension = D; //!< Edge dimension as static member variable.
  virtual int dimension() const {return D;} //!< Return edge dimension by function call.
    
  //! Typedef to represent the static Eigen::Vector of cost/constraint values.  
  using ValueVector =  Eigen::Matrix<double, D, 1>;
  //! Typedef to represent the vertex container.
  using VertexContainer = const std::array<VertexType*, NoVertices>;
    

  BaseEdge() = delete; //!< Delete default constructor
  
  template <class ...VerticesT>
  BaseEdge(VerticesT&... args) : EdgeType(), _vertices({&args...})
  {
    // Check if types and number of template parameters match class template paremters for vertices
    static_assert(variadic_temp_equal<std::tuple<Vertices...>, std::tuple<VerticesT...>>::value, "BaseEdge(): Number and types of vertices passed via the constructor does not match number of class template parameters");
  }  //!< Construct edge by passing all vertices references.
  
  virtual ~BaseEdge() {} //!< Empty Destructor
  
  //! Return function typed given by template parameter FuncType
  virtual FUNCT_TYPE functType() const { return FuncType; }
    
  // accessor functions
  virtual const double* valuesData() const {return _values.data();} // Implements EdgeType::valuesData()
  virtual double* valuesData() {return _values.data();} // Implements EdgeType::valuesData()
  virtual const ValueVector& values() const { return _values;} //!< Return cost/constraint values as static Eigen::Vector (read-only).
  virtual ValueVector& values() { return _values;} //!< Return cost/constraint values as static Eigen::Vector.
  
 
 /**
  * @brief Allocate memory for JacobianWorkspace and HessianWorkspace and get vertices dimensions.
  * @remarks Before calling, call resizeVertexContainer() edge and add all vertices (setVertex()) first.
  * @todo Implement automatic memory allocation instead calling this function manually.
  * @param skip_hessian If no hessian calculation is required by the solver skip memory allocation.
  */
  virtual void allocateMemory(bool skip_hessian = false)
  {
      // determine dimensions for all vertices
      calculateVertexDimensions();
      // allocate block-jacobian
      _jacobians.allocate(D, _vertices_dim);
      // allocate block-hessian
      if (!skip_hessian)
      {
	_hessians.allocate(D, _vertices_dim);
      }
  }
  
  
   virtual VertexType* getVertex(unsigned int idx) {assert(idx< (unsigned int) _vertices.size()); return _vertices.at(idx);} //!< Access attached vertex with index \c idx. 
   virtual const VertexType* getVertex(unsigned int idx) const {assert(idx< (unsigned int) _vertices.size()); return _vertices.at(idx);} //!< Access attached vertex with index \c idx (read-only). 
  
   virtual unsigned int noVertices() const {return (unsigned int) NoVertices;}; //!< Return the number of attached vertices
  
    /**
     * @brief Query dimensions of all vertices attached to this edge and store them internally.
     *
     * Stores dimensions to class container EdgeType::_vertices_dim.
     * This function is called within BaseEdge::allocateMemory().
     * @todo Maybe implement static/constexpr version of collecting all dimensions since they are nown at compile-time (maybe http://stackoverflow.com/questions/19019252/c11-create-0-to-n-constexpr-array-in-c)
     */
    virtual void calculateVertexDimensions()
    {
        for (unsigned int i=0; i<_vertices.size(); ++i)
        {
            _vertices_dim.at(i) = _vertices.at(i)->dimension();
        }   
    }
                
    /**
     * @brief Return vertex container of the edge.
     * @return (Read-only) reference to the vertex container.
     */
    virtual const VertexContainer& vertices() const {return _vertices;};
        

    virtual void computeValues() {}; //!< Actual cost function or constraint function. For constraints it is computeValues() <= 0.
    
    virtual void computeJacobian(); //!< Compute Block-Jacobian and store it to EdgeType::_jacobians according to the description in JacobianWorkspace.
    virtual void computeHessian(); //!< Compute Block-Hessian and store it to EdgeType::_hessians according to the description in HessianWorkspace.
    
    
  
protected:
  
    // Use member from the parent template class.
    using EdgeType::_jacobians;
    using EdgeType::_hessians;
    
    //! Actual cost/constraint value data object (initialized to zero).
    ValueVector _values = ValueVector::Zero();
    
    const VertexContainer _vertices; //!< Container that stores all attached vertices.
    std::array<int, NoVertices> _vertices_dim; //!< Store dimensions for all vertices (EdgeType::_vertices, calculateVertexDimensions()) 

    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// =========== Partial specialization for dynamic types ================

/**
 * @brief Templated base edge class that stores an arbitary number of values (Partial template specialization that allows changing vertices dimensions at runtime).
 * 
 * @ingroup controller
 *  
 * This class defines the basis for nearly all edges. The dimension
 * has to be known at compile time and is set by the template parameter
 * \c D.
 * 
 * @remarks Call allocateMemory() before using the edge for optimization.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa EdgeType
 *
 * @tparam D Dimension of the edge (value vector).
 * @tparam FuncType Function type of the underlying cost function (according to FUNC_TYPE enum)
 */    
template <int D, FUNCT_TYPE FuncType>
class BaseEdge<D, FuncType> : public EdgeType
{
public:
 
  static const int Dimension = D; //!< Edge dimension as static member variable.
  virtual int dimension() const {return D;} //!< Return edge dimension by function call.
    
  //! Typedef to represent the static Eigen::Vector of cost/constraint values.  
  using ValueVector =  Eigen::Matrix<double, D, 1>;
  //! Typedef to represent the vertex container.
  using VertexContainer = std::vector<VertexType*>;
  
  
  BaseEdge() : EdgeType() {} //!< Empty Constructor
  virtual ~BaseEdge() {} //!< Empty Destructor
    
  //! Return function typed given by template parameter FuncType
  virtual FUNCT_TYPE functType() const { return FuncType; }    
    
  // accessor functions
  virtual const double* valuesData() const {return _values.data();} // Implements EdgeType::valuesData()
  virtual double* valuesData() {return _values.data();} // Implements EdgeType::valuesData()
  virtual const ValueVector& values() const { return _values;} //!< Return cost/constraint values as static Eigen::Vector (read-only).
  virtual ValueVector& values() { return _values;} //!< Return cost/constraint values as static Eigen::Vector.
  
 
 /**
  * @brief Allocate memory for JacobianWorkspace and HessianWorkspace and get vertices dimensions.
  * @remarks Before calling, call resizeVertexContainer() edge and add all vertices (setVertex()) first.
  * @todo Implement automatic memory allocation instead calling this function manually.
  * @param skip_hessian If no hessian calculation is required by the solver skip memory allocation.
  */
  virtual void allocateMemory(bool skip_hessian = false)
  {
      // determine dimensions for all vertices
      calculateVertexDimensions();
      // allocate block-jacobian
      _jacobians.allocate(D, _vertices_dim);
      // allocate block-hessian
      if (!skip_hessian)
      {
	_hessians.allocate(D, _vertices_dim);
      }
  }
  
   virtual VertexType* getVertex(unsigned int idx) {assert(idx< (unsigned int) _vertices.size()); return _vertices.at(idx);} //!< Access attached vertex with index \c idx. 
   virtual const VertexType* getVertex(unsigned int idx) const {assert(idx< (unsigned int) _vertices.size()); return _vertices.at(idx);} //!< Access attached vertex with index \c idx (read-only). 
  
   virtual unsigned int noVertices() const {return (unsigned int) _vertices.size();}; //!< Return the number of attached vertices
  
    /**
     * @brief Query dimensions of all vertices attached to this edge and store them internally.
     *
     * Stores dimensions to class container EdgeType::_vertices_dim.
     * This function is called within BaseEdge::allocateMemory().
     * @todo Maybe implement another strategy to store the dimensions.
     */
    virtual void calculateVertexDimensions()
    {
        _vertices_dim.resize(_vertices.size());
        for (unsigned int i=0; i<_vertices.size(); ++i)
        {
            _vertices_dim.at(i) = _vertices.at(i)->dimension();
        }   
    }

        
    /**
     * @brief Attach a new vertex of type TebVertex* with index \c idx to the edge.
     *
     * Make sure that memory is allocated before using resizeVertexContainer()
     * or that it is controlled by a specific child class.
     *
     * @param idx index of the vertex inside this edge. Start with 0.
     * @param pvertex pointer to the TebVertex object or to one of its children.
     */
    virtual void setVertex(unsigned int idx, VertexType* pvertex)
    {
        assert(idx<_vertices.size());
        _vertices.at(idx) = pvertex;
    };
    
    /**
     * @brief Return vertex container of the edge.
     * @return (Read-only) reference to the vertex container.
     */
    virtual const VertexContainer& vertices() const {return _vertices;};
    
    /**
     * @brief Return vertex container of the edge.
     * @return Reference to the vertex container.
     */
    virtual VertexContainer& vertices() {return _vertices;};
    
       
    virtual void computeJacobian(); //!< Compute Block-Jacobian and store it to EdgeType::_jacobians according to the description in JacobianWorkspace.
    virtual void computeHessian(); //!< Compute Block-Hessian and store it to EdgeType::_hessians according to the description in HessianWorkspace.
    
    
     void resizeVertexContainer(unsigned int n) {_vertices.resize(n);} //!< Set number \c n of vertices attached to this edge.
  
protected:
  
    // Use member from the parent template class.
    using EdgeType::_jacobians;
    using EdgeType::_hessians;
    
    //! Actual cost/constraint value data object (initialized to zero).
    ValueVector _values = ValueVector::Zero();
    
    VertexContainer _vertices; //!< Container that stores all attached vertices.
    std::vector<int> _vertices_dim; //!< Store dimensions for all vertices (EdgeType::_vertices, calculateVertexDimensions())    
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




} // end namespace teb


#include <teb_package/base/base_edge.hpp>

#endif /* defined(__teb_package__base_edge__) */
