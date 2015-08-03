#ifndef __teb_package__teb_edges_usual__
#define __teb_package__teb_edges_usual__

#include <teb_package/base/typedefs.h>
#include <teb_package/base/bound_constraints.h>
#include <teb_package/base/base_edge_dynamics.h>
#include <teb_package/base/system_dynamics.h>

namespace teb
{

    
/**
 * @brief Objective edge: minimize \f$ \Delta T \f$.
 *
 * @ingroup controller
 *
 * The following cost function is implemented:
 * \f[ v = \sigma \Delta T  \f]
 * \f$ \sigma \f$ denotes the weight. Choosing (\c n -1) has performed fine in the past,
 *  since \f$ T = (n-1) \Delta T \f$.
 * 
 * Set the weight \f$ \sigma \f$ using setData().
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. TimeDiff \f$ \Delta T \f$
 *
 * @remarks Call allocateMemory() before using the edge for optimization.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class EdgeMinimizeTime : public BaseEdge<1, FUNCT_TYPE::LINEAR, TimeDiff>
{
public:

  /**
   * @brief Construct edge and attach the corresponding TimeDiff vertex.
   * @param dt_vertex Reference to the TimeDiff vertex
   */
  EdgeMinimizeTime(TimeDiff& dt_vertex) : BaseEdge<1, FUNCT_TYPE::LINEAR, TimeDiff>(dt_vertex)
  {
  }
  
    
  virtual void computeValues() // See EdgeType::computeValues()
  {
      TimeDiff* sample_i = static_cast<TimeDiff*>(_vertices[0]);
      _values[0] =  _data * sample_i->dt();
  }

  //! Specification of the analytic block jacobian
  virtual void computeJacobian()
  {
      _jacobians.getWorkspace(0).fill(_data);
  }
  
  //! Specification of the analytic block hessian
  
  virtual void computeHessian()
  {
      _hessians.getWorkspace(0,0).fill(0.0);
  }
    
  void setData(double data) {_data = data;} //!< Set weight \f$ \sigma \f$ for the cost function.
    
protected:
    double _data = 1; //!< Weight of the objective function
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};

    
/**
 * @brief Bound constraint edge for the time difference: \f$ \Delta T \ge \epsilon \f$.
 *
 * @ingroup controller
 *
 * The following inequality constraint is implemented:
 * \f[ \Delta T \ge \epsilon \f]
 * \f$ \epsilon \f$ denotes an offset. 
 *
 * Set the offset \f$ \epsilon \f$ using setBounds().
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. TimeDiff \f$ \Delta T \f$
 */
using EdgePositiveTime = BoundConstraint<BOUND_TYPE::LOWER, TimeDiff, BOUND_VARS::SINGLE, 0>;


/**
 * @brief Bound constraint edge for control inputs: \f$ \mathbf{u}_{min} \le \mathbf{u}_k \le \mathbf{u}_{max} \f$.
 *
 * @ingroup controller
 *
 * The following two inequality constraints are implemented:
 * \f{eqnarray} 
 *      \mathbf{u}_k \ge \mathbf{u}_{min} \\
 *      \mathbf{u}_k \le \mathbf{u}_{max} \f}
 *
 * \f$ \mathbf{u}_{min} \f$ and \f$ \mathbf{u}_{max} \f$ are the lower and upper bounds on the control input \f$ \mathbf{u}_k \f$.
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. ControlVertex  \f$ \mathbf{u}_k \f$
 * 
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template<int q>
using EdgeControlBounds = BoundConstraint<BOUND_TYPE::LOWERUPPER, ControlVertex<q>, BOUND_VARS::ALL>;  
    

/**
 * @brief Bound constraint edge for states: \f$ \mathbf{x}_{min} \le \mathbf{x}_k \le \mathbf{x}_{max} \f$.
 *
 * @ingroup controller
 *
 * The following two inequality constraints are implemented:
 * \f{eqnarray}
 *      \mathbf{x}_k \ge \mathbf{x}_{min} \\
 *      \mathbf{x}_k \le \mathbf{x}_{max} \f}
 *
 * \f$ \mathbf{x}_{min} \f$ and \f$ \mathbf{x}_{max} \f$ are the lower and upper bounds on the state vector \f$ \mathbf{x}_k \f$.
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. StateVertex  \f$ \mathbf{x}_k \f$
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template<int p>
using EdgeStateBounds = BoundConstraint<BOUND_TYPE::LOWERUPPER, StateVertex<p>, BOUND_VARS::ALL>;

       
    
    
  
/**
 * @brief Objective edge: quadratic form for states and control input.
 *
 * @ingroup controller
 *
 * The following cost function is implemented:
 * \f[ J_k = (\mathbf{x}_k-\mathbf{x}_{ref})^T \mathbf{Q} (\mathbf{x}_k-\mathbf{x}_{ref}) + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k \f]
 *
 * \f$ \mathbf{Q} \f$ denotes the weight for the state vector \f$ \mathbf{x}_k \f$. and \f$ \mathbf{R} \f$ denotes the weight for the control input vector \f$ \mathbf{u}_k \f$.
 *
 * If this edge is added to all samples (states and control inputs) while composing the hyper-graph, it generates the following cost-function for the underlying optimization problem:
 * \f[ J = (\mathbf{x}_n-\mathbf{x}_{ref})^T \mathbf{Q}_n (\mathbf{x}_n-\mathbf{x}_{ref}) + \sum_{k=0}^{n-1} ((\mathbf{x}_k-\mathbf{x}_{ref})^T \mathbf{Q} (\mathbf{x}_k-\mathbf{x}_{ref}) + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k) \f]
 * The weight for the final constraint (and for all others) can be choosen by specifying different weights.
 * Set the weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ using setWeights().
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. StateVertex \f$ \mathbf{x}_k \f$
 * 2. ControlVertex \f$ \mathbf{u}_k \f$
 *
 * If you only want to apply the quadratic form to either only states or control inputs, set q=0 or p=0. A specialized template is provided.
 *
 * @todo Currently the Levenberg-Marquardt solver take the cost function squared by itself, therefore we need to implement the sqrt here: For other solvers this cost function will be problematic without taking the square.
 *
 * @remarks We decided to allow only diagonal matrices for the weights at the moment in order to improve speed.
 * @remarks Call allocateMemory() before using the edge for optimization.
 *
 * @warning Don't forget to initialize Q and R matrices, since they are constructed uninitialized!
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
template <int p, int q>
class EdgeQuadraticForm: public BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>
{
public:
    
  /**
   * @brief Construct edge and attach the corresponding state and control vertex.
   * @param state_vertex Reference to the StateVertex
   * @param control_vertex Reference to the ControlVertex
   */
	EdgeQuadraticForm(StateVertex<p>& state_vertex, ControlVertex<q>& control_vertex) : BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>(state_vertex, control_vertex)
    {
    }
    
    
    virtual void computeValues() // See EdgeType::computeValues()
    {
		StateVertex<p>* state_i = static_cast<StateVertex<p>*>(_vertices[0]);
		ControlVertex<q>* control_i = static_cast<ControlVertex<q>*>(_vertices[1]);
        //_values[0] =  (sample_i->states() - _ref_state).transpose() * _Q * (sample_i->states() - _ref_state);
        //_values[0] += sample_i->controls().transpose() * _R * sample_i->controls();
        //HACK: just use Q diag vec : use only sqrt of the problem in order to test levenbergmarquardt
		_values[0] = (state_i->states() - _ref_state).transpose() * _Q.diagonal();
		_values[0] += control_i->controls().transpose() * _R.diagonal();
    }
    
    /** 
     * @brief Specification of the analytic block jacobian.
     *
     * It is: \f[ \frac{\partial}{\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = \mathbf{x}^T \mathbf{A} + \mathbf{A}^T = 2 \mathbf{x}^T \mathbf{A} \f]
     * The last step is allowed since \f$ \mathbf{A} \f$ is symmetric.
     * This comutation is now performed for both states and control inputs w.r.t \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$.
     */
    virtual void computeJacobian()
    {
        //TebSample<p,q>* sample_i = static_cast<TebSample<p,q>*>(_vertices[0]);
        //_jacobians.getWorkspace(0).block(0,0,1,p) = 2 * (sample_i->states() - _ref_state).transpose() * _Q;
        //_jacobians.getWorkspace(0).block(0,p,1,q) = 2 * sample_i->controls().transpose() * _R;
        _jacobians.getWorkspace(0) = _Q.diagonal().transpose();
        _jacobians.getWorkspace(1) = _R.diagonal().transpose();
    };
    
    /** 
     * @brief Specification of the analytic block hessian.
     *
     * The hessian w.r.t. a symmetric weight matrix \f$ \mathbf{A} \f$ is defined by
     * \f[ \frac{\partial^2}{\partial \mathbf{x},\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = 2 \mathbf{A} \f]
     *
     * Applying this formular to the states and control inputs with respect to weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ implies a block diagonal matrix (even a diagonal matrix, since the blocks are diagonal as well).
     */
    virtual void computeHessian()
    {
        _hessians.getWorkspace(0,0) = 2 * _Q;
        _hessians.getWorkspace(1,0) = 2 * _R;
        // everything else is zero by initialization (inside the workspace)
    }
    
    //! Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
    void setWeights(const Eigen::DiagonalMatrix<double,p>& Q, const Eigen::DiagonalMatrix<double,q>&  R )
    {
        _Q = Q;
        _R = R;
    }
    
    //! Set diagonal weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
    void setWeights(const Eigen::Ref<const Eigen::Matrix<double,p,1>>& Q, const Eigen::Ref<const Eigen::Matrix<double,q,1>>& R )
    {
        _Q.diagonal() = Q;
        _R.diagonal() = R;
    }
    
    //! Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function using uniform diagonal elements
    void setWeights(double Q, double  R )
    {
        _Q.diagonal().fill(Q);
        _R.diagonal().fill(R);
    }
    
    //! Set reference state \f$ \mathbf{x}_f f\$
    void setReference(const Eigen::Ref<const Eigen::Matrix<double,p,1>>& xf)
    {
        _ref_state = xf;
    }
    
    // Use methods from the parent template class.
    using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>::dimension;
    
protected:
    
    // Use members from the parent template class.
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>::_vertices;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>::_values;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>::_jacobians;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>, ControlVertex<q>>::_hessians;

    Eigen::DiagonalMatrix<double,p> _Q; //!< Weight for the state vector \f$ \mathbf{x}_k \f$.
    Eigen::DiagonalMatrix<double,q> _R; //!< Weight for the control input \f$ \mathbf{u}_k \f$.
    
    typedef Eigen::Matrix<double,p,1> StateVector; //!< Typedef for a StateVector type (required here to avoid gcc bug within the next line)
    StateVector _ref_state = StateVector::Zero(); //!< Store reference state (final/goal state)
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
    
  
/* --- Template specialization for q=0 --- */
//! @cond 0
template <int p>
class EdgeQuadraticForm<p,0> : public BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>
{
public:

	/**
	* @brief Construct edge and attach the corresponding state vertex.
	* @param state_vertex Reference to the StateVertex
	*/
	EdgeQuadraticForm(StateVertex<p>& state_vertex) : BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>(state_vertex)
	{
	}

	/**
	* @brief Construct edge and attach the corresponding state vertex.
	* This version accepts a control vertex as well, just to preserve the interface compatibility, but the vertex will be ignored.
	* @param state_vertex Reference to the StateVertex
	*/
	EdgeQuadraticForm(StateVertex<p>& state_vertex, VertexType&) : BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>(state_vertex)
	{
	}

	virtual void computeValues() // See EdgeType::computeValues()
	{
		StateVertex<p>* state_i = static_cast<StateVertex<p>*>(_vertices[0]);
		//_values[0] =  (sample_i->states() - _ref_state).transpose() * _Q * (sample_i->states() - _ref_state);
		//HACK: just use Q diag vec : use only sqrt of the problem in order to test levenbergmarquardt
		_values[0] = (state_i->states() - _ref_state).transpose() * _Q.diagonal();
	}

	/**
	* @brief Specification of the analytic block jacobian.
	*
	* It is: \f[ \frac{\partial}{\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = \mathbf{x}^T \mathbf{A} + \mathbf{A}^T = 2 \mathbf{x}^T \mathbf{A} \f]
	* The last step is allowed since \f$ \mathbf{A} \f$ is symmetric.
	* This comutation is now performed for the states w.r.t \f$ \mathbf{Q} \f$.
	*/
	virtual void computeJacobian()
	{
		//TebSample<p,q>* sample_i = static_cast<TebSample<p,q>*>(_vertices[0]);
		//_jacobians.getWorkspace(0).block(0,0,1,p) = 2 * (sample_i->states() - _ref_state).transpose() * _Q;
		//_jacobians.getWorkspace(0).block(0,p,1,q) = 2 * sample_i->controls().transpose() * _R;
		_jacobians.getWorkspace(0) = _Q.diagonal().transpose();
	}

	/**
	* @brief Specification of the analytic block hessian.
	*
	* The hessian w.r.t. a symmetric weight matrix \f$ \mathbf{A} \f$ is defined by
	* \f[ \frac{\partial^2}{\partial \mathbf{x},\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = 2 \mathbf{A} \f]
	*
	* Applying this formular to the states and control inputs with respect to weights \f$ \mathbf{Q} \f$ implies a block diagonal matrix (even a diagonal matrix, since the blocks are diagonal as well).
	*/
	virtual void computeHessian()
	{
		_hessians.getWorkspace(0, 0) = 2 * _Q;
		// everything else is zero by initialization (inside the workspace)
	}

	//! Set weights \f$ \mathbf{Q} \f$ and for the cost function.
	void setWeights(const Eigen::DiagonalMatrix<double, p>& Q)
	{
		_Q = Q;
	}

	//! Set diagonal weights \f$ \mathbf{Q} \f$ for the cost function.
	void setWeights(const Eigen::Ref<const Eigen::Matrix<double, p, 1>>& Q)
	{
		_Q.diagonal() = Q;
	}

	//! Set weights \f$ \mathbf{Q} \f$ for the cost function using uniform diagonal elements
	void setWeights(double Q)
	{
		_Q.diagonal().fill(Q);
	}

	//! Preserve Interface Compatibility: Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(const Eigen::DiagonalMatrix<double, p>& Q, const Eigen::MatrixXd&) { _Q = Q; }

	//! Preserve Interface Compatibility: Set diagonal weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(const Eigen::Ref<const Eigen::Matrix<double, p, 1>>& Q, const Eigen::Ref<const Eigen::MatrixXd>&)	{ _Q.diagonal() = Q; }

	//! Preserve Interface Compatibility: Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function using uniform diagonal elements
	void setWeights(double Q, double){ _Q.diagonal().fill(Q); }

	//! Set reference state \f$ \mathbf{x}_f f\$
	void setReference(const Eigen::Ref<const Eigen::Matrix<double, p, 1>>& xf)
	{
		_ref_state = xf;
	}

	// Use methods from the parent template class.
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>::dimension;

protected:

	// Use members from the parent template class.
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>::_vertices;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>::_values;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>::_jacobians;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, StateVertex<p>>::_hessians;

    Eigen::DiagonalMatrix<double, p> _Q; //!< Weight for the state vector \f$ \mathbf{x}_k \f$.

	typedef Eigen::Matrix<double, p, 1> StateVector; //!< Typedef for a StateVector type (required here to avoid gcc bug within the next line)
	StateVector _ref_state = StateVector::Zero(); //!< Store reference state (final/goal state)

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
//! @endcond
    


/* --- Template specialization for p=0 --- */
//! @cond 0
template <int q>
class EdgeQuadraticForm<0,q> : public BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>
{
public:

	/**
	* @brief Construct edge and attach the corresponding control vertex.
	* @param control_vertex Reference to the ControlVertex
	*/
	EdgeQuadraticForm(ControlVertex<q>& control_vertex) : BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>(control_vertex)
	{
	}

	/**
	* @brief Construct edge and attach the corresponding control vertex.
	* This version accepts a state vertex as well, just to preserve the interface compatibility, but the vertex will be ignored.
	* @param control_vertex Reference to the ControlVertex
	*/
	EdgeQuadraticForm(VertexType&, ControlVertex<q>& control_vertex) : BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>(control_vertex)
	{
	}


	virtual void computeValues() // See EdgeType::computeValues()
	{
		ControlVertex<q>* control_i = static_cast<ControlVertex<q>*>(_vertices[0]);
		//_values[0] =  control_i->controls().transpose() * _R * control_i->controls();
		//HACK: just use Q diag vec : use only sqrt of the problem in order to test levenbergmarquardt
		_values[0] = control_i->controls().transpose() * _R.diagonal();
	}

	/**
	* @brief Specification of the analytic block jacobian.
	*
	* It is: \f[ \frac{\partial}{\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = \mathbf{x}^T \mathbf{A} + \mathbf{A}^T = 2 \mathbf{x}^T \mathbf{A} \f]
	* The last step is allowed since \f$ \mathbf{A} \f$ is symmetric.
	* This comutation is now performed for control inputs w.r.t \f$ \mathbf{R} \f$.
	*/
	virtual void computeJacobian()
	{
		//TebSample<p,q>* sample_i = static_cast<TebSample<p,q>*>(_vertices[0]);
		//_jacobians.getWorkspace(0).block(0,0,1,p) = 2 * (sample_i->states() - _ref_state).transpose() * _Q;
		//_jacobians.getWorkspace(0).block(0,p,1,q) = 2 * sample_i->controls().transpose() * _R;
		_jacobians.getWorkspace(0) = _R.diagonal().transpose();
	}

	/**
	* @brief Specification of the analytic block hessian.
	*
	* The hessian w.r.t. a symmetric weight matrix \f$ \mathbf{A} \f$ is defined by
	* \f[ \frac{\partial^2}{\partial \mathbf{x},\partial \mathbf{x}} \mathbf{x}^T \mathbf{A} \mathbf{x} = 2 \mathbf{A} \f]
	*
	* Applying this formular to the control inputs with respect to weights \f$ \mathbf{R} \f$ implies a block diagonal matrix (even a diagonal matrix, since the blocks are diagonal as well).
	*/
	virtual void computeHessian()
	{
		_hessians.getWorkspace(0, 0) = 2 * _R;
		// everything else is zero by initialization (inside the workspace)
	}

	//! Set weights \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(Eigen::DiagonalMatrix<double, q> R)
	{
		_R = R;
	}

	//! Set diagonal weights \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(const Eigen::Ref<const Eigen::Matrix<double, q, 1>>& R)
	{
		_R.diagonal() = R;
	}

	//! Set weights \f$ \mathbf{R} \f$ for the cost function using uniform diagonal elements
	void setWeights(double R)
	{
		_R.diagonal().fill(R);
	}

	//! Preserve Interface Compatibility: Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(const Eigen::MatrixXd&, const Eigen::DiagonalMatrix<double, q>& R) { _R = R; }

	//! Preserve Interface Compatibility: Set diagonal weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function.
	void setWeights(const Eigen::Ref<const Eigen::MatrixXd>&, const Eigen::Ref<const Eigen::Matrix<double, q, 1>>& R)	{ _R.diagonal() = R; }

	//! Preserve Interface Compatibility: Set weights \f$ \mathbf{Q} \f$ and \f$ \mathbf{R} \f$ for the cost function using uniform diagonal elements
	void setWeights(double, double R){ _R.diagonal().fill(R); }

	// Use methods from the parent template class.
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>::dimension;

protected:

	// Use members from the parent template class.
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>::_vertices;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>::_values;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>::_jacobians;
	using BaseEdge<1, FUNCT_TYPE::LINEAR_SQUARED, ControlVertex<q>>::_hessians;

    Eigen::DiagonalMatrix<double, q> _R; //!< Weight for the control input vector \f$ \mathbf{u}_k \f$.

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
//! @endcond 



    
  
/**
 * @brief Generic edge for functions \f$ \mathbb{R}^n \to \mathbb{R} \f$.
 *
 * @ingroup controller
 *
 * This edge can be used for passing a generic function (e.g. a lambda)
 * to the optimization problem. Jacobians and Hessians are calculated numerically
 * if necessary.
 *
 * This edge should be only used for Rapid-Prototyping puroposes, due to the
 * computational overhead that rises from copying and calling the external function resp. functor.
 *
 * You can pass a function that accepts a EdgeGenericScalarFun::VertexContainer as
 * argument. Each vertex stored in that container corresponds to the vertex passed using the
 * class constructor (in the same order). The corresponding Vertex types are specified as template
 * arguments of this class as well.
 *
 * Example usage for defining (x-1)^2 (Note, keep in mind the special definitions for squaring within different solvers):
 * @code
 *      StateVertex<1> x;
 *      using MyEdgeT = EdgeGenericScalarFun<FUNCT_TYPE::LINEAR_SQUARED, StateVertex<1>>; // LINEAR_SQUARED functions will by squared by all solvers
 *      auto fun = [] (MyEdgeT::VertexContainer& vertices) {return vertices.at(0)->getData(0)-1;}; // Note, we define only (x-1) here
 *      MyEdgeT* my_edge = new MyEdgeT(fun, x);
 *      // now add to the hyper-graph
 * @endcode
 *
 * @sa EdgeGenericScalarFun
 *
 * @remarks Call allocateMemory() before using the edge for optimization.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
template <FUNCT_TYPE Funtype, class... VerticesT>
class EdgeGenericScalarFun : public BaseEdge<1, Funtype, VerticesT...>
    {
    public:
        
        using VertexContainer = typename BaseEdge<1, Funtype, VerticesT...>::VertexContainer;
        
        using ExtFunT = std::function<double(VertexContainer&)>;

        //! Construct generic scalar function edge by passing the function object and all vertices
        EdgeGenericScalarFun(const ExtFunT& fun, VerticesT&... vertices) : BaseEdge<1, Funtype, VerticesT...>(vertices...), _fun(fun)
        {
        }
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
            _values[0] = _fun(_vertices);
        }
        
        
        // Use methods from the parent template class.
        using BaseEdge<1, Funtype, VerticesT...>::dimension;
        
    protected:
        
        ExtFunT _fun; //!< Store the external function or functor
        
        // Use members from the parent template class.
        using BaseEdge<1, Funtype, VerticesT...>::_vertices;
        using BaseEdge<1, Funtype, VerticesT...>::_values;
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
};
    
    
    
/**
 * @brief Generic edge for functions \f$ \mathbb{R}^n \to \mathbb{R}^D \f$.
 *
 * @ingroup controller
 *
 * This edge can be used for passing a generic function (e.g. a lambda)
 * to the optimization problem. Jacobians and Hessians are calculated numerically
 * if necessary.
 *
 * This edge should be only used for Rapid-Prototyping puroposes, due to the
 * computational overhead that rises from copying and calling the external function resp. functor.
 *
 * You can pass a function that accepts a EdgeGenericScalarFun::VertexContainer as
 * argument. Each vertex stored in that container corresponds to the vertex passed using the
 * class constructor (in the same order). The corresponding Vertex types are specified as template
 * arguments of this class as well.
 *
 * The return type of the vector valued external function should be a Eigen::Matrix<double,D,1> type.
 *
 * @sa EdgeGenericScalarFun
 *
 * @remarks Call allocateMemory() before using the edge for optimization.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
template <int D, FUNCT_TYPE Funtype, class... VerticesT>
class EdgeGenericVectorFun : public BaseEdge<D, Funtype, VerticesT...>
{
public:
    using VertexContainer = typename BaseEdge<D, Funtype, VerticesT...>::VertexContainer;
    
    using ExtFunT = std::function<Eigen::Matrix<double,D,1>(VertexContainer&)>;
    
    //! Construct generic vector function edge by passing the dimension D, the function object and all vertices
    EdgeGenericVectorFun(const ExtFunT& fun, VerticesT&... vertices) : BaseEdge<D, Funtype, VerticesT...>(vertices...), _fun(fun)
    {
    }
    
    virtual void computeValues() // See EdgeType::computeValues()
    {
        _values = _fun(_vertices);
    }
    
protected:
    
    ExtFunT _fun; //!< Store the external function or functor
    
    // Use members from the parent template class.
    using BaseEdge<D, Funtype, VerticesT...>::_vertices;
    using BaseEdge<D, Funtype, VerticesT...>::_values;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
    
    
/**
 * @brief Equality Constraint Edge: add control input derivatives.
 *
 * @ingroup controller
 *
 * This edge can be used to add derivatives to control inputs:
 * E.g. if a plant model includes zero dynamics, the control input vector \f$ \mathbf{u}_k \f$
 * can be assumend as followed: \f$ \mathbf{u}_k = [u_k, \dot{u}_k, \ddot{u}_k, \dotsc]^T \f$.
 * Now assume a generic control input vector  \f$ \mathbf{u}_k = [u_{1,k}, u_{2,k}, u_{3,k}, \dotsc]^T \f$,
 * this edge adds the equality constraints: \f$ u_{2,k} = \dot{u}_{1_k}, \dotsc \f$.
 *
 * In order to allow MIMO Systems in which a single control has non-zero derivatives, this edge accepts
 * a template parameter \f$ n < q \f$ that can be used to specify only the first \c n+1 control input vector components to
 * be a single input and its derivatives. The other \c q - \c n +1 components are not representing any derivatives.
 *
 * Vertices required (connect the edge with the following vertices):
 * 1. ControlVector \f$ \mathbf{u}_k \f$
 * 2. ControlVector \f$ \mathbf{u}_{k+1} \f$
 * 3. TimeDiff \f$ \Delta T \f$
 *
 * @remarks Call allocateMemory() before using the edge for optimization.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
template <int q, int n=q, bool init=false, typename dummy = void>
class EdgeInputDerivatives : public BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>
{
public:
    
    /**
     * @brief Construct edge and attach the corresponding TimeDiff vertex.
     * @param u1 Reference to the ControlVertex \f$ \mathbf{u}_{k-1} \f$
     * @param u2 Reference to the ControlVertex \f$ \mathbf{u}_k\f$
     * @param dt_vertex Reference to the TimeDiff vertex
     */
    EdgeInputDerivatives(ControlVertex<q>& u1, ControlVertex<q>& u2, TimeDiff& dt_vertex) : BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>(u1, u2, dt_vertex)
    {
        static_assert(n<=q, "The template parameter k must be smaller than or equal q (the number of control input components)");
    }
    
    
    virtual void computeValues() // See EdgeType::computeValues()
    {
        ControlVertex<q>* u1 = static_cast<ControlVertex<q>*>(_vertices[0]);
        ControlVertex<q>* u2 = static_cast<ControlVertex<q>*>(_vertices[1]);
        TimeDiff* dt_vert = static_cast<TimeDiff*>(_vertices[2]);
        
        _values = u2->controls().segment(1, n-1) -  ( u2->controls().head(n-1) - u1->controls().head(n-1) ) / dt_vert->dt();
        
    }

    
protected:
    
    // Use members from the parent template class.
    using BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>::_vertices;
    using BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>::_values;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
    
//! @cond 0
template <int q, int n>
class EdgeInputDerivatives<q,n,true, typename std::enable_if<(n>0)>::type> : public BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>>
{
public:
    
    /**
     * @brief Construct edge and attach the corresponding TimeDiff vertex.
     * @param u1 Reference to the ControlVertex \f$ \mathbf{u}_k \f$
     */
    EdgeInputDerivatives(ControlVertex<q>& u1) : BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>>(u1)
    {
        static_assert(n<=q, "The template parameter k must be smaller than or equal q (the number of control input components)");
    }
    
    
    virtual void computeValues() // See EdgeType::computeValues()
    {
        ControlVertex<q>* u1 = static_cast<ControlVertex<q>*>(_vertices[0]);
        
        _values = u1->controls().segment(1, n-1) -  ( u1->controls().head(n-1) - _ref ) / _dt;
        
    }
    
    
    void setInitialReference(const Eigen::Ref< const Eigen::Matrix<double,n-1,1>>& ref, double dt)
    {
        _ref = ref;
        _dt = dt;
    }
    
protected:
    
    // Use members from the parent template class.
    using BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>>::_vertices;
    using BaseEdge<n-1, FUNCT_TYPE::NONLINEAR, ControlVertex<q>>::_values;
    
    using InputRef = Eigen::Matrix<double,n-1,1>; // We need this typedef to bypass a gcc bug for the following line
    InputRef _ref = InputRef::Zero();
    double _dt = -1;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
//! @endcond
    
//! @cond 0
// This specialization avoids compilation errors with n=0;
template <int q, int n, bool init>
class EdgeInputDerivatives<q,n,init,typename std::enable_if<n==0>::type> : public BaseEdge<0, FUNCT_TYPE::NONLINEAR>
{
public:
    
    template <class ...T>
    EdgeInputDerivatives(T&...)
    {
        //static_assert(false, "The template parameter k must be smaller than or equal q (the number of control input components) and greater than zero");
        PRINT_ERROR("EdgeInputDerivatives with template parameter n=0 is undefined.");
    }
    
    virtual void computeValues() // See EdgeType::computeValues()
    {
        PRINT_ERROR("EdgeInputDerivatives with template parameter n=0 is undefined.");
    }
    
    
    void setInitialReference(const Eigen::Ref< const Eigen::Matrix<double,0,1>>& ref, double dt)
    {
        PRINT_ERROR("EdgeInputDerivatives with template parameter n=0 is undefined.");
    }
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
//! @endcond

    
    
} // end namespace teb


#endif /* defined(__teb_package__teb_edges_usual__) */
