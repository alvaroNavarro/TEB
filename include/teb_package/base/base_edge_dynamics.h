#ifndef __teb_package__base_edge_dynamics__
#define __teb_package__base_edge_dynamics__

#include <teb_package/base/base_edge.h>
#include <teb_package/base/system_dynamics.h>

namespace teb
{


/**
* @brief Equality constraint edge for satisfying continious system dynamics specified by an SystemDynamics object.
*
* @ingroup controller
*
* This edge creates an equility constraint based on nonlinear system-dynamics equations.
* System dynamics can be specified using a SystemDynamics object.
* The pointer to the system dynamics must be passed to the constructor of this object
* or by calling setSystemObject().
* See SystemDynamics for more information about how to model a specific system.
*
* This edge approximates the continuous-timde derivatives that appear in each system equation by
* finite differences (see teb::FiniteDifferences).
* Two different types of finite differences are supported by this edge at the moment.
* - Forward differences: \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k}}{\Delta T} \f$
* - Central differences: \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k-1}}{2\Delta T} \f$
*
* The number of vertices required by this edge depend on whether central differences are enabled or not:
* - Forward differences:
*   1. StateVertex  \f$ \mathbf{x}_k \f$
*   2. ControlVertex  \f$ \mathbf{u}_k \f$
*   3. StateVertex  \f$ \mathbf{x}_{k+1} \f$
*   4. TimeDiff  \f$ \Delta T \f$
* - Central differences:
*   1. StateVertex  \f$ \mathbf{x}_{k-1} \f$
*   2. StateVertex  \f$ \mathbf{x}_k \f$
*   3. ControlVertex  \f$ \mathbf{u}_k \f$
*   4. StateVertex  \f$ \mathbf{x}_{k+1} \f$
*   4. TimeDiff  \f$ \Delta T \f$
*
* @remarks Call allocateMemory() before using the edge for optimization.
*
* @bug Central differences do not seem to work as expected at the moment.
*
* @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
*
* @tparam p Number of state variables
* @tparam q Number of input variables
* @tparam central Enable central differences (make sure to read the info text -> vertex type may change)
*/
template<int p, int q, bool central = false>
class EdgeSystemDynamics : public BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>
{
	template <int pf, int qf>
	friend class SystemDynamics;

public:
	//! Dimension of this edge equals always the number of system equations
	static const int Dimension = p;

	static_assert(p>0, "System dynamics only possible for p>0");

	/**
	* @brief Constructor that requires a pointer to a SystemDynamics object
	* @param system Pointer to a SystemDynamics object
	* @param px1 Pointer to a StateVertex related to \f$ \mathbf{x}_{k-1} \f$
	* @param pu1 Pointer to a ControlVertex related to \f$ \mathbf{u}_k \f$
	* @param px2 Pointer to a StateVertex related to \f$ \mathbf{x}_k \f$
	* @param pdt Pointer to a TimeDiff \f$ \Delta T \f$
	*/
	EdgeSystemDynamics(SystemDynamics<p, q>* system, StateVertex<p>& px1, ControlVertex<q>& pu1, StateVertex<p>& px2, TimeDiff& pdt) : BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>(px1, pu1, px2, pdt), _system(system)
	{
	}


	virtual void computeValues() // See EdgeType::computeValues()
	{
		assert(_system != nullptr);

		const StateVertex<p>* sample_x1 = static_cast<const StateVertex<p>*>(_vertices[0]);
		const ControlVertex<q>* sample_u1 = static_cast<const ControlVertex<q>*>(_vertices[1]);
		const StateVertex<p>* sample_x2 = static_cast<const StateVertex<p>*>(_vertices[2]);
		const TimeDiff* sample_dt = static_cast<const TimeDiff*>(_vertices[3]);

		_values = _system->finiteElemFwdEuler(sample_x1->states(), sample_x2->states(), sample_dt->dt()) - _system->stateSpaceModel(sample_x1->states(), sample_u1->controls()); // x1 = x_{k} , x2 = x_{k+1}
	};


	/**
	* @brief Set SystemDynamics object
	* @param system Pointer to a SystemDynamics object
	*/
	void setSystemDynamics(SystemDynamics<p, q>* system) { _system = system; }

	// Use methods from the parent template class.
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::computeJacobian;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::computeHessian;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::dimension;

protected:

	SystemDynamics<p, q>* _system = nullptr; //!< Pointer to the system dynamics object.

	// Use members from the parent template class.
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_vertices;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_values;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_jacobians;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_hessians;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/*--- Template Specialization for central differences ---*/
//! @cond 0
template<int p, int q>
class EdgeSystemDynamics<p, q, true> : public BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>
{
	template <int pf, int qf>
	friend class SystemDynamics;

public:
	//! Dimension of this edge equals always the number of system equations
	static const int Dimension = p;

	static_assert(p>0, "System dynamics only possible for p>0");

	/**
	* @brief Constructor that requires a pointer to a SystemDynamics object
	* @param system Pointer to a SystemDynamics object
	* @param px1 Pointer to a StateVertex related to \f$ \mathbf{x}_{k-1} \f$
	* @param px2 Pointer to a StateVertex related to \f$ \mathbf{x}_k \f$
	* @param pu1 Pointer to a ControlVertex related to \f$ \mathbf{u}_k \f$
	* @param px3 Pointer to a StateVertex related to \f$ \mathbf{x}_{k+1} \f$
	* @param pdt Pointer to a TimeDiff \f$ \Delta T \f$
	*/
	EdgeSystemDynamics(SystemDynamics<p, q>* system, StateVertex<p>& px1, StateVertex<p>& px2, ControlVertex<q>& pu2, StateVertex<p>& px3, TimeDiff& pdt) : BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>(px1, px2, pu2, px3, pdt), _system(system)
	{
	}


	virtual void computeValues() // See EdgeType::computeValues()
	{
		assert(_system != nullptr);

		const StateVertex<p>* sample_x1 = static_cast<const StateVertex<p>*>(_vertices[0]);
		const StateVertex<p>* sample_x2 = static_cast<const StateVertex<p>*>(_vertices[1]);
		const ControlVertex<q>* sample_u2 = static_cast<const ControlVertex<q>*>(_vertices[2]);
		const StateVertex<p>* sample_x3 = static_cast<const StateVertex<p>*>(_vertices[3]);
		const TimeDiff* sample_dt = static_cast<const TimeDiff*>(_vertices[4]);
			
		_values = _system->finiteElemCentralDiff(sample_x1->states(), sample_x3->states(), sample_dt->dt()) - _system->stateSpaceModel(sample_x2->states(), sample_u2->controls()); // x1 = x_{k-1} , x2 = x_{k+1}
	};


	/**
	* @brief Set SystemDynamics object
	* @param system Pointer to a SystemDynamics object
	*/
	void setSystemDynamics(SystemDynamics<p, q>* system) { _system = system; }

	// Use methods from the parent template class.
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::computeJacobian;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::computeHessian;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::dimension;

protected:

	SystemDynamics<p, q>* _system = nullptr; //!< Pointer to the system dynamics object.

	// Use members from the parent template class.
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_vertices;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_values;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_jacobians;
	using BaseEdge<p, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, ControlVertex<q>, StateVertex<p>, TimeDiff>::_hessians;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
//! @endcond


  
} // end namespace teb


#endif /* defined(__teb_package__base_edge_dynamics__) */
