#ifndef __teb_package__teb_vertices__
#define __teb_package__teb_vertices__

#include <teb_package/base/graph.h>


namespace teb
{
  
/**
* @brief Vertex that contains the StateVector
*
* @ingroup controller
*
* This class wraps the discrete state vector \f$ \mathbf{x}_k \in \mathbb{R}^p \f$
* into a vertex type for the hyper-graph.
*
* @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
*
* @sa ControlVertex, VertexType, TimeDiff, BaseEdge
*
* @tparam p Number of states / Size of the state vector
*/
template <int p>
class StateVertex : public VertexType
{
public:
	static const int DimStates = p; //!< Number of states / Size of the state vector as static variable
	static const int Dimension = p; //!< Number of total values stored in this vertex (number of state vector components).

	//! Typedef for state vector with p states [p x 1].
	using StateVector = Eigen::Matrix<double, p, 1>;
	//! Typedef for a 1D-scalar value represented as Eigen::Matrix type [1 x 1].
	using ScalarType = Eigen::Matrix<double, 1, 1>;
	//! Typedef for logical mask that stores fixed and unfixed states [p x 1].
	using FixedStates = Eigen::Matrix<bool, p, 1>;

	//! Typdef for the backup stack.
	//#ifdef RTW
	//using BackupStackType = std::stack<std::pair<StateVector, ControlVector>, std::vector<std::pair<StateVector, ControlVector>>>; // There are some compile errors if compiled for Simulink Target using Simulink Coder. Without the allocator it works fine.
	//#else
	using BackupStackType = std::stack<StateVector, std::vector<StateVector, Eigen::aligned_allocator<StateVector> > >;
	//#endif



	// ===== Constructors =====


	StateVertex() : StateVertex(StateVector::Zero())  { } //!< Default constructor: states vector initialized to zero.

	/**
	*  @brief Construct StateVertex with predefined state values.
	*  @param states Eigen::Vector with p states.
	*/
	StateVertex(const Eigen::Ref<const StateVector>& states) : _states(states)  { }

	/**
	*  @brief Construct StateVertex with predefined state values and predefined control inputs.
	*  @param states double array with p states.
	*/
	StateVertex(const double* states) : _states(states)
	{
	}


	/** @brief Copy constructor
	*  @param obj Object of type StateVertex     *
	*/
	StateVertex(const StateVertex<p>& obj) : StateVertex(obj.states())
	{
		_fixed_states = obj.fixed_states();
	}


	// ===== Public Methods =====

	// Implements VertexType::dimension()
	virtual int dimension() const { return p; };
	// Implements VertexType::dimensionFree()
	virtual int dimensionFree() const { return p - (int)_fixed_states.array().count(); };

	// Implements VertexType::plus()
	virtual void plus(const VertexType* rhs)
	{
		const StateVertex<p>* state_vertex = static_cast<const StateVertex<p>*>(rhs);
		_states += state_vertex->states();
	};

	// Implements VertexType::plus()
	virtual void plus(const double* rhs)
	{
		std::transform(_states.data(), _states.data() + p, rhs, _states.data(), std::plus<double>());
	};

	// Implements VertexType::plusFree()
	virtual void plusFree(const double* rhs)
	{
		int idx = 0;
		for (unsigned int i = 0; i<p; ++i)
		{
			if (!isFixedComp(i))
			{
				_states.coeffRef(i) += *(rhs + idx);
				++idx;
			}
		}
	};

	// Implements VertexType::setFree()
	virtual void setFree(const double* rhs)
	{
		int idx = 0;
		for (unsigned int i = 0; i<p; ++i)
		{
			if (!isFixedComp(i))
			{
				_states.coeffRef(i) = *(rhs + idx);
				++idx;
			}

		}
	};

	// Implements VertexType::getDataFree()
	virtual void getDataFree(double* target_vec) const
	{
		unsigned int idx = 0;
		for (int i = 0; i<p; ++i)
		{
			if (!isFixedComp(i))
			{
				target_vec[idx++] = _states.coeffRef(i);
			}
		}
	}

	// Implements VertexType::getData()
	virtual const double& getData(unsigned int idx) const
	{
		assert(idx < p);
		return _states.coeffRef(idx);
	}


	// === Accessor functions for states ===

	const StateVector& states() const { return _states; } //!< Get StateVector (read-only). @return state vector [p x 1]
	StateVector& states() { return _states; } //!< Get StateVector. @return state vector [p x 1]
	virtual void setStates(const Eigen::Ref<const StateVector>& s) { _states = s; } //!< Set or update StateVector. @param s state vector [p x 1]
	virtual void setStates(double state) { _states[0] = state; } //!< Set or update only first component of the StateVector (useful for 1d states). @param state single state component.
	const FixedStates& fixed_states() const { return _fixed_states; } //!< Get logical map that defines fixed and unfixed states (fixed = \c true). @return logical map (p x 1 vector of booleans

	/**
	*  @brief Set all states of this object fixed or unfixed.
	*  @param fixed set to \c true in order to fix all \a p states
	*/
	void setFixedAll(bool fixed)
	{
		_fixed_states.setConstant(fixed);
	}


	/**
	*  @brief Set a single component of the StateVector to fixed/unfixed.
	*  @param state_idx index of the underlying StateVector component that should be fixed.
	*  @param fixed set to \c true in order to fix the selected state.
	*/
	void setFixedState(unsigned int state_idx, bool fixed) { assert(state_idx < p); _fixed_states.coeffRef(state_idx) = fixed; }

	/**
	*  @brief Set all states of the StateVector to fixed/unfixed.
	*  @param fixed set to \c true in order to fix all \a p states.
	*/
	void setFixedStates(bool fixed) { _fixed_states.setConstant(fixed); }

	/**
	*  @brief Set selected states of the StateVector to fixed/unfixed at once.
	*  @param fixed boolean array with \a p elements in which each \c true sets the corresponding state of the StateVector to fixed.
	*/
	void setFixedStates(const bool* fixed) { memcpy(_fixed_states.data(), fixed, p); };

	/**
	*  @brief Set selected states of the StateVector to fixed/unfixed at once (Eigen version).
	*  @param fixed_states Eigen::Vector with \a p booleans in which each \c true sets the corresponding state of the StateVector to fixed.
	*/
	void setFixedStates(const Eigen::Ref<const FixedStates>& fixed_states) { _fixed_states = fixed_states; }


	virtual bool isFixedAll() const { return (_fixed_states.array() == true).all(); } // Implements VertexType::isFixedAll()
	bool isFixedAny() const { return (_fixed_states.array() == true).any(); } // Implements VertexType::isFixedAny()

	/**
	* @brief Check if selected element of the StateVector is fixed.
	* @param idx index to element in the StateVector
	* @return \c true if element with index \c state_idx is fixed.
	*/
	virtual bool isFixedComp(int idx) const { return _fixed_states.coeffRef(idx); } // Implements VertexType::isFixedComp()

	// === backup stack (e.g. for numerical diff) ===

	virtual void push() { _backup.push(_states); } // Implements VertexType::push()
	virtual void pop() // Implements VertexType::pop()
	{
		top();
		_backup.pop();
	}
	virtual void top() // Implements VertexType::top()
	{
		assert(!_backup.empty());
		_states = _backup.top();
	}
	virtual void discardTop() { assert(!_backup.empty()); _backup.pop(); } // Implements VertexType::discardTop()
	virtual unsigned int stackSize() const { return (unsigned int)_backup.size(); } // Implements VertexType::stackSize()


	// === Copy assignment operator ===
	StateVertex<p>& operator=(const StateVertex<p>& rhs)
	{
		if (this != &rhs) // otherwise self-assignment
		{
			_states = rhs.states();
			_fixed_states = rhs.fixed_states();
		}
		return *this;
	};


	// === relational operators ===
	friend bool operator==(const StateVertex<p>& lhs, const StateVertex<p>& rhs)
	{
		return lhs.states() == rhs.states();
	}

	friend bool operator!=(const StateVertex<p>& lhs, const StateVertex<p>& rhs) { return !operator==(lhs, rhs); }

    
	// === output stream operator ===

	/**
	* @brief Print state vector using std::ostream.
	* @param os output stream object
	* @param state vector specific StateVertex object
	* @return output stream object including a formatted string containing the state vector.
	*/
	friend std::ostream& operator<<(std::ostream& os, const StateVertex<p>& state)
	{
		os << "x: [" << state.states().transpose() << "]";
		return os;
	}

protected:
	StateVector _states; //!< State vector with \a p states [p x 1]
	BackupStackType _backup; //!< backup stack (store and restore states). 

	int _dimension = p; //!< Store dimension for all values stored (\a p states)
	FixedStates _fixed_states = FixedStates::Constant(false); //!<  Mask that stores fixed and unfixed states [p x 1] true: fixed, false: free

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



/**
* @brief Vertex that contains the ControlVector
*
* @ingroup controller
*
* This class wraps the discrete control input vector \f$ \mathbf{u}_k \in \mathbb{R}^q \f$ into
* a vertex for the hyper-graph.
*
* @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
*
* @sa StateVertex, VertexType, TimeDiff, BaseEdge
*
* @tparam q Number of control inputs / Size of the control vector
*/
template <int q>
class ControlVertex : public VertexType
{
public:
	static const int DimControls = q; //!< Number of control inputs / Size of the control vector as static variable
	static const int Dimension = q; //!< Number of total values stored in this vertex (number of control input vector components).

	//! Typedef for control input vector with q controls [q x 1].
	using ControlVector = Eigen::Matrix<double, q, 1>;
	//! Typedef for a 1D-scalar value represented as Eigen::Matrix type [1 x 1].
	using ScalarType = Eigen::Matrix<double, 1, 1>;
	//! Typedef for logical mask that stores fixed and unfixed controls [q x 1].
	using FixedCtrls = Eigen::Matrix<bool, q, 1>;

	//! Typdef for the backup stack.
	//#ifdef RTW
	//using BackupStackType = std::stack<std::pair<StateVector, ControlVector>, std::vector<std::pair<StateVector, ControlVector>>>; // There are some compile errors if compiled for Simulink Target using Simulink Coder. Without the allocator it works fine.
	//#else
	using BackupStackType = std::stack<ControlVector, std::vector<ControlVector, Eigen::aligned_allocator<ControlVector> > >;
	//#endif



	// ===== Constructors =====


	ControlVertex() : ControlVertex(ControlVector::Zero())  { } //!< Default constructor: control vector initialized to zero.

	/**
	*  @brief Construct ControlVertex with given control inputs.
	*  @param controls Eigen::Vector with \c q control inputs.
	*/
	ControlVertex(const Eigen::Ref<const ControlVector>& controls) : _controls(controls)  { }


	/** @brief Copy constructor
	*  @param obj Object of type ControlVertex  
	*/
	ControlVertex(const ControlVertex<q>& obj) : ControlVertex(obj.controls())
	{
		_fixed_ctrls = obj.fixed_ctrls();
	}


	// ===== Public Methods =====

	// Implements VertexType::dimension()
	virtual int dimension() const { return q; };
	// Implements VertexType::dimensionFree()
	virtual int dimensionFree() const { return q - (int)_fixed_ctrls.array().count(); };

	// Implements VertexType::plus()
	virtual void plus(const VertexType* rhs)
	{
		const ControlVertex<q>* control = static_cast<const ControlVertex<q>*>(rhs);
		_controls += control->controls();
	};

	// Implements VertexType::plus()
	virtual void plus(const double* rhs)
	{
		std::transform(_controls.data(), _controls.data() + q, rhs, _controls.data(), std::plus<double>());
	};

	// Implements VertexType::plusFree()
	virtual void plusFree(const double* rhs)
	{
		int idx = 0;
		for (unsigned int i = 0; i<q; ++i)
		{
			if (!isFixedComp(i))
			{
				_controls.coeffRef(i) += *(rhs + idx);
				++idx;
			}

		}
	};

	// Implements VertexType::setFree()
	virtual void setFree(const double* rhs)
	{
		int idx = 0;
		for (unsigned int i = 0; i<q; ++i)
		{
			if (!isFixedComp(i))
			{
				_controls.coeffRef(i) = *(rhs + idx);
				++idx;
			}

		}
	};

	// Implements VertexType::getDataFree()
	virtual void getDataFree(double* target_vec) const
	{
		unsigned int idx = 0;
		for (int i = 0; i<q; ++i)
		{
			if (!isFixedComp(i))
			{
				target_vec[idx++] = _controls.coeffRef(i);
			}
		}
	}

	// Implements VertexType::getData()
	virtual const double& getData(unsigned int idx) const
	{
		assert(idx < q);
		return _controls.coeffRef(idx);
	}


	// === Accessor functions for controls ===

	const ControlVector& controls() const { return _controls; } //!< Get ControlVector (read-only). @return control vector [q x 1]
	ControlVector& controls() { return _controls; } //!< Get ControlVector. @return control vector [q x 1]
	virtual void setControls(const Eigen::Ref<const ControlVector>& c) { _controls = c; } //!< Set or update ControlVector. @param c control vector [p x 1]
	virtual void setControls(double control) { _controls[0] = control; } //!< Set or update only first component of the ControlVector (useful for 1d controls). @param control single control component.
	const FixedCtrls& fixed_ctrls() const { return _fixed_ctrls; } //!< Get logical map that defines fixed and unfixed controls (fixed = \c true). @return logical map (q x 1 vector of booleans)

	/**
	*  @brief Set all controls of this object fixed or unfixed.
	*  @param fixed set to \c true in order to fix all \a q controls.
	*/
	void setFixedAll(bool fixed)
	{
		_fixed_ctrls.setConstant(fixed);
	}


	/**
	*  @brief Set a single component of the ControlVector to fixed/unfixed.
	*  @param ctrl_idx index of the underlying ControlVector component that should be fixed.
	*  @param fixed set to \c true in order to fix the selcted control.
	*/
	void setFixedCtrl(unsigned int ctrl_idx, bool fixed) { assert(ctrl_idx < q); _fixed_ctrls.coeffRef(ctrl_idx) = fixed; }

	/**
	*  @brief Set all controls of the ControlVector to fixed/unfixed.
	*  @param fixed set to \c true in order to fix all \a q controls.
	*/
	void setFixedCtrls(bool fixed) { _fixed_ctrls.setConstant(fixed); }

	/**
	*  @brief Set selected controls of the ControlVector to fixed/unfixed at once (Eigen version).
	*  @param fixed_ctrls Eigen::Vector with \a q booleans in which each \c true sets the corresponding control of the ControlVector to fixed.
	*/
	void setFixedCtrls(const Eigen::Ref<const FixedCtrls>& fixed_ctrls) { _fixed_ctrls = fixed_ctrls; }

	/**
	*  @brief Set selected components of the ControlVector to fixed/unfixed at once.
	*  @param fixed boolean array with \a q elements in which each \c true sets the corresponding control of the ControlVector to fixed.
	*/
	void setFixedCtrls(const bool* fixed) { memcpy(_fixed_ctrls.data(), fixed, q); };

	virtual bool isFixedAll() const { return (_fixed_ctrls.array() == true).all(); } // Implements VertexType::isFixedAll()
	bool isFixedAny() const { return (_fixed_ctrls.array() == true).any(); } // Implements VertexType::isFixedAny()

	virtual bool isFixedComp(int idx) const { return _fixed_ctrls.coeffRef(idx); } // Implements VertexType::isFixedComp()

	// === backup stack (e.g. for numerical diff) ===

	virtual void push() { _backup.push(_controls); } // Implements VertexType::push()
	virtual void pop() // Implements VertexType::pop()
	{
		top();
		_backup.pop();
	}
	virtual void top() // Implements VertexType::top()
	{
		assert(!_backup.empty());
		_controls = _backup.top();
	}
	virtual void discardTop() { assert(!_backup.empty()); _backup.pop(); } // Implements VertexType::discardTop()
	virtual unsigned int stackSize() const { return (unsigned int)_backup.size(); } // Implements VertexType::stackSize()


	// === Copy assignment operator ===
	ControlVertex<q>& operator=(const ControlVertex<q>& rhs)
	{
		if (this != &rhs) // otherwise self-assignment
		{
			_controls = rhs.controls();
			_fixed_ctrls = rhs.fixed_ctrls();
		}
		return *this;
	};


	
	// === relational operators ===
	friend bool operator==(const ControlVertex<q>& lhs, const ControlVertex<q>& rhs)
	{
		return lhs.controls() == rhs.controls();
	}

	friend bool operator!=(const ControlVertex<q>& lhs, const ControlVertex<q>& rhs) { return !operator==(lhs, rhs); }

	// === output stream operator ===

	/**
	* @brief Print  control vector using std::ostream.
	* @param os output stream object
	* @param control specific ControlVertex object
	* @return output stream object including a formatted string containing control input vector.
	*/
	friend std::ostream& operator<<(std::ostream& os, const ControlVertex<q>& control)
	{
		os << "u: [" << control.controls().transpose() << "]";
		return os;
	}

protected:
	ControlVector _controls; //!< Control input vector with \a q controls [q x 1]
	BackupStackType _backup; //!< backup stack (store and restore control input). 

	int _dimension = q; //!< Store dimension for all values stored (\a q controls)
	FixedCtrls _fixed_ctrls = FixedCtrls::Constant(false); //!<  Mask that stores fixed and unfixed controls [q x 1] true: fixed, false: free

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

    

/**
 * @brief Vertex that contains the time information of the TEB: \f$ \Delta T \f$.
 * 
 * @ingroup controller
 *   
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa VertexType, StateVertex, ControlVertex, BaseEdge
 */
class TimeDiff : public VertexType
{
public:
    static const int Dimension = 1; //!< The dimension of the TimeDiff vertex is always 1.
    
    /**
     * @brief Construct new TimeDiff and optinally set vertex fixed.
     * @param fixed if \c true , the TimeDiff is fixed during optimization (no time-optimal control possible).
     */
    TimeDiff(bool fixed=false) {_fixed=fixed;};
    
    /**
     * @brief Construct new TimeDiff with specified initial value and optinally set vertex fixed.
     * @param dt initial \f$ \Delta T \f$
     * @param fixed if \c true , the TimeDiff is fixed during optimization (no time-optimal control possible).
     */
    TimeDiff(double dt, bool fixed=false) : _dt(dt), _fixed(fixed) {};
    
    /**
     * @brief Construct new TimeDiff with specified initial value and optinally set vertex fixed.
     * @param dt Pointer to initial \f$ \Delta T \f$ (will be copied)
     * @param fixed if \c true , the TimeDiff is fixed during optimization (no time-optimal control possible).
     */
    TimeDiff(double *dt, bool fixed=false) : _dt(*dt), _fixed(fixed) {}; // make copy!
    
    /**
     * @brief Copy constructor
     * @param obj TimeDiff object
     */
    TimeDiff(const TimeDiff& obj) : _dt(obj.dt()), _fixed(obj.isFixedAll()) {};
    
    ~TimeDiff() {};
    
    virtual bool isFixedAll() const {return _fixed;} // Implements VertexType::isFixedAll()
    virtual void setFixedAll(bool fixed) {_fixed = fixed;} //!< set VertexType fixed. @param fixed if \c true TimeDiff is fixed.
    virtual bool isFixedAny() const {return _fixed;} // Implements VertexType::isFixedAny()
    virtual bool isFixedComp(int) const {return _fixed;} // Implements VertexType::isFixedComp()
    
    double& dt() {return _dt;} //!< return current \f$ \Delta T \f$.
    const double& dt() const {return _dt;} //!< return current \f$ \Delta T \f$ (read-only).
    
    // === backup stack ===
    virtual void push() { _backup.emplace(_dt);} // Implements VertexType::push()
    virtual void pop() // Implements VertexType::pop()
    {
         top();
        _backup.pop();
    }
    virtual void top() // Implements VertexType::top()
    {
        assert(!_backup.empty());
        _dt = _backup.top();
    }
    virtual void discardTop() { assert(!_backup.empty()); _backup.pop();} // Implements VertexType::discardTop()
    virtual unsigned int stackSize() const {return (unsigned int) _backup.size();} // Implements VertexType::stackSize()
    
    virtual int dimension() const {return 1;}; // Implements VertexType::dimension()
    virtual int dimensionFree() const {return _fixed ? 0 : 1;}; // Implements VertexType::dimensionFree()
    
    
    virtual void plus(const VertexType* rhs) // Implements VertexType::plus()
    {
      const TimeDiff* dt = static_cast<const TimeDiff*>(rhs);
      _dt += dt->dt();
    };
    
    virtual void plus(const double* rhs) // Implements VertexType::plus()
    {
      _dt += *rhs;
    };
    
    virtual void plusFree(const double* rhs) // Implements VertexType::plusFree()
    {
        if (!_fixed) _dt += *rhs;
    }
    
    virtual void setFree(const double* rhs) // Implements VertexType::setFree()
    {
        if (!_fixed) _dt = *rhs;
    }
    
    // Implements VertexType::getDataFree()
    virtual void getDataFree(double* target_vec) const
    {
      if (!_fixed)
      {
	*target_vec = _dt;
      }	
    }
    
    // Implements VertexType::getData()
    virtual const double& getData(unsigned int) const
    {
      return _dt;
    }
    
    // === assignment operator ===
    TimeDiff& operator=(const TimeDiff &rhs) 
    {
      if (this != &rhs) // otherwise self-assignment
      {
        _dt = rhs.dt();
        _fixed = rhs.isFixedAll();
      }
        return *this;
    };
    
    inline void operator()(double& dt)
    {
        _dt = dt;
    }
    
    // === output stream operator ===
    /**
     * @brief Print \f$ \Delta T \f$ using std::ostream.
     * @param os output stream object
     * @param dt specific TimeDiff object
     * @return output stream object including a formatted string containing state and control vector.
     */ 
    friend std::ostream& operator<<(std::ostream& os, const TimeDiff& dt)
    {
        os << dt.dt();
        return os;
    }
    
protected:
    double _dt = 0.1;
    bool _fixed = false;
    std::stack<double> _backup;
};

} // end namespace teb

#endif /* defined(__teb_package__teb_vertices__) */
