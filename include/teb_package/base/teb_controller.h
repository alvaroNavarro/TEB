#ifndef __teb_package__teb_controller__
#define __teb_package__teb_controller__

#include <teb_package/base/typedefs.h>
#include <teb_package/base/base_controller.h>
#include <teb_package/base/teb_vertices.h>
#include <teb_package/base/common_teb_edges.h>
#include <teb_package/base/base_solver.h>
#include <teb_package/base/obstacle.h>

#include <Eigen/Core>
#include <Eigen/StdDeque>
#include "algorithm"

#include <vector>


//! General project namespace for all library functions and objects
namespace teb 
{

  
/**
 * @brief Main Timed-Elastic-Band controller class
 * 
 * @ingroup controller
 *  
 * This class defines the main TEB controller functions required to controll
 * a dynamic system.
 * 
 * The underlying optimziation problem is defined as a hyper-graph in which 
 * edges define cost/objective functions (see TebController::buildOptimizationGraph and 
 * TebController::customOptimizationGraph class methods).
 * 
 * To overcome the creation of custom hyper-graphs, one can use the SystemDynamics class
 * and pass the system to this class using TebController::setSystemDynamics.
 * Other predifined edges are listed below.
 * 
 * The TEB controller requires a dedicated solver (inherited from BaseSolver) that
 * can be passed using the setSolver method.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @todo Using the TebController inside a standard container (e.g. vector) does not work as supposed to do.
 * @todo Create copy and move constructors.
 *
 * @sa TebController::buildOptimizationGraph, TebController::customOptimizationGraph, 
 * 	SystemDynamics, TebController::setSystemDynamics,
 * 	TebController::activateObjectiveTimeOptimal, TebController::activateControlBounds,
 *	TebController::activateStateBounds,
 * 	TebController::setSolver BaseSolver
 * 
 * @tparam p Number of states / Size of the state vector
 * @tparam q Number of control inputs / Size of the control vector
 */
template <int p, int q> //! p: number of states, q: number of control inputs
class TebController : public BaseController
{
    
public:        
  
    
  
    static const unsigned int NoStates = p; //!< Store number of states as static variable.
    static const unsigned int NoControls = q; //!< Store number of control inputs as static variable.
    
    // ====== Type Definitions ========
    
    //! Typedef for state vector with p states [p x 1].
	using StateVector = Eigen::Matrix<double,p,1>;
    //! Typedef for control input vector with q controls [q x 1].
	using ControlVector = Eigen::Matrix<double, q, 1>;
	//! Typedef for the state sequence that contains all state vertices.
	using StateSequence = std::deque< StateVertex<p>, Eigen::aligned_allocator<StateVertex<p>> >;
	//! Typedef for the control sequence that contains all control vertices.
	using ControlSequence = std::deque< ControlVertex<q>, Eigen::aligned_allocator<ControlVertex<q>> >;
    //! Typedef for edge containers that stores all edges required for the optimization (Hyper-Graph).
    using EdgeContainer = typename EdgeType::EdgeContainer;
    //! Typedef for vertex containers that stores all vertices required for the optimization (Hyper-Graph).
    using VertexContainer = typename VertexType::VertexContainer;
    //! Typedef to represent a fixed/unfixed boolean vector w.r.t. states (StateVector) [p x 1].
    using FixedStates = Eigen::Matrix<bool,p,1>;
    
    using ObstacleContainer = typename std::vector<Obstacle*>;
    
    
    // ====== Public Member Variables ========
    
    /** @brief Store pointer to the Config class.
     *
     *  If the Config class is provided via TebController::TebController() 
     *  than the memory management is not touched by this class.
     *  If no Config class is provided manually, memory management is owned by this class.
     * 
     *  @sa TebController::TebController(), TebController::_cfg_owned
     */
    const Config* cfg;
    
    // ====== Methods ========
        
    /** @brief Default constructor.
     *
     *  This constructor creates a default Config object for TEB and solver settings. 
     * 
     *  @param solver Register solver type as subclass of BaseSolver.
     *		      If not passed, you must use setSolver() later. 
     */
    TebController(BaseSolver* solver = nullptr) : TebController(new Config(), solver) {_cfg_owned = true;};
    
    /** @brief Extended constructor with config pointer.
     *
     *  Pass config class to controller and solver. TebController does not free Config afterwards.
     * 
     *  @param config Set user-defined config for the controller and solver.
     *  @param solver Register solver type as subclass of BaseSolver.
     *		      If not passed, you must use setSolver() later. 
     */
    TebController(const Config* config, BaseSolver* solver = nullptr) : cfg(config),
									_dt(cfg->teb.dt_ref),
									_optimized(false)
    {
        if (solver) setSolver(solver);
    };
    
    ///! @brief Destructor. Memory is freed if necessary.
    virtual ~TebController()
    {
      _graph.clearGraph(); // free edges and clear vector afterwards
      if (_cfg_owned && cfg) delete cfg;
    }

    // State and control direct accessors
    
    /** @brief Access first state of the TEB sequence
     *  @return Copy of the first state
     */
    virtual Eigen::VectorXd firstState() const {return _state_seq.front().states();}
    
    /** @brief Access first state of the TEB sequence
     *  @return Reference to the first state (read-only)
     */
	const StateVector& firstStateRef() const { return _state_seq.front().states(); }
    
    /** @brief Access last state of the TEB sequence
     *  @return Copy of the last state
     */
	virtual Eigen::VectorXd lastState() const { return _state_seq.back().states(); }
    
    /** @brief Access last state of the TEB sequence
     *  @return Reference to the last state (read-only)
     */
	const StateVector& lastStateRef() const { return _state_seq.back().states(); }
    
    /** @brief Access first control input of the TEB sequence (used for MPC)
     *  @return Copy of the first control input (read-only)
     */
    virtual Eigen::VectorXd firstControl() const {return _ctrl_seq.front().controls();}
    
    /** @brief Access first control input of the TEB sequence (used for MPC)
     *  @return Reference to the first control input (read-only)
     */
	const ControlVector& firstControlRef() const { return _ctrl_seq.front().controls(); }
    
    /** @brief Access saturated first control inputs of the TEB sequence (used for MPC)
     * 
     * This function saturates / bounds the returned control input \f$ \mathbf{u} \f$
     * to \f$ [\mathbf{u}_{min} ,  \mathbf{u}_{max}] \f$. \n
     * The saturation could be helpful in case of soft constraints, that are violated for the planning
     * and prediction, but that should be satisfied for the real system.
     * 
     * Specify the bounds via activateControlBounds().
     * @todo Support multiple control input bounds (now its scalar for all elements of \f$ \mathbf{u} \f$).
     * @return Saturated control input u_k [q x 1].
     */
    Eigen::VectorXd firstControlSaturated() const
    {
      ControlVector u = firstControl();
      u = (u.array() < std::get<1>(_active_control_bounds)).select(std::get<1>(_active_control_bounds),u);
      u = (u.array() > std::get<2>(_active_control_bounds)).select(std::get<2>(_active_control_bounds),u);
      return u;
    }
    
    // Accessor methods for private variables
    
    /** @brief Access complete state sequence container
     *  @return Reference to the state sequence (read-only)
     */
    const StateSequence& stateSequence() const {return _state_seq;};
    
    /** @brief Access complete control input sequence container
     *  @return Reference to the control input sequence (read-only)
     */
    const ControlSequence& controlSequence() const {return _ctrl_seq;};
    
    /** @brief Access vertex that stores the TEB time difference \f$ \Delta T \f$
     *  @return Reference to time-diff vertex (read-only)
     */
    const TimeDiff& dt() const {return _dt;};
    
    /** @brief Return the TEB time difference \f$ \Delta T \f$
     *  @return \f$ \Delta T \f$
     */
    virtual double getDt() const {return _dt.dt();};

    /** @brief Get current number of discrete state vectors in the state sequence
     * @return number of state vectors.
     */    
    virtual unsigned int getN() const {return (unsigned int) _state_seq.size();}

	/** @brief Get current number of discrete control vectors in the control input sequence
	* @return number of control vectors.
	*/
	virtual unsigned int getM() const { return (unsigned int)_ctrl_seq.size(); }
    
    /** @brief Return the complete control input sequence \f$ u_k, k=1,\dotsc,n-1 \f$
     *  @returns Matrix containing all planned control inputs [q x n-1]
     */
    virtual Eigen::MatrixXd returnControlInputSequence() const
    {
		/** @todo Here we assume, that the last control input is invalid -
		* We need to change this, if we have different sizes for state and control input sequences
		*/
        Eigen::MatrixXd u_seq(q, getM()-1);
        for (unsigned int i=0; i<getM()-1; ++i) 
        {
            u_seq.col(i) = _ctrl_seq.at(i).controls();
        }
        return u_seq;
    }
    
    
    /** @brief Access optimizable hyper-graph instance (read-only).
     *
     *  The hyper-graph stores all active vertices and edges
     *  (objectives and constraints) and is passed to the dedicated
     *  solver class.
     *  The hyper-graph is created within buildOptimizationGraph().
     *  Users can add custom edges by subclassing and overriding customOptimizationGraph().
     *  All active vertices are collected within getActiveVertices().
     *
     *  @return Read-only reference to the hyper-graph instance.
     */
    const HyperGraph& graph() const {return _graph;};
    
    virtual ObstacleContainer& getObstacles() = 0;
    virtual Obstacle* getObstacle() = 0;
    //virtual ObstacleContainer* getObstacles() const = 0;    

    virtual unsigned int getNumberObstaclesAdded() = 0;
    
    virtual void addObstacle(Obstacle* obstacle) = 0;
    virtual void removeObstacle() = 0;
    virtual bool isObstacleContainerEmpty() = 0;
    virtual void setObstacleContainer(const ObstacleContainer& obs_container) = 0;        

	/** @brief Access optimizable hyper-graph instance.
	*
	*  The hyper-graph stores all active vertices and edges
	*  (objectives and constraints) and is passed to the dedicated
	*  solver class.
	*  The hyper-graph is created within buildOptimizationGraph().
	*  Users can add custom edges by subclassing and overriding customOptimizationGraph().
	*  All active vertices are collected within getActiveVertices().
	*
	*  @return Read-only reference to the hyper-graph instance.
	*/
	HyperGraph& graph() { return _graph;};
    
    
    virtual void initTrajectory(const Eigen::Ref<const StateVector>& x0, const Eigen::Ref<const StateVector>& xf, unsigned int n); //!< Initialize trajectory between start \c x0 and goal \c xf with n discrete samples.
    void resizeTrajectory(); //!< Resize existing trajectory according to current sample time \f$ \Delta T \f$.
    void resampleTrajectory(unsigned int n_new); //!< Resample trajectory using linear interpolation.
    virtual void updateStart(const Eigen::Ref<const StateVector>& x0); //!< Set new start state.
    virtual void updateGoal(const Eigen::Ref<const StateVector>& xf); //!< Set new final state.
	void removeStateControlInputPair(unsigned int sample_no, bool predict_control = true); //!< Remove StateVector and ControlVector at index \c sample_no from the state and control sequences.
	void insertStateControlInputPair(unsigned int sample_idx, const Eigen::Ref<const StateVector>& state, const Eigen::Ref<const ControlVector>& control, bool predict_control = true); //!< Insert StateVector and ControlVector to the state and contorl sequences at index \c sample_idx.
    inline void predictControl(Eigen::Ref<ControlVector> ctrl_out, const Eigen::Ref<const StateVector>& x1, const Eigen::Ref<const StateVector>& x2); //!< Predict control \c u that corresponds to the transition from \c x1 to \c x2.

	/**
	* @brief Add a state vector \f$ \mathbf{x}_k \f$ and a control input \f$ \mathbf{u}_k \f$ to the end of the state and control sequences.
	* @param state State vector [p x 1] to be added to _state_seq
	* @param control Control input vector [q x 1] to be added _ctrl_seq
	*/
	void pushBackStateControlInputPair(const Eigen::Ref<const StateVector>& state, const Eigen::Ref<const ControlVector>& control)
	{
		_state_seq.emplace_back(state);
		_ctrl_seq.emplace_back(control);
	}

	/**
	* @brief Add a state vector \f$ \mathbf{x}_k \f$ and a control input \f$ \mathbf{u}_k \f$ to the beginning of the state and control sequences.
	* @param state State vector [p x 1] to be added to _state_seq
	* @param control Control input vector [q x 1] to be added _ctrl_seq
	*/
	void pushFrontStateControlInputPair(const Eigen::Ref<const StateVector>& state, const Eigen::Ref<const ControlVector>& control)
	{
		_state_seq.emplace_front(state);
		_ctrl_seq.emplace_front(control);
	}

    /** @brief Set individual goal state vector components fixed or unfixed for optimization
     *  @param fixed_goal_states boolean array of size \c p. Each component of lastState() is set to fixed() if \c fixed_goal_states[i] == \c true.
     */   
    void setGoalStatesFixedOrUnfixed(const bool* fixed_goal_states)
    {
        memcpy(_fixed_goal_states.data(),fixed_goal_states,p);
        // notify graph that vertices are changed
        _graph.notifyGraphModified();
    };
    
	/** @brief Set individual goal state vector components fixed or unfixed for optimization
	*  @param fixed_goal_states boolean std::array or std::initializer_list of size \c p. Each component of lastState() is set to fixed() if \c fixed_goal_states[i] == \c true.
	*/
	void setGoalStatesFixedOrUnfixed(const std::array<double,p>& fixed_goal_states)
	{
		setGoalStatesFixedOrUnfixed(fixed_goal_states.data());
	};

    /** @brief Set individual goal state vector components fixed or unfixed for optimization (Eigen version)
     *  @param fixed_goal_states Eigen::Vector of size \c p containing booleans. Each component of lastState() is set to fixed() if \c fixed_goal_states[i] == \c true.
     */   
    void setGoalStatesFixedOrUnfixed(const Eigen::Ref<const Eigen::Matrix<bool,p,1>>& fixed_goal_states)
    {
        _fixed_goal_states = fixed_goal_states;
        // notify graph that vertices are changed
        _graph.notifyGraphModified();
    };
    
    /** @brief Set all goal state vector components fixed or unfixed for optimization
     *  @sa setupHorizon(), setFixedDt()
     *  @param all_goal_states_fixed if t\c rue, all components of lastState() are set to fixed()
     */
    void setGoalStatesFixedOrUnfixed(bool all_goal_states_fixed)
    {
        _fixed_goal_states = FixedStates::Constant(all_goal_states_fixed);
        // notify graph that vertices are changed
        _graph.notifyGraphModified();
    };
    
    
    /**
     * @brief Setup horizon for the underlying optimal control problem
     *
     * Use this function to setup different common horizon settings at once instead
     * of changing config parameters and class members individually.
     *
     * Common horizons:
     * - Receding/moving horizon: unfixed goal and fixed resolution of samples (fixed dt)
     * - Fixed end-point: fixed goal and fixed resolution of samples (fixed dt)
     * - TEB default: fixed goal and unfixed resolution (unfixed dt, in order to minimize time)
     *
     * If you choose an unfixed resoltion (\c fixed_resolution == \c false) make sure that there
     * are objectives considering the time difference \f$ \Delta T \f$, otherwise the problam will 
     * probably be ill-posed.
     *
     * If you want to fix or unfix only a subset of goal states, use the method setGoalStatesFixedOrUnfixed()
     * after calling setupHorizon or without calling setupHorizon.
     *
     * @sa setFixedDt(), setGoalStatesFixedOrUnfixed()
     *
     * @param fixed_goal Set to \c true in order to fix or unfix all goal state variables.
     * @param fixed_resolution Set to \c true in order to fix the resolution (the time difference \f$ \Delta T \f$ between consecutive samples).
     * @param no_samples Set default/initial number of samples (it remains constant, if the resolution is fixed as well). If -1, Config::Teb::n_pre is used.
     * @param dt Set reference time difference for discretization (it remains constant, if the resolution is fixed as well). If -1, Config::Teb::dt_ref is used.
     */
    void setupHorizon(bool fixed_goal, bool fixed_resolution, int no_samples = -1, double dt = -1)
    {
        if (dt == -1)
        {
			// use time diff stored in cfg
			_dt_ref = cfg->teb.dt_ref;
        }
        else
        {
            assert(dt>0 && dt >= cfg->teb.dt_min);
            _dt_ref = dt;
        }
        _dt.dt() = _dt_ref; // update current dt value as well
        
        if (no_samples == -1)
        {
            // use no_samples stored in cfg
            _no_samples = cfg->teb.n_pre;
        }
        else
        {
            assert(no_samples>0 && (unsigned int) no_samples >= cfg->teb.n_min && (unsigned int) no_samples <= cfg->teb.n_max);
            _no_samples = no_samples;
			// check if trajectory is already initialized
			if (getN() > 0)
			{
				resampleTrajectory(no_samples);
				// we need to reset dt as well
				_dt.dt() = _dt_ref;
			}
        }
        
        setFixedDt(fixed_resolution); // fix time difference (the resolution of the horizon is fixed)
        setGoalStatesFixedOrUnfixed(fixed_goal); // unfix goal to make horizon moving
    }
    
    //! Delete all Vertices (State and control input sequences), all Edges (Hyper-Graph, clearEdges()) and reset \f$ \Delta T \f$ to Config::Teb::dt_ref.
    virtual void resetController() 
    {
      _state_seq.clear();
	  _ctrl_seq.clear();
      _dt.dt() = _dt_ref==-1 ? cfg->teb.dt_ref : _dt_ref;
      _graph.clearGraph();
    };
    
    virtual void buildOptimizationGraph(); //!< Build Hyper-Graph for graph based optimization.
    
    /** @brief Override to add or modify the hyper-graph created in buildOptimizationGraph()
     * 
     *  Derive TEB class and override customOptimizationGraph() to manually construct the hyper-graph
     *  or add edges in addition to activated common edges in buildOptimizationGraph().
     *  See the description of buildOptimizationGraph() for more information on how to add edges.
     * 
     *  @sa buildOptimizationGraph, customOptimizationGraphHotStart, HyperGraph, BaseEdge, initOptimization()
     */ 
    virtual void customOptimizationGraph() {};
    
    /** @brief Use this method to apply some changes before hot-starting from previous graph structures.
     *
     *  Depending on HyperGraph::isGraphModified() the graph is either cleared and constructed again or 
     *  the previous graph is used. In the latter case customOptimizationGraph(), buildOptimizationGraph()
     *  and getActiveVertices() are not called. If you wish to apply changes without recreating the strucutre
     *  add them to this function. It is called inside initOptimization().
     *
     *  @sa customOptimizationGraph, buildOptimizationGraph, HyperGraph, BaseEdge, initOptimization()
     */
    virtual void customOptimizationGraphHotStart() {};
    
    void initOptimization(); //!< Initialize optimiziation - run before calling optimzieTEB().
    void optimizeTEB(); //!< TEB optimization function (includes outer loop and BaseSolver calls).
    
    
	/** @brief Perform complete MPC step (initialization if necessary or update and optimization)
	*
	*  @param x0 double initializer list {x0(1), x0(2),...,x0(p)} or array containing start state \c x0 with \c p components [\c p x 1]
	*  @param xf double initializer list {xf(1), xf(2),...,xf(q)} or array containing final state \c xf with \c p components [\c p x 1]
	*  @param ctrl_out [output] store control input (firstControl()) to control the plant [\c q x 1]
	*/
	virtual void step(const std::array<double, p>& x0, const std::array<double, p>& xf, double* ctrl_out = nullptr)
	{
		step(x0.data(), xf.data(), ctrl_out);
	}


    /** @brief Perform complete MPC step (initialization if necessary or update and optimization)
     *  
     *  @param x0 double array containing start state \c x0 with \c p components [\c p x 1]
     *  @param xf double array containing final state \c xf with \c p components [\c p x 1]
     *  @param ctrl_out [output] store control input (firstControl()) to control the plant [\c q x 1]
     */     
    virtual void step(const double* const x0, const double* const xf, double* ctrl_out = nullptr)
    {
		if (ctrl_out)
		{
			Eigen::Map<ControlVector> ctrl_temp(ctrl_out);
			step(Eigen::Map<const StateVector>(x0), Eigen::Map<const StateVector>(xf), ctrl_temp);
		}
		else
		{
			step(Eigen::Map<const StateVector>(x0), Eigen::Map<const StateVector>(xf));
		}
    }
    

	/** @brief Perform complete MPC step (initialization if necessary or update and optimization) (Eigen version)
	*
	* 	Calls updateStart(), updateGoal(), optimizeTEB() and firstControl().
	*
	*  @param x0 StateVector containing start state \c x0 with \c p components [\c p x 1]
	*  @param xf StateVector containing final state \c xf with \c p components [\c p x 1]
	*  @param ctrl_out [output] store control input (firstControl()) to ControlVector-Map [\c q x 1]
	*/
	void step(const Eigen::Ref<const StateVector>& x0, const Eigen::Ref<const StateVector>& xf, Eigen::Ref<ControlVector> ctrl_out)
	{
		step(x0, xf);
		ctrl_out = firstControl();
	}

    /** @brief Perform complete MPC step (initialization if necessary or update and optimization) (Eigen version)
     * 
     * 	Calls updateStart(), updateGoal(), optimizeTEB() and firstControl().
     * 
     *  @param x0 StateVector containing start state \c x0 with \c p components [\c p x 1]
     *  @param xf StateVector containing final state \c xf with \c p components [\c p x 1]
     */ 
    void step(const Eigen::Ref<const StateVector>& x0, const Eigen::Ref<const StateVector>& xf)
    {
        updateStart(x0);
        updateGoal(xf);
        optimizeTEB();
    }


        
    
    // Predefined edges (costs and constraints)
    
    /**
     * @brief Activate predefined edge for time-optimal control.
     * 
     * Adds the following cost function \f$ V(\mathcal{B}) = (n-1) \Delta T \f$ to the global cost/objective function.
     * 
     * Additionally, the inequality constraint \f$ \Delta T > \Delta T_{min} \f$ is added.
     * \f$ \Delta T_{min} \f$ can be changed in Config::Teb::teb_min.
     * 
     * @param activate if set to \c true than the edge is activated.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveQuadraticForm(), activateControlBounds(), activateStateBounds(), setSystemDynamics()
     */ 
    void activateObjectiveTimeOptimal(bool activate=true)
    {
        _active_time_optimal = activate;
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Activate predefined edge for minimizing the quadratic form.
     *
     * Adds the following cost function
     * \f[ J = (\mathbf{x}_n-\mathbf{x}_{f})^T \mathbf{Q}_f (\mathbf{x}_n-\mathbf{x}_{f} + \sum_{k=0}^{n-1} ((\mathbf{x}_k-\mathbf{x}_{f})^T \mathbf{Q} (\mathbf{x}_k-\mathbf{x}_{f}) + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k)  \f]
     * to the global cost/objective function.
     *
     * The weight matrices are assumed to be diagonal matrices with unifrom values \f$ Q \f$, \f$ R \f$ and \f$ Qf \f$.
     *
     * See EdgeQuadraticForm for further details.
     *
     * @param Q Uniform scalar weight for minimizing the squared state error
     * @param R Uniform scalar weight for minimizing the squared control input
     * @param Qf Uniform scalar weight for minimizing the squared state error for the final state (last state of the horizon)
     * @param activate if set to \c true than the edge is activated.
     *
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), activateStateBounds(), setSystemDynamics()
     */
    void activateObjectiveQuadraticForm(double Q, double R, double Qf, bool activate=true)
    {
        _active_quadratic_form = std::make_tuple(activate,Q,R,Qf);
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    
    
    /**
     * @brief Activate predefined edge for control input bounds.
     * 
     * Adds the following inequality constraints to the optimization problem:
     *  - \f$ \mathbf{u}_k \le u_{max}\f$
     *  - \f$ \mathbf{u}_k \ge u_{min}\f$
     * 
     * @param u_min minimum bound on the control input \f$ \mathbf{u}_k\f$
     * @param u_max maximum bound on the control input \f$ \mathbf{u}_k\f$
     * @param activate if set to \c true than the edge is activated.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateStateBounds(), setSystemDynamics()
     */ 
    void activateControlBounds( const Eigen::Ref<const ControlVector>& u_min , const Eigen::Ref<const ControlVector>& u_max, bool activate=true)
    {
      _active_control_bounds = std::make_tuple(activate,u_min,u_max);
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }

	/**
	* @brief Activate predefined edge for control input bounds.
	*
	* Adds the following inequality constraints to the optimization problem:
	*  - \f$ \mathbf{u}_k \le u_{max}\f$
	*  - \f$ \mathbf{u}_k \ge u_{min}\f$
	*
	* @param u_min minimum bound on the control input \f$ \mathbf{u}_k\f$ as std::array or std::initializer_list type
	* @param u_max maximum bound on the control input \f$ \mathbf{u}_k\f$ as std::array or std::initializer_list type
	* @param activate if set to \c true than the edge is activated.
	*
	* @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateStateBounds(), setSystemDynamics()
	*/
	void activateControlBounds(const std::array<double, q>& u_min, const std::array<double, q>& u_max, bool activate = true)
	{
		activateControlBounds(Eigen::Map<const ControlVector>(u_min.data()), Eigen::Map<const ControlVector>(u_max.data()), activate);
	}
    
    /**
     * @brief Activate predefined edge for control input bounds (single component version)
     * 
     * Adds the following inequality constraints to the optimization problem:
     *  - \f$ \mathbf{u}_k \le \mathbf{u}_{max}\f$ (component-wise)
     *  - \f$ \mathbf{u}_k \ge \mathbf{u}_{min}\f$ (component-wise)
     * 
     * The default initialization of the bounds is \c -INF and \c INF respectively, if this predefined edge
     * is activated.
     * This version of activateControlBounds() makes use of the default initialization and just updates 
     * the bounds for index \c state_index.
     * All other bounds remain untouched (and if not changed before, they are bound to \c -INF and \c INF.
     * 
     * @param control_idx selected component of \f$ \mathbf{x}_k\f$ that should be bounded to \c x_min and \c x_max.
     * @param u_min minimum bound on the component with index \c control_idx of control input \f$ \mathbf{u}_k\f$.
     * @param u_max maximum bound on the component with index \c control_idx of the control input \f$ \mathbf{u}_k\f$.
     * @param activate if set to \c true than the edge is activated.
     * 
     * @todo Avoid unnecessary calculations for unbounded values (Jacobian, Hessian, ...). Maybe change edge-dimension dynamically in that case.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), setSystemDynamics()
     */ 
    void activateControlBounds( unsigned int control_idx, double u_min, double u_max, bool activate=true)
    {
        static_assert(q>0,"Bounds on control inputs are only supported for q>0");
        std::get<0>(_active_control_bounds) = activate;
        std::get<1>(_active_control_bounds)[control_idx] = u_min;
        std::get<2>(_active_control_bounds)[control_idx] = u_max;
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Activate predefined edge for state bounds.
     * 
     * Adds the following inequality constraints to the optimization problem:
     *  - \f$ \mathbf{x}_k \le \mathbf{x}_{max}\f$ (component-wise)
     *  - \f$ \mathbf{x}_k \ge \mathbf{x}_{min}\f$ (component-wise)
     * 
     * To just bound a subset of components of the state \f$ \mathbf{x}_k\f$,
     * set unbounded components to \f$ \textrm{-Inf} \le x_k \le \textrm{Inf} \f$ using the \c INF macro.
     * Or see overloaded function activateStateBounds().
     * 
     * @param x_min minimum bounds on the state \f$ \mathbf{x}_k\f$ [p x 1].
     * @param x_max maximum bounds on the state \f$ \mathbf{x}_k\f$ [p x 1].
     * @param activate if set to \c true than the edge is activated.
     * 
     * @todo Avoid unnecessary calculations for unbounded values (Jacobian, Hessian, ...). Maybe change edge-dimension dynamically in that case.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), setSystemDynamics()
     */ 
    void activateStateBounds( const Eigen::Ref<const StateVector>& x_min , const Eigen::Ref<const StateVector>& x_max, bool activate=true)
    {
        _active_state_bounds = std::make_tuple(activate,x_min,x_max);
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }

	/**
	* @brief Activate predefined edge for state bounds.
	*
	* Adds the following inequality constraints to the optimization problem:
	*  - \f$ \mathbf{x}_k \le \mathbf{x}_{max}\f$ (component-wise)
	*  - \f$ \mathbf{x}_k \ge \mathbf{x}_{min}\f$ (component-wise)
	*
	* To just bound a subset of components of the state \f$ \mathbf{x}_k\f$,
	* set unbounded components to \f$ \textrm{-Inf} \le x_k \le \textrm{Inf} \f$ using the \c INF macro.
	* Or see overloaded function activateStateBounds().
	*
	* @param x_min minimum bounds on the state \f$ \mathbf{x}_k\f$ [p x 1] as std::array or std::initializer_list.
	* @param x_max maximum bounds on the state \f$ \mathbf{x}_k\f$ [p x 1] as std::array or std::initializer_list.
	* @param activate if set to \c true than the edge is activated.
	*
	* @todo Avoid unnecessary calculations for unbounded values (Jacobian, Hessian, ...). Maybe change edge-dimension dynamically in that case.
	*
	* @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), setSystemDynamics()
	*/
	void activateStateBounds(const std::array<double, p>& x_min, const std::array<double, p>& x_max, bool activate = true)
	{
		activateStateBounds(Eigen::Map<const StateVector>(x_min.data()), Eigen::Map<const StateVector>(x_max.data()), activate);
	}
    
    /**
     * @brief Activate predefined edge for state bounds (single component version)
     * 
     * Adds the following inequality constraints to the optimization problem:
     *  - \f$ \mathbf{x}_k \le \mathbf{x}_{max}\f$ (component-wise)
     *  - \f$ \mathbf{x}_k \ge \mathbf{x}_{min}\f$ (component-wise)
     * 
     * The default initialization of the bounds is \c -INF and \c INF respectively, if this predefined edge
     * is activated.
     * This version of activateStateBounds() makes use of the default initialization and just updates 
     * the bounds for index \c state_index.
     * All other bounds remain untouched (and if not changed before, they are bound to \c -INF and \c INF.
     * 
     * @param state_idx selected component of \f$ \mathbf{x}_k\f$ that should be bounded to \c x_min and \c x_max.
     * @param x_min minimum bound on the component with index \c state_idx of state \f$ \mathbf{x}_k\f$.
     * @param x_max maximum bound on the component with index \c state_idx of the state \f$ \mathbf{x}_k\f$.
     * @param activate if set to \c true than the edge is activated.
     * 
     * @todo Avoid unnecessary calculations for unbounded values (Jacobian, Hessian, ...). Maybe change edge-dimension dynamically in that case.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), setSystemDynamics()
     */ 
    void activateStateBounds( unsigned int state_idx, double x_min, double x_max, bool activate=true)
    {
        std::get<0>(_active_state_bounds) = activate;
        std::get<1>(_active_state_bounds)[state_idx] = x_min;
        std::get<2>(_active_state_bounds)[state_idx] = x_max;
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    

    /**
     * @brief Activate predefined edge for control input derivatives
     *
     * Define the control input vector components to be a single control and its deriviatives \f$ u, \dot{u}, ..., u^(p-1) \f$
     * This edge forces the components of the first ControlVertex \f$ \dot{u}_k, \ddot{u}_k \f$ to be fixed during optimization.
     * Therefore change these values inside the control input sequence in order to set initial values.
     *
     * @param activate if set to \c true than the edge is activated.
     *
     * @sa setPreviousControlInput(), isControlVectorSingleInputWithDerivatives(), buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateControlBounds(), setSystemDynamics()
     */
    void activateControlInputDerivatives(bool activate=true)
    {
        _active_control_input_derivatives = activate;
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Fix the TimeDiff \f$ \Delta T \f$ during optimization.
     * @sa setupHorizon(), setGoalStatesFixedOrUnfixed()
     * @param fixed Set to \c true in order to fix TebController::_dt.
     */
    void setFixedDt(bool fixed)
    {
        _dt.setFixedAll(fixed);
        // notify graph that vertices are changed
        _graph.notifyGraphModified();
    }

    
    /**
     * @brief Set system dynamics and activate the predifined equality-constraint-edge.
     * 
     * Add system dynamics equations to the optimization problem. System equations are defined
     * as a equality constraints since they should be satisfied all the time.
     * 
     * See SystemDynamics class for more details.
     * 
     * @param system pointer to the SystemDynamics (sub-)class that stores the system dynamics equations.
     * @param activate if set to \c true than the edge is activated.
     * 
     * @sa buildOptimizationGraph(), activateObjectiveTimeOptimal(), activateStateBounds(), activateControlBounds()
     */     
    void setSystemDynamics(SystemDynamics<p,q>* system ,bool activate=true)
    {
        _active_system_dynamics = std::make_pair(activate, system);
        // notify graph that new edges are demanded
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Store control input from a previous run / step for calculating input derivatives
     *
     * Store the control input from the previous run in order to enable the calculation of input derivatives
     * The corresponding equality constraints for a single input with q-1 derivatives can be enabled using
     * activateControlInputDerivatives().
     *
     * @param prev_control ControlVector of the previous run
     * @param dt the time elapsed (in sense of sampling time) until the current call
     *
     * @sa activateControlInputDerivatives(), isControlVectorSingleInputWithDerivatives()
     */
    void setPreviousControlInput(const Eigen::Ref<const ControlVector>& prev_control, double dt)
    {
        _prev_control_input.first = prev_control;
        _prev_control_input.second = dt;
    }
    
    /**
     * @brief Store control input from a previous run / step for calculating input derivatives
     *
     * Store the control input from the previous run in order to enable the calculation of input derivatives
     * The corresponding equality constraints for a single input with q-1 derivatives can be enabled using
     * activateControlInputDerivatives().
     *
     * @param prev_control double array containing q compoments of the control ControlVector of the previous run
     * @param dt the time elapsed (in sense of sampling time) until the current call
     *
     * @sa activateControlInputDerivatives(), isControlVectorSingleInputWithDerivatives()
     */
    void setPreviousControlInput(const std::array<double,q>& prev_control, double dt)
    {
        setPreviousControlInput(Eigen::Map<const ControlVector>(prev_control.data()),dt);
    }
    
    /**
     * @brief Check if the ControlVector contains derivatives
     *
     * @return \c true if activateControlInputDerivatives() was called before.
     *
     * @sa activateControlInputDerivatives(), setPreviousControlInput()
     */
    bool isControlVectorSingleInputWithDerivatives() const {return _active_control_input_derivatives;}
    
    
    /**
     * @brief Set algorithm to solve the underlying optimization problem.
     * 
     * Set subclasses of BaseSolver here. Solver classes are frequently expanded.
     * 
     * @remarks Subclasses of BaseSolverLeastSquares are least-squares solvers that are efficient,
     * 		but originally not suited for hard constraints. In addition, the objective function must be a quadratic function.
     * 		In order to allow similar functionalities, the cost/objective function is squared \f$ \min V^2(\mathcal{B}) \f$
     * 		and equality/inequality constraints are approximated by quadratic soft-constraints \cite rosmann2014.
     * 
     * @param solver Pointer to subclass of BaseSolver. TebController does not free memory of the solver class.
     * 
     * @sa BaseSolver BaseSolverLeastSquares
     */  
    void setSolver(BaseSolver* solver)
    {
        _solver = solver;
        _solver->setConfig(cfg);
    }
    
  
	void getActiveVertices(); //!< Collect active vertices and determine the Hessian start index for each vertex (StateVertex, ControlVertex and TimeDiff).



    Eigen::VectorXd getAbsoluteTimeVec() const; //!< Get vector of absolute times \f$ t=[0,\Delta T, 2 \Delta T, \dotsc, n \Delta T] \f$.
    
    // Debug functions
    Eigen::MatrixXd getStateCtrlInfoMat() const; //!< Get TEB states and control inputs in matrix form (Use for debugging).
    Eigen::Matrix<bool,p+q,-1> getTEBFixedMap() const; //!< Get TEB matrix as bool matrix: \c true: fixed, \c false = free (Use for debugging).
    Eigen::Matrix<double,-1,2> getSampleOptVecIdxVMat() const; //!< Get Hessian index (col 1) and number of free variables (col 2) for each VertexType (StateVertex, ControlVertex, TimeDiff) (rowwise).
                                                               
    
protected:
    
  
    /**
     * @brief Fix the complete first StateVector of the state sequence (start state vector).
     * @param fixed Set to \c true in order to fix the states. 
     */
    void setFixedStart(bool fixed)
    {
        assert(!_state_seq.empty());
        _state_seq.front().setFixedAll(fixed);
        // notify graph that vertices are modified
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Fix selected components of the first StateVector of the state sequence (start state vector).
     * @param fixed Boolean array with \a p components. Set i-th component to \c true in order to fix the corresponding state. 
     */
    void setFixedStart(const bool* fixed)
    {
		assert(!_state_seq.empty());
		_state_seq.front().setFixedStates(fixed);
        // notify graph that vertices are modified
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Fix the complete last StateVector of the state sequence (final/goal state vector).
     * @param fixed Set to \c true in order to fix the states. 
     */
    void setFixedGoal(bool fixed) {
		assert(!_state_seq.empty());
		_state_seq.back().setFixedAll(fixed);
        // notify graph that vertices are modified
        _graph.notifyGraphModified();
    }
    
    
    /**
     * @brief Fix selected components of the last StateVector of the state sequence (final state vector).
     * @param fixed Boolean array with \a p components. Set i-th component to \c true in order to fix the corresponding state. 
     */
    void setFixedGoal(const bool* fixed)
    {
		assert(!_state_seq.empty());
		_state_seq.back().setFixedStates(fixed);
        // notify graph that vertices are modified
        _graph.notifyGraphModified();
    }
    
    /**
     * @brief Fix selected components of the last StateVector of the state sequence (final state vector).
     * @param fixed Eigen::Vector with \a p booleans. Set i-th component to \c true in order to fix the corresponding state. 
     */
    void setFixedGoal(const Eigen::Ref<const Eigen::Matrix<bool,p,1>>& fixed)
    {
		assert(!_state_seq.empty());
		_state_seq.back().setFixedStates(fixed);
        // notify graph that vertices are modified
        _graph.notifyGraphModified();
    }

   

    
    // private variables
	StateSequence _state_seq; //!< Store the complete sequence of state vectors (represented as StateVertex for the representation in a hyper-graph)
	ControlSequence _ctrl_seq; //!< Store the complete sequence of contorl input vectors (represented as ControlVertex for the representation in a hyper-graph)
   
    /**
     * @brief TimeDiff vertex that represents the required transition time between consecutive states and control inputs.
     * Stores \f$ \Delta T \f$.
     * Another interpretation: discretization step width.
     * This implementation supports currently a uniformly distributed dt only.
     */ 
    TimeDiff _dt; 
    
    std::pair<ControlVector, double> _prev_control_input = std::make_pair(ControlVector::Zero(),1); //!< Store control input vector from the past (maybe previous step) in order to enable input derivatives and store time elapsed until the current call
    
    StateVector _goal_backup; //!< Store and backup goal vector (should be changed in all functions that set the goal point).
    
    int _no_samples = -1; //!< Number of samples for trajectory initialization (if -1, Config::Teb::n_pre is used, see setupHorizon()).
    double _dt_ref = -1; //!< Reference time difference (discretization) (if -1, Config::Teb::dt_ref is used, see setupHorizon()).
    
                  
    bool _optimized; //!< Status flag that is \c true, if at least one optimization step in optimizeTEB() is performed before.
    
    HyperGraph _graph; //!< Instance of the optimization hyper-graph to store all active vertices and edges (interface for the solver)
    
    FixedStates _fixed_goal_states = FixedStates::Constant(true); //!< Store information about (un-)/fixed components of the goal StateVector (initialized to fixed, see buildOptimizationGraph()).
    
    BaseSolver* _solver = nullptr; //!< Pointer to solver class specified using setSolver() or TebController().
    
    bool _cfg_owned = false; //!< If \c true, the Config TebController::cfg is owned by this class, that requires its deletion inside ~TebController().
    
    
    // ==== Activation status and storage for commonly used and predefined hyper-graph edges ====
    
    bool _active_time_optimal = false; //!< if \c true, EdgeMinimizeTime is activated (see activateObjectiveTimeOptimal()).
    
    /**
     * @brief Stores activation status of EdgeQuadraticForm
     * - First type (\c bool): if \c true, EdgeQuadraticForm is activated (quadratic cost on states and control inputs).
     * - Second type (\c Weight Q): Weight for the state vectors (uniform weight here)
     * - Third type (\c Weight R): Weight for the control input vectors (uniform weight here)
     * - Fourth type (\c Weight Qf): Weight for the final state vector (uniform weight here)
     * @sa activateObjectiveQuadraticForm(), EdgeControlBounds, buildOptimizationGraph()
     */
    std::tuple<bool, double, double, double> _active_quadratic_form = std::make_tuple(false,1,1,1);
    
    
    /**
     * @brief Stores activation status of EdgeSystemDynamics and a pointer to the problem specific SystemDynamics object.
     * - First type (\c bool): if \c true, EdgeSystemDynamics is activated.
     * - Second type (\c SystemDynamics): pointer to SystemDynamics object.  
     * @sa setSystemDynamics(), SystemDynamics, EdgeSystemDynamics, buildOptimizationGraph()
     */ 
    std::pair<bool,SystemDynamics<p, q>*> _active_system_dynamics = std::make_pair(false,nullptr);
    
    /**
     * @brief Define the control input vector components to be a single control and its deriviatives \f$ u, \dot{u}, ..., u^(p-1) \f$
     * - First type (\c bool): if \c true, EdgeInputDerivatives is activated.
     * @sa activateControlInputDerivatives(), EdgeInputDerivatives, SystemDynamics, EdgeSystemDynamics, buildOptimizationGraph()
     */
    bool _active_control_input_derivatives = false;
    
    /**
     * @brief Stores activation status of EdgeControlBounds and mininum/maximum bounds on each ControlVector.
     * - First type (\c bool): if \c true, EdgeControlBounds is activated.
     * - Second type (\c ControlVector): lower bound on each ControlVector \f$ u_{min} \f$ (component-wise).
     * - Third type (\c ControlVector): upper bound on each ControlVector \f$ u_{max} \f$ (component-wise).
     * @sa EdgeControlBounds, buildOptimizationGraph()
     */ 
    std::tuple<bool, ControlVector, ControlVector> _active_control_bounds = std::make_tuple(false,ControlVector::Constant(-INF),ControlVector::Constant(INF));
    
    /**
     * @brief Stores activation status of EdgeStateBounds and mininum/maximum bounds on each StateVector.
     * - First type (\c bool): if \c true, EdgeStateBounds is activated.
     * - Second type (\c StateVector): lower bound on each StateVector \f$ x_{min} \f$ (component-wise).
     * - Third type (\c StateVector): upper bound on each StateVector \f$ x_{max} \f$ (component-wise).
     * @sa EdgeStateBounds, buildOptimizationGraph()
     */ 
    std::tuple<bool, StateVector, StateVector> _active_state_bounds = std::make_tuple(false, StateVector::Constant(-INF), StateVector::Constant(INF));

    
public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};

} // end namespace teb


#include <teb_package/base/teb_controller.hpp>


// ======================================= EXAMPLES (DOXYGEN) ===================================================

/** 
 * @example integrator_system1.cpp 
 *
 * The following example shows the TEB controller 
 * applied to an integrator system with \a p integrator states:
 * \f[  T x^{(p)} = u \f]
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 * Considering the particular case \a p = 4.
 * The state vector is defined by \f$ \mathbf{x} = [x, \dot{x}, \ddot{x}, \dddot{x}]^T \f$.
 * Transforming ODE (1) into a state space corresponding to \f$ \mathbf{x} \f$ leads to:
 * \f[ 
 *      \begin{bmatrix}
 *          \dot{x} \\
 *          \ddot{x} \\
 * 	    \dddot{x} \\
 *          \ddddot{x} 
 *      \end{bmatrix} = 
 *      \begin{bmatrix}
 *          0 & 1 & 0 & 0 \\
 *          0 & 0 & 1 & 0 \\
 * 	    0 & 0 & 0 & 1 \\
 * 	    0 & 0 & 0 & 0
 *       \end{bmatrix}
 *      \begin{bmatrix}
 *          x \\
 *          \dot{x} \\
 * 	    \ddot{x} \\
 * 	    \dddot{x}
 *      \end{bmatrix} +
 *      \begin{bmatrix}
 *          0 \\
 * 	    0 \\
 * 	    0 \\
 *          1/T 
 *      \end{bmatrix} u \f]
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0, 0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0, 0 , 0]^T \f$.
 *
 * The system dynamics are specified using the IntegratorSystem class. 
 * See the following header file how to add the state space model mentioned above for an integrator with \a p states. \n
 * File \b integrator_system.h:
 *
 * @include integrator_system.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds 
 * and how to add the IntegratorSystem object specified in integrator_system.h. \n
 * The example program solves the optimization problem once without actually controlling the system.
 * See other examples for open-loop and closed-loop control applications. \n
 * The output of the program is as follows:
 * 
 * @image html integrator_system1.png
 * @image latex integrator_system1.png "Result of integrator_system1.cpp" width=0.99\linewidth
 * 
 * File \b integrator_system1.cpp
 */

// ========================================================================================================


/** 
 * @example integrator_system2.cpp 
 *
 * The following example shows the TEB controller 
 * applied to an integrator system with \a p integrator states:
 * \f[  T x^{(p)} = u \f]
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 * Considering the particular case \a p = 2.
 * The state vector is defined by \f$ \mathbf{x} = [x, \dot{x}]^T \f$.
 * Transforming ODE (1) into a state space corresponding to \f$ \mathbf{x} \f$ leads to:
 * \f[ 
 *      \begin{bmatrix}
 *          \dot{x} \\
 *          \ddot{x}
 *      \end{bmatrix} = 
 *      \begin{bmatrix}
 *          0 & 1 \\
 *          0 & 0
 *       \end{bmatrix}
 *      \begin{bmatrix}
 *          x \\
 *          \dot{x}
 *      \end{bmatrix} +
 *      \begin{bmatrix}
 *          0 \\
 *          1/T 
 *      \end{bmatrix} u \f]
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0]^T \f$.
 *
 * The system dynamics are specified using the IntegratorSystem class. 
 * See the following header file how to add the state space model mentioned above for an integrator with \a p states. \n
 * File \b integrator_system.h:
 *
 * @include integrator_system.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds 
 * and how to add the IntegratorSystem object specified in integrator_system.h. \n
 * Additionally, a Simulator object is initialized in order to perform closed-loop and open-loop simulations. \n
 * The example program compares the result for the open-loop control and the simulated closed-loop control.
 * In addition the first optimziation result (red) is shown. If the system dynamic equation (equality constraint)
 * is fully satisfied, red and green should be similar. They can only be identical, if the integrator method NumericalIntegrators:ForwardEuler
 * is used in simulation, since the TEB relies on the very same integration model. \n
 * The output of the program is as follows:
 * 
 * @image html integrator_system2.png
 * @image latex integrator_system2.png "Result of integrator_system2.cpp" width=0.99\linewidth
 * 
 * File \b integrator_system2.cpp
 */



// ========================================================================================================


/**
 * @example integrator_system_classic_mpc.cpp
 *
 * The following example shows a classic MPC controller (with a quadratic cost functional and a receding horizon)
 * applied to an integrator system with \a p integrator states:
 * \f[  T x^{(p)} = u \f]
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 * Considering the particular case \a p = 3.
 * The state vector is defined by \f$ \mathbf{x} = [x, \dot{x}, \ddot{x}]^T \f$.
 * Transforming ODE (1) into a state space corresponding to \f$ \mathbf{x} \f$ leads to:
 * \f[
 *      \begin{bmatrix}
 *          \dot{x} \\
 *          \ddot{x} \\
 *          \dddot{x}
 *      \end{bmatrix} =
 *      \begin{bmatrix}
 *          0 & 1 & 0 \\
 *          0 & 0 & 1 \\
 *          0 & 0 & 0
 *       \end{bmatrix}
 *      \begin{bmatrix}
 *          x \\
 *          \dot{x} \\
 *          \ddot{x}
 *      \end{bmatrix} +
 *      \begin{bmatrix}
 *          0 \\
 *          0 \\
 *          1/T
 *      \end{bmatrix} u \f]
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0]^T \f$ while minimizing the quadratic control effort and control energy:
 * \f[ J = (\mathbf{x}_n-\mathbf{x}_{f})^T \mathbf{Q}_f (\mathbf{x}_n-\mathbf{x}_{f}) + \sum_{k=0}^{n-1} [ (\mathbf{x}_k-\mathbf{x}_{f})^T \mathbf{Q} (\mathbf{x}_k-\mathbf{x}_{f}) + \mathbf{u}_k^T \mathbf{R} \mathbf{u}_k ] \f]
 *
 * The receding/moving horizon is set to \f$ n = 10 \f$.
 * The system dynamics are descretized using finite differences and with a uniform and fixed resolution (\f$ \Delta T = 0.1 \f$s).
 *
 * The continuous system dynamics are specified using the IntegratorSystem class.
 * See the following header file how to add the state space model mentioned above for an integrator with \a p states. \n
 * File \b integrator_system.h:
 *
 * @include integrator_system.h
 *
 * The following cpp-file shows how to create the TebController object with common MPC characteristics, how to add control bounds
 * and how to add the IntegratorSystem object specified in integrator_system.h. \n
 * Additionally, a Simulator object is initialized in order to perform closed-loop and open-loop simulations. \n
 * The example program compares the result for the open-loop control and the simulated closed-loop control.
 * In addition the first optimziation result (red) is shown (the first horizon!).\n
 * The output of the program is as follows:
 *
 * @image html integrator_system_classic_mpc.png
 * @image latex integrator_system_classic_mpc.png "Result of integrator_system_classic_mpc.cpp" width=0.99\linewidth
 *
 * File \b integrator_system_classic_mpc.cpp
 */



// ========================================================================================================


/** 
 * @example integrator_system_mex.cpp 
 *
 * The following examples clarifies, how to prepare the TEB Controller (in particular the Controller of the IntegratorSystem example)
 * for the usage in Matlab as a mex function. \n
 * The header file remains unchainged. File \b integrator_system.h:
 *
 * @include integrator_system.h
 *
 * The following cpp-file shows how embed the TebController object into Matlab's function wrapper \c mexFunction.
 * The example also wraps the mex function into a Matlab class. The controller class object is transformed to a Matlab
 * handle using a small help (undocumented) helper class teb::matlab::matlabClassHandle().
 * See the Matlab class \c control_interface.cpp in \e examples/matlab_interface directory for more details on how to use it in Matlab. \n
 * An alternate approach without the helper class would be to store a persistent object (and its pointer) within the file (like for the Solver and 
 * System class).
 * \b Note, the wrapper class can be easily adapted to other controllers by just changing the typedefs at the top of the file. Controller settings can be modified below
 * the comment \e // \e Setup \e Controller.
 * 
 * File \b integrator_system_mex.cpp
 */


// ========================================================================================================


/** 
 * @example integrator_system_sfun.cpp 
 *
 * The following examples clarifies, how to prepare the TEB Controller (in particular the Controller of the IntegratorSystem example)
 * for the usage in Matlab Simulink as a S-Function block. \n
 * The header file remains unchainged. File \b integrator_system.h:
 *
 * @include integrator_system.h
 *
 * The following cpp-file shows how embed the TebController object into Matlab's S-Function wrappers.
 * 
 * File \b integrator_system_sfun.cpp
 */


// ========================================================================================================

/**
 * @example linear_system_ode.cpp
 *
 * The following example shows the TEB controller
 * applied to a common linear system described by an ordinary differential equation of the \a p -th order:
 * \f[  a_p x^{(p)}(t) + a_{p-1} x^{(p-1)}(t) + \dotsc + a_1 \dot{x}(t) + a_0 x(t) = b u(t) \f]
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 * Let \f$ p=2 \f$, \f$ b=1 \f$ and \f$ \mathbf{a} = [a_2, a_1, a_0] = [0.8, 0.9, 1.0] \f$
 * that leads to the following ode:
 * \f[  0.8 \ddot{x}(t) + 0.9 \dot{x}(t) + 1.0 x(t) = u(t) \f]
 * The state vector is defined by \f$ \mathbf{x} = [x, \dot{x}]^T \f$.
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0]^T \f$.
 *
 * The system dynamics are specified using the LinearSystemODE class.
 * See the following header file how to add the state space model mentioned above. \n
 * File \b linear_system_ode.h:
 *
 * @include linear_system_ode.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds
 * and how to add the LinearSystemODE object specified in linear_system_ode.h. \n
 * The example program solves the optimization problem once without actually controlling the system.
 * See other examples for open-loop and closed-loop control applications. \n
 * The output of the program is as follows:
 *
 * @image html linear_system_ode.png
 * @image latex linear_system_ode.png "Result of linear_system_ode.cpp" width=0.99\linewidth
 *
 * File \b linear_system_ode.cpp
 */



// ========================================================================================================



/**
 * @example linear_system_state_space.cpp
 *
 * The following example shows the TEB controller
 * applied to a common linear system described by a continuous-time state space model of the \a p -th order:
 * \f[  \dot{\mathbf{x}} = \mathbf{A} \mathbf{x} + \mathbf{B} \mathbf{u} \qquad \mathbf{x} \in \mathbb{R}^p, \ \mathbf{u} \in \mathbb{R}^q  \f]
 *
 * For this example, let \f$ p=2 \f$ and \f$ q = 1 \f$.
 * We define \f$ \mathbf{A} = \bigl[ \begin{smallmatrix} 0 & 1 \\ 0 & 1 \end{smallmatrix} \bigr] \f$ and \f$ \mathbf{b} = [0, 1]^T \f$. \n
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0]^T \f$.
 *
 * The system dynamics are specified using the LinearSystemStateSpace class.
 * See the following header file how to add the state space model mentioned above. \n
 * File \b linear_system_state_space.h:
 *
 * @include linear_system_state_space.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds
 * and how to add the LinearSystemStateSpace object specified in linear_system_state_space.h. \n
 * The example program solves the optimization problem once without actually controlling the system.
 * See other examples for open-loop and closed-loop control applications. \n
 * The output of the program is as follows:
 *
 * @image html linear_system_state_space.png
 * @image latex linear_system_state_space.png "Result of linear_system_state_space.cpp" width=0.99\linewidth
 *
 * File \b linear_system_state_space.cpp
 */


// ========================================================================================================

/**
 * @example linear_system_ode_ctrl_comparison.cpp
 *
 * The following example compares the time-optimal TEB controller with the common quadratic form MPC. \n
 * The controlled system is specified using an (unstable) ordinary differential equation (refer to other examples for more details).
 *
 * The system dynamics are specified using the LinearSystemODE class.
 * See the following header file how to add the state space model mentioned above. \n
 * File \b linear_system_ode.h:
 *
 * @include linear_system_ode.h
 *
 * The following cpp-file shows how to create both controllers, how to add control bounds
 * and how to add the LinearSystemODE object specified in linear_system_ode.h. \n
 *
 * The output of the program is as follows (closed-loop results only):
 *
 * @image html linear_system_ode_ctrl_comparison.png
 * @image latex linear_system_ode_ctrl_comparison.png "Result of linear_system_ode_ctrl_comparison.png" width=0.99\linewidth
 *
 * File \b linear_system_ode_ctrl_comparison.cpp
 */

// ========================================================================================================



/** 
 * @example van_der_pol_system.cpp 
 *
 * The following example shows the TEB controller
 * applied to the Van der Pol oscillator (http://en.wikipedia.org/wiki/Van_der_Pol_oscillator)
 * \f[  \ddot{x}(t) - a (1 - x^2(t)) \dot{x}(t) + x(t) = u(t) \f]
 * The control input \f$ u \f$ should be bounded to the interval [-1, 1].
 * \n
 * The state vector is defined by \f$ \mathbf{x} = [x, \dot{x}]^T \f$.
 * In this example it is \f$ a = 1 \f$.
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0]^T \f$
 * to the final state \f$ \mathbf{x}_f = [1, 0]^T \f$.
 *
 * The system dynamics are specified using the VanDerPolSystem class.
 * See the following header file how to add the nonlinear state space model mentioned above. \n
 * File \b van_der_pol_system.h:
 *
 * @include van_der_pol_system.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds
 * and how to add the VanDerPolSystem object specified in van_der_pol_system.h. \n
 * The example program solves the optimization problem once without actually controlling the system.
 * See other examples for open-loop and closed-loop control applications. \n
 * The output of the program is as follows:
 *
 * @image html van_der_pol_system.png
 * @image latex van_der_pol_system.png "Result of van_der_pol_system.cpp" width=0.99\linewidth
 *
 * File \b van_der_pol_system.cpp
 */


// ========================================================================================================



/**
 * @example rocket_system.cpp
 *
 * The following example shows the TEB controller
 * applied to a simple free space rocket model with three differential states \a s, \a v and \a m.
 * (http://acado.sourceforge.net/doc/html/d9/d65/example_001.html)
 * \f{eqnarray}
 *   \dot{s}(t) &=& v(t) \\
 *   \dot{v}(t) &=& \frac{u(t) - 0.02 v^2(t)}{m(t)} \\
 *   \dot{m}(t) &=& -0.01 u^2(t)
 * \f}
 * The control input \f$ u \f$ should be bounded to the interval [-1.1, 1.1].
 * In addition the state \f$ v \f$ should be bounded to the interval [-0.5, 1.7].
 * \n
 * The state vector is defined by \f$ \mathbf{x} = [s, v, m]^T \f$.
 *
 * The objective of the control task is to transite the system from the start state \f$ \mathbf{x}_s = [0, 0, 1]^T \f$
 * to the final state \f$ \mathbf{x}_f = [10, 0, -]^T \f$. \n
 * In addition to the other examples, the third state of the final state vector is not fixed. The final mass is not known a-prioi and can be chosen by
 * the optimizer in order to minimze the objective function.
 * But we initialize/estimate the final value to \f$ m_f = 0.5 \f$.
 *
 * The system dynamics are specified using the FreeSpaceRocketSystem class.
 * See the following header file how to add the nonlinear state space model mentioned above. \n
 * File \b rocket_system.h:
 *
 * @include rocket_system.h
 *
 * The following cpp-file shows how to create the TebController object, how to add control bounds
 * and how to add the FreeSpaceRocketSystem object specified in van_der_pol_system.h. \n
 * The example program solves the optimization problem once without actually controlling the system.
 * See other examples for open-loop and closed-loop control applications. \n
 * The output of the program is as follows (\b Note, the number of iterations and weights of the soft constraints is too low using the default settings,
 * therefore the constraints are not satisfied completely after the first optimization):
 *
 * @image html rocket_system.png
 * @image latex rocket_system.png "Result of rocket_system.cpp" width=0.99\linewidth
 *
 * File \b rocket_system.cpp
 */



/**
 * @example mobile_robot_teb.cpp
 * 
 * This example shows how to build a customized controller that minimizes a used-defined
 * optimization problem rather than using default optimization functions. \n
 * 
 * 
 * Part of the example is a mobile robot trajectory planning and control application.
 * The optimization problem is given by the original timed-elastic-band presented in \cite rosmann2012.\n
 * For the sake of simplicity, we ignore some optional cost-functions like constraints on accelerations and the forward drive direction. In addition, we apply only a uniform \f$ \Delta T \f$. \n
 * 
 * 
 * We first start by introducing the optimization problem. 
 * The robot pose is defined by \f$ \mathbf{x} = [x, y, \beta]^T \f$.
 * The robot trajectory is than given by the sequence of \f$ n \in \mathbb{N} \f$ poses \f$ \mathbf{x}_i, i=1,\dotsc,n \f$. Between two consecutive poses \f$ \{\mathbf{x}_i,\mathbf{x}_{i+1}\} \f$
 * we define the time difference \f$ \Delta T \f$, that the robot requires to transit from the a configuration to the subsequent one in the sequence.\n
 * The robot hardware interface accepts translational and rotational velocities \f$ \mathbf{u} = [v, \omega]^T \f$ as control inputs. \n
 * 
 * @image html robot_teb_sequence.png "Mobile robot trajectory representation"
 * @image latex robot_teb_sequence.png "Mobile robot trajectory representation" width=0.3\linewidth
 * 
 * In conrast to common MPC approaches, the sequence of poses and the time difference are part of the optimzation rather than the control inputs \f$ \mathbf{u} \f$,
 * because they are implicitly part of the pose sequence and they can be derived via difference quotients.
 * 
 * The joint trajectory respentation is than defined as follows:
 * \f[ \mathcal{B} := \{\mathbf{x}_1, \mathbf{x}_2, \dotsc, \mathbf{x}_{n-1},\mathbf{x}_n, \Delta T \} \f]
 * 
 * 
 * In the following we construct the optimization problem. \n
 * 
 * A general cost function can often be decomposed into the sum of \f$ z \f$ "local" cost terms:
 * \f[  V(\mathcal{B}) = f(\mathcal{B}) = \sum_i^z f_i(\mathcal{B}_i) \f]
 * \f$ f_i \f$ denotes a local cost term and \f$ \mathcal{B}_i \subseteq \mathcal{B}, i=1,\dotsc,z \f$ denotes a (small) subset of \f$ \mathcal{B} \f$. \n
 * Obviously, if the subset of depending optimization variables is low for each local cost term, the resuling optimization problem has a sparse structure. To exploit this structure later,
 * a hyper-graph (like mentioned) is ideally suited to capture this property already during the problem formulation phase.
 * 
 * 
 * Optimization problems in this package are formulated as hyper-graphs (according to \cite kummerle2011) with additional distinction between constraints and objective functions.
 * A hyper-graph is a graph, in which edges connect an arbitary number of vertices (not just two). In sense of optimization, edges represent local cost function terms \f$ f_k(\mathcal{B}_i) \f$
 * and vertices represent the optimization variables (in this package commonly \f$ \mathbf{x}_k, \mathbf{u}_k, \Delta T \f$). Consequently for the mobile robot problem,
 * vertices are: \f$ \mathbf{x}_k \f$ and \f$ \Delta T \f$ (see the image a few lines below for an example hyper-graph).\n
 * 
 * The considered mobile robot has two wheels connected with a differential drive.
 * The formulation of the optimization problem in \cite rosmann2012 does not take a common robot motion model into account.
 * The kinodynamic motion behavior (constraints on robot / wheel velocities / accelerations and the non-holonomic kinematic) is captured using the superposition of dedicated
 * cost functions for each objective or constraint. For the sake of simplicty we reduce the optimization task to just limit velocities of the center of the robot, to satisfy non-holonomic kinematics and
 * to avoid obstacles:
 * \f[  V(\mathcal{B}) = f(\mathcal{B}) = f_{mintime}(\Delta T) + \sum_{i=1}^{n-1} [ f_{i,vel}(\mathcal{B}) + f_{i,kin}(\mathcal{B}) + f_{i,obst}(\mathcal{B}) ] \f]
 * Each cost term above is a squared function and hence the optimization problem can be solved using the Levenberg-Marquardt algorithm. Bounding the velocity, satisfying a minimum
 * distance to obstacles and satisfying the kinematic constraints are implemented using soft-constraints in \cite rosmann2011.
 * 
 * Within this MPC package, we have the oppertunity to specify constraints in a more genalized way. And if the selected solver is a least-squares solver (like teb::SolverLevenbergMarquardtEigenSparse), than
 * the soft-constraint approach is applied in similar way like in \cite rosmann2012.
 * As a result, we can generalize the above optimization problem for the mobile robot to:
 * \f{eqnarray}
 *   & & V^*(\mathcal{B}) = \min f_{mintime}(\Delta T)\\
 *   &\textrm{s.t.:} & \nonumber \\
 *   &\quad & \mathbf{x}_1 = \mathbf{x}_s \\
 *   &\quad & \mathbf{x}_n = \mathbf{x}_f\\
 *   &\quad & \mathbf{h}_{i,kin} (\mathcal{B}) = \mathbf{0} \quad i \in 1,\dotsc,n-1 \\
 *   &\quad & \mathbf{g}_{i,vel} (\mathcal{B}) \le \mathbf{0} \quad i \in 1,\dotsc,n-1 \\
 *   &\quad & \mathbf{g}_{i,obst} (\mathcal{B}) \le \mathbf{0} \quad i \in 1,\dotsc,n-1
 *  \f}
 * \f$ \mathbf{h} \f$ and \f$ \mathbf{g} \f$ denote the equality and inequality constraints without squared soft-constraint approximations. \n
 * According to \cite rosmann2012 the cost terms are as follows (\b without \b soft-constraint \b approx and \b without \b taking \b squared):
 * - Minimum transition time: \f$  f_{mintime}(\Delta T) = (n-1) \Delta T \f$ (Note, least-square solvers square all objectives later!)
 * - Limit velocity: \f{eqnarray*} 
 *  	\Delta \mathbf{s} &=& \begin{bmatrix} 
 * 		\left|\left| \begin{bmatrix} x_{i+1} - x_{i} \\ y_{i+1} - y_i \end{bmatrix} \right|\right| \\
 * 		\operatorname{normAngle}( \beta_{i+1} - \beta_i )
 * 	\end{bmatrix} \\
 * 	\mathbf{g}_{i,vel} (\mathcal{B}) &=& 
 * 	\frac{1}{\Delta T}
 * 	\begin{bmatrix}
 *	\Delta \mathbf{s} \\
 * 	-\Delta \mathbf{s} 
 * 	\end{bmatrix} +
 * 	\begin{bmatrix}
 * 		-v_{max} \\ -\omega_{max} \\ v_{min} \\ \omega_{min}
 * 	\end{bmatrix} \f}
 * - Nonholonomic Kinematics: \f{eqnarray*}
 * \mathbf{d}_i &=& \begin{bmatrix} x_{i+1} - x_i \\ y_{i+1} - y_i \\ 0 \end{bmatrix} \\
 * \mathbf{h}_{i,kin} &=& \left|\left| \left[\begin{bmatrix} \cos \beta_i \\ \sin \beta_i \\ 0 \end{bmatrix} +
 * 	\begin{bmatrix} \cos \beta_{i+1} \\ \sin \beta_{i+1} \\ 0 \end{bmatrix} \right] \times \mathbf{d}_i \right|\right|
 * \f}
 * - Obstacle avoidance (State constraint): \f[
 * \mathbf{g}_{i,obst} = -\left| \left| \begin{bmatrix} x_i - x_{obst} \\ y_i - y_{obst} \end{bmatrix} \right| \right| + d_{min}
 * \f]
 * 
 * Please note, that using a least-squares soft-constrained solver (as mentioned above), all constraints and objectives are incorporated in a weighted objective/cost function and are becoming squared.
 * The above optimization problem can easily be transfered to a hyper-graph. See the following figure for the resulting example graph. All indices are increased by one subsequently per constraint/objective type.
 * 
 * @image html robot_teb_hyper_graph2.svg "Example hyper-graph of the mobile robot application"
 * @image latex robot_teb_hyper_graph2.png "Example hyper-graph of the mobile robot application" width=0.5\linewidth
 * 
 * The first state \f$ \mathbf{x}_1 \f$ is marked with a double circle. That means the state is fixed during optimization (to define the current/initial state).
 * The goal state is fixed as well (according to a fixed horizon). Fixing these values implicitly satisfies \f$ \mathbf{x}_1 = \mathbf{x}_s \f$ and \f$ \mathbf{x}_n = \mathbf{x}_f \f$ of the problem definition above. \n
 * In contrast to \cite rosmann2012 we are using a single \f$ \Delta T \f$ that is part of all velocity edges (note the connection of \f$ \mathbf{g}_1, \mathbf{g}_3 \f$ and \f$ \Delta T \f$ in the example graph.
 * Formulating the optimization problem as a hyper-graph immediately indicates the structure of the underlying hessian/jabian matrix.
 * For example, considering the ordering of the optimization variables of \f$ \mathcal{B} \f$, than the hessian of the Lagrangian or the hessian of a least-square cost-function with soft-constraints would contain a band diagonal
 * (since the maximum number of connected states \f$ \mathbf{x}_i \f$ is at least two. Only the velocity bound edges that include always the same \f$ \Delta T \f$ would lead to a dense last row and column in the matrix.
 * 
 * The following header-file demonstrates how to implement the "local" cost functions mentioned above (please refer to teb::BaseEdge for an overview of possible edges to derive from):\n
 * File \b rob_cost_edges.h:
 * @include rob_cost_edges.h
 * 
 * In order to add the hyper-graph to the TebController, you can derive the class teb::TebController an override the method teb::TebController::customOptimizationGraph().
 * This ensures that the default edges like teb::EdgeMinimizeTime or teb::EdgeQuadraticForm can be added as well using the controller API. You can change states to fixed or unfixed as well. (Please note, that the default
 * fixing and unfixing of the start and goal state as well as the default edges for the hyper-graph are applied in teb::TebController::buildOptimizationGraph().
 * 
 * 
 * The derived class for the mobile-robot example is defined as follows. (\b Note, since the problem is defined with only implicit control inputs, by means of \c q=0, the teb::TebController::firstControl() 
 * and teb::TebController::returnControlInputSequence() are overridden as well. Additionally, we extend the new controller API with more debug and visualization functions.\n
 * File \b rob_controller.h:
 * @include rob_controller.h
 * 
 * 
 * Furthermore, for simulation purposes we define a teb::SystemDynamics model with simple integrator dynamics and the non-holonomic kinematics: \n
 * File \b mobile_robot_teb.h:
 * @include mobile_robot_teb.h
 * 
 * 
 * Finally, the main function that creates all objects, defines the optimization weights an initializes the simulation can be found in \b mobile_robot_teb.cpp (see below at the end). \n
 * The output of the program is as follows (the \b obstacle is placed at \f$ x_{obst}=2.5\,\text{m} \f$ and \f$ y_{obst}=0.1\,\text{m} \f$):
 *
 * @image html mobile_rob_states.png "TEB States after the first open-loop optimization"
 * @image latex mobile_rob_states.png "TEB States after the first open-loop optimization" width=0.6\linewidth
 * @image html mobile_rob_xy.png "X-Y Trajectory after the first open-loop optimization"
 * @image latex mobile_rob_xy.png "X-Y Trajectory after the first open-loop optimization" width=0.6\linewidth
 * @image html mobile_rob_vel.png "Velocity profile after the first open-loop optimization"
 * @image latex mobile_rob_vel.png "Velocity profile after the first open-loop optimization" width=0.6\linewidth
 * @image html mobile_rob_closedloop.png "Closed-loop simulation of the mobile robot"
 * @image latex mobile_rob_closedloop.png "Closed-loop simulation of the mobile robot" width=0.99\linewidth
 * 
 * File \b mobile_robot_teb.cpp:
 */



#endif /* defined(__teb_package__teb_controller__) */
