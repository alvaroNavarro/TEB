namespace teb
{

#include <teb_package/base/teb_controller.h>

/**
 * This basic implementation linearly interpolates the trajectory
 * between start \f$ \mathbf{x}_0 \f$ and final state \f$ \mathbf{x}_f \f$.
 * Control inputs \f$ \mathbf{u}_i \f$ are intialized to zero.
 * This method also updates the number of discrete samples (state and control input vector pairs) \a n.
 * 
 * All existing states and control inputs of the sequences TebController::_state_seq and TebController::_ctrl_seq are deleted first.
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 *
 * @param x0 copy of the current start state vector [p x 1]
 * @param xf current final state vector [p x 1]
 * @param n number of discrete samples (state and control input vector pairs) between start and goal
 */
template <int p, int q>
void TebController<p, q>::initTrajectory(const Eigen::Ref<const StateVector>& x0, const Eigen::Ref<const StateVector>& xf, unsigned int n)
{
    // clear current state and control sequences, but copy start, since it could be part of the teb vector, that will be cleared...
	StateVector x0_tmp = x0;

    _state_seq.clear(); //! @todo More efficient implementation that keeps previously allocated memory
	_ctrl_seq.clear();

    // create straight line in state space
	StateVector state_diff = xf - x0_tmp;
    double step_length = 1/(double(n)-1);
    const ControlVector zero_control = ControlVector::Zero();
    
    for (unsigned int i=0; i<n; ++i)
    {
		pushBackStateControlInputPair(x0_tmp + double(i)*step_length*state_diff, zero_control);
    }
    
    // set dt to dt_ref
    _dt.dt() = _dt_ref==-1 ? cfg->teb.dt_ref : _dt_ref;
    
    // update no_samples
    _no_samples = n;
    
    // backup goal state
    _goal_backup = xf;
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();
}

/**
 * Resizing the trajectory is helpful e.g. for the following
 * scenarios:
 *     - Large disturbances or obstacles requires the teb to
 *       be extended in order to satisfy the given sample rate
 *       and avoid undesirable behavior due to a large/small
 *       discretization step width \f$ \Delta T \f$. After clearance
 *       of disturbances/obstacles the teb should contract to
 *       its (time-)optimal trajectory again.
 *     - If the distance to the goal state is getting smaller,
 *       dt is decreasing as well. This leads to a heavily
 *       fine-grained discretization in combination with many
 *       discrete samples. Thus, the computation time will
 *       be/remain high and in addition numerical instabilities
 *       can appear (e.g. due to the division by a small \f$ \Delta T \f$).
 *
 * The implemented stragety observes the timediff \f$ \Delta T \f$.
 * and
 *     - inserts a new sample if \f$ \Delta T > \Delta T_{ref} + \Delta T_{hyst} \f$
 *     - removes a sample if \f$ \Delta T < \Delta T_{ref} - \Delta T_{hyst} \f$
 *
 * \f$ \Delta T_{ref} \f$ and \f$ \Delta T_{hyst} \f$ can be changed in Config::teb.
 * 
 * Since the \f$ \Delta T \f$ is currently chosen uniformly, no information
 * about a possible position (at which the new sample should
 * be inserted) can be obtained. This function resamples linearly
 * the whole trajectory using resampleTrajectory().
 * 
 * If the time difference vertex TimeDiff is fixed during optimization
 * (TimeDiff::_fixed == \c true , TimeDiff::isFixedAll()), this method returns without any
 * changes.
 */
template <int p, int q>
void TebController<p,q>::resizeTrajectory()
{
    if (_dt.isFixedAll() || !_optimized) return;
    
    if (_dt_ref==-1) _dt_ref = cfg->teb.dt_ref;
    
    unsigned int n = getN();
    if (_dt.dt() > _dt_ref + cfg->teb.dt_hyst && n < cfg->teb.n_max)
    {
        resampleTrajectory(n+1);
    }
    else if (_dt.dt() < _dt_ref - cfg->teb.dt_hyst && n > cfg->teb.n_min)
    {
        resampleTrajectory(n-1);
    }
    
}

/**
 * This function changes the number of samples by resampling the
 * complete trajectories TebController::_state_seq and TebController::_ctrl_seq (\f$ \mathbf{x}_i \f$, \f$ \mathbf{u}_i \f$, \f$ \Delta T \f$).
 * Currently only linear interpolation is implemented.
 *
 * This function may called withing resizeTrajectory() that is part of the
 * TEB optimization loop (see optimizeTEB()).
 *
 * To add/remove a single state refer to insertSample() / removeSample().
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 *
 * @param n_new new number of samples
 */
template <int p, int q>
void TebController<p,q>::resampleTrajectory(unsigned int n_new)
{
    unsigned int n = getN();
    if (n==n_new) return;

	assert(n == getM() && "Resampling the trajectory is currently only supported for control and state sequences with getN()==getM()");
    
    // copy states
    //! @todo More efficient strategy without copying the complete sequences.
	StateSequence state_seq_old = _state_seq;
	ControlSequence ctrl_seq_old = _ctrl_seq;
        
    // get new time diff
    double dt_new = _dt.dt() * double(n-1)/double(n_new-1);
    
    double t_new;
    unsigned int idx_old=1;
    double t_old_p1=_dt.dt(); // time for old sample with index idx_old (acutally its the subsequent time step w.r.t t_new)
    
    for (unsigned int idx_new = 1; idx_new < n_new-1; ++idx_new) // n_new-1 since last state is identical and copied later. idx_new=1, since start sample is already valid (we do not touch it)
							// we allow a small mismatch for the control u_1 and let the optimizer correct it later
    {
        t_new = dt_new * double(idx_new);
        while ( t_new > double(idx_old)*_dt.dt() && idx_old<n ) {++idx_old;}; // find idx_old that represents the state subsequent to the new one (w.r.t. time)
        t_old_p1 = double(idx_old) * _dt.dt();
	
		if (idx_new < n-1)
		{
			_state_seq.at(idx_new).states() = state_seq_old.at(idx_old - 1).states() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(state_seq_old.at(idx_old).states() - state_seq_old.at(idx_old - 1).states());
			_ctrl_seq.at(idx_new).controls() = ctrl_seq_old.at(idx_old - 1).controls() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(ctrl_seq_old.at(idx_old).controls() - ctrl_seq_old.at(idx_old - 1).controls()); // t_old_p1-_dt = t_old
		}
		else if (idx_new == n - 1)
		{
				_state_seq.at(idx_new).states() = state_seq_old.at(idx_old - 1).states() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(state_seq_old.at(idx_old).states() - state_seq_old.at(idx_old - 1).states());
				_ctrl_seq.at(idx_new).controls() = ctrl_seq_old.at(idx_old - 1).controls() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(ctrl_seq_old.at(idx_old).controls() - ctrl_seq_old.at(idx_old - 1).controls()); // t_old_p1-_dt = t_old
				// Force last "old" vertices to be unfixed, since they could be fixed
				//!@ todo If someone freezes states within the sequence (in addition to start and goal), it is not set to unfreezed here... But we do not want to force all states to be unfreezed due to lack of efficiency...
				_state_seq.at(idx_new).setFixedAll(false);
				_ctrl_seq.at(idx_new).setFixedAll(false);
		}
		else
		{
			pushBackStateControlInputPair(state_seq_old.at(idx_old - 1).states() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(state_seq_old.at(idx_old).states() - state_seq_old.at(idx_old - 1).states()),
									ctrl_seq_old.at(idx_old - 1).controls() + (t_new - (t_old_p1 - _dt.dt())) / _dt.dt()*(ctrl_seq_old.at(idx_old).controls() - ctrl_seq_old.at(idx_old - 1).controls()));
		}
    }
    
    // clear invalid states
	if (n_new != n)
	{
		_state_seq.resize(n_new);
		_ctrl_seq.resize(n_new); //!@todo: later this should be n_new-1
	}
    
    // add goal
	_state_seq.back() = state_seq_old.back();  // t_old_p1-_dt = t_old
	_ctrl_seq.back() = ctrl_seq_old.back();  // t_old_p1-_dt = t_old
    
    // save new dt
    _dt.dt() = dt_new;
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();
}


/**
 * Update first/start StateVector (start of the trajectory) with a
 * new one (x_start). This can be obtained from the most
 * recent measurements.
 * The teb vector is pruned if a state with idx i>0 is found, that has
 * a smaller distance (l2-norm) to the new x0 in comparance to firstState().
 * 
 * If the TEB vector is empty, the start state is added anyway (as single state trajectory).
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 *  
 * @param x0 new start state vector [p x 1]
 * 
 * @sa updateGoal(), initTrajectory()
 */
template <int p, int q>
void TebController<p,q>::updateStart(const Eigen::Ref<const StateVector>& x0)
{
    if (_state_seq.empty()) // if seq is empty
    {
        // add just start and init trajectory after goal is updated
        pushBackStateControlInputPair(x0, ControlVector::Zero());
    }
    else
    {
	
        Eigen::VectorXd teb_x0 = firstStateRef();

        // update only, if something is changed
        if (x0 != teb_x0)
        {
            // find nearest state (using l2-norm)
            double dist_cache = (x0-teb_x0).norm();
            double dist;
            unsigned int max_iter = std::min<int>(getN()-cfg->teb.n_min,40); // satisfy n_min, otherwise max 40 samples
            
            for (unsigned int i=1; i<max_iter; ++i)
            {
                dist = (x0-_state_seq.at(i).states()).norm();
                if (dist<dist_cache)
                {
                    dist_cache = dist;
					_state_seq.pop_front(); // delete first teb state if second one is nearer to current x0
					_ctrl_seq.pop_front(); // delete corresponding input sequence as well
                }
                else break;
            }
            // update start
			_state_seq.front().setStates(x0);
        }
        
    }
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();
}

/**
 * Update goal/final StateVector.
 * If getN()==1 (that is if only start state exists after setting updateStart())
 * than initTrajectory() is called to initialize a trajectory between previously defiend start and xf.
 * 
 * If an existing goal (lastState()) is far away (Config::Teb::goal_dist_force_reinit),
 * the trajectory is reinitialized using initTrajectory() as well.
 * 
 * If the goal is not completely fixed during optimization, only unfixed components of
 * the final state are updated.
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 * 
 * @param xf new final state vector [p x 1]
 */
template <int p, int q>
void TebController<p,q>::updateGoal(const Eigen::Ref<const StateVector>& xf)
{
    if (getN()==1)
    {
		// Only start state is available
		// Replan trajectory
        initTrajectory(firstStateRef(), xf, _no_samples==-1 ? cfg->teb.n_pre : _no_samples);
    }
    else
    {
        StateVector teb_xf = lastStateRef();
        
        // copy unfreezed goal state variables (since they should not be updated)
        StateVector new_xf = _state_seq.back().fixed_states().select(xf,teb_xf); // component-wise (expr ? a : b);

        // reinit if goal is far away
        if ( (new_xf-teb_xf).norm() > cfg->teb.goal_dist_force_reinit )
        {
            initTrajectory(firstStateRef(), new_xf, getN()); // goal is backuped inside this method
        }
        else
        {
			_state_seq.back().setStates(new_xf);
            
            // backup and store new goal to a member variable
            // Can be accessed if the goal is completely unfixed.
            _goal_backup = lastStateRef();
        }
        
    }
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();

}

/**
 * Start the actual TEB optimization routine.
 * 
 * The outer-loop resizes the trajectory (refer to resizeTrajectory() for infos),
 * initializes the optimization problem (e.g. by creating the hyper-graph),
 * and it calls the BaseSolver::solve() routine of the selected solver.
 * The number of outer-loop iterations can be set in Config::Teb::teb_iter.
 * 
 * @todo The current implementation is really inefficient, since in each outer-loop-iteration
 * 	 all hyper-graph edges are deleted and afterwards reinstantiated. Create a dedicated update function
 * 	 in the future to hot-start from a previous hyper-graph.
 * 
 * @sa initOptimization(), resizeTrajectory(), BaseSolver::solve(), setSolver()
 */
template <int p, int q>
void TebController<p,q>::optimizeTEB()
{
    for (unsigned int i=0; i < cfg->teb.teb_iter; ++i)
    {
		resizeTrajectory();
		//_graph.clearGraph();
		initOptimization();

		if (_solver) _solver->solve( &_graph );
		//else PRINT_ERROR("Cannot optimize TEB. No solver selected. Use setSolver() method.");
	
		_optimized = true;
		
		// remove modified flag inside graph since structure has been build
		// We do this after calling the solve() method, because it allows the solver to 
		// perform hostarting stuff and keeping track of structure changes as well.
		_graph.notifyGraphModified(false);
    }
    
}

    
    
/**
 * Remove an existing discrete state control input pair from the
 * object properties TebController::_state_seq and TebController::_ctrl_seq at position \c sample_idx.
 * Existing elements between \c sample_idx+1 and \c n (getN()) 
 * are shifted to the interval \c idx and  \c n-1.
 *
 * If \c predict_control is \c true A new control input u at position \c sample_idx is
 * predicted using the class method predictControl().
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 * 
 * @param sample_idx index in TebController::_state_seq and TebController::_ctrl_seq to be deleted
 * @param predict_control if \c true, the control for the new interval is calculated using the method predictControl() [default=\c true];
 */
template <int p, int q>
void TebController<p,q>::removeStateControlInputPair(unsigned int sample_idx, bool predict_control)
{
    const unsigned int n = getN();
    
	assert(n >= getM() && "Removing state and control input pairs from the sequence is currently only supported for sequence lengths getN()==getM()");

    if (sample_idx==0)
    {
        _state_seq.pop_front();
		_ctrl_seq.pop_front();
    }
    else if (sample_idx==n-1)
    {
		_state_seq.pop_back();
		_ctrl_seq.pop_back();
    }
    else if (sample_idx<n-1)
    {
        if (predict_control)
        {
            // update u_{k-1}, since it controls the system to the sample that should be deleted
			predictControl(_ctrl_seq.at(sample_idx - 1).controls(), _state_seq.at(sample_idx - 1).states(), _state_seq.at(sample_idx + 1).states());
        }
		_state_seq.erase(_state_seq.begin() + sample_idx);
		_ctrl_seq.erase(_ctrl_seq.begin() + sample_idx);
    }
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();
}


/**
 * Insert a new discrete StateVector ControlInput pair to TebController::_state_seq and TebController::_ctrl_seq at position \c sample_idx.
 * Existing elements between \c idx+1 and \c n (getN())
 * are shifted to the interval \c idx+1 and \c n+1.
 *
 * if \c predict_u is \c true, a new control input u at position \c sample_idx is
 * predicted using the class method predictControl().
 *
 * @warning Be careful, if you override this function in a subclass make sure to call
 *          HyperGraph::notifyGraphModified() inside.
 *
 * @param sample_idx position at which the StateVertex and ControlVertex should be inserted
 * @param state State vector [p x 1] to be added to _state_seq
 * @param control Control input vector [q x 1] to be added _ctrl_seq
 * @param predict_control if \c true, the control is not taken from \c sample, but it is calculated using the method predictControl() [default= \c true];
 */
template <int p, int q>
void TebController<p, q>::insertStateControlInputPair(unsigned int sample_idx, const Eigen::Ref<const StateVector>& state, const Eigen::Ref<const ControlVector>& control, bool predict_control)
{
    const unsigned int n = getN();

	assert(n >= getM() && "Inserting state and control input pairs from the sequence is currently only supported for sequence lengths getN()==getM()");
    
    if (sample_idx==0)
    {
		pushFrontStateControlInputPair(state, control);
        if (predict_control && n>0)
        {
            predictControl(_ctrl_seq.at(0).controls(), _state_seq.at(0).states(), _state_seq.at(1).states() );
        }
    }
    else if (sample_idx>=n)
    {
		pushBackStateControlInputPair(state, control);
        if (predict_control && n>0)
        {
			predictControl(_ctrl_seq.at(n - 1).controls(), _state_seq.at(n - 1).states(), _state_seq.back().states()); // we did not increase n after calling pop_back, therefore "n=last_index" now
        }
    }
    else
    {
		_state_seq.insert(_state_seq.begin() + sample_idx, state);
		_ctrl_seq.insert(_ctrl_seq.begin() + sample_idx, control);
        if (predict_control)
        {
			predictControl(_ctrl_seq.at(sample_idx - 1).controls(), _state_seq.at(sample_idx - 1).states(), _state_seq.at(sample_idx).states());
			predictControl(_ctrl_seq.at(sample_idx).controls(), _state_seq.at(sample_idx).states(), _state_seq.at(sample_idx + 1).states());
            
        }
    }
    
    // notify graph that vertices are changed
    _graph.notifyGraphModified();
}

/**
 * This method can be used to predict the plant input \f$ \mathbf{u}_i \f$
 * that transite drive the system from \f$ \mathbf{x}_i \f$ to \f$ \mathbf{x}_{i+1} \f$.
 * The method can be used e.g. in the trajectory initialization phase, or inseration/deletion of
 * new discrete samples.
 *
 * The current implementation just copies the previous
 * prediction (which is set to 0 inside the method initTrajectory()).
 * Problem specific subclasses can override this function and
 * can use the inverse of the discrete system dynamics
 * to derive a function for \f$ \mathbf{u}_i \f$ with respect \f$ \mathbf{x}_i \f$ and \f$ \mathbf{x}_{i+1} \f$.
 * 
 * E.g. for linear scalar systems: 
 * \f{eqnarray}
 * 	\dot{x} = A x + b u \\
 *  	x_{k+1} = x_{k} + \Delta T A x_k + \Delta T b u_k \\
 * 	u_k = \frac{ x_{k+1} - x_k - \Delta T A }{ \Delta T b}
 * \f}
 * 
 * @param ctrl_out [output] predicted control input vector \f$ \mathbf{u}_i \f$ [\a q x 1]
 * @param x1 vector state \f$ \mathbf{x}_i \f$ at discrete time \a i [\a p x 1]
 * @param x2 vector state \f$ \mathbf{x}_{i+1} \f$ at discrete time \a i+1 [\a p x 1]
 */
template <int p, int q>
inline void TebController<p,q>::predictControl(Eigen::Ref<ControlVector> ctrl_out, const Eigen::Ref<const StateVector>& x1, const Eigen::Ref<const StateVector>& x2)
{
    
}


/**
 * Convert TEB to a full (p+q by n) matrix.
 * The new matrix is composed as follows:
 * - size: [p+q x n] (double), n=getN()
 * - rows 1:p        -> states
 * - rows p+1:p+q    -> control inputs
 * - columns 1:n     -> discrete samples
 *
 * @returns new copy of the TEB matrix [p+q x n]
 */
template <int p, int q>
Eigen::MatrixXd TebController<p,q>::getStateCtrlInfoMat() const
{
    Eigen::MatrixXd teb_mat(p+q,getN());

	assert(getN() >= getM() && "getStateCtrlInfoMat() currently supports only for state and control input sequences with lengths getN()==getM()");
    
	for (unsigned int i = 0; i < getN(); ++i)
	{
		teb_mat.col(i).head(p) = _state_seq.at(i).states();
		teb_mat.col(i).tail(q) = _ctrl_seq.at(i).controls();
	}
    
    return teb_mat;
}

    
    
/**
 * Show fixed states of the TEB as a (p+q by n) matrix
 * for debugging purposes.
 * The matrix is composed as follows:
 * - size: [p+q x n] (double)
 * - rows 1:p        -> states
 * - rows p+1:p+q    -> control inputs
 * - columns 1:n     -> samples
 *    
 * @returns a copy of the fixed state TEB info matrix [p+q x n]
*/
template <int p, int q>
Eigen::Matrix<bool,p+q,-1> TebController<p,q>::getTEBFixedMap() const
{
    Eigen::Matrix<bool,-1,-1> fixed_mat(p+q,getN());
    
	assert(getN() == getM() && "getTEBFixedMap() currently supports only for state and control input sequences with lengths getN()==getM()");


	for (unsigned int i = 0; i < getN(); ++i)
	{
		fixed_mat.col(i).head(p) = _state_seq.at(i).fixed_states();
		fixed_mat.col(i).tail(q) = _ctrl_seq.at(i).fixed_ctrls();
	}

    return fixed_mat;
}
    
/**
 * Get info matrix that contains the Hessian index (first column) and the number of
 * free variables (second column) for all states and control inputs (row-wise).
 * This function is used for debugging purposes, e.g. to test getActiveVertices().
 *
 * @remarks The Hessian index correspond to the row/col index of the first component
 * 	    of the current VertexType (StateVertex and ControlVertex). With the number of free variables, the position of
 * 	    the current VertexType inside the "big" composed Hessian is determined completely.
 *
 * @returns a copy of the hessian-idx-info-matrix [n x 2]
*/   
template <int p, int q>
Eigen::Matrix<double,-1,2> TebController<p,q>::getSampleOptVecIdxVMat() const
{

	assert(getN() == getM() && "getSampleOptVecIdxVMat() currently supports only for state and control input sequences with lengths getN()==getM()");

	Eigen::Matrix<double,-1,2> idx_mat(getN()+getM()+1,2); // n states + m control inputs + 1 dt

	unsigned int idx = 0;
	for (unsigned int i = 0; i < getN(); ++i)
	{
		idx_mat.coeffRef(idx, 0) = _state_seq.at(i).getOptVecIdx();
		idx_mat.coeffRef(idx, 1) = _state_seq.at(i).dimensionFree();
		++idx;
		idx_mat.coeffRef(idx, 0) = _ctrl_seq.at(i).getOptVecIdx();
		idx_mat.coeffRef(idx, 1) = _ctrl_seq.at(i).dimensionFree();
		++idx;		
	}
    idx_mat.coeffRef(idx, 0) = _dt.getOptVecIdx();
    idx_mat.coeffRef(idx, 1) = _dt.dimensionFree();
    return idx_mat;
}
   
   
/**
 * Construct the hyper-graph for the underlying optimization problem.
 * Instantiate edges as subclasses of BaseEdge and add them to the corresponding
 * containers.
 * - Add objective/cost function edges to TebController::_graph via HyperGraph::addEdgeObjective().
 * - equality constraints to TebController::_graph via HyperGraph::addEdgeEquality().
 * - and inequality constraints to TebController::_graph via HyperGraph::addEdgeInequality().
 *
 * Example for the objective EdgeMinimizeTime that is derived from BaseEdge:
 * \code
 * 	EdgeMinimizeTime* to_edge = new EdgeMinimizeTime; // Instantiate new edge object.
 *      to_edge->setVertex(&_dt); // Connect edge with vertex TimeDiff.
 *      to_edge->setData(getN()-1); // Set weight/data (only for EdgeMinimizeTime).
 *      _graph.addEdgeObjective(to_edge); // Store pointer to the container.
 * \endcode
 * 
 * The implementation contains a few default edges commonly used in MPC and Optimal Control scenarios.
 * See:
 * - activateObjectiveTimeOptimal()
 * - activateObjectiveQuadraticForm()
 * - activateControlBounds()
 * - activateStateBounds()
 * - setSystemDynamics()
 * 
 * After checking and adding activated default edges, the function calls customOptimizationGraph() for extended user supplied edges.
 * 
 * This method is called within initOptimization().
 * 
 * @sa customOptimizationGraph, customOptimizationGraphHotStart, HyperGraph, BaseEdge
 */
template <int p, int q>
void TebController<p,q>::buildOptimizationGraph()
{
    setFixedStart(true);
    setFixedGoal(_fixed_goal_states);

    
    //! @todo Here we need to fix the last control input since it undefined (no control is needed to go to state n+1). Maybe we can change the code somewhere else, that the contorl sequence is always getN()-1.
    _ctrl_seq.back().setFixedAll(true);
    
    // add predefined edges if desired
    if (_active_time_optimal)
    {
        EdgeMinimizeTime* to_edge = new EdgeMinimizeTime(_dt);
        to_edge->setData(getN()-1);
        _graph.addEdgeObjective(to_edge);
        
        EdgePositiveTime* pt_edge = new EdgePositiveTime(_dt);
        pt_edge->setBounds(cfg->teb.dt_min);
        _graph.addEdgeInequality(pt_edge);
    }
    
    
    // add edges that will be added to all (n-1) samples
    bool activate_ctrl_bounds = std::get<0>(_active_control_bounds);
    bool activate_state_bounds = std::get<0>(_active_state_bounds);
    bool activate_quadratic_form = std::get<0>(_active_quadratic_form);

    double Q = std::get<1>(_active_quadratic_form);
    double R = std::get<2>(_active_quadratic_form);
    
    if (activate_quadratic_form || activate_ctrl_bounds || activate_state_bounds || _active_system_dynamics.first)
    {
      for (unsigned int i=0; i<getN()-1; ++i)
      {
          if (activate_quadratic_form && (Q!=0 || R!=0))
          {
              // add quadratic form objective ( for the last sample see below this loop )
              EdgeQuadraticForm<p,q>* q_form_edge = new EdgeQuadraticForm<p,q>(_state_seq.at(i),_ctrl_seq.at(i));
              q_form_edge->setWeights(std::get<1>(_active_quadratic_form),std::get<2>(_active_quadratic_form));
              q_form_edge->setReference(_goal_backup);
              _graph.addEdgeObjective(q_form_edge);
          }
          
          if (activate_ctrl_bounds)
          {
            // add bounds on control inputs
            EdgeControlBounds<q>* ctrl_bound_edge = new EdgeControlBounds<q>(_ctrl_seq.at(i));
            ctrl_bound_edge->setBounds(std::get<1>(_active_control_bounds), std::get<2>(_active_control_bounds));
            _graph.addEdgeInequality(ctrl_bound_edge);
          }
          
          if (activate_state_bounds && i>0)
          {
               // add bounds on states
            EdgeStateBounds<p>* state_bound_edge = new EdgeStateBounds<p>(_state_seq.at(i));
            state_bound_edge->setBounds(std::get<1>(_active_state_bounds), std::get<2>(_active_state_bounds));
            _graph.addEdgeInequality(state_bound_edge);
          }
              
          if (_active_system_dynamics.first)
          {	    
            // add system equations:
            if (cfg->teb.diff_method == FiniteDifferences::FORWARD || i==0)
            {
                EdgeSystemDynamics<p,q>* system_edge = new EdgeSystemDynamics<p,q>(_active_system_dynamics.second, _state_seq.at(i), _ctrl_seq.at(i), _state_seq.at(i+1), _dt);
                _graph.addEdgeEquality(system_edge);
            }
            else
            {
				EdgeSystemDynamics<p, q, true>* system_edge = new EdgeSystemDynamics<p, q, true>(_active_system_dynamics.second, _state_seq.at(i-1), _state_seq.at(i),_ctrl_seq.at(i), _state_seq.at(i+1), _dt);
                _graph.addEdgeEquality(system_edge);
            }
          }
          
          // add control input derivatives
          if (_active_control_input_derivatives)
          {
              if (i==0)
              {
                  EdgeInputDerivatives<q,q,true>* ctrl_dev_edge = new EdgeInputDerivatives<q,q,true>(_ctrl_seq.at(i));
                  ctrl_dev_edge->setInitialReference(_prev_control_input.first.head(q-1), _prev_control_input.second);
                  _graph.addEdgeEquality(ctrl_dev_edge);
              }
              else //if (i < getN()-2) // todo: change dependency to getM(), if ctrl_seq is defined as getN()-1 in the future
              {
                  EdgeInputDerivatives<q,q>* ctrl_dev_edge = new EdgeInputDerivatives<q,q>(_ctrl_seq.at(i-1), _ctrl_seq.at(i), _dt);
                  _graph.addEdgeEquality(ctrl_dev_edge);
              }
          }
      }
    }
    
	// Add costs for the last state if not fixed
	if (!_state_seq.back().isFixedAll())
	{
		// add state bounds
		if (activate_state_bounds)
		{
			// add bounds on states
			EdgeStateBounds<p>* state_bound_edge = new EdgeStateBounds<p>(_state_seq.back());
			state_bound_edge->setBounds(std::get<1>(_active_state_bounds), std::get<2>(_active_state_bounds));
			_graph.addEdgeInequality(state_bound_edge);
		}

		// add quadratic form objective for the final state constraint if unfixed and activated
		Q = std::get<3>(_active_quadratic_form);
		if (activate_quadratic_form && Q != 0)
		{
			EdgeQuadraticForm<p, 0>* q_form_edge = new EdgeQuadraticForm<p, 0>(_state_seq.back());
			q_form_edge->setWeights(Q); // last control input is not part of the optimization
			q_form_edge->setReference(_goal_backup);
			_graph.addEdgeObjective(q_form_edge);
		}
	}
    
    // call function that can be derived to customize the graph
    customOptimizationGraph();  
}
    
/**
 * Do everything what is necessary before calling the actual optimization optimizeTEB().
 * 
 * This wrapper function calls buildOptimizationGraph() to construct the hyper-graph.
 * Afterwards it collects all active vertices using getActiveVertices().
 * 
 * In addition a few undesired config settings are checked (only if compiled in Debug mode).
 */    
template<int p, int q>
void TebController<p,q>::initOptimization()
{
    
    PRINT_DEBUG_COND_ONCE(cfg->teb.dt_min > (_dt_ref==-1 ? cfg->teb.dt_ref : _dt_ref) - cfg->teb.dt_hyst, "Maybe unexpected results: cfg->teb.dt_min > cfg->teb.dt_ref - cfg->teb.dt_hyst");
    PRINT_DEBUG_COND_ONCE(cfg->teb.diff_method==FiniteDifferences::CENTRAL, "Central differences are not working as supposed to do now. Please test and fix. Otherwise it will be removed from code.");

    if (_graph.isGraphModified() || cfg->optim.force_rebuild_optim_graph)
    {
        _graph.clearGraph();
        buildOptimizationGraph();
        getActiveVertices();
    }
    else
    {
        customOptimizationGraphHotStart();
    }
    
    
}

    
/**
 * Iterate through the TEB sequence/trajectory and determine if a vertex (StateVertex, ControlVertex or TimeDiff) is completely fixed.
 * In that case the corresponding vertex is not active and can be skipped during optimization (Jacobian calculation, Hessian, ...).
 * All active vertices (pointers) are stored into TebController::_active_vertices.
 * 
 * While iterating all vertices, the hessian index is determined and stored inside the corresponding vertex
 * (see TebVertex::setOptVecIdx() and TebVertex::getOptVecIdx()).
 * For example:
 *   - The final state (goal state) is fixed for many problems, in that case it does not appear in the optimization vector and therefore it is not part of the Hessian.
 *     But if at least a single component is unfixed, the complete vertex (TebSample in this case) becomes active. 
 *     It's hessian or optimization vector index is zero. The number of free/unfixed variables is \a q.
 *   - The second sample has no fixed components. It's hessian or optimization vector index is \a q since the last vertex starts at zero and has
 *     \a q free variables.
 *   - The following vertex starts at \a 2 * \a q + \a p and so on ...	
 *   - The TimeDiff \f$ \Delta T \f$ is stored at the end (and therefore it has the largest index if unfixed).
 *
 * @todo Add case for different state and control input sequence lengths (e.g. for move blocking)
 */      
template<int p, int q>
void TebController<p,q>::getActiveVertices()
{
	assert(getN() == getM() && "getActiveVertices() - currently supports  only sequence lengths getN()==getM() - TODO");

    _graph.clearActiveVertices();
    _graph.activeVertices().reserve(getN()+getM()+1); // reserve memory for N states, M control inputs + Time (their pointers)
    
    // iterate samples
    int idx = 0;
    for (unsigned int i = 0; i < getN(); ++i)
    {
		// first process state
        if (_state_seq.at(i).dimensionFree() > 0) // keep default idx otherwise: "-1" which means: not relevant
        {
			_state_seq.at(i).setOptVecIdx(idx);
			_graph.addActiveVertex(&_state_seq.at(i));
        }
        else
        {
			_state_seq.at(i).setOptVecIdx(-1);
        }
		idx += _state_seq.at(i).dimensionFree();

		// and now control input
		if (_ctrl_seq.at(i).dimensionFree() > 0) // keep default idx otherwise: "-1" which means: not relevant
		{
			_ctrl_seq.at(i).setOptVecIdx(idx);
			_graph.addActiveVertex(&_ctrl_seq.at(i));
		}
		else
		{
			_ctrl_seq.at(i).setOptVecIdx(-1);
		}
		idx += _ctrl_seq.at(i).dimensionFree();
    }
    
    // timediff
    if (_dt.dimensionFree() > 0)
    {
        _dt.setOptVecIdx(idx);
        _graph.addActiveVertex(&_dt);
    }
}


/**
 * Calculate vector of absolute times \f$ t=[0,\Delta T, 2 \Delta T, \dotsc, n \Delta T] \f$.
 * 
 * @return Row vector containing time information [\c n x 1].
 */
template<int p, int q>
Eigen::VectorXd TebController<p,q>::getAbsoluteTimeVec() const
{
    Eigen::VectorXd time(getN());
    time[0] = 0.;
    for (unsigned int i=1; i<getN(); ++i)
    {
        time[i] = i * _dt.dt();
    }
    return time;
}

    
} // end namespace teb