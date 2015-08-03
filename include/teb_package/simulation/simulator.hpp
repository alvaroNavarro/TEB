#include <teb_package/simulation/simulator.h>


namespace teb
{
  
  
/**
 * This method performs an open-loop simulation of the system controlled by 
 * the TebSimulator, both specified in the constructor of this class.
 * 
 * The TEB optimization is solved once to determine the optimal control for the
 * point to point transition from the start state vector \f$ \mathbf{x}_0 \f$ to 
 * the final state vector \f$ \mathbf{x}_f \f$.
 * 
 * The determined control is applied to the simulated plant.
 * In the abscence of model mismatch and disturbances the output should be similar
 * to the planned one by the TEB, if the system dynamics equations are satisfied.
 * 
 * @param x0 Double array with \a p values representing the start state vector
 * @param xf Double array with \a p values representing the final state vector
 * @param plot if \c true, the results are plotted into a figure using TebPlotter.
 * @param add_step_response Pass double[q] as control step to additionally simulate step response and add it to the figure
 * @param ctrl Pointer to an external controller: set to nullptr in order to use the class member _controller.
 * @return SimResults containing the time series of the open-loop control 
 *         and the states and controls planned by the controller.
 */   
template <int p, int q>
std::unique_ptr<SimResults> Simulator<p, q>::simOpenLoop(const double* const x0, const double* const xf, bool plot, const double* const add_step_response, BaseController* ctrl)
{
  if (!ctrl) ctrl = _controller;
  assert(ctrl && _system);
  
  // Call user-defined function if desired
  if (_callback_sim_pre) _callback_sim_pre(ctrl, _system, this);
  
  // clear current teb
  ctrl->resetController();
  
  // Call user-defined function if desired
  if (_callback_step_pre) _callback_step_pre(ctrl, _system, this);
  
  // Perform open-loop optimization
  ctrl->step(x0, xf);
      
  // Call user-defined function if desired
  if (_callback_step_post) _callback_step_post(ctrl, _system, this);
  
  // collect first simulation results
  std::unique_ptr<SimResults> results(new SimResults);
  auto temp_x = ctrl->getStateCtrlInfoMat(); // inefficient, but we are in simulation only
  auto temp_u = ctrl->returnControlInputSequence(); // inefficient, but we are in sim only
  temp_u.conservativeResize(temp_u.rows(),temp_u.cols()+1); // adjust size to fit to the time vec and add a zero control at time "n"
  temp_u.rightCols(1).setZero();
  results->series.emplace_back(); // add series for initial teb
  results->series.front().states = temp_x.topRows(p);
  results->series.front().controls = temp_u.bottomRows(q);
  results->series.front().time = ctrl->getAbsoluteTimeVec();
  results->series.front().dt = ctrl->getDt();
  
  // Add series for post open-loop control
  results->series.emplace_back();
  // Allocate space for the post simulation results;
  results->allocateMemory(1,p,q,ctrl->getN());
  
  // Set x0
  results->series.at(1).states.col(0) = ctrl->firstState();
  results->series.at(1).dt = ctrl->getDt();

  // Simulate system with the determined control (use TEB timediff, since it is an open-loop control)
  Eigen::MatrixXd control_sequence = ctrl->returnControlInputSequence();
  for (unsigned int idx=1; idx < ctrl->getN(); ++idx)
  {
    ControlVector u =  control_sequence.col(idx-1);
    // saturate control
    saturateControl(u);
    
    results->series.at(1).states.col(idx) = systemStep(results->series.at(1).states.col(idx-1), u, ctrl->getDt() );
    results->series.at(1).controls.col(idx-1) = u;
    results->series.at(1).time(idx) = results->series.at(1).time(idx-1) + ctrl->getDt();
  }    
  

  // Simulate step response if desired
  if (add_step_response)
  {
	  results->merge( simStepResponse(add_step_response, results->series.at(1).time.tail(1)[0], x0, false) );
  }

  // Plot stuff
  if (_plotter && plot)
  {
    teb::PlotOptions opt;
    opt.title = "TEB Open-loop Simulation";
    opt.legend = true;
    opt.legend_entries.emplace_back("TEB optimization only");
    opt.legend_entries.emplace_back("TEB open-loop control");
	if (add_step_response) opt.legend_entries.emplace_back("Plant step response");
    plotResults(results->series,&opt);
  }
  
  return results;  
}


/**
 * This method performs a closed-loop simulation of the system controlled by 
 * the TebSimulator, both specified in the constructor of this class.
 * 
 * The TEB optimization is solved in each sampling-interval specified by setSampleTime()
 * to determine the next optimal control for the
 * point to point transition from the most recent measurement of the state vector \f$ \mathbf{x}_k \f$ to 
 * the final state vector \f$ \mathbf{x}_f \f$.
 * As a starting point, the first state vector is given by \f$ \mathbf{x}_0 \f$.
 * 
 * @bug If we set a title or legend in the plot options for stepping (!), than the plot won't be rendered correctly.
 *
 * @param x0 Double array with \a p values representing the start state vector
 * @param xf Double array with \a p values representing the final state vector
 * @param sim_time Total simulation time (duration of the simulation)
 * @param manual_stepping if \c true, the simulation is paused after each sampling interval.
 * 	 		  During pause, the user can type in the desired number of steps until the next pause will be instantiated.
 * 			  If the user chooses "-1" steps, the simulation will be proceeded until sim_time is exceeded.
 * @param plot_result if \c true, the results at the end of the simulation are plotted into a figure using TebPlotter.
 * @param plot_steps if \c true, the results after each sample_inteveral are plotted into a figure using TebPlotter (shows the progress of the control).
 * @param add_step_response Pass double[q] as control step to additionally simulate step response and add it to the figure
 * @param ctrl Pointer to an external controller: set to nullptr in order to use the class member _controller.
 * @return SimResults containing the time series of the closed-loop control.
 */  
template <int p, int q>
std::unique_ptr<SimResults> Simulator<p, q>::simClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping, bool plot_result, bool plot_steps, const double* const add_step_response, BaseController* ctrl)
{
  if (!ctrl) ctrl = _controller;
  assert(ctrl && _system);
    
  if (plot_steps && _plotter && _plotter->isExportToFileEnabled())
  {
      PRINT_INFO_ONCE("simClosedLoop(): Plotting each step of the closed loop control is currently not supported in file export mode. Only the final results after the simulation is completed will be stored.");
      plot_steps = false;
  }
  
  // Call user-defined function if desired
  if (_callback_sim_pre) _callback_sim_pre(ctrl, _system, this);
  
  // plot options
  teb::PlotOptions opt;
  teb::PlotOptions opt_steps;
  if (_plotter && (plot_result || plot_steps))
  {
      opt.title = "TEB Closed-loop Simulation";
      opt.legend = true;
      opt.legend_entries.emplace_back("TEB closed-loop control");
      opt.skip_last_value_right_column = true; // skip invalid control at time n
      
      opt_steps.title = ""; // bug, if we set a title or legend, the image is not updated correctly in the subsequent step
      opt_steps.legend = false;
      opt_steps.skip_last_value_right_column = true; // skip invalid control at time n
  }
  
  // clear current teb
  ctrl->resetController();
  
  // allocate sim result container
  std::unique_ptr<SimResults> results(new SimResults);
  results->series.emplace_back(); // add time_series for post closed-loop control data
  results->conservativeResize(0,p,q,1);
  
  // Set x0
  Eigen::Map<const StateVector> x0_(x0);
  Eigen::Map<const StateVector> xf_(xf);
  results->series.front().states.col(0) = x0_;
  results->series.front().time(0) = 0.;
  
  double dt = _sample_time;
  double t = 0;
  unsigned int time_idx = 0;
  
  int steps_to_perform = 1;
  
  StateVector x_k = x0_;
  do
  {
      // Call user-defined function if desired
      if (_callback_step_pre) _callback_step_pre(ctrl, _system, this);
    
      // Perform open-loop optimization for the current step
      ctrl->step(x_k.data(), xf_.data());
	  
      // Call user-defined function if desired
      if (_callback_step_post) _callback_step_post(ctrl, _system, this);
      
      if (steps_to_perform>0 && manual_stepping) --steps_to_perform;
      
      // get control value for the next step
      ControlVector u_k = ctrl->firstControl();
      
      // saturate control
      saturateControl(u_k);
            
      // simulate plant
      if (_sample_time == -1) // inherit sample_time from teb
          dt = ctrl->getDt();
      
      x_k = systemStep( x_k,  u_k, dt );
      
      // Store results
      results->conservativeResize(0,p,q,time_idx+2); // +2 instead of +1 since it stores x0 already
      results->series.front().controls.col(time_idx) = u_k;
      results->series.front().controls.col(time_idx+1) = ControlVector::Zero(); // redundant and not necessary, but only to create a valid value for plotting
      results->series.front().time(time_idx+1) = results->series.front().time(time_idx) + dt;
      results->series.front().states.col(time_idx+1) = x_k;

      if (_plotter && plot_steps)
      {
          plotResults(results->series, &opt_steps);
      }
  
      if (manual_stepping && steps_to_perform==0)
      {
		  PRINT_INFO("Simulation paused. Type in number of steps to perform. Type in '-1' to continue without pausing anymore. Input: ");
		  INPUT_STREAM(steps_to_perform, -1);
      }

      ++time_idx;
      t += dt;
  } while (t < sim_time);
  
  
  // fill last invalid control
  results->series.front().controls.col(time_idx) = ControlVector::Zero();
  

  // Simulate step response if desired
  if (add_step_response)
  {
	  results->merge(simStepResponse(add_step_response, t, x0, false));
	  opt.legend_entries.emplace_back("Plant step response");
  }

  if (_plotter && plot_result)
  {
    plotResults(results->series, &opt);
  }
  
  return results;  
}



/**
 * This method performs both, an open-loop simulation of the system controlled by 
 * the TebSimulator and the closed-loop simulation of the system.
 * 
 * Refer to simOpenLoop() and simClosedLoop() for details.
 * 
 * The results are combined and shown in a single figure using TebPlotter.
 * 
 * @param x0 Double array with \a p values representing the start state vector
 * @param xf Double array with \a p values representing the final state vector
 * @param sim_time Total simulation time for the closed-loop simulation (duration of the simulation)
 * @param manual_stepping if \c true, the closed-loop simulation is paused after each sampling interval.
 * 	 		  During pause, the user can type in the desired number of steps until the next pause will be instantiated.
 * 			  If the user chooses "-1" steps, the simulation will be proceeded until sim_time is exceeded.
 * @param plot_steps if \c true, the results after each sample_inteveral of the closed-loop control are plotted into a figure using TebPlotter (shows the progress of the control).
 * @param add_step_response Pass double[q] as control step to additionally simulate step response and add it to the figure
 * @return SimResults containing the time series of the open-loop control, the closed-loop control
 *         and the states and controls planned by the controller.
 */
template <int p, int q>
std::unique_ptr<SimResults> Simulator<p, q>::simOpenAndClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping, bool plot_steps, const double* const add_step_response)
{
  // closed-loop sim
  std::unique_ptr<SimResults> cl_results = simClosedLoop(x0, xf, sim_time, manual_stepping, false, plot_steps, add_step_response); 
  // open-loop sim
  std::unique_ptr<SimResults> results = simOpenLoop(x0, xf, false);
  // copy cl time-series to ol_results in order to plot everything into one plot.
  // efficiency doesn't matter since simulation is not required to be performed in real-time
  results->merge(std::move(cl_results));
  
  if (_plotter)
  {
    // plot options
    teb::PlotOptions opt;
    opt.title = "TEB Open-loop and Closed-loop Simulation";
    opt.legend = true;
    opt.legend_entries.emplace_back("TEB optimization only");
    opt.legend_entries.emplace_back("TEB open-loop control");
    opt.legend_entries.emplace_back("TEB closed-loop control");
	if (add_step_response) opt.legend_entries.emplace_back("Plant step response");
    opt.skip_last_value_right_column = true; // skip invalid control at time n
    plotResults(results->series, &opt);
  }
  
  return results;
}
   
/**
 * This method performs an open-loop simulation of the system controlled by different
 * Controllers passed to this method. The system is specified using the constructor of this class.
 *
 * The TEB optimization is solved once to determine the optimal control for the
 * point to point transition from the start state vector \f$ \mathbf{x}_0 \f$ to
 * the final state vector \f$ \mathbf{x}_f \f$.
 *
 * @param controllers array of different TebControllers with augmented name as string
 * @param x0 Double array with \a p values representing the start state vector
 * @param xf Double array with \a p values representing the final state vector
 * @param plot if \c true, the results are plotted into a figure using TebPlotter.
 * @param add_step_response Pass double[q] as control step to additionally simulate step response and add it to the figure
 */
template <int p, int q>
void Simulator<p, q>::simOpenLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, bool plot, const double* const add_step_response)
{
    SimResults results;
    PlotOptions options;
    options.title = "Controller Comparison Open-Loop Simulation";
    options.legend = true;
    
    for (std::pair<BaseController*,std::string>& ctrl : controllers)
    {
		std::unique_ptr<SimResults> sim = simOpenLoop(x0, xf, false, nullptr, ctrl.first);
		results.merge(std::move(sim));
		options.legend_entries.emplace_back(ctrl.second);
    }
    
	// Simulate step response if desired
	// Call it here manually instead of within simOpenLoop, otherwise we would simulate it multiple times
	if (add_step_response)
	{
		results.merge(simStepResponse(add_step_response, results.series.front().time.tail(0)[0], x0, false));
		options.legend_entries.emplace_back("Plant step response");
	}

    if (plot && _plotter)
    {
        options.skip_last_value_right_column = true; // skip last invalid control at time n
        plotResults(results.series,&options);
    }
}
    
/**
 * This method performs an closed-loop simulation of the system controlled by different
 * Controllers passed to this method. The system is specified using the constructor of this class.
 *
 * The TEB optimization is solved in each sampling-interval specified by setSampleTime()
 * to determine the next optimal control for the
 * point to point transition from the most recent measurement of the state vector \f$ \mathbf{x}_k \f$ to
 * the final state vector \f$ \mathbf{x}_f \f$.
 * As a starting point, the first state vector is given by \f$ \mathbf{x}_0 \f$.
 *
 * @param controllers array of different TebControllers with augmented name as string
 * @param x0 Double array with \a p values representing the start state vector
 * @param xf Double array with \a p values representing the final state vector
 * @param sim_time Total simulation time (duration of the simulation)
 * @param plot_result if \c true, the results at the end of the simulation are plotted into a figure using TebPlotter.
 * @param plot_steps if \c true, the results after each sample_inteveral are plotted into a figure using TebPlotter (shows the progress of the control).
 * @param add_step_response Pass double[q] as control step to additionally simulate step response and add it to the figure
 */
template <int p, int q>
void Simulator<p, q>::simClosedLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, double sim_time, bool plot_result, bool plot_steps, const double* const add_step_response)
{
    SimResults results;
    PlotOptions options;
    options.title = "Controller Comparison Closed-Loop Simulation";
    options.legend = true;
    
    for (std::pair<BaseController*,std::string>& ctrl : controllers)
    {
        std::unique_ptr<SimResults> sim = simClosedLoop(x0, xf, sim_time, false, false, plot_steps, nullptr, ctrl.first);
		results.merge(std::move(sim));
        options.legend_entries.emplace_back(ctrl.second);
    }
    
	// Simulate step response if desired
	// Call it here manually instead of within simOpenLoop, otherwise we would simulate it multiple times
	if (add_step_response)
	{
		results.merge(simStepResponse(add_step_response, sim_time, x0, false));
		options.legend_entries.emplace_back("Plant step response");
	}

    if (plot_result && _plotter)
    {
        options.skip_last_value_right_column = true; // skip last invalid control at time n
        plotResults(results.series,&options);
    }
}


    
/**
 * This method performs a simulation of the system stimulated by a step of the control from zero to \f$ \mathbf{u} \f$.
 *
 * No optimization is conducted.
 *
 * @param u_ref Final step value of the control
 * @param sim_time Total simulation time (duration of the simulation)
 * @param x0 Double array with \a p values representing the start state vector, if nullptr: x0 is set to zero
 * @param plot if \c true, the results after each sample_inteveral are plotted into a figure using TebPlotter (shows the progress of the control).
 * @return SimResults containing the time series of the closed-loop control.
 */
template <int p, int q>
std::unique_ptr<SimResults> Simulator<p,q>::simStepResponse(const double* const u_ref, double sim_time, const double* const x0, bool plot)
{
    assert(_system);
    
    
    // allocate sim result container
    std::unique_ptr<SimResults> results(new SimResults);
    results->series.emplace_back(); // add time_series for post closed-loop control data
    results->conservativeResize(0,p,q,1);
    
    // Set x0
    Eigen::Map<const StateVector> x0_(x0);
    StateVector x_k;
    if (x0) x_k = x0_;
    else x_k = StateVector::Zero();
    
    results->series.front().states.col(0) = x_k;
    results->series.front().time(0) = 0.;
    
    Eigen::Map<const ControlVector> u(u_ref);
    
    double dt = _sample_time;
    double t = 0;
    unsigned int time_idx = 0;
    
    
    do
    {
        
        // simulate plant
        if (_sample_time == -1) // inherit sample_time from teb
        {
            PRINT_ERROR("Cannot simulate step response without a valid sample time. Please set a sample time using setSampleTime(dt)");
            return std::unique_ptr<SimResults>();
        }
        
        x_k = systemStep( x_k,  u, dt );
        
        // Store results
        results->conservativeResize(0,p,q,time_idx+2); // +2 instead of +1 since it stores x0 already
        results->series.front().controls.col(time_idx) = u;
        results->series.front().controls.col(time_idx+1) = ControlVector::Zero(); // redundant and not necessary, but only to create a valid value for plotting
        results->series.front().time(time_idx+1) = results->series.front().time(time_idx) + dt;
        results->series.front().states.col(time_idx+1) = x_k;
        
        ++time_idx;
        t += dt;
    } while (t < sim_time);
    
    
    // fill last invalid control
    results->series.front().controls.col(time_idx) = ControlVector::Zero();
    
    if (_plotter && plot)
    {
        // plot options
        teb::PlotOptions opt;
        teb::PlotOptions opt_steps;
        opt.title = "System Step Response";
        opt.legend = false;
        plotResults(results->series, &opt);
    }
    
    return results;  
}
    
    
    
// see header
template <int p, int q>
void Simulator<p,q>::setIntegrator(NumericalIntegrators int_type)
{
  switch (int_type)
  {
    case NumericalIntegrators::EXPLICIT_EULER:
      _integrator = IntegatorPtr(new ExplicitEuler<p,q>());
      break;
    case NumericalIntegrators::RUNGE_KUTTA_CLASSIC:
      _integrator = IntegatorPtr(new RungeKuttaClassic<p,q>());
      break;
    case NumericalIntegrators::RUNGE_KUTTA_5TH:
      _integrator = IntegatorPtr(new RungeKutta5thOrder<p,q>());
      break;
  }
};


/**
 * @param u The control input that should be bounded. The result will be stored directly to \c u.
 */
template <int p, int q>
void Simulator<p,q>::saturateControl(Eigen::Ref<ControlVector> u)
{
  u = (u.array() < _control_bounds.first.array()).select(_control_bounds.first,u);
  u = (u.array() > _control_bounds.second.array()).select(_control_bounds.second,u);
}


/**
 * This method prepares a container of multiple time_series object in order to allow the utilization
 * of the TebPlotter for visualization.
 * 
 * Customize the plot by optionally passing a PlotOptions object to this method.
 * 
 * @param time_series Container of time series.
 * @param options Custom plot options.
 */ 
template <int p, int q>
void Simulator<p,q>::plotResults(const std::vector<SimResults::TimeSeries>& time_series, teb::PlotOptions* options) const
{
    if (_plotter)
    {
      std::vector<const Eigen::VectorXd*> time;
      std::vector<const Eigen::MatrixXd*> states;
      std::vector<const Eigen::MatrixXd*> controls;

      for (const SimResults::TimeSeries& plot : time_series)
      {
          time.push_back(&plot.time);
          states.push_back(&plot.states);
          controls.push_back(&plot.controls);
      }
      _plotter->plotTwoCol(time, states, time, controls, options);
    }
    else PRINT_INFO("plotResults() - Cannot plot, since no TebPlotter object assigned");
}

  
  
} // end namespace teb
 
    
    
