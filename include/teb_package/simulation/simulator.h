#ifndef __teb_package__simulator__
#define __teb_package__simulator__

#ifndef RTW

#include <teb_package/base/base_controller.h>
#include <teb_package/base/system_dynamics.h>
#include <teb_package/simulation/integrators.h>
#include <teb_package/simulation/derivatives.h>
#include <teb_package/visualization/teb_plotter.h>
#include <functional>


namespace teb
{
	
/**
 * @brief Simulation results are stored and combined in this container class.
 *
 * @ingroup simulation
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */  
class SimResults
{
  template <int pf, int qf>
  friend class Simulator;
  
public:  
  
	SimResults() {} //!< Empty Constructor

	/**
	* @brief  Merge Results with another SimResults object (Copy TimeSeries)
	* @param results_to_add SimResults object from which the TimeSeries should be copied/merged
	* @param idx Merge only TimeSeries with index \c idx. If set to -1, all TimeSeries elements are copied
	*/
	void merge(const SimResults& results_to_add, int idx = -1)
	{
		if (idx==-1) for (const TimeSeries& oth_series : results_to_add.series) series.push_back(oth_series);
		else
		{
			assert((unsigned int)idx < results_to_add.series.size());
			series.push_back(results_to_add.series.at(idx));
		}
	}

	/**
	* @brief  Merge Results with another SimResults object (Copy TimeSeries)
	* @param results_to_add SimResults object from which the TimeSeries should be copied/merged
	* @param idx Merge only TimeSeries with index \c idx. If set to -1, all TimeSeries elements are copied
	*/
	void merge(std::unique_ptr<SimResults> results_to_add, int idx = -1) { merge(*results_to_add, idx); }

  /**
   * @brief Store measurements of dynamic systems (states and control inputs) w.r.t. time
   * The time is discretized into \a n discrete time samples
   */
  struct TimeSeries
  {
    Eigen::MatrixXd states; //!< Contains state vectors [p x n]
    Eigen::MatrixXd controls; //!< Contains control input vectors [q x n]
    Eigen::VectorXd time; //!< Time vector [0 .. T]  [n x 1]
    double dt; //!< Store time difference \f$ \Delta T \f$ of the current TEB.
  };
  
  std::vector<TimeSeries> series; //!< Container for multiple time series (e.g. to perform store different experiments or closed- and open-loop sim results).
  
protected:
  
  /**
   * @brief Resize time series, but preserve data with indices below \c n
   * @param data TimeSeries object which should be resized.
   * @param p Size of the state vector.
   * @param q Size of the control input vector.
   * @param n New number of discrete samples.
   */
  static void conservativeResize(TimeSeries& data, int p, int q, int n)
  {
	 data.states.conservativeResize(p, n);
	 data.controls.conservativeResize(q, n);
	 data.time.conservativeResize(n);
  }
  
  /**
   * @brief Resize time series, but preserve data with indices below \c n
   * @param id Id/index of the time series object stored in SimResults::series
   * @param p Size of the state vector.
   * @param q Size of the control input vector.
   * @param n New number of discrete samples.
   */
  void conservativeResize(unsigned int id, int p, int q, int n)
  {
    assert(id<series.size());
    conservativeResize(series.at(id),p,q,n);
  }
  
  /**
   * @brief Allocate memory for all matrices and vectors of the underlying TimeSeries object
   * 
   * This method can be used to resize the underlying matrices without preserving data.
   * 
   * @sa conservativeResize()
   * 
   * @param data TimeSeries object for which the memory should be allocated.
   * @param p Size of the state vector.
   * @param q Size of the control input vector.
   * @param n Number of discrete samples.
   * @param set_zero if \c true, all matrices and vectors are initialized to zero.
   */
  static void allocateMemory(TimeSeries& data, int p, int q, int n, bool set_zero=true)
  {
    if (set_zero)
    {
      data.states.setZero(p, n);
      data.controls.setZero(q, n);
      data.time.setZero(n);
    }
    else
    {
      data.states.resize(p,n);
      data.controls.resize(q,n);
      data.time.resize(n);
    }
  }
  
  /**
   * @brief Allocate memory for all matrices and vectors of the underlying TimeSeries object
   * 
   * This method can be used to resize the underlying matrices without preserving data.
   * 
   * @sa conservativeResize()
   * 
   * @param id Id/index of the time series object stored in SimResults::series
   * @param p Size of the state vector.
   * @param q Size of the control input vector.
   * @param n Number of discrete samples.
   * @param set_zero if \c true, all matrices and vectors are initialized to zero.
   */
  void allocateMemory(unsigned int id, int p, int q, int n, bool set_zero=true)
  {
    assert(id<series.size());
    allocateMemory(series.at(id),p,q,n,set_zero);
  }
};
  

/**
 * @brief Simulator for dynamic systems controlled by a TebController.
 * 
 * This simulator supports open-loop and closed-loop simulations for systems
 * specified using the SystemDynamics object.
 * 
 * Results can be visualized using the TebPlotter class.
 *
 * @ingroup simulation
 *
 * @todo Add disturbance models.
 * @todo Add reference trajectory or multiple reference steps for closed-loop control.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */  
template<int p, int q>
class Simulator
{
  template <int pf, int qf>
  friend class SystemDynamics;
  
  friend class SimResults; 
  
public:
  
  //! Typedef for state vector with p states [p x 1].
  using StateVector = Eigen::Matrix<double,p,1>;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = Eigen::Matrix<double, q, 1>;
  //! Typedef for a smart pointer that points to the prefered numerical integration method object.
  using IntegatorPtr = std::unique_ptr<NumericalIntegrator<p,q>>;
  
  
  /**
   * @brief Construct the simulator by accepting a TebController object, a SystemDynamics object and the TebPlotter object.
   * 
   * @param controller Reference to the Controller
   * @param system Reference to the underlying dynamic system.
   * @param plotter Pointer to a TebPlotter object that enables visualization.
   */
  Simulator(BaseController* controller, SystemDynamics<p,q>& system, TebPlotter* plotter = nullptr) : _controller(controller), _system(&system), _plotter(plotter)
  {}
  
  /**
    * @brief Construct the simulator using a SystemDynamics object and the TebPlotter object.
    *
    * Usually you want to use the other constructor and pass a controller object.
    * Use this constructor only, if you want to call simulations, that accept the controller object directly.
    *
    * @param system Reference to the underlying dynamic system.
    * @param plotter Pointer to a TebPlotter object that enables visualization.
    */
  Simulator(SystemDynamics<p,q>& system, TebPlotter* plotter = nullptr) : _system(&system), _plotter(plotter)
    {}
    
  ~Simulator() {} //!< Empty Destructor
  
  /**
   * @brief Set the sample time for the simulation.
   * 
   * The system is simulated for the duration \c sample_time [sec] until a new control is applied.
   * If \c sample_time == -1, the sample time is inherited from the TebController.
   * 
   * @todo Allow different sample_times for controller and plant simulation (the TEB sample time \f$ \Delta T \f$ might be higher ...)
   * 
   * @param sample_time Specified sample time.
   */
  void setSampleTime(double sample_time) {_sample_time = sample_time;}
  
  /**
   * @brief Get the sample time of the simulator.
   * @sa setSampleTime()
   * @return Simulator sample time.
   */
  double sampleTime() {return _sample_time;}

  /**
   * @brief Pass a TebPlotter object to the Simulator class in order to enable visualization.
   * @param plotter Pointer to the plotter object (no memory will be freed).
   */
  void setPlotter(TebPlotter* plotter) {_plotter = plotter;};
  
  /**
   * @brief Set a custom numerical integration method for simulation as subclass of NumericalIntegrator.
   * @param integrator Unique pointer to a NumericalIntegrator subclass.
   */ 
  void setIntegrator(IntegatorPtr integrator) {_integrator = integrator;};
  
  /**
   * @brief Set a predined numerical integration method for simulation (select from teb::NumericalIntegrators)
   * @param int_type Integrator type selected from teb::NumericalIntegrators.
   */ 
  void setIntegrator(NumericalIntegrators int_type);
  
    
  /** @name Simulate Open-Loop control */
  ///@{ 

  //! Perform an open-loop simulation of the system in combination with the TebController.
  std::unique_ptr<SimResults> simOpenLoop(const double* const x0, const double* const xf, bool plot = true, const double* const add_step_response = nullptr, BaseController* ctrl = nullptr);

  /** \overload std::unique_ptr<SimResults> simOpenLoop(const double* const x0, const double* const xf, bool plot, const double* const add_step_response, BaseController* ctrl) */
  std::unique_ptr<SimResults> simOpenLoop(const std::array<double, p>& x0, const std::array<double, p>& xf, bool plot = true, const std::array<double, q>& add_step_response = {}, BaseController* ctrl = nullptr)
  {
	  return simOpenLoop(x0.data(), xf.data(), plot, std::all_of(add_step_response.begin(), add_step_response.end(), [](double i){return i == 0; }) ? nullptr : add_step_response.data(), ctrl);
  }

  //! Perform an open-loop simulation of the system for different TebControllers
  void simOpenLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, bool plot = true, const double* const add_step_response = nullptr);

  /** \overload void simOpenLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, bool plot, const double* const add_step_response) */
  void simOpenLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const std::array<double, p>& x0, const std::array<double, p>& xf, bool plot = true, const std::array<double, q>& add_step_response = {})
  {
	  simOpenLoop(controllers, x0.data(), xf.data(), plot, std::all_of(add_step_response.begin(), add_step_response.end(), [](double i){return i == 0; }) ? nullptr : add_step_response.data());
  }

  ///@}
    


  /** @name Simulate Closed-Loop control */
  ///@{ 

  //! Perform a closed-loop simulation of the system in combination with the TebController.
  std::unique_ptr<SimResults> simClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping = false, bool plot_result = true, bool plot_steps = true, const double* const add_step_response = nullptr, BaseController* ctrl = nullptr);
  
  /** \overload std::unique_ptr<SimResults> simClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping, bool plot_result, bool plot_steps, const double* const add_step_response, BaseController* ctrl) */
  std::unique_ptr<SimResults> simClosedLoop(const std::array<double, p>& x0, const std::array<double, p>& xf, double sim_time, bool manual_stepping = false, bool plot_result = true, bool plot_steps = true, const std::array<double, q>& add_step_response = {}, BaseController* ctrl = nullptr)
  {
	  return simClosedLoop(x0.data(), xf.data(), sim_time, manual_stepping, plot_result, plot_steps, std::all_of(add_step_response.begin(), add_step_response.end(), [](double i){return i == 0; }) ? nullptr : add_step_response.data(), ctrl);
  }
  
  //! Perform a closed-loop simulation of the system for different TebControllers
  void simClosedLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, double sim_time, bool plot_result = true, bool plot_steps = true, const double* const add_step_response = nullptr);

  /** \overload void simClosedLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const double* const x0, const double* const xf, double sim_time, bool plot_result, bool plot_steps, const double* const add_step_response) */
  void simClosedLoop(std::vector<std::pair<BaseController*, std::string>>& controllers, const std::array<double, p>& x0, const std::array<double, p>& xf, double sim_time, bool plot_result = true, bool plot_steps = true, const std::array<double, q>& add_step_response = {})
  {
	  simClosedLoop(controllers, x0.data(), xf.data(), sim_time, plot_result, plot_steps, std::all_of(add_step_response.begin(), add_step_response.end(), [](double i){return i == 0; }) ? nullptr : add_step_response.data());
  }

  ///@}



  /** @name Simulate Open- and Closed-Loop control at once */
  ///@{ 

  //! Perform both an open-loop and a closed-loop simulation of the system in combination with the TebController.
  std::unique_ptr<SimResults> simOpenAndClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping = false, bool plot_steps = true, const double* const add_step_response = nullptr);

  /** \overload std::unique_ptr<SimResults> simOpenAndClosedLoop(const double* const x0, const double* const xf, double sim_time, bool manual_stepping, bool plot_steps, const double* const add_step_response) */
  std::unique_ptr<SimResults> simOpenAndClosedLoop(const std::array<double, p>& x0, const std::array<double, p>& xf, double sim_time, bool manual_stepping = false, bool plot_steps = true, const std::array<double, q>& add_step_response = {})
  {
	  return simOpenAndClosedLoop(x0.data(), xf.data(), sim_time, manual_stepping, plot_steps, std::all_of(add_step_response.begin(), add_step_response.end(), [](double i){return i == 0; }) ? nullptr : add_step_response.data());
  }

  ///@}

    

  /** @name Simulate Step Response of the Plant */
  ///@{ 
    
  //! Simulate the step response of the system for a given step input vector \f$ \mathbf{u}_ref \f$
  std::unique_ptr<SimResults> simStepResponse(const double* const u_ref, double sim_time, const double* const x0 = nullptr, bool plot = true);
    
  /** \overload std::unique_ptr<SimResults> simStepResponse(const double* const u_ref, double sim_time, const double* const x0, bool plot) */
  std::unique_ptr<SimResults> simStepResponse(const std::array<double,q>& u_ref, double sim_time, const std::array<double,p>& x0 = {}, bool plot = true)
  {
       return simStepResponse(u_ref.data(), sim_time, x0.data(), plot);
  }
    
  ///@}  
    
    
  StateVector systemStep(const Eigen::Ref<const StateVector>& x_k, const Eigen::Ref<const ControlVector>& u_k, double dt)
  {
    assert(_system && _integrator);
    //return _integrator->integrate( x_k, _system->stateSpaceModel( x_k, u_k ), dt );
    return _integrator->integrate( x_k, u_k, _system, dt );
  }
  
  /**
   * @brief Set bounds on the control inputs for simulatin (saturation).
   * @param u_min pointer to the minimum control bounds [q x 1]
   * @param u_max pointer to the maximum control bounds [q x 1]
   */
  void setControlInputSaturation(const double* u_min, const double* u_max)
  {
      _control_bounds = std::make_pair(ControlVector(u_min),ControlVector(u_max));
  }
  
  
  /**
   * @brief Set a pointer to a function that is called at the beginning of each control step.
   * The callback function should have the following arguments:
   * - BaseController* ctrl - Pointer to the current controller object
   * - SystemDynamics* system - Pointer to the current system object
   * - Simulator* sim - Pointer to the caller (TebSimulator class) itself
   * @remarks Use std::bind() in order to pass class methods
   * @param callback function (object) that should be called
   */
  void setPreStepCallback(const std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)>& callback)
  {
    _callback_step_pre = callback;
  }
  
  /**
   * @brief Set a pointer to a function that is called after each control step.
   * The callback function should have the following arguments:
   * - BaseController* ctrl - Pointer to the current controller object
   * - SystemDynamics* system - Pointer to the current system object
   * - Simulator* sim - Pointer to the caller (TebSimulator class) itself
   * @remarks Use std::bind() in order to pass class methods
   * @param callback function (object) that should be called
   */
  void setPostStepCallback(const std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)>& callback)
  {
    _callback_step_post = callback;
  }
  
  
  /**
   * @brief Set a pointer to a function that is called at the beginning of each complete simulation.
   * The callback function should have the following arguments:
   * - BaseController* ctrl - Pointer to the current controller object
   * - SystemDynamics* system - Pointer to the current system object
   * - Simulator* sim - Pointer to the caller (TebSimulator class) itself
   * @remarks Use std::bind() in order to pass class methods
   * @param callback function (object) that should be called
   */
  void setPreSimCallback(const std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)>& callback)
  {
    _callback_sim_pre = callback;
  }
  
  void setNumberOfRobots(int n) { _number_of_robots = n; };
  
protected:
  

  //! Plot a container of SimResults::TimeSeries objects.
  void plotResults(const std::vector<SimResults::TimeSeries>& time_series, teb::PlotOptions* options = nullptr) const;   
  
  
  //! Saturate the control inputs \f$ \mathbf{u} \f$ specified by setControlInputSaturation().
  void saturateControl(Eigen::Ref<ControlVector> u);
  
  unsigned int  _number_of_robots = 5;
  
  
  BaseController* _controller = nullptr; //!< Pointer to the Controller used for controlling the system.
  SystemDynamics<p,q>* _system = nullptr; //!< Pointer to the SystemDynamics object that specifies the state space equations of the dynamic system.
  IntegatorPtr _integrator = IntegatorPtr(new RungeKutta5thOrder<p,q>()); //!< Numerical integration method (default: RungeKutta5thOrder)
  
  TebPlotter* _plotter = nullptr; //!< Pointer to a TebPlotter object used for visualization
  double _sample_time = -1.; //!< Sample time for simulation (-1: inherit variable dt from teb)
  
  //! Store hard control input bounds \f$ \mathbf{u}_{min} \f$ and \f$ \mathbf{u}_{max} \f$ for simulation (saturation).
  std::pair<ControlVector,ControlVector> _control_bounds = std::make_pair(ControlVector::Constant(-INF), ControlVector::Constant(INF));
  
  
  /** @brief Store a pointer to a function that is called at the beginning of each control step
   *  Use this function to make user-defined changes a-priori to each step.
   */
  std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)> _callback_step_pre;
  
  /** @brief Store a pointer to a function that is called after each control step
   *  Use this function to make user-defined changes a-posteriori to each step.
   */
  std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)> _callback_step_post;
  
  /** @brief Store a pointer to a function that is called at the beginning of each simulation
   *  Use this function to make user-defined changes a-priori to the complete simulation.
   */
  std::function<void(BaseController*,SystemDynamics<p,q>*, Simulator*)> _callback_sim_pre;
 
     
};


} // end namespace teb
 
    
#include <teb_package/simulation/simulator.hpp>

    
#endif // end ifdef RTW

#endif /* defined(__teb_package__simulator__) */
