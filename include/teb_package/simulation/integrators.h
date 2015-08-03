#ifndef __teb_package__integrators__
#define __teb_package__integrators__

#include <teb_package/base/system_dynamics.h>
#include <Eigen/Core>
#include <queue>

namespace teb
{

/** Enumeration for different numerical integration methods */
enum class NumericalIntegrators
{
    EXPLICIT_EULER, //!< Forward Euler @sa ExplicitEuler
    RUNGE_KUTTA_CLASSIC, //!< Runge Kutta 4th Order @sa RungeKuttaClassic
    RUNGE_KUTTA_5TH //!< Runge Kutta 5th Order @sa RungeKutta5thOrder
};
  


/**
 * @brief Interface for numerical integration methods used in simulation.
 *
 * @ingroup simulation
 *
 * This class defines the interface for numerical integration schemes.
 *
 * @todo Implement and test Runge Kutta with derivatives http://www.emis.de/journals/EJDE/conf-proc/02/g1/goeken.pdf
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template <int p, int q>
class NumericalIntegrator
{
public:  

  //! Typedef for state vector with p states [p x 1].
  using StateVector = Eigen::Matrix<double,p,1>;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = Eigen::Matrix<double,q,1>;
  
  NumericalIntegrator() {} //!< Empty Constructor.
  virtual ~NumericalIntegrator() {} //!< Empty Destructor
  
  /** 
   * @brief Integrate a nonlinear state-space model \f[ \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \f]
   *
   * Integrate a nonlinear state-space model \f[ \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \f]
   * w.r.t. \f$ \mathbf{x} \f$. In particular, it relies on discrete states. The method calculates
   * \f$ \mathbf{x}_{k+1} \f$ based on \f$ \mathbf{x}_{k} \f$, \f$ \mathbf{u}_{k} \f$ and the duration \f$ \Delta T \f$.
   * The dedicated system dynamics equations may be specified using a SystemDynamics object.
   * 
   * @param x_k Current state vector \f$ \mathbf{x}_k \f$ [p x 1]
   * @param u_k Current state vector \f$ \mathbf{u}_k \f$ [q x 1]
   * @param system_equation Pointer to the SystemDynamics object that stores the system equations.
   * @param dt Time interval for the integration 
   * @return Integrated state vector \f$ \mathbf{x}_{k+1} \f$ [p x 1]
   */
  virtual StateVector integrate(const Eigen::Ref<const StateVector>& x_k, const Eigen::Ref<const ControlVector>& u_k, SystemDynamics<p,q>* system_equation, double dt) const = 0; // maybe replace with static polymorphism in order to allow template based function pointers and inheritance (more generic functions)
  
};


    
/**
 * @brief Simple explicit euler method for integration
 *
 * @ingroup simulation
 *
 * For more information please refer to http://en.wikipedia.org/wiki/Euler_method
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template <int p, int q>
class ExplicitEuler : public NumericalIntegrator<p,q>
{
public:
  //! Typedef for state vector with p states [p x 1].
  using StateVector = typename NumericalIntegrator<p,q>::StateVector;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = typename NumericalIntegrator<p,q>::ControlVector;
  
  ExplicitEuler() : NumericalIntegrator<p,q>() {} //!< Empty Constructor.
  virtual ~ExplicitEuler() {} //!< Empty Destructor.
  
  // Implements NumericalIntegrator::integrate()
  virtual StateVector integrate(const Eigen::Ref<const StateVector>& x_k, const Eigen::Ref<const ControlVector>& u_k, SystemDynamics<p,q>* system_equation, double dt) const
  {
       return x_k + dt * system_equation->stateSpaceModel(x_k, u_k); 
  }
  
  
};
  

/**
 * @brief Classic Runge Kutta method (fourth order)
 *
 * @ingroup simulation
 *
 * For more information please refer to http://en.wikipedia.org/wiki/Runge–Kutta_methods
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template <int p, int q>
class RungeKuttaClassic : public NumericalIntegrator<p,q>
{
public:
  //! Typedef for state vector with p states [p x 1].
  using StateVector = typename NumericalIntegrator<p,q>::StateVector;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = typename NumericalIntegrator<p,q>::ControlVector;

  
  RungeKuttaClassic() : NumericalIntegrator<p,q>() {} //!< Empty Constructor.
  virtual ~RungeKuttaClassic() {} //!< Empty Destructor.
  
    
  // Implements NumericalIntegrator::integrate()
  virtual StateVector integrate(const Eigen::Ref<const StateVector>& x_k, const Eigen::Ref<const ControlVector>& u_k, SystemDynamics<p,q>* system_equation, double dt) const
  {
       StateVector k1 = dt * system_equation->stateSpaceModel(x_k, u_k);
       StateVector k2 = dt * system_equation->stateSpaceModel(x_k + k1 /2, u_k);
       StateVector k3 = dt * system_equation->stateSpaceModel(x_k + k2 /2, u_k);
       StateVector k4 = dt * system_equation->stateSpaceModel(x_k + k3, u_k);
    
       return x_k + (k1 + 2*k2 + 2*k3 + k4) / 6;
  } 
  
};



/**
 * @brief Runge Kutta method (fifth order, slighly modified)
 *
 * @ingroup simulation
 *
 * For more information please refer to
 * http://www.jstor.org/stable/pdfplus/2027775.pdf?acceptTC=true&jpdConfirm=true (Equation 2)
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template <int p, int q>
class RungeKutta5thOrder : public NumericalIntegrator<p,q>
{
public:
  //! Typedef for state vector with p states [p x 1].
  using StateVector = typename NumericalIntegrator<p,q>::StateVector;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = typename NumericalIntegrator<p,q>::ControlVector;

  
  RungeKutta5thOrder() : NumericalIntegrator<p,q>() {} //!< Empty Constructor.
  virtual ~RungeKutta5thOrder() {} //!< Empty Destructor.
  
    
  // Implements NumericalIntegrator::integrate()
  virtual StateVector integrate(const Eigen::Ref<const StateVector>& x_k, const Eigen::Ref<const ControlVector>& u_k, SystemDynamics<p,q>* system_equation, double dt) const
  {
       StateVector k1 = dt * system_equation->stateSpaceModel(x_k, u_k);
       StateVector k2 = dt * system_equation->stateSpaceModel(x_k + 4*k1 /11 , u_k);
       StateVector k3 = dt * system_equation->stateSpaceModel(x_k + (9*k1 + 11*k2)/50, u_k);
       StateVector k4 = dt * system_equation->stateSpaceModel(x_k + (-11*k2 + 15*k3)/4, u_k);
       StateVector k5 = dt * system_equation->stateSpaceModel(x_k + ( (81 + 9*sqrt(6))*k1 + (255-55*sqrt(6))*k3 + (24 - 14*sqrt(6))*k4)/400, u_k);
       StateVector k6 = dt * system_equation->stateSpaceModel(x_k + ( (81 - 9*sqrt(6))*k1 + (255+55*sqrt(6))*k3 + (24 + 14*sqrt(6))*k4)/600, u_k);
    
       return x_k + (4*k1 + (16 + sqrt(6))*k5 + (16 - sqrt(6))*k6) / 36;
  } 
  
};

} // end namespace teb

#endif /* defined(__teb_package__integrators__) */
