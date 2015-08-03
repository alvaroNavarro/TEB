#ifndef __teb_package__system_dynamics__
#define __teb_package__system_dynamics__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <teb_package/utilities/car_like_type_driving.h>

namespace teb
{


/**
 * @brief Helper class for modeling nonlinear dynamic systems.
 *
 * @ingroup controller
 *
 * This abstract class provides an interface for modeling dynamic systems
 * utilizing nonlinear state space equations:
 * \f[ \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \f]
 * \f$ \mathbf{x} \in \mathbb{R}^p \f$ denotes the state vector and \f$ \mathbf{u} \in \mathbb{R}^q \f$ denotes the control input vector.
 * The system dynamics are modeled in continuous time-domain and discretized by the program itself.
 * - E.g.the TebController class discretizes the system equations using teb::FiniteDifferences
 *   (see EdgeSystemDynamics, finiteElemFwdEuler() and finiteElemCentralDiff()).
 * - And the Simulator class is able to apply different numerical integration schemes to the
 *   state space model implemented here (see NumericalIntegrator).
 *
 * @todo Methods/Intefrace for jacobian and hessian calculation
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @tparam p Number of state variables
 * @tparam q Number of input variables
 */
template <int p, int q>  
class SystemDynamics
{
  template <int pf, int qf, bool cf>
  friend class EdgeSystemDynamics; // Forward declaration
  
  template <int pf, int qf>
  friend class Simulator; // Forward declaration
  
  
public:    
  
  static const int DimStates = p; //!< Store number of states as static variable.
  static const int DimControls = q; //!< Store number of control inputs as static variable.  
  
  //using MobileRobotCarLikeSystem<p,q>::TypeDriving;
   
  //! Typedef for state vector with p states [p x 1].
  using StateVector = Eigen::Matrix<double, p, 1>;
  //! Typedef for control input vector with q controls [q x 1].
  using ControlVector = Eigen::Matrix<double, q, 1>;
    
  SystemDynamics() {} //!< Empty Constructor.
  virtual ~SystemDynamics() {} //!< Empty Destructor
  
  virtual teb::TypeDriving getTypeDriving() const = 0;
  virtual const Eigen::Vector2d& getRobotDimension() const = 0;  
  
  
   /** 
    * @brief Implement the nonlinear state space model \f$ \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \f$ here.
    *
    * This method defines the state space model of the dedicated dynamic system. \n
    * Implement only the right hand side \f$ f(\cdot) \f$ of \f$ \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \f$
    * and return the equation as a StateVector type.
    *
    * E.g. \f$ \dot x = x + u \ \ x,u \in \mathbb{R} \f$ (with \c p = 1 and \c q = 1)
    * \code
    *  StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
    *  {
    *       StateVector f = x[0] + u[0];
    *       return f;
    *  }
    * \endcode
    *
    * @param x State vector \f$ \mathbf{x} \f$ [p x 1]
    * @param u Control input vector \f$ \mathbf{u} \f$ [q x 1]
    * @return \f$ f(\mathbf{x}, \mathbf{u}) \f$ as vector [p x 1]
    */
  virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const = 0; //! xdot = stateSpaceModel(x,u) 
   

protected:
  
  /**
   * @brief Forward difference approximation for continuous-time derivatives
   *
   * Approximate derivatives using \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k}}{\Delta T} \f$
   *
   * @param x1 Discrete state vector \f$ \mathbf{x}_k \f$ [p x 1]
   * @param x2 Discrete state vector \f$ \mathbf{x}_{k+1} \f$ [p x 1]
   * @param dt Time interval / discretization step width \f$ \Delta T \f$
   * @return Finite difference approximation
   */
  StateVector finiteElemFwdEuler(const Eigen::Ref<const StateVector>& x1, const Eigen::Ref<const StateVector>& x2, double dt)
  {
      return ( x2 - x1 ) / dt;
  }
    

  /**
   * @brief Central difference approximation for continuous-time derivatives
   *
   * Approximate derivatives using \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k-1}}{2\Delta T} \f$
   *
   * @param x_km1 Discrete state vector \f$ \mathbf{x}_{k-1} \f$ [p x 1]
   * @param x_kp1 Discrete state vector \f$ \mathbf{x}_{k+1} \f$ [p x 1]
   * @param dt Time interval / discretization step width \f$ \Delta T \f$
   * @return Finite difference approximation
   */
  StateVector finiteElemCentralDiff(const Eigen::Ref<const StateVector>& x_km1, const Eigen::Ref<const StateVector>& x_kp1, double dt)
  {
      // simple central differences
      return ( x_kp1 - x_km1 ) / (2*dt);
  }  
    
};
   





} // end namespace teb

#endif /* defined(__teb_package__system_dynamics__) */
