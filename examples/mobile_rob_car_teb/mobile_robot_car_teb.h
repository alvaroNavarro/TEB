#ifndef __teb_package__examples_mobile_robot_car_teb__
#define __teb_package__examples_mobile_robot_car_teb__


#include <teb_package.h>

#include "rob_controller_car_like.h" // include customized controller

// Specify system model (here for simulation only, but could be used for the controller as well (as an alternative))
// We use a simple kinematic model here with integrator dynamics
class MobileRobotCarLike : public teb::SystemDynamics<4,2>
{
public:
  using StateVector = teb::SystemDynamics<4,2>::StateVector;
  using ControlVector = teb::SystemDynamics<4,2>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
      
      // kinematic motion constraints
      // states = [x y theta phi]
      // inputs = [v, omega_steering]
      
      state_equations[0] = u[0]  * std::cos(x[2]);        // xdot = v*cos(theta)
      state_equations[1] = u[0]  * std::sin(x[2]);        // ydot = v*sin(theta)
      state_equations[2] = (u[0] * std::tan(x[3]))/L;     // thetadot = tan(phi)*v/L   
      state_equations[3] = norm_angle(u[1]);              // phidot = omega_steer
      return state_equations;
  }
  
  double L = 0.3;
    
};



#endif /* defined(__teb_package__examples_mobile_robot_car_teb__) */
