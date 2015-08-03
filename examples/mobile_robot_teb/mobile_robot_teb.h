#ifndef __teb_package__examples_mobile_robot_teb__
#define __teb_package__examples_mobile_robot_teb__


#include <teb_package.h>

#include "rob_controller.h" // include customized controller


// Specify system model (here for simulation only, but could be used for the controller as well (as an alternative))
// We use a simple kinematic model here with integrator dynamics
class MobileRobot : public teb::SystemDynamics<3,2>
{
public:
  using StateVector = teb::SystemDynamics<3,2>::StateVector;
  using ControlVector = teb::SystemDynamics<3,2>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
      
      // kinematic motion constraints
      // states = [x y theta]
      // inputs = [v, omega]
      
      state_equations[0] = u[0] * std::cos(x[2]); // xdot = v*cos(theta)
      state_equations[1] = u[0] * std::sin(x[2]); // ydot = v*sin(theta)
      state_equations[2] = norm_angle(u[1]); // thetadot = omega
      return state_equations;
  }
    
};



#endif /* defined(__teb_package__examples_mobile_robot_teb__) */