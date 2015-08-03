#ifndef __teb_package__examples_rocket_system__
#define __teb_package__examples_rocket_system__


#include <teb_package.h>


class FreeSpaceRocketSystem : public teb::SystemDynamics<3,1>
{
public:
  using StateVector = teb::SystemDynamics<3,1>::StateVector;
  using ControlVector = teb::SystemDynamics<3,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      state_equations[0] = x[1]; // sdot = v
      state_equations[1] = (u[0] - 0.02 * x[1] * x[1]) / x[2]; // (u-0.02*v^2)/m
      state_equations[2] = -0.01 * u[0] * u[0]; // -0.01 * u^2
      
      
      return state_equations;
  }
    
};



#endif /* defined(__teb_package__examples_rocket_system__) */