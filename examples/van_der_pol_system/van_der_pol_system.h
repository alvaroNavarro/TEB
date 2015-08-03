#ifndef __teb_package__examples_van_der_pol_system__
#define __teb_package__examples_van_der_pol_system__


#include <teb_package.h>


class VanDerPolSystem : public teb::SystemDynamics<2,1>
{
public:
  using StateVector = teb::SystemDynamics<2,1>::StateVector;
  using ControlVector = teb::SystemDynamics<2,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      state_equations[0] = x[1]; // xdot = xdot
      state_equations[1] = - _a * (x[0] - 1) * x[1] - x[0] + u[0];
      
      
      return state_equations;
  }
    
    void setParameters(double a) {_a = a;};
    
protected:
    double _a = 1;
    
};



#endif /* defined(__teb_package__examples_van_der_pol_system__) */