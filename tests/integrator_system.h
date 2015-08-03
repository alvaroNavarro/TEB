#ifndef __teb_package__examples_integrator_system__
#define __teb_package__examples_integrator_system__


#include <teb_package.h>


template <int p>
class IntegratorSystem : public teb::SystemDynamics<p,1>
{
public:
  using StateVector = typename teb::SystemDynamics<p,1>::StateVector;
  using ControlVector = typename teb::SystemDynamics<p,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      const int no_int = p-1;
      if (p>1) state_equations.head(no_int) = x.segment(1,no_int); // x^(1:n-1) = x^(2:n)
      
      // add last equation containing control u
      state_equations[no_int] = u[0] / _time_constant; // x^(n) = u / T
      return state_equations;  
  }

    
    void setTimeConstant(double time_constant) {_time_constant = time_constant;};
    
protected:

    double _time_constant = 1;
};



#endif /* defined(__teb_package__examples_integrator_system__) */