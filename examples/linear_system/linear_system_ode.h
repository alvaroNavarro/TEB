#ifndef __teb_package__examples_linear_system_ode__
#define __teb_package__examples_linear_system_ode__


#include <teb_package.h>


template <int p>
class LinearSystemODE : public teb::SystemDynamics<p,1>
{
public:
  using StateVector = typename teb::SystemDynamics<p,1>::StateVector;
  using ControlVector = typename teb::SystemDynamics<p,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      const int no_int = p-1;
      // integrator states -> control normal form
      if (p>1) state_equations.head(no_int) = x.segment(1,no_int); // x^(1:n-1) = x^(2:n)
      
      // add last equation containing ODE coefficients
      state_equations[no_int] = _coeff_b*u[0]; // u
      for (int i = 0; i < p; ++i) // add all other ode coefficients
      {
          state_equations[no_int] -= _coeff_a[p-i] * x[i];
      }
      state_equations[no_int] /= _coeff_a[0];
      return state_equations;
  }
    
void setODECoefficients(const double* coeff_a, double coeff_b)
{
    _coeff_a = coeff_a;
    _coeff_b = coeff_b;
}
    
protected:
    const double* _coeff_a = nullptr;
    double _coeff_b = 1;
};



#endif /* defined(__teb_package__examples_linear_system_ode__) */