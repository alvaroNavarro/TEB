#ifndef __teb_package__examples_linear_system_state_space__
#define __teb_package__examples_linear_system_state_space__


#include <teb_package.h>


template <int p, int q>
class LinearSystemStateSpace : public teb::SystemDynamics<p,q>
{
public:
  using StateVector = typename teb::SystemDynamics<p,q>::StateVector;
  using ControlVector = typename teb::SystemDynamics<p,q>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      return _A * x + _B * u;
  }
    
  void setStateSpaceModel(const Eigen::Ref<const Eigen::Matrix<double,p,p>>& A, const Eigen::Ref<const Eigen::Matrix<double,p,q>>& B)
  {
    _A = A;
    _B = B;
  }
    
protected:
    Eigen::Matrix<double,p,p> _A;
    Eigen::Matrix<double,p,q> _B;
};



#endif /* defined(__teb_package__examples_linear_system_state_space__) */