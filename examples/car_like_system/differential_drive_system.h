#ifndef __teb_package__examples_differential_drive_system__
#define __teb_package__examples_differential_drive_system__

#include <teb_package.h>
#include <Eigen/Core>

template <int p, int q>
class DifferentalDriveRobotSystem : public teb::SystemDynamics<p,q>
{
public:     
  
  using StateVector   = typename teb::SystemDynamics<p,q>::StateVector;
  using ControlVector = typename teb::SystemDynamics<p,q>::ControlVector;
  
  virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;        
      
       state_equations[0] = u[0] * std::cos(x[2]);     //! xdot = v * cos(theta)
       state_equations[1] = u[0] * std::sin(x[2]);     //! ydot = v * sin(theta)
       state_equations[2] = norm_angle(u[1]);          //! thetadot = omega       
            
      return state_equations;
  }

  
  void setRobotDimension(double radius){_radius = radius;};              
  virtual const double& getRobotDimension() const {return(_radius); }
    
  void setEpsilonCostFunction(double epsilon) {_epsilon = epsilon;}
  double getEpsilonCostFunction() {return(_epsilon);}
  
  double getUpperBoundEpsilon() {return(_limit_epsilon);}
  double getStepEpsilon() {return(_step_epsilon);}
  
  void incrementEpsilon() { _epsilon += _step_epsilon;}
   
protected:
    
    double _radius = 0.5;
    
    double _epsilon = 0;
    double _step_epsilon = 0.1;
    double _limit_epsilon = 0.2;
            
};


#endif /* defined(__teb_package__examples_differential_drive_system__) */