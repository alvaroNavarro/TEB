#ifndef __teb_package__examples_trailer_system__
#define __teb_package__examples_trailer_system__

#include <teb_package.h>

#include "controller_car_like.h"


class MobileRobotTruckSystem : public teb::SystemDynamics<5,2>
{
public:
  using StateVector   = typename teb::SystemDynamics<5,2>::StateVector;
  using ControlVector = typename teb::SystemDynamics<5,2>::ControlVector;
  
  virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      state_equations[0] = u[0] * std::cos(x[3]);     			//! xdot = v * cos(theta0)
      state_equations[1] = u[0] * std::sin(x[3]);    			//! ydot = v * sin(theta0)
      state_equations[2] = norm_angle(u[1]);                            //! phidot = omega_steer
      state_equations[3] = u[0] * std::tan(x[2])/_L0;                   //! theta0dot = v * tan(phi) / L0
      state_equations[4] = -u[0] * std::sin(x[4] - x[3])/_L1;           //! theta1dot = v * sin(theta1 - theta0) / L1
      
      return state_equations;
  }

    
  void setLengthTrailer(double length_trailer) { _L1 = length_trailer; };
  void setLengthTruck(double length_truck) { _L0 = length_truck; };
  void setWidthTruck(double width_truck){ _width0 = width_truck; };
  void setWidthTrailer(double width_trailer){ _width1 = width_trailer; };
  
  double getLengthTruck() {return(_L0); };
  double getLengthTrailer() {return(_L1); };
  double getWidthTruck(){ return(_width0); };
  double getWidthTrailer(){ return(_width1); };
  
  void setTrackDimension(double *dim)
  {
      _L0     = dim[0];
      _width0 = dim[1];
      _L1     = dim[2];
      _width1 = dim[3];
  }
    
protected:
    
    double _L0 = 1.0;
    double _L1 = 1.0;
    double _width0 = 0.5;
    double _width1 = 0.5;
   
};


#endif /* defined(__teb_package__examples_trailer_system__) */