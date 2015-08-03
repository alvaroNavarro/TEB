#ifndef __teb_package__examples_overhead_crane__
#define __teb_package__examples_overhead_crane__


#include <teb_package.h>

template<bool linearized, bool friction>
class OverheadCrane : public teb::SystemDynamics<4,1>
{
public:
  using StateVector = teb::SystemDynamics<4,1>::StateVector;
  using ControlVector = teb::SystemDynamics<4,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;
  
      // x = [x xdot theta thetadot]^T
      
      
      if (!friction)
      {
	  if (linearized)
	  {
	    state_equations[0] = x[1];
	    state_equations[1] = g*M/m * x[2] + 1/m * u[0];
	    state_equations[2] = x[3];
	    state_equations[3] = -g/l*(1+M/m) * x[2] - 1/(l*m) * u[0];
	  }
	  else
	  {
	    state_equations[0] = x[1];
	    state_equations[1] = g*M/m * std::sin(x[2]) + 1/m * u[0];
	    state_equations[2] = x[3];
	    state_equations[3] = -g/l*(1+M/m) * std::sin(x[2]) - 1/(l*m) * u[0];	
	  }
      }
      else
      if (linearized)
	  {
	    state_equations[0] = x[1];
	    state_equations[1] = g*M/m * x[2] + 1/m * ( u[0] - cx * x[1] ) + ctheta*M/m * x[3];
	    state_equations[2] = x[3];
	    state_equations[3] = -g/l*(1+M/m) * x[2] - 1/(l*m) * ( u[0] - cx * x[1]) - ctheta/l*(1+M/m) * x[3];
	  }
	  else
	  {
		  /*	
	  // deduced model 
	  state_equations[0] = x[1];
	  state_equations[1] = (u[0]+m*std::sin(x[2])*(l*x[3]*x[3]+g*std::cos(x[2])))/(M+m-m*std::cos(x[2])*std::cos(x[2])) ;
	  state_equations[2] = x[3];
	  state_equations[3] = (u[0]*std::cos(x[2])+(M+m)*g*std::sin(x[2])+m*l*x[3]*x[3]*std::sin(x[2])*std::cos(x[2]))/(m*l*std::cos(x[2])*std::cos(x[2])-M*l-m*l);
	  */
	  
	    state_equations[0] = x[1];
	    state_equations[1] = g*M/m * std::sin(x[2]) + 1/m * ( u[0] - cx * x[1] ) + ctheta*M/m * x[3];
	    state_equations[2] = x[3];
	    state_equations[3] = -g/l*(1+M/m) * std::sin(x[2]) - 1/(l*m) *( u[0] - cx * x[1] ) - ctheta/l*(1+M/m) * x[3];	
	  }
	  
	  
      return state_equations;
  }
    
    
  void setParameters(double mass_cart, double mass_pend, double length_pend, double x_frict = 0, double theta_frict = 0, double gravity = 9.81)
  {
    m = mass_cart;
    M = mass_pend;
    l = length_pend;
    g = gravity;
    
    
    cx = x_frict;
    ctheta = theta_frict;
  }
    
protected:
    double m = 0.95;//0.1;
    double M = 0.28;//0.5;
    double l = 0.42;//0.5;
    double g = 9.81;
    
    double cx = 0.1;
    double ctheta = 0.1;
    
};



#endif /* defined(__teb_package__examples_overhead_crane__) */