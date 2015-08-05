#ifndef __teb_package__examples_car_like_system__
#define __teb_package__examples_car_like_system__

#include <teb_package.h>
#include <Eigen/Core>



template <int p, int q>
class MobileRobotCarLikeSystem : public teb::SystemDynamics<p,q>
{
public:     
  
  using StateVector   = typename teb::SystemDynamics<p,q>::StateVector;
  using ControlVector = typename teb::SystemDynamics<p,q>::ControlVector;
  
  virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      StateVector state_equations;        
      
      if(_type == teb::TypeDriving::FRONT_WHEEL)
      {
	 state_equations[0] = u[0] * std::cos(x[2]) * std::cos(x[3]);     //! xdot = v * cos(theta) * cos(phi)
	 state_equations[1] = u[0] * std::sin(x[2]) * std::cos(x[3]);     //! ydot = v * sin(theta) * cos(phi)
         state_equations[2] = u[0] * std::sin(x[3])/_L;                   //! thetadot = v * sin(phi) / L
         state_equations[3] = norm_angle(u[1]);                           //! phidot = omega_steer
      
      }
      
      else
      {
	state_equations[0] = u[0] * std::cos(x[2]);			//! xdot = v * cos(theta)
        state_equations[1] = u[0] * std::sin(x[2]);			//! ydot = v * sin(theta)
        state_equations[2] = u[0] * std::tan(x[3])/_L;                    //! thetadot = v * tan(phi) / L
        state_equations[3] = norm_angle(u[1]);				//! phidot = omega_steer
      	
      }      
      
      return state_equations;
  }

  
  void setDistanceBetweenFrontRearAxles(double distance) {_L = distance;};
  void setTypeDriving(teb::TypeDriving type){_type = type;};
  void setRobotWidth(double width){_width = width;};
  
  double getRobotLength(){return(_L);}
  double getRobotWidth(){return(_width);}  
  
  void setRobotDimesion(double length, double width)
  {
    _robot_dim[0] = length;
    _robot_dim[1] = width;
  }    
  
  //const Eigen::Vector2d& getRobotDimension() const { return(_robot_dim); };
  virtual const Eigen::Vector2d& getRobotDimension() const {return(_robot_dim); }
  
  double getMaxTurningRadius() {return(_phi_max); }
  
  void setRadiusRobot() { _R_max = _width/2 + _L; }  
  double getRadiusRobot() {return(_R_max);}
  
  virtual teb::TypeDriving getTypeDriving() const  {return (_type); }
  
  void setEpsilonCostFunction(double epsilon) {_epsilon = epsilon;}
  double getEpsilonCostFunction() {return(_epsilon);}
  
  double getUpperBoundEpsilon() {return(_limit_epsilon);}
  double getStepEpsilon() {return(_step_epsilon);}
  
  void incrementEpsilon() { _epsilon += _step_epsilon;}
   
protected:

    double _L = 0.7;
    double _width = 0.5;
    teb::TypeDriving _type = teb::TypeDriving::REAR_WHEEL;   //! In case the forget to invoke the function setTypeDriving, the default type of
						             //! driving selected is REAR_WHEEL    
						   
    Eigen::Vector2d _robot_dim;		      
    
    double _phi_max = M_PI/6;    //! Correspond to the maximum angle of turning    
    double _R_max = 1;        
    
    double _epsilon = 0.1;
    double _step_epsilon = 0.1;
    double _limit_epsilon = 0.9;
            
};


#endif /* defined(__teb_package__examples_car_like_system__) */