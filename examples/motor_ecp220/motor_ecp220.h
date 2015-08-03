#ifndef __teb_package__examples_motor_ecp220__
#define __teb_package__examples_motor_ecp220__


#include <teb_package.h>


class MotorECP220 : public teb::SystemDynamics<3,1>
{
public:
  using StateVector = teb::SystemDynamics<3,1>::StateVector;
  using ControlVector = teb::SystemDynamics<3,1>::ControlVector;
  
  inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
  {
      //! x = [q qdot i]
      StateVector state_equations;
  
      state_equations[0] = x[1]; // qdot = qdot
      state_equations[1] = 1/Jr * ( k_tau * x[2] - D * x[1] - tau_l );
      state_equations[2] = 1/L * ( u[0] - R * x[2] - x[1] / kE );
      
      
      return state_equations;
  }
    double Jr = 3.39e-5; // kgm^2    (Rotor Inertia)
    double k_tau = 0.087; // Nm/A     (Torque Constant);
    double D = 1.68e-5; // Nm/(rad/s)    (Viscous Damping Factor)
    double tau_l = 0.3036; // Nm  (Continuous Torque)
    double kE = 0.087; // V/(rad/s)  (Voltage Constant)
    double R = 1.21; // Ohm    (Terminal Resistance);
    double L = 0.41e-3; // H     (Inductance)
    

};



#endif /* defined(__teb_package__examples_linear_system_state_space__) */