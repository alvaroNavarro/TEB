#ifndef __teb_package__examples_rob_cost_edges_car_like__
#define __teb_package__examples_rob_cost_edges_car_like__


#include <teb_package.h>


namespace teb
{
    
    // Declarations for cost functions / hyper edges
    
    // Non-holonomic-Kinematics
	class EdgeKinematicsCarLike : public BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>> // Cost vector dimension: 1, Vertices: StateVertex
    {
    public:
        
        //! Empty Constructor
	EdgeKinematicsCarLike(StateVertex<4>& state1, StateVertex<4>& state2) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>>(state1, state2) {}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
		StateVertex<4>* sample_i = static_cast<StateVertex<4>*>(_vertices[0]);
		StateVertex<4>* sample_ip1 = static_cast<StateVertex<4>*>(_vertices[1]);
            
            // Direction vector between two samples:
            Eigen::Vector2d deltaS = sample_ip1->states().head(2) - sample_i->states().head(2);
            double angle_beta_i   = sample_i->states()[2];
            double angle_beta_ip1 = sample_ip1->states()[2];
	    double angle_phi_i    = sample_i->states()[3];
            double angle_phi_ip1  = sample_ip1->states()[3];
        
            // Nonholonomic Kinematics
            _values[0] = fabs( ( cos(angle_i)+cos(angle_ip1) ) * deltaS[1] - ( sin(angle_i)+sin(angle_ip1) ) * deltaS[0] );
        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
            
            
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    // Limit Velocity
	class EdgeLimitVelocityCarLike : public BaseEdge<4, FUNCT_TYPE::NONLINEAR, StateVertex<4>, StateVertex<4>, TimeDiff> // Cost vector dimension: 4, Vertices: StateVertex, StateVertex, TimeDiff
    {
    public:
        
        //! Empty Constructor
	EdgeLimitVelocityCarLike(StateVertex<4>& state1, StateVertex<4>& state2, TimeDiff& dt) : BaseEdge<4, FUNCT_TYPE::NONLINEAR, StateVertex<4>, StateVertex<4>, TimeDiff>(state1, state2, dt)
        {
        }
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
		StateVertex<4>* sample_i = static_cast<StateVertex<4>*>(_vertices[0]);
		StateVertex<4>* sample_ip1 = static_cast<StateVertex<4>*>(_vertices[1]);
                TimeDiff* dt = static_cast<TimeDiff*>(_vertices[2]);
            
            // Direction vector between two samples:
            Eigen::Vector2d deltaS = sample_ip1->states().head(2) - sample_i->states().head(2);
            double vel   = deltaS.norm() / dt->dt();            
	    double phi   = norm_angle(sample_ip1->states()[3] - sample_i->states()[3]) / dt->dt();
            
            // Velocity limits: c(x) < 0
            _values[0] = vel - v_max;
            _values[1] = -vel + v_min;
            _values[2] = phi - phi_max;
            _values[3] = -phi + phi_min;   
        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
        
        double v_max = 1;
        double v_min = -1;
        double phi_max = 0.5;
        double phi_min = -0.5;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };                  


} // end namespace teb



#endif /* defined(__teb_package__examples_rob_cost_edges_car_like__) */
