#ifndef __teb_package__examples_rob_cost_edges__
#define __teb_package__examples_rob_cost_edges__


#include <teb_package.h>


namespace teb
{
    
    // Declarations for cost functions / hyper edges
    
    // Non-holonomic-Kinematics
	class EdgeKinematics : public BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>> // Cost vector dimension: 1, Vertices: StateVertex
    {
    public:
        
        //! Empty Constructor
		EdgeKinematics(StateVertex<3>& state1, StateVertex<3>& state2) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>>(state1, state2) {}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
			StateVertex<3>* sample_i = static_cast<StateVertex<3>*>(_vertices[0]);
			StateVertex<3>* sample_ip1 = static_cast<StateVertex<3>*>(_vertices[1]);
            
            // Direction vector between two samples:
            Eigen::Vector2d deltaS = sample_ip1->states().head(2) - sample_i->states().head(2);
            double angle_i = sample_i->states()[2];
            double angle_ip1 = sample_ip1->states()[2];
        
            // Nonholonomic Kinematics
            _values[0] = fabs( ( cos(angle_i)+cos(angle_ip1) ) * deltaS[1] - ( sin(angle_i)+sin(angle_ip1) ) * deltaS[0] );
        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
            
            
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    // Limit Velocity
	class EdgeLimitVelocity : public BaseEdge<4, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>, TimeDiff> // Cost vector dimension: 4, Vertices: StateVertex, StateVertex, TimeDiff
    {
    public:
        
        //! Empty Constructor
		EdgeLimitVelocity(StateVertex<3>& state1, StateVertex<3>& state2, TimeDiff& dt) : BaseEdge<4, FUNCT_TYPE::NONLINEAR, StateVertex<3>, StateVertex<3>, TimeDiff>(state1, state2, dt)
        {
        }
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
			StateVertex<3>* sample_i = static_cast<StateVertex<3>*>(_vertices[0]);
			StateVertex<3>* sample_ip1 = static_cast<StateVertex<3>*>(_vertices[1]);
            TimeDiff* dt = static_cast<TimeDiff*>(_vertices[2]);
            
            // Direction vector between two samples:
            Eigen::Vector2d deltaS = sample_ip1->states().head(2) - sample_i->states().head(2);
            double vel = deltaS.norm() / dt->dt();
            double omega = norm_angle( sample_ip1->states()[2] - sample_i->states()[2] ) / dt->dt();
            
            // Velocity limits: c(x) < 0
            _values[0] = vel - v_max;
            _values[1] = -vel + v_min;
            _values[2] = omega - omega_max;
            _values[3] = -omega + omega_min;   
        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
        
        double v_max = 1;
        double v_min = -1;
        double omega_max = 0.5;
        double omega_min = -0.5;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

       
    // Obstacle-Avoidance (point-representation)
	class EdgeObstacle : public BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>> // Cost vector dimension: 1, Vertices: StateVertex
    {
    public:
        
        //! Empty Constructor
		EdgeObstacle(StateVertex<3>& state) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<3>>(state) {}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
			StateVertex<3>* sample_i = static_cast<StateVertex<3>*>(_vertices[0]);
            
            // Direction vector between sample and obstacle:
            Eigen::Vector2d deltaS = sample_i->states().head(2) - obstacle_position;
            //double angle_i = sample_i->states()[2];
        
            // Projection of the distance deltaS orthogonal to the TEB
	    //double angdiff = atan2(deltaS[1],deltaS[0])-angle_i;
	    double proj_dist = deltaS.norm();//*fabs(sin(angdiff));
    
	    // Constraint proj_dist > min_distance -> -proj_dist + min_distance < 0
            _values[0] = -proj_dist + min_distance;
        }
           
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
            
            
        Eigen::Vector2d obstacle_position; // DO NOT FORGET TO SET VALUE
        double min_distance = 0.5;
            
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
        


} // end namespace teb



#endif /* defined(__teb_package__examples_rob_cost_edges__) */