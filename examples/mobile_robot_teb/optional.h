#ifndef __teb_package__examples_rob_cost_edges_optional__
#define __teb_package__examples_rob_cost_edges_optional__


#include <teb_package.h>


namespace teb
{
    
    // Declarations for optional cost functions / hyper edges
    
  
    //Limit Acceleration (currently not included)
    template <bool start_acc=false, bool goal_acc=false>
    class EdgeLimitAcceleration : public ConstraintEdgeMulti<4> // Cost vector dimension: 1, Vertices: TebSample
    {
    public:
        
        //! Empty Constructor
        EdgeLimitAcceleration() : ConstraintEdgeMulti<4>()
        {
	    static_assert(!(start_acc && goal_acc), "This edge cannot define a start and a goal boundary for acceleration/velocity at once");
	    if (start_acc || goal_acc) this->resize(3); // The boundaries are copied from class members
            else this->resize(4); // Set size number of attached vertices to 4 (sample i, sample i+1, sample i+2 and dt)
        }
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
	    double vel1, vel2, omega1, omega2, dt;
	    
	    if (start_acc==false && goal_acc==false)
	    {
		TebSample<3,0>* sample_i = static_cast<TebSample<3,0>*>(_vertices[0]);
		TebSample<3,0>* sample_ip1 = static_cast<TebSample<3,0>*>(_vertices[1]);
		TebSample<3,0>* sample_ip2 = static_cast<TebSample<3,0>*>(_vertices[2]);
		TimeDiff* dt_vertex = static_cast<TimeDiff*>(_vertices[3]);
		
		// Direction vector between each two consecutive samples:
		Eigen::Vector2d deltaS1 = sample_ip1->states().head(2) - sample_i->states().head(2);
		Eigen::Vector2d deltaS2 = sample_ip2->states().head(2) - sample_ip1->states().head(2);
		

		dt = dt_vertex->dt();
		vel1 = deltaS1.norm() / dt;
		vel2 = deltaS2.norm() / dt;
		omega1 = norm_angle( sample_ip1->states()[2] - sample_i->states()[2] ) / dt;
		omega2 = norm_angle( sample_ip2->states()[2] - sample_ip1->states()[2] ) / dt;
	    }
	    else if (start_acc==true)
	    {
		TebSample<3,0>* sample_ip1 = static_cast<TebSample<3,0>*>(_vertices[0]);
		TebSample<3,0>* sample_ip2 = static_cast<TebSample<3,0>*>(_vertices[1]);
		TimeDiff* dt_vertex = static_cast<TimeDiff*>(_vertices[2]);
		
		// Direction vector between each two consecutive samples:
		Eigen::Vector2d deltaS2 = sample_ip2->states().head(2) - sample_ip1->states().head(2);
		
		dt = dt_vertex->dt();
		vel1 = v_start;
		vel2 = deltaS2.norm() / dt;
		omega1 = omega_start;
		omega2 = norm_angle( sample_ip2->states()[2] - sample_ip1->states()[2] ) / dt;      
	    }
	    else if (goal_acc==true)
	    {
		TebSample<3,0>* sample_i = static_cast<TebSample<3,0>*>(_vertices[0]);
		TebSample<3,0>* sample_ip1 = static_cast<TebSample<3,0>*>(_vertices[1]);
		TimeDiff* dt_vertex = static_cast<TimeDiff*>(_vertices[2]);
		
		// Direction vector between each two consecutive samples:
		Eigen::Vector2d deltaS1 = sample_ip1->states().head(2) - sample_i->states().head(2);

		dt = dt_vertex->dt();
		vel1 = deltaS1.norm() / dt;
		vel2 = v_goal;
		omega1 = norm_angle( sample_ip1->states()[2] - sample_i->states()[2] ) / dt;
		omega2 = omega_goal; 
	    }
	    
	    
	    double acc_trans = (vel2 - vel1) / dt;
	    double acc_rot = (omega2 - omega1) / dt;
            
            // Acceleration limits: c(x) < 0
            _values[0] = acc_trans - a_max;
            _values[1] = -acc_trans + a_min;
            _values[2] = acc_rot - omegadot_max;
            _values[3] = -acc_rot + omegadot_min;   
        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
        
        double a_max = 0.5;
        double a_min = -0.5;
        double omegadot_max = 0.5;
        double omegadot_min = -0.5;
	
	// Boundary values (select mode via template arguments)
	double v_start = 0;
	double omega_start = 0;
	double v_goal = 0;
	double omega_goal = 0;
	

	        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };        
    


} // end namespace teb



#endif /* defined(__teb_package__examples_rob_cost_edges_optional__) */