#ifndef __teb_package__examples_cost_edges_diff_drive__
#define __teb_package__examples_cost_edges_diff_drive__


#include <teb_package.h>
#include "differential_drive_system.h"


namespace teb
{
    
    // Declarations for cost functions / hyper edges           
   
   //! Edge for the rate steering angle..
    
    //! Edge for the linear velocity
    // Limit Velocity
    template <int p>
    class EdgeLimitVelocity : public BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, TimeDiff> // Cost vector dimension: 4, Vertices: StateVertex, StateVertex, TimeDiff
    {
	protected:
		using BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, TimeDiff>::_vertices;
		using BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, TimeDiff>::_values;


       public:
        
        //! Empty Constructor
	EdgeLimitVelocity(StateVertex<p>& state1, StateVertex<p>& state2, TimeDiff& dt) : BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>, StateVertex<p>, TimeDiff>(state1, state2, dt)
        {
        }
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
		StateVertex<p>* sample_i = static_cast<StateVertex<p>*>(_vertices[0]);
		StateVertex<p>* sample_ip1 = static_cast<StateVertex<p>*>(_vertices[1]);
                TimeDiff* dt = static_cast<TimeDiff*>(_vertices[2]);
            
                // Direction vector between two samples:		
		Eigen::Vector2d deltaS = sample_i->states().head(2) - sample_ip1->states().head(2);

                double vel = deltaS.norm() / dt->dt();
            
                // Velocity limits: c(x) < 0
                //_values[0] = vel - v_max;
                _values[0] = -vel + v_min;
	        _values[1] = vel - v_max;

        }
        
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
        
        double v_max = 1;
        double v_min = -0.5;

        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
   
    // Obstacle-Avoidance (point-representation)
    template <int p, int q>
    class EdgeObstacle : public BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<p>> // Cost vector dimension: 1, Vertices: StateVertex
    {
     
    protected:               

      using BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<p>>::_vertices;
      using BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<p>>::_values;

     
     private:
      
       Obstacle* _obstacle = nullptr;
       DifferentialDriveRobotSystem<p,q>* _robot = nullptr;
    
    public:
            
        
        //! Empty Constructor
	EdgeObstacle(StateVertex<p>& state) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<p>>(state) {}
	
	void setObstacle(Obstacle *obs)
	{
		_obstacle = obs;
	}
	
	void setCar(DifferentialDriveRobotSystem<p, q>* robot)
	{
	  _robot = robot; 
	}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
	    StateVertex<p>* sample_i = static_cast<StateVertex<p>*>(_vertices[0]); 	    	    
            
            // Direction vector between sample and obstacle:
            Eigen::Vector2d deltaS = sample_i->states().head(2) - _obstacle->getObstaclePosition();
            
	    double proj_dist = deltaS.norm();	    	    	    
    
	    // Constraint proj_dist > min_distance -> -proj_dist + min_distance < 0
            _values[0] = -proj_dist + _obstacle->getRadiusObstacle()  + _robot->getEpsilonCostFunction() + min_distance;// 
	    
	   // PRINT_INFO("value: " << _values[0]);
        }
           
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
            
            
        //Eigen::Vector2d obstacle_position; // DO NOT FORGET TO SET VALUE
        double min_distance = 0.5;
            
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    
    };        

} // end namespace teb


#endif /* defined(__teb_package__examples_cost_edges_diff_drive__) */
