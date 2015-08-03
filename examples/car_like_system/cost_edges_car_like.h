#ifndef __teb_package__examples_cost_edges_car_like__
#define __teb_package__examples_cost_edges_car_like__


#include <teb_package.h>
#include "car_like_system.h"


namespace teb
{
    
    // Declarations for cost functions / hyper edges   
    
    //! Edge for the steering angle...
    template <int p>
    class EdgeYawAngle: public BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>>
    {
    protected:
	using BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>>::_vertices;
	using BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>>::_values;

    public:
	EdgeYawAngle(StateVertex<p> & state) : BaseEdge<2, FUNCT_TYPE::NONLINEAR, StateVertex<p>>(state) {}

	virtual void computeValues()
	{
		StateVertex<p>* sample_i = static_cast<StateVertex<p>*>(_vertices[0]);

		double phi = sample_i->states()[3];

		_values[0] = -phi + phi_min;
		_values[1] = phi - phi_max;
	}

    private:
 	double phi_max = PI/4;    //! 45 degree..
	double phi_min = -PI/4;
   };
   
   //! Edge for the rate steering angle..
   template <int q>
   class EdgeRateSteeringWheelAngle : public BaseEdge<2, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>
   {
   protected:
	using BaseEdge<2, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>::_vertices;
	using BaseEdge<2, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>::_values;

   public:
	EdgeRateSteeringWheelAngle(ControlVertex<q> & control1, ControlVertex<q> & control2, TimeDiff& dt) : BaseEdge<2, FUNCT_TYPE::NONLINEAR, ControlVertex<q>, ControlVertex<q>, TimeDiff>(control1, control2, dt) {}

	virtual void computeValues()
	{
		ControlVertex<q>* sample_i = static_cast<ControlVertex<q>*>(_vertices[0]);
		ControlVertex<q>* sample_ip1 = static_cast<ControlVertex<q>*>(_vertices[1]);
		TimeDiff* dt = static_cast<TimeDiff*>(_vertices[2]);

		double omega_phi = norm_angle(sample_ip1->controls()[1] - sample_i->controls()[1]) / dt->dt();

		_values[0] = omega_phi - omega_phi_max;
		_values[1] = -omega_phi + omega_phi_min; 
	}
	
    private:
      double omega_phi_max = 0.5;
      double omega_phi_min = -0.5;
   };
    
    
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
       MobileRobotCarLikeSystem<p,q>* _robot = nullptr;
    
    public:
            
        
        //! Empty Constructor
	EdgeObstacle(StateVertex<p>& state) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<p>>(state) {}
	
	void setObstacle(Obstacle *obs)
	{
		_obstacle = obs;
	}
	
	void setCar(MobileRobotCarLikeSystem<p, q>* robot)
	{
	  _robot = robot; 
	}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
	    StateVertex<p>* sample_i = static_cast<StateVertex<p>*>(_vertices[0]); 	    	    
            
            // Direction vector between sample and obstacle:
            Eigen::Vector2d deltaS = sample_i->states().head(2) - _obstacle->getObstaclePosition("current");
            
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


#endif /* defined(__teb_package__examples_cost_edges_car_like__) */
