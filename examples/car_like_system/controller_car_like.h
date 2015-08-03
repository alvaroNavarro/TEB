#ifndef __teb_package__examples_controller_car_like__
#define __teb_package__examples_controller_car_like__


#include <teb_package.h>
#include "cost_edges_car_like.h"

namespace teb	
{
        
    // Controller implementation
    template <int p, int q>
    class RobControllerCarLike : public TebController<p,q>
    {
        
    public:            
      
        using ObstacleContainer = std::vector<Obstacle*>;
	using TebController<p, q>::getN;	      

#ifdef WIN32 // VS2013 does not support constructor inheritance (only starting from Nov 2013 CTP)																																																																																																																																																																																																																																																																																																	
		RobControllerCarLike(BaseSolver* solver = nullptr) : TebController(solver) {};
		RobControllerCarLike(const Config* config, BaseSolver* solver = nullptr) : TebController(config, solver) {};
#else
        using TebController<p,q>::TebController; // inherite base constructors
#endif

        virtual void addObstacle(Obstacle* obstacle) 
	{
		_obstacles.push_back(obstacle);
	}
	
	virtual void removeObstacle() 
	{
	  //delete _obstacles.front();
	  _obstacles.pop_back(); 	  
	}
	
	virtual bool isObstacleContainerEmpty() { return(_obstacles.empty());}
	
	virtual Obstacle* getObstacle() { return(_obstacles.back()); }		
	
	virtual void setObstacleContainer(const ObstacleContainer& obs_container)
	{ 
	  for(unsigned int i=0; i<obs_container.size(); ++i)
	     addObstacle(obs_container.at(i));  	  	  
	}		

	void setSystem(MobileRobotCarLikeSystem<p,q>* robot)
	{
	  _system = robot; 
	}
	
	void setWeightYawAngle(double weight_YawAngle) { _weight_YawAngle = weight_YawAngle; }
	void setWeightObstacle(double weight_Obstacle) { _weight_Obstacle = weight_Obstacle; }
	
	virtual unsigned int getNumberObstaclesAdded() {return(_obstacles.size());}
	virtual ObstacleContainer& getObstacles() {return(_obstacles);}
        
        // Overload customOptimizationGraph() to define a custom hyper-graph
        // and thus preserve the option to add predifined edges like time-optimality
         virtual void customOptimizationGraph()
         {                          
             
           for (unsigned int i=0; i<getN()-1; ++i)
            {  
	   
		  //Add the yaw angle..
 	          EdgeYawAngle<p>* yaw_angle_edge = new EdgeYawAngle<p>(_state_seq.at(i));
 		  yaw_angle_edge->setEdgeWeight(_weight_YawAngle);
 		 _graph.addEdgeInequality(yaw_angle_edge);
		 
		 //Add the edge for rate steering angle
 		 EdgeRateSteeringWheelAngle<q>* delta_dot_edge = new EdgeRateSteeringWheelAngle<q>(_ctrl_seq.at(i), _ctrl_seq.at(i + 1), _dt);
 		 delta_dot_edge->setEdgeWeight(_weight_DeltaDot);
 		_graph.addEdgeInequality(delta_dot_edge);
		
		//! Add edge for linear velocity
		EdgeLimitVelocity<p>* velocity_edge = new EdgeLimitVelocity<p>(_state_seq.at(i), _state_seq.at(i+1), _dt);
		_graph.addEdgeInequality(velocity_edge);
	      
	      
	          for (/*const Obstacle& obst : _obstacles*/ unsigned int j=0; j<_obstacles.size(); ++j)
		  {
		      EdgeObstacle<p, q>* obst_edge = new EdgeObstacle<p, q>(_state_seq.at(i));
		      obst_edge->setObstacle(_obstacles.at(j));
		      obst_edge->setCar(_system);
		      
		      _graph.addEdgeInequality(obst_edge);
		  }
	    }     
	 }
	 
	 //! This function updates the obstacle velocity....
	 void updateObstaclePosition(BaseController* ctrl, SystemDynamics<p,q>* system, Simulator<p,q>* sim)
	 {
	     static unsigned int index = 0;
	   
	     if(!isObstacleContainerEmpty()){
	        
	       Eigen::Vector2d obs_curr = _obstacles.back()->getObstaclePosition("current");
	       
	       if((obs_curr - _obstacles.back()->getObstaclePosition("final")).norm() < 0.1){
		   obs_curr = _obstacles.back()->getObstaclePosition("final");
		   index = 0;
	       }
	       
	       else{		   
		  _obstacles.back()->setObstaclePosition(++index, "current");   
	       }	       	     	             
	     }	   	     	     	     
	 }
	 	 	
    protected:            
         
	 
	 ObstacleContainer _obstacles;	 
	 MobileRobotCarLikeSystem<p, q>* _system = nullptr;	 	 
	 
	 using TebController<p, q>::_graph;
	 using TebController<p, q>::_state_seq;
	 using TebController<p, q>::_ctrl_seq;
	 using TebController<p, q>::_dt;
	 
	 double _weight_YawAngle = 1;
	 double _weight_Obstacle = 1;
	 double _weight_DeltaDot = 1;	 	 
        
    };                                        

} // end namespace teb



#endif /* defined(__teb_package__examples_controller_car_like__) */
