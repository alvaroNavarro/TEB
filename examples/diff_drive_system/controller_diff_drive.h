#ifndef __teb_package__examples_controller_diff_drive__
#define __teb_package__examples_controller_diff_drive__


#include <teb_package.h>
#include "cost_edges_diff_drive.h"
#include "differential_drive_system.h"

namespace teb	
{
        
    // Controller implementation
    template <int p, int q>
    class RobControllerDiffDrive : public TebController<p,q>
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
	
	Obstacle* getObstacle() { return(_obstacles.back()); }		
	
	virtual void setObstacleContainer(const ObstacleContainer& obs_container)
	{ 
	  for(unsigned int i=0; i<obs_container.size(); ++i)
	     addObstacle(obs_container.at(i));  	  	  
	}		

	void setSystem(DifferentialDriveRobotSystem<p,q>* robot)
	{
	  _system = robot; 
	}
		
	virtual unsigned int getNumberObstaclesAdded() {return(_obstacles.size());}
	virtual ObstacleContainer& getObstacles() {return(_obstacles);}
        
        // Overload customOptimizationGraph() to define a custom hyper-graph
        // and thus preserve the option to add predifined edges like time-optimality
         virtual void customOptimizationGraph()
         {                          
             
           for (unsigned int i=0; i<getN()-1; ++i)
            {  
	   		 
	          for (/*const Obstacle& obst : _obstacles*/ unsigned int j=0; j<_obstacles.size(); ++j)
		  {
		      EdgeObstacle<p, q>* obst_edge = new EdgeObstacle<p, q>(_state_seq.at(i));
		      obst_edge->setObstacle(_obstacles.at(j));
		      obst_edge->setCar(_system);
		      
		      _graph.addEdgeInequality(obst_edge);
		  }
	    }     
	 }
	
    protected:            
         
	 
	 ObstacleContainer _obstacles;	 
	 DifferentialDriveRobotSystem<p, q>* _system = nullptr;
	 
	 using TebController<p, q>::_graph;
	 using TebController<p, q>::_state_seq;
	 using TebController<p, q>::_ctrl_seq;
	 using TebController<p, q>::_dt;	 	 
        
    };                                        

} // end namespace teb



#endif /* defined(__teb_package__examples_controller_diff_drive__) */
