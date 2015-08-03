#ifndef __teb_package__examples_rob_controller_car_like__
#define __teb_package__examples_rob_controller_car_like__


#include <teb_package.h>
#include "rob_cost_edges_car_like.h"
#include "rob_cost_edges_obstacles.h"


namespace teb
{
        
    // Controller implementation
    class RobControllerCarLike : public TebController<4,0>
    {
        
    public:

#ifdef WIN32 // VS2013 does not support constructor inheritance (only starting from Nov 2013 CTP)
		RobControllerCarLike(BaseSolver* solver = nullptr) : TebController(solver) {};
		RobControllerCarLike(const Config* config, BaseSolver* solver = nullptr) : TebController(config, solver) {};
#else
        using TebController<4,0>::TebController; // inherite base constructors
#endif
        
        // Overload customOptimizationGraph() to define a custom hyper-graph
        // and thus preserve the option to add predifined edges like time-optimality
        virtual void customOptimizationGraph()
        {
            
            for (unsigned int i=0; i<getN()-1; ++i)
            {
                // Add non-holonomic kinematics
		EdgeKinematicsCarLike* kinematics_edge = new EdgeKinematicsCarLike(_state_seq.at(i), _state_seq.at(i + 1));
                _graph.addEdgeEquality(kinematics_edge);
                
                // Add velocity limits (translational and rotational)
		EdgeLimitVelocityCarLike* vel_edge = new EdgeLimitVelocityCarLike(_state_seq.at(i), _state_seq.at(i + 1), _dt);
                _graph.addEdgeInequality(vel_edge);
				
                // Add single obstacle edge
		EdgeObstacle* obst_edge = new EdgeObstacle(_state_seq.at(i));
                obst_edge->min_distance = 0.5;
                obst_edge->obstacle_position = _obstacle_pos;
                _graph.addEdgeInequality(obst_edge);
            } 
            
        
        };
	
	 void setObstaclePosition(const Eigen::Ref<const Eigen::Vector2d>& obstacle_pos) {_obstacle_pos = obstacle_pos;}; 
	 void setObstaclePosition(const double* obstacle_pos) {_obstacle_pos = Eigen::Vector2d(obstacle_pos);}; 
	 void setCurrentVelocity(const Eigen::Ref<const Eigen::Vector2d>& curr_velocity) {_curr_velocity = curr_velocity;}; 
	 void setCurrentVelocity(double v, double omega) {_curr_velocity[0]=v; _curr_velocity[1]=omega;}; 
	 
	 // Get all velocities (implicit from states)
	 // This method is not efficient, but it is only used for visualization here.
	 // Returns a [2 x n-1] matrix in which each column denotes: [sqrt(vx^2+vy^2), omega]^T
	 Eigen::MatrixXd getVelocities()
	 {
	      unsigned int n = getN(); // Number of states
	      Eigen::MatrixXd teb_mat = getStateCtrlInfoMat(); // inefficient here, but comfortable
	      Eigen::Matrix<double,3,-1> vel3 = (teb_mat.topRightCorner(3,n-1) - teb_mat.topLeftCorner(3,n-1));
	      // Normalize angle differnece
	      norm_angle_vec(vel3.bottomRows(0));
	      vel3 /= dt().dt(); // scale to actual velocities 
	      
	      Eigen::MatrixXd vel2(2,vel3.cols());
	      vel2.row(0) = vel3.topRows(2).colwise().norm();
	      vel2.row(1) = vel3.row(2);
	      return vel2;
	 }
        

     // Override original function to return the current control input, since
     // we use an implicit control input model (as a function of the states)
     Eigen::VectorXd firstControl() const
     {
         Eigen::VectorXd u(2,1);
		 u[0] = (_state_seq.at(1).states().head(2) - _state_seq.at(0).states().head(2)).norm() / dt().dt(); // transl. velocity
		 u[1] = norm_angle((_state_seq.at(1).states()[2] - _state_seq.at(0).states()[2])) / dt().dt(); // rot. velocity
	 
	 // Check if Goal reached:
	 if ( (firstStateRef() - lastStateRef() ).norm() < 0.1)
	 {
	   u.setZero();
	   PRINT_INFO_ONCE("RobController: Goal Reached.");
	 }

         return u;
     }
        
     // Override this function as well to get all implicit controls at once
     virtual Eigen::MatrixXd returnControlInputSequence() const
     {
         Eigen::MatrixXd states = getStateCtrlInfoMat();
         
         unsigned int m = getN()-1;
         Eigen::MatrixXd u_seq(2,m);
        
         u_seq.row(0) = ( states.topRightCorner(2,m) - states.topLeftCorner(2,m) ).colwise().norm() / getDt();
         u_seq.row(1) = states.bottomRightCorner(1, m) - states.bottomLeftCorner(1, m);
         norm_angle_vec(u_seq.bottomRows(0));
         u_seq.row(1) /= getDt();
         
         return u_seq;
     }
	
      // Update current velocity after each step to initialize the subsequent step
      void robSaveVelocityAfterStep(BaseController* ctrl, SystemDynamics<3,2>* system, Simulator<3,2>* sim)
      {
	setCurrentVelocity(firstControl());
      }
	
	
	
      // We need some additional visualization
      void plotRobotStuff()
      {
	// Plot states and control input
	TebPlotter plotter;
	plotter.plotTEB(*this);

	// Create x-y plot
	plotter.switchWindow(1,true); // Create new window
	Eigen::MatrixXd teb_mat = getStateCtrlInfoMat(); // Get all TEB states in a single matrix
	plotter.plot(teb_mat.row(0).transpose(), teb_mat.row(1).transpose(),"X-Y Trajectory","x [m]", "y [m]");

	// Get velocity from the states
	Eigen::MatrixXd vel = getVelocities();
	
	// plot resulting translational velocity
	plotter.switchWindow(2,true);
	
	PlotOptions options;
	options.title = "Robot velocities";
	options.ylabels.emplace_back("trans. vel [m/s]");
	options.ylabels.emplace_back("rot. vel [rad/s]");

	plotter.plotMulti(getAbsoluteTimeVec().head(getN()-1), vel, &options); 
      }
	
    protected:
      Eigen::Vector2d _curr_velocity = Eigen::Vector2d::Zero(); // [v, omega]^T
      Eigen::Vector2d _obstacle_pos = Eigen::Vector2d::Zero();
        
    };
    
    
    
    // Velocity bounds

    
    
        
        


} // end namespace teb



#endif /* defined(__teb_package__examples_rob_controller_car_like__) */
