
#include "car_like_system.h"
#include "controller_car_like.h"
#include <fcntl.h>
#include <fstream>

int main()
{        
    double x0[4];
    double xf[4];    
    
    const int n_state   = 4;
    const int n_control = 2;
    
    std::fstream simulation_file_obs;
    std::string folder_simulation  = "/home/alvarin/Desktop/Simulations/";  
    std::string _fileNameObs  = "distance.dat"; 
    
    simulation_file_obs.open((folder_simulation+_fileNameObs).c_str(), std::ofstream::out);
    
    // test SQP solver
    teb::Config cfg;
    
    // optimization settings
    cfg.teb.teb_iter = 5;
    cfg.optim.solver.solver_iter = 5;
    cfg.optim.solver.lsq.weight_equalities = 200;
    cfg.optim.solver.lsq.weight_inequalities = 30;
    cfg.optim.solver.lsq.weight_adaptation_factor = 2;        
    
    // trajectory settings
    cfg.teb.n_pre = 50;
    cfg.teb.dt_ref = 0.1;
    cfg.teb.dt_hyst = cfg.teb.dt_ref/10;
    
    teb::InitialFinalStates states_values;
    
//  System specific settings
    MobileRobotCarLikeSystem<n_state, n_control> robotSystem;
    //robotSystem.setRMax();
    
    if(states_values.getInitialFinalState(x0, xf, 8) == false)
    {
       std::cout << "Invalid scenario..." << std::endl;
       return 0;
    }
    
    robotSystem.setTypeDriving(teb::TypeDriving::REAR_WHEEL);
    robotSystem.setRobotDimesion(0.9,0.35);    

    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::RobControllerCarLike<n_state, n_control> teb(&cfg, &solver);            
   
    //teb::SolverSQPLocalDense solver;                  
          
    teb.activateObjectiveTimeOptimal();        
    teb.setSystemDynamics(&robotSystem);    
    teb.activateControlBounds({-0.1,-0.5},{1,0.5});
    teb.setSystem(&robotSystem);
    
     teb.addObstacle(new teb::Obstacle(-1, 0.5, 2));
   /*  teb.addObstacle(new teb::Obstacle(-1,-1, 2));
     teb.addObstacle(new teb::Obstacle(-1, -2, 2));
     teb.addObstacle(new teb::Obstacle(1, 4, 2));
     teb.addObstacle(new teb::Obstacle(1, 0, 2));
     teb.addObstacle(new teb::Obstacle(1, -2, 2));     */
   
    teb::TebPlotter plotter;    
    plotter.setDistanceRobotPlot(1.0);
    
    teb::RobotFrame<n_state, n_control> robot_frame(robotSystem);                          
    teb::RobotPlot<n_state, n_control>  rplot(&plotter,&robot_frame, &teb);                
    teb::Simulator<n_state, n_control>  sim(&teb,robotSystem,&plotter);
    
    sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);    
    
    using namespace std::placeholders;
    sim.setPostStepCallback(std::bind(&teb::RobControllerCarLike<n_state, n_control>::updateObstaclePosition, &teb, _1, _2, _3));
    //sim.setPreStepCallback(std::bind(&teb::RobControllerCarLike<n_state, n_control>::addDistance, &teb, _1, _2, _3));
    
    //! Fixed the final position for the obstacle
    teb.getObstacle()->setObstaclePosition(-1, -4, "final_fixed");
    teb.getObstacle()->initializeObstaclePath();
    
   /* double x_obs_f = teb.getObstacle()->getObstacleFinalPosition()[0];
    double y_obs_f = teb.getObstacle()->getObstacleFinalPosition()[1]; */
        
    unsigned int idx_motion = 0;
    unsigned int window_id = 1;
    unsigned int point_to_print = (unsigned int)(teb.getObstacle()->getLengthPath() / 4);
    
     // Perform open-loop planning
	START_TIMER;
	teb.step(x0, xf);
	STOP_TIMER("TEB optimization loop");   
	
	PRINT_INFO("First Step: " << teb.getN());
	
    PRINT_INFO("total simulations: " << teb.getObstacle()->getLengthPath());
    
    //! Due to the obstacle will be moved according to the path defined (straight line), the final temporal obstacle position is set
    teb.getObstacle()->setObstaclePosition(-1, 0.5, "final");
    
    do{          
	
	PRINT_INFO("Obstacle Pos: " << teb.getObstacle()->getObstaclePosition("final"));
      
        //Eigen::MatrixXd teb_mat1 = teb.getStateCtrlInfoMat(); // Get all TEB states in a single matrix
	//PRINT_INFO("teb_mat_opt:\n" << teb_mat1);		
	    
	// Start simulation
	plotter.switchWindow(window_id,true); // Create new window
	std::unique_ptr<teb::SimResults> results = sim.simOpenAndClosedLoop(x0,xf,25.0);
	
	
	if(!(idx_motion % point_to_print))
	{
	   // Create x-y plot
	  plotter.switchWindow(window_id++,true); // Create new window	  
	  Eigen::Matrix2d mat_temp = Eigen::Matrix2d::Zero();
	  rplot.plotRobot(results->series.back(), mat_temp);  
	}
				
	//teb.printDistance();
	double dis = 0;
	unsigned int iter =0;
	  for(unsigned int j=0; j<results->series.back().states.cols()-2; ++j)
	  {
	    if((results->series.back().states.topRows(2).col(j) - results->series.back().states.topRows(2).col(j+1)).norm() > 0.1){
		dis += (results->series.back().states.topRows(2).col(j) - results->series.back().states.topRows(2).col(j+1)).norm();
		iter = j;
	    }
	    
	    simulation_file_obs << std::fixed << std::setprecision(2) << dis << " ";  
	  }
	  
	  simulation_file_obs << std::endl;    
	
	simulation_file_obs.close();
	
	PRINT_INFO("No of states: " << results->series.back().states.cols());
	PRINT_INFO("No of states_ini: " << results->series.at(0).states.cols());
	PRINT_INFO("Distance: " << dis);
	PRINT_INFO("iter: " << iter);
        PRINT_INFO("Time: " << (double)iter * teb.dt().dt());
    
	idx_motion++;
	
	//! Update the obstacle position..	
	
	if(idx_motion == teb.getObstacle()->getLengthPath())  continue;
	
	teb.getObstacle()->setObstaclePosition(idx_motion, "final");            
      
      PRINT_INFO("Stage " << idx_motion << " finished..");
      
      //! The obstacle starts again from its initial position...
      teb.getObstacle()->setObstaclePosition(-1, 0.5, "current");
      
    }while(idx_motion < teb.getObstacle()->getLengthPath());
                
    //std::cin.get();
    
    return 0;
}
