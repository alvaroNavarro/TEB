
#include "testSimulation.h"

#include <fcntl.h>
#include <string.h>
#include <iomanip>
#include <fcntl.h>
#include <stdio.h>
#include <limits>
#include <assert.h>
#include <fstream>

using namespace std;

namespace test_suite
{

template <int p, int q>
void Test<p,q>::insertObstacle()
{
  
  Eigen::Vector2d pos_obs;  
  double radius_obs;
  
  std::uniform_real_distribution<double> value_radius(0.1, 1.5);
  std::uniform_real_distribution<double> value_x_pos(_workspace.coeffRef(0), _workspace.coeffRef(1));
  std::uniform_real_distribution<double> value_y_pos(_workspace.coeffRef(2), _workspace.coeffRef(3));
  
  for(unsigned int i=0; i<_number_obstacles; ++i)
  {
      
      radius_obs  = value_radius(gen);            
      pos_obs.coeffRef(0) = value_x_pos(gen);      
      pos_obs.coeffRef(1) = value_y_pos(gen);
      
      //obs = new teb::Obstacle(pos_obs.coeffRef(0), pos_obs.coeffRef(1), radius_obs);
      _robController->addObstacle(new teb::Obstacle(pos_obs.coeffRef(0), pos_obs.coeffRef(1), radius_obs));   //! Added to obstacle container....
      
      //! It is important to check if the subsequent obstacle position is overlapped.. If this happens, a new obstacle muss be defined
      for(unsigned int j=0; j<_robController->getNumberObstaclesAdded(); ++j)
      {
	if(i == j) continue;  //! No compare with itself..
	
	if(isObstacleOverlapped(_robController->getObstacles().at(i), _robController->getObstacles().at(j)) == true)
	{	  
	  _robController->removeObstacle();
	  
	  //! New Obstacle is created... 	  
	  radius_obs  = value_radius(gen);	  	  
	  pos_obs.coeffRef(0) = value_x_pos(gen);	  	  
	  pos_obs.coeffRef(1) = value_y_pos(gen);
	  
	  _robController->addObstacle(new teb::Obstacle(pos_obs.coeffRef(0), pos_obs.coeffRef(1), radius_obs));
	  
	  j = 0;
	}
      }                  
  }  
  
}

template <int p, int q>
void Test<p,q>::removeAllObstacles()
{
   while(!_robController->isObstacleContainerEmpty()) 
      _robController->removeObstacle(); 
   
   //! Remove the cluster if exist..
   if(_cluster.getNumberCluster() != 0)
     _cluster.removeAllClusters();
}

template <int p, int q>
bool Test<p,q>::isObstacleOverlapped(teb::Obstacle* obs1, teb::Obstacle* obs2)
{
  double dis = (obs1->getObstaclePosition("current") - obs2->getObstaclePosition("current")).norm();  
  
  if((dis - obs1->getRadiusObstacle() - obs2->getRadiusObstacle()) < 0)  return true;
  else return false;								  								 
}

template <int p, int q>
typename Test<p,q>::StateVector Test<p,q>::getPredefinedState(unsigned num_state, std::string string)
{
   StateVector state = StateVector::Zero();
   
   assert(!string.compare("initial") && !string.compare("final"));
   
   switch(num_state)
   {
     case 1:
       
       if(!string.compare("initial")){
	  state.coeffRef(0) = 0;
          state.coeffRef(1) = 1;
	  state.coeffRef(2) = M_PI/2;
       }
       
       else{
	  state.coeffRef(0) = 2;
          state.coeffRef(1) = -3; 
	  state.coeffRef(2) = M_PI/2;
       }
              
     break;
     
     case 2:
     
       if(!string.compare("initial")){
	  state.coeffRef(0) = 5;
          state.coeffRef(1) = 0; 
       }
       
       else{
	  state.coeffRef(0) = 0;
          state.coeffRef(1) = -2; 
       }
       
     break;
   }
   
   return state;
}

template <int p, int q>
typename Test<p,q>::StateVector Test<p,q>::getRandomState(std::string string)
{
   StateVector state;      
   Eigen::Vector2d bound;
               
   //! State[0] -> X
   //! State[1] -> Y
   //! State[2] -> theta (car-like mobile robot) or State[2] -> phi (truck and trailer system)
   //! State[3] -> phi   (car-like mobile robor) or State[3] -> theta0 (truck and trailer) ... so forth
   
   if(string == "initial"){
      bound.coeffRef(0) = _workspace.coeffRef(0);
      bound.coeffRef(1) = _workspace.coeffRef(0) + 5;
   }
   
   else{
     bound.coeffRef(0) = _workspace.coeffRef(1) - 5;
     bound.coeffRef(1) = _workspace.coeffRef(1);
   }
   
   std::uniform_real_distribution<double> value_x_pos(bound.coeffRef(0), bound.coeffRef(1));
   std::uniform_real_distribution<double> value_y_pos(_workspace.coeffRef(2) - 5, _workspace.coeffRef(3) + 5);
   std::uniform_real_distribution<double> value_angle1(_theta_bound.coeffRef(0), _theta_bound.coeffRef(1));
   std::uniform_real_distribution<double> value_angle2(_phi_bound.coeffRef(0), _phi_bound.coeffRef(1));
   
   state.coeffRef(0) = value_x_pos(gen);
   state.coeffRef(1) = value_y_pos(gen);
      
   if(_robot->DimStates == 3)            
     state.coeffRef(2) = value_angle1(gen);     
   
   else if(_robot->DimStates == 4){
     state.coeffRef(2) = value_angle1(gen);
     //state.coeffRef(3) = value_angle2(generator);     
     state.coeffRef(3) = 0;
   }
   
   else{
     //state.coeffRef(2) = value_angle2(generator);
     state.coeffRef(2) = 0;
     state.coeffRef(3) = value_angle1(gen);     
   }
         
   //! The rest of states are completed in a loop
   for(unsigned int i = 4; i<_robot->DimStates; ++i)         
      state.coeffRef(i) = value_angle2(gen);   
     
   return state;
}

template <int p, int q>              
void Test<p,q>::createGeneralMatrix(const teb::SimResults::TimeSeries& ts)
{                  
      _state_control_time.conservativeResize(ts.states.rows()+ts.controls.rows()+1, ts.states.cols());
      _vector_obstacles.conservativeResize(ts.states.cols());
      _number_samples = ts.states.cols();
      
      _state_control_time.block(0,0,ts.states.rows(),ts.states.cols()) = ts.states;
      if ( ts.states.cols() == ts.controls.cols() )
      {
	_state_control_time.block(ts.states.rows(),0,ts.controls.rows(),ts.controls.cols()) = ts.controls;
      }
      
      else if ( ts.states.cols()-1 == ts.controls.cols() )
      {
	_state_control_time.block(ts.states.rows(),0,ts.controls.rows(),ts.controls.cols()) = ts.controls;
	_state_control_time.block(ts.states.rows(),ts.controls.cols(),ts.controls.rows(),1).setZero();      	
      }
      
      else
      {
	PRINT_INFO("plotRobot(): TimeSeries object must contain state and control sequences of similar size (n=m, or m=n-1)");	
      }
      
      if (ts.time.rows() != ts.states.cols())
      {
	PRINT_INFO("plotRobot(): TimeSeries object must contain state and time sequences of identical size (n=m)");      	
      }
      
      _state_control_time.bottomRows(1) = ts.time.transpose();       
 }
 
template <int p, int q>
void Test<p,q>::createFeasibilityVector(const teb::SimResults::TimeSeries& ts)
{            
  if(_state_control_time.rows() == 0 || _state_control_time.cols() == 0)
    return;
  
  _vector_obstacles.setZero();
   
   for(unsigned int i=0; i<_number_samples; ++i)
   {
      for(unsigned int j=0; j<_number_obstacles; ++j)
      {
	 if(isRobotOverlappedToObstacle(_state_control_time.col(i).head(2), _robController->getObstacles().at(j)) == true)	 
	    _vector_obstacles.coeffRef(j) += 1;	 
      }
   }  
   
   //! Compute the distance traversed by the robot according to the distance of each node..
   _dis = 0;
   unsigned int iter =0;
   for(unsigned int j=0; j<_number_samples - 1; ++j)
   {
     if((ts.states.topRows(2).col(j) - ts.states.topRows(2).col(j+1)).norm() > 0.1){
 	_dis += (ts.states.topRows(2).col(j) - ts.states.topRows(2).col(j+1)).norm();
	iter = j;
     }	    	    
   }
   
   //! Compute the time taken by the robot traverse from the start location to the target location
   _time = (double)iter * _robController->dt().dt();
}

template <int p, int q>
bool Test<p,q>::isRobotOverlappedToObstacle(const Eigen::Ref<const Eigen::Vector2d>& posRobot, teb::Obstacle* obstacle)
{
  
  if(obstacle->getRadiusObstacle() + obstacle->getMaximumRadius() - (posRobot - obstacle->getObstaclePosition("current")).norm() > 0)
    return true;
  
  else
    
    return false;
}

template <int p, int q>
bool Test<p,q>::isRobotOverlappedToAnyCluster(const Eigen::Ref<const Eigen::Vector2d>& posRobot)
{
   bool ret_value = false;       
  
   for(unsigned int i=0; i<_cluster.getNumberCluster(); ++i)
   {
      if(isRobotInsidePolygon(posRobot, _cluster.getCluster().at(i)->getCornerPolygon()) == true){
	ret_value = true;
        break;
      }
   }
    
   return ret_value; 
}

template <int p, int q>
bool Test<p,q>::isRobotInsidePolygon(const Eigen::Ref<const Eigen::Vector2d>& posRobot, const PointPolygon& point_polygon)
{
    bool ret_val = false;   
  
    Eigen::Vector2d p1 = point_polygon.at(0);   //! Lower left point
    Eigen::Vector2d p2 = point_polygon.at(2);   //! Upper right point
    
    Eigen::Matrix<bool,2,1> xy_in = Eigen::Matrix<bool,2,1>(false, false);
    
    if(posRobot.coeffRef(0) > p1.coeffRef(0) && posRobot.coeffRef(0) < p2.coeffRef(0))
      xy_in.coeffRef(0) = true;
    
    if(posRobot.coeffRef(1) > p1.coeffRef(1) && posRobot.coeffRef(1) < p2.coeffRef(1))
      xy_in.coeffRef(1) = true;
    
    if(xy_in.coeffRef(0) == true && xy_in.coeffRef(1) == true)
      ret_val = true;
    
    return ret_val;
}

template <int p, int q>
bool Test<p,q>::AreBothStatesOverlapped(const Eigen::Ref<const Eigen::Vector2d>& pos1, const Eigen::Ref<const Eigen::Vector2d>& pos2)
{
  double Threshold = 0.5;
  
  if((pos1 - pos2).norm() > Threshold)
    return false;
  
  else
    return true;
}

template <int p, int q>
bool Test<p,q>::isRobotOverlappedToAnyObstacle(const Eigen::Ref<const Eigen::Vector2d>& posRobot)
{
  bool overlapped = false;
  
   for(unsigned int i=0; i<_robController->getNumberObstaclesAdded();++i)
   {
     if(isRobotOverlappedToObstacle(posRobot,_robController->getObstacles().at(i)) == true)
     {
       overlapped = true;
       break;
     }
   }
   
   return overlapped;
}

template <int p, int q>
void Test<p,q>::saveInfo()
{
   saveStatesControlTimeMatrix();
   saveFeasibilityVector();   
}

template <int p, int q>
void Test<p,q>::saveStatesControlTimeMatrix()
{
   for(unsigned int i=0; i<_robot->DimStates + _robot->DimControls + 1; ++i)
    {
      for(unsigned int j=0; j<_number_samples; ++j)
        _simulation_file_info << std::fixed << std::setprecision(2) << _state_control_time.coeffRef(i,j) << " ";
      
      _simulation_file_info << std::fixed << std::setprecision(1) << _robot->getEpsilonCostFunction() << " ";
      
      if(_type_scenario == TypeScenario::DYNAMIC_OBSTACLE)
	_simulation_file_info  << _robController->getObstacle()->getLengthPath() << std::endl;
      
      else 
	_simulation_file_info << " ";
    } 
}

template <int p, int q>
void Test<p,q>::saveFeasibilityVector()
{
    Eigen::VectorXd initial_state(_number_obstacles); 
    Eigen::VectorXd final_state(_number_obstacles);
    
    initial_state.setZero();
    final_state.setZero();
    
    //! The initial and final state are copied into the vector...
    initial_state.head(2) = _initial_state.head(2);
    final_state.head(2)   = _final_state.head(2);
    
    for(unsigned int i=0; i<_number_obstacles; ++i)
      _simulation_file_obs << _vector_obstacles.coeffRef(i) << " ";
      
    _simulation_file_obs << std::fixed << std::setprecision(1) << _robot->getEpsilonCostFunction() << " " << std::fixed << std::setprecision(2) << _dis << " " << std::fixed << std::setprecision(2) << _time << std::endl;    
    
    //! If is the last simulation of the scene, the obstacle position is saved at the last two rows..
    if((_idx_epsilon == _number_steps_epsilon) && (_sim_idx == _number_simulations-1))
    {
      for(unsigned int i=0; i<2; ++i)
      {
	for(unsigned int j=0; j<_number_obstacles; ++j)
	      _simulation_file_obs << std::fixed << std::setprecision(1) << _robController->getObstacles().at(j)->getObstaclePosition("current")[i] << " ";  
	
	_simulation_file_obs << _scene << " " << std::fixed << std::setprecision(2) << _dis << " " << std::fixed << std::setprecision(2) << _time << std::endl;
      }
      
      //! Save the initial and final state generated by the random generator...            
      for(unsigned int j=0; j< _number_obstacles; ++j)
	 _simulation_file_obs << std::fixed << std::setprecision(1) << initial_state.coeffRef(j) << " ";
	  
      _simulation_file_obs << _scene << std::fixed << std::setprecision(2) << _dis << " " << std::fixed << std::setprecision(2) << _time << std::endl;
      
      for(unsigned int j=0; j< _number_obstacles; ++j)
	 _simulation_file_obs << std::fixed << std::setprecision(1) << final_state.coeffRef(j) << " ";
	  
      _simulation_file_obs << _scene << std::fixed << std::setprecision(2) << _dis << " " << std::fixed << std::setprecision(2) << _time << std::endl;     
    }    
}

template <int p, int q>
void Test<p,q>::saveObstaclePositionMatrix()
{
  for(unsigned int i=0; i<2; ++i)
  {
     for(unsigned int j=0; j<_number_obstacles; ++j)
          _simulation_file_obs_pos << std::fixed << std::setprecision(1) << _robController->getObstacles().at(j)->getObstaclePosition()[i] << " ";  
     
     _simulation_file_obs_pos << _scene <<  std::endl;
  }
  
  
}


template <int p, int q>
void Test<p,q>::menu()
{
    unsigned int number_simulations;
    unsigned int number_obstacles;
    unsigned int number_scene;
    
    struct stat myStat;            
    unsigned int input_scenario;    
          
    PRINT_INFO("Welcome to the Test Suite......\n");
    
    //!Menu to choose if the user wants to simulate the scenarios or if wants check the results...
    unsigned int op_stage;
    unsigned int op;
    unsigned int amount_scenarios;
    Eigen::Matrix2d initial_final_state;
    
    PRINT_INFO("1: Simulate \n");
    PRINT_INFO("2: Get Results \n");
    PRINT_INFO("3: Exit\n");
    PRINT_INFO("Please enter the option: ");    
    INPUT_STREAM(op_stage,-1);
    
    if(op_stage >= 3)
    {
       PRINT_INFO("Good luck ....");
       std::cin.get();
       return;
    }
    
    switch(op_stage)
    {
      
      //! Option to simulate depending on the type of scenario the user chooses 
      case 1:
 	
	  PRINT_INFO("Option to simulate has been chosen...\n");
	  
	  //! If the folder do not exist... it must be created..
	  if (!((stat((_pws->pw_dir + _folder_simulation).c_str(), &myStat) == 0) && (((myStat.st_mode) & S_IFMT) == S_IFDIR)))
              mkdir((_pws->pw_dir + _folder_simulation).c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	 
	  PRINT_INFO("The type of Scenarios are: \n 1 -> Single Obstacles \n 2 -> Scenarios \n 3 -> Car Parking \n 4 -> Dynamic Obstacle \n");
	  PRINT_INFO("Please enter the number of scenario: ");
	  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          INPUT_STREAM(input_scenario,-1);	  	  
	  
	  if(input_scenario > 4)
	  {
	    PRINT_INFO("The input string is invalid....");
	    break;
	  }
	  
	  switch(input_scenario)
	  {
	    case  1:  //! Singles Obstacles
	    
	        setTypeScenario(test_suite::TypeScenario::SINGLE_OBSTACLES);
	    
		PRINT_INFO("Please enter number of obstacles: ");
		INPUT_STREAM(number_obstacles,-1);	    
		
		if(number_obstacles > _max_number_obs)
		    number_obstacles = _max_number_obs;
		
		setNumberObstacles(number_obstacles);
	      
	    break;
	     
	    case 2:	 //! Scenario     	        
		
		setTypeScenario(test_suite::TypeScenario::SCENARIO);
		
		do{
		  
		  PRINT_INFO("Do you want to simulate all the scenarios? \n 1: YES \n 0: NO \n");
		  PRINT_INFO("Please enter the option: ");
		  INPUT_STREAM(amount_scenarios,-1);	      
		  
		  if(amount_scenarios > 1)  PRINT_INFO("Option invalid... Try again :-)");
		  
		}while(amount_scenarios > 1);       
		
		if(amount_scenarios == 0)
		{
		    PRINT_INFO("Please enter number of scene ");
		    INPUT_STREAM(number_scene,-1);		
		    setNumberScene(number_scene);	  	  
		}
		
		setVerboseOption(amount_scenarios);
			
		
		PRINT_INFO("The options to plot the obstacles are: \n 1: Plot only the obstacles \n 2: Plot both the obstacle and the border \n 3: Plot only the border \n");
		PRINT_INFO("Enter option: ");
		INPUT_STREAM(op,-1);	    
		
		if(op > 3){
		    PRINT_INFO("Option invalid.... By default the option chosen will be 1");
		    op = 1;
		}
		
		setOptionPlotObstacle(op);	       
	      
	    break;
	    
	    case 3:	  //! Car Parking        
	       
	        setTypeScenario(test_suite::TypeScenario::CAR_PARKING);
	       
	        PRINT_INFO("The options to plot the obstacles are: \n 1: Plot only the obstacles \n 2: Plot both the obstacle and the border \n 3: Plot only the border \n");
		PRINT_INFO("Enter option: ");
		INPUT_STREAM(op,-1);	    
		
		if(op > 3){
		    PRINT_INFO("Option invalid.... By default the option chosen will be 1");
		    op = 1;
		}
		
		setOptionPlotObstacle(op);	
		
		setVerboseOption(0);
	       
	    break;
	    
	    case 4:   //! Simulation for dynamic obstacle
	      
	        double xi;
		double yi;
		double xf;
		double yf;
	        
	        setTypeScenario(test_suite::TypeScenario::DYNAMIC_OBSTACLE);
		
		PRINT_INFO("The scenario is <Scene 1> with one obstacle...\n");
		PRINT_INFO("Please enter the initial and final position by considering that the dimension of the map is\n");
		PRINT_INFO("x->[" << _workspace[0] << ", " << _workspace[1] << "] and y->[" << _workspace[2] << ", " << _workspace[3] << "]\n");
		
		PRINT_INFO("Enter Initial position : ");
		INPUT_STREAM(xi, -1);
		INPUT_STREAM(yi, -1);
		PRINT_INFO("Enter Final position : ");
		INPUT_STREAM(xf, -1);
		INPUT_STREAM(yf, -1);
		
		if((xi < _workspace[0] && xi > _workspace[1]) || (xf < _workspace[0] && xf > _workspace[1]) ||
		   (yi < _workspace[2] && yi > _workspace[3]) || (yf < _workspace[2] && yf > _workspace[3]))
		   {
		      PRINT_INFO("Either the initial position or the final position are invalid....");
		      break;
		   }
		   
		unsigned int op_vel;   
		PRINT_INFO("Set velocity: 1-> Manual \n 2-> By deafult of 1 m/s");   
		PRINT_INFO("Please enter a option: ");
		INPUT_STREAM(op_vel, -1);
		
		if(op_vel > 2)
		{
		   PRINT_INFO("Invalid option..."); 
		   break;
		}				 		
		
		Eigen::Vector2d obs_pos_ini   = Eigen::Vector2d(xi, yi);
		Eigen::Vector2d obs_pos_final = Eigen::Vector2d(xf, yf); 
		
		//using namespace std::placeholders;
                //_sim->setPostStepCallback(std::bind(&teb::TebController<p, q>::updateObstaclePosition, _robController, _1, _2, _3));
	
		initialize();
		setParameters();
		runSimulationDynamicObstacle(obs_pos_ini, obs_pos_final, op_vel);
	      
	    break;
	  }
	  	  
	  
	  if(input_scenario != 4)
	  {
	      //PRINT_INFO("Epsilon will vary from 0 to 0.9\n");
	      PRINT_INFO("Please enter number of simulations: ");
	      INPUT_STREAM(number_simulations,-1);	  
	      setNumberSimulations(number_simulations);
	      
	      //! Everything is set and as result we are ready to start the simulation....    
	      //! Do not forget to initialize...
	      initialize();
	      run();     //! Execute the complete simulation...
	  }   
	  	  
	 
       break;
       
       
       case 2:
	 
	 //! Option to get and plot the results
	 //! First it is important if there are data saved....	 	 
	 
	 PRINT_INFO("The option to check the results has been chosen.. \n");
	 if (!((stat((_pws->pw_dir + _folder_simulation).c_str(), &myStat) == 0) && (((myStat.st_mode) & S_IFMT) == S_IFDIR)))
	 {
	    PRINT_INFO("Sorry ... The main folder has not been created before..."); 
	    break;
	 }
	 
	 unsigned int op_revision;
	 PRINT_INFO("Please choice the type of environment \n 1-> Scenario \n 2-> Dynamic Obstacle \n");
	 PRINT_INFO("Enter the option: ");
	 INPUT_STREAM(op_revision, -1);
	 
	 if(op_revision > 2)
	 {
	    PRINT_INFO("Invalid Option...");
	    break;
	 }
	 	       	     
	 if(op_revision == 1)  //! Case to check the results of scenarios simulations
	 {
	     
	      PRINT_INFO("Please enter the scene Id: 1 - " << _cluster._max_scenes << " \n");
	      PRINT_INFO("Scene Id: ");	 
	      INPUT_STREAM(_scene,-1);
			      
	      unsigned int op;
	      
	      PRINT_INFO("The options to plot the obstacles are: \n 1: Plot only the obstacles \n 2: Plot both the obstacle and the border \n 3: Plot only the border \n");
	      PRINT_INFO("Enter option: ");
	      INPUT_STREAM(op,-1);	      
		  
	      if(op > 3){
		  PRINT_INFO("Option invalid.... By default the option chosen will be 1");
		  op = 1;
	      }
		  
	      setOptionPlotObstacle(op);		      
	      
	      Eigen::MatrixXd data_only_obs;	 	 	 	 	 	 
	      unsigned int number_rows_data_info = _robot->DimStates + _robot->DimControls + 1;
	      
		//Getting the information from the text file --- First The obstacles and then the states control and time
		//-----------------------------------------------------------------
	      
	      Eigen::MatrixXd data_obs_complete  = getInfoFromText("obs");
	      Eigen::MatrixXd data_info          = getInfoFromText("info");	 	 	 
	      
	      data_only_obs.conservativeResize(data_obs_complete.rows()-2, data_obs_complete.cols()-1);	 
	      
	      if(data_obs_complete.rows() == 0 || data_info.rows() == 0)
	      {
		PRINT_INFO("The file does not exist!!!!");
		break;
	      }
	      
	      //! Invoke prepareScene just to create the obstacle framework to plot the results... 
	      //! It is not effcient to do it because we are allocating memory for each obstacle which will not be used but only to get the 
	      //! coordinates and to plot over trajectory... At the end, the memory will be deleted..
	      _rob_plotter->setCluster(&_cluster);
	      prepareScene();
		      
	      Eigen::MatrixXd data_info_best_sim(number_rows_data_info, data_info.cols());
		      
	      //! The last two rows contain the obstacle position and the last column contains the epsilon value given by the cost function
	      data_only_obs = data_obs_complete.topLeftCorner(data_obs_complete.rows()-4,data_obs_complete.cols()-1);	 	 	 
	      unsigned int index = getBestSimulation(data_only_obs);           //!Get the index of the best simulation..
	      
	      PRINT_INFO("The minimum violations to the obstacles in the cost function was with eps : " << data_obs_complete.coeffRef(index, data_obs_complete.cols()-1));
			      
	      data_info_best_sim = data_info.block(number_rows_data_info * index, 0, number_rows_data_info,  data_info.cols()-1);	 
	      
	      PRINT_INFO("Initial state: " << data_info_best_sim.col(0).head(_robot->DimStates).transpose());
	      PRINT_INFO("Final state: " << data_info_best_sim.col(data_info_best_sim.cols() - 1).head(_robot->DimStates).transpose());
	      
	      //! Plot the results...------------------------------------
	      //! Plot the states and control inputs
	      
	      // plot options
	      teb::PlotOptions opt;
	      
	      opt.title = "TEB Closed-loop Simulation";
	      opt.legend = true;
	      opt.legend_entries.emplace_back("TEB closed-loop control");
	      opt.skip_last_value_right_column = true; // skip invalid control at time n	    	    
	      
	      Eigen::VectorXd time    = data_info_best_sim.bottomRows(1).transpose();
	      Eigen::MatrixXd states  = data_info_best_sim.block(0, 0, _robot->DimStates, data_info_best_sim.cols());
	      Eigen::MatrixXd control = data_info_best_sim.block(_robot->DimStates, 0, _robot->DimControls, data_info_best_sim.cols());		
	      
	      _plotter->plotTwoCol(time, states, time, control, &opt);
	      
	      //! Plot the robot and the trajectory...	 	 	 	      	
	      
	      initial_final_state.col(0) = data_obs_complete.row(data_obs_complete.rows() - 2).head(2);
	      initial_final_state.col(1) = data_obs_complete.row(data_obs_complete.rows() - 1).head(2);
	      
	      _plotter->switchWindow(1,true); // Create new window    
	      _rob_plotter->plotRobot(data_info_best_sim, initial_final_state);
	     
	     
	 }
	   
	 else    //! Case to check the simulation of dynamic obstacles..
	 {
	   
	      Eigen::MatrixXd data_dyn_obs_completed  = getInfoFromText("dyn_obs");
	      Eigen::MatrixXd data_dyn_obs;
	      unsigned int window_id = 1;
	      
	      unsigned int number_rows_data_info = _robot->DimStates + _robot->DimControls + 1;
	      
	      //! Five simulations will be presented..
	      const unsigned int n_sim = 5;	
	      
	      setTypeScenario(test_suite::TypeScenario::DYNAMIC_OBSTACLE);
	      initialize();
	      setParameters();	      
	      
	      initial_final_state.col(0) = _initial_state.head(2);
	      initial_final_state.col(1) = _final_state.head(2);
	      
	      setOptionPlotObstacle(1);       //! PLOT SINGLE_OBSTACLE
	      	      
	      //! It is not effcient to do it because we are allocating memory for each obstacle which will not be used but only to get the 
	      //! coordinates and to plot over trajectory... At the end, the memory will be deleted..	      	      
	      
	      unsigned int n =  data_dyn_obs_completed.rows() / 8;
	      unsigned int index_show = n / n_sim;
	      
	      Eigen::Vector2d obs_pos;
	      for(unsigned int i=0; i<n_sim; ++i)
	      {
		 data_dyn_obs = data_dyn_obs_completed.block(i * index_show * (number_rows_data_info + 1), 0, number_rows_data_info ,data_dyn_obs_completed.cols()-1);	 	 	 
		 obs_pos = data_dyn_obs_completed.row(i * index_show * (number_rows_data_info + 1) + number_rows_data_info).leftCols(2);
		 _robController->addObstacle(new teb::Obstacle(obs_pos, 2));
		 
		 _plotter->switchWindow(window_id++,true);   // Create new window    
	         _rob_plotter->plotRobot(data_dyn_obs, initial_final_state); 
		 
		 _robController->removeObstacle();
	      }	      	      	      	      	     	  
	 }
	 	 	 	 
       break;
       
    }       
    
    PRINT_INFO("Press key to continue ...");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
    
    PRINT_INFO("Bye... :-))");
}

template <int p, int q>
Eigen::MatrixXd Test<p,q>::getInfoFromText(std::string str)
{
   Eigen::MatrixXd data;
   std::vector<double> entries;   
   
   //! Check if the scene is already in the folder...
   std::stringstream ss_scene;
   ss_scene << _scene;	 
   
   unsigned int simu_idx = 0;
   unsigned int n_obs;
   std::string line;      
   std::string str1 = "obstacle_" + ss_scene.str() + ".dat";
   std::string str2 = "states_control_time_" + ss_scene.str() + ".dat";      
   std::string str3 = "dyn_obs.dat"; 
   
   //! The file is open to extract the information
   if(str.compare("obs") == 0)
      _simulation_file_obs.open((_pws->pw_dir + _folder_simulation + str1).c_str(), std::ios::in);
   
   else if(str.compare("info") == 0)
      _simulation_file_info.open((_pws->pw_dir + _folder_simulation + str2).c_str(), std::ios::in);
   
   else if(str.compare("pos_obs") == 0)
      _simulation_file_obs_pos.open((_pws->pw_dir + _folder_simulation + str2).c_str(), std::ios::in);
   
   else if(str.compare("dyn_obs") == 0)
   {
     if(fileExists(_pws->pw_dir + _folder_simulation + str3) != true)
       return data;
     
     _simulation_file_info.open((_pws->pw_dir + _folder_simulation + str3).c_str(), std::ios::in);
   }      
      	 
   while(true)
   {
     if(str.compare("obs") == 0) 
        getline(_simulation_file_obs,line);
     
     else if(str.compare("info") == 0)
        getline(_simulation_file_info,line);
     
     else if(str.compare("pos_obs") == 0)
        getline(_simulation_file_obs_pos,line);
     
     else if(str.compare("dyn_obs") == 0)
        getline(_simulation_file_info,line);
      
      if(line.compare("") == 0) break;
      
      std::stringstream ss(line);
      ss.exceptions( ios::failbit | ios::badbit );
	     
      n_obs = 0;
	     
      while(!ss.eof())
      {
        double value;
        ss >> value;
	 
	entries.push_back(value);        
        ++n_obs;	       
      }
      
      if(str.compare("obs") == 0)      
         data.conservativeResize(_robot->getUpperBoundEpsilon()/_robot->getStepEpsilon() + 5, entries.size());  //! + 5 because there are n_eps + 1 the obstacle position (2) and the initial/final state(2)
      
      else if(str.compare("info") == 0)
         data.conservativeResize((_robot->DimStates + _robot->DimControls + 1)*(simu_idx + 1), entries.size());
      
      else if(str.compare("dyn_obs") == 0)
         data.conservativeResize(entries.back() * 8 , entries.size());
      
      data.row(simu_idx) = Eigen::Map<Eigen::MatrixXd>(entries.data(),1,entries.size());
      
      entries.erase(entries.begin(), entries.end());
	     
      ++simu_idx;
  }
    
  return data;
}

template <int p, int q>
unsigned int Test<p,q>::getBestSimulation(const Eigen::Ref<const Eigen::MatrixXd>& mat)
{
  unsigned int idx = 0;
  unsigned int minimum_weight = std::numeric_limits< int >::max(); 
  unsigned int weight_obs;    
  
  for(unsigned int i=0; i<mat.rows(); ++i)
  {
    weight_obs = getWeightsPerEps(mat.row(i).transpose());
    if(weight_obs < minimum_weight)
    {
      minimum_weight = weight_obs; 
      idx = i;
    }
  }
  
  return idx;
}

template <int p, int q>
bool Test<p,q>::checkRobotToGoal(const teb::SimResults::TimeSeries& ts)
{
   double tolerance = 0.3;
   Eigen::MatrixXd state(_robController->getN(), ts.states.cols());
   state = ts.states;
   
   if((_final_state.head(2) - state.col(ts.states.cols() - 1).head(2)).norm() > tolerance)
     return false;
   
   else
     return true;
}

template <int p, int q>
void Test<p,q>::run()
{
   //! Execute the simulation the amount of times given by _number_simulation parameter... 
  int window_id = 1;
  std::stringstream ss;         
   
  _number_steps_epsilon = _robot->getUpperBoundEpsilon()/_robot->getStepEpsilon();    //          _limit_epsilon/_step_epsilon;                                
  _number_simulations_left = _number_simulations * _number_steps_epsilon * ((_verboseScene == "all") ? _cluster._max_scenes : 1);  
  
  do{
       prepareScene();
    
       ss.str("");
       ss << _scene;
    
      _fileNameObs  = "obstacle_" + ss.str() + ".dat";    
      _fileNameInfo = "states_control_time_" + ss.str() + ".dat"; 
      
      //! If the file already exist.. it is removed otherwise an exception is trigged...
      //if (!stat((_pws->pw_dir + _folder_simulation + _fileNameObs).c_str(), &myStat))
      if(fileExists(_pws->pw_dir + _folder_simulation + _fileNameObs) == true)
        std::remove((_pws->pw_dir + _folder_simulation + _fileNameObs).c_str());
      
      //if (!stat((_pws->pw_dir + _folder_simulation + _fileNameInfo).c_str(), &myStat))
      if(fileExists(_pws->pw_dir + _folder_simulation + _fileNameInfo) == true)
        std::remove((_pws->pw_dir + _folder_simulation + _fileNameInfo).c_str());
      
      //----------
      
      _simulation_file_obs.open((_pws->pw_dir + _folder_simulation+_fileNameObs).c_str(),      std::ofstream::out);
      _simulation_file_info.open((_pws->pw_dir + _folder_simulation+_fileNameInfo).c_str(),    std::ofstream::out);     
      
       _simulation_file_obs.exceptions(ios::failbit); 
       _simulation_file_info.exceptions(ios::failbit); 
      
      //! The simulation will run number_steps_epsilon times _number_simulation
      do{               
	
	_sim_idx = 0;    //! Restart the number of simulations...
	
	do{
		    
	    //! The _epsilon parameter is updated to each simulation..
	  //_solver_ref->soft_constr_epsilon = _epsilon;
	  
	  PRINT_INFO("Simulating scene: " << _scene);
	  PRINT_INFO("Simulation: " << (_sim_idx+1) << " with epsilon -> " << _robot->getEpsilonCostFunction() << "\n");
	  PRINT_INFO("Number of simulations left is: " << _number_simulations_left);
	  
	  //! The first part is set the respective parameters...
	    setParameters();
	    
	    PRINT_INFO("xi: " << _x0[0]);
	    PRINT_INFO("yi: " << _x0[1]);
	    PRINT_INFO("xf: " << _xf[0]);
	    PRINT_INFO("yf: " << _xf[1]);
	    
	    // Perform open-loop planning
	    START_TIMER;
	    _robController->step(_x0, _xf);
	    STOP_TIMER("TEB optimization loop");  
	    
// 	    Eigen::MatrixXd teb_mat = _robController->getStateCtrlInfoMat(); // Get all TEB states in a single matrix
// 	    Eigen::Matrix2d temp_mat = Eigen::Matrix2d::Zero();
// 	    _rob_plotter->plotRobot(teb_mat, temp_mat);
	    //saveMatrix(teb_mat, sim_idx, "state_control_time");    
	    
	    // Start simulation
	    // Create x-y plot
	    _plotter->switchWindow(window_id,true); // Create new window    
	    std::unique_ptr<teb::SimResults> results = _sim->simOpenAndClosedLoop(_x0,_xf,_time_sim);
	    
	    //The next step is to check the feasibility of the trayectory and create the result matrix 
	    createGeneralMatrix(results->series.back());
	    createFeasibilityVector(results->series.back());
		
	    //! Save the information into the textfile...
	    saveInfo();
	    PRINT_INFO("Saved the information \n");
	    
	    //! Check the simulation result to analize if the robot achieved the goal...
	    if(checkRobotToGoal(results->series.back()) == true){
	      PRINT_INFO("The robot reached to the goal...\n");	 
	    }
	    	    
	    else
	      PRINT_INFO("The robot did not reach to the goal...\n");
	    
	    // Create x-y plot
	    /*Eigen::Matrix2d initial_final_state; 
	    initial_final_state.col(0) = _initial_state.head(2);
	    initial_final_state.col(1) = _final_state.head(2);
	    
	    _plotter->switchWindow(window_id++,true); // Create new window    
	    _rob_plotter->plotRobot(results->series.back(),initial_final_state);*/		
	    
	    if(_type_scenario == TypeScenario::SINGLE_OBSTACLES)
		removeAllObstacles();
	    
	    PRINT_INFO("Simulation: " << _sim_idx+1 <<" has finished...\n");
	    PRINT_INFO("-----------------------------------------------------\n");
	    
	    ++_sim_idx;
	    
	    _number_simulations_left -= 1;
	    
	  }while(_sim_idx < _number_simulations);
	      
	  ++_idx_epsilon;
	  _robot->incrementEpsilon();            
	
      }while(_idx_epsilon <= _number_steps_epsilon);     
      
      _robot->setEpsilonCostFunction(0.1);
      
      _idx_epsilon = 1;
      
      //! The files are closed to preserve the information..
	  _simulation_file_info.close();
	  _simulation_file_obs.close();	 
	  _simulation_file_obs_pos.close();
	  
	  _initial_simulation = true;    //! In order to carry out the next simulation, new random states are generated...    
	  removeAllObstacles();
	  
      _scene++;	  
    
  }while(((unsigned int)_scene <= _cluster._max_scenes) && (_verboseScene == "all"));    
    
    
   PRINT_INFO("The simulation has finished...\n");
   PRINT_INFO("check the plots...");      
}

template <int p, int q>
void Test<p,q>::runSimulationDynamicObstacle(const Eigen::Ref<const Eigen::Vector2d>& obs_pos_ini, const Eigen::Ref<const Eigen::Vector2d>& obs_pos_final, unsigned int op_vel)
{        
    if(prepareDynamicObstacle(obs_pos_ini, obs_pos_final, op_vel) == false) return;
        
    unsigned int idx_motion = 0;
    unsigned int window_id = 1;
    unsigned int point_to_print = (unsigned int)(_robController->getObstacle()->getLengthPath() / 5);                
    
    _fileNameDynamicObs = "dyn_obs.dat";
             
    if(fileExists(_pws->pw_dir + _folder_simulation + _fileNameDynamicObs) == true)
        std::remove((_pws->pw_dir + _folder_simulation + _fileNameDynamicObs).c_str());
    
    //! Open the file to write
    _simulation_file_info.open(_pws->pw_dir + _folder_simulation + _fileNameDynamicObs, std::ofstream::out);
    
    _simulation_file_info.exceptions(ios::failbit);   //! If the open process fails... a exception is trigged..
        
     //! Due to the obstacle will be moved according to the path defined (straight line), the final temporal obstacle position is set
    _robController->getObstacle()->setObstaclePosition(obs_pos_ini, "final");
    
    do{          
	
        // Perform open-loop planning
	START_TIMER;
	_robController->step(_x0, _xf);
	STOP_TIMER("TEB optimization loop");       
      
	PRINT_INFO("Obstacle Pos: " << _robController->getObstacle()->getObstaclePosition("final"));              
	    
	// Start simulation
	_plotter->switchWindow(window_id,true); // Create new window
	std::unique_ptr<teb::SimResults> results = _sim->simOpenAndClosedLoop(_x0,_xf,25.0);		

	//! Check the simulation result to analize if the robot achieved the goal...
	if(checkRobotToGoal(results->series.back()) == true){
	   PRINT_INFO("The robot reached to the goal...");	 
	}
	    	    
	else
	   PRINT_INFO("The robot did not reach to the goal...");
	
	if(!(idx_motion % point_to_print) && idx_motion != 0)
	{
	   // Create x-y plot
	   Eigen::Matrix2d initial_final_state; 
	   initial_final_state.col(0) = _initial_state.head(2);
	   initial_final_state.col(1) = _final_state.head(2);
	  
	  _plotter->switchWindow(window_id++,true); // Create new window	  	  
	  _rob_plotter->plotRobot(results->series.back(), initial_final_state);  
	}
					
	double dis = 0;
	unsigned int iter =0;
	  for(unsigned int j=0; j<results->series.back().states.cols()-1; ++j)
	  {
	    if((results->series.back().states.topRows(2).col(j) - results->series.back().states.topRows(2).col(j+1)).norm() > 0.1){
		dis += (results->series.back().states.topRows(2).col(j) - results->series.back().states.topRows(2).col(j+1)).norm();
		iter = j;
	    }	    	    
	  }
	  	  
	saveDynamicObstaclePath(results->series.back());    //! Save the information into the textfile
	
	PRINT_INFO("No of states: " << results->series.back().states.cols());	
	PRINT_INFO("Distance: " << dis);
	PRINT_INFO("iter: " << iter);
        PRINT_INFO("Time: " << (double)iter * _robController->dt().dt());
    
	idx_motion++;
	
	//! Update the obstacle position..	
	if(idx_motion == _robController->getObstacle()->getLengthPath())  continue;
	
	_robController->getObstacle()->setObstaclePosition(idx_motion, "final");            
      
      PRINT_INFO("Stage " << idx_motion << " finished..");
      
      //! The obstacle starts again from its initial position...
      _robController->getObstacle()->setObstaclePosition(obs_pos_ini, "current");
      
    }while(idx_motion < _robController->getObstacle()->getLengthPath());
    
    PRINT_INFO("The simulation has finished...\n");    
    
    _simulation_file_info.close();
}

template <int p, int q>
void Test<p,q>::saveDynamicObstaclePath(const struct teb::SimResults::TimeSeries& ts)
{
   createGeneralMatrix(ts);
   
   saveStatesControlTimeMatrix();	
	
   Eigen::VectorXd v1(ts.states.cols()-1);    
    
   v1.setZero();   
   v1.head(2) = _robController->getObstacle()->getObstaclePosition("final");

   for(unsigned int j=0; j< ts.states.cols()-1; ++j)	 
     _simulation_file_info << std::fixed << std::setprecision(1) << v1.coeffRef(j) << " ";
   
   _simulation_file_info << _robController->getObstacle()->getLengthPath() << std::endl;	   
}

} // end namespace test_suite