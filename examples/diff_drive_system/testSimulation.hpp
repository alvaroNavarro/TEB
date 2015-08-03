
#include "testSimulation.h"

#include <fcntl.h>
#include <string.h>
#include <iomanip>
#include <fcntl.h>
#include <stdio.h>
#include <limits>
#include <assert.h>

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
}

template <int p, int q>
bool Test<p,q>::isObstacleOverlapped(teb::Obstacle* obs1, teb::Obstacle* obs2)
{
  double dis = (obs1->getObstaclePosition() - obs2->getObstaclePosition()).norm();  
  
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
}

template <int p, int q>
bool Test<p,q>::isRobotOverlappedToObstacle(const Eigen::Ref<const Eigen::Vector2d>& posRobot, teb::Obstacle* obstacle)
{
  
  if(obstacle->getRadiusObstacle() + obstacle->getMaximumRadius() - (posRobot - obstacle->getObstaclePosition()).norm() > 0)
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
      
      _simulation_file_info << std::fixed << std::setprecision(1) << _robot->getEpsilonCostFunction() << std::endl;
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
      
    _simulation_file_obs << std::fixed << std::setprecision(1) << _robot->getEpsilonCostFunction() << std::endl;    
    
    //! If is the last simulation of the scene, the obstacle positio is saved at the last two rows..
    if((_idx_epsilon == _number_steps_epsilon) && (_sim_idx == _number_simulations-1))
    {
      for(unsigned int i=0; i<2; ++i)
      {
	for(unsigned int j=0; j<_number_obstacles; ++j)
	      _simulation_file_obs << std::fixed << std::setprecision(1) << _robController->getObstacles().at(j)->getObstaclePosition()[i] << " ";  
	
	_simulation_file_obs << _scene <<  std::endl;
      }
      
      //! Save the initial and final state generated by the random generator...            
      for(unsigned int j=0; j< _number_obstacles; ++j)
	 _simulation_file_obs << std::fixed << std::setprecision(1) << initial_state.coeffRef(j) << " ";
	  
      _simulation_file_obs << _scene << std::endl;
      
      for(unsigned int j=0; j< _number_obstacles; ++j)
	 _simulation_file_obs << std::fixed << std::setprecision(1) << final_state.coeffRef(j) << " ";
	  
      _simulation_file_obs << _scene << std::endl;     
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
	  if (!((stat(_folder_simulation.c_str(), &myStat) == 0) && (((myStat.st_mode) & S_IFMT) == S_IFDIR)))
              mkdir(_folder_simulation.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	 
	  PRINT_INFO("The type of Scenarios are: \n 1 -> Single Obstacles \n 2 -> Scenarios \n 3 -> Car Parking \n");
	  PRINT_INFO("Please enter the number of scenario: ");
	  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          INPUT_STREAM(input_scenario,-1);	  	  
	  
	  if(input_scenario > 3)
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
	  }
	  	  
	  
	  //PRINT_INFO("Epsilon will vary from 0 to 0.9\n");
	  PRINT_INFO("Please enter number of simulations: ");
	  INPUT_STREAM(number_simulations,-1);	  
	  setNumberSimulations(number_simulations);
	  
	  //! Everything is set and as result we are ready to start the simulation....    
	  //! Do not forget to initialize...
	  initialize();
	  run();     //! Execute the complete simulation...
	 
       break;
       
       
       case 2:
	 
	 //! Option to get and plot the results
	 //! First it is important if there are data saved....
	 PRINT_INFO("The option to check the results has been chosen.. \n");
	 if (!((stat(_folder_simulation.c_str(), &myStat) == 0) && (((myStat.st_mode) & S_IFMT) == S_IFDIR)))
	 {
	    PRINT_INFO("Sorry ... The main folder has not created before..."); 
	    break;
	 }
	 
	 PRINT_INFO("Please enter the scene Id: 1 - " << _cluster._max_scenes << " \n");
	 PRINT_INFO("Scene Id: ");	 
	 INPUT_STREAM(_scene,-1);
	 	 	 
	 unsigned int op;
	 
	 PRINT_INFO("The options to plot the obstacles are: \n 1: Plot only the obstacles \n 2: Plot both the obstacle and the border \n 3: Plot only the border \n");
	 PRINT_INFO("Enter option: ");
	 INPUT_STREAM(op,-1);
	 //std::cin >> op;
	    
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
	Eigen::Matrix2d initial_final_state;//	
	
	initial_final_state.col(0) = data_obs_complete.row(data_obs_complete.rows() - 2).head(2);
	initial_final_state.col(1) = data_obs_complete.row(data_obs_complete.rows() - 1).head(2);
	
	_plotter->switchWindow(1,true); // Create new window    
	_rob_plotter->plotRobot(data_info_best_sim, initial_final_state);
	 
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
   
   unsigned int simu_idx_epsilon = 0;
   unsigned int n_obs;
   std::string line;      
   std::string str1 = "/obstacle_" + ss_scene.str() + ".dat";
   std::string str2 = "/states_control_time_" + ss_scene.str() + ".dat";      
   
   //! The file is open to extract the information
   if(str.compare("obs") == 0)
      _simulation_file_obs.open((_folder_simulation + str1).c_str(), std::ios::in);
   
   else if(str.compare("info") == 0)
      _simulation_file_info.open((_folder_simulation + str2).c_str(), std::ios::in);
   
   else if(str.compare("pos_obs") == 0)
      _simulation_file_obs_pos.open((_folder_simulation + str2).c_str(), std::ios::in);
   
   if(fileExists(_folder_simulation + str1) != true || fileExists(_folder_simulation + str2) != true)          
      return data;         
      	 
   while(true)
   {
     if(str.compare("obs") == 0) 
        getline(_simulation_file_obs,line);
     
     else if(str.compare("info") == 0)
        getline(_simulation_file_info,line);
     
     else if(str.compare("pos_obs") == 0)
        getline(_simulation_file_obs_pos,line);
      
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
         data.conservativeResize((_robot->DimStates + _robot->DimControls + 1)*(simu_idx_epsilon + 1), entries.size());
      
      data.row(simu_idx_epsilon) = Eigen::Map<Eigen::MatrixXd>(entries.data(),1,entries.size());
      
      entries.erase(entries.begin(), entries.end());
	     
      ++simu_idx_epsilon;
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
   
  _number_steps_epsilon = _robot->getUpperBoundEpsilon()/_robot->getStepEpsilon();    //          _limit_epsilon/_step_epsilon;      
  
  std::stringstream ss;                  
     
  _number_simulations_left = _number_simulations * _number_steps_epsilon * ((_verboseScene == "all") ? _cluster._max_scenes : 1);  
  
  do{
       prepareScene();
    
       ss.str("");
       ss << _scene;
    
      _fileNameObs  = "obstacle_" + ss.str() + ".dat";    
      _fileNameInfo = "states_control_time_" + ss.str() + ".dat"; 
      
      _simulation_file_obs.open((_folder_simulation+_fileNameObs).c_str(),      std::ofstream::out);
      _simulation_file_info.open((_folder_simulation+_fileNameInfo).c_str(),    std::ofstream::out);               
      
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
	    
	    //Eigen::MatrixXd teb_mat = _robController->getStateCtrlInfoMat(); // Get all TEB states in a single matrix
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
	    PRINT_INFO("Saved the information");
	    
	    //! Check the simulation result to analize if the robot achieved the goal...
	    if(checkRobotToGoal(results->series.back()) == true){
	      PRINT_INFO("The robot reached to the goal...");	 
	    }
	    	    
	    else
	      PRINT_INFO("The robot did not reach to the goal...");
	    
	    // Create x-y plot
	    Eigen::Matrix2d initial_final_state; 
	    initial_final_state.col(0) = _initial_state.head(2);
	    initial_final_state.col(1) = _final_state.head(2);
	    
	    //_plotter->switchWindow(window_id++,true); // Create new window    
	    //_rob_plotter->plotRobot(results->series.back(),initial_final_state);		
	    
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
      
      _robot->setEpsilonCostFunction(0);
      
      _idx_epsilon = 0;
      
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


} // end namespace test_suite