
#ifndef __teb_package_examples_testSimulation__
#define __teb_package_examples_testSimulation__

#include <teb_package/base/typedefs.h>
#include <teb_package/visualization/teb_plotter.h>
#include <teb_package/base/teb_controller.h>
#include <teb_package/simulation/simulator.h>
#include <teb_package/base/system_dynamics.h>
#include <teb_package/base/obstacle.h>
#include <teb_package/base/config.h>

#include <random>
#include <fcntl.h>
#include <string.h>
#include <fstream>
#include <sys/stat.h>
#include <chrono>

//#include "car_like_system.h"
#include "differential_drive_system.h"

namespace test_suite
{

  
  enum class TypeScenario
  {
     SINGLE_OBSTACLES,
     SCENARIO,
     CAR_PARKING
  };
  
template <int p, int q>
class Test 
 {
 public:
        
   using StateVector = typename Eigen::Matrix<double,p,1>;
   using PointPolygon = typename std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >;
   
   Test(teb::TebController<p,q>* robController, DifferentialDriveRobotSystem<p,q>& robot, teb::Simulator<p,q>* sim, teb::TebPlotter* plotter) : _robController(robController), _robot(&robot), _sim(sim), _plotter(plotter) 
   {
     _folder_simulation  = "/home/navarro/Desktop/Simulations/";    
   }     
   
   ~Test()
   {
     //delete _x0;
     //delete _xf;     
   }
   
   void initialize()
   {                    
     gen.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());                             
     //gen.seed(1);
     
     _x0 = new double[_robot->DimStates];
     _xf = new double[_robot->DimStates];
     
     if(_type_scenario == TypeScenario::SCENARIO)
     {
       //! If you forget to set the number of scene before initialize Cluster. By default a random shuffle scene is chosen  
       if(_verboseScene == "no")
       {
	   if(_scene == -1){
	      std::uniform_int_distribution<unsigned int> random_scene(1, _cluster._max_scenes); 
	      _scene = random_scene(gen);
	  } 
       }
       
       else   _scene = 1;                     	       	                     
     }
     
     else if(_type_scenario == TypeScenario::CAR_PARKING)
     {
       _scene = 9;   //! Scene to simulate the car parking  
     }
     
     //!Set the cluster to reference the obstacles..
     _rob_plotter->setCluster(&_cluster);
   }
   
   void prepareScene()
   {
       _cluster.addScene((unsigned int)_scene);
       
       //! Now the scene is generated
       _cluster.generateScene();
       
       //! Set the reference of Obstacle container instanciated in the Cluster class to Robot controller
       for(unsigned int i=0; i<_cluster.getCluster().size(); ++i)
	 _robController->setObstacleContainer(_cluster.getCluster().at(i)->getObstacleContainer());
       
       _number_obstacles = _robController->getObstacles().size();
   }
   
   void setNumberScene(int scene) {_scene = scene;}
   
   void setRefSolver(teb::Config::Optim::Solver::Lsq* solver_ref) {_solver_ref = solver_ref;};
   void setRobPlotter(teb::RobotPlot<p,q>* rob_plotter) {_rob_plotter = rob_plotter;}      
   void setNumberObstacles(unsigned int n_obs) { _number_obstacles = n_obs; }
   void setNumberSimulations(unsigned int n_sim) {_number_simulations = n_sim; }
   void setNumberSamples(unsigned int number_samples) {_number_samples = number_samples;}
   unsigned int getNumberSamples(){return(_number_samples);}
   
   void setWorkSpace(const double* dim) { _workspace = Eigen::Vector4d(dim);}
   void setWorkSpace(const Eigen::Ref<const Eigen::Vector4d>& workspace) {_workspace = workspace;};     
   
   void setTimeSimulation(double time_sim) { _time_sim = time_sim; }
   double getTimeSimulation(){ return(_time_sim); }
   
   void insertObstacle();
   void removeAllObstacles();
   
   StateVector getRandomState(std::string string);
   StateVector getPredefinedState(unsigned int num_state, std::string string);
   void getArrayStates()
   { 
     if(_robot)
     {
       Eigen::Map<StateVector>(_x0,_robot->DimStates,1) = _initial_state; 
       Eigen::Map<StateVector>(_xf,_robot->DimStates,1) = _final_state;       
     }          
   }   
   
   void setTypeScenario(TypeScenario type){ _type_scenario = type;}
   
   void setOptionPlotObstacle(unsigned int op)
   {
      switch(op)
      {
	case 1: _rob_plotter->_option = teb::RobotPlot<p,q>::OptionPlot::PLOT_SINGLE_OBSTACLES;       break;
	case 2: _rob_plotter->_option = teb::RobotPlot<p,q>::OptionPlot::PLOT_OBSTACLES_WITH_BORDER;  break;
	case 3: _rob_plotter->_option = teb::RobotPlot<p,q>::OptionPlot::PLOT_ONLY_BORDER;            break;
      }
   }
   
   void menu();
   void run();  
   
   void setParameters()
   {           
     if(_type_scenario == TypeScenario::SINGLE_OBSTACLES)     
         insertObstacle();          
     
     //! The initial and final state must be placed in a free collision region, it means, overlapping between robot and obstacle
     //! does not have to exist...
     
     if(_initial_simulation == true)
     {
         
       if(_type_scenario == TypeScenario::SINGLE_OBSTACLES)
       {
          do{     
	      _initial_state = getRandomState("initial");       
	    }while(isRobotOverlappedToAnyObstacle(_initial_state.head(2)));
	    
	    do{
	    _final_state   = getRandomState("final");     
	    }while(isRobotOverlappedToAnyObstacle(_final_state.head(2)));
	    
	    getArrayStates();   	    	    
                   
       }
       
       else if(_type_scenario == TypeScenario::SCENARIO)
       {
	  
	 //! Guarantee that both initial and final state are not overlapped....
	 do{
	 
	    do{     
	      _initial_state = getRandomState("initial");       
	    }while(isRobotOverlappedToAnyCluster(_initial_state.head(2)));
	    
	    do{
	    _final_state   = getRandomState("final");     
	    }while(isRobotOverlappedToAnyCluster(_final_state.head(2)));
	   
	 }while(AreBothStatesOverlapped(_initial_state.head(2), _final_state.head(2)));
	 	  	    
	  getArrayStates();   	    	    
       }
       
       else if(_type_scenario == TypeScenario::CAR_PARKING)
       {
	  //! The initial and final state are predefined... 
	 _initial_state = getPredefinedState(1,"initial");
	 _final_state   = getPredefinedState(1,"final");
	 
	  getArrayStates(); 
       }
       
       _initial_simulation = false;
       
     }
       /*PRINT_INFO("xi: " << _initial_state.coeffRef(0));
       PRINT_INFO("yi: " << _initial_state.coeffRef(1));
       PRINT_INFO("xf: " << _final_state.coeffRef(0));
       PRINT_INFO("yf: " << _final_state.coeffRef(1));*/
   }
   
   bool isObstacleOverlapped(teb::Obstacle* obs1, teb::Obstacle* obs2);    
   bool isRobotOverlappedToAnyObstacle(const Eigen::Ref<const Eigen::Vector2d>& posRobot);
   bool isRobotOverlappedToAnyCluster(const Eigen::Ref<const Eigen::Vector2d>& posRobot);
   bool isRobotInsidePolygon(const Eigen::Ref<const Eigen::Vector2d>& posRobot, const PointPolygon& point_polygon);
   bool AreBothStatesOverlapped(const Eigen::Ref<const Eigen::Vector2d>& pos1, const Eigen::Ref<const Eigen::Vector2d>& pos2);
              
   void createGeneralMatrix(const teb::SimResults::TimeSeries& ts);
   void createFeasibilityVector(const teb::SimResults::TimeSeries& ts);
   bool isRobotOverlappedToObstacle(const Eigen::Ref<const Eigen::Vector2d>& posRobot, teb::Obstacle* obstacle);
   
   void saveInfo();      
   void saveStatesControlTimeMatrix();
   void saveFeasibilityVector();
   void saveObstaclePositionMatrix();
   
   void setVerboseOption(unsigned int op)
   {
      if(op == 0)  _verboseScene = "no";
      else         _verboseScene = "all";
   }
   
   bool fileExists(const std::string& filename)
   {
     struct stat buf;
     
	if (stat(filename.c_str(), &buf) != -1) 
	    return true;
	     
      return false;
   }
   
   unsigned int getBestSimulation(const Eigen::Ref<const Eigen::MatrixXd>& mat);   
   unsigned int getWeightsPerEps(const Eigen::Ref<const Eigen::VectorXd>& sim_per_eps) {  return(sim_per_eps.sum()); }
   
   Eigen::MatrixXd getInfoFromText(std::string str);
   
   bool checkRobotToGoal(const teb::SimResults::TimeSeries& ts);
    
 protected:      
   
   unsigned int _idx_epsilon = 0;
   unsigned int _sim_idx; 
   unsigned int _number_steps_epsilon;
   
   TypeScenario _type_scenario;
   int _scene = -1;
   std::string _verboseScene;
   
   teb::ClusterObstacle _cluster;
   
   teb::Config::Optim::Solver::Lsq* _solver_ref;
   
   //std::random_device rd;   // non-deterministic generator
   //std::mt19937_64 gen(rd());  				                      // to seed mersenne twister.     
   //std::default_random_engine generator;
   
   //std::default_random_engine gen;
   
   std::mt19937_64 gen;      
   
   std::fstream _simulation_file_obs;
   std::fstream _simulation_file_info;
   std::fstream _simulation_file_obs_pos;
      
   std::string _folder_simulation;   
   std::string _fileNameObs;
   std::string _fileNameInfo;
   std::string _fileNameObsPos;
   
   unsigned int _number_simulations;
   unsigned int _number_obstacles = 10;
   
   Eigen::Vector4d _workspace   = Eigen::Vector4d(-10,10,-5,5);     //! xmin - xmax - ymin - ymax
   Eigen::Vector2d _theta_bound = Eigen::Vector2d(-M_PI, M_PI);       //! Define the bound for the robot orientation,,
   Eigen::Vector2d _phi_bound   = Eigen::Vector2d(-M_PI/2, M_PI/2);   //! Define the bound for the steering angle..
   
   Eigen::VectorXi _vector_obstacles;
   Eigen::MatrixXd _state_control_time; 
      
   double* _x0;
   double* _xf;
   StateVector _initial_state;
   StateVector _final_state;
          
   teb::TebController<p,q>* _robController = nullptr;
   //MobileRobotCarLikeSystem<p,q>* _robot = nullptr;   
   DifferentialDriveRobotSystem<p,q>* _robot = nullptr;   
   
   teb::Simulator<p,q>* _sim = nullptr;
   teb::RobotPlot<p,q>* _rob_plotter = nullptr;
   teb::TebPlotter* _plotter;
   
   double _time_sim = 10;
   unsigned int _number_samples = 20;
   double _number_simulations_left;
   
   const unsigned int _max_number_obs = 15;
   
   bool _initial_simulation = true;      //! At the beginning of the simulation...
   
 public:
   
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
 };
 
} // end namespace test_suite
 
// include implementation of all template methods 
#include "testSimulation.hpp"

#endif  /* defined(__teb_package_test__) */