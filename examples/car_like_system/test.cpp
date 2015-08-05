
#include "testSimulation.h"
#include "controller_car_like.h"

int main()
{
   const int n_state   = 4;
   const int n_control = 2;   
  
   teb::Config cfg; 
   
   // optimization settings   -   Default values but the weight factor change during the simulation..
   cfg.teb.teb_iter = 5;
   cfg.optim.solver.solver_iter = 5;
   cfg.optim.solver.lsq.weight_equalities = 200;
   cfg.optim.solver.lsq.weight_inequalities = 20;
   cfg.optim.solver.lsq.weight_adaptation_factor = 2;        
   
   // trajectory settings
   cfg.teb.n_pre = 50;
   cfg.teb.dt_ref = 0.1;
   cfg.teb.dt_hyst = cfg.teb.dt_ref/10;
   
   //  System specific settings
   MobileRobotCarLikeSystem<n_state, n_control> robotSystem;
   
   robotSystem.setTypeDriving(teb::TypeDriving::REAR_WHEEL);
   robotSystem.setRobotDimesion(0.9,0.35);
   
   // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::RobControllerCarLike<n_state, n_control> teb(&cfg, &solver);            
                              
    teb.activateObjectiveTimeOptimal();        
    teb.setSystemDynamics(&robotSystem);    
    teb.activateControlBounds({-0.1,-0.5},{1,0.5});
    teb.setSystem(&robotSystem);
    
    teb::TebPlotter plotter;    
    plotter.setDistanceRobotPlot(1.0);
    
    teb::Simulator<n_state, n_control>  sim(&teb, robotSystem, &plotter);    
    sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);
    
    teb::RobotFrame<n_state, n_control> robot_frame(robotSystem);                          
    teb::RobotPlot<n_state, n_control>  rplot(&plotter, &robot_frame, &teb);                        
    
    //! Set the Test suite.....----------------
    test_suite::Test<n_state,n_control> test(&teb, robotSystem, &sim, &plotter);
    test.setTimeSimulation(45);    
    test.setRefSolver(&cfg.optim.solver.lsq);
    test.setRobPlotter(&rplot);    
    
//     using namespace std::placeholders;
//     sim.setPostStepCallback(std::bind(&teb::RobControllerCarLike<n_state, n_control>::updateObstaclePosition, &teb, _1, _2, _3));
	
    
    //! Present the menu to the user..
    test.menu();        
                  
   return 0;
}