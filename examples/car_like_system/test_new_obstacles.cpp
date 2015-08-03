
#include "controller_car_like.h"

int main()
{
   const int n_state   = 4;
   const int n_control = 2;   
   
   double x0[] = {-7,1,0,0};
   double xf[] = {5,0,0,0};
  
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
    
    //! Setup obstacles...        
    teb::ClusterObstacle cluster(7);
    cluster.generateScene();       //! The scenario is chosen of random way...
    
    //! Set the reference of Obstacle container instanciated in the Cluster class to Robot controller
    for(unsigned int i=0; i<cluster.getCluster().size(); ++i)
       teb.setObstacleContainer(cluster.getCluster().at(i)->getObstacleContainer());
    
     // Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf);
    STOP_TIMER("TEB optimization loop");   
    
    //!----------------------------------------------------------
    teb::RobotFrame<n_state, n_control> robot_frame(robotSystem);                          
    teb::RobotPlot<n_state, n_control>  rplot(&plotter,&robot_frame, &teb);    
    rplot._option = teb::RobotPlot<n_state, n_control>::OptionPlot::PLOT_SINGLE_OBSTACLES;
    //rplot.setIndivudualObstacleBool(false);                                  //! The obstacles are a set of clusters
    rplot.setCluster(&cluster);
    teb::Simulator<n_state, n_control>  sim(&teb,robotSystem,&plotter);
    
    sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);    
        
    // Start simulation
    std::unique_ptr<teb::SimResults> results = sim.simOpenAndClosedLoop(x0,xf,35.0);
    //!-----------------------------------------------------------
    
    plotter.switchWindow(1,true); // Create new window
    rplot.plotRobot(results->series.back());            //! Option 1: Plot the singles obstacles
						        //! Option 2: Plot the tranparent polygon and the obstacles inside
                                                        //! Option 3: Plot the solid polygon 
  
  return 0;
}
