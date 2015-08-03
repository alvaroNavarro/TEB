
#include <teb_package.h>
#include "differential_drive_system.h"
#include "controller_diff_drive.h"
#include <Eigen/Core>

int main()
{
  
  using StateVector = Eigen::MatrixXd;     
  
  double max_range = 10;
  double dt = 0.1;
  double t = dt;
  double factor = 2;
  unsigned int time_idx = 0;   
  
  double x,y;
  double yk_1 = 0;  
   
  StateVector data;        
  Eigen::MatrixXd info(3,1);
  //data = StateVector::Zero();    //! Initial Data..
  teb::NumericalDerivatives derivative(dt);
  
  
  // test SQP solver
    teb::Config cfg;
    
    // optimization settings
    cfg.teb.teb_iter = 5;
    cfg.optim.solver.solver_iter = 5;
    cfg.optim.solver.lsq.weight_equalities = 200;
    cfg.optim.solver.lsq.weight_inequalities = 20;
    cfg.optim.solver.lsq.weight_adaptation_factor = 2;        
    
    // trajectory settings
    cfg.teb.n_pre = 50;
    cfg.teb.dt_ref = 0.1;
    cfg.teb.dt_hyst = cfg.teb.dt_ref/10;
  
       
   do{
     
     data.conservativeResize(3,time_idx+1);
     
     x = t;
     y = factor * std::sin(x);             
     //y = 2;
     y = 3;

     info(0, 0) = x;
     info(1, 0) = y;
     info(2, 0) = derivative.computeAngle(yk_1, y);         
     
     yk_1 = y;
     
     data.col(time_idx) = info.col(0);     
     ++time_idx;     
     t += dt;
     
   }while(t < max_range);
   
   teb::TebPlotter plotter;
   //plotter.plot(x,y,"Test","X","Y");
  
   
   DifferentialDriveRobotSystem<3,2> robotSystem;
   //MobileRobotTruckSystem truckSystem;
   
   teb::SolverLevenbergMarquardtEigenSparse solver;
   teb::RobControllerDiffDrive<3,2> teb(&cfg, &solver);
   
    //teb.addObstacle(new teb::Obstacle(5, 4));
    //teb.addObstacle(new teb::Obstacle(10, 4));
    //teb.addObstacle(new teb::Obstacle(18, 2));
      
   //truckSystem.setTruckDimension(dimTracksystem);
   
   teb::RobotFrame<3,2> robot(robotSystem);                    
   
   teb::RobotPlot<3,2> rplot(&plotter,&robot, &teb);
   rplot.plotRobot(data);
   
            
   //plotter.updateParameters(robotSystem.getRobotLength(),robotSystem.getRobotWidth());  
   //plotter.plotRobotPlusTrajectory(data);
   //plotter.plotRobot(x,y,theta);
   
   
   return 0;
}