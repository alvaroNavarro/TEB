
#include <teb_package.h>
#include "car_like_system.h"
#include "trailer_system.h"
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
  Eigen::MatrixXd info(5,1);
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
     
     data.conservativeResize(5,time_idx+1);
     
     x = t;
     //y = factor * std::sin(x);             
     //y = 2;
     y = x;

     info(0, 0) = x;
     info(1, 0) = y;
     info(2, 0) = derivative.computeAngle(yk_1, y);
     info(3, 0) = 0;     
     info(4, 0) = M_PI/4; 
     
     
     yk_1 = y;
     
     data.col(time_idx) = info.col(0);     
     ++time_idx;     
     t += dt;
     
   }while(t < max_range);
   
   teb::TebPlotter plotter;
   //plotter.plot(x,y,"Test","X","Y");
  
   /*double x = 3;
   double y = 3;
   double theta = -2*M_PI/3;*/
   
   //double *dimTracksystem[] = {0.9,0.35,0.6,0.35};     //! Dimension for the track system:  Fist:   truck Length
						       //! 				    Second: truck width 
						       //!				    Third:  trailer length
						       //!				    Fourth trailer width
   
   MobileRobotCarLikeSystem<4,2> robotSystem;
   //MobileRobotTruckSystem truckSystem;
   
   teb::SolverLevenbergMarquardtEigenSparse solver;
   teb::RobControllerCarLike<4,2> teb(&cfg, &solver);
   
    //teb.addObstacle(new teb::Obstacle(5, 4));
    //teb.addObstacle(new teb::Obstacle(10, 4));
    //teb.addObstacle(new teb::Obstacle(18, 2));
   
   robotSystem.setTypeDriving(teb::TypeDriving::FRONT_WHEEL);   			    
   //truckSystem.setTruckDimension(dimTracksystem);
   
   teb::RobotFrame<4,2> robot(&robotSystem);                
   
   teb::RobotPlot<4,2> rplot(&plotter,&robot, &teb);
   rplot.plotRobot(data);
   
            
   //plotter.updateParameters(robotSystem.getRobotLength(),robotSystem.getRobotWidth());  
   //plotter.plotRobotPlusTrajectory(data);
   //plotter.plotRobot(x,y,theta);
   
   
   return 0;
}