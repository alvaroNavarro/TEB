#include "car_like_system.h"

int main()
{
    
    double x0[] = {0,0,0,0}; 		//! start state
    double xf[] = {7,7,0,0}; 		//! goal state    

    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
        
    // test SQP solver
    teb::Config cfg;
    
    // optimization settings
    cfg.teb.teb_iter = 5;
    cfg.optim.solver.solver_iter = 5;
    cfg.optim.solver.lsq.weight_equalities = 20;
    cfg.optim.solver.lsq.weight_inequalities = 2;
    cfg.optim.solver.lsq.weight_adaptation_factor = 2;
    
    
    // trajectory settings
    cfg.teb.n_pre = 50;
    cfg.teb.dt_ref = 0.1;
    cfg.teb.dt_hyst = cfg.teb.dt_ref/10;
   
    //teb::SolverSQPLocalDense solver;        
    teb::TebController<4,2> teb(&cfg, &solver);
    
    // System specific settings
    MobileRobotCarLikeSystem robotSystem;
    
    robotSystem.setTypeDriving(TypeDriving::FRONT_WHEEL);
    robotSystem.setDistanceBetweenFrontRearAxles(0.5);       //! 40cm between axles
    robotSystem.setRobotWidth(0.35);	
          
    teb.activateObjectiveTimeOptimal();        
    teb.setSystemDynamics(&robotSystem);    
    teb.activateControlBounds({-0.1,-0.5},{1,0.5});
    
    // Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf);
    STOP_TIMER("TEB optimization loop");    
    
    // Print determined control for the current sampling interval
    //PRINT_INFO("Control u: " << u);
    
    // Plot states and control input
    teb::TebPlotter plotter;    
    
    teb::Robot robot;
    Eigen::Vector2d dim;
    robotSystem.getRobotDimesion(dim);
    robot.setNumberOfCars(1);
    robot.initialize(dim);            
    teb::RobotPlot rplot(&plotter,&robot);
    
    
    // optional: plotter.setOutputToFile("teb_opt_int",teb::TebPlotter::FileFormat::PNG);
    // plotter.plotTEB(teb);
        
    teb::Simulator<4,2> sim(&teb,robotSystem,&plotter);
    
    sim.setIntegrator(teb::NumericalIntegrators::EXPLICIT_EULER);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);    
    
    // Start simulation
    sim.simOpenAndClosedLoop(x0,xf,10.0);
            
     // Create x-y plot
    plotter.switchWindow(1,true); // Create new window
    Eigen::MatrixXd teb_mat = teb.getStateCtrlInfoMat(); // Get all TEB states in a single matrix
    //plotter.plot(teb_mat.row(0).transpose(), teb_mat.row(1).transpose(),"X-Y Trajectory","x [m]", "y [m]");
    //plotter.updateParameters(robotSystem.getRobotLength(),robotSystem.getRobotWidth()); 
    //plotter.plotRobotPlusTrajectory(teb_mat);
    
    rplot.plotRobot(teb_mat);
    
    return 0;
}
