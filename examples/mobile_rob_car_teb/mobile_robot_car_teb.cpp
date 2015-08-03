#include "mobile_robot_car_teb.h"


int main()
{
    
    teb::Config cfg;
    cfg.optim.solver.lsq.weight_equalities = 500;
    cfg.optim.solver.lsq.weight_inequalities = 50;
    cfg.optim.solver.lsq.weight_adaptation_factor = 2;
    cfg.teb.dt_min = 0.05;
    cfg.teb.n_pre = 50;
    cfg.teb.n_max = 150;
    cfg.teb.n_min = 2;
    
    double x0[] = {0,0, 0,0}; //! start state
    double xf[] = {5,0, 0,0}; //! goal state
    double obst_pos[] = {2.5, 0.1}; //! obstacle position  
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::RobControllerCarLike teb(&cfg, &solver);
    
    teb.activateObjectiveTimeOptimal();
    teb.setObstaclePosition(obst_pos);
       
    // Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf);
    STOP_TIMER("TEB optimization loop");

    // Plot open-loop results
    teb.plotRobotStuff();
    
    // Perform closed-loop sim
    teb::TebPlotter plotter;
    MobileRobot robot; // Motion model for simulation
    teb::Simulator<3,2> sim(&teb,robot,&plotter);
    
    // Set simulation sample time
    sim.setSampleTime(0.1);
    
    using namespace std::placeholders;
    sim.setPostStepCallback(std::bind(&teb::RobController::robSaveVelocityAfterStep, &teb, _1, _2, _3));
    
    // Start simulation
    sim.simOpenAndClosedLoop(x0,xf,7);
    
    return 0;
}
