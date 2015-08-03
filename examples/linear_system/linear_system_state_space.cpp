#include "linear_system_state_space.h"


int main()
{
  
    const int p = 2; //! number of states
    const int q = 1; //! number of inputs
    
    Eigen::Matrix2d A;
    A(0,0) = 0;
    A(0,1) = 1;
    A(1,0) = 0;
    A(1,1) = 1;
    
    Eigen::Vector2d b;
    b(0) = 0;
    b(1) = 1;
    
    //! Define start and goal
    double x0[] = {0,0}; //! start state
    double xf[] = {1,0}; //! goal state
    double u; //! store control
    
    //! Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<p,1> teb(&solver);
    
    //! System specific settings
    LinearSystemStateSpace<p,q> system;
    system.setStateSpaceModel(A,b);
    
    teb.setSystemDynamics(&system);
    
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1.0, 1.0);
    
    //! Perform open-loop planning
    START_TIMER;
    teb.step(x0, xf, &u);
    STOP_TIMER("TEB optimization loop");
    
    //! Print determined control for the current sampling interval
    PRINT_INFO("Control u: " << u);
    
    //! Plot states and control input
    teb::TebPlotter plotter;
    plotter.plotTEB(teb);
    
    return 0;
}
