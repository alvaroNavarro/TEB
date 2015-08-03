#include <teb_package.h>


template <int p, int q>
class LinearSystemODE : public teb::SystemDynamics<p,q>
{
public:
    using StateVector = typename teb::SystemDynamics<p,q>::StateVector;
    using ControlVector = typename teb::SystemDynamics<p,q>::ControlVector;
    
    inline virtual StateVector stateSpaceModel(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u) const
    {
        StateVector xdot;
        
        xdot[0] =  x[1];
        xdot[1] =  -x[0] - 0.5*x[1] + u[0] - 0.7*u[1];
        //xdot[2] = u[0];
        //xdot[0] = -x[1] + u[0];
       // xdot[1] = x[0] - 0.5*x[1] + 0.2*u[1];

        return xdot;
    }
};


int main()
{
  
    
    double x0[] = {0,0}; // start state
    double xf[] = {1,0}; // goal state
    double u[] = {0,0}; // store control
    
    
    teb::Config cfg;
    
    //cfg.optim.solver.solver_iter = 10;
    //cfg.teb.teb_iter = 10;
    //cfg.teb.dt_ref = 0.01;
    //cfg.teb.dt_hyst = 0.001;
    //cfg.teb.dt_min = 0.0001;
    //cfg.teb.n_pre = 50;
    //cfg.optim.solver.lsq.weight_equalities = 10;
    //cfg.optim.solver.lsq.weight_inequalities = 200;
    //cfg.optim.solver.lsq.weight_adaptation_factor = 2;
    
    
    // Setup solver and controller
    teb::SolverLevenbergMarquardtEigenSparse solver;
    teb::TebController<2,2> teb(&cfg, &solver);
    
    // System specific settings
    LinearSystemODE<2,2> system;
    
    
    teb.setSystemDynamics(&system);
    
    //teb.activateObjectiveQuadraticForm(1, 0, 0.1);
    teb.activateObjectiveTimeOptimal();
    teb.activateControlBounds(0, -1, 1);
    //teb.activateControlBounds(1, -1, 1);
    
    teb.activateControlInputDerivatives();
    teb.setPreviousControlInput({0},0.1);
    
    teb::TebPlotter plotter;
    
    // Perform open-loop planning
    teb.step(x0, xf, u);
    plotter.plotTEB(teb);
    
    
    //teb::Simulator<3,1> sim(&teb, system, &plotter);
    //sim.simOpenLoop(x0,xf);
    //sim.simOpenAndClosedLoop(x0, xf, 3);
    
    
    
    // Print determined control for the current sampling interval
    //PRINT_INFO("Control u: " << u);
    

    
    
    return 0;
}
