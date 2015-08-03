#ifndef __teb_package__config__
#define __teb_package__config__

#include <teb_package/base/typedefs.h>
#include <cstdint>


namespace teb {

/** Enumeration for different finite difference types used especially for dynamic system discretization */
enum class FiniteDifferences
{
    FORWARD, //!< Forward differences: \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k}}{\Delta T} \f$
    CENTRAL //!< Central differences: \f$ \dot{x}(t) = \frac{x_{k+1} - x_{k-1}}{2\Delta T} \f$
};
    

/** Enumeration for different hessian calculation methods (not necessary for least-squares solver) */
enum class HessianMethod
{
    NUMERIC, //!< Calculate hessian numerically via difference quotients (based on graph structure)
    BLOCK_BFGS, //!< Calculate hessian for each block (defined by the graph) via BFGS.
    FULL_BFGS, //!< Calculate the hessian via BFGS quasi-netwon method without considering strucutre.
    FULL_BFGS_WITH_STRUCTURE_FILTER, //!< Calculate hessian like with FULL_BFGS but consider structure for non-zeros.
    ZERO_HESSIAN, //!< Set Hessian to zero
    GAUSS_NEWTON //!< Set \f$ \mathbf{H} \approx \mathbf{J}^T \mathbf{J}, but use it only for XXX_SQUARED function types
};
    
/** Enumeration for the hessian initialization (relevant for BFGS hessian approximations) */
enum class HessianInit
{
    ZERO, //!< Use the zero matrix for initialization
    IDENTITY, //!< Use the identity matrix for hessian initialization (scale it via seperate config parameter)
    NUMERIC //!< Calculate the initialization of the hessian numerically using the original problem
};
    
    
/** Enumeration for different solver algorithms supported by the Nlopt library (only those that are working here)*/
enum class NloptAlgorithms
{
    SLSQP //!< Sequential quadratic programming approach
};


/**
 * @brief Configurations for Controller and Solver classes.
 * 
 * @ingroup controller solver
 *  
 * This class defines the main config for controller and solver classes.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa BaseController TebController BaseSolver
 */
class Config
{
public:
    
    //! Configurations related to the TEB algorithm
    struct Teb
    {
        unsigned int n_min = 3; //!< Minimum number of samples (states and control input pairs) allowed.
        unsigned int n_pre = 20; //!< Initial number of samples (used in TebController::updateGoal()).
        unsigned int n_max = 150; //!< Maximum number of samples allowed.
        double dt_min = 0.01; //!< Minimum dt (only considered if EdgePositiveTime is activated).
        double dt_ref = 0.1; //!< Desired reference sampling time (the TEB tries to converge towards dt_ref) (make sure dt_ref>0).
        double dt_hyst = 0.01; //!< Sampling time hysteresis used in resizeTEB to avoid oscillations (default: 0.1*dt_ref).
        double goal_dist_force_reinit = 0.5; //!< Force reinit of the trajectory if distance between current and new goal is above threshold.
        unsigned int teb_iter = 5; //!< Number of outer-loop TEB iterations (used for guiding \f$ \Delta T \f$ towards dt_ref, see TebController::optimizeTEB()).
        FiniteDifferences diff_method = FiniteDifferences::FORWARD; //!< Select finite difference method for discretizing system dynamics and obtaining \f$ \Delta T \f$.
	
    } teb; //!< Configurations related to the TEB algorithm
   
    //! Configurations related to the trajectory optimization
    struct Optim
    {
        //! Configurations related to solvers
        struct Solver
        {
            unsigned int solver_iter = 5;
            
	    //! Configurations especially for least-squares-solvers
            struct Lsq
            {
                double soft_constr_epsilon = 0.; //!< Add a bigger margin to soft-constraints @bug Do not use at the moment, because it is not implemented correctly
                double weight_equalities = 2; //!< Soft-constraint weight for equality constraints.
                double weight_inequalities = 2; //!< Soft-constraint weight for inequality constraints.
                double weight_adaptation_factor = 2; //!< Factor for soft-constraint weight adapation (see BaseSolverLeastSquares::adaptWeights()).
            } lsq; //!< Configurations especially for least-squares-solvers
            
            
            //! Settings for constrained optimiziation solver (nonlinear program solver)
            struct NonlinearProgram
            {
	        //! Configurations for the hessian of the lagrangian calculation
                struct Hessian
                {
                    HessianMethod hessian_method = HessianMethod::NUMERIC; //!< Define the hessian calcuation methods (numeric, bfgs quasi-newton, ...)
                    HessianInit hessian_init = HessianInit::NUMERIC; //!< Define which hessian initialization should be used in BFGS methods
                    double hessian_init_identity_scale = 1; //!< Scale the hessian initialization matrix if hessian_init is set to HessianInit::IDENTITY
                    bool bfgs_damped_mode = true; //!< Specify if BFGS damped mode should be used (slower, but more robust).
                } hessian; //!< Configurations for the hessian of the lagrangian calculation
                
                //! Settings for the line-search algorithm (force merit function descent)
                struct LineSearch
                {
                    double sigma = 0.5; // Scale projected gradient in backtracking line-search: [0,1]
                    double beta = 0.5; // Reduction factor of the optimization result (increment) in each step: should be in range [0,1]
                    double alpha_init = 0.1; // Initial parameter of the l1 exact merit function
                } linesearch; //!< Settings for the line-search algorithm (force merit function descent)
            } nonlin_prog; //!< Settings for constrained optimiziation solver (nonlinear program solver)
            
            
            //! Configurations especially for the nlopt solver wrapper
            struct Nlopt
            {	     
	      NloptAlgorithms algorithm = NloptAlgorithms::SLSQP;
	      
	      double tolerance_equalities = 0.001; //!< Stop optimization depending of the equality constraint satisfaction tolerance
	      double tolerance_inequalities = 0.001; //!< Stop optimization depending of the inequality constraint satisfaction tolerance
	      
	      double stopping_criteria_ftol_abs = 0.0001; //!< Stop optimization depending on the absolute value of the objective function.
	      double stopping_criteria_ftol_rel = 0.0001; //!< Stop optimization depending on the relative change of the objective function value.
	      double stopping_criteria_xtol_abs = 0.0001; //!< Stop optimization depending on the absolute values of the optimization vector.
	      double stopping_criteria_xtol_rel = 0.0001; //!< Stop optimization depending on the relative change of values of the optimization vector.
	      double max_optimization_time = 5; //!< Stop optimization after a given duration in seconds.
	      
	    } nlopt;
            
            
        } solver; //!< Configurations related to solvers
        
        bool force_rebuild_optim_graph = false; //!< Disable hot-starting from previous graph structers even if no changes to the graph are made.
   
    } optim; //!< Configurations related to the trajectory optimization

    //! Configurations for helper miscellaneous and utilities
    struct Utilities
    {
    } util; //!< Configurations for helper miscellaneous and utilities
 
};

    
} // end namespace teb



#endif /* defined(__teb_package__config__) */
