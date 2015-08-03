#include <teb_package/base/base_solver_least_squares.h>


namespace teb
{

    
/**
 * This method constructs the full value/cost vector \f$ \mathbf{f} \f$ (not \f$ \mathbf{f}^2 \f$)
 * for the underlying least-squares optimization problem. The lenght of the vector can be obtained
 * using getValueDimension().
 *
 * Constraints are transformed into costs/objectives using soft constraints:
 * - Equality constraints \f$ c(x) = 0 \f$. \n
 *   These constraints are directly taken as objectives since \f$ c^2(x) \f$ has a local minima at \f$ x = 0 \f$.
 * - Inequality constraints \f$ c(x) \le 0 \f$ \n
 *   These constraints are transformed using \f$ f = \max{(c(x),0)} \f$ (component-wise). \n
 *   This notation is similar to: c(x) <= epsilon ? 0 : c(x). \n
 *   Afterwards (in the actual solveImpl() method, the cost function will be squared, that leads to twice differnetiable
 *   costs for the constraints, if \f$ f \f$ is piecewise differentiable once and if it intersectects with the abscissa.
 *
 * In addition the weights BaseSolverLeastSquares::_weight_equalities and BaseSolverLeastSquares::_weight_inequalities
 * are taken into account in order to weight the "soft constraint objectives".
 * To make the weights comparable to \cite kummerle2011 and our Matlab TEB version, we take the square root of both weights,
 * since the least-square problem here is formulated as \f$ f^2(x,\sigma) \f$ instead of \f$ \sigma f^2(x) \f$. \f$ \sigma \f$ denotes the weight.
 *
 * The results are stored internally to BaseSolverLeastSquares::_values.
 * 
 * @sa adaptWeights(), getValueDimension()
 */
void BaseSolverLeastSquares::buildValueVector()
{
    // resize rhs to current problem size
    _values.resize(_val_dim);
    
    int idx = 0;
    for (EdgeType* edge : *_objectives)
    {
        edge->computeValues();
        _values.segment(idx,edge->dimension()) = edge->valuesMap();
        idx += edge->dimension();
    };
    
    
    
    
    for (EdgeType* edge : *_equalities)
    {
        edge->computeValues();
        _values.segment(idx,edge->dimension()) = edge->valuesMap() * sqrt(_weight_equalities) * sqrt(edge->getEdgeWeight()); // sqrt(weight) only to make it comparable to matlab version
        idx += edge->dimension();
    };
    // in case of inequalities, we need to calculate soft constraints. see jacobian information.
    // if c(x)<=0 costs are zero, otherwise we have costs (that can be negative, since they will be squared)
    // actually we check c(x)<=epsilon to shift the boarder towards the safe region (soft constraints could be violated)
    for (EdgeType* edge : *_inequalities)
    {
        edge->computeValues();
	
	//PRINT_INFO("Data: " << edge->valuesMap().array());
	//PRINT_INFO("dim: " << edge->dimension());
        _values.segment(idx,edge->dimension()) = (edge->valuesMap().array() <= -cfg->optim.solver.lsq.soft_constr_epsilon).select(0., edge->valuesMap() * sqrt(_weight_inequalities) * sqrt(edge->getEdgeWeight())); // c(x)<=epsilon ? 0 : c(x) ; same as max(c(x),0), // sqrt(weight) only to make it comparable to matlab version
        idx += edge->dimension();
	//PRINT_INFO("Values: " << _values.coeffRef(idx,edge->dimension()));
    };
    
}
    

/**
 * This method returns the dimension of the full cost/value vector including
 * all objective and constraints.
 * In particular it sums up all dimensions of single edges stored in the given hyper-graph.
 *
 * @remarks Make sure BaseSolver::_objective_dim, BaseSolver::_equalities_dim and BaseSolver::_inequalities_dim are valid (see solve()).
 * 
 * @return dimension of the complete value/cost vector.
 * 
 * @sa buildValueVector(), getOptVecDimension(), getDimObjectives(), getDimEqualities(), getDimInequalities()
 */
int BaseSolverLeastSquares::getValueDimension() const
{
    return _objective_dim + _equalities_dim + _inequalities_dim;
}


 
/**
 * This method increases the soft constraint weights for inequalities
 * and equalities after each outer TEB optimization loop (see TebController::optimizeTEB()).
 * After the outer TEB loop is completed (Config::Teb::teb_iter), the weights are set back to
 * Config::Optim::Solver::Lsq::weight_equalities and Config::Optim::Solver::Lsq::weight_inequalities.
 *
 * The weights are increased by \f$ \sigma = \sigma_{init} \cdot \gamma^i \f$.
 * \f$ \gamma \f$ denotes the increasement factor Config::Optim::Solver::Lsq::weight_adaptation_factor.
 * \f$ i \f$ denotes the current iteration number. \f$ \sigma_{init} \f$ is obtained from the config (see above).
 *
 * Call this method at the beginning of solveImpl()!
 *
 * @remarks This procedere forces the optimization result to first contract the trajectory in sense of the objective,
 *          and afterwards trying to satisfy the constraints.
 *          Starting with very high weights in advance could lead to abrupt gradients for the solver.
 *          In addition the effect for the objectives (e.g. time optimallity) could be slow, if the corresponding gradients
 *          are extremly small in comparision to constraint gradients.
 *
 * @sa TebController::optimizeTEB()
 */
void BaseSolverLeastSquares::adaptWeights()
{
    if (_weight_adapt_count>=int(cfg->teb.teb_iter)-1)
    {
        _weight_adapt_count = 0; // reset weights after all inner solver_iterations are performed once
                                 // assuming that adaptWeights() is only called once within each iteration
    }
    if (_weight_adapt_count==0)
    {
        // use weights from config
        _weight_equalities = cfg->optim.solver.lsq.weight_equalities;
        _weight_inequalities = cfg->optim.solver.lsq.weight_inequalities;
    }
    else
    {
        // scale weights by a given adaptation factor (cfg)
        _weight_equalities *= cfg->optim.solver.lsq.weight_adaptation_factor;
        _weight_inequalities *= cfg->optim.solver.lsq.weight_adaptation_factor;
    }
    ++_weight_adapt_count;
}
    
} // end namespace teb



