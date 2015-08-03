#ifndef __teb_package__base_solver__
#define __teb_package__base_solver__

#include <teb_package/base/typedefs.h>
#include <teb_package/base/config.h>
#include <teb_package/base/graph.h>
#include <memory>
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace teb
{

  
/**
 * @brief Base class for solver implementations
 *
 * @ingroup solver
 * 
 * This abstract class defines the interface for general solvers that are
 * suited for solving the TebController optimization problem. \n
 * The solver needs to handle the optimziation problem as a
 * hyper-graph (see EdgeType and VertexType).
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */    
class BaseSolver
{
public:
  
    //! Typedef for vertex containers that stores all vertices required for the optimization (Hyper-Graph).
    using VertexContainer = HyperGraph::VertexContainer;
    //! Typedef for edge containers that stores all edges required for the optimization (Hyper-Graph).
    using EdgeContainer = HyperGraph::EdgeContainer;
    
    //! Empty Constructor
    BaseSolver() {}
    
    //! Empty Destructor
    virtual ~BaseSolver() {}
    
    /**
     * @brief Set config including solver settings.
     * @remarks This function is called from the TEB class within setSolver method.
     * @param config Pointer to Config object.
     */
    void setConfig(const Config* config) {cfg = config;}
    
    /**
     * @brief Solve an optimization problem defined by an hyper-graph.
     * 
     * The method copies pointers to the active vertices and edges (objectives, equality constraints and inequality constraints)
     * and calls the actual solver with solveImpl().
     * 
     * @todo Split initWorkspaces into init and update phase in order to hot-start from previous initializations.
     * 
     * @param optimizable_graph pointer to the HyperGraph that should be solved
     */
    virtual bool solve(HyperGraph* optimizable_graph)
    {


      // copy pointers to arguments for internal use later
        
      _objectives = &(optimizable_graph->objectives());
      _equalities = &(optimizable_graph->equalities());
      _inequalities = &(optimizable_graph->inequalities());
      _active_vertices = &(optimizable_graph->activeVertices());
      
	  // Check if we have a someting to optimize
	  assert(!_active_vertices->empty());
	  assert(!_objectives->empty() || !_equalities->empty() || !_inequalities->empty());

      // Get information about a possibly changed graph
      _graph_structure_modified = optimizable_graph->isGraphModified();
           
      // Get optimization vector dimension
      _opt_vec_dim = getOptVecDimension();
      
	  assert(_opt_vec_dim > 0 && "solve() - the dimension of the opt_vec_dim is zero. Maybe you forgot to set indices for the optimization vector in each vertex... call getActiveVertices()");

      // Get dimensions for objectives and constraints
      _objective_dim = getDimObjectives();
      _equalities_dim = getDimEqualities();
      _inequalities_dim = getDimInequalities();
      
      // initialize jacobian and hessian workspaces
      initWorkspaces();
      
      // call actual solve function
      bool ret_val = solveImpl();
        
      // check if there are unrestored backups
      while(_no_vert_backups>0) {discardBackupVertices();}
        
      return ret_val;
    }
    
   
    const Config* cfg = nullptr; //!< Store pointer to Config object @see setConfig()
    
    
    
protected:
  /**
   * @brief Solve the actual optimization problem
   * 
   * Store results to the vertices of the active vertices container.
   *
   * @retval true optimization was successfull.
   * @retval false optimization was not successfull.
   */
  virtual bool solveImpl() = 0;
  
  /**
   * @brief Initialize workspaces for the block jacobians and hessians.
   * Implement this function to allocate new memory for the jacobian and hessian workspaces
   * (as part of each EdgeType) or map to existing memory.
   */
  virtual void initWorkspaces()
  {
    // Implement if you want to use edge jacobians and hessians!!!
  };
  
   /**
    * @brief Apply a new increment optained by a local optimization to the active vertices
    *
    * This method iterates the active vertices container and calls
    * VertexType::plusFree() for each vertex.
    * @param delta Eigen::Vector containing all increments. The length euquals the dimension of all free variables.
    */
   void applyIncrement(const Eigen::Ref<const Eigen::VectorXd>& delta)
   {
       int idx=0;
       for (VertexType* vertex : *_active_vertices)
       {
           vertex->plusFree(delta.data()+idx);
           idx += vertex->dimensionFree();
       }
   }
   
   /**
    * @brief Overwrite TEB states and control inputs with new values
    *
    * This method iterates the active vertices container and calls
    * VertexType::setFree() for each vertex.
    * @param opt_vec Eigen::Vector containing all values. The length equals the dimension of all free variables.
    */
   void applyOptVec(const Eigen::Ref<const Eigen::VectorXd>& opt_vec)
   {
       int idx=0;
       for (VertexType* vertex : *_active_vertices)
       {
           vertex->setFree(opt_vec.data()+idx);
           idx += vertex->dimensionFree();
       }
   }
   
   /**
    * @brief Get a copy of the optimization vector (TEB states, control inputs and dt)
    *
    * This method iterates the active vertices container and calls
    * VertexType::getDataFree() for each vertex.
    * @remarks Make sure BaseSolver::_opt_vec_dim is valid (see solve()).
    * @return Eigen::Vector containing all values. The length equals the dimension of all free variables (getOptVecDimension()).
    */
   Eigen::VectorXd getOptVecCopy() const
   {
      Eigen::VectorXd opt_vec(_opt_vec_dim); // Check whether this will be optimized by the compiler (since a copy will be returned as well).
       int idx=0;
       for (const VertexType* vertex : *_active_vertices)
       {
	   assert(idx < _opt_vec_dim);
           vertex->getDataFree( opt_vec.data()+idx );
           idx += vertex->dimensionFree();
       }
       return opt_vec;
   }
    
    /**
     * @brief Get dimension of the optimization vector (that equals the number of cols and rows in the Hessian).
     * The dimension is obtained by checking the previously determined index of the last active vertice stored in the graph.
     * @sa TebController::getActiveVertices(), getValueDimension()
     */
    int getOptVecDimension() const
    {
        assert(_active_vertices);
        return _active_vertices->back()->getOptVecIdx() + _active_vertices->back()->dimensionFree();
    }
    
    /**
     * @brief Get dimension of the objective function.
     * The dimension is obtained by checking each edge dimension.
     */
    int getDimObjectives() const
    {
        assert(_objectives);
	int ret_val = 0;
	for (const EdgeType* edge : *_objectives) {ret_val += edge->dimension();};
	return ret_val;
    }
    
    /**
     * @brief Get dimension of the equality constraints.
     * The dimension is obtained by checking each edge dimension.
     */  
    int getDimEqualities() const
    {
        assert(_equalities);
	int ret_val = 0;
	for (const EdgeType* edge : *_equalities) {ret_val += edge->dimension();};
	return ret_val;
    }
    
    /**
     * @brief Get dimension of the inequality constraints.
     * The dimension is obtained by checking each edge dimension.
     */  
    int getDimInequalities() const
    {
        assert(_inequalities);
	int ret_val = 0;
	for (const EdgeType* edge : *_inequalities) {ret_val += edge->dimension();};
	return ret_val;
    }
    
   //! Backup all active vertices @see VertexType::push()
   void backupVertices() { for (VertexType* vertex : *_active_vertices) vertex->push(); ++_no_vert_backups;}
   //! Restore all values of active vertices from the backup stacks @see VertexType::pop()
   void restoreVertices() { for (VertexType* vertex : *_active_vertices) vertex->pop(); --_no_vert_backups;}
   //! Restore all values of active vertices from the backup stacks WITHOUT discarding the backup @see VertexType::top()
   void restoreVerticesButKeepBackup() { for (VertexType* vertex : *_active_vertices) vertex->top(); }
   //! Discard all values of active vertices from the backup stacks @see VertexType::discardTop()
   void discardBackupVertices() { for (VertexType* vertex : *_active_vertices) vertex->discardTop(); --_no_vert_backups;}
    
    
  
  VertexContainer* _active_vertices = nullptr; //!< Pointer to active vertex container (active = non-fixed)
  EdgeContainer* _objectives = nullptr; //!< Pointer to edges representing objective functions
  EdgeContainer* _equalities = nullptr; //!< Pointer to edges representing equality constraints
  EdgeContainer* _inequalities = nullptr; //!< Pointer to edges representing inequality constraints
    
  int _opt_vec_dim = -1; //!< Store dimension of the optimziation vector here (see getOptVecDimension()). 
  int _objective_dim = -1; //!< Store dimension of the objective function (see getDimObjectives()). 
  int _equalities_dim = -1; //!< Store dimension of the equality constraints (see getDimEqualities()). 
  int _inequalities_dim = -1; //!< Store dimension of the inequality constraints (see getDimInequalities()). 
  
  unsigned int _no_vert_backups = 0; //!< Track number of vertices backups made
    
   /** 
    * @brief Mark if a new graph structure is available, thus we need to reinitialze all workspaces.
    * In addition some solver can implement hotstarting and decide whether hotstart or not using this flag.
    * It is obtained from the graph passed to the solve() method.
    */
  bool _graph_structure_modified = true;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


} // end namespace teb

#endif /* defined(__teb_package__base_solver__) */
