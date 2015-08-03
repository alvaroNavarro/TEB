#ifndef __teb_package__graph__
#define __teb_package__graph__

#include <teb_package/base/typedefs.h>
#include <teb_package/base/workspaces.h>
#include <Eigen/Core>
#include <stack>
#include <vector>
#include <memory>

namespace teb
{
  
/**
 * @brief Generic interface class for vertices.
 * 
 * @ingroup controller solver
 *  
 * This abstract class defines the interface for dedicated vertices.
 * The underlying TEB optimization problem is formulated as a hyper-graph
 * in which state vectors (StateVertex), control input vectors (ControlVertex) and a TimeDiff are
 * represented as vertices. That means, vertices have to be considered by the
 * solver as changeable quantities.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa HyperGraph, EdgeType, StateVertex, ControlVertex, TimeDiff, BaseEdge
 */  
class VertexType
{
public:
    //! Typedef for vertex containers that stores all vertices required for the optimization (Hyper-Graph).
    using VertexContainer = std::vector<VertexType*>;
    
    virtual bool isFixedAll() const = 0; //!< return \c true, if ALL values of this vertex are fixed and therefore are not necessary for optimization.
    virtual bool isFixedAny() const = 0; //!< return \c true, if ANY element/value/component of this vertex is fixed and therefore the vertex is relevant for optimization.
    virtual bool isFixedComp(int idx) const = 0; //!< return \c true, if element/value/component with index \c idx of this vertex is fixed. @param idx component index
    
    // Backup values
    virtual void push() = 0; //!< This function should store all values into a internal backup stack.
    virtual void pop() = 0; //!< This function should restore the previously stored values of the backup stack and remove them from the stack.
    virtual void top() = 0; //!< This function should restore the previously stored values of the backup stack WITHOUT removing them from the stack.
    virtual void discardTop() = 0; //!< This function should delete the previously made backup from the stack without restoring it.
    virtual unsigned int stackSize() const = 0; //!< This function should return the current size/number of backups of the backup stack.
    
    virtual int dimension() const = 0; //!< Return number of elements/values/components stored in this vertex.
    virtual int dimensionFree() const = 0; //!< Return only the dimension of free (unfixed) variables
   
    /**
     * @brief Update the position of the first value of this vertex inside the optimization vector / Hessian.
     * @param opt_vec_idx index of the first value
     */
    void setOptVecIdx(int opt_vec_idx) {_opt_vec_idx = opt_vec_idx;}; 
    
    /**
     * @brief Get the position of the first value of this Vertex inside the optimization vector / Hessian.
     * @return position of the first element of this vertex.
     */
    int getOptVecIdx() const {return _opt_vec_idx;};
   
    virtual void plus(const VertexType* rhs) = 0; //!< Define the increment for the vertex: x = x + rhs
    virtual void plus(const double* rhs) = 0; //!< Define the increment for the vertex: x = x + rhs (rhs[] with dimension()=p+q values)
    virtual void plusFree(const double* rhs) = 0; //!< Define the increment for the vertex: x = x + rhs (But only add FREE variables, rhs[] with dimensionFree() values).
    virtual void setFree(const double* rhs) = 0; //!< Overwrite all free variables with the values given by rhs (rhs[] with dimensionFree() values).
    virtual void getDataFree(double* target_vec) const = 0; //!< Return a copy of the free (unfixed) values as a single array ( length: dimensionFree() ).
    virtual const double& getData(unsigned int idx) const = 0; //!< Return pointer to data with the index \c idx ).
    
protected:
    int _opt_vec_idx = -1; //!< Start-index in optimization vector (idx in hessian (row/column) and jacobian (column))
};
    

    
    
// Enum for different edge types (see EdgeType class)
//enum class EDGE_TYPE {BASE, OBJECTIVE, CONSTRAINT, BOUNDCONSTRAINT};

//! Enum for different function types (see EdgeType class)
enum class FUNCT_TYPE
{
    NONLINEAR, //!< General nonlinear function that is at least two times differentiable
    NONLINEAR_SQUARED, //!< General nonlinear function that will be squared within the solver
    NONLINEAR_ONCE_DIFF, //!< Nonlinear function that is only differentiable once: the hessian is zero
    QUADRATIC, //!< Quadratic function (hessian is constant)
    LINEAR_SQUARED, //!< Linear function that will be squared by the compiler
    LINEAR //!< Linear function (hessian is zero)
};

/**
 * @brief Generic interface class for edges.
 *
 * @ingroup controller solver
 *
 * This abstract class defines the interface for dedicated edges.
 * The underlying TEB optimization problem is formulated as a hyper-graph
 * in which state vectors (StateVertex), control input vectors (ControlVertex) and a TimeDiff are
 * represented as vertices.
 * Cost functions and constraints (that depend on vertices) are formulated as multi-edges.
 * Multi-edges can connect an arbitary number of vertices. The result graph
 * is therefore called hyper-graph. If the number of vertices attached to a single
 * edge is low, the resulting optimization problem will be sparse probably.
 * Block-Jacobians and Block-Hessians can be calculated for each edge independently
 * and may be combined afterwards by the solver.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @sa HyperGraph, VertexType, BaseEdge
 */
class EdgeType
{
    public:
  
    //! Typedef for edge containers that stores all edges required for the optimization (Hyper-Graph).
    using EdgeContainer = std::vector<EdgeType*>;
    //! Typedef for an Eigen::Vector wrapper that points to the actual data somewhere else.
    using ValueVectorMap = Eigen::Map< Eigen::VectorXd >;
    
    /**
     * @brief Store edge function type information according to the members of enum FUNCT_TYPE.
     */   
    virtual FUNCT_TYPE functType() const = 0;
    
    //! Override this method in a subclass that constitutes a bound constraint since it can be handled seperately by some solvers to speed up optimization
    virtual const bool isBoundConstraint() const {return false;}
    
    //! Constructor that updates the internal edge_type.
    EdgeType()
    {
    };
    
    virtual ~EdgeType() {}; //!< Empty destructor.
    
    virtual int dimension() const = 0; //!< Return number of scalar cost/constraint values covered by this edge.
    virtual void allocateMemory(bool skip_hessian = false) = 0; //!< Allocate memory to store cost/constraint values, jacobians and hessians
    virtual void computeValues() = 0; //!< Actual cost function or constraint function. For constraints it is computeValues() <= 0.
    virtual void computeJacobian() = 0; //!< Compute Block-Jacobian and store it to EdgeType::_jacobians according to the description in JacobianWorkspace.
    virtual void computeHessian() = 0; //!< Compute Block-Hessian and store it to EdgeType::_hessians according to the description in HessianWorkspace.
    
    virtual const double* valuesData() const = 0; //!< Return pointer to cost/constraint values [dimension() x 1] (read-only).
    virtual double* valuesData() = 0; //!< Return pointer to cost/constraint values [dimension() x 1].
    
    virtual unsigned int noVertices() const = 0; //!< Return the number of attached vertices
    virtual VertexType* getVertex(unsigned int idx) = 0; //!< Access attached vertex with index \c idx. 
    virtual const VertexType* getVertex(unsigned int idx) const = 0; //!< Access attached vertex with index \c idx (read-only). 
    
    /**
     * @brief Get cost/constraints values as Eigen type matrix.
     * return Eigen::Vector type that maps to the actual cost/constraints values
     */
    virtual ValueVectorMap valuesMap() {return ValueVectorMap(valuesData(),dimension(),1);}
    
    
    //! Use this function to return a pointer to custom data that can be used by the solver without knowing template parameters e.g.
    virtual void* getCustomData() {return nullptr;};
    
    
    virtual JacobianWorkspace& jacobians() {return _jacobians;} //!< Access the jacobian workspace of this edge.
    virtual const JacobianWorkspace& jacobians() const {return _jacobians;} //!< Access the jacobian workspace of this edge (read-only).
    virtual HessianWorkspace& hessians() {return _hessians;} //!< Access the hessian workspace of this edge.
    virtual const HessianWorkspace& hessians() const {return _hessians;} //!< Access the jacobian workspace of this edge (read-only).
        
    virtual void backupJacobian() { _jacob_backup.push(_jacobians);} //!< Make a backup of the current jacobian block matrix
    
    //! Restore jacobian block matrix from the backup stack
    virtual void restoreJacobian()  
    {
      assert(!_jacob_backup.empty());
      _jacobians = _jacob_backup.top();
      _jacob_backup.pop();
    }
    virtual void discardJacobianBackup() { assert(!_jacob_backup.empty()); _jacob_backup.pop();} //!< Discard current jacobian backup
    
    virtual JacobianWorkspace& getJacobianBackup() {return _jacob_backup.top(); };
    
   void setEdgeWeight(double w) { edge_weight = w; }
   const double getEdgeWeight() const { return edge_weight; }
    
    protected:
        
    JacobianWorkspace _jacobians; //!< Block-Jacobians of the edge (see JacobianWorkspace).
    HessianWorkspace _hessians; //!< Block-Hessians of the edge (see HessianWorkspace)
    
    BackupStackType<JacobianWorkspace> _jacob_backup;
    
    double edge_weight = 1;
        
};
    
    
/**
 * @brief Graph object that stores pointers to active vertices and edges
 *
 * @ingroup controller solver
 *
 * This class stores pointer to all active vertices and edges in the optimizable
 * hyper-graph. An instance of this class needs to be created inside the controller
 * class and afterwards it is passed to a dedicated solver class via BaseSolver::solve().
 * The solver solves the optimization problem based on the hyper-graph specified
 * within this class. \n
 * Note: Active vertices are all vertices with at least one degree of freedom.
 *
 * @remarks The graph takes ownership for the memory management of the edges. Vertices remain untouched, since
 *          they are pointing to real states in the controller class (and only to the active ones, only the vector of pointers
 *          will be cleared).
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *
 * @sa VertexType, EdgeType, StateVertex, ControlVertex, TimeDiff, BaseEdge
 */  
class HyperGraph
{
    public:
        using EdgeContainer = EdgeType::EdgeContainer;
        using VertexContainer = VertexType::VertexContainer;
    
    protected:
    
    EdgeContainer _objectives; //!< Store all edges containing local objective/cost functions (see buildOptimizationGraph()).
    EdgeContainer _constraints_eq; //!< Store all edges containing equality constraints (see buildOptimizationGraph()).
    EdgeContainer _constraints_ineq; //!< Store all edges containing inequality constraints (see buildOptimizationGraph()).
    VertexContainer _active_vertices; //!< Store all active vertices (active = at least one unfixed variable, see getActiveVertices()).
    
    bool _graph_modified = true; //!< Store status flag if the graph structure is changed or not
    
    public:
    
    /** @brief Access objective container.
     *
     *  Objectives (subclasses of BaseEdge) are inserted within TebController::buildOptimizationGraph()
     *  and TebController::customOptimizationGraph() for instance.
     *
     *  @return Reference to the objective container.
     */
    EdgeContainer& objectives() {return _objectives;};
    
    const EdgeContainer& objectives() const {return _objectives;} //!< Access all edges containing objectives (read-only)
    
    /** @brief Access equality constraint container.
     *
     *  Equality constraints (subclasses of BaseEdge) are inserted within TebController::buildOptimizationGraph()
     *  and TebController::customOptimizationGraph().
     *
     *  @return Reference to the equality constraint container.
     */
    EdgeContainer& equalities() {return _constraints_eq;};
    
    const EdgeContainer& equalities() const {return _constraints_eq;} //!< Access all edges containing inequality constraints (read-only)
    
    /** @brief Access inequality constraint container.
     *
     *  Inequality constraints (subclasses of BaseEdge) are inserted within TebController::buildOptimizationGraph()
     *  and TebController::customOptimizationGraph().
     *
     *  @return Reference to the inequality constraint container.
     */
    EdgeContainer& inequalities() {return _constraints_ineq;};
    
    const EdgeContainer& inequalities() const {return _constraints_ineq;} //!< Access all edges containing equality constraints (read-only)
    
    /** @brief Access container that stores only the active vertices of the hyper-graph
     *
     *  Active vertices are vertices that store at least one free optimization variable.
     *  That means, that iff a vertex (e.g. StateVertex, ControlVertex or TimeDiff) is completly fixed,
     * 	than it is not active.
     *  In that case the vertex could be skipped during optimization to speed up optimization time.
     *
     *  @return Reference to the active vertices container.
     */
    VertexContainer& activeVertices() {return _active_vertices;};

    const VertexContainer& activeVertices() const {return _active_vertices;} //!< Access all \b active vertices (read-only)
    
    /**
     * @brief Clears all existing edges and frees memory.
     *
     * Clears all edges (objectives, inequality constraints and equality constraints).
     * All memory will be freed.
     */
    void clearEdges()
    {
        for (EdgeType* edge : _objectives) {delete edge;};
        for (EdgeType* edge : _constraints_eq) {delete edge;};
        for (EdgeType* edge : _constraints_ineq) {delete edge;};
        _objectives.clear();
        _constraints_eq.clear();
        _constraints_ineq.clear();
        _graph_modified = true;
    };
    
    /**
     * @brief Clears all active vertices (the actual vertices remain untouched).
     */
    void clearActiveVertices()
    {
        _active_vertices.clear();
        _graph_modified = true;
    };
    
    /**
     * @brief Clear the complete graph.
     *
     * Clears all edges (objectives, inequality constraints and equality constraints).
     * All memory for the edges will be freed. The actual vertices remain untouched.
     */
    void clearGraph()
    {
        clearEdges();
        clearActiveVertices();
    };
    
    
    /**
     * @brief Add new vertex to the graph.
     * The graph does not take over memory management for vertices.
     * @param vertex Pointer to the VertexType subclass.
     */
    void addActiveVertex(VertexType* vertex)
    {
        _active_vertices.push_back(vertex);
    }
    
    /**
     * @brief Add new objective edge to the graph.
     * The graph takes care about memory management for this edge.
     * @param edge Pointer to the EdgeType subclass.
     */
    void addEdgeObjective(EdgeType* edge)
    {
        //PRINT_DEBUG_COND_ONCE( edge->edgeType() != EDGE_TYPE::OBJECTIVE , "HyperGraph::addEdgeObjective() - Warning: You are adding an objective edge to the hyper-graph that is not marked as EDGE_TYPE::OBJECTIVE. Its type id is: " << (int) edge->edgeType())
        _objectives.push_back(edge);
    }
    
    /**
     * @brief Add new inequality constraint edge to the graph.
     * The graph takes care about memory management for this edge.
     * @param edge Pointer to the EdgeType subclass.
     */
    void addEdgeInequality(EdgeType* edge)
    {
        //PRINT_DEBUG_COND_ONCE( edge->edgeType() != EDGE_TYPE::CONSTRAINT && edge->edgeType() != EDGE_TYPE::BOUNDCONSTRAINT, "HyperGraph::addEdgeInequality() - Warning: You are adding an inequality edge to the hyper-graph that is not marked as EDGE_TYPE::CONSTRAINT or EDGE_TYPE::BOUNDCONSTRAINT. Its type id is: " << (int) edge->edgeType())
        _constraints_ineq.push_back(edge);
    }
    
    /**
     * @brief Add new equality constraint edge to the graph.
     * The graph takes care about memory management for this edge.
     * @param edge Pointer to the EdgeType subclass.
     */
    void addEdgeEquality(EdgeType* edge)
    {
        //PRINT_DEBUG_COND_ONCE( edge->edgeType() != EDGE_TYPE::CONSTRAINT , "HyperGraph::addEdgeEquality() - Warning: You are adding an equality edge to the hyper-graph that is not marked as EDGE_TYPE::CONSTRAINT. Its type id is: " << (int) edge->edgeType())
        _constraints_eq.push_back(edge);
    }
    
    /**
     * @brief Set status of the graph to modified.
     * The status is tracked in order to allow the controller class
     * and the solver to hotstart from previous settings. If the structure
     * of the optimization problem remains unchanged, one do not need to collect
     * all active edges again. Therefore call this function whenever the graph structure is changed
     * (e.g. by adding/removing edges or vertices).
     * @param modified If \c true the graph is modified, otherwise reset the status.
     */
    void notifyGraphModified(bool modified = true)
    {
        _graph_modified = modified;
    }
    
    /**
     * @brief Return status that tracks if the graph has been modified
     * The status is tracked in order to allow the controller class
     * and the solver to hotstart from previous settings. If the structure
     * of the optimization problem remains unchanged, one do not need to collect
     * all active edges again. Therefore check the status using this function 
     * whenever vertices and edges are processed.
     * @return \c true if the graph has been modified
     */
    bool isGraphModified() const
    {
        return _graph_modified;
    }
    
    
};
    
    

} // end namespace teb

#endif /* defined(__teb_package__graph__) */
