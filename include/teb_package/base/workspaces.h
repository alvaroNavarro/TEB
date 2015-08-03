#ifndef __teb_package__workspaces__
#define __teb_package__workspaces__

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
//#include <teb_package/base/teb_vertices.h>

namespace teb
{

/**
 * @brief Memory management and accessor functions for the Block-Jacobians (for each edge).
 * 
 * @ingroup controller solver
 *  
 * This class controls the memory management of Block-Jacobians that are calculated for
 * each edge. 
 * Each edge instantiates its own JacobianWorkspace and allocates the memory
 * required to store all entries. Since the size depends on the type and number of attached
 * vertices and the error dimension, it is not known a-priori. This workspace helps
 * managing memory and accessing elements at run-time.
 * 
 * @remarks For each vertex attached to a subclass of BaseEdge, a single Jacobian of
 * 	    the size [ BaseEdge::Dimension x TebVertex::dimension() ] is required. \n
 *          Assume that a vertex has the following unfixed variables: [x1, x2, x3]
 * 	    and an edge has the cost values [v1, v2]: 
 * 	    \f[ \mathbf{J}_{block} =  
 * 		\begin{bmatrix}
 * 		   \partial_{x1} \ v1 & \partial_{x2} \ v1 & \partial_{x3} \ v1 \\
 * 		   \partial_{x1} \ v2 & \partial_{x2} \ v2 & \partial_{x3} \ v2 
 * 		\end{bmatrix}
 *          \f]
 * 	    Let the vertex be the first one of the corresponding edge. 
 * 	    The above Block-Jacobian can be stored from an edge (after calling allocate() once) as follows:
 * 	   \code 
 * 		this->getWorkspace(0) = J_block; // exchange "this" by a pointer/ref to a valid workspace instance with allocated and reserved memory.
 * 	   \endcode
 *
 * @todo TebVertex::dimension() is used at the moment. Acutally only a number of TebVertex::dimensionFree()
 *       non-zeros may appear. Test if it speeds up the optimization to skip those values by using the fixed-mask.
 *	 But for common MPC problems only the first or/and the final state are fixed partially.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa BaseEdge, HessianWorkspace
 */
class JacobianWorkspace
{
public:
    /** Typedef for the BlockJacobian container. A single block-matrix per attached TebVertex is required. */
    using WorkspaceMatrix = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    
    JacobianWorkspace() {}; //!< Empty constructor
    ~JacobianWorkspace() {}; //!< Empty destructor
    
    /**
     * @brief Allocate memory for all block-Jacobians.
     * @param edge_dim Dimension of the edge. Typically BaseEdge::Dimension().
     * @param vertices_dim Vector of dimensions of the attached vertices. Typically TebVertex::dimension() each.
     * @tparam T storage class (std::vector, std::array, std::deque)
     * @retval true Status flag. Currently it returns only \c true. 
     */
    template <typename T>
    bool allocate(int edge_dim, const T& vertices_dim)
    {
      _workspace.clear();
        //_workspace.resize(vertices_dim.size()); // resize to store all vertices that are attached to current edge
        for (unsigned int i=0; i<vertices_dim.size(); ++i)
        {
	    assert(vertices_dim.at(0)>0);
            _workspace.push_back(Eigen::MatrixXd::Zero(edge_dim, vertices_dim.at(i)));
        }
        return true;
    }
    
    /**
     * @brief Access Block-Jacobian for TebVertex with index \c vertex_idx.
     * @param vertex_idx Index of the corresponding vertex.
     * @return Reference to Block-Jacobian of vertex with index \c vertex_idx.
     */
    Eigen::MatrixXd& getWorkspace(int vertex_idx) {return _workspace.at(vertex_idx);};
    
protected:
    WorkspaceMatrix _workspace; //!< Block-Jacobian container.
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
};
    
    


/**
 * @brief Memory management and accessor functions for the Block-Hessian (for each edge).
 * 
 * @ingroup controller solver
 *  
 * This class controls the memory management of Block-Hessians that are calculated for
 * each edge (depends on the BaseSolver subclass, if Hessians are required). 
 * Each edge instantiates its own HessianWorkspace and allocates the memory
 * required to store all entries. Since the size depends on the type and number of attached
 * vertices, it is not known a-priori. This workspace helps managing memory and accessing elements at run-time.
 * 
 * @remarks For each vertex attached to a subclass of BaseEdge and addition for each cost-value (!), a single Hessian of
 * 	    the size [ TebVertex::dimension() x TebVertex::dimension() ] is required. \n
 *  	    Assume that there exist a vertex1 with the following unfixed variables: [x1, x2, x3].
 * 	    In addition let vertex2 be a vertex with the unfixed variables: [x4,x5].
 * 	    For the an edge with two cost/constraint values [v1, v2], the corresponding Block-Hessians are as follow: 
 * 	    \f[ \mathbf{H}_{block1,vertex1} =  
 * 		\begin{bmatrix}
 * 		   \partial^2_{x1,x1} \ v1 & \partial^2_{x1,x2} \ v1  & \partial^2_{x1,x3} \ v1 \\
 * 		   \partial^2_{x2,x1} \ v1 & \partial^2_{x2,x2} \ v1  & \partial^2_{x2,x3} \ v1 \\
 * 		   \partial^2_{x3,x1} \ v1 & \partial^2_{x3,x2} \ v1  & \partial^2_{x3,x3} \ v1
 * 		\end{bmatrix}
 *          \f]
 *   	    \f[ \mathbf{H}_{block1,vertex2} =  
 * 		\begin{bmatrix}
 * 		   \partial^2_{x4,x4} \ v1 & \partial^2_{x4,x5} \ v1  \\
 * 		   \partial^2_{x5,x4} \ v1 & \partial^2_{x5,x5} \ v1 
 * 		\end{bmatrix} 
 * 	    \f]
 *  	    \f[ \mathbf{H}_{block2,vertex1} =  
 * 		\begin{bmatrix}
 * 		   \partial^2_{x1,x1} \ v2 & \partial^2_{x1,x2} \ v2  & \partial^2_{x1,x3} \ v2 \\
 * 		   \partial^2_{x2,x1} \ v2 & \partial^2_{x2,x2} \ v2  & \partial^2_{x2,x3} \ v2 \\
 * 		   \partial^2_{x3,x1} \ v2 & \partial^2_{x3,x2} \ v2  & \partial^2_{x3,x3} \ v2
 * 		\end{bmatrix}
 *          \f]
 * 	    \f[ \mathbf{H}_{block2,vertex2} =  
 * 		\begin{bmatrix}
 * 		   \partial^2_{x4,x4} \ v2 & \partial^2_{x4,x5} \ v2  \\
 * 		   \partial^2_{x5,x4} \ v2 & \partial^2_{x5,x5} \ v2 
 * 		\end{bmatrix} 
 * 	    \f]
 * 	    The above Block-Hessians can be stored from the edge class (after calling allocate() once) as follows:
 * 	   \code 
 * 		this->getWorkspace(0,0) = H_block1_vertex1; // exchange "this" by a pointer/ref to a valid workspace instance with allocated and reserved memory.
 * 		this->getWorkspace(1,0) = H_block1_vertex2; 
 * 		this->getWorkspace(1,1) = H_block2_vertex1; 
 * 		this->getWorkspace(1,1) = H_block2_vertex2; 
 * 	   \endcode
 *
 * @todo TebVertex::dimension() is used at the moment. Acutally only a number of TebVertex::dimensionFree()
 *       non-zeros may appear. Test if it speeds up the optimization to skip those values by using the fixed-mask.
 *	 But for common MPC problems only the first or/and the final state are fixed partially.
 * 
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 *  
 * @sa BaseEdge, JacobianWorkspace
 */
class HessianWorkspace
{
public:
    /** Typedef for the BlockHessian container. A single block-matrix per attached TebVertex and per Cost-value is required. */
    using WorkspaceMatrix = std::vector<std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>>;
    
    HessianWorkspace() {}; //!< Empty constructor
    ~HessianWorkspace() {}; //!< Empty destructor
    
    /**
     * @brief Allocate memory for all block-Jacobians.
     * @param edge_dim Dimension of the edge. Typically BaseEdge::Dimension().
     * @param vertices_dim Vector of dimensions of the attached vertices. Typically TebVertex::dimension() each.
     * @tparam T storage class (std::vector, std::array, std::deque)
     * @retval true Status flag. Currently it returns only \c true. 
     */
    template <typename T>
    bool allocate(int edge_dim, const T& vertices_dim)
    {
        _workspace.resize(vertices_dim.size());
        for (unsigned int i=0; i<vertices_dim.size(); ++i)
        {
            _workspace.at(i).resize(edge_dim);
            for (int j=0; j<edge_dim; ++j)
                _workspace.at(i).at(j) = Eigen::MatrixXd::Zero(vertices_dim.at(i), vertices_dim.at(i));
        }
        return true;
    }
    
    /**
     * @brief Access Block-Hessian for TebVertex with index \c vertex_idx.
     * @param vertex_idx Index of the corresponding vertex.
     * @param value_idx Index of the corresponding cost-value.
     * @return Reference to Block-Jacobian of vertex with index \c vertex_idx and cost value with index \c value_idx.
     */
    Eigen::MatrixXd& getWorkspace(int vertex_idx, int value_idx) {return _workspace.at(vertex_idx).at(value_idx);};
    
protected:
    WorkspaceMatrix _workspace; //!< Block-Hessian container.
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
    
} // end namespace teb


#endif /* defined(__teb_package__workspaces__) */
