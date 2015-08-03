#ifndef __teb_package__examples_rob_cost_edges_obstacles__
#define __teb_package__examples_rob_cost_edges_obstacles__


#include <teb_package.h>


namespace teb
{
                       
    // Obstacle-Avoidance (point-representation)
    class EdgeObstacle : public BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<4>> // Cost vector dimension: 1, Vertices: StateVertex
    {
    public:
        
        //! Empty Constructor
	EdgeObstacle(StateVertex<4>& state) : BaseEdge<1, FUNCT_TYPE::NONLINEAR, StateVertex<4>>(state) {}
        
        virtual void computeValues() // See EdgeType::computeValues()
        {
	    StateVertex<4>* sample_i = static_cast<StateVertex<4>*>(_vertices[0]);
            
            // Direction vector between sample and obstacle:
            Eigen::Vector2d deltaS = sample_i->states().head(2) - obstacle_position;
            //double angle_i = sample_i->states()[2];
        
            // Projection of the distance deltaS orthogonal to the TEB
	    //double angdiff = atan2(deltaS[1],deltaS[0])-angle_i;
	    double proj_dist = deltaS.norm();//*fabs(sin(angdiff));
    
	    // Constraint proj_dist > min_distance -> -proj_dist + min_distance < 0
            _values[0] = -proj_dist + min_distance;
        }
           
        // Hessian and Jacobians are calculated numerically, otherwise add implementation here.
            
            
        Eigen::Vector2d obstacle_position; // DO NOT FORGET TO SET VALUE
        double min_distance = 0.5;
            
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
        


} // end namespace teb



#endif /* defined(__teb_package__examples_rob_cost_edges_obstacles__) */
