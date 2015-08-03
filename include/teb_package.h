#ifndef __teb_package__teb_package__
#define __teb_package__teb_package__


#include <teb_package/base/typedefs.h>
#include <teb_package/utilities/initial_final_states.h>
#include <teb_package/utilities/car_like_type_driving.h>
#include <teb_package/utilities/utilities.h>
#include <teb_package/base/obstacle.h>
#include <teb_package/base/workspaces.h>
#include <teb_package/base/teb_controller.h>
#include <teb_package/base/base_edge.h>
#include <teb_package/base/bound_constraints.h>
#include <teb_package/base/common_teb_edges.h>
#include <teb_package/base/system_dynamics.h>
#include <teb_package/solver/solver_levenbergmarquardt_eigen_dense.h>
#include <teb_package/solver/solver_levenbergmarquardt_eigen_sparse.h>
#include <teb_package/simulation/simulator.h>
#include <teb_package/visualization/teb_plotter.h>
#include <teb_package/visualization/teb_robot_footprint.h>

#include <teb_package/solver/solver_sqp_dense.h>
#include <teb_package/solver/solver_sqp_local_dense.h>

// optional solvers
#ifdef NLOPT
#include <teb_package/solver/solver_nlopt_package.h>
#endif



#endif /* defined(__teb_package__teb_package__) */