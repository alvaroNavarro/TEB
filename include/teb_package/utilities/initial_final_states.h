
#ifndef __teb_package__initial_final_states__
#define __teb_package__initial_final_states__

#include <Eigen/Core>

namespace teb
{

    class InitialFinalStates
    {
      
    public:      
      
      bool getInitialFinalState(double* x0, double* xf,int scenario)
      {
	bool ok = true;     
	
	if(scenario > _x0.rows() || scenario > _xf.rows())  ok = false;
	    
	Eigen::Map<Eigen::RowVector4d>(x0,1,4) = _x0.row(scenario);
	Eigen::Map<Eigen::RowVector4d>(xf,1,4) = _xf.row(scenario);
		  
	return ok;
      }
      
    protected:

    const Eigen::Matrix<double,10,4> _x0 = (Eigen::Matrix<double,10,4>() <<       0,   0,   0,        0, 		//! 1
										0,   0,   0,        0,              //! 2
										0,   0,  -M_PI/4,   0,		//! 3
										0,   0, -2*M_PI/3,  0,		//! 4
										0,   0, -M_PI,      0,		//! 5
										4,  -3, -5*M_PI/6,  0,		//! 6
										0,   0,  M_PI,  M_PI/4,             //! 7
										-4, -5, 5*M_PI/6,   0,           //! 8
										-7,   0,   0,        0,		//! 9
										0,   0,   M_PI/4,   0).finished();   //! 10
										
    const Eigen::Matrix<double,10,4> _xf = (Eigen::Matrix<double,10,4>() <<       8,   12,   0,       0,		//! 1
										6,   3,  M_PI/4,    0,		//! 2
										8,   5,  M_PI/2,    0,		//! 3
										7,   7,  2*M_PI/3,  0,		//! 4
										9,  -7,  M_PI/2,    0,		//! 5
										8,   8,  2*M_PI/3,  0,		//! 6	    		
										10,   0,   0,       0,		//! 7
										20,  -9, -5*M_PI/6, 0,		//! 8
										7,    0,  0,       0,		//!9
										10,  10,   M_PI/4,  0).finished();  //! 10	
										
    };	
    
}

#endif