
#ifndef __teb_package__derivatives__
#define __teb_package__derivatives__

#include <cmath>

namespace teb
{
 
  class NumericalDerivatives
  {
    public:
      
      NumericalDerivatives(double step) { _step = step;};            
      
      double computeDerivative(const double x1, const double x2) { return((x2-x1)/_step);}; 
      double getAngle(const double derivative) { return(std::atan(derivative));};
      
      double computeAngle(const double x1, const double x2) { return(std::atan(computeDerivative(x1,x2)));};
      	            
  protected:
    
    double _step = 0.1;
    
  };
}

#endif