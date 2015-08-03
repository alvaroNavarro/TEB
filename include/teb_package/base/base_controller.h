#ifndef __teb_package__base_controller__
#define __teb_package__base_controller__

#include <teb_package/base/config.h>
#include <memory>
#include <cmath>
#include <deque>


namespace teb
{

/**
  * @brief Base controller class (General Controller API)
  * 
  * @ingroup controller
  *  
  * This abstract class defines the general controller interface.
  * 
  * @todo Implement complete API (abstract functions, etc.) to allow a generic controller
  *       usage for teb::Simulator and teb::TebPlotter
  *
  * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
  *  
  * @sa teb::TebController
  */
class BaseController
{
    
public:
        
    /** @brief Get the time difference \f$ \Delta T \f$ between two discrete samples within the horizon. 
     *  It can also be interpreted as the step width.
     *  @return \f$ \Delta T \f$
     */
    virtual double getDt() const = 0;
    
    /** @brief Get the horizon length, resp. the number of discrete samples.
     *  @return Number of discrete samples 
     */ 
    virtual unsigned int getN() const = 0;
  
    /** @brief Perform complete MPC step (initialization if necessary or update and optimization)
     *  
     *  @param x0 double array containing start state \c x0 with \c p components [\c p x 1]
     *  @param xf double array containing final state \c xf with \c p components [\c p x 1]
     *  @param ctrl_out [output] store control input (firstControl()) to control the plant [\c q x 1]
     */  
    virtual void step(const double* const x0, const double* const xf, double* ctrl_out = nullptr) = 0;
    
    
    /** @brief Get a matrix containing all state and control input sequences.
     *  The purpose of this function is mainly for debugging and visualization stuff.
     *  @return [p+q x n] matrix with \c p states, \c q control inputs and n=getN().
     */
    virtual Eigen::MatrixXd getStateCtrlInfoMat() const = 0;
    
    
    /** @brief Get the vector of absolute times for the complete horizon.
     *  In case of a uniform step width / time difference (see getDt()) the vector 
     *  corresponds to \f$ t=[0,\Delta T, 2 \Delta T, \dotsc, n \Delta T] \f$.
     *  @return vector of absolute times starting with t=0s.
     */
    virtual Eigen::VectorXd getAbsoluteTimeVec() const = 0; //!< Get vector of absolute times \f$ t=[0,\Delta T, 2 \Delta T, \dotsc, n \Delta T] \f$.

    /** @brief Access the first state \f$ \mathbf{x}_0 \f$ of the horizon
     *  @return Copy of the first state
     */
    virtual Eigen::VectorXd firstState() const = 0;
    
    /** @brief Access the last state \f$ \mathbf{x}_n \f$ of the horizon
     *  @return Copy of the last state
     */
    virtual Eigen::VectorXd lastState() const = 0;
    
    /** @brief Access the first control input \f$ \mathbf{u}_1 \f$ of the horizon
     *  @return Copy of the first state
     */
    virtual Eigen::VectorXd firstControl() const = 0;
    
    /** @brief Return the complete control input sequence \f$ u_k, k=1,\dotsc,n-1 \f$
     *  @returns Matrix containing all planned control inputs [q x n-1]
     */
    virtual Eigen::MatrixXd returnControlInputSequence() const = 0;
    
    /** @brief Reset the controller
     */
    virtual void resetController() = 0;

};


} // end namesapce teb

#endif /* defined(__teb_package__base_controller__) */
