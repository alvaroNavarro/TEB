#ifndef __teb_package__typedefs__
#define __teb_package__typedefs__

//#define RTW

#ifdef RTW
#define DISABLE_IO
#define __XPCTARGET__ // setup qpOases for xpc target as well
#endif


#include <limits.h>

#ifndef DISABLE_IO
#include <iostream>
#endif
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <stack>
#include <deque>


#if !defined(NDEBUG) && defined(_DEBUG)
#define NDEBUG
#endif
#if defined(NDEBUG) && !defined(_DEBUG)
#define _DEBUG
#endif

namespace teb
{

// === constants ===
  
//! Define \f$ \pi \f$
#define PI 3.14159265358979323846264
  
//! Define Inifity \f$ \infty \f$.
#define INF HUGE_VAL//std::numeric_limits<double>::max()
 
// Typedefs

//! Define 1x1 Eigen::Matrix
template<typename T>
using EigenScalar = Eigen::Matrix<T,1,1>;

//! Define 1x1 Eigen::Matrix of type double
using EigenScalarD = EigenScalar<double>;

//! Backup stack type
template<typename T>
using BackupStackType = std::stack<T, std::deque<T> >;

// === Debug messages ===

#if defined(NDEBUG) || defined(DISABLE_IO)
    #define PRINT_DEBUG(msg)
    #define PRINT_DEBUG_ONCE(msg)
    #define PRINT_DEBUG_COND(cond, msg)
    #define PRINT_DEBUG_COND_ONCE(cond, msg)
#else
    //! Print \c msg-stream only if project is compiled in Debug-mode
    #define PRINT_DEBUG(msg) std::cout << "Debug: " << msg << std::endl;
  
    //! Print \c msg-stream only once and only if project is compiled in Debug-mode
    #define PRINT_DEBUG_ONCE(msg) { static const auto debugOnce = [&] { std::cout << "Debug: " << msg << std::endl; return true;}(); (void)debugOnce; } // void cast: avoid compiler warnings since it is unused later
  
    //! Print \c msg-stream only if \c cond == \c true and only if project is compiled in Debug-mode
    #define PRINT_DEBUG_COND(cond, msg) if (cond) std::cout << "Debug: " << msg << std::endl;
  
    //! Print \c msg-stream only if \c cond == \c true, only once and only if project is compiled in Debug-mode
    #define PRINT_DEBUG_COND_ONCE(cond, msg) { static const auto debugOnce = [&] { if (cond) std::cout << "Debug: " << msg << std::endl; return true;}(); (void)debugOnce; }

#endif


#ifdef DISABLE_IO

#define PRINT_INFO(msg)
#define PRINT_INFO_ONCE(msg)
#define PRINT_INFO_COND(cond, msg)
#define PRINT_INFO_COND_ONCE(cond, msg)

#define PRINT_ERROR(msg)

#define INPUT_STREAM(variable, default_val) variable = default_val;

#else

//! Print \c msg-stream
#define PRINT_INFO(msg) std::cout << "Info: " << msg << std::endl;

//! Print \c msg-stream only once
#define PRINT_INFO_ONCE(msg) { static const auto infoOnce = [&] { std::cout << "Info: " << msg << std::endl; return true;}(); (void)infoOnce; } // void cast: avoid compiler warnings since it is unused later

//! Print \c msg-stream only if \c cond == \c true
#define PRINT_INFO_COND(cond, msg) if (cond) std::cout << "Info: " << msg << std::endl;

//! Print \c msg-stream only if \c cond == \c true, only once
#define PRINT_INFO_COND_ONCE(cond, msg) { static const auto infoOnce = [&] { if (cond) std::cout << "Info: " << msg << std::endl; return true;}(); (void)infoOnce; }

//! Print \c msg-stream as error msg
#define PRINT_ERROR(msg) std::cerr << "Error: " << msg << std::endl;

#define INPUT_STREAM(variable, default_val) std::cin >> variable;

#endif


} // end namespace teb

#endif /* defined(__teb_package__typedefs__) */
