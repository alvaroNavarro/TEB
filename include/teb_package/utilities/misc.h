#ifndef __teb_package__misc__
#define __teb_package__misc__

#include <teb_package/base/typedefs.h>
#include <math.h>
#include <functional>
#include <tuple>


/**
 * @brief Normalize angle to interval [-pi, pi)
 * @param angle angle in radiant
 * @return normalized angle to [-pi, pi) in rad
 */
inline double norm_angle(double angle)
{

    if (angle >= -PI && angle < PI)
    {
        return angle;
    }
    
    double mu = std::floor(angle / (2*PI));
    angle -= mu * 2 * PI;
    if (angle >= PI) angle -= 2*PI;
    if (angle < -PI) angle += 2*PI;

    return angle;
}

/**
 * @brief Normalize vector or matrix of angles to interval [-pi, pi)
 * @param angles angle vector/matrix in radiant [all values will be replaced by its normalized version]
 */
inline void norm_angle_vec(Eigen::Ref<Eigen::MatrixXd> angles)
{
  angles = angles.unaryExpr(std::ptr_fun(norm_angle));
}



/// @cond 0 // exclude from doxygen
template <typename ...A>
struct variadic_temp_equal : std::false_type { };

/**
 * @brief This template construct checks whether two variadic template parameter packs have the same type (pairwise).
 * 
 * Application for parameter packs A... and B... :
 * @code
 * 	static_assert(variadic_temp_equal<std::tuple<A...>, std::tuple<B...>>::value, "Type mismatch for both template parameter packs.");
 * @endcode
 */
template <typename A1, typename ...Aother, typename B1, typename ...Bother>
struct variadic_temp_equal<std::tuple<A1, Aother...>, std::tuple<B1, Bother...>>
{
  static const bool value = std::is_same<A1, B1>::value && variadic_temp_equal<std::tuple<Aother...>, std::tuple<Bother...>>::value;
};

template <typename ...B>
struct variadic_temp_equal<std::tuple<>, std::tuple<B...>> : std::true_type { };
/// @endcond


#endif /* defined(__teb_package__misc__) */
