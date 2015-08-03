#ifndef __teb_package__car_like_type_driving__
#define __teb_package__car_like_type_driving__

namespace teb
{
  enum class TypeDriving
  {
    FRONT_WHEEL,    // The car has front wheel driving -> alpha 1 = u0
    REAR_WHEEL      // The car has rear wheel drving   -> alpha 1 = u0 / Cos(phi)
  };
}


#endif /* defined(__teb_package__car_like_type_driving__) */