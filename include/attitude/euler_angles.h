#ifndef ATTITUDE_EULER_ANGLES_H__
#define ATTITUDE_EULER_ANGLES_H__

#include "config.h"
#include "quaternion.h"
#include "types.h"

namespace attitude {

/**
 * Heading/Pitch/Roll
 *
 * Tait bryan angle representation of orientation or rotation.
 * One of the possible combination of "Euler Angles" commonly used
 * in shipping and aeronautics. The full rotation is achieved by
 * consecutavely rotating around the Z, Y, and X axes of a body,
 * in that order. Note that the axes rotate along with the body.
 */
template <typename T = Number>
struct HeadingPitchRoll : Components<3, T> {
  using Base = Components<3, T>;
  using Base::Base;
  explicit constexpr HeadingPitchRoll(UnitQuaternion<T> const& q)
      : Base{/* TODO */} {}
  explicit operator UnitQuaternion<T>() const {
    return UnitQuaternion<T>{/* TODO */};
  }
};

} // namespace attitude

#endif
