#ifndef ATTITUDE_ROTATION_VECTOR_H__
#define ATTITUDE_ROTATION_VECTOR_H__

#include "config.h"
#include "quaternion.h"
#include "types.h"

namespace attitude {

/**
 * Rotation Vector
 *
 * Attitude or rotation representation by means of a 3D vector
 * that points in the direction of the rotation axis and has
 * a length that matches the angle of rotation. This is somewhat
 * similar to a Quaternion representation, but with the contrained
 * that the quaternion should be a unit quaternion removed.
 */
template <typename T = Number>
struct RotationVector : Vector<T> {
  using Base = Vector<T>;
  using Base::Base;
  explicit constexpr RotationVector(UnitQuaternion<T> const& q)
      : Base{q.i(), q.j(), q.k()} {
    *this *= 2 * std::acos(q.r()) / std::sqrt(1 - q.r() * q.r());
  }
  explicit operator UnitQuaternion<T>() const {
    return UnitQuaternion<T>{this->length(), (*this)[0] * 0.5, (*this)[1] * 0.5,
                             (*this)[2] * 0.5};
  }
};

} // namespace attitude

#endif
