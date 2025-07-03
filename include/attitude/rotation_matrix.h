#ifndef ATTITUDE_ROTATION_MATRIX_H__
#define ATTITUDE_ROTATION_MATRIX_H__

#include "config.h"
#include "quaternion.h"
#include "types.h"
#include "utils.h"

namespace attitude {

/**
 * Rotation Matrix
 *
 * Matrix representation of orientation or rotation.
 */
template <typename T = Number>
struct RotationMatrix : trix::Matrix<3, 3, T> {
  using Base = trix::Matrix<3, 3, T>;
  using Base::Base;
  explicit constexpr RotationMatrix(UnitQuaternion<T> const& q)
      : Base{sqr(q[0]) + sqr(q[1]) - sqr(q[2]) - sqr(q[3]),
             2 * (q[1] * q[2] + q[0] * q[3]),
             2 * (q[1] * q[3] - q[0] * q[2]),
             2 * (q[1] * q[2] - q[0] * q[3]),
             sqr(q[0]) - sqr(q[1]) + sqr(q[2]) - sqr(q[3]),
             2 * (q[2] * q[3] + q[0] * q[1]),
             2 * (q[1] * q[3] + q[0] * q[2]),
             2 * (q[2] * q[3] - q[0] * q[1]),
             sqr(q[0]) - sqr(q[1]) - sqr(q[2]) + sqr(q[3])} {}
  explicit operator UnitQuaternion<T>() const {
    using value_type = typename Base::value_type;
    auto const& d = this->diagonal();
    auto const& m = *this;
    if (d[1] > -d[2] && d[0] > -d[1] && d[0] > -d[2]) {
      value_type f = 2 * std::sqrt(1 + d[0] + d[1] + d[2]);
      return UnitQuaternion<T>{f / 4, (m[1, 2] - m[2, 1]) / f,
                               (m[2, 0] - m[0, 2]) / f,
                               (m[0, 1] - m[1, 0]) / f};
    } else if (d[1] < -d[2] && d[0] > d[1] && d[0] > d[2]) {
      value_type f = 2 * std::sqrt(1 + d[0] - d[1] - d[2]);
      return UnitQuaternion<T>{(m[1, 2] - m[2, 1]) / f, f / 4,
                               (m[0, 1] + m[1, 0]) / f,
                               (m[2, 0] + m[0, 2]) / f};
    } else if (d[1] > d[2] && d[0] < d[1] && d[0] < -d[2]) {
      value_type f = 2 * std::sqrt(1 - d[0] + d[1] - d[2]);
      return UnitQuaternion<T>{(m[2, 0] - m[0, 2]) / f, (m[0, 1] + m[1, 0]) / f,
                               f / 4, (m[1, 2] + m[2, 1]) / f};
    } else {
      value_type f = 2 * std::sqrt(1 - d[0] - d[1] + d[2]);
      return UnitQuaternion<T>{(m[0, 1] - m[1, 0]) / f, (m[2, 0] + m[0, 2]) / f,
                               (m[1, 2] + m[2, 1]) / f, f / 4};
    }
  }
};

} // namespace attitude

#endif
