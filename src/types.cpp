#include "attitude/types.h"

namespace attitude {

constexpr static Quaternion mul(Quaternion const& q1, Quaternion const& q2) {
  return Quaternion(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[3] * q2[2] - q1[2] * q2[3],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[1] * q2[3] - q1[3] * q2[1],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2]);
}

Quaternion Quaternion::mul(Quaternion const& other) const {
  return attitude::mul(*this, other);
}

} // namespace attitude
