#include "../include/types.h"

namespace attitude {

Quaternion operator*(Quaternion const& q1, Quaternion const& q2) {
  return Quaternion(
    q1[0] * q2[0] - dot(q1.slice(1, 4), q2.slice(1, 4)),
      0,
      0,
      0);
}

}  // namespace attitude
