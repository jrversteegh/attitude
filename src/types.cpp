#include "../include/types.h"

using namespace attitude;

Quaternion operator*(Quaternion const& q1, Quaternion const& q2) {
  return Quaternion{0, 0, 0, 0};
}
