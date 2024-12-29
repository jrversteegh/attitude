#ifndef ATTITUDE_TYPES_H__
#define ATTITUDE_TYPES_H__

#include <iostream>
#include <string>

#include <fmt/format.h>

namespace attitude {

using Number = float;

struct Vector3 {
  Vector3(Number x, Number y, Number z) : components_{x, y, z} {}
  Vector3(Vector3 const& v)
      : components_{v.components_[0], v.components_[1], v.components_[2]} {}

  std::string to_string() const {
    return fmt::format("%f, %f, %f", x(), y(), z());
  }

  Number x() const {
    return components_[0];
  }
  Number y() const {
    return components_[1];
  }
  Number z() const {
    return components_[2];
  }
  Vector3& set_x(Number const value) {
    components_[0] = value;
    return *this;
  }
  Vector3& set_y(Number const value) {
    components_[1] = value;
    return *this;
  }
  Vector3& set_z(Number const value) {
    components_[2] = value;
    return *this;
  }
  Number operator[](int i) const {
    return components_[i];
  }
  Number& operator[](int i) {
    return components_[i];
  }

private:
  Number components_[3] = {0, 0, 0};
};

inline Number operator*(Vector3 const& v1, Vector3 const& v2) {
  return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

inline Vector3 operator+(Vector3 const& v1, Vector3 const& v2) {
  return Vector3{v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z()};
}

inline bool operator==(Vector3 const& v1, Vector3 const& v2) {
  return v1.x() == v2.x() && v1.y() == v2.y() && v1.z() == v2.z();
}

inline std::ostream& operator<<(std::ostream& os, Vector3 const& v) {
  os << v.to_string();
  return os;
}

struct Quaternion {};

} // namespace attitude

#endif
