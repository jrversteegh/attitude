#ifndef ATTITUDE_TYPES_H__
#define ATTITUDE_TYPES_H__

#include <cmath>
#include <iostream>
#include <string>

#include <fmt/format.h>

namespace attitude {

using Number = float;

struct Vector3 {
  Vector3(Number x, Number y, Number z) : c_{x, y, z} {}
  Vector3(Vector3 const& v) : c_{v.x(), v.y(), v.z()} {}

  std::string to_string() const {
    return fmt::format("%f, %f, %f", x(), y(), z());
  }

  Number x() const {
    return c_[0];
  }
  Number y() const {
    return c_[1];
  }
  Number z() const {
    return c_[2];
  }
  Vector3& set_x(Number const value) {
    c_[0] = value;
    return *this;
  }
  Vector3& set_y(Number const value) {
    c_[1] = value;
    return *this;
  }
  Vector3& set_z(Number const value) {
    c_[2] = value;
    return *this;
  }
  Number operator[](int i) const {
    return c_[i];
  }
  Number& operator[](int i) {
    return c_[i];
  }

  Vector3& operator+=(Vector3 const& v) {
    c_[0] += v.x();
    c_[1] += v.y();
    c_[2] += v.z();
    return *this;
  }

  Vector3& operator*=(Number const n) {
    c_[0] *= n;
    c_[1] *= n;
    c_[2] *= n;
    return *this;
  }

  Vector3& operator/=(Number const n) {
    return this->operator*=(1 / n);
  }

private:
  Number c_[3] = {0, 0, 0};
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

struct Quaternion {
  Quaternion(Number const a, Number const b, Number const c, Number const d)
      : c_{a, b, c, d} {}
  Quaternion(Quaternion const& q) : c_{q.a(), q.b(), q.c(), q.d()} {}
  Quaternion(Vector3 const& v) : c_{0, v.x(), v.y(), v.z()} {}

  Number a() const {
    return c_[0];
  }
  Number b() const {
    return c_[1];
  }
  Number c() const {
    return c_[2];
  }
  Number d() const {
    return c_[3];
  }

  Quaternion adjoint() const {
    return Quaternion(c_[0], -c_[1], -c_[2], -c_[3]);
  }

  Number norm() const {
    return std::sqrt(c_[0] * c_[0] + c_[1] * c_[1] + c_[2] * c_[2] +
                     c_[3] * c_[3]);
  }

  Quaternion inverse() const {
    Quaternion result = adjoint();
    result /= norm();
    return result;
  }

  Quaternion& operator+=(Quaternion const& q) {
    c_[0] += q.a();
    c_[1] += q.b();
    c_[2] += q.c();
    c_[3] += q.d();
    return *this;
  }

  Quaternion& operator*=(Number const n) {
    c_[0] *= n;
    c_[1] *= n;
    c_[2] *= n;
    c_[3] *= n;
    return *this;
  }

  Quaternion& operator/=(Number const n) {
    return this->operator*=(1 / n);
  }

private:
  Number c_[4] = {0, 0, 0, 0};
};

} // namespace attitude

#endif
