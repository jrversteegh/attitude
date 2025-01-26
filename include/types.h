#ifndef ATTITUDE_TYPES_H__
#define ATTITUDE_TYPES_H__

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <string>
#include <type_traits>

#include <fmt/format.h>

#include "utils.h"

namespace attitude {

using Number = float;

template <typename T> constexpr Number dot(T const& v1, T const& v2) {
  return std::inner_product(v1.cbegin(), v1.cend(), v2.cbegin(), 0);
}

template <int N> struct Components {

  template <typename... Args,
            typename = std::enable_if_t<(sizeof...(Args) == N)>>
  Components(Args&&... args) : c_{std::forward<Args>(args)...} {}
  Components(Components const& cs) : c_{cs.c_} {}

  bool equals(Components<N> const& other) const {
    return this->c_ == other.c_;
  }

  Number operator[](int i) const {
    return c_[i];
  }

  Number& operator[](int i) {
    return c_[i];
  }

  template <int I = 0> Components<N>& operator+=(Components<N> const& v) {
    if constexpr (I < N) {
      c_[I] += v.c_[I];
      return this->operator+= <I + 1>(v);
    } else {
      return *this;
    }
  }

  template <int I = 0> Components<N>& operator*=(Number const n) {
    if constexpr (I < N) {
      c_[I] *= n;
      return this->operator*= <I + 1>(n);
    } else {
      return *this;
    }
  }

  Components<N>& operator/=(Number const n) {
    return this->operator*=(1 / n);
  }

  auto cbegin() const {
    return c_.cbegin();
  }
  auto cend() const {
    return c_.cend();
  }

  constexpr size_t size() const {
    return N;
  }

  constexpr auto slice(int const from, int const to) const {
    return const_span(c_).template subspan<from, to - from>();
  }

private:
  std::array<Number, N> c_{};
};

struct Vector3 : public Components<3> {
  Vector3(Number x, Number y, Number z) : Components<3>{x, y, z} {}
  using Components<3>::Components;

  std::string to_string() const {
    return fmt::format("{:f}, {:f}, {:f}", x(), y(), z());
  }

  Number x() const {
    return this->operator[](0);
  }
  Number y() const {
    return this->operator[](1);
  }
  Number z() const {
    return this->operator[](2);
  }
  Vector3& set_x(Number const value) {
    this->operator[](0) = value;
    return *this;
  }
  Vector3& set_y(Number const value) {
    this->operator[](1) = value;
    return *this;
  }
  Vector3& set_z(Number const value) {
    this->operator[](2) = value;
    return *this;
  }
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

struct Quaternion : Components<4> {
  Quaternion(Quaternion const& q) : Components<4>{q.a(), q.b(), q.c(), q.d()} {}
  Quaternion(Vector3 const& v) : Components<4>{0, v.x(), v.y(), v.z()} {}
  using Components<4>::Components;

  std::string to_string() const {
    return fmt::format("{:f}, {:f}, {:f}, {:f}", a(), b(), c(), d());
  }

  Number a() const {
    return this->operator[](0);
  }
  Number b() const {
    return this->operator[](1);
  }
  Number c() const {
    return this->operator[](2);
  }
  Number d() const {
    return this->operator[](3);
  }

  Quaternion adjoint() const {
    return Quaternion(a(), -b(), -c(), -d());
  }

  Number norm() const {
    return std::sqrt(dot(*this, *this));
  }

  Quaternion inverse() const {
    Quaternion result = adjoint();
    result /= norm();
    return result;
  }
};

extern Quaternion operator*(Quaternion const& q1, Quaternion const& q2);

template <typename T>
concept HasEquals = requires(T t, T const& v) {
  { t.equals(v) } -> std::same_as<bool>;
};

template <HasEquals T> inline bool operator==(T const& v1, T const& v2) {
  return v1.equals(v2);
}

template <typename T>
concept HasToString = requires(T t) {
  { t.to_string() } -> std::convertible_to<std::string>;
};

template <HasToString T>
std::ostream& operator<<(std::ostream& os, T const& v) {
  os << v.to_string();
  return os;
}

} // namespace attitude

#endif
