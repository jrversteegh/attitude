#ifndef ATTITUDE_TYPES_H__
#define ATTITUDE_TYPES_H__

#include <array>
#include <cmath>
#include <concepts>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <type_traits>

#include <fmt/format.h>

#include "config.h"
#include "utils.h"

namespace attitude {

template <typename T, std::size_t I = 0>
constexpr Number dot(T const& v1, T const& v2) {
  if constexpr (I < T::size) {
    return v1[I] * v2[I] + dot<T, I + 1>(v1, v2);
  } else {
    return 0;
  }
}

template <std::size_t N> struct Components {
  constexpr static std::size_t const size = N;
  constexpr static bool const is_components = true;
  using value_type = Number;

  template <typename... Args,
            typename = std::enable_if_t<(sizeof...(Args) == N)>>
  constexpr Components(Args&&... args) : c_{std::forward<Args>(args)...} {}
  constexpr Components(Components const& cs) : c_{cs.c_} {}
  constexpr Components() : c_{} {}

  constexpr bool equals(Components<N> const& other) const {
    return this->c_ == other.c_;
  }

  constexpr Number operator[](std::size_t const i) const {
    if (i >= N) {
#ifdef __cpp_exceptions
      throw std::out_of_range("Component index out of range");
#else
      return std::numeric_limits<Number>::max();
#endif
    }
    return c_[i];
  }

  Number& operator[](std::size_t const i) {
    if (i >= N) {
#ifdef __cpp_exceptions
      throw std::out_of_range("Component index out of range");
#else
      static Number out_of_range = std::numeric_limits<Number>::max();
      return out_of_range;
#endif
    }
    return c_[i];
  }

  template <int I = 0>
  constexpr Components<N>& operator+=(Components<N> const& v) {
    if constexpr (I < N) {
      c_[I] += v.c_[I];
      return this->operator+= <I + 1>(v);
    } else {
      return *this;
    }
  }

  template <int I = 0>
  constexpr Components<N>& operator-=(Components<N> const& v) {
    if constexpr (I < N) {
      c_[I] -= v.c_[I];
      return this->operator-= <I + 1>(v);
    } else {
      return *this;
    }
  }

  template <int I = 0> constexpr Components<N>& operator*=(Number const n) {
    if constexpr (I < N) {
      c_[I] *= n;
      return this->operator*= <I + 1>(n);
    } else {
      return *this;
    }
  }

  constexpr Components<N>& operator/=(Number const n) {
    return this->operator*=(1 / n);
  }

  constexpr auto cbegin() const {
    return c_.cbegin();
  }

  constexpr auto cend() const {
    return c_.cend();
  }

  template <std::size_t FROM, std::size_t TO = size, std::size_t STRIDE = 1>
  constexpr auto slice() const {
    return attitude::slice<FROM, TO, STRIDE>(*this);
  }

  constexpr auto count() const {
    return size;
  }

private:
  std::array<Number, N> c_{};
};

template <typename T>
concept HasEquals = requires(T t, T const& v) {
  { t.equals(v) } -> std::same_as<bool>;
};

template <typename T>
concept IsComponents = requires { T::is_components; };

template <typename T>
concept IsSlice = requires { T::is_slice; };

template <class T, class I = std::size_t>
concept IsSubscriptable = requires(T& t, I const& i) {
  { t[i] };
};

template <class T, std::size_t S>
concept HasSize = requires { T::size == S; };

template <typename T>
concept HasToString = requires(T t) {
  { t.to_string() } -> std::convertible_to<std::string>;
};

template <HasEquals T> inline bool operator==(T const& v1, T const& v2) {
  return v1.equals(v2);
};

template <HasToString T>
std::ostream& operator<<(std::ostream& os, T const& v) {
  os << v.to_string();
  return os;
};

template <IsComponents T>
constexpr decltype(auto) operator+(T const& c1, T const& c2) {
  T result{c1};
  result += c2;
  return result;
};

template <IsComponents T>
constexpr decltype(auto) operator-(T const& c1, T const& c2) {
  T result{c1};
  result -= c2;
  return result;
};

template <IsComponents T>
constexpr decltype(auto) operator*(T const& c1, Number const n) {
  T result{c1};
  result *= n;
  return result;
};

template <IsComponents T>
constexpr decltype(auto) operator*(T const& c1, T const& c2) {
  return dot(c1, c2);
}

template <IsComponents T>
constexpr decltype(auto) operator*(Number const n, T const& c1) {
  return c1 * n;
};

struct Vector3 : public Components<3> {
  constexpr Vector3(Number x, Number y, Number z) : Components<3>{x, y, z} {}
  using Components<3>::Components;

  std::string to_string() const {
    return fmt::format("{:f}, {:f}, {:f}", x(), y(), z());
  }

  constexpr Number x() const {
    return this->operator[](0);
  }
  constexpr Number y() const {
    return this->operator[](1);
  }
  constexpr Number z() const {
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

template <IsSubscriptable T>
  requires HasSize<T, 3>
constexpr Vector3 cross(T const& v1, T const& v2) {
  return Vector3{v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2],
                 v1[0] * v2[1] - v1[1] * v2[0]};
}

struct Quaternion : Components<4> {
  constexpr Quaternion(Quaternion const& q)
      : Components<4>{q.r(), q.i(), q.j(), q.k()} {}
  constexpr Quaternion(Number real, Vector3 const& v)
      : Components<4>{real, v.x(), v.y(), v.z()} {}
  constexpr Quaternion(Vector3 const& v)
      : Components<4>{0.f, v.x(), v.y(), v.z()} {}
  using Components<4>::Components;

  std::string to_string() const {
    return fmt::format("{:f}, {:f}, {:f}, {:f}", r(), i(), j(), k());
  }

  constexpr Number r() const {
    return this->operator[](0);
  }
  constexpr Number i() const {
    return this->operator[](1);
  }
  constexpr Number j() const {
    return this->operator[](2);
  }
  constexpr Number k() const {
    return this->operator[](3);
  }

  constexpr Quaternion adjoint() const {
    return Quaternion(r(), -i(), -j(), -k());
  }

  constexpr Number norm() const {
    return std::sqrt(dot(*this, *this));
  }

  constexpr Vector3 vector() const {
    return Vector3(i(), j(), k());
  }

  constexpr operator Vector3() const {
    return vector();
  }

  constexpr Quaternion inverse() const {
    Quaternion result = adjoint();
    result /= norm();
    return result;
  }

  constexpr Quaternion mul(Quaternion const& other) const {
    return Quaternion(this->r() * other.r() -
                          dot(this->slice<1>(), other.slice<1>()),
                      this->r() * other.vector() + other.r() * this->vector() -
                          cross(this->slice<1>(), other.slice<1>()));
  }
};

constexpr Quaternion operator*(Quaternion const& q1, Quaternion const& q2) {
  return Quaternion(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[3] * q2[2] - q1[2] * q2[3],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[1] * q2[3] - q1[3] * q2[1],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2]);
}

/**
 * Heading/Pitch/Roll
 *
 * Tait bryan angle representation of orientation or rotation.
 * One of the possible combination of "Euler Angles" commonly used
 * in shipping and aeronautics. The full rotation is achieved by
 * consecutavely rotating around the Z, Y, and X axes of a body,
 * in that order. Note that the axes rotate along with the body.
 */
struct HeadingPitchRoll : Components<3> {
  using Components<3>::Components;
};

/**
 * Rotation Vector
 *
 * Attitude or rotation representation by means of a 3D vector
 * that points in the direction of the rotation axis and has
 * a length that matches the angle of rotation. This is somewhat
 * similar to a Quaternion representation, but with the contrained
 * that the quaternion should be a unit quaternion removed.
 */
struct RotationVector : Vector3 {
  using Vector3::Vector3;
};

/**
 * Rotation Matrix
 *
 * Matrix representation of orientation or rotation.
 */
struct RotationMatrix : Components<9> {
  using Components<9>::Components;
};

} // namespace attitude

#endif
