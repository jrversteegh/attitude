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

template <typename T, typename S, std::size_t I = 0>
constexpr Number dot(T const& v1, S const& v2) {
  if constexpr (I < T::size) {
    return v1[I] * v2[I] + dot<T, S, I + 1>(v1, v2);
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

  template <std::size_t I> constexpr Number get() const {
    static_assert(I < N);
    return c_[I];
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

template <typename T>
concept IsMatrix = requires { T::is_matrix; };

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
    return this->get<0>();
  }
  constexpr Number y() const {
    return this->get<1>();
  }
  constexpr Number z() const {
    return this->get<2>();
  }
};

constexpr Number operator*(Vector3 const& v1, Vector3 const& v2) {
  return dot(v1, v2);
}

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
  explicit constexpr Quaternion(Vector3 const& v)
      : Components<4>{0.f, v.x(), v.y(), v.z()} {}
  using Components<4>::Components;

  std::string to_string() const {
    return fmt::format("{:f}, {:f}, {:f}, {:f}", r(), i(), j(), k());
  }

  constexpr Number r() const {
    return this->get<0>();
  }
  constexpr Number i() const {
    return this->get<1>();
  }
  constexpr Number j() const {
    return this->get<2>();
  }
  constexpr Number k() const {
    return this->get<3>();
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

  explicit constexpr operator Vector3() const {
    return vector();
  }

  constexpr Quaternion inverse() const {
    Quaternion result = adjoint();
    result /= norm();
    return result;
  }

  // Present for benchmark comparison to operator*
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

struct UnitQuaternion : private Quaternion {
  constexpr UnitQuaternion(Number const r, Number const i, Number const j,
                           Number const k)
      : Quaternion{r, i, j, k} {}
  constexpr UnitQuaternion(UnitQuaternion const& q)
      : Quaternion{q.r(), q.i(), q.j(), q.k()} {}
  using Quaternion::i;
  using Quaternion::j;
  using Quaternion::k;
  using Quaternion::r;
  using Quaternion::size;
  using Quaternion::to_string;
  constexpr Number operator[](std::size_t const i) const {
    return Quaternion::operator[](i);
  }
  constexpr UnitQuaternion inverse() const {
    return UnitQuaternion(r(), -i(), -j(), -k());
  }
};

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

template <typename M> std::string matrix_to_string(M const& m) {
  return fmt::format("{:f}, {:f}, {:f}\n{:f}, {:f}, {:f}\n{:f}, {:f}, {:f}",
                     m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * 3x3 Matrix
 */
struct Matrix3 : Components<9> {
  using Components<9>::Components;
  constexpr static bool const is_matrix = true;
  std::string to_string() const {
    return matrix_to_string(*this);
  }
};

/**
 * Rotation Matrix
 *
 * Matrix representation of orientation or rotation.
 */
struct RotationMatrix : Components<6> {
  using Components<6>::Components;
  explicit constexpr RotationMatrix(UnitQuaternion const& q)
      : Components{sqr(q[0]) + sqr(q[1]) - sqr(q[2]) - sqr(q[3]),
                   2 * (q[1] * q[2] + q[0] * q[3]),
                   2 * (q[1] * q[3] - q[0] * q[2]),
                   sqr(q[0]) - sqr(q[1]) + sqr(q[2]) - sqr(q[3]),
                   2 * (q[2] * q[3] + q[0] * q[1]),
                   sqr(q[0]) - sqr(q[1]) - sqr(q[2]) + sqr(q[3])} {}
  constexpr static bool const is_matrix = true;
  constexpr static size_t const size = 9;
  std::string to_string() const {
    return matrix_to_string(*this);
  }

  template <std::size_t I> static constexpr size_t adapt_index() {
    return I < 3 ? I : (I < 6 ? (I == 3 ? 1 : I - 1) : (I == 6 ? 2 : I - 3));
  }

  static constexpr size_t adapt_index(std::size_t const i) {
    return i < 3 ? i : (i < 6 ? (i == 3 ? 1 : i - 1) : (i == 6 ? 2 : i - 3));
  }

  constexpr Number operator[](std::size_t const i) const {
    return Components::operator[](adapt_index(i));
  }

  Number& operator[](std::size_t const i) {
    return Components::operator[](adapt_index(i));
  }

  template <std::size_t I> constexpr Number get() const {
    return Components::get<adapt_index<I>()>();
  }
};

template <IsMatrix M, IsMatrix N>
constexpr auto operator*(M const& m1, N const& m2) {
  auto r1 = slice<0, 3, 1>(m1);
  auto r2 = slice<3, 6, 1>(m1);
  auto r3 = slice<6, 9, 1>(m1);
  auto c1 = slice<0, 9, 3>(m2);
  auto c2 = slice<1, 9, 3>(m2);
  auto c3 = slice<2, 9, 3>(m2);
  return Matrix3{dot(r1, c1), dot(r1, c2), dot(r1, c3),
                 dot(r2, c1), dot(r2, c2), dot(r2, c3),
                 dot(r3, c1), dot(r3, c2), dot(r3, c3)};
}

template <>
constexpr auto operator*
    <RotationMatrix, RotationMatrix>(RotationMatrix const& m1,
                                     RotationMatrix const& m2) {
  auto r1 = slice<0, 3, 1>(m1);
  auto r2 = slice<3, 6, 1>(m1);
  auto r3 = slice<6, 9, 1>(m1);
  auto c1 = slice<0, 9, 3>(m2);
  auto c2 = slice<1, 9, 3>(m2);
  auto c3 = slice<2, 9, 3>(m2);
  return RotationMatrix{dot(r1, c1), dot(r1, c2), dot(r1, c3),
                        dot(r2, c2), dot(r2, c3), dot(r3, c3)};
}

template <IsMatrix M> constexpr auto operator*(M const& m, Vector3 const& v) {
  return Vector3{dot(slice<0, 3, 1>(m), v), dot(slice<3, 6, 1>(m), v),
                 dot(slice<6, 9, 1>(m), v)};
}

} // namespace attitude

#endif
