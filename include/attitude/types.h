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

#include <trix/matrix.h>
#include <trix/vector.h>

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

  template <int I = 0> constexpr Components<N>& negate() {
    if constexpr (I < N) {
      c_[I] = -c_[I];
      return this->negate<I + 1>();
    } else {
      return *this;
    }
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
concept HasNegate = requires(T t) { t.negate(); };

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

template <HasNegate T> inline T operator-(T const& v) {
  T result{v};
  result.negate();
  return result;
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
};

constexpr Quaternion operator*(Quaternion const& q1, Quaternion const& q2) {
  return Quaternion(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[3] * q2[2] - q1[2] * q2[3],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[1] * q2[3] - q1[3] * q2[1],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2]);
}

struct UnitQuaternion : protected Quaternion {
  constexpr UnitQuaternion(Number const r, Number const i, Number const j,
                           Number const k)
      : Quaternion{r, i, j, k} {}
  constexpr UnitQuaternion(UnitQuaternion const& q) : Quaternion{q} {}
  using Quaternion::Components;
  using Quaternion::equals;
  using Quaternion::i;
  using Quaternion::j;
  using Quaternion::k;
  using Quaternion::negate;
  using Quaternion::r;
  using Quaternion::size;
  using Quaternion::to_string;
  using Quaternion::value_type;
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
  explicit constexpr HeadingPitchRoll(UnitQuaternion const& q)
      : Components{ /* TODO */ } {}
  explicit operator UnitQuaternion() const {
    return UnitQuaterion{ /* TODO */ };
  }
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
  explicit constexpr RotationVector(UnitQuaternion const& q)
      : Vector3{ q.i(), q.j(), q.k()  } {
        *this *=  2 * std::acos(q.r()) / std::sqrt(1 - q.r() * q.r())
      }
  explicit operator UnitQuaternion() const {
    return UnitQuaternion{
      lenght(), get<0>() * 0.5, get<1>() * 0.5, get<2>() * 0.5
    }
  }
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
  constexpr Vector3 diag() const {
    return Vector3{get<0>(), get<4>(), get<8>()};
  }
};

/**
 * Tensor
 *
 * Symmetric 3x3 matrix
 */
struct Tensor : Components<6> {
  using Components<6>::Components;
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

/**
 * Rotation Matrix
 *
 * Matrix representation of orientation or rotation.
 */
struct RotationMatrix : private Matrix3 {
  explicit constexpr RotationMatrix(UnitQuaternion const& q)
      : Matrix3{sqr(q[0]) + sqr(q[1]) - sqr(q[2]) - sqr(q[3]),
                2 * (q[1] * q[2] + q[0] * q[3]),
                2 * (q[1] * q[3] - q[0] * q[2]),
                2 * (q[1] * q[2] - q[0] * q[3]),
                sqr(q[0]) - sqr(q[1]) + sqr(q[2]) - sqr(q[3]),
                2 * (q[2] * q[3] + q[0] * q[1]),
                2 * (q[1] * q[3] + q[0] * q[2]),
                2 * (q[2] * q[3] - q[0] * q[1]),
                sqr(q[0]) - sqr(q[1]) - sqr(q[2]) + sqr(q[3])} {}
  using Matrix3::diag;
  using Matrix3::equals;
  using Matrix3::get;
  using Matrix3::is_matrix;
  using Matrix3::size;
  using Matrix3::value_type;
  explicit operator UnitQuaternion() const {
    auto d = diag();
    if (d[1] > -d[2] && d[0] > -d[1] && d[0] > -d[2]) {
      value_type f = 2 * std::sqrt(1 + d[0] + d[1] + d[2]);
      return UnitQuaternion{f / 4, (get<5>() - get<7>()) / f,
                            (get<6>() - get<2>()) / f,
                            (get<1>() - get<3>()) / f};
    } else if (d[1] < -d[2] && d[0] > d[1] && d[0] > d[2]) {
      value_type f = 2 * std::sqrt(1 + d[0] - d[1] - d[2]);
      return UnitQuaternion{(get<5>() - get<7>()) / f, f / 4,
                            (get<1>() + get<3>()) / f,
                            (get<6>() + get<2>()) / f};
    } else if (d[1] > d[2] && d[0] < d[1] && d[0] < -d[2]) {
      value_type f = 2 * std::sqrt(1 - d[0] + d[1] - d[2]);
      return UnitQuaternion{(get<6>() - get<2>()) / f,
                            (get<1>() + get<3>()) / f, f / 4,
                            (get<5>() + get<7>()) / f};
    } else {
      value_type f = 2 * std::sqrt(1 - d[0] - d[1] + d[2]);
      return UnitQuaternion{(get<1>() - get<3>()) / f,
                            (get<6>() + get<2>()) / f,
                            (get<5>() + get<7>()) / f, f / 4};
    }
  }
};

template <IsMatrix M, IsMatrix N>
constexpr auto operator*(M const& m1, N const& m2) {
  return Matrix3{m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6],
                 m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7],
                 m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8],
                 m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6],
                 m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7],
                 m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8],
                 m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6],
                 m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7],
                 m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8]};
}

template <IsMatrix M, IsMatrix N, int I = 0>
constexpr bool operator==(M const& m1, N const& m2) {
  if constexpr (I < M::size) {
    if (m1[I] != m2[I]) {
      return false;
    } else {
      return operator== <M, N, I + 1>(m1, m2);
    }
  } else {
    return true;
  }
}

template <>
constexpr auto operator* <Tensor, Tensor>(Tensor const& t1, Tensor const& t2) {
  return Tensor{t1[0] * t2[0] + t1[1] * t2[1] + t1[2] * t2[2],
                t1[0] * t2[1] + t1[1] * t2[4] + t1[2] * t2[5],
                t1[0] * t2[2] + t1[1] * t2[5] + t1[2] * t2[8],
                t1[1] * t2[1] + t1[4] * t2[4] + t1[5] * t2[5],
                t1[1] * t2[2] + t1[4] * t2[5] + t1[5] * t2[8],
                t1[2] * t2[2] + t1[5] * t2[5] + t1[8] * t2[8]};
}

template <IsMatrix M> constexpr auto operator*(M const& m, Vector3 const& v) {
  return Vector3{m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
                 m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
                 m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

} // namespace attitude

#endif
