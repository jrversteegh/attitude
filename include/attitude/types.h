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
#include <utility>

#include <fmt/format.h>

#include <trix/matrix.h>
#include <trix/vector.h>

#include "config.h"
#include "utils.h"

namespace attitude {

template <typename T = Number>
using Vector = trix::Vector<3, T>;

template <typename T = Number>
using Matrix = trix::Matrix<3, 3, T>;

template <class C>
concept ComponentsConcept = requires(C const c, size_t i) {
  typename C::value_type;
  { c.operator[](i) } -> std::same_as<typename C::value_type>;
  { C::components } -> std::convertible_to<size_t>;
};

template <size_t N, typename T = Number>
struct Components {
  constexpr static size_t const components = N;
  using value_type = T;

  template <typename... Args>
    requires(sizeof...(Args) == N)
  constexpr Components(Args&&... args) : c_{std::forward<Args>(args)...} {}
  constexpr Components(Components const& cs) : c_{cs.c_} {}
  constexpr Components() : c_{} {}

  constexpr bool operator==(Components const& other) const {
    return this->c_ == other.c_;
  }

  template <ComponentsConcept C, size_t... Is>
  constexpr bool equals(C const& other, std::index_sequence<Is...>) const {
    return (... && ((*this)[Is] == other[Is]));
  }

  template <ComponentsConcept C>
    requires(C::components == components)
  constexpr bool operator==(C const& other) const {
    return equals(other, std::make_index_sequence<components>{});
  }

  constexpr T operator[](size_t const i) const {
    assert(i < N);
    return c_[i];
  }

  template <typename F>
    requires std::same_as<std::invoke_result_t<F, size_t>, void>
  constexpr void for_each_element(F fun) {
    for (size_t i = 0; i < components; ++i) {
      fun(i);
    }
  }

  template <size_t... Is>
  constexpr auto negate(this auto&& self, std::index_sequence<Is...>) {
    using ReturnType = decltype(self);
    return ReturnType{(-self[Is])...};
  }

  constexpr auto operator-() const {
    return negate(std::make_index_sequence<components>{});
    // return Components{} - *this;
  }

  constexpr operator std::string() const {
    return fmt::format(attitude_fmtstr, fmt::join(c_, ", "));
  }

  static_assert(ComponentsConcept<Components>,
                "Expected Components to satisfy ComponentsConcept");

protected:
  std::array<T, components> c_{};
};

template <size_t N, typename T = Number>
struct MutableComponents : public Components<N, T> {
  using Components<N, T>::Components;

  using Components<N, T>::operator[];
  constexpr T& operator[](size_t const i) {
    assert(i < N);
    return this->c_[i];
  }

  constexpr MutableComponents& operator+=(Components<N, T> const& v) {
    for_each_element([this, &v](size_t i) { (*this)[i] += v[i]; });
    return *this;
  }

  constexpr MutableComponents& operator-=(Components<N, T> const& v) {
    for_each_element([this, &v](size_t i) { (*this)[i] -= v[i]; });
    return *this;
  }

  constexpr MutableComponents& operator*=(T const v) {
    for_each_element([this, v](size_t i) { (*this)[i] *= v; });
    return *this;
  }

  constexpr MutableComponents& operator/=(T const v) {
    for_each_element([this, v](size_t i) { (*this)[i] /= v; });
    return *this;
  }

  static_assert(ComponentsConcept<MutableComponents>,
                "Expected Components to satisfy ComponentsConcept");
};

template <ComponentsConcept C1, ComponentsConcept C2, size_t... Is>
constexpr auto plus(C1 const& c1, C2 const& c2, std::index_sequence<Is...>) {
  return std::common_type<C1, C2>{(c1[Is] + c2[Is])...};
}

template <ComponentsConcept C1, ComponentsConcept C2, size_t... Is>
constexpr auto minus(C1 const& c1, C2 const& c2, std::index_sequence<Is...>) {
  return std::common_type<C1, C2>{(c1[Is] - c2[Is])...};
}

template <ComponentsConcept C, std::convertible_to<typename C::value_type> S,
          size_t... Is>
constexpr auto times(C const& c, S const s, std::index_sequence<Is...>) {
  return C{(c[Is] * s)...};
}

template <ComponentsConcept C1, ComponentsConcept C2>
  requires(C1::components == C2::components)
constexpr auto operator+(C1 const& c1, C2 const& c2) {
  return plus(c1, c2, std::make_index_sequence<C1::components>{});
};

template <ComponentsConcept C1, ComponentsConcept C2>
constexpr auto operator-(C1 const& c1, C2 const& c2) {
  return minus(c1, c2, std::make_index_sequence<C1::components>{});
};

template <ComponentsConcept C, std::convertible_to<typename C::value_type> S>
constexpr auto operator*(C const& c, S const s) {
  return times(c, s, std::make_index_sequence<C::components>{});
};

template <ComponentsConcept C, std::convertible_to<typename C::value_type> S>
constexpr auto operator*(S const s, C const& c) {
  return c * s;
};

template <typename Q>
concept QuaternionConcept = ComponentsConcept<Q> && requires(Q const q) {
  { q.r() } -> std::convertible_to<typename Q::value_type>;
  { q.i() } -> std::convertible_to<typename Q::value_type>;
  { q.j() } -> std::convertible_to<typename Q::value_type>;
  { q.k() } -> std::convertible_to<typename Q::value_type>;
};

template <typename Q, typename T>
struct QuaternionComponents {
  constexpr T r() const {
    return static_cast<Q>(*this)[0];
  }
  constexpr T i() const {
    return static_cast<Q>(*this)[1];
  }
  constexpr T j() const {
    return static_cast<Q>(*this)[2];
  }
  constexpr T k() const {
    return static_cast<Q>(*this)[3];
  }

  constexpr Q adjoint() const {
    return Q(r(), -i(), -j(), -k());
  }

  explicit constexpr operator Vector<T>() const {
    return Vector<T>(i(), j(), k());
  }
};

template <typename T = Number>
struct Quaternion : MutableComponents<4, T>,
                    QuaternionComponents<Quaternion<T>, T> {
  using Base = MutableComponents<4, T>;
  using Base::Base;
  template <std::convertible_to<T> S>
  constexpr Quaternion(S real, Vector<S> const& v)
      : MutableComponents<4, T>{real, v.x(), v.y(), v.z()} {}
  template <std::convertible_to<T> S>
  explicit constexpr Quaternion(Vector<S> const& v)
      : MutableComponents<4, T>{0.f, v.x(), v.y(), v.z()} {}

  using Base::operator==;

  template <size_t... Is>
  constexpr T
  norm(std::index_sequence<Is...> is = std::make_index_sequence<4>{}) const {
    return std::sqrt((... + ((*this)[Is] * (*this)[Is])));
  }

  constexpr auto inverse() const {
    Quaternion result = this->adjoint();
    result /= norm();
    return result;
  }

  static_assert(QuaternionConcept<Quaternion>,
                "Expected Quaternion to satisfy QuaternionConcept");
};

template <QuaternionConcept Q1, QuaternionConcept Q2>
constexpr auto operator*(Q1 const& q1, Q2 const& q2) {
  return Quaternion<
      std::common_type_t<typename Q1::value_type, typename Q2::value_type>>(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[3] * q2[2] - q1[2] * q2[3],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[1] * q2[3] - q1[3] * q2[1],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[2] * q2[1] - q1[1] * q2[2]);
}

template <typename T = Number>
struct UnitQuaternion : Components<4, T>,
                        QuaternionComponents<UnitQuaternion<T>, T> {
  using Base = Components<4, T>;
  using Base::Base;
  constexpr UnitQuaternion(T const r, T const i, T const j, T const k)
      : Base{r, i, j, k} {}
  template <QuaternionConcept Q>
  constexpr UnitQuaternion(Q const& q) : Base{q.r(), q.i(), q.j(), q.k()} {}

  using Base::operator==;

  constexpr auto inverse() const {
    return this->adjoint();
  }

  constexpr T norm() const {
    return 1;
  }

  static_assert(QuaternionConcept<UnitQuaternion>,
                "Expected UnitQuaternion to satisfy QuaternionConcept");
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
template <typename T = Number>
struct HeadingPitchRoll : Components<3, T> {
  using Base = Components<3, T>;
  using Base::Base;
  explicit constexpr HeadingPitchRoll(UnitQuaternion<T> const& q)
      : Base{/* TODO */} {}
  explicit operator UnitQuaternion<T>() const {
    return UnitQuaternion<T>{/* TODO */};
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
template <typename T = Number>
struct RotationVector : Vector<T> {
  using Base = Vector<T>;
  using Base::Base;
  explicit constexpr RotationVector(UnitQuaternion<T> const& q)
      : Base{q.i(), q.j(), q.k()} {
    *this *= 2 * std::acos(q.r()) / std::sqrt(1 - q.r() * q.r());
  }
  explicit operator UnitQuaternion<T>() const {
    return UnitQuaternion<T>{this->length(), (*this)[0] * 0.5, (*this)[1] * 0.5,
                             (*this)[2] * 0.5};
  }
};

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

template <typename T = Number>
struct Tensor : trix::SymmetricMatrix<3, T> {
  using Base = trix::SymmetricMatrix<3, T>;
  using Base::Base;
};

template <ComponentsConcept C>
std::ostream& operator<<(std::ostream& out, C const& c) {
  out << static_cast<std::string>(c);
  return out;
}

} // namespace attitude

#endif
