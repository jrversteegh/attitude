#ifndef ATTITUDE_QUATERNION_H__
#define ATTITUDE_QUATERNION_H__

#include "config.h"
#include "types.h"

namespace attitude {

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

} // namespace attitude

#endif
