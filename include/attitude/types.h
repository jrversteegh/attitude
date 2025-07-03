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

namespace attitude {

template <typename T = Number>
using Vector = trix::Vector<3, T>;

template <typename T = Number>
using Matrix = trix::Matrix<3, 3, T>;

template <typename T = Number>
struct Tensor : trix::SymmetricMatrix<3, T> {
  using Base = trix::SymmetricMatrix<3, T>;
  using Base::Base;
};

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

template <ComponentsConcept C>
std::ostream& operator<<(std::ostream& out, C const& c) {
  out << static_cast<std::string>(c);
  return out;
}

} // namespace attitude

#endif
