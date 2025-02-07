#ifndef ATTITUDE_UTILS_H__
#define ATTITUDE_UTILS_H__

#include <cstddef>
#include <limits>
#include <stdexcept>

#include "config.h"

namespace attitude {

template <typename S> struct SliceIterator {
  using value_type = typename S::value_type;

  constexpr SliceIterator(S& slice, std::size_t index = 0)
      : slice_(slice), index_(index) {}

  constexpr value_type operator*() {
    return slice_[index_];
  }

  constexpr value_type const& operator*() const {
    return slice_[index_];
  }

  constexpr SliceIterator& operator++() {
    ++index_;
    return *this;
  }

  constexpr bool operator!=(SliceIterator const& other) const {
    return index_ != other.index_;
  }

private:
  S& slice_;
  std::size_t index_;
};

template <typename T, std::size_t B, std::size_t E = T::size, std::size_t S = 1>
struct Slice {
  static constexpr bool const is_slice = true;
  using value_type = T::value_type;
  static constexpr std::size_t const start = B;
  static constexpr std::size_t const stop = E;
  static constexpr std::size_t const size = (E - B - 1) / S + 1;
  static constexpr std::size_t const stride = S;

  constexpr Slice(T& array) : array_(array) {}

  constexpr value_type operator[](std::size_t const index) const {
    std::size_t const offset = start + index * stride;
    if (offset >= stop) {
#ifdef __cpp_exceptions
      throw std::out_of_range("Slice index out of range");
#else
      return std::numeric_limits<Number>::max();
#endif
    }
    return array_[offset];
  }

  constexpr value_type& operator[](std::size_t const index) {
    std::size_t const offset = start + index * stride;
    if (offset >= stop) {
#ifdef __cpp_exceptions
      throw std::out_of_range("Slice index out of range");
#else
      return std::numeric_limits<Number>::max();
#endif
    }
    return array_[offset];
  }

  constexpr auto begin() {
    return SliceIterator(*this);
  }

  constexpr auto end() {
    return SliceIterator(*this, size);
  }

  constexpr auto cbegin() const {
    return SliceIterator(const_cast<Slice const&>(*this));
  }

  constexpr auto cend() const {
    return SliceIterator(const_cast<Slice const&>(*this), size);
  }

private:
  T& array_;
};

template <std::size_t B, std::size_t E, std::size_t S, typename T>
constexpr auto slice(T& array) -> Slice<T, B, E, S> {
  return Slice<T, B, E, S>(array);
}

} // namespace attitude

#endif
