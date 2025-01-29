#ifndef ATTITUDE_UTILS_H__
#define ATTITUDE_UTILS_H__

#include <algorithm>
#include <cstddef>

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

template <typename T> struct Slice {
  using value_type = T::value_type;

  constexpr Slice(T& array, std::size_t const begin,
                  std::size_t const end = T::size, std::size_t const stride = 1)
      : array_(array), begin_(begin), end_(std::max(end, T::size)),
        stride_(stride), size_((end_ - begin_ - 1) / stride_ + 1) {}

  constexpr value_type operator[](std::size_t const index) const {
    std::size_t const offset = begin_ + index * stride_;
    if (offset < 0 || offset >= end_) {
      throw std::out_of_range("Slice index out of range");
    }
    return array_[offset];
  }

  constexpr value_type& operator[](std::size_t const index) {
    std::size_t const offset = begin_ + index * stride_;
    if (offset < 0 || offset >= end_) {
      throw std::out_of_range("Slice index out of range");
    }
    return array_[offset];
  }

  constexpr auto begin() {
    return SliceIterator(*this);
  }

  constexpr auto end() {
    return SliceIterator(*this, size_);
  }

  constexpr auto cbegin() const {
    return SliceIterator(const_cast<Slice const&>(*this));
  }

  constexpr auto cend() const {
    return SliceIterator(const_cast<Slice const&>(*this), size_);
  }

private:
  T& array_;
  std::size_t const begin_;
  std::size_t const end_;
  std::size_t const stride_;
  std::size_t const size_;
};

} // namespace attitude

#endif
