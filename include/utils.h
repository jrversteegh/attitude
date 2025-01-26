#ifndef ATTITUDE_UTILS_H__
#define ATTITUDE_UTILS_H__

#include <span>

template <typename T, std::size_t Extent = std::dynamic_extent>
struct const_span : public std::span<T, Extent> {
  using std::span<T, Extent>::span;
  constexpr auto cbegin() const noexcept {
    return this->begin();
  }
  constexpr auto cend() const noexcept {
    return this->end();
  }
  template <std::size_t Offset, std::size_t Count>
  constexpr const_span<T, Extent> subspan() const {
    return std::span<T, Extent>::template subspan<Offset, Count>();
  };
};

#endif
