#ifndef ATTITUDE_UTILS_H__
#define ATTITUDE_UTILS_H__

#include <cmath>

#include "config.h"

namespace attitude {

template <typename T>
constexpr auto sqr(T const& value) {
  return value * value;
}

template <typename... Values>
constexpr auto norm(Values... values) {
  return std::sqrt((... + (values * values)));
}

} // namespace attitude

#endif
