#pragma once

#include <utility>

#include "boost/optional.hpp"

namespace neodrive {
namespace planning {
namespace base {

template <typename T>
using Optional = boost::optional<T>;

using None = boost::none_t;

const None none{boost::none};

// Returns Optional<T>(v)
template <class T>
inline Optional<std::decay_t<T>> make_optional(T&& v) {
  return Optional<std::decay_t<T>>(std::forward<T>(v));
}

// Returns Optional<T>(args...)
template <class T, class... Args>
inline Optional<T> make_optional(Args&&... args) {
  return Optional<T>(std::forward<Args>(args)...);
}

// Returns Optional<T>(cond, v)
template <class T>
inline Optional<std::decay_t<T>> make_optional(bool cond, T&& v) {
  return Optional<std::decay_t<T>>(cond, std::forward<T>(v));
}

}  // namespace base
}  // namespace planning
}  // namespace neodrive
