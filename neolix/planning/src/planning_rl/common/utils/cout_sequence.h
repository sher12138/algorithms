#pragma once
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace std {
// Forward declare these two, and define them after all the container streams
// operators so that we can recurse from pair -> container -> container -> pair
// properly.
template <class First, class Second>
std::ostream& operator<<(std::ostream& out, const std::pair<First, Second>& p);
}  // namespace std

namespace c10 {
template <class Iter>
void PrintSequence(std::ostream& ss, Iter begin, Iter end);
}  // namespace c10

namespace std {
#define INSTANTIATE_FOR_CONTAINER(container)                 \
  template <class... Types>                                  \
  std::ostream& operator<<(std::ostream& out,                \
                           const container<Types...>& seq) { \
    c10::PrintSequence(out, seq.begin(), seq.end());         \
    return out;                                              \
  }

INSTANTIATE_FOR_CONTAINER(std::vector)
INSTANTIATE_FOR_CONTAINER(std::map)
INSTANTIATE_FOR_CONTAINER(std::set)
#undef INSTANTIATE_FOR_CONTAINER

template <class First, class Second>
inline std::ostream& operator<<(std::ostream& out,
                                const std::pair<First, Second>& p) {
  out << '(' << p.first << ", " << p.second << ')';
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const std::nullptr_t&) {
  out << "(null)";
  return out;
}
}  // namespace std

namespace c10 {
template <class Iter>
inline void PrintSequence(std::ostream& out, Iter begin, Iter end) {
  // Output at most 100 elements -- appropriate if used for logging.
  for (int i = 0; begin != end && i < 100; ++i, ++begin) {
    if (i > 0) out << ' ';
    out << *begin;
  }
  if (begin != end) {
    out << " ...";
  }
}
}  // namespace c10