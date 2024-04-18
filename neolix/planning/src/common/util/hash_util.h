#pragma once

#include <string>

namespace neodrive {
namespace common {

static inline uint64_t HashString(const std::string& str) {
  return std::hash<std::string>()(str);
};

}  // namespace common
}  // namespace neodrive
