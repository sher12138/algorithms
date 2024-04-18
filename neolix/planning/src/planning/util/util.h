#pragma once

#include <fstream>

namespace neodrive {
namespace planning {

template <typename T>
std::string DoubleFormat(const T val, const int fixed) {
  auto str = std::to_string(val);
  return str.substr(0, str.find(".") + fixed + 1);
}

}  // namespace planning
}  // namespace neodrive
