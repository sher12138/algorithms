#include "math_utils.h"

namespace neodrive {
namespace planning {

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

bool IsDoubleEqual(const double a, const double b, const double precision) {
  return abs(a - b) < precision;
}

}  // namespace planning
}  // namespace neodrive
