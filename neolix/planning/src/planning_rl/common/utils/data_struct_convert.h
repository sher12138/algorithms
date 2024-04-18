#include <vector>
#include "common/feature/feature_data_struct.h"

namespace neodrive {
namespace planning_rl {

std::vector<RefPoint> CvtVecref2Point(
    const std::vector<std::vector<double>> &refline_points);

}  // namespace planning_rl
}  // namespace neodrive