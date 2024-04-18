#include "data_struct_convert.h"
#include <vector>
#include "common/feature/feature_data_struct.h"

namespace neodrive {
namespace planning_rl {

std::vector<RefPoint> CvtVecref2Point(
    const std::vector<std::vector<double>> &refline_points) {
  std::vector<RefPoint> res;
  int points_num = refline_points.size();
  for (int i = 0; i < points_num; i++) {
    RefPoint temp;
    temp.x = refline_points[i][0];
    temp.y = refline_points[i][1];
    temp.theta = refline_points[i][2];
    temp.s = refline_points[i][3];
    temp.curvature = refline_points[i][4];
    res.emplace_back(temp);
  }
  return res;
}

}  // namespace planning_rl
}  // namespace neodrive