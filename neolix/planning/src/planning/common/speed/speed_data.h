#pragma once

#include <algorithm>
#include <sstream>
#include <vector>

#include "speed_data_generator.h"
#include "speed_point.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/common/st_graph_data/st_graph_data.h"

namespace neodrive {
namespace planning {

class SpeedData {
 public:
  SpeedData() = default;
  ~SpeedData() = default;

  SpeedData(const std::vector<SpeedPoint> &speed_points);

  bool build_from_st_points(const std::vector<STPoint> &st_points,
                            const double init_v = 0.0,
                            const double init_a = 0.0,
                            const double init_j = 0.0);

  bool get_speed_point_with_time(const double time,
                                 SpeedPoint *const speed_point) const;

  double total_time() const;

  bool sanity_check() const;

  void clear();

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<SpeedPoint>, speed_vector)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(STGraphData, st_graph_data)

 private:
  bool find_index(const double s, std::size_t &index) const;

  std::vector<SpeedPoint> speed_vector_;
  STGraphData st_graph_data_;
};

}  // namespace planning
}  // namespace neodrive
