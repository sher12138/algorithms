#pragma once

#include <vector>

namespace neodrive {
namespace planning {

class ReedsSheppPath {
 public:
  struct Path {
    std::vector<double> segs_lengths;
    std::vector<char> segs_types;
    double total_length = 0.0;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;
    // true for driving forward and false for driving backward
    std::vector<bool> gear;
  };

  struct Param {
    bool flag = false;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
  };

  struct Config {
    double max_kappa{0.};
    double step_size{0.1};

    double cost_reverse{10};
    double cost_dir_change{3};
    double cost_str_change{3};
    double cost_traversal{1};
  };

  struct Point3d {
    double x{0.};
    double y{0.};
    double phi{0.};
  };

 public:
  /// Must construct with configure
  ReedsSheppPath() = delete;

  /// Constructor with configures
  /// Usage: ReedsShepp obj{{.max_kappa = 1.0, .step_size= 2}};
  /// @param conf Configure of the class
  explicit ReedsSheppPath(Config&& conf);

  /// Pick the shortest ReedsShepp Path from all possible primitives
  /// @param start_node Start pose
  /// @param end_node End pose
  /// @return optimial_path Shortest path of all primitives
  /// @return Generate successfully or not
  bool ShortestRsp(const Point3d& start_node, const Point3d& end_node,
                   Path* optimal_path);

 private:
  const Config conf_{};
};

}  // namespace planning
}  // namespace neodrive
