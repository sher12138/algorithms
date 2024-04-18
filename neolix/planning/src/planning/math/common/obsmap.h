#pragma once

#include <deque>
#include <functional>
#include <map>
#include <queue>
#include <vector>

#include "common/math/polygon2d.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"
#include "cyber/common/macros.h"
#include "obstacle_frame_container.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/math/common/kinematics.h"

namespace neodrive {
namespace planning {
namespace obsmap {

struct NodeCompGreater {
  bool operator()(const math::Node<math::Polygon> *a,
                  const math::Node<math::Polygon> *b) const {
    return a->shape.aabox().lb[1] < b->shape.aabox().lb[1];
  }
};

struct NodeCompLess {
  bool operator()(const math::Node<math::Polygon> *a,
                  const math::Node<math::Polygon> *b) const {
    return a->shape.aabox().rt[1] > b->shape.aabox().rt[1];
  }
};

class ObsMap {
 public:
  ObsMap(int size = 31, bool inverse = false);
  ~ObsMap() = default;
  ObsFrameContainer &operator[](int frame);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int, max_size);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int, time);

  const ObsFrameContainer &At(int frame) const;

  std::deque<ObsFrameContainer> &obs_frame_container() {
    return obs_frame_container_;
  }

  void Update();

  void Update(ObsFrameContainer &currentFrame);

  inline const int Length() const { return size_; }

  inline const bool IsEmpty() const { return size_ == 0; }

  inline const bool IsFull() const { return size_ == max_size_; }

  void VisObstacles(int frame = 0);

  void Reset();

  std::pair<math::AD2, double> Mapping(const math::AD2 &pt, double heading,
                                       std::string &src2dst);

  std::pair<math::AD2, double> Odom2Veh(const global::common::Pose &utm_pose,
                                        const global::common::Pose &odom_pose,
                                        const math::AD2 &pt, double heading);

  std::pair<math::AD2, double> Veh2Odom(const global::common::Pose &utm_pose,
                                        const global::common::Pose &odom_pose,
                                        const math::AD2 &pt, double heading);

  std::pair<math::AD2, double> Utm2Veh(const global::common::Pose &utm_pose,
                                       const global::common::Pose &odom_pose,
                                       const math::AD2 &pt, double heading);

  std::pair<math::AD2, double> Veh2Utm(const global::common::Pose &utm_pose,
                                       const global::common::Pose &odom_pose,
                                       const math::AD2 &pt, double heading);

  std::pair<math::AD2, double> Utm2Odom(const global::common::Pose &utm_pose,
                                        const global::common::Pose &odom_pose,
                                        const math::AD2 &pt, double heading);

  std::pair<math::AD2, double> Odom2Utm(const global::common::Pose &utm_pose,
                                        const global::common::Pose &odom_pose,
                                        const math::AD2 &pt, double heading);

 protected:
  void Push();

  void Push(ObsFrameContainer &frame);

  inline void Pop() { obs_frame_container_.pop_back(); }

 protected:
  std::unordered_map<std::string,
                     std::function<std::pair<std::array<double, 2>, double>(
                         ObsMap *, const global::common::Pose &utm_pose,
                         const global::common::Pose &odom_pose,
                         const math::AD2 &pt, double heading)>>
      coordinate_transformations_;
  int size_{0};
  int time_{0};
  int max_size_{31};
  bool inverse_{false};
  std::deque<ObsFrameContainer> obs_frame_container_{};
  math::KdTree<math::Polygon> idx_kdtree_;
};

}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive
