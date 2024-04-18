#pragma once

#include <deque>
#include <functional>
#include <map>
#include <queue>
#include <vector>

#include "cyber/common/macros.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/temporal_solver.h"

namespace neodrive {
namespace planning {
namespace obsmap {

void TemporalCollision::PreProcess(const std::vector<TrajectoryData>& data) {
  localmap::ObsState tstate;
  obsmap_.Reset();
  math::AD2 time{data[0].timestamp.front(), data[0].timestamp.back()};
  time_idx_.reserve(data[0].timestamp.size() + 1);
  for (int i = 0; i < data.size(); i++) {
    int start = 0, end = data[0].timestamp.size() - 1;
    while (start < data[i].timestamp.size()) {
      if (math::Sign(data[0].timestamp[start] - data[i].timestamp.front(),
                     1e-4) == 1) {
        break;
      }
      start++;
    }
    while (end >= 0) {
      if (math::Sign(data[0].timestamp[end] - data[i].timestamp.back(), 1e-4) ==
          -1) {
        break;
      }
      end--;
    }
    time_idx_.push_back(std::make_pair(start - 1 > 0 ? start - 1 : 0,
                                       end + 1 >= data[i].timestamp.size()
                                           ? data[i].timestamp.size() - 1
                                           : end + 1));
  }
  for (auto f = time_idx_[0].first; f <= time_idx_[0].second; f++) {
    obsmap_.Update();
    LOG_INFO_IF(log_flag_, "obsmap size: {},{}->[{}, {}]", obsmap_.Length(), f,
                time_idx_[0].first, time_idx_[0].second);
    auto& frame = obsmap_.obs_frame_container().front();
    frame.set_timestamp(data[0].timestamp[f]);
    for (int i = 1; i < data.size(); i++) {
      int j = f - time_idx_[i].first;
      if (j >= 0 && j < data[i].timestamp.size()) {
        frame.AddPolygon(data[i].id, 0, 0.0,
                         Obstacle::ObstacleType::UNKNOWN_MOVABLE,
                         data[i].polygon[j], tstate);
        LOG_INFO_IF(log_flag_, "\t#frame {}: {}", f, i);
      }
    }
    frame.GenerateFrame();
  }
  LOG_INFO("time interval: [{}({}), {}({})]", time_idx_[0].first, time[0],
           time_idx_[0].second, time[1]);
  for (int i = 1; i < data.size(); i++) {
    LOG_INFO_IF(log_flag_, "#agent {}: [{}({}), {}({})]", i, time_idx_[i].first,
                data[i].timestamp[time_idx_[i].first], time_idx_[i].second,
                data[i].timestamp[time_idx_[i].second]);
  }
}

bool TemporalCollision::Solve(const std::vector<TrajectoryData>& data) {
  localmap::ObsState tstate;
  res_.Reset();
  for (auto f = time_idx_[0].first; f <= time_idx_[0].second; f++) {
    int idx = f - time_idx_[0].first;
    if (idx < 0 || idx >= obsmap_.Length()) {
      LOG_ERROR("Exception: Array index is out of bounds in obsmap");
      continue;
    }
    auto nodes = DetectCollisionAt(idx, data[0].polygon[f]);
    if (nodes.size() == 0) {
      continue;
    } else {
      std::ostringstream ss;
      for (auto& node : nodes) {
        ss << "#" << obsmap_[idx].GetObsReverseId(node->id) << ": "
           << node->shape.aabox().cen[0] << "," << node->shape.aabox().cen[1];
        if (res_.obs_idx < 0) {
          res_.time = data[0].timestamp[f];
          res_.obs_idx = obsmap_[idx].GetObsReverseId(node->id);
          res_.position = node->shape.aabox().cen;
        }
      }
      LOG_INFO("Collision occurs in {} s: {}", obsmap_[idx].timestamp(),
               ss.str());
      // return true;
    }
  }
  return res_.obs_idx >= 0;
}

bool TemporalCollision::Process(const std::vector<TrajectoryData>& data) {
  PreProcess(data);
  return Solve(data);
}

std::vector<const math::Node<math::Polygon>*>
TemporalCollision::DetectCollisionAt(int frame_id, const math::Polygon& ego) {
  std::vector<const math::Node<math::Polygon>*> results{};
  if (frame_id < 0 || frame_id >= obsmap_.Length()) {
    LOG_ERROR("Exception: Array index is out of bounds in obsmap");
    return results;
  }
  auto& frame = obsmap_[frame_id];
  LOG_INFO_IF(log_flag_, "#ego: ({}, {})", ego.aabox().cen[0],
              ego.aabox().cen[1]);
  if (frame.obs_polygon_tree().HasOverlapNodesOf(ego)) {
    results = std::move(frame.obs_polygon_tree().GetOverlapNodesOf(ego));
  }
  return results;
}

std::vector<const math::Node<math::Polygon>*>
TemporalCollision::DetectCollisionAt(double frame_timestamp,
                                     const math::Polygon& ego) {}

}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive
