#pragma once
#include "navigation_base.h"

#include <string>

#include "config/navigation_config.h"
#include "src/common/util/hash_util.h"
#include "src/common/util/vector_angle_util.h"
#include "src/planning/planning_map/planning_map.h"
#include "strategy/a_star_lane_search.h"
#include "strategy/breadth_first_search.h"

namespace neodrive {
namespace planning {
using cyberverse::LaneInfoConstPtr;
using cyberverse::RoadInfoConstPtr;
using global::hdmap::Lane;
using JunctionType = autobot::cyberverse::Junction::JunctionType;

void NavigationProcessBase::InitBase() {
  data_center_ = neodrive::planning::DataCenter::Instance();
}

void NavigationProcessBase::UpdateAccumulatedS() {
  auto &lane_seq = ctx_->lane_seq;
  accumulated_s_.clear();
  accumulated_s_.reserve(lane_seq.size() + 1);
  accumulated_s_.emplace_back(0.0);
  for (size_t i = 0; i < lane_seq.size(); ++i) {
    auto &lane = lane_seq[i];
    accumulated_s_.emplace_back(accumulated_s_.back() + lane->TotalLength());
    LOG_DEBUG("id:{}, accu s:{}", hdmap_->GetIdHashString(lane->Id()),
              accumulated_s_.back());
  }
}

void NavigationProcessBase::ProcessIntegratedNavigation() {
  LOG_INFO("ProcessIntegratedNavigation: request_type: {}",
           ctx_->request_msg.ptr->request_type());
  if (ctx_->request_msg.ptr->request_type() != RoutingRequest::NAVIGATER) {
    return;
  }
  if (!UpdateEgoPose()) return;
  LOG_INFO("receive integrated navigation request");
  auto &road_seq = ctx_->road_seq;
  std::vector<std::vector<size_t>> road_seqs;
  std::unordered_map<uint64_t, int> first_road;
  // std::vector<std::string> tmp{
  //     "31564155722974089216", "31564150006284537856", "31564149995689725952",
  //     "31564149995014443008", "31564149987871543296", "31564149994808922112",
  //     "31564155648059625472", "31564155682293534720",
  //     "31564149994775367680"};
  int i = 0;
  for (auto &roads : ctx_->request_msg.ptr->road_seq()) {
    std::vector<size_t> road_ids;
    for (auto id : roads.road_id()) {
      LOG_INFO("road id string:{}, hash:{}", id, common::HashString(id));
      road_ids.push_back(common::HashString(id));
    }
    road_ids = NavigationProcessBase::PostProcessRoadSeq(road_ids);
    if (road_ids.empty()) break;
    LOG_INFO("road_ids size:{}, first road id:{}", road_ids.size(),
             road_ids[0]);
    road_seqs.push_back(road_ids);
    first_road[road_ids[0]] = i++;
    for (auto id : road_ids) LOG_INFO("res id:{}", hdmap_->GetIdHashString(id));
  }
  double x = data_center_->vehicle_state_utm().X(),
         y = data_center_->vehicle_state_utm().Y();
  std::vector<cyberverse::RoadInfoConstPtr> roads;
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  // hdmap_->GetRoads({x, y}, kDefaultRadius, &roads);
  hdmap_->GetLanes({x, y}, kDefaultRadius, &lanes);
  auto cur_road_id = lanes[0]->RoadId();
  if (!lanes.empty())
    LOG_INFO("current road id:{}, string:{}", cur_road_id,
             hdmap_->GetIdHashString(cur_road_id));
  static int cnt = 0;
  if (lanes.empty() || first_road.find(cur_road_id) == first_road.end()) {
    LOG_INFO("current position is not on the first road of any seq");
    cnt = 0;
    return;
  }
  LOG_INFO("cnt:{}", cnt);
  if (cnt++ == 0) {
    LOG_INFO("cnt:{}", cnt);
    int road_idx = first_road[cur_road_id];
    LOG_INFO("first_road size:{}, cur id:{}, idx:{}", first_road.size(),
             cur_road_id, road_idx);
    road_seq.clear();
    for (auto id : road_seqs[road_idx]) {
      LOG_INFO("road id string:{}, hash:{}", hdmap_->GetIdHashString(id), id);
      road_seq.push_back(id);
    }
    ctx_->request_msg.is_updated = true;
  }
}

void NavigationProcessBase::ProcessRoadSeqRequest(
    std::shared_ptr<RoutingRequest> &routing_request) {
  LOG_INFO("receive road seq navigation request");
  auto &road_seqs = ctx_->road_seqs;
  auto &road_set = ctx_->road_set;
  auto &motorway_road_set = ctx_->motorway_road_set;
  const auto &motorway_pts_vec = ctx_->motorway_pts_vec;
  road_seqs.clear();
  road_set.clear();
  motorway_road_set.clear();
  auto GetLanePoint = [](const cyberverse::LaneInfoConstPtr lane,
                         bool is_start) {
    common::math::Vec2d pt;
    int size = lane->Points().size();
    common::math::Vec2d pt1 =
        is_start ? lane->Points()[0] : lane->Points()[size - 1];
    common::math::Vec2d pt2 =
        is_start ? lane->Points()[1] : lane->Points()[size - 2];

    double dist = std::sqrt(std::pow(pt2.x() - pt1.x(), 2) +
                            std::pow(pt2.y() - pt1.y(), 2));
    double unit_x = (pt2.x() - pt1.x()) / dist;
    double unit_y = (pt2.y() - pt1.y()) / dist;
    pt.set_x(pt1.x() + unit_x * 0.2);
    pt.set_y(pt1.y() + unit_y * 0.2);

    return pt;
  };
  for (auto &roads : routing_request->road_seq()) {
    std::vector<size_t> road_ids;
    // get and post-process road id seq
    for (auto id : roads.road_id()) {
      LOG_INFO("road id string:{}, hash:{}", id, common::HashString(id));
      road_ids.push_back(common::HashString(id));
    }
    road_ids = NavigationProcessBase::PostProcessRoadSeq(road_ids);
    if (road_ids.empty()) break;

    // get routing start and end point
    auto start_road = hdmap_->GetRoadById(road_ids[0]);
    auto start_lane_ids = start_road->Sections()[0]->LaneIds();
    auto start_lane = hdmap_->GetLaneById(start_lane_ids.back());
    auto start_lane_pt = GetLanePoint(start_lane, true);
    auto start_pt = routing_request->add_waypoint();
    start_pt->mutable_pose()->set_x(start_lane_pt.x());
    start_pt->mutable_pose()->set_y(start_lane_pt.y());

    auto end_road = hdmap_->GetRoadById(road_ids.back());
    auto end_lane_ids = end_road->Sections()[0]->LaneIds();
    auto end_lane = hdmap_->GetLaneById(end_lane_ids.back());
    auto end_lane_pt = GetLanePoint(end_lane, false);
    auto end_pt = routing_request->add_waypoint();
    end_pt->mutable_pose()->set_x(end_lane_pt.x());
    end_pt->mutable_pose()->set_y(end_lane_pt.y());

    // save road ids
    LOG_INFO("road_ids size:{}, first road id:{}", road_ids.size(),
             road_ids[0]);
    road_seqs.push_back(road_ids);
    for (auto id : road_ids) {
      LOG_INFO("res id:{}", hdmap_->GetIdHashString(id));
      road_set.insert(id);
    }
  }

  // save motorway road from motorway_pts
  for (auto &motorway_pts : motorway_pts_vec) {
    for (auto &pt : motorway_pts) {
      std::vector<cyberverse::LaneInfoConstPtr> lanes;
      hdmap_->GetLanes(pt, kDefaultRadius, &lanes);
      if (lanes.empty()) continue;
      if (road_set.count(lanes[0]->RoadId()) != 0) {
        if (motorway_road_set.count(lanes[0]->RoadId()) == 0)
          LOG_INFO("find motorway road id:{}",
                   hdmap_->GetIdHashString(lanes[0]->RoadId()));
        motorway_road_set.insert(lanes[0]->RoadId());
      }
    }
  }
}

// void NavigationProcessBase::ProcessKeyPointSeqRequest() {
//   LOG_INFO("ProcessKeyPointSeqRequest: request_type: {}",
//            ctx_->request_msg.ptr->request_type());
//   auto context = ctx_;
//   if (context->request_msg.ptr->request_type() != RoutingRequest::KEY_POINTS)
//     return;
//   auto request = context->request_msg.ptr;
//   auto key_point_seq = request->key_point_seq();
//   std::unordered_map<uint64_t, int> first_road;
//   // TODO(yqp): do not generate first_road every frame
//   int i = 0;
//   for (auto key_pts : key_point_seq) {
//     uint64_t first_road_id;
//     std::vector<cyberverse::LaneInfoConstPtr> first_lanes;
//     hdmap_->GetLanes({key_pts.key_points()[0].x(),
//     key_pts.key_points()[0].y()},
//                      kDefaultRadius, &first_lanes);
//     first_road[first_lanes[0]->RoadId()] = i++;
//     LOG_INFO("first key pt x,y: {},{}, road id:{}",
//     key_pts.key_points()[0].x(),
//              key_pts.key_points()[0].y(),
//              hdmap_->GetIdHashString(first_lanes[0]->RoadId()));
//   }
//   std::vector<cyberverse::RoadInfoConstPtr> roads;
//   std::vector<cyberverse::LaneInfoConstPtr> lanes;
//   double x = data_center_->vehicle_state_utm().X(),
//          y = data_center_->vehicle_state_utm().Y();
//   hdmap_->GetLanes({x, y}, kDefaultRadius, &lanes);
//   auto cur_road_id = lanes[0]->RoadId();
//   if (!lanes.empty())
//     LOG_INFO("current road id:{}, string:{}", cur_road_id,
//              hdmap_->GetIdHashString(cur_road_id));
//   static int cnt = 0;
//   if (lanes.empty() || first_road.find(cur_road_id) == first_road.end()) {
//     LOG_INFO("current position is not on the first road of any seq");
//     cnt = 0;
//     return;
//   }
//   LOG_INFO("cnt:{}", cnt);
//   if (cnt++ == 0) {
//     LOG_INFO("cnt:{}", cnt);
//     int road_idx = first_road[cur_road_id];
//     LOG_INFO("first_road size:{}, cur id:{}, idx:{}", first_road.size(),
//              cur_road_id, road_idx);
//     auto new_request = std::make_shared<RoutingRequest>();
//     new_request->mutable_header()->CopyFrom(request->header());
//     if (request->has_request_type()) {
//       new_request->set_request_type(request->request_type());
//     }
//     key_point_seq[road_idx];
//     for (int i = 0; i < key_point_seq[road_idx].key_points_size(); ++i) {
//       auto new_pt = new_request->add_waypoint();
//       new_pt->mutable_pose()->set_x(
//           key_point_seq[road_idx].key_points()[i].x());
//       new_pt->mutable_pose()->set_y(
//           key_point_seq[road_idx].key_points()[i].y());
//     }
//     ctx_->request_msg.ptr = new_request;
//     ctx_->request_msg.is_updated = true;
//   }
// }

void NavigationProcessBase::ProcessKeyPointSeqRequest() {
  auto context = ctx_;
  LOG_INFO("ProcessKeyPointSeqRequest: request_type: {}",
           context->request_msg.ptr->request_type());
  if (context->request_msg.ptr->request_type() != RoutingRequest::KEY_POINTS)
    return;
  auto request = context->request_msg.ptr;
  auto key_point_seq = request->key_point_seq();
  static std::vector<std::unordered_set<uint64_t>> key_road_set;
  if (context->request_msg.is_updated) {
    key_road_set.clear();
    for (auto key_pts : key_point_seq) {
      std::unordered_set<uint64_t> road_set;
      GetRoadSeqFromKeyPoints(key_pts, road_set);
      key_road_set.push_back(road_set);
    }
  }
  std::vector<cyberverse::RoadInfoConstPtr> roads;
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  double x = data_center_->vehicle_state_utm().X(),
         y = data_center_->vehicle_state_utm().Y();
  static int cnt = 0;
  hdmap_->GetLanes({x, y}, kDefaultRadius, &lanes);
  if (lanes.empty()) {
    cnt = 0;
    return;
  }
  int road_idx = -1;
  for (auto lane : lanes) {
    if (!lane->IsOnLane({x, y})) continue;
    auto cur_road_id = lane->RoadId();
    LOG_INFO("current road id:{}, string:{}", cur_road_id,
             hdmap_->GetIdHashString(cur_road_id));
    for (int i = 0; i < key_road_set.size(); ++i) {
      if (key_road_set[i].count(cur_road_id) != 0) {
        road_idx = i;
        break;
      }
    }
    if (road_idx != -1) break;
  }
  if (road_idx == -1) {
    LOG_INFO("current position is not on the first road of any seq");
    cnt = 0;
    return;
  }

  if (context->request_msg.is_updated) {
    LOG_INFO("get new request, reset cnt");
    cnt = 0;
  }
  LOG_INFO("cnt:{}", cnt);
  if (cnt == 0) {
    cnt = 1;
    LOG_INFO("ProcessKeyPointSeqRequest request:{}", request->DebugString());
    auto new_request = std::make_shared<RoutingRequest>();
    new_request->mutable_header()->CopyFrom(request->header());
    if (request->has_request_type()) {
      new_request->set_request_type(request->request_type());
    }
    for (int i = 0; i < key_point_seq[road_idx].key_points_size(); ++i) {
      auto new_pt = new_request->add_waypoint();
      new_pt->mutable_pose()->set_x(
          key_point_seq[road_idx].key_points()[i].x());
      new_pt->mutable_pose()->set_y(
          key_point_seq[road_idx].key_points()[i].y());
      new_pt->mutable_pose()->set_z(
          key_point_seq[road_idx].key_points()[i].z());
      CheckMapPointZLegality(*new_pt);
    }
    new_request->mutable_key_point_seq()->CopyFrom(request->key_point_seq());
    context->request_msg.ptr = new_request;
    context->request_msg.is_updated = true;
  }
}

bool NavigationProcessBase::GenerateKeyWayPoints(
    const std::shared_ptr<RoutingRequest> &routing_request,
    std::shared_ptr<RoutingResult> &routing_respons) {
  auto GetPointZOnLane = [](cyberverse::LaneInfoConstPtr lane, double x,
                            double y) {
    double s = 0.0, l = 0.0, z = 0.0;
    common::math::Vec2d tmp_pt;
    lane->GetProjection({x, y}, &s, &l);
    lane->GetSmoothPoint(s, tmp_pt, &z);
    return z;
  };

  auto context = ctx_;
  const auto &lane_seq = context->lane_seq;
  if (lane_seq.empty()) {
    LOG_ERROR("lane seq is empty, skip GenerateKeyWayPoints");
    return false;
  }
  auto key_points = routing_respons->mutable_key_points();
  auto start_pt = key_points->add_path_points();
  start_pt->set_x(routing_request->waypoint(0).pose().x());
  start_pt->set_y(routing_request->waypoint(0).pose().y());
  start_pt->set_z(GetPointZOnLane(lane_seq[0], start_pt->x(), start_pt->y()));
  int waypoint_idx = 1;
  while (waypoint_idx < routing_request->waypoint_size() &&
         lane_seq[0]->IsOnLane(
             {routing_request->waypoint(waypoint_idx).pose().x(),
              routing_request->waypoint(waypoint_idx).pose().y()})) {
    ++waypoint_idx;
  }
  for (int i = 1; i + 1 < lane_seq.size(); ++i) {
    if (waypoint_idx < routing_request->waypoint_size() &&
        lane_seq[i]->IsOnLane(
            {routing_request->waypoint(waypoint_idx).pose().x(),
             routing_request->waypoint(waypoint_idx).pose().y()})) {
      auto way_pt = key_points->add_path_points();
      way_pt->set_x(routing_request->waypoint(waypoint_idx).pose().x());
      way_pt->set_y(routing_request->waypoint(waypoint_idx).pose().y());
      ++waypoint_idx;
      while (waypoint_idx < routing_request->waypoint_size() &&
             lane_seq[i]->IsOnLane(
                 {routing_request->waypoint(waypoint_idx).pose().x(),
                  routing_request->waypoint(waypoint_idx).pose().y()})) {
        ++waypoint_idx;
      }
      continue;
    }
    auto cur_road = hdmap_->GetRoadById(lane_seq[i]->RoadId());
    auto cur_junction = hdmap_->GetJunctionById(cur_road->JunctionId());
    if (cur_junction != nullptr || lane_seq[i]->TotalLength() < 1.0) continue;
    auto road = hdmap_->GetRoadById(lane_seq[i + 1]->RoadId());
    auto junction = hdmap_->GetJunctionById(road->JunctionId());
    auto last_road = hdmap_->GetRoadById(lane_seq[i - 1]->RoadId());
    auto last_junction = hdmap_->GetJunctionById(last_road->JunctionId());
    LOG_DEBUG("next lane id:{}, is junc:{}, last lane id:{}, is junc:{}",
              hdmap_->GetIdHashString(lane_seq[i + 1]->Id()),
              junction != nullptr,
              hdmap_->GetIdHashString(lane_seq[i - 1]->Id()),
              last_junction != nullptr);
    if ((junction != nullptr &&
         junction->Type() != static_cast<uint32_t>(JunctionType::IN_ROAD)) ||
        (last_junction != nullptr &&
         last_junction->Type() !=
             static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      auto key_pt = key_points->add_path_points();
      if (lane_seq[i]->Points().size() > 3) {
        key_pt->set_x(
            lane_seq[i]->Points()[lane_seq[i]->Points().size() / 2].x());
        key_pt->set_y(
            lane_seq[i]->Points()[lane_seq[i]->Points().size() / 2].y());
        key_pt->set_z(GetPointZOnLane(lane_seq[i], key_pt->x(), key_pt->y()));
      } else {
        double x =
            (lane_seq[i]->Points()[0].x() +
             lane_seq[i]->Points()[lane_seq[i]->Points().size() - 1].x()) /
            2.0;
        double y =
            (lane_seq[i]->Points()[0].y() +
             lane_seq[i]->Points()[lane_seq[i]->Points().size() - 1].y()) /
            2.0;
        key_pt->set_x(x);
        key_pt->set_y(y);
        key_pt->set_z(GetPointZOnLane(lane_seq[i], key_pt->x(), key_pt->y()));
      }
      LOG_DEBUG("keypoint x,y: {}, {}", key_pt->x(), key_pt->y());
    }
  }
  auto end_pt = key_points->add_path_points();
  end_pt->set_x(routing_request->waypoint(routing_request->waypoint_size() - 1)
                    .pose()
                    .x());
  end_pt->set_y(routing_request->waypoint(routing_request->waypoint_size() - 1)
                    .pose()
                    .y());
  end_pt->set_z(GetPointZOnLane(lane_seq.back(), end_pt->x(), end_pt->y()));

  LOG_INFO("keypoints size:{}", key_points->path_points_size());
  return true;
}

void NavigationProcessBase::CheckMapPointZLegality(LaneWaypoint &point) {
  auto GetPointZOnLane = [](cyberverse::LaneInfoConstPtr lane,
                            const LaneWaypoint &point) {
    double s = 0.0, l = 0.0, z = 0.0;
    common::math::Vec2d tmp_pt;
    lane->GetProjection({point.pose().x(), point.pose().y()}, &s, &l);
    lane->GetSmoothPoint(s, tmp_pt, &z);
    return z;
  };

  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  double min_z_diff = std::numeric_limits<double>::max(), nearest_z = 0.0;
  hdmap_->GetLanes({point.pose().x(), point.pose().y()}, kMaxGetLaneRadius,
                   &lanes);
  if (lanes.empty())
    LOG_ERROR("get no lane at x,y: {},{}", point.pose().x(), point.pose().y());
  for (auto &each_lane : lanes) {
    double z_on_lane = GetPointZOnLane(each_lane, point);
    double z_diff = std::fabs(point.pose().z() - z_on_lane);
    if (z_diff < min_z_diff) {
      min_z_diff = z_diff;
      nearest_z = z_on_lane;
    }
  }
  double diff_z_abs = std::fabs(point.pose().z() - nearest_z);
  if (diff_z_abs >= NavigationProcessBase::kMaxZDiffThreshold) {
    LOG_INFO("pose z:{}, nearest z:{}", point.pose().z(), nearest_z);
    point.mutable_pose()->set_z(nearest_z);
  } else {
    LOG_INFO("diff z:{},out of threshold", diff_z_abs);
  }
}

std::string NavigationProcessBase::GetPointMotorwayType(double x, double y,
                                                        double z) {
  std::vector<LaneInfoConstPtr> lanes;
  hdmap_->GetLanes({x, y}, 3.0, &lanes, z);
  if (lanes.empty()) return "0";
  bool is_biking = false, is_on_no_lane = true;
  for (auto lane : lanes) {
    auto lane_multiple_type = lane->LaneMultipleType();
    auto is_on_lane = lane->IsOnLane({x, y});
    is_on_no_lane = !is_on_lane && is_on_no_lane;
    for (int j = 0; j < lane_multiple_type.size(); ++j) {
      if ((lane_multiple_type[j] == global::hdmap::Lane::BIKING ||
           lane_multiple_type[j] == global::hdmap::Lane::INDOOR_LANE) &&
          is_on_lane) {
        is_biking = true;
        break;
      }
    }
  }
  if (is_on_no_lane) return "0";
  // "1": motorway type, "2": non-motorway type
  if (is_biking) return "2";
  return "1";
}

bool NavigationProcessBase::UpdateEgoPose() {
  if (ctx_->is_cloud_navigation) {
    LOG_INFO("is_cloud_navigation, skip check ego pose");
    return true;
  }
  if (data_center_->vehicle_state_utm().IsValid() &&
      data_center_->vehicle_state_proxy().IsValid()) {
    ego_pose_ = data_center_->vehicle_state_utm();
    LOG_INFO("get cur ego pose x,y,z: {:.4f},{:.4f},{:.4f}", ego_pose_.X(),
             ego_pose_.Y(), ego_pose_.Z());
    return true;
  }
  LOG_ERROR("No valid ego pose");
  return false;
}

bool NavigationProcessBase::SearchRoutingRoadSequence(
    const std::shared_ptr<RoutingRequest> &routing_request, int start_idx,
    int end_idx) {
  if (routing_request->request_type() == RoutingRequest::NAVIGATER) {
    LOG_INFO("RoutingRequest::NAVIGATER, skip road navigation");
    return true;
  }
  auto &road_seq = ctx_->road_seq;
  auto &last_road_seq = ctx_->last_road_seq;
  road_seq.clear();

  auto start = routing_request->waypoint(start_idx);
  auto end = routing_request->waypoint(end_idx);
  // cloud navigation waypoint position may be incorrect
  start.set_type(GetPointMotorwayType(start.pose().x(), start.pose().y(),
                                      start.pose().z()));
  end.set_type(
      GetPointMotorwayType(end.pose().x(), end.pose().y(), end.pose().z()));
  LOG_INFO("start type:{}, end type:{}", start.type(), end.type());
  if (ctx_->is_cloud_navigation) {
    // CorrectWaypoint(start);
    // CorrectWaypoint(end);
  }
  ctx_->start_type = start.type();
  ctx_->end_type = end.type();
  auto start_roads =
      GetRoad({start.pose().x(), start.pose().y(), start.pose().z()}, true);
  auto end_roads =
      GetRoad({end.pose().x(), end.pose().y(), end.pose().z()}, false);
  if (start_roads.empty() || end_roads.empty()) {
    LOG_ERROR("failed to locate start or end road!");
    return false;
  }

  std::vector<std::vector<uint64_t>> road_seqs;
  for (const auto start_road : start_roads) {
    if (!last_road_seq.empty()) {
      if (start_road != last_road_seq.back() &&
          (last_road_seq.size() == 1 ||
           (last_road_seq.size() > 1 &&
            start_road != last_road_seq[last_road_seq.size() - 2]))) {
        LOG_INFO("start_road{} is not successor of last route road {}, skip",
                 hdmap_->GetIdHashString(start_road),
                 hdmap_->GetIdHashString(last_road_seq.back()));
        continue;
      }
    }
    for (const auto end_road : end_roads) {
      std::vector<uint64_t> each_road_seq{};
      a_star_road_search_->Search(start_road, end_road, each_road_seq);

      if (each_road_seq.size() == 1) {
        auto road = hdmap_->GetRoadById(each_road_seq[0]);
        if (road == nullptr) continue;
        auto lane_id = road->Sections()[0]->LaneIds()[0];
        if (CheckPtOrderOnLane(
                lane_id, {start.pose().x(), start.pose().y(), start.pose().z()},
                {end.pose().x(), end.pose().y(), end.pose().z()}))
          road_seqs.push_back(each_road_seq);
        continue;
      }
      if (!each_road_seq.empty()) road_seqs.push_back(each_road_seq);
    }
  }
  if (road_seqs.empty()) {
    LOG_ERROR("failed to find road seq!");
    road_seq.clear();
    return false;
  }

  road_seq = FindShortestRoadSeq(
      road_seqs, {start.pose().x(), start.pose().y(), start.pose().z()},
      {end.pose().x(), end.pose().y(), end.pose().z()});

  for (int i = 0; i < road_seq.size(); ++i) {
    LOG_INFO("find road id:{}. string:{}", road_seq[i],
             hdmap_->GetIdHashString(road_seq[i]));
  }

  LOG_INFO("routing success,ret size:{}", ctx_->road_seq.size());
  return ctx_->road_seq.empty();
}

bool NavigationProcessBase::GetRoadSeqFromKeyPoints(
    const RoutingRequest::KeyPoints &key_pts,
    std::unordered_set<uint64_t> &road_set) {
  for (int i = 0; i + 1 < key_pts.key_points().size(); ++i) {
    auto &start = key_pts.key_points()[i];
    auto &end = key_pts.key_points()[i + 1];
    auto start_road = GetRoad({start.x(), start.y(), start.z()}, true);
    auto end_road = GetRoad({end.x(), end.y(), end.z()}, false);
    if (start_road.empty() || end_road.empty()) {
      LOG_ERROR("failed to locate start or end road!");
      return false;
    }
    std::vector<uint64_t> res;
    a_star_road_search_->Search(start_road[0], end_road[0], res);

    for (int i = 0; i < res.size(); ++i) {
      LOG_INFO("road id:{}", hdmap_->GetIdHashString(res[i]));
      road_set.insert(res[i]);
    }
  }
  LOG_INFO("routing success,ret size:{}", road_set.size());
  return !road_set.empty();
}

bool NavigationProcessBase::GenerateLaneSequenceFromRoadSequence(
    const std::shared_ptr<RoutingRequest> &routing_request, int start_idx,
    int end_idx, std::shared_ptr<RoutingResult> &routing_respons) {
  routing_respons->mutable_routing_request()->CopyFrom(*routing_request);
  if (ctx_->road_seq.empty()) {
    LOG_ERROR("failed to search road route with navigator.");
    return false;
  }
  auto start = routing_request->waypoint(start_idx);
  auto end = routing_request->waypoint(end_idx);
  // cloud navigation waypoint position may be incorrect
  start.set_type(GetPointMotorwayType(start.pose().x(), start.pose().y(),
                                      start.pose().z()));
  end.set_type(
      GetPointMotorwayType(end.pose().x(), end.pose().y(), end.pose().z()));
  LOG_INFO("start type:{}, end type:{}", start.type(), end.type());
  if (ctx_->is_cloud_navigation) {
    // CorrectWaypoint(start);
    // CorrectWaypoint(end);
  }
  auto &road_seq = ctx_->road_seq;
  auto start_lanes =
      GetRoadLanes(road_seq[0], {start.pose().x(), start.pose().y()});
  auto end_lanes =
      GetRoadLanes(road_seq.back(), {end.pose().x(), end.pose().y()});
  if (start_lanes.empty() || end_lanes.empty()) {
    LOG_ERROR("failed to locate start or end lane!");
    return false;
  }
  for (const auto &start_lane : start_lanes) {
    for (const auto &end_lane : end_lanes) {
      ctx_->routing_start_lane = start_lane;
      ctx_->routing_end_lane = end_lane;
      ctx_->routing_start_pt = {start.pose().x(), start.pose().y(),
                                start.pose().z()};
      ctx_->routing_end_pt = {end.pose().x(), end.pose().y(), end.pose().z()};
      ctx_->routing_start_lane_id = start_lane->Id();
      ctx_->routing_end_lane_id = end_lane->Id();
      if (!ctx_->renavigation_process) {
        lane_keypoint_map_.insert(std::make_pair(start_lane->Id(), start_idx));
        lane_keypoint_map_.insert(std::make_pair(end_lane->Id(), end_idx));
      }
      LOG_INFO("start lane:{}, end lane:{}",
               hdmap_->GetIdHashString(start_lane->Id()),
               hdmap_->GetIdHashString(end_lane->Id()));

      if (GetRoutingFromRoadSeq(routing_respons)) return true;
      // find no routing, reset lane_keypoint_map_
      if (!ctx_->renavigation_process) {
        lane_keypoint_map_.erase(start_lane->Id());
        lane_keypoint_map_.erase(end_lane->Id());
      }
    }
  }
  return false;
}

std::string NavigationProcessBase::GetRoadType(uint64_t road_id) {
  auto road = hdmap_->GetRoadById(road_id);
  if (road == nullptr) {
    LOG_ERROR("cannot find road in hdmap, road_id:{}",
              hdmap_->GetIdHashString(road_id));
    return "0";
  }
  auto lane_ids = road->Sections()[0]->LaneIds();
  if (lane_ids.empty()) {
    LOG_ERROR("cannot find lane_ids of road road_id:{}",
              hdmap_->GetIdHashString(road_id));
    return "0";
  }
  bool is_motorway = false, is_biking = false;
  for (int i = 0; i < lane_ids.size(); ++i) {
    auto lane = hdmap_->GetLaneById(lane_ids[i]);
    if (lane == nullptr) {
      LOG_ERROR("cannot find lane in hdmap, id:{}, string:{}", lane->Id(),
                hdmap_->GetIdHashString(lane->Id()));
      return "0";
    }
    auto lane_multiple_type = lane->LaneMultipleType();
    for (int j = 0; j < lane_multiple_type.size(); ++j) {
      if (lane_multiple_type[j] == global::hdmap::Lane::CITY_DRIVING)
        is_motorway = true;
      if (lane_multiple_type[j] == global::hdmap::Lane::BIKING)
        is_biking = true;
    }
  }
  if (is_motorway && is_biking) {
    LOG_INFO("road {} is mix type", hdmap_->GetIdHashString(road_id));
    return "0";
  }
  if (is_motorway && !is_biking) {
    LOG_INFO("road {} is motorway type", hdmap_->GetIdHashString(road_id));
    return "1";
  }
  if (!is_motorway && is_biking) {
    LOG_INFO("road {} is nonmotorway type", hdmap_->GetIdHashString(road_id));
    return "2";
  }
  LOG_INFO("road {} is mix type", hdmap_->GetIdHashString(road_id));
  return "0";
}

std::vector<uint64_t> NavigationProcessBase::GetRoad(const AD3 ego_pose,
                                                     bool find_pre) {
  std::vector<uint64_t> res;
  auto const road_cost_map = road_topo_->GetNodeCost();
  auto context = ctx_;
  // get cloest lanes near ego pose
  std::vector<LaneInfoConstPtr> lanes;
  constexpr double kRadius = 0.3;
  for (int i = 0; i < 20; ++i) {
    hdmap_->GetLanes({ego_pose[0], ego_pose[1]}, kRadius + i * kRadius, &lanes,
                     ego_pose[2]);
    if (lanes.size() > 0) {
      break;
    }
  }
  if (lanes.empty()) {
    LOG_ERROR("find no road from ({:.4f}, {:.4f})", ego_pose[0], ego_pose[1]);
    return res;
  }
  const auto &from_to_map = road_topo_->GetFromToMap();
  const auto &to_from_map = road_topo_->GetToFromMap();

  // get cloest roads near ego pose
  for (int i = 0; i < lanes.size(); ++i) {
    auto road = hdmap_->GetRoadById(lanes[i]->RoadId());
    if (!lanes[i]->IsOnLane({ego_pose[0], ego_pose[1]})) continue;
    if ((find_pre && from_to_map.find(road->Id()) != from_to_map.end()) ||
        (!find_pre && to_from_map.find(road->Id()) != to_from_map.end())) {
      // ego pose is not in junction, save current roads
      for (auto lane : lanes) {
        if (!lane->IsOnLane({ego_pose[0], ego_pose[1]})) continue;
        LOG_INFO("find road id:{}, string:{}", lane->RoadId(),
                 hdmap_->GetIdHashString(lane->RoadId()));
        res.push_back(lane->RoadId());
      }
      if (find_pre) context->start_in_junction = false;
      LOG_INFO("start_in_junction:{}", context->start_in_junction);
      return res;
    }
  }

  // ego car is in junction, save prev roads for start pose, successor roads
  // for end pose
  if (context->road_seq.size() < 2 || !find_pre) {
    auto const lane_map =
        find_pre ? lane_topo_->GetToFromMap() : lane_topo_->GetFromToMap();
    int cnt = 0;
    std::unordered_set<uint64_t> road_set;
    for (int i = 0; i < lanes.size(); ++i) {
      auto lane_iter = lane_map.find(lanes[i]->Id());
      if (lane_iter == lane_map.end() || lane_iter->second.empty()) continue;
      auto new_lane = hdmap_->GetLaneById(*(lane_iter->second.begin()));
      if (road_set.find(new_lane->RoadId()) == road_set.end()) {
        road_set.insert(new_lane->RoadId());
        res.push_back(new_lane->RoadId());
        LOG_INFO("find turn road id:{}, string:{}", new_lane->RoadId(),
                 hdmap_->GetIdHashString(new_lane->RoadId()));
      }
    }
  } else {
    context->road_seq.pop_back();
    res.push_back(context->road_seq.back());
  }
  if (find_pre) context->start_in_junction = true;

  LOG_INFO("start_in_junction:{}", context->start_in_junction);
  return res;
}

void NavigationProcessBase::GeneratePathPoints(
    std::shared_ptr<RoutingResult> &routing_respons) {
  auto context = ctx_;
  routing_respons->mutable_measurement()->set_distance(context->route_length);
  auto path = routing_respons->mutable_routing_path();
  for (const auto &road : routing_respons->route()) {
    for (const auto &passage_region : road.road_info().passage_region()) {
      for (const auto &segment : passage_region.segment()) {
        if (segment.start_s() >= segment.end_s()) {
          continue;
        }
        std::vector<global::common::PointENU> tmp_points;
        hdmap_->GetPointsBetweenS(common::HashString(segment.id()),
                                  segment.start_s(), segment.end_s(),
                                  tmp_points);
        for (auto &pt : tmp_points) {
          auto new_pt = path->add_path_points();
          new_pt->set_x(pt.x());
          new_pt->set_y(pt.y());
          new_pt->set_z(pt.z());
        }
      }
    }
  }
}

cyberverse::LaneInfoConstPtr NavigationProcessBase::GetRoadLane(
    const uint64_t road_id, common::math::Vec2d pt) {
  auto road_ = hdmap_->GetRoadById(road_id);
  auto &lanes = road_->Sections()[0]->LaneIds();
  if (lanes.empty()) {
    LOG_ERROR("road:{} has no lane", hdmap_->GetIdHashString(road_id));
    return nullptr;
  }
  double min_dist = std::numeric_limits<double>::max();
  uint32_t min_idx = 0;
  for (uint32_t i = 0; i < lanes.size(); ++i) {
    double dist = hdmap_->GetLaneById(lanes[i])->DistanceTo(pt);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return hdmap_->GetLaneById(lanes[min_idx]);
}

std::vector<LaneInfoConstPtr> NavigationProcessBase::GetRoadLanes(
    const uint64_t road_id, common::math::Vec2d pt) {
  std::vector<LaneInfoConstPtr> res{};
  auto road_ = hdmap_->GetRoadById(road_id);
  auto &lanes = road_->Sections()[0]->LaneIds();
  if (lanes.empty()) {
    LOG_ERROR("road:{} has no lane", hdmap_->GetIdHashString(road_id));
    return {};
  }
  double min_dist = std::numeric_limits<double>::max();
  LaneInfoConstPtr min_lane{nullptr};
  for (uint32_t i = 0; i < lanes.size(); ++i) {
    auto lane = hdmap_->GetLaneById(lanes[i]);
    if (lane == nullptr) continue;
    if (lane->IsOnLane(pt)) res.push_back(lane);
    double dist = lane->DistanceTo(pt);
    if (dist < min_dist) {
      min_dist = dist;
      min_lane = lane;
    }
  }
  if (res.empty() && min_lane != nullptr) res.push_back(min_lane);
  return res;
}

int NavigationProcessBase::GetNearestLaneWithHeading(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq, double x,
    double y) {
  LOG_INFO("GetNearestLaneWithHeading");
  double heading = data_center_->vehicle_state_utm().Heading();
  double min_dist = DBL_MAX;
  int min_idx = -1;
  common::math::Vec2d proj_pt(0.0, 0.0);
  double s_offset = 0.0;
  int s_offset_index = 0;
  for (int i = 0; i < lane_seq.size(); ++i) {
    double dist =
        lane_seq[i]->DistanceTo({x, y}, &proj_pt, &s_offset, &s_offset_index);
    double heading_diff = fabs(lane_seq[i]->heading(s_offset_index) - heading);
    if (fabs(common::math::normalize_angle(heading_diff)) <= 3.1415926 / 2.0 &&
        dist < min_dist) {
      min_idx = i;
      min_dist = dist;
    }
  }
  return min_idx;
}

bool NavigationProcessBase::CheckLaneHeadingValidity(
    const cyberverse::LaneInfoConstPtr &lane) {
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  double heading = data_center_->vehicle_state_utm().Heading();
  common::math::Vec2d proj_pt(0.0, 0.0);
  double s_offset = 0.0;
  int s_offset_index = 0;
  double dist = lane->DistanceTo({x, y}, &proj_pt, &s_offset, &s_offset_index);
  double heading_diff = fabs(lane->heading(s_offset_index) - heading);
  LOG_INFO("lane id:{}, heading diff:{}", hdmap_->GetIdHashString(lane->Id()),
           heading_diff);
  if (fabs(common::math::normalize_angle(heading_diff)) <= 3.1415926 / 2.0)
    return true;
  LOG_ERROR("invalid lane");
  return false;
}

void NavigationProcessBase::CalcLaneSequenceIndex(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq) {
  LOG_INFO("redindex current lane");
  for (auto lane : lane_seq)
    LOG_DEBUG("lane id:{}", hdmap_->GetIdHashString(lane->Id()));
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  auto lane_iter = std::min_element(
      lane_seq.begin(), lane_seq.end(),
      [&](cyberverse::LaneInfoConstPtr left,
          cyberverse::LaneInfoConstPtr right) {
        return left->DistanceTo({x, y}) < right->DistanceTo({x, y});
      });
  auto &current_s = ctx_->current_s;
  current_s = 0;
  if (lane_iter != lane_seq.end())
    current_lane_idx_ = GetIterIndex(lane_seq, *lane_iter, current_s);
  current_s += GetLaneS(*lane_iter, x, y);
}

bool IsParking() {
  auto parking_space = DataCenter::Instance()
                           ->GetNavigationSwapContext()
                           .parking_space_ptr.Get();
  return parking_space && !parking_space->is_park_in() &&
         parking_space->OriginPark() &&
         parking_space->OriginPark()->Type() !=
             global::hdmap::ParkingSpace_ParkingType_ONLANE &&
         parking_space->OriginPark()->Type() !=
             global::hdmap::ParkingSpace_ParkingType_UNKNOWN;
}

void NavigationProcessBase::CheckTakeOverReplan() {
  const auto &chassis = data_center_->vehicle_state_proxy().chassis();
  static bool is_auto_driving = data_center_->is_auto_driving();
  static bool have_left_route = false;
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  auto context = ctx_;
  context->need_reset_ref_generator = false;
  context->manual_trigger_renavigation = false;
  auto &lane_seq = context->lane_seq;
  auto &lane_change_seq = context->lane_change_seq;
  if (!is_auto_driving && data_center_->is_auto_driving() && have_left_route) {
    LOG_INFO("first switch to auto driving mode when return route seq");
    bool is_on_route = false;
    std::vector<cyberverse::LaneInfoConstPtr> new_lane_seq{};
    for (int i = 0; i < lane_seq.size(); ++i) {
      if (lane_seq[i]->IsOnLane({x, y})) is_on_route = true;
      if (is_on_route) {
        LOG_INFO("get new lane seq id: {}",
                 hdmap_->GetIdHashString(lane_seq[i]->Id()));
        new_lane_seq.push_back(lane_seq[i]);
      }
    }
    if (!new_lane_seq.empty()) {
      have_left_route = false;

      lane_seq = new_lane_seq;
      lane_change_seq.clear();
      context->is_lane_change_point_updated = false;
      current_lane_idx_ = -1;
      context->need_reset_ref_generator = true;
      context->navigator_request =
          planning::NavigatorLaneChangeRequest::NO_REQUEST;
      UpdateAccumulatedS();
    } else {
      have_left_route = true;
      context->manual_trigger_renavigation = true;
    }
  } else {
    have_left_route = have_left_route ||
                      (!CheckIsOnRoute() && !data_center_->is_auto_driving());
  }
  is_auto_driving = data_center_->is_auto_driving();
  LOG_INFO("CheckTakeOverReplan result: drive mode:{}, have_left_route:{}",
           is_auto_driving, have_left_route);
}

void NavigationProcessBase::UpdateLaneSequenceIndex(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq) {
  if (lane_seq.empty()) {
    LOG_ERROR("empty lane_seq, skip UpdateLaneSequenceIndex");
    return;
  }

  double x{data_center_->vehicle_state_utm().X()};
  double y{data_center_->vehicle_state_utm().Y()};
  auto &parking_start_pt = ctx_->park_out_start_pt;
  if (IsParking() && parking_start_pt.x() > 1e2) {
    x = parking_start_pt.x();
    y = parking_start_pt.y();
    LOG_INFO("Is parking, update lane index with x = {:.2f}, y = {:.2f}", x, y);
  }
  // CheckTakeOverReplan();
  LOG_INFO("cur lane idx:{}, lane seq size:{}", current_lane_idx_,
           lane_seq.size());
  if (current_lane_idx_ < 0 ||
      lane_seq[current_lane_idx_]->DistanceTo({x, y}) > kDefaultRadius) {
    if (current_lane_idx_ >= 0) {
      LOG_INFO("dist :{},threshold:{}",
               lane_seq[current_lane_idx_]->DistanceTo({x, y}), kDefaultRadius);
    }
    CalcLaneSequenceIndex(lane_seq);
  }
  if (current_lane_idx_ < 0) {
    return;
  }
  auto curr_lane_s = GetLaneS(lane_seq[current_lane_idx_], x, y);
  if (lane_seq[current_lane_idx_]->TotalLength() - curr_lane_s < kLaneEndDist &&
      current_lane_idx_ < lane_seq.size() - 1) {
    current_lane_idx_ += 1;
    curr_lane_s = GetLaneS(lane_seq[current_lane_idx_], x, y);
  }
  LOG_INFO("i:{}, accu s:{}, lane s:{}, id:{}", current_lane_idx_,
           accumulated_s_[current_lane_idx_], curr_lane_s,
           hdmap_->GetIdHashString(lane_seq[current_lane_idx_]->Id()));
  LOG_INFO("lane_boundary_type left:{}, right:{}",
           lane_seq[current_lane_idx_]->right_divider()[0].type,
           lane_seq[current_lane_idx_]->right_divider()[0].type);
  ctx_->current_s = accumulated_s_[current_lane_idx_] + curr_lane_s;
  ctx_->current_lane_idx = current_lane_idx_;
}

double NavigationProcessBase::GetLaneS(const cyberverse::LaneInfoConstPtr &lane,
                                       double x, double y) {
  if (lane == nullptr) {
    return 0.;
  }
  double s, l;
  lane->GetProjection({x, y}, &s, &l);
  LOG_INFO("id:{}, x,y: {},{}, s:{}", hdmap_->GetIdHashString(lane->Id()), x, y,
           s);
  return s;
}

int NavigationProcessBase::GetIterIndex(
    const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq,
    const cyberverse::LaneInfoConstPtr &lane_info, double &current_s) {
  for (auto i = 0; i < lane_seq.size(); ++i) {
    if (lane_seq[i]->Id() == lane_info->Id()) return i;
    current_s += lane_seq[i]->TotalLength();
  }
}

int NavigationProcessBase::GetRoadLaneIdx(const uint64_t lane_id) {
  auto lane = hdmap_->GetLaneById(lane_id);
  if (lane == nullptr) return -1;
  // lane->LaneNo() starts from -1, -2, -3 ...
  return -1 - lane->LaneNo();
}

int NavigationProcessBase::GetSeqLaneIdx(const uint64_t lane_id) {
  for (int i = 0; i < ctx_->lane_seq.size(); ++i) {
    if (ctx_->lane_seq[i]->Id() == lane_id) return i;
  }
  return -1;
}

void NavigationProcessBase::DFSTraverse(
    const uint64_t &end_id, std::vector<uint64_t> &path,
    std::unordered_set<uint64_t> visted_road, double cur_length,
    double fit_length, double continuous_unfit_length, double &res_length,
    double &max_fit_rate) {
  auto cur_id = path.back();
  double cur_road_len = road_topo_->GetRoadCost(cur_id);
  fit_length += cloud_road_set_.count(cur_id) != 0 ? cur_road_len : 0.0;
  cur_length += cur_road_len;
  continuous_unfit_length = cloud_road_set_.count(cur_id) == 0
                                ? continuous_unfit_length + cur_road_len
                                : 0.0;
  double fit_rate = fit_length / cur_length;
  if ((fit_rate < max_fit_rate && cur_length >= res_length / 2) ||
      continuous_unfit_length > 250.0 ||
      visted_road.find(cur_id) != visted_road.end()) {
    return;
  }
  if (cur_id == end_id && fit_rate > max_fit_rate) {
    LOG_INFO("find path, fit rate:{}", fit_rate);
    max_fit_rate = fit_rate;
    cloud_search_paths_.push_back(path);
    res_length = 0.0;
    for (auto id : path) {
      res_length += road_topo_->GetRoadCost(id);
      LOG_INFO("path id:{}", hdmap_->GetIdHashString(id));
    }
    return;
  }
  if (road_topo_->GetFromToMap().find(cur_id) ==
      road_topo_->GetFromToMap().end()) {
    return;
  }
  visted_road.insert(cur_id);
  auto cur_behinds = road_topo_->GetFromToMap().find(cur_id)->second;
  for (auto &next_id : cur_behinds) {
    path.push_back(next_id.first);
    DFSTraverse(end_id, path, visted_road, cur_length, fit_length,
                continuous_unfit_length, res_length, max_fit_rate);
    path.pop_back();
  }
}

std::vector<uint64_t> NavigationProcessBase::PostProcessRoadSeq(
    const std::vector<uint64_t> &road_ids) {
  std::vector<uint64_t> res;
  LOG_INFO("PostProcessRoadSeq");

  if (road_ids.empty()) {
    LOG_ERROR("empty road_ids, skip PostProcessRoadSeq");
    return {};
  }
  uint64_t begin_id = road_ids.front();
  uint64_t end_id = road_ids.back();
  LOG_INFO("road topo addr:{}", (void *)road_topo_);
  LOG_INFO("from to map size:{}, begin:{}, end:{}",
           road_topo_->GetFromToMap().size(), begin_id, end_id);
  std::vector<uint64_t> path{begin_id};
  std::unordered_set<uint64_t> visted_road;
  double max_fit_rate = 0.0;
  double res_length = 0.0;
  cloud_search_paths_.clear();
  cloud_road_set_.clear();
  fully_search_road_set_.clear();
  fully_search_road_set_.insert(begin_id);
  for (auto id : road_ids) cloud_road_set_.insert(id);
  DFSTraverse(end_id, path, visted_road, 0.0, 0.0, 0.0, res_length,
              max_fit_rate);

  // std::vector<std::pair<uint64_t, uint64_t>> path_wights;
  // auto sort_road_ids = road_ids;
  // std::sort(sort_road_ids.begin(), sort_road_ids.end());
  // int paths_size = cloud_search_paths_.size();
  // LOG_INFO("cloud_search_paths_ size:{},sort_road_ids size:{}",
  //          cloud_search_paths_.size(), sort_road_ids.size());
  if (cloud_search_paths_.empty()) {
    LOG_ERROR("cloud_search_paths_ is empty!");
    return {};
  }
  // for (int i = 0; i < paths_size; i++) {
  //   auto cur_path = cloud_search_paths_[i];
  //   std::sort(cur_path.begin(), cur_path.end());
  //   std::vector<uint64_t> difference;
  //   std::set_intersection(sort_road_ids.begin(), sort_road_ids.end(),
  //                         cur_path.begin(), cur_path.end(),
  //                         std::back_inserter(difference));
  //   path_wights.push_back(std::make_pair(i, difference.size()));
  // }
  // LOG_INFO("path_wights size:{}", path_wights.size());
  // std::sort(path_wights.begin(), path_wights.end(),
  //           [](std::pair<uint64_t, uint64_t> path1,
  //              std::pair<uint64_t, uint64_t> path2) {
  //             return path1.second > path2.second;
  //           });
  return cloud_search_paths_.back();
}

TrajectoryPoint NavigationProcessBase::GetTrajectoryPoint(
    const VehicleStateProxy &vehicle_state_proxy) {
  double x = vehicle_state_proxy.X();
  double y = vehicle_state_proxy.Y();
  double heading = vehicle_state_proxy.Heading();
  double vel = vehicle_state_proxy.LinearVelocity();

  TrajectoryPoint init_pt;
  init_pt.set_x(x);
  init_pt.set_y(y);
  init_pt.set_theta(heading);
  init_pt.set_velocity(vel);

  return init_pt;
}

void NavigationProcessBase::CheckTurnHornLights() {
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  double vel = data_center_->vehicle_state_utm().LinearVelocity();
  auto context = ctx_;
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  const auto lane_seq = context->lane_seq;
  auto &light_turn = context->light_turn;
  double turn_light_check_length =
      std::max(static_cast<double>(navigation_config.traffic_turn_light_dist),
               navigation_config.turn_light_ahead_time * vel);
  auto curr_scenario = data_center_->master_info().curr_scenario();
  if ((curr_scenario != ScenarioState::INTERSECTION ||
       curr_scenario != ScenarioState::MOTORWAY_INTERSECTION) &&
      data_center_->last_frame() != nullptr) {
    light_turn = data_center_->last_frame()
                     ->outside_planner_data()
                     .path_observe_ref_l_info.light_turn;
    LOG_INFO("ego car is going to turn by ref_l,HornLightsCmd: {}", (int)light_turn);
  }
  if (CheckCrossroadTurn(turn_light_check_length, light_turn)) {
    // traffic intersection turn
    LOG_INFO("set traffic intersection light,HornLightsCmd: {}", (int)light_turn);
    return;
  } else if (context->navigator_request !=
                 planning::NavigatorLaneChangeRequest::NO_REQUEST &&
             !context->lane_change_seq.empty() &&
             light_turn == HornLightsCmd::NO_TURN) {
    // lane change
    light_turn = context->lane_change_direct == Lane::LEFT_TURN
                     ? HornLightsCmd::LEFT_TURN
                     : HornLightsCmd::RIGHT_TURN;
    LOG_INFO("Find lane change, set turn HornLightsCmd: {}", (int)light_turn);
    return;
  } else if (CheckFrontMergeIn(turn_light_check_length, light_turn) &&
             light_turn == HornLightsCmd::NO_TURN) {
    LOG_INFO("set merge in light,HornLightsCmd: {}", (int)light_turn);
    LOG_INFO("cur s:{}, acc s:{}", ctx_->current_s,
             accumulated_s_[current_lane_idx_ + 1]);
    return;
  }
}

bool NavigationProcessBase::CheckCrossroadTurn(
    double turn_light_check_length, HornLightsCmd::TurnLevel &light_turn) {
  auto context = ctx_;
  const auto &lane_seq = context->lane_seq;
  double len = 0.0;

  for (int i = current_lane_idx_; i < lane_seq.size(); ++i) {
    if (lane_seq[i]->TurnType() != static_cast<uint32_t>(Lane::NO_TURN)) {
      if (len < config::NavigationConfig::Instance()
                    ->navigation_config()
                    .turn_light_close_length ||
          light_turn == HornLightsCmd::NO_TURN) {
        light_turn =
            lane_seq[i]->TurnType() == static_cast<uint32_t>(Lane::RIGHT_TURN)
                ? HornLightsCmd::RIGHT_TURN
                : HornLightsCmd::LEFT_TURN;
        LOG_INFO("set light turn by intersection: {}", (int)light_turn);
        return true;
      } else if (light_turn != HornLightsCmd::NO_TURN) {
        LOG_INFO("far from intersection,use turn light by refl first");
        return false;
      }
    }
    len = accumulated_s_[i] + lane_seq[i]->TotalLength() - context->current_s;
    if (len > turn_light_check_length) break;
  }
  return false;
}  // namespace planning

bool NavigationProcessBase::CheckFrontMergeIn(
    double turn_light_check_length, HornLightsCmd::TurnLevel &light_turn) {
  auto context = ctx_;
  const auto &lane_seq = context->lane_seq;
  double len = 0.0;
  for (int i = current_lane_idx_; i < lane_seq.size(); ++i) {
    if (CheckMergeIn(light_turn, i)) {
      return true;
    }
    // current lane is junction, stop searching forward for merge-in
    auto road = hdmap_->GetRoadById(lane_seq[i]->RoadId());
    auto junction = hdmap_->GetJunctionById(road->JunctionId());
    if (junction != nullptr) {
      LOG_INFO("lane {} is in junction",
               hdmap_->GetIdHashString(lane_seq[i]->Id()));
      break;
    }
    // out of turn_light_check_length, stop searching forward for merge-in
    len = accumulated_s_[i] + lane_seq[i]->TotalLength() - context->current_s;
    LOG_INFO("length:{}, turn_light_check_length:{}", len,
             turn_light_check_length);
    if (len > turn_light_check_length) break;
  }
  return false;
}

bool NavigationProcessBase::CheckMergeIn(HornLightsCmd::TurnLevel &light_turn,
                                         int lane_idx) {
  auto context = ctx_;
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  const auto &to_from_map = lane_topo_->GetToFromMap();
  const auto &lane_seq = context->lane_seq;
  if (lane_idx + 1 >= lane_seq.size()) {
    LOG_INFO("last lane ,skip merge in light check");
    return false;
  }
  if (!lane_seq[lane_idx]->Signals().empty()) {
    LOG_INFO("lane has signals ,skip merge in light check");
    return false;
  }

  // search for left and right merge lane
  auto cur_lane = lane_seq[lane_idx];
  auto next_lane = lane_seq[lane_idx + 1];
  bool left_merge = false, right_merge = false;
  uint64_t left_merge_id = 0, right_merge_id = 0;
  auto road = hdmap_->GetRoadById(cur_lane->RoadId());
  if (road == nullptr) return false;
  if (lane_topo_->IsConnectLane(cur_lane->Id(), next_lane->Id()) &&
      to_from_map.find(next_lane->Id()) != to_from_map.end() &&
      to_from_map.find(next_lane->Id())->second.size() > 1) {
    const auto &from_lane_ids = to_from_map.find(next_lane->Id())->second;
    double left_merge_angle = -std::numeric_limits<double>::max(),
           right_merge_angle = std::numeric_limits<double>::max();
    for (auto id : from_lane_ids) {
      if (id == cur_lane->Id()) continue;
      auto neigh_lane = hdmap_->GetLaneById(id);
      double angle = common::CalcAngle2NeighborVec(cur_lane->Points(),
                                                   neigh_lane->Points());
      LOG_INFO("cur id:{}, neigh id:{}, angle diff:{}",
               hdmap_->GetIdHashString(cur_lane->Id()),
               hdmap_->GetIdHashString(id), angle);
      if (angle > 0 && angle <= right_merge_angle) {
        // find closer right merge lane
        right_merge_id = id;
        right_merge_angle = angle;
        right_merge = true;
      }
      if (angle < 0 && angle >= left_merge_angle) {
        // find closer left merge lane
        left_merge_id = id;
        left_merge_angle = angle;
        left_merge = true;
      }
    }
  }
  LOG_INFO("cur id:{} ,left id:{}, left_merge:{}, right id:{}, right_merge:{}",
           hdmap_->GetIdHashString(cur_lane->Id()),
           hdmap_->GetIdHashString(left_merge_id), left_merge,
           hdmap_->GetIdHashString(right_merge_id), right_merge);

  // check merge condition to decide turn light type
  auto left_neighbour =
      lane_topo_->GetNeighbour(cur_lane->Id(), Lane::LEFT_TURN);
  auto right_neighbour =
      lane_topo_->GetNeighbour(cur_lane->Id(), Lane::RIGHT_TURN);

  if (left_merge) {
    auto left_merge_lane = hdmap_->GetLaneById(left_merge_id);
    auto merge_left_neighbour =
        lane_topo_->GetNeighbour(left_merge_lane->Id(), Lane::LEFT_TURN);
    // check left merge by topological connection
    // if (right_neighbour == 0 && merge_left_neighbour != 0) {
    //   LOG_INFO("left merge from {} to {}",
    //            hdmap_->GetIdHashString(cur_lane->Id()),
    //            hdmap_->GetIdHashString(left_merge_id));
    //   light_turn = HornLightsCmd::LEFT_TURN;
    //   return true;
    // }
    // if (right_neighbour != 0 && merge_left_neighbour == 0) {
    //   return false;
    // }
    // check left merge by lane angle
    double cur_angle =
        common::CalcAngle2NextVec(cur_lane->Points(), next_lane->Points());
    double left_merge_angle = common::CalcAngle2NextVec(
        left_merge_lane->Points(), next_lane->Points());
    LOG_INFO("cur angle:{}, left angle:{}, difference:{}", cur_angle,
             left_merge_angle, cur_angle - left_merge_angle);
    if (cur_angle - left_merge_angle >
        navigation_config.min_merge_angle_diff * kDegree2Radian) {
      LOG_INFO("left merge from {} to {}",
               hdmap_->GetIdHashString(cur_lane->Id()),
               hdmap_->GetIdHashString(left_merge_id));
      light_turn = HornLightsCmd::LEFT_TURN;
      return true;
    } else if (!right_merge && lane_idx != 0 &&
               std::abs(cur_angle - left_merge_angle) <
                   navigation_config.merge_angle_range * kDegree2Radian) {
      auto prev_lane = lane_seq[lane_idx - 1];
      if (right_neighbour != 0 &&
          lane_topo_->IsConnectLane(prev_lane->Id(), cur_lane->Id()) &&
          lane_topo_->IsConnectLane(prev_lane->Id(), right_neighbour) &&
          (left_neighbour == 0 ||
           !lane_topo_->IsConnectLane(prev_lane->Id(), left_neighbour))) {
        LOG_INFO("left merge from {} to {}",
                 hdmap_->GetIdHashString(cur_lane->Id()),
                 hdmap_->GetIdHashString(left_merge_id));
        light_turn = HornLightsCmd::LEFT_TURN;
        return true;
      }
    }
  }
  if (right_merge) {
    auto right_merge_lane = hdmap_->GetLaneById(right_merge_id);
    auto merge_right_neighbour =
        lane_topo_->GetNeighbour(right_merge_lane->Id(), Lane::RIGHT_TURN);
    // check right merge by topological connection
    // if (left_neighbour == 0 && merge_right_neighbour != 0) {
    //   LOG_INFO("right merge from {} to {}",
    //            hdmap_->GetIdHashString(cur_lane->Id()),
    //            hdmap_->GetIdHashString(right_merge_id));
    //   light_turn = HornLightsCmd::RIGHT_TURN;
    //   return true;
    // }
    // if (left_neighbour != 0 && merge_right_neighbour == 0) {
    //   return false;
    // }
    // check roght merge by lane angle
    double cur_angle =
        common::CalcAngle2NextVec(cur_lane->Points(), next_lane->Points());
    double right_merge_angle = common::CalcAngle2NextVec(
        right_merge_lane->Points(), next_lane->Points());
    LOG_INFO("cur angle:{}, right angle:{}, difference:{}", cur_angle,
             right_merge_angle, cur_angle - right_merge_angle);
    if (cur_angle - right_merge_angle >
        navigation_config.min_merge_angle_diff * kDegree2Radian) {
      LOG_INFO("right merge from {} to {}",
               hdmap_->GetIdHashString(cur_lane->Id()),
               hdmap_->GetIdHashString(right_merge_id));
      light_turn = HornLightsCmd::RIGHT_TURN;
      return true;
    } else if (!left_merge && lane_idx != 0 &&
               std::abs(cur_angle - right_merge_angle) <
                   navigation_config.merge_angle_range * kDegree2Radian) {
      auto prev_lane = lane_seq[lane_idx - 1];
      if (left_neighbour != 0 &&
          lane_topo_->IsConnectLane(prev_lane->Id(), cur_lane->Id()) &&
          lane_topo_->IsConnectLane(prev_lane->Id(), left_neighbour) &&
          (right_neighbour == 0 ||
           !lane_topo_->IsConnectLane(prev_lane->Id(), right_neighbour))) {
        LOG_INFO("right merge from {} to {}",
                 hdmap_->GetIdHashString(cur_lane->Id()),
                 hdmap_->GetIdHashString(right_merge_id));
        light_turn = HornLightsCmd::RIGHT_TURN;
        return true;
      }
    }
  }
  return false;
}

std::vector<uint64_t> NavigationProcessBase::FindShortestLaneSeq(
    const std::vector<std::vector<uint64_t>> &lane_seqs, const Vec3d &start,
    const Vec3d &end) {
  const auto context = ctx_;
  double min_len = DBL_MAX;
  int min_idx = 0;
  for (int i = 0; i < lane_seqs.size(); ++i) {
    if (lane_seqs[i].empty()) continue;
    double len = 0.0;
    for (int j = 0; j < lane_seqs[i].size(); ++j) {
      auto lane = hdmap_->GetLaneById(lane_seqs[i][j]);
      if (j == 0) {
        len += lane->TotalLength() - GetLaneS(lane, start.x(), start.y());
      } else if (j == lane_seqs[i].size() - 1) {
        len += GetLaneS(lane, end.x(), end.y());
      } else {
        len += lane->TotalLength();
      }
    }
    if (len < min_len) {
      min_len = len;
      min_idx = i;
    }
  }
  return lane_seqs[min_idx];
}

std::vector<uint64_t> NavigationProcessBase::FindShortestRoadSeq(
    const std::vector<std::vector<uint64_t>> &road_seqs, const Vec3d &start,
    const Vec3d &end) {
  const auto context = ctx_;
  double min_len = DBL_MAX;
  int min_idx = 0;
  for (int i = 0; i < road_seqs.size(); ++i) {
    if (road_seqs[i].empty()) continue;
    double len = 0.0;
    for (int j = 0; j < road_seqs[i].size(); ++j) {
      auto road = hdmap_->GetRoadById(road_seqs[i][j]);
      if (j == 0) {
        auto start_lane_ids = road->Sections()[0]->LaneIds();
        for (int k = 0; k < start_lane_ids.size(); ++k) {
          auto lane = hdmap_->GetLaneById(start_lane_ids[k]);
          if (lane->IsOnLane({start.x(), start.y()})) {
            len += lane->TotalLength() - GetLaneS(lane, start.x(), start.y());
            break;
          }
        }
      } else if (j == road_seqs[i].size() - 1) {
        auto end_lane_ids = road->Sections()[0]->LaneIds();
        for (int k = 0; k < end_lane_ids.size(); ++k) {
          auto lane = hdmap_->GetLaneById(end_lane_ids[k]);
          if (lane->IsOnLane({end.x(), end.y()})) {
            len += GetLaneS(lane, end.x(), end.y());
            break;
          }
        }
      } else {
        len += road_topo_->GetRoadCost(road_seqs[i][j]);
      }
    }
    if (len < min_len) {
      min_len = len;
      min_idx = i;
    }
  }
  return road_seqs[min_idx];
}

bool NavigationProcessBase::CheckIsOnRoute() {
  if (ctx_->is_sim) return true;
  common::math::Vec2d pose{0., 0.};
  if (ctx_->localization_msg.ptr->has_pose()) {
    const auto &imu_pose = ctx_->localization_msg.ptr->pose().position();
    pose = {imu_pose.x(), imu_pose.y()};
    LOG_INFO("imu pose ({}, {})", pose.x(), pose.y());
  } else {
    LOG_ERROR("not get localization pose, clear ego pose");
    return false;
  }

  auto lane_seq = ctx_->lane_seq;
  for (auto &each_lane : lane_seq) {
    if (each_lane->IsOnLane(pose)) return true;
  }
  return false;
}

bool NavigationProcessBase::CheckIsOnRouteWithBaseLink() {
  if (ctx_->is_sim) return true;
  common::math::Vec2d pose{data_center_->vehicle_state_utm().X(),
                           data_center_->vehicle_state_utm().Y()};
  LOG_INFO("base link pose {}, {}", pose.x(), pose.y());

  auto lane_seq = ctx_->lane_seq;
  for (auto &each_lane : lane_seq) {
    if (each_lane->IsOnLane(pose)) return true;
  }
  if (!ctx_->lane_change_seq.empty()) {
    auto lane_change_seq = ctx_->lane_change_seq;
    for (auto &each_lane : lane_change_seq) {
      if (each_lane->IsOnLane(pose)) return true;
    }
  }
  return false;
}

bool NavigationProcessBase::CheckIsOnLaneChangeRoute() {
  if (ctx_->is_sim) return true;
  common::math::Vec2d pose{0., 0.};
  if (ctx_->localization_msg.ptr->has_pose()) {
    const auto &imu_pose = ctx_->localization_msg.ptr->pose().position();
    pose = {imu_pose.x(), imu_pose.y()};
    LOG_INFO("imu pose ({}, {})", pose.x(), pose.y());
  } else {
    LOG_ERROR("not get localization pose, clear ego pose");
    return false;
  }

  if (!ctx_->lane_change_seq.empty()) {
    auto lane_change_seq = ctx_->lane_change_seq;
    for (auto &each_lane : lane_change_seq) {
      if (each_lane->IsOnLane(pose)) return true;
    }
  }
  return false;
}

void NavigationProcessBase::CalculateMileage() {
  auto lane_seq = ctx_->lane_seq;
  if (lane_seq.empty()) {
    routing_destination_s_ = 0.0;
    return;
  }
  auto &routing_end_pt = ctx_->routing_request->waypoint(
      ctx_->routing_request->waypoint_size() - 1);
  routing_destination_s_ = accumulated_s_[lane_seq.size() - 1] +
                           GetLaneS(lane_seq.back(), routing_end_pt.pose().x(),
                                    routing_end_pt.pose().y());
  LOG_INFO("dist_to_routing_destination:{}", routing_destination_s_);
}

void NavigationProcessBase::AddRoute(
    std::shared_ptr<RoutingResult> &routing_respons,
    const cyberverse::LaneInfoConstPtr &lane, uint64_t road_id, Vec3d *start,
    Vec3d *end) {
  auto new_road_info = routing_respons->add_route()->mutable_road_info();
  new_road_info->set_id(hdmap_->GetIdHashString(road_id));
  auto new_seg = new_road_info->add_passage_region()->add_segment();
  new_seg->set_id(hdmap_->GetIdHashString(lane->Id()));
  new_seg->set_start_s(start ? GetLaneS(lane, start->x(), start->y()) : 0.0);
  new_seg->set_end_s(end ? GetLaneS(lane, end->x(), end->y())
                         : lane->TotalLength());
  // special s for one road lane change
  const auto &map = ctx_->one_road_lane_change_s_map;
  const auto map_iter = map.find(lane->Id());
  if (map_iter != map.end()) {
    if (map_iter->second.first)
      new_seg->set_end_s(map_iter->second.second);
    else
      new_seg->set_start_s(map_iter->second.second);
  }
  double &route_length = ctx_->route_length;
  if (start) route_length = 0;
  route_length += (new_seg->end_s() - new_seg->start_s());
}

void NavigationProcessBase::CorrectWaypoint(LaneWaypoint &pt) {
  if (ctx_->is_nonmotorway_map) {
    pt.set_type("0");
    LOG_INFO("current map is beijing/shanghai/yancheng, skip");
    return;
  }

  auto GetLaneType = [](LaneInfoConstPtr lane) {
    for (int i = 0; i < lane->LaneMultipleType().size(); ++i) {
      if (lane->LaneMultipleType()[i] ==
              static_cast<uint32_t>(global::hdmap::Lane::BIKING) ||
          lane->LaneMultipleType()[i] ==
              static_cast<uint32_t>(global::hdmap::Lane::INDOOR_LANE))
        return "2";
    }
    return "1";
  };

  std::vector<LaneInfoConstPtr> lanes;
  hdmap_->GetLanes({pt.pose().x(), pt.pose().y()}, 10.0, &lanes, pt.pose().z());
  LaneInfoConstPtr same_type_lane = nullptr;
  for (auto lane : lanes) {
    LOG_INFO("lane id: {}", hdmap_->GetIdHashString(lane->Id()));
    if (pt.type() == GetLaneType(lane)) {
      same_type_lane = lane;
      break;
    }
  }
  if (same_type_lane == nullptr) {
    LOG_ERROR("find no correct lane for pt x, y, type:{}, {}, {}",
              pt.pose().x(), pt.pose().y(), pt.type());
    pt.set_type("0");
    return;
  }
  auto road = hdmap_->GetRoadById(same_type_lane->RoadId());
  auto lane_ids = road->Sections()[0]->LaneIds();
  for (int i = lane_ids.size() - 1; i >= 0; --i) {
    auto lane = hdmap_->GetLaneById(lane_ids[i]);
    if (pt.type() == GetLaneType(lane)) {
      same_type_lane = lane;
      break;
    }
  }
  if (same_type_lane->IsOnLane({pt.pose().x(), pt.pose().y()})) {
    LOG_INFO("current pt is on right lane, x, y, type:{}, {}, {}",
             pt.pose().x(), pt.pose().y(), pt.type());
    return;
  }
  auto new_pt = same_type_lane->Points();
  if (same_type_lane->Points().size() > 2) {
    pt.mutable_pose()->set_x(
        same_type_lane->Points()[same_type_lane->Points().size() / 2].x());
    pt.mutable_pose()->set_y(
        same_type_lane->Points()[same_type_lane->Points().size() / 2].y());
    pt.mutable_pose()->set_z(-1000.0);
  } else {
    double x =
        (same_type_lane->Points()[0].x() + same_type_lane->Points()[1].x()) /
        2.0;
    double y =
        (same_type_lane->Points()[0].y() + same_type_lane->Points()[1].y()) /
        2.0;
    pt.mutable_pose()->set_x(x);
    pt.mutable_pose()->set_y(y);
    pt.mutable_pose()->set_z(-1000.0);
  }
  LOG_INFO("get new pt x, y, type:{}, {}, {}", pt.pose().x(), pt.pose().y(),
           pt.type());
}

bool NavigationProcessBase::CheckPtOrderOnLane(uint64_t id, const Vec3d &start,
                                               const Vec3d &end) {
  auto lane = hdmap_->GetLaneById(id);
  if (lane == nullptr) {
    LOG_ERROR("find no lane by id:{}", hdmap_->GetIdHashString(id));
    return false;
  }
  double start_s = 0.0, end_s = 0.0, l = 0.0;
  lane->GetProjection({start.x(), start.y()}, &start_s, &l);
  lane->GetProjection({end.x(), end.y()}, &end_s, &l);
  LOG_INFO("start s:{}, end s:{}", start_s, end_s);
  return (start_s < end_s);
}

}  // namespace planning
}  // namespace neodrive
