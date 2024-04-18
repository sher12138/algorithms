#include "navigation_component.h"

#include "a_star_navigation.h"
#include "breadth_first_navigation.h"
#include "nonmotorway_navigation.h"
#include "reader_manager/reader_manager.h"
#include "src/common/util/hash_util.h"
#include "src/common/util/time_util.h"
#include "src/planning/common/parking_space/horizontal_park.h"
#include "src/planning/common/parking_space/oblique_park.h"
#include "src/planning/common/parking_space/onlane_park.h"
#include "src/planning/common/parking_space/park_util.h"
#include "src/planning/common/parking_space/vertical_park.h"

namespace neodrive {
namespace planning {
namespace {
static constexpr double kMaxZDiffThreshold = 2.0;
}  // namespace
using cyberverse::LaneInfoConstPtr;

void NavigationComponent::SetRoutingDestination(
    const std::shared_ptr<RoutingRequest> &new_request) {
  auto data_center = DataCenter::Instance();
  auto hdmap = cyberverse::HDMap::Instance();
  if (GetContext()->request_msg.ptr->request_type() !=
      RoutingRequest::NAVIGATER) {
    auto size = new_request->waypoint_size();
    data_center->set_routing_destination(
        {new_request->waypoint(size - 1).pose().x(),
         new_request->waypoint(size - 1).pose().y(),
         new_request->waypoint(size - 1).pose().z()});
  } else {
    const auto &road_seq = GetContext()->road_seq;
    auto road = hdmap->GetRoadById(road_seq.back());
    if (road == nullptr) return;
    auto lane_ids = road->Sections()[0]->LaneIds();
    if (lane_ids.empty()) return;
    auto lane = hdmap->GetLaneById(lane_ids[0]);
    data_center->set_routing_destination(
        {lane->Points().back().x(), lane->Points().back().y(), 0.0});
  }
}

NavigationComponent::~NavigationComponent() { proc_thread_->detach(); }

bool NavigationComponent::Init() {
  if (initialized_) {
    return true;
  }
  auto reader_manager = neodrive::common::ReaderManager::Instance();
  if (!reader_manager->Init(node_)) {
    LOG_ERROR("ReaderManager init failed!");
    return false;
  }
  auto drive_strategy = neodrive::common::config::GetDriveStrategy();
  auto drive_stratedy_type_ =
      neodrive::common::config::GetDriveStrategyType(drive_strategy);
  if (drive_stratedy_type_ ==
      neodrive::common::config::DriveStrategyType::ERROR) {
    LOG_ERROR("GetDriveStrategyType::ERROR");
    return false;
  }

  data_center_ = neodrive::planning::DataCenter::Instance();

  auto &topic_config = neodrive::common::config::CommonConfig::Instance()
                           ->topics_config()
                           .topics;
  auto &navigation_config =
      config::NavigationConfig::Instance()->navigation_config();
  routing_pub_ = node_->CreateWriter<RoutingResult>(
      topic_config.at("routing_result").topic);
  routing_request_pub_ = node_->CreateWriter<RoutingRequest>(
      topic_config.at("routing_request").topic);
  is_on_map_pub_ =
      node_->CreateWriter<IsOnMap>(topic_config.at("is_on_map").topic);
  horn_light_cmd_pub_ = node_->CreateWriter<HornLightsCmd>(
      topic_config.at("horn_light_cmd").topic);
  refer_line_pub_ = node_->CreateWriter<neodrive::planning::ADCTrajectory>(
      topic_config.at("pnc_refer_line").topic);

  GetContext()->localization_msg.reader =
      reader_manager->CreateReader<LocalizationEstimate>(
          topic_config.at("localization_pose").topic);
  GetContext()->request_msg.reader =
      reader_manager->CreateReader<RoutingRequest>(
          topic_config.at("routing_request").topic);
  GetContext()->chassis_msg.reader = reader_manager->CreateReader<Chassis>(
      topic_config.at("planning_chassis").topic);
  GetContext()->global_state_msg.reader =
      reader_manager->CreateReader<GlobalState>(
          topic_config.at("pnc_global_state").topic);
  GetContext()->planning_interface_msg.reader =
      reader_manager->CreateReader<PlanningInterface>(
          topic_config.at("openapi_planning_interface").topic);
  GetContext()->pose_base_link_in_utm_msg.reader =
      reader_manager->CreateReader<PoseStamped>(
          topic_config.at("pose_base_link_in_utm").topic);

  GetContext()->hdmap = cyberverse::HDMap::Instance();
  GetContext()->road_topo = cyberverse::RoadTopo::Instance();
  GetContext()->lane_topo = cyberverse::LaneTopo::Instance();

  if (common::config::CommonConfig::Instance()
          ->drive_strategy_config()
          .enable_motorway_navigation) {
    NavigationStrategy type =
        static_cast<NavigationStrategy>(navigation_config.navigation_strategy);
    if (type == NavigationStrategy::ASTAR) {
      navigation_ptr_ = std::make_shared<AStarNavigationProcess>(GetContext());
    } else if (type == NavigationStrategy::BREADTH_FIRST) {
      navigation_ptr_ =
          std::make_shared<BreadthFirstNavigationProcess>(GetContext());
    } else {
      return false;
    }
  } else {
    navigation_ptr_ =
        std::make_shared<NonMotorwayNavigationProcess>(GetContext());
  }
  current_ref_generator_ = std::make_shared<RefeLineGenerator>(&ctx_);
  target_ref_generator_ = std::make_shared<RefeLineGenerator>(&ctx_);
  latest_request_ = std::make_shared<RoutingRequest>();
  latest_result_ = std::make_shared<RoutingResult>();
  if (!navigation_ptr_->Init()) return false;
  proc_thread_ = std::make_unique<std::thread>(
      std::bind(&NavigationComponent::Proc, this));
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "navigation_thread", proc_thread_.get());
  initialized_ = true;
  return true;
}

void NavigationComponent::CheckParkingSpace(
    const std::shared_ptr<RoutingRequest> &new_request) {
  auto &navigation_swap_context =
      DataCenter::Instance()->GetNavigationSwapContext();
  auto &current_parking_space = GetContext()->current_parking_space;
  auto hdmap = cyberverse::HDMap::Instance();
  if (new_request->has_parking_id()) {
    auto parking_space_ptr = hdmap->GetParkingSpaceById(
        common::HashString(new_request->parking_id()));
    if (parking_space_ptr != nullptr) {
      std::shared_ptr<ParkingSpace> parking_ptr;
      if (parking_space_ptr->Type() ==
          global::hdmap::ParkingSpace_ParkingType_VERTICAL) {
        parking_ptr = std::make_shared<VerticalPark>(parking_space_ptr);
      } else if (parking_space_ptr->Type() ==
                 global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
        parking_ptr = std::make_shared<HorizontalPark>(parking_space_ptr);
      } else if (parking_space_ptr->Type() ==
                 global::hdmap::ParkingSpace_ParkingType_OBLIQUE) {
        parking_ptr = std::make_shared<ObliquePark>(parking_space_ptr);
      } else if (parking_space_ptr->Type() ==
                 global::hdmap::ParkingSpace_ParkingType_ONLANE) {
        parking_ptr = std::make_shared<OnlanePark>(parking_space_ptr);
      }
      if (parking_ptr) {
        parking_ptr->set_is_park_in(true);
        parking_ptr->set_is_finished(false);
        LOG_INFO("recv park in task,park id:{},type:{}",
                 hdmap->GetIdHashString(parking_ptr->OriginPark()->Id()),
                 parking_ptr->OriginPark()->Type());
        navigation_swap_context.parking_space_ptr.Set(parking_ptr);
      }
    }
  } else if (current_parking_space) {
    std::shared_ptr<ParkingSpace> parking_ptr;
    if (current_parking_space->Type() ==
        global::hdmap::ParkingSpace_ParkingType_VERTICAL) {
      parking_ptr = std::make_shared<VerticalPark>(current_parking_space);
    } else if (current_parking_space->Type() ==
               global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
      parking_ptr = std::make_shared<HorizontalPark>(current_parking_space);
    } else if (current_parking_space->Type() ==
               global::hdmap::ParkingSpace_ParkingType_OBLIQUE) {
      parking_ptr = std::make_shared<ObliquePark>(current_parking_space);
    } else if (current_parking_space->Type() ==
               global::hdmap::ParkingSpace_ParkingType_ONLANE) {
      parking_ptr = std::make_shared<OnlanePark>(current_parking_space);
    }
    if (!use_parking_out_) {
      parking_ptr = nullptr;
      navigation_swap_context.parking_space_ptr.Set(nullptr);
    }
    if (parking_ptr) {
      parking_ptr->set_is_park_in(false);
      LOG_INFO("recv park out task,park id:{},type:{}",
               hdmap->GetIdHashString(parking_ptr->OriginPark()->Id()),
               parking_ptr->OriginPark()->Type());
      navigation_swap_context.parking_space_ptr.Set(parking_ptr);
    }
  } else {
    navigation_swap_context.parking_space_ptr.Set(nullptr);
  }
}

void NavigationComponent::Proc() {
  LOG_WARN("[thread_name]navigation_component");
  auto frequency = config::NavigationConfig::Instance()
                       ->navigation_config()
                       .routing_period_frequency;
  auto &openapi_cmd = GetContext()->planning_interface_msg;
  std::string junction_id;
  bool has_traffic_light;
  bool is_on_route;
  double distance_to_zone;
  uint32_t junction_type;
  uint64_t period_time = 1000 / std::max(frequency, 1);
  uint64_t start_time{0}, end_time{0}, delta_time{0};

  while (!neodrive::cyber::IsShutdown()) {
    start_time = common::NowMilliSec();
    Observe();
    config::NavigationConfig::Instance()->ReLoadConfigFromJson();
    UpdateIsOnMapStatus();
    if (GetContext()->current_parking_space)
      LOG_INFO("park id:{}", cyberverse::HDMap::Instance()->GetIdHashString(
                                 GetContext()->current_parking_space->Id()));
    ProcessRoutingRequest();
    if (GetContext()->request_msg.is_updated) {
      LOG_INFO("Get new route request!");
      Reset();
      auto new_request = GetContext()->request_msg.ptr;
      Vec2d new_end;
      if (IsChangeEndPoint(new_request, new_end)) {
        auto changed_request = std::make_shared<RoutingRequest>();
        changed_request->CopyFrom(*new_request);
        auto pose = changed_request->mutable_end()->mutable_pose();
        pose->set_x(new_end.x());
        pose->set_y(new_end.y());
        LOG_INFO(
            "Change request end to x = {:.2f}, y = {:.2f}, "
            "original end is x = {:.2f}, y = {:.2f}",
            pose->x(), pose->y(), new_request->end().pose().x(),
            new_request->end().pose().y());

        RequestProcess(changed_request);
        SetRoutingDestination(latest_request_);
      } else {
        RequestProcess(new_request);
        SetRoutingDestination(latest_request_);
      }
      CheckParkingSpace(latest_request_);
      navigation_ptr_->Process(latest_request_, latest_result_);
      // navigation_ptr_->GenerateKeyWayPoints(latest_request_, latest_result_);
      LOG_DEBUG("routing result:{}", latest_result_->DebugString());
      routing_pub_->Write(latest_result_);
      broadcast_last_result_ = true;
    }
    navigation_ptr_->RunOnce();
    for (auto lane : GetContext()->lane_seq)
      LOG_INFO("lane id:{}",
               cyberverse::HDMap::Instance()->GetIdHashString(lane->Id()));
    data_center_->set_route_lanes(GetContext()->lane_seq);

    // generate reference line
    if (GetContext()->need_reset_ref_generator) {
      LOG_INFO("need_reset_ref_generator");
      GetContext()->need_reset_ref_generator = false;
      current_ref_generator_.reset(new RefeLineGenerator(GetContext()));
      target_ref_generator_.reset(new RefeLineGenerator(GetContext()));
      GetContext()->target_utm_ref = nullptr;
      GetContext()->current_utm_ref = nullptr;
    }
    if (GetContext()->lane_seq.size() >= 1)
      navigation_ptr_->ComputeReferenceLine(current_ref_generator_,
                                            target_ref_generator_);

    // check clear task
    if ((openapi_cmd.is_available &&
         openapi_cmd.ptr->state() == global::status::FINISH) ||
        data_center_->planning_clear_task() || FinishProcess()) {
      LOG_INFO("clear task, planning clear task:{}",
               data_center_->planning_clear_task());
      broadcast_last_result_ = false;
      openapi_cmd.is_available = false;

      GetContext()->lane_seq.clear();
      GetContext()->lane_change_seq.clear();
      current_ref_generator_.reset(new RefeLineGenerator(&ctx_));
      target_ref_generator_.reset(new RefeLineGenerator(&ctx_));
      GetContext()->target_utm_ref = nullptr;
      GetContext()->current_utm_ref = nullptr;
      GetContext()->request_msg.ptr->Clear();
      GetContext()->routing_request = nullptr;
      GetContext()->light_turn = HornLightsCmd::NO_TURN;
      data_center_->set_have_task(false);
      data_center_->set_planning_clear_task(false);
    }
    UpdateHornLightCmd();
    SaveResult();
    BroadcastResult();
    GetContext()->request_msg.is_updated = false;
    if (alive_notify_count_++ > frequency) {
      alive_notify_count_ = 0;
      LOG_INFO("routing is running");
    }
    end_time = common::NowMilliSec();
    if (end_time < start_time) {
      LOG_ERROR("end_time < start time, error");
      continue;
    }
    delta_time = end_time - start_time;
    if (delta_time < period_time) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(period_time - delta_time));
    } else {
      std::this_thread::yield();
    }
  }
}

void NavigationComponent::Observe() {
  OBSERVE_FUNCTION(GetContext()->chassis_msg)
  OBSERVE_FUNCTION(GetContext()->request_msg)
  OBSERVE_FUNCTION(GetContext()->global_state_msg)
  OBSERVE_FUNCTION(GetContext()->localization_msg)
  OBSERVE_FUNCTION(GetContext()->planning_interface_msg)
  OBSERVE_FUNCTION(GetContext()->pose_base_link_in_utm_msg)

  const auto &utm_pose =
      GetContext()->pose_base_link_in_utm_msg.ptr->pose().position();
  GetContext()->ego_pose = {utm_pose.x(), utm_pose.y(), utm_pose.z()};
  LOG_INFO("utm pose ({}, {}), data center pose({}, {})", utm_pose.x(),
           utm_pose.y(), data_center_->vehicle_state_utm().X(),
           data_center_->vehicle_state_utm().Y());

  if (!have_set_is_sim_ && GetContext()->localization_msg.ptr->has_header() &&
      GetContext()->localization_msg.ptr->header().module_name() ==
          "sim_localization") {
    LOG_INFO("current environment is simulation");
    GetContext()->is_sim = true;
    data_center_->set_is_sim(true);
    have_set_is_sim_ = true;
  }
}

void NavigationComponent::Reset() {
  GetContext()->Reset();
  latest_result_->Clear();
  SaveResult();
}

void NavigationComponent::ProcessRoutingRequest() {
  if (latest_request_ != nullptr &&
      latest_request_->header().sequence_num() ==
          GetContext()->request_msg.ptr->header().sequence_num()) {
    LOG_INFO("same request sequence_num");
    GetContext()->request_msg.is_updated = false;
  }
  // process combination navigation
  // navigation_ptr_->ProcessIntegratedNavigation();
  // process key-point navigation
  // navigation_ptr_->ProcessKeyPointSeqRequest();
}

bool NavigationComponent::AddPathPoint() {}

void NavigationComponent::SaveResult() {
  auto context = GetContext();
  data_center_->GetNavigationSwapContext().navigation_result.Set(
      {context->current_utm_ref, context->target_utm_ref,
       context->is_swap_ref_line, context->navigator_request,
       context->preview_navigator_request, context->lane_change_end_point,
       context->dist_to_lane_change_end});
}

void NavigationComponent::BroadcastResult() {
  double broadcast_interval = config::NavigationConfig::Instance()
                                  ->navigation_config()
                                  .broadcast_result_interval;
  double now_t = common::NowSec();
  if (broadcast_last_result_ && now_t - last_send_t_ > broadcast_interval) {
    CHECK_NOTNULL(latest_result_);
    CHECK_NOTNULL(GetContext()->request_msg.ptr);
    routing_pub_->Write(latest_result_);
    routing_request_pub_->Write(GetContext()->request_msg.ptr);
    LOG_INFO("BroadcastRequest:{}",
             GetContext()->request_msg.ptr->DebugString());
    last_send_t_ = now_t;
  }
}

bool NavigationComponent::UpdateCurrentRef() {
  data_center_->set_last_vehicle_state_odometry(
      data_center_->vehicle_state_odometry());
  data_center_->set_last_vehicle_state_utm(data_center_->vehicle_state_utm());

  return true;
}

bool NavigationComponent::IsChangeStartPoint(
    const std::shared_ptr<RoutingRequest> &request,
    global::routing::RoutingRequest_LaneWaypoint &new_start) const {
  const auto &utm_pose = ctx_.pose_base_link_in_utm_msg.ptr->pose().position();
  auto &current_parking_space = ctx_.current_parking_space;
  if (current_parking_space &&
      current_parking_space->Type() ==
          global::hdmap::ParkingSpace_ParkingType_ONLANE &&
      current_parking_space->Points().size() == 4 &&
      !current_parking_space->OverlapLaneIds().empty() &&
      request->openapi_request_type() == RoutingRequest::FROM_PARK_CLUSTER) {
    auto lane_info_ptr = cyberverse::HDMap::Instance()->GetLaneById(
        current_parking_space->OverlapLaneIds().front());
    auto &right = current_parking_space->Points()[2];
    auto &left = current_parking_space->Points()[3];
    Vec3d changed_start_pt{(left.x + right.x) / 2.0, (left.y + right.y) / 2.0,
                           (left.z + right.z) / 2.0};
    double s, l;
    lane_info_ptr->GetProjection({changed_start_pt.x(), changed_start_pt.y()},
                                 &s, &l);
    auto *pose = new_start.mutable_pose();
    if (request->has_start()) {
      if (request->start().has_id()) {
        new_start.set_id(cyberverse::HDMap::Instance()->GetIdHashString(
            lane_info_ptr->Id()));
      }
      if (request->start().has_s()) {
        new_start.set_s(s);
      }
      pose->set_x(changed_start_pt.x());
      pose->set_y(changed_start_pt.y());
      pose->set_z(changed_start_pt.z());
      pose->set_heading(
          lane_info_ptr->Heading({changed_start_pt.x(), changed_start_pt.y()}));
    }
    return true;
  }
  if (!current_parking_space ||
      current_parking_space->Type() ==
          global::hdmap::ParkingSpace_ParkingType_ONLANE ||
      current_parking_space->Type() ==
          global::hdmap::ParkingSpace_ParkingType_UNKNOWN ||
      current_parking_space->Points().size() != 4) {
    LOG_INFO("Get Parking space failed form x = {:.2f}, y = {:.2f}",
             utm_pose.x(), utm_pose.y());
    return false;
  }
  if (!current_parking_space || !use_parking_out_) {
    LOG_INFO("parking out from hz, not parking out");
    return false;
  }
  auto &overlap_lane_ids = current_parking_space->OverlapLaneIds();
  if (overlap_lane_ids.empty()) {
    LOG_ERROR("No overlap_lane_ids matched parking space {}",
              current_parking_space->Id());
    return false;
  }
  auto lane_info_ptr =
      cyberverse::HDMap::Instance()->GetLaneById(overlap_lane_ids.front());
  if (lane_info_ptr == nullptr || lane_info_ptr->Points().empty()) {
    LOG_ERROR("No lane matched parking space {}", current_parking_space->Id());
    return false;
  }
  auto predecessor = GetOnlyPredecessor(lane_info_ptr);
  auto prepredecessor = GetOnlyPredecessor(predecessor);
  if (!prepredecessor) {
    LOG_ERROR("overlap lane have no prepredecessor");
    return false;
  }
  auto *pose = new_start.mutable_pose();
  if (request->has_start()) {
    if (request->start().has_id()) {
      new_start.set_id(request->start().id());
    }
    if (request->start().has_s()) {
      new_start.set_s(request->start().s());
    }
    pose->set_x(prepredecessor->Points().front().x());
    pose->set_y(prepredecessor->Points().front().y());
    if (request->start().has_pose()) {
      if (request->start().pose().has_z()) {
        pose->set_z(request->start().pose().z());
      }
      if (request->start().pose().has_heading()) {
        pose->set_heading(request->start().pose().heading());
      }
    }
  }
  return true;
}

bool NavigationComponent::IsChangeEndPoint(
    const std::shared_ptr<RoutingRequest> &request, Vec2d &new_end) const {
  if (!request->has_parking_id()) return false;
  auto hdmap = cyberverse::HDMap::Instance();
  auto parking_ptr =
      hdmap->GetParkingSpaceById(common::HashString(request->parking_id()));
  if (!parking_ptr ||
      parking_ptr->Type() != global::hdmap::ParkingSpace_ParkingType_ONLANE ||
      parking_ptr->Points().size() != 4) {
    return false;
  }
  auto &overlap_lane_ids = parking_ptr->OverlapLaneIds();
  if (overlap_lane_ids.empty()) {
    LOG_ERROR(
        "No overlap_lane_ids matched parking space {}",
        cyberverse::HDMap::Instance()->GetIdHashString(parking_ptr->Id()));
    return false;
  }
  auto lane_info_ptr =
      cyberverse::HDMap::Instance()->GetLaneById(overlap_lane_ids.front());
  if (lane_info_ptr == nullptr || lane_info_ptr->Points().empty()) {
    LOG_ERROR(
        "No lane matched parking space {}",
        cyberverse::HDMap::Instance()->GetIdHashString(parking_ptr->Id()));
    return false;
  }
  new_end.set_x(lane_info_ptr->Points().back().x());
  new_end.set_y(lane_info_ptr->Points().back().y());
  return true;
}

void NavigationComponent::RequestProcess(
    const std::shared_ptr<RoutingRequest> &request) {
  LOG_DEBUG("RequestProcess new:{}", request->DebugString());
  if (request->request_type() == RoutingRequest::KEY_POINTS) {
    KeypointRequestProcess(request);
    return;
  }
  latest_request_->Clear();
  latest_request_.reset(new RoutingRequest);
  latest_request_->mutable_header()->CopyFrom(request->header());
  if (request->has_parking_id()) {
    latest_request_->set_parking_id(request->parking_id());
  }
  global::routing::RoutingRequest_LaneWaypoint new_start;
  bool change_start = IsChangeStartPoint(request, new_start);
  if (change_start) {
    latest_request_->add_waypoint()->CopyFrom(new_start);
    GetContext()->park_out_start_pt = {new_start.pose().x(),
                                       new_start.pose().y()};
  } else {
    latest_request_->add_waypoint()->CopyFrom(request->start());
    GetContext()->park_out_start_pt = {0., 0.};
  }
  for (int i = 0; i < request->waypoint_size(); ++i) {
    latest_request_->add_waypoint()->CopyFrom(request->waypoint(i));
  }
  latest_request_->add_waypoint()->CopyFrom(request->end());
  if (request->has_request_type()) {
    latest_request_->set_request_type(request->request_type());
  }
  for (auto i = 0; i < latest_request_->waypoint_size(); ++i) {
    auto &each_pt = *latest_request_->mutable_waypoint(i);
    navigation_ptr_->CheckMapPointZLegality(each_pt);
  }
  GetContext()->routing_request = latest_request_;
  LOG_INFO("RequestProcess latest:{}", latest_request_->DebugString());
}

void NavigationComponent::KeypointRequestProcess(
    const std::shared_ptr<RoutingRequest> &request) {
  LOG_INFO("enter KeypointRequestProcess");
  LOG_INFO("RequestProcess latest:{}", request->DebugString());
  latest_request_->Clear();
  latest_request_->mutable_header()->CopyFrom(request->header());
  if (request->has_parking_id()) {
    latest_request_->set_parking_id(request->parking_id());
  }
  if (request->key_point_seq().empty() ||
      request->key_point_seq()[0].key_points().empty()) {
    LOG_ERROR("empty keypoint seq!");
    return;
  }
  // add start and start waypoint
  const auto &key_point_seq = request->key_point_seq()[0];
  global::routing::RoutingRequest_LaneWaypoint new_start;
  auto start_keypoint = key_point_seq.key_points()[0];
  latest_request_->mutable_start()->mutable_pose()->set_x(start_keypoint.x());
  latest_request_->mutable_start()->mutable_pose()->set_y(start_keypoint.y());
  latest_request_->mutable_start()->mutable_pose()->set_z(start_keypoint.z());
  bool change_start = IsChangeStartPoint(latest_request_, new_start);
  if (change_start) {
    latest_request_->add_waypoint()->CopyFrom(new_start);
    latest_request_->mutable_start()->CopyFrom(new_start);
    GetContext()->park_out_start_pt = {new_start.pose().x(),
                                       new_start.pose().y()};
  } else {
    latest_request_->add_waypoint()->CopyFrom(latest_request_->start());
    GetContext()->park_out_start_pt = {0., 0.};
  }
  // add waypoint after start
  auto size = key_point_seq.key_points_size();
  for (int i = 1; i + 1 < size; ++i) {
    auto new_pt = latest_request_->add_waypoint();
    new_pt->mutable_pose()->set_x(key_point_seq.key_points()[i].x());
    new_pt->mutable_pose()->set_y(key_point_seq.key_points()[i].y());
    new_pt->mutable_pose()->set_z(key_point_seq.key_points()[i].z());
  }
  // add end
  Vec3d end_keypoint{};
  if (request->has_end()) {
    end_keypoint = {request->end().pose().x(), request->end().pose().y(), 0.0};
  } else {
    end_keypoint = {key_point_seq.key_points()[size - 1].x(),
                    key_point_seq.key_points()[size - 1].y(), 0.0};
  }
  latest_request_->mutable_end()->mutable_pose()->set_x(end_keypoint.x());
  latest_request_->mutable_end()->mutable_pose()->set_y(end_keypoint.y());
  latest_request_->mutable_end()->mutable_pose()->set_z(end_keypoint.z());
  auto new_end_waypoint = latest_request_->add_waypoint();
  new_end_waypoint->mutable_pose()->set_x(end_keypoint.x());
  new_end_waypoint->mutable_pose()->set_y(end_keypoint.y());
  new_end_waypoint->mutable_pose()->set_z(end_keypoint.z());
  // add request type
  if (request->has_request_type()) {
    latest_request_->set_request_type(request->request_type());
  }
  // add z for all waypoints
  for (auto i = 0; i < latest_request_->waypoint_size(); ++i) {
    auto &each_pt = *latest_request_->mutable_waypoint(i);
    navigation_ptr_->CheckMapPointZLegality(each_pt);
  }
  GetContext()->routing_request = latest_request_;
}

void NavigationComponent::UpdateHornLightCmd() {
  auto horn_lights_cmd = horn_lights_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(horn_lights_cmd);
  horn_lights_cmd->Clear();
  LOG_INFO("light_turn:{}",
           HornLightsCmd::TurnLevel_Name(GetContext()->light_turn));
  data_center_->set_turn_light(GetContext()->light_turn);
  horn_lights_cmd->set_turn(data_center_->is_parking()
                                ? HornLightsCmd::NO_TURN
                                : GetContext()->light_turn);
  horn_light_cmd_pub_->Write(horn_lights_cmd);
}

bool NavigationComponent::HasTraffciLiaght(const uint64_t junction_id) {
  auto hdmap = cyberverse::HDMap::Instance();
  auto junction = hdmap->GetJunctionById(junction_id);
  if (!junction) return false;
  auto &lanes = junction->LaneIds();
  for (uint32_t i = 0; i < lanes.size(); ++i) {
    auto lane = hdmap->GetLaneById(lanes[i]);
    if (lane != nullptr && !lane->Signals().empty()) {
      return true;
    }
  }
  return false;
}

double GetParkZ(cyberverse::ParkingSpaceInfoConstPtr park) {
  auto &points = park->Points();
  return (points[0].z + points[1].z + points[2].z + points[3].z) / 4.0;
}

std::string NavigationComponent::SortParkingSpaceByZDiff(
    std::vector<cyberverse::ParkingSpaceInfoConstPtr> &parks,
    const double current_z, cyberverse::HDMap *hdmap) {
  auto &current_park_ptr = ctx_.current_parking_space;
  if (parks.empty()) {
    current_park_ptr = nullptr;
    return std::string();
  }
  if (parks.size() > 1) {
    std::sort(parks.begin(), parks.end(),
              [&](cyberverse::ParkingSpaceInfoConstPtr first,
                  cyberverse::ParkingSpaceInfoConstPtr second) {
                auto &first_park_points = first->Points();
                auto &second_park_points = second->Points();
                double first_park_z = GetParkZ(first);
                double second_park_z = GetParkZ(second);
                return std::fabs(first_park_z - current_z) <
                       std::fabs(second_park_z - current_z);
              });
  }
  double park_z = GetParkZ(parks[0]);
  LOG_INFO("park z:{},current z:{},diff_z:{}", park_z, current_z,
           std::fabs(park_z - current_z));
  if (ctx_.is_sim) {
    current_park_ptr = parks[0];
  } else {
    current_park_ptr =
        std::fabs(park_z - current_z) < kMaxZDiffThreshold ? parks[0] : nullptr;
  }

  if (current_park_ptr == nullptr) {
    return std::string();
  }

  auto &not_parking_out_ids = config::PlanningConfig::Instance()
                                  ->scene_special_config()
                                  .parking.not_parking_out_ids;

  bool not_parking_out =
      std::any_of(not_parking_out_ids.begin(), not_parking_out_ids.end(),
                  [current_park_ptr](const std::string &id) {
                    return current_park_ptr->Id() == common::HashString(id);
                  });
  if (not_parking_out) {
    use_parking_out_ = false;
    LOG_INFO("not parking out, id = {}",
             hdmap->GetIdHashString(current_park_ptr->Id()));
  }

  return hdmap->GetIdHashString(current_park_ptr->Id());
}

void GetVehicleCenter(Vec3d &center_utm, const Vec3d &baselink_utm) {
  auto &ego_car_config =
      common::config::CommonConfig::Instance()->ego_car_config();
  double dist_to_center =
      ego_car_config.length / 2.0 - ego_car_config.base_link_in_car_x;
  Vec3d center_rel{dist_to_center, 0.0, 0.0};
  common::ConvertToWorldCoordinate(center_rel, baselink_utm, center_utm);
}

void NavigationComponent::UpdateIsOnMapStatus() {
  auto hdmap = cyberverse::HDMap::Instance();
  double x = data_center_->vehicle_state_utm().X();
  double y = data_center_->vehicle_state_utm().Y();
  double z = data_center_->vehicle_state_utm().Z();
  std::vector<cyberverse::LaneInfoConstPtr> lanes;
  std::vector<cyberverse::JunctionInfoConstPtr> junctions;
  std::vector<cyberverse::ParkingSpaceInfoConstPtr> parks;
  Vec3d vehicle_center_utm;
  GetVehicleCenter(vehicle_center_utm,
                   {x, y, data_center_->vehicle_state_utm().Heading()});
  hdmap->GetLanes({x, y}, 5.0, &lanes);
  hdmap->Getjunctions({x, y}, 5.0, &junctions);
  hdmap->GetParkingSpaces({vehicle_center_utm.x(), vehicle_center_utm.y()}, 0.0,
                          &parks);
  bool is_on_map = !lanes.empty() && lanes[0]->IsOnLane({x, y});
  auto nearest_lane = lanes.empty() ? nullptr : lanes[0];
  double dist_to_zone = 999.0;
  std::string junction_id;
  bool has_traffic_light = false;
  bool is_on_route = GetContext()->is_on_route;
  uint32_t junction_type = 0;
  if (!junctions.empty()) {
    junction_id = hdmap->GetIdHashString(junctions[0]->Id());
    has_traffic_light = HasTraffciLiaght(junctions[0]->Id());
    junction_type = junctions[0]->Type();
  }
  data_center_->set_is_on_map(is_on_map);
  auto is_on_map_msg = is_on_map_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(is_on_map_msg);
  is_on_map_msg->Clear();
  std::string current_parking_space = SortParkingSpaceByZDiff(parks, z, hdmap);
  is_on_map_msg->set_current_park_id(current_parking_space);
  is_on_map_msg->set_is_on_map(is_on_map);
  is_on_map_msg->set_distance_to_zone(dist_to_zone);
  is_on_map_msg->set_junction_type(junction_type);
  is_on_map_msg->set_junction_id(junction_id);
  is_on_map_msg->set_has_traffic_light(has_traffic_light);
  is_on_map_msg->set_is_on_route(is_on_route);
  is_on_map_msg->set_timestamp_sec(cyber::Time::Now().ToSecond());
  is_on_map_msg->set_is_on_motorway(IsOnMotorway(x, y, nearest_lane));
  is_on_map_pub_->Write(is_on_map_msg);
}

bool NavigationComponent::FinishProcess() {
  double dis_to_dest{data_center_->GetNavigationSwapContext()
                         .dist_to_routing_destination.Get()};
  if (dis_to_dest > 20.) {
    LOG_INFO("Ego is too far from routing destination, dis_to_dest = {:.2f}",
             dis_to_dest);
    return false;
  }
  if (GetContext()->current_utm_ref == nullptr) return false;
  const auto &end_pt =
      latest_request_->waypoint(latest_request_->waypoint_size() - 1).pose();
  const auto &ego_pose = GetContext()->ego_pose;
  double distance_to_end{data_center_->master_info().distance_to_end()};
  double speed = std::abs(
      data_center_->environment().vehicle_state_proxy().LinearVelocity());
  LOG_INFO("distance_to_end:{}, linear_velocity:{}", distance_to_end, speed);
  auto &global_state = data_center_->global_state_proxy().global_state();
  double finish_distance_threshold =
      FLAGS_planning_arrive_to_destination_distance_threshold;
  const auto &config =
      config::NavigationConfig::Instance()->navigation_config();
  double frequency = static_cast<double>(config.routing_period_frequency);
  double reach_station_time = config.reach_station_time;

  bool reach_station = distance_to_end <= 5. * finish_distance_threshold &&
                       speed <= FLAGS_planning_adc_stop_velocity_threshold;
  if (reach_station) {
    ++reach_station_count_;
  } else {
    reach_station_count_ = 0;
  }
  if (static_cast<double>(reach_station_count_) / frequency >
      reach_station_time) {
    LOG_INFO("stop at destination, FinishProcess");
    data_center_->mutable_global_state_proxy()->SetReachStation(true);
    return true;
  }
  return false;
}

bool NavigationComponent::IsOnMotorway(
    double x, double y, const cyberverse::LaneInfoConstPtr &lane) {
  LOG_INFO("check IsOnMotorway");
  const auto &hdmap = cyberverse::HDMap::Instance();
  if (GetContext()->current_utm_ref != nullptr) {
    const auto &ref_line = GetContext()->current_utm_ref;
    ReferencePoint ref_pt{};
    ref_line->GetNearestRefPoint({x, y}, &ref_pt);
    LOG_INFO(
        "ego x, y: {:.3f}, {:.3f}, ref x, y: {:.3f}, {:.3f}, is city "
        "driving:{}, is "
        "biking:{}, is indoor:{}",
        x, y, ref_pt.x(), ref_pt.y(), ref_pt.lane_type_is_city_driving(),
        ref_pt.lane_type_is_biking(), ref_pt.lane_type_is_indoor_lane());
    return ref_pt.lane_type_is_city_driving() &&
           !ref_pt.lane_type_is_biking() && !ref_pt.lane_type_is_indoor_lane();
  } else if (lane != nullptr) {
    const auto &lane_multiple_type = lane->LaneMultipleType();
    for (int i = 0; i < lane_multiple_type.size(); ++i) {
      if ((lane_multiple_type[i] == global::hdmap::Lane::BIKING ||
           lane_multiple_type[i] == global::hdmap::Lane::INDOOR_LANE) &&
          lane->IsOnLane({x, y})) {
        LOG_INFO("lane id:{} is non-motorway",
                 hdmap->GetIdHashString(lane->Id()));
        return false;
      }
    }
    LOG_INFO("lane id:{} is motorway", hdmap->GetIdHashString(lane->Id()));
    return true;
  }
  LOG_INFO("find no lane and no ref-line, return non-motorway");
  return false;
}

}  // namespace planning
}  // namespace neodrive
