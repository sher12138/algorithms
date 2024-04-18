#include "proto_check_util.h"

#include "common/common_macros.h"
#include "common/data_center/data_center.h"
#include "neolix_log.h"

namespace neodrive {
namespace planning {

bool CheckProto(const Chassis &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, speed_mps);
  check_proto(proto, driving_mode);
  check_proto(proto, gear_location);
  check_proto(proto, throttle_percentage);
  check_proto(proto, brake_percentage);
  check_proto(proto, steering_percentage);
  check_proto(proto, error_code);
  return true;
}

bool CheckProto(const AebCmd &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, aeb_state);
  check_proto(proto, brake);
  return true;
}

bool CheckProto(const RoutingResult &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);

  if (proto.route_size() < 1) {
    LOG_ERROR("[RoutingResult error] received a message with no route.");
    return false;
  }

  for (const auto &tmp : proto.route()) {
    if (tmp.has_road_info()) {
      if (tmp.road_info().passage_region_size() < 1) {
        LOG_ERROR(
            "[RoutingResult error] received a road_info with no "
            "passage_region.");
        return false;
      }
    } else if (tmp.has_junction_info()) {
      check_proto(tmp.junction_info(), passage_region);
    } else {
      LOG_ERROR("received a message with no junction_info or road_info");
    }
  }
  check_proto(proto, error_code);
  check_proto(proto.error_code(), error_id);
  LOG_DEBUG("routing msg: {}", proto.DebugString());
  return true;
}

bool CheckProto(const ControlCommand &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

bool CheckProto(const PadNormalCommand &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, action);
  return true;
}

bool CheckProto(const Freespace &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

bool CheckProto(const CameraSegmentation &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

bool CheckProto(const DisCode &proto) {
  check_proto(proto, stamp);
  check_proto(proto, dis_type);
  return true;
}

bool CheckProto(const PatrolStatus &proto) {
  check_proto(proto, stamp);
  if (proto.err_flag() == neodrive::global::patrol::ErrFlag::FLAG_PLANNING) {
    return false;
  }
  return true;
}

bool CheckProto(const PlanningInterface &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, state);
  return true;
}

bool CheckProto(const LocalizationStatus &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, sensor_status);
  check_proto(proto, msf_status);
  return true;
}

bool CheckProto(const TwistStamped &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, twist);
  check_proto(proto.twist(), linear);
  check_proto(proto.twist().linear(), x);
  check_proto(proto.twist().linear(), y);
  check_proto(proto.twist().linear(), z);
  check_proto(proto.twist(), angular);
  check_proto(proto.twist().angular(), x);
  check_proto(proto.twist().angular(), y);
  check_proto(proto.twist().angular(), z);
  check_proto(proto.twist(), linear_acc);
  check_proto(proto.twist().linear_acc(), x);
  check_proto(proto.twist().linear_acc(), y);
  check_proto(proto.twist().linear_acc(), z);

  return true;
}

bool CheckProto(const PoseStamped &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, pose);
  check_proto(proto.pose(), position);
  check_proto(proto.pose().position(), x);
  check_proto(proto.pose().position(), y);
  check_proto(proto.pose().position(), z);
  check_proto(proto.pose(), orientation);
  check_proto(proto.pose().orientation(), qx);
  check_proto(proto.pose().orientation(), qy);
  check_proto(proto.pose().orientation(), qz);
  check_proto(proto.pose().orientation(), qw);
  return true;
}

bool CheckProto(const PerceptionObstacles &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

bool CheckProto(const PredictionObstacles &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

bool CheckProto(const TrafficLightDetection &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  LOG_DEBUG("traffic light msg: {}", proto.DebugString());
  return true;
}

bool CheckProto(const PerceptionLanes &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  return true;
}

}  // namespace planning
}  // namespace neodrive
