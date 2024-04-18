#include "speed_limit.h"

#include "common_config/config/common_config.h"
#include "neolix_log.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/util/util.h"

namespace neodrive {
namespace planning {
using neodrive::global::planning::SpeedLimit;

BehaviorSpeedLimit::BehaviorSpeedLimit() {
  speed_limits_.resize(SpeedLimit::SourceType_MAX + 1);
  for (size_t i = 0; i <= SpeedLimit::SourceType_MAX; ++i) {
    SpeedLimit::SourceType type = static_cast<SpeedLimit::SourceType>(i);
    speed_limits_[i].set_source_type(type);
  }
  speed_limits_enable_.resize(SpeedLimit::SourceType_MAX + 1, false);
}

void BehaviorSpeedLimit::Reset() {
  speed_limit_ = drive_strategy_max_speed_;
  speed_limit_type_ = "SPEED_LIMIT";
  for (size_t i = 0; i <= SpeedLimit::SourceType_MAX; ++i) {
    speed_limits_enable_[i] = false;
  }
}

void BehaviorSpeedLimit::SetSpeedLimit(
    const neodrive::global::planning::SpeedLimit& speed_limit) {
  if (speed_limit.source_type() < SpeedLimit::SourceType_MIN ||
      speed_limit.source_type() > SpeedLimit::SourceType_MAX ||
      speed_limit.upper_bounds_size() <= 0) {
    LOG_ERROR("source_type out of range or invalid size {}",
              SpeedLimit_SourceType_Name(speed_limit.source_type()));
    return;
  }

  /// for single v_bound
  if (speed_limit.upper_bounds_size() == 1 &&
      speed_limit.upper_bounds().at(0) > drive_strategy_max_speed_ + 0.1) {
    LOG_ERROR("speed limit exceeds limit bound, ignore it: {:.3f} KPH",
              speed_limit.upper_bounds().at(0) * 3.6);
    return;
  }
  if (speed_limit.upper_bounds_size() == 1 &&
      !speed_limits_enable_[speed_limit.source_type()]) {
    speed_limits_enable_[speed_limit.source_type()] = true;
    speed_limits_[speed_limit.source_type()] = speed_limit;
    ComputeSpeedLimit();
    return;
  }
  if (speed_limit.upper_bounds_size() == 1 &&
      speed_limits_enable_[speed_limit.source_type()] &&
      speed_limits_[speed_limit.source_type()].upper_bounds_size() == 1 &&
      speed_limits_[speed_limit.source_type()].upper_bounds().at(0) >
          speed_limit.upper_bounds().at(0)) {
    speed_limits_enable_[speed_limit.source_type()] = true;
    speed_limits_[speed_limit.source_type()] = speed_limit;
    ComputeSpeedLimit();
    return;
  }
  /// for sequence v_bound
  if (speed_limits_[speed_limit.source_type()].upper_bounds_size() > 1) {
    speed_limits_enable_[speed_limit.source_type()] = true;
    speed_limits_[speed_limit.source_type()] = speed_limit;
    ComputeSpeedLimit();
    return;
  }
}

void BehaviorSpeedLimit::ComputeSpeedLimit() {
  speed_limit_ = drive_strategy_max_speed_;
  for (size_t i = 0; i < speed_limits_.size(); ++i) {
    if (speed_limits_enable_[i]) {
      if (speed_limits_[i].upper_bounds_size() == 1) {
        if (speed_limits_[i].upper_bounds().at(0) < speed_limit_) {
          speed_limit_ = speed_limits_[i].upper_bounds().at(0);
          speed_limit_type_ =
              SpeedLimit::SourceType_Name(speed_limits_[i].source_type());
        }
      }
    }
  }
}

std::string BehaviorSpeedLimit::AggregateSpeedLimitStr() const {
  std::vector<neodrive::global::planning::SpeedLimit> single_soft_limits{};
  std::vector<neodrive::global::planning::SpeedLimit> single_hard_limits{};
  for (size_t i = 0; i <= SpeedLimit::SourceType_MAX; ++i) {
    if (speed_limits_enable_[i] && speed_limits_[i].upper_bounds_size() == 1) {
      if (speed_limits_[i].constraint_type() ==
          neodrive::global::planning::SpeedLimit::SOFT) {
        single_soft_limits.push_back(speed_limits_[i]);
      }
      if (speed_limits_[i].constraint_type() ==
          neodrive::global::planning::SpeedLimit::HARD) {
        single_hard_limits.push_back(speed_limits_[i]);
      }
    }
  }
  std::sort(single_soft_limits.begin(), single_soft_limits.end(),
            [](const auto& a, const auto& b) {
              return a.upper_bounds().at(0) < b.upper_bounds().at(0);
            });
  std::sort(single_hard_limits.begin(), single_hard_limits.end(),
            [](const auto& a, const auto& b) {
              return a.upper_bounds().at(0) < b.upper_bounds().at(0);
            });

  /// min speed limit
  std::string limit_str =
      "[SPEED_LIMIT: " + DoubleFormat(speed_limit_ * 3.6, 1) + "KPH]";

  /// speed_limit for single bound hard
  int hard_count = 0;
  std::string hard_str = "[HARD: ";
  for (size_t i = 0; i < single_hard_limits.size(); ++i) {
    if (hard_count >= 2) break;
    hard_str +=
        SpeedLimit::SourceType_Name(single_hard_limits[i].source_type()) + " " +
        DoubleFormat(single_hard_limits[i].upper_bounds().at(0) * 3.6, 1) +
        " " + DoubleFormat(single_hard_limits[i].acceleration(), 1) + " ";
    ++hard_count;
  }
  hard_str += "]";

  /// speed_limit for single bound soft
  int soft_count = 0;
  std::string soft_str = "[SOFT: ";
  for (size_t i = 0; i < single_soft_limits.size(); ++i) {
    if (soft_count >= 2) break;
    soft_str +=
        SpeedLimit::SourceType_Name(single_soft_limits[i].source_type()) + " " +
        DoubleFormat(single_soft_limits[i].upper_bounds().at(0) * 3.6, 1) +
        " " + DoubleFormat(single_soft_limits[i].acceleration(), 1) + " ";
    ++soft_count;
  }
  soft_str += "]";

  return limit_str + hard_str + soft_str;
}

std::string BehaviorSpeedLimit::GetSpeedLimitResStr() const {
  return std::string((speed_limit_type_ == "" ? "Null" : speed_limit_type_) +
                     ":" + DoubleFormat(speed_limit_ * 3.6, 1) + "KPH");
}

}  // namespace planning
}  // namespace neodrive
