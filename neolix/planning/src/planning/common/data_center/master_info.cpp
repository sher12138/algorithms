#include "master_info.h"

#include "data_center.h"

namespace neodrive {
namespace planning {
using namespace neodrive::global::status;

void MasterInfo::FrameReset() {
  // bias_driving_val_ = 0.0;
  need_bias_driving_ = false;
  is_ask_for_takeover_ = false;
}

bool MasterInfo::IsChangeLaneSignalOpen() const {
  return change_lane_signal_open_time_ > 0;
}

void MasterInfo::OpenChangeLaneSignal(
    MasterInfo::LaneChangeType lane_change_type) {
  change_lane_signal_open_time_ =
      common::util::TimeLogger::GetCurrentTimestamp();
  lane_change_type_ = lane_change_type;
}

void MasterInfo::CloseChangeLaneSignal() {
  change_lane_signal_open_time_ = -1;
  lane_change_type_ = MasterInfo::LaneChangeType::NONE;
}

ADCSignals::SignalType MasterInfo::ChangeLaneTurnType() const {
  if (lane_change_type_ == LaneChangeType::LEFT_FORWARD ||
      lane_change_type_ == LaneChangeType::LEFT_BACKWARD) {
    return ADCSignals::LEFT_TURN;
  } else if (lane_change_type_ == LaneChangeType::RIGHT_FORWARD ||
             lane_change_type_ == LaneChangeType::RIGHT_BACKWARD) {
    return ADCSignals::RIGHT_TURN;
  }
  return ADCSignals::LOW_BEAM_LIGHT;
}

ADCSignals::SignalType MasterInfo::LaneTurnLightType() const {
  if (vehicle_point_.is_left_signal()) {
    return ADCSignals::LEFT_TURN;
  } else if (vehicle_point_.is_right_signal()) {
    return ADCSignals::RIGHT_TURN;
  }
  return ADCSignals::LOW_BEAM_LIGHT;
}

}  // namespace planning
}  // namespace neodrive
