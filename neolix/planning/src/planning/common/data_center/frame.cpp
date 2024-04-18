#include "frame.h"

namespace neodrive {
namespace planning {

Frame::Frame(uint32_t sequence_num) : sequence_num_(sequence_num) {
  planning_data_ = std::make_unique<PlanningData>();
}

void Frame::set_planning_data(std::unique_ptr<PlanningData> &planning_data) {
  if (planning_data != nullptr) {
    planning_data_ = std::move(planning_data);
  }
}

const PlanningData &Frame::planning_data() const {
  if (planning_data_ == nullptr) {
    LOG_ERROR("planning_data_ is nullptr, crash error");
  }
  return *(planning_data_.get());
}

PlanningData *Frame::mutable_planning_data() {
  if (planning_data_ == nullptr) {
    LOG_ERROR("planning_data_ is nullptr, crash error");
    return nullptr;
  } else {
    return planning_data_.get();
  }
}

std::unique_ptr<PlanningData> Frame::planning_data_ptr() {
  return std::move(planning_data_);
}

bool Frame::IsPlanningDataSafe() const {
  if (planning_data_ == nullptr) return false;
  return true;
}

}  // namespace planning
}  // namespace neodrive
