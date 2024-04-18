#include "task_info.h"

#include "common_config/config/common_config.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

TaskInfo::TaskInfo(std::unique_ptr<Frame> &&curr_frame, const Frame *last_frame,
                   const ReferenceLinePtr reference_line_raw,
                   const ReferenceLinePtr reference_line)
    : current_frame_(std::move(curr_frame)),
      last_frame_(last_frame),
      reference_line_raw_(reference_line_raw),
      reference_line_(reference_line) {
  if (last_frame_ != nullptr) {
    *(current_frame_->mutable_traffic_law_context()) =
        last_frame_->traffic_law_context();
  }
  limited_speed_from_behavior_ =
      DataCenter::Instance()->drive_strategy_max_speed();
}

std::unique_ptr<Frame> &TaskInfo::current_frame() { return current_frame_; }

const Frame *TaskInfo::frame() const { return current_frame_.get(); }

const Frame *TaskInfo::last_frame() const { return last_frame_; }

const ReferenceLinePtr TaskInfo::reference_line() const {
  return reference_line_;
}

const ReferenceLinePtr TaskInfo::reference_line_raw() const {
  return reference_line_raw_;
}

}  // namespace planning
}  // namespace neodrive
