#pragma once

#include "common/macros.h"
#include "reference_line/reference_line.h"
#include "src/planning/common/data_center/frame.h"
#include "src/planning/common/data_center/master_info.h"
#include "src/planning/common/obstacle/decision_data.h"

namespace neodrive {
namespace planning {

class TaskInfo {
 public:
  TaskInfo(std::unique_ptr<Frame>&& curr_frame, const Frame* last_frame,
           const ReferenceLinePtr reference_line_raw,
           const ReferenceLinePtr reference_line);

  ~TaskInfo() = default;

  std::unique_ptr<Frame>& current_frame();
  const Frame* frame() const;
  const Frame* last_frame() const;

  const ReferenceLinePtr reference_line() const;
  const ReferenceLinePtr reference_line_raw() const;

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::shared_ptr<DecisionData>,
                                       decision_data);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(Boundary, adc_boundary);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(Boundary, adc_boundary_origin);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(TrajectoryPoint, adc_point);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(ReferencePoint, curr_referline_pt)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(SLPoint, curr_sl);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(size_t, referline_curr_index);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(ErrorCode, error_code);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(std::size_t, current_multi_level_mode);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(std::size_t, current_pass_by_mode);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, skip_curr_task_planning);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, limited_speed_from_behavior);

 private:
  std::unique_ptr<Frame> current_frame_;
  const Frame* last_frame_;
  ReferenceLinePtr reference_line_;
  ReferenceLinePtr reference_line_raw_;
  ErrorCode error_code_{ErrorCode::PLANNING_OK};
  std::size_t current_multi_level_mode_{0};
  double limited_speed_from_behavior_{3.0};
  std::size_t current_pass_by_mode_{0};
  bool skip_curr_task_planning_{false};
  std::shared_ptr<DecisionData> decision_data_{nullptr};

  // reference line info.
  TrajectoryPoint adc_point_;
  SLPoint curr_sl_;
  Boundary adc_boundary_;             // buffer: default_adc_boundary_buffer.
  Boundary adc_boundary_origin_;      // buffer: no buffer.
  ReferencePoint curr_referline_pt_;  // closest heading reference point.
  size_t referline_curr_index_;       // closest reference point index.
};
}  // namespace planning
}  // namespace neodrive
