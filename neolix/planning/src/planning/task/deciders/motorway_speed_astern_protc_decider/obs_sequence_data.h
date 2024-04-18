#pragma once

#include "common/math/util.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/util/stack.h"
namespace neodrive {
namespace planning {

namespace {
struct SingleObsInfo {
  SingleObsInfo() = default;
  SingleObsInfo(const Obstacle& obs) {
    x = obs.center().x();
    y = obs.center().y();
    s = obs.center_sl().s();
    l = obs.center_sl().l();
    width = obs.width();
    length = obs.length();
    speed = obs.speed();
    heading = obs.heading();
    speed_heading = obs.velocity_heading();
    is_static = obs.is_static();
    type = obs.type();
    timestamp = obs.get_time_stamp();
  }

  void Clear() {
    seq_index = -1;
    x = 0.0;
    y = 0.0;
    s = 0.0;
    l = 0.0;
    width = 0.0;
    length = 0.0;
    speed = 0.0;
    heading = 0.0;
    speed_heading = 0.0;
    is_static = false;
    type = Obstacle::ObstacleType::UNKNOWN;
    timestamp = 0.0;
  }

  int seq_index{-1};
  double x{};
  double y{};
  double s{};
  double l{};
  double width{};
  double length{};
  double speed{};
  double heading{};
  double speed_heading{};
  bool is_static{};
  Obstacle::ObstacleType type{};
  double timestamp{};
};

struct ObsStatisticInfo {
  // origin
  // 1 two
  // 1.1 speed
  double obs_history_mean_speed{};
  double obs_history_max_speed{};
  double obs_history_min_speed{};
  double obs_history_max_speed_heading{};
  double obs_history_min_speed_heading{};

  double obs_history_sliding_window_max_speed{};
  double obs_history_sliding_window_min_speed{};

  // 1.2 heading
  double obs_history_max_heading{};
  double obs_history_min_heading{};
  double obs_history_last_heading{};

  double obs_history_cur_sum_heading{};
  int obs_history_cur_sum_count{};

  double obs_history_cur_sum_heading_2{};
  int obs_history_cur_sum_count_2{};
  // double obs_history_heading_change_accumulated{};
  // double obs_continues_heading_accelerate_max_value{};
  int last_change_max_value_to_now_accululate_count{};

  // 2 one
  // 2.1 shape
  double obs_history_max_length{};
  double obs_history_min_length{};
  double obs_history_max_width{};
  double obs_history_min_width{};

  // 3 position

  double obs_history_max_euclidean_dis{};

  // filter
  // 1. two
  // 1.1 speed

  // 2 one
  // 2.1 shape

  // util
  double max_gap_time{};

  void Clear() {
    obs_history_max_speed = 0.0;

    obs_history_last_heading = 0.0;
    // to valid interval extreme
    obs_history_max_heading = 0.0;
    obs_history_min_heading = 0.0;

    obs_history_cur_sum_heading = 0.0;
    obs_history_cur_sum_count = 0;

    obs_history_cur_sum_heading_2 = 0.0;
    obs_history_cur_sum_count_2 = 0;
    // obs_history_heading_change_accumulated = 0.0;
    // obs_continues_heading_accelerate_max_value = 0.0;
    obs_history_max_euclidean_dis = 0.0;
  }

  void Print() {
    // LOG_INFO("obs_history_max_speed: {:.3f}", obs_history_max_speed);

    // LOG_INFO("obs_history_cur_sum_heading: {:.3f}",
    //          obs_history_cur_sum_heading);
    // LOG_INFO("obs_history_cur_sum_count: {}", obs_history_cur_sum_count);

    // LOG_INFO("obs_history_cur_sum_heading_2: {:.3f}",
    //          obs_history_cur_sum_heading_2);
    // LOG_INFO("obs_history_cur_sum_count_2: {}", obs_history_cur_sum_count_2);

    // LOG_INFO("obs_history_cur_sum_count: {:.3f}", obs_history_cur_sum_count);
    // LOG_INFO("obs_history_last_heading: {:.3f}", obs_history_last_heading);
    LOG_INFO("obs_history_max_euclidean_dis: {:.3f}",
             obs_history_max_euclidean_dis);

    LOG_INFO("obs_history_max_heading: {:.3f}", obs_history_max_heading);
    LOG_INFO("obs_history_min_heading: {:.3f}", obs_history_min_heading);
  }
};

struct SingleFrameData {
  SingleFrameData(const Obstacle& obs, const int count) {
    accunt_number = count;
    timestamp = obs.get_time_stamp();
    single_obs_info = SingleObsInfo(obs);
  }

  int accunt_number{0};
  double timestamp{0.0};
  SingleObsInfo single_obs_info{};

  void Clear() {
    accunt_number = 0;
    timestamp = 0.0;
  }

  void Print() {
    LOG_INFO("accunt_number: {}", accunt_number);
    LOG_INFO("timestamp: {:.3f}", timestamp);
    LOG_INFO("single_obs_info: {:.3f},{:.3f},{:.3f}", single_obs_info.x,
             single_obs_info.y, single_obs_info.s);
  }
};

// 单障碍物的所有信息
struct SingleObsTimeSequenceData {
  int obs_id{-1};
  double max_gap_time{0.0};
  double first_record_time{0.0};
  int accumulate_frame_number{0};  // accumulate frame
  int max_store_frame_number{0};   // max store frame

  neodrive::planning::util::ExtremumStack<double>
      obs_history_sliding_window_max_speed_stack{};
  neodrive::planning::util::ExtremumStack<double>
      obs_history_sliding_window_heading_extremum_stack{};
  ObsStatisticInfo obs_statistic_info{};
  std::deque<SingleFrameData> frame_data_vec{};
  void Clear() {
    obs_id = -1;
    frame_data_vec.clear();
    max_gap_time = 0.0;
    accumulate_frame_number = 0;
    obs_statistic_info.Clear();
    max_store_frame_number = 0;
    obs_history_sliding_window_max_speed_stack.Clear();
    obs_history_sliding_window_heading_extremum_stack.Clear();
  }

  void Print() {
    LOG_INFO("obs_id: {}", obs_id);
    // LOG_INFO("max_gap_time: {:.3f}", max_gap_time);
    LOG_INFO("accumulate_frame_number: {}", accumulate_frame_number);
    // LOG_INFO("max_store_frame_number: {}", max_store_frame_number);
    obs_statistic_info.Print();
    LOG_INFO("speed ext:");
    obs_history_sliding_window_max_speed_stack.PrintExt();
    LOG_INFO("heading ext:");
    obs_history_sliding_window_heading_extremum_stack.PrintExt();
  }
};
}  // namespace

class TimeSequenceData {
 public:
  static TimeSequenceData& Instance() {
    static TimeSequenceData instance;
    return instance;
  }

  void Clear() { obs_time_sequence_data_vec_.clear(); }
  int FindById(const int obs_id);
  int FindByIndex(const int obs_index);
  bool UpdateObsData(const Obstacle& obs);
  void UpdateObsQueueCount();
  SingleObsTimeSequenceData& FindObsSeqData(const int obs_id);
  SingleObsTimeSequenceData& FindObsSeqDataByIndex(const int index);
  void RemoveUnavailableData();
  void UpdateCurrentTimeStamp(const double time_stamp);
  void UpdateObsIdMapUpdateSign();
  void DestoryObsStatisAccelerateHeadingConitnueChangeMaxValue(
      const int obs_id);

  bool IsTempStatic(const int obs_id);

  bool IsExtremeHeadingChange(const int obs_id, const double value);

  // tools
  double euclidean_dis(const double x1, const double y1, const double x2,
                       const double y2) const;
  bool IsObsStatic(const int obs_id);
  // bool IsObsContinueHeadingChangeOverValue(const int obs_id,
  //                                          const double value);
  bool IsObsExistSinglePosChange(const int obs_id, const double value);

  void AddObsToGarbage(const int obs_id);
  void UpdateGarbage(const std::vector<int>& obs_id_vec);

  void PrintAllData();

  const SingleObsTimeSequenceData& operator[](const int index) const {
    return obs_time_sequence_data_vec_[index];
  }

  SingleObsTimeSequenceData* begin() { return &obs_time_sequence_data_vec_[0]; }
  SingleObsTimeSequenceData* end() {
    return &obs_time_sequence_data_vec_[obs_time_sequence_data_vec_.size()];
  }

 private:
  std::vector<std::pair<int, bool>>
      obs_index_map_relative_{};                        // obs_id:false，
  std::unordered_map<int, int> obs_id_map_relative_{};  //  obs_id:index
  std::vector<SingleObsTimeSequenceData> obs_time_sequence_data_vec_{};

  std::unordered_map<int, SingleObsInfo> obs_id_map_garbage_{};  // obs_id:index

 private:
  TimeSequenceData();
  TimeSequenceData(const TimeSequenceData&) = delete;
  TimeSequenceData& operator=(const TimeSequenceData&) = delete;
  ~TimeSequenceData() = default;

  void Init();
  void RemoveObsInfoByObsId(const int obs_id);
  bool AddObsInfo(const Obstacle& obs);
  int FindAvailableSpaceIndex(const Obstacle& obs);
  void CreateNewPositionForObs(const Obstacle& obs, const int index);
  void AppendInTailForObs(const Obstacle& obs);
  void CreateDataForNewObs(const Obstacle& obs);
  void AppendDataForExistObs(const Obstacle& obs);
  void UpdateObsStatisticInfo(const int index);
  void UpdateObsHistoryMaxEuclideanDis(
      SingleObsTimeSequenceData& obs_time_sequence_data,
      const SingleObsInfo& obs_last_info);
  void UpdateObsHistoryHeadingInfo(ObsStatisticInfo& obs_sta_info,
                                   const SingleObsInfo& obs_last_info);

  double curent_time_stamp_{0.0};  // invalid
};

}  // namespace planning
}  // namespace neodrive
