#include "obs_sequence_data.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kFilterGapTime = 0.2;
constexpr double kMaxHandleObsNUmber = 20;
// constexpr double kMaxStoreObsObserveSeq = 20;  //
constexpr double kMaxObservationNumber = 50;
constexpr double kHeadingChangeInInternal = 1 / 3 * M_PI;
constexpr double kSlidingWindowMaxSize = 50;
constexpr double kSinglePosMov = 1.0;
}  // namespace

TimeSequenceData::TimeSequenceData() { Init(); }

void TimeSequenceData::Init() {
  obs_time_sequence_data_vec_.clear();
  obs_index_map_relative_.clear();
  obs_time_sequence_data_vec_.resize(kMaxHandleObsNUmber);
  obs_index_map_relative_.resize(kMaxHandleObsNUmber);
  for (int i = 0; i < obs_index_map_relative_.size(); ++i) {
    obs_index_map_relative_[i] = {-1, false};
  }
  LOG_INFO("TimeSequenceData init! MAX HANDLE OBS NUMBER: {}",
           kMaxHandleObsNUmber);
}

int TimeSequenceData::FindById(const int obs_id) {
  if (obs_id_map_relative_.find(obs_id) == obs_id_map_relative_.end()) {
    return -1;
  }
  return obs_id_map_relative_[obs_id];
}
int TimeSequenceData::FindByIndex(const int obs_index) {
  return obs_index_map_relative_[obs_index].first;
}
bool TimeSequenceData::UpdateObsData(const Obstacle& obs) {
  if (obs_id_map_garbage_.find(obs.id()) != obs_id_map_garbage_.end()) {
    if (obs_id_map_garbage_[obs.id()].speed >= 0.1 ||
        euclidean_dis(obs_id_map_garbage_[obs.id()].x,
                      obs_id_map_garbage_[obs.id()].y, obs.center().x(),
                      obs.center().y()) >= 0.3) {
      LOG_INFO("obs {} in garbage and obs change, can't update", obs.id());
      return false;
    }
  }
  if (!AddObsInfo(obs)) {
    LOG_INFO("Can't update obs {} info in time sequence data", obs.id());
  }

  // PrintAllData();

  return false;
}

void TimeSequenceData::UpdateObsQueueCount() {
  for (auto& single_obs_info : obs_time_sequence_data_vec_) {
    if (single_obs_info.accumulate_frame_number >= kMaxObservationNumber) {
      single_obs_info.accumulate_frame_number = kMaxObservationNumber;
    }
    double over_number = single_obs_info.accumulate_frame_number -
                         single_obs_info.max_store_frame_number;

    for (auto& fdv : single_obs_info.frame_data_vec) {
      fdv.accunt_number -= over_number;
    }
  }
}

SingleObsTimeSequenceData& TimeSequenceData::FindObsSeqData(const int obs_id) {
  // TODO: insert return statement here
  return obs_time_sequence_data_vec_[FindById(obs_id)];
}

SingleObsTimeSequenceData&
neodrive::planning::TimeSequenceData::FindObsSeqDataByIndex(const int index) {
  // TODO: insert return statement here
  return obs_time_sequence_data_vec_[index];
}

void TimeSequenceData::RemoveUnavailableData() {
  std::vector<int> remove_obs_id_vec;
  for (const auto id_index : obs_id_map_relative_) {
    if (id_index.first == -1) continue;
    if (IsObsStatic(id_index.first) &&
        !IsExtremeHeadingChange(id_index.first, 1 / 18 * M_PI) &&
        obs_time_sequence_data_vec_[id_index.second].accumulate_frame_number >=
            60) {
      LOG_INFO("Remove Static Data for obs id: {}", id_index.first);
      AddObsToGarbage(id_index.first);
      remove_obs_id_vec.push_back(id_index.first);
    }
  }
  //
  for (int index = 0; index < obs_index_map_relative_.size(); ++index) {
    // null
    if (obs_index_map_relative_[index].first != -1 &&
        !obs_index_map_relative_[index].second) {
      LOG_INFO("Remove Unavailable Data for obs id: {}, occupy index:{}",
               obs_time_sequence_data_vec_[index].obs_id, index);
      remove_obs_id_vec.push_back(obs_time_sequence_data_vec_[index].obs_id);
    }
  }

  for (auto& obs_id : remove_obs_id_vec) {
    RemoveObsInfoByObsId(obs_id);
  }
}

void TimeSequenceData::UpdateCurrentTimeStamp(const double time_stamp) {
  curent_time_stamp_ = time_stamp;
}

void TimeSequenceData::UpdateObsIdMapUpdateSign() {
  for (auto& obs_id : obs_index_map_relative_) {
    obs_id.second = false;
  }
}

void TimeSequenceData::DestoryObsStatisAccelerateHeadingConitnueChangeMaxValue(
    const int obs_id) {
  const int index = FindById(obs_id);
  auto& obs_sta_info = obs_time_sequence_data_vec_[index].obs_statistic_info;
  obs_sta_info.obs_history_cur_sum_heading = 0.0;
  obs_sta_info.last_change_max_value_to_now_accululate_count = 0;
}

bool TimeSequenceData::IsTempStatic(const int obs_id) {
  const int index = FindById(obs_id);
  return obs_time_sequence_data_vec_[index]
             .obs_history_sliding_window_max_speed_stack.Max() < 0.1;
}

bool TimeSequenceData::IsExtremeHeadingChange(const int obs_id,
                                              const double value) {
  const int index = FindById(obs_id);
  double heading_change_internal =
      (obs_time_sequence_data_vec_[index]
           .obs_history_sliding_window_heading_extremum_stack.Max() -
       obs_time_sequence_data_vec_[index]
           .obs_history_sliding_window_heading_extremum_stack.Min());
  double heading_change = obs_time_sequence_data_vec_[index]
                              .obs_statistic_info.obs_history_max_heading -
                          obs_time_sequence_data_vec_[index]
                              .obs_statistic_info.obs_history_min_heading;
  LOG_INFO("obs_id:{}, heading_change:{:.3f}, heading_change_inernal:{:.3f}",
           obs_id, heading_change, heading_change_internal);
  return heading_change > value;
}

bool TimeSequenceData::IsObsStatic(const int obs_id) {
  const int index = FindById(obs_id);
  LOG_INFO("obs {} history max speed = {:.3f}",
           obs_time_sequence_data_vec_[index]
               .obs_statistic_info.obs_history_max_speed);
  return (obs_time_sequence_data_vec_[index]
              .obs_statistic_info.obs_history_max_speed <= 0.5);
}

bool TimeSequenceData::IsObsExistSinglePosChange(const int obs_id,
                                                 const double value) {
  const int index = FindById(obs_id);
  return obs_time_sequence_data_vec_[index]
             .obs_statistic_info.obs_history_max_euclidean_dis >= value;
}

void TimeSequenceData::AddObsToGarbage(const int obs_id) {
  const int index = FindById(obs_id);
  obs_id_map_garbage_[obs_id] =
      obs_time_sequence_data_vec_[index].frame_data_vec.back().single_obs_info;
}

void TimeSequenceData::UpdateGarbage(const std::vector<int>& obs_id_vec) {
  for (auto& garbage : obs_id_map_garbage_) {
    if (std::find(obs_id_vec.begin(), obs_id_vec.end(), garbage.first) ==
        obs_id_vec.end()) {
      obs_id_map_garbage_.erase(garbage.first);
    }
  }
}

void TimeSequenceData::PrintAllData() {
  // for (const auto& obs_id : obs_index_map_relative_) {
  //   LOG_INFO("obs_id: {}, index: {}", obs_id.first, obs_id.second);
  // }
  // for (const auto& obs_id : obs_id_map_relative_) {
  //   LOG_INFO("obs_id: {}, index: {}", obs_id.first, obs_id.second);
  // }

  for (int index = 0; index < obs_index_map_relative_.size(); ++index) {
    if (obs_index_map_relative_[index].first != -1) {
      LOG_INFO("obs_id: {}, index: {}", obs_index_map_relative_[index].first,
               index);
      auto& otsdv = obs_time_sequence_data_vec_[index];
      otsdv.Print();
    }
  }
}  // namespace planning

void TimeSequenceData::RemoveObsInfoByObsId(const int obs_id) {
  const int index = FindById(obs_id);
  obs_time_sequence_data_vec_[index].Clear();
  obs_index_map_relative_[index] = {-1, false};
  obs_id_map_relative_.erase(obs_id);
}

bool TimeSequenceData::AddObsInfo(const Obstacle& obs) {
  if (obs.is_virtual()) {
    return false;
  }

  if (obs_id_map_relative_.find(obs.id()) == obs_id_map_relative_.end()) {
    const int index = FindAvailableSpaceIndex(obs);
    if (index == -1) {
      LOG_INFO("Can't find available space for obs {}", obs.id());
      return false;
    }
    CreateNewPositionForObs(obs, index);  // no exist obs ,
  }

  AppendInTailForObs(obs);  // exist obs
  return true;
}

int TimeSequenceData::FindAvailableSpaceIndex(const Obstacle& obs) {
  for (int index = 0; index < obs_index_map_relative_.size(); ++index) {
    if (obs_index_map_relative_[index].first == -1) {
      return index;
    }
  }
  return -1;
}

void TimeSequenceData::CreateNewPositionForObs(const Obstacle& obs,
                                               const int index) {
  obs_index_map_relative_[index] = {obs.id(), true};
  obs_id_map_relative_[obs.id()] = index;
  obs_time_sequence_data_vec_[index].Clear();
  LOG_INFO("CreateNewPositionForObs, obs_id: {}, index: {}", obs.id(), index);
  //
}

void TimeSequenceData::AppendInTailForObs(const Obstacle& obs) {
  const int index = FindById(obs.id());
  auto& need_add_obs_seq_data = obs_time_sequence_data_vec_[index];
  if (need_add_obs_seq_data.accumulate_frame_number == 0) {
    CreateDataForNewObs(obs);
  } else {
    AppendDataForExistObs(obs);
  }
}

void TimeSequenceData::CreateDataForNewObs(const Obstacle& obs) {
  const int index = FindById(obs.id());
  auto& need_add_obs_seq_data = obs_time_sequence_data_vec_[index];
  need_add_obs_seq_data.Clear();
  need_add_obs_seq_data.obs_id = obs.id();
  need_add_obs_seq_data.max_gap_time = 0.0;
  need_add_obs_seq_data.first_record_time = obs.get_time_stamp();
  need_add_obs_seq_data.accumulate_frame_number++;
  need_add_obs_seq_data.max_store_frame_number = kMaxObservationNumber;
  need_add_obs_seq_data.obs_history_sliding_window_max_speed_stack.Resize(
      kSlidingWindowMaxSize);
  need_add_obs_seq_data.obs_history_sliding_window_heading_extremum_stack
      .Resize(kSlidingWindowMaxSize);
  SingleFrameData frame_data{obs,
                             need_add_obs_seq_data.accumulate_frame_number};
  need_add_obs_seq_data.frame_data_vec.push_back(std::move(frame_data));
  LOG_INFO("CreateDataForNewObs, obs_id: {}, index: {}", obs.id(), index);
  LOG_INFO("data, {},{:.3f},{:.3f}",
           need_add_obs_seq_data.frame_data_vec.back().accunt_number,
           need_add_obs_seq_data.frame_data_vec.back().single_obs_info.x,
           need_add_obs_seq_data.frame_data_vec.back().single_obs_info.y);
  UpdateObsStatisticInfo(index);
}

void TimeSequenceData::AppendDataForExistObs(const Obstacle& obs) {
  const int index = FindById(obs.id());
  obs_index_map_relative_[index].second = true;
  auto& need_add_obs_seq_data = obs_time_sequence_data_vec_[index];
  const double gap_time = curent_time_stamp_ -
                          need_add_obs_seq_data.frame_data_vec.back().timestamp;
  need_add_obs_seq_data.max_gap_time =
      std::max(need_add_obs_seq_data.max_gap_time, std::fabs(gap_time));
  SingleFrameData frame_data{obs,
                             need_add_obs_seq_data.accumulate_frame_number};
  need_add_obs_seq_data.accumulate_frame_number++;
  if (need_add_obs_seq_data.frame_data_vec.size() ==
      need_add_obs_seq_data.max_store_frame_number) {
    need_add_obs_seq_data.frame_data_vec.pop_front();
  }

  LOG_INFO("append obs data, x:{:.3f},y:{:.3f}", obs.center().x(),
           obs.center().y());
  need_add_obs_seq_data.frame_data_vec.push_back(std::move(frame_data));

  LOG_INFO(
      "AppendDataForExistObs, obs_id: {}, index: {},accumulate frame number:{}",
      obs.id(), index, need_add_obs_seq_data.accumulate_frame_number);
  LOG_INFO("data, accunt_number:{},tail info x:{:.3f}, tail info y{:.3f}",
           need_add_obs_seq_data.frame_data_vec.back().accunt_number,
           need_add_obs_seq_data.frame_data_vec.back().single_obs_info.x,
           need_add_obs_seq_data.frame_data_vec.back().single_obs_info.y);
  UpdateObsStatisticInfo(index);
}

void TimeSequenceData::UpdateObsStatisticInfo(const int index) {
  auto& obs_sta_info = obs_time_sequence_data_vec_[index].obs_statistic_info;
  const auto& obs_last_info =
      obs_time_sequence_data_vec_[index].frame_data_vec.back().single_obs_info;
  const int obs_f_size =
      obs_time_sequence_data_vec_[index].accumulate_frame_number;
  if (obs_f_size == 0) {
    LOG_ERROR("obs_f_size == 0,error");
    return;
  }

  // Stack
  obs_time_sequence_data_vec_[index]
      .obs_history_sliding_window_max_speed_stack.Push(obs_last_info.speed);
  obs_time_sequence_data_vec_[index]
      .obs_history_sliding_window_heading_extremum_stack.Push(
          obs_last_info.heading);

  if (obs_f_size == 1) {
    obs_sta_info.obs_history_max_speed = obs_last_info.speed;

    obs_sta_info.obs_history_max_euclidean_dis = 0;

    obs_sta_info.obs_history_max_heading = obs_last_info.heading;
    obs_sta_info.obs_history_min_heading = obs_last_info.heading;
  } else {
    obs_sta_info.obs_history_max_speed =
        std::max(obs_sta_info.obs_history_max_speed, obs_last_info.speed);

    UpdateObsHistoryMaxEuclideanDis(obs_time_sequence_data_vec_[index],
                                    obs_last_info);
    UpdateObsHistoryHeadingInfo(obs_sta_info, obs_last_info);
  }
}

void TimeSequenceData::UpdateObsHistoryMaxEuclideanDis(
    SingleObsTimeSequenceData& obs_time_sequence_data,
    const SingleObsInfo& obs_last_info) {
  auto& frame_data_vec = obs_time_sequence_data.frame_data_vec;
  for (int i = frame_data_vec.size() - 1; i >= 0; --i) {
    const double dis = euclidean_dis(frame_data_vec[i].single_obs_info.x,
                                     frame_data_vec[i].single_obs_info.y,
                                     obs_last_info.x, obs_last_info.y);
    obs_time_sequence_data.obs_statistic_info
        .obs_history_max_euclidean_dis = std::max(
        obs_time_sequence_data.obs_statistic_info.obs_history_max_euclidean_dis,
        dis);
  }

  LOG_INFO(
      "obs id:{} history max euclidean dis:{:.3f}",
      obs_time_sequence_data.obs_id,
      obs_time_sequence_data.obs_statistic_info.obs_history_max_euclidean_dis);
}

double TimeSequenceData::euclidean_dis(const double x1, const double y1,
                                       const double x2, const double y2) const {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void TimeSequenceData::UpdateObsHistoryHeadingInfo(
    ObsStatisticInfo& obs_sta_info, const SingleObsInfo& obs_last_info) {
  LOG_INFO(
      "obs_sta_info.obs_history_last_heading:{:.3f}, "
      "obs_last_info.heading:{:.3f}",
      obs_sta_info.obs_history_last_heading, obs_last_info.heading);

  obs_sta_info.obs_history_max_heading =
      std::max(obs_sta_info.obs_history_max_heading, obs_last_info.heading);
  obs_sta_info.obs_history_min_heading =
      std::min(obs_sta_info.obs_history_min_heading, obs_last_info.heading);
}

}  // namespace planning
}  // namespace neodrive
