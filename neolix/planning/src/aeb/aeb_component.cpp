#include "aeb_component.h"

#include "config/aeb_config.h"
#include "neolix_common/reader_manager/reader_manager.h"
#include "neolix_common/util/performance_test.h"

namespace neodrive {
namespace aeb {
namespace {
template <typename T>
void CheckMessageTimeout(T &unit, const double threshold, const double now_t) {
  unit.is_available =
      (!unit.is_available && now_t - unit.latest_recv_time_sec > threshold)
          ? false
          : unit.is_available;
}
}  // namespace

using neodrive::cyber::Node;
PERFORMANCE_TEST_DEFINE(aeb_component);
AebComponent::~AebComponent() { proc_thread_->detach(); }

bool AebComponent::Init() {
  auto reader_manager = common::ReaderManager::Instance();
  if (!reader_manager->Init(node_)) {
    LOG_ERROR("ReaderManager init failed!");
    return false;
  }
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  auto &topic_config = common_config->topics_config().topics;
  aeb_ptr_ = std::make_shared<Aeb>();
  collision_ptr_ = std::make_shared<CollisionCheck>();
  aeb_cmd_pub_ = node_->CreateWriter<AebCmd>(topic_config.at("aeb_cmd").topic);
  event_reporter_pub_ = node_->CreateWriter<EventOfInterest>(
      topic_config.at("event_of_interest").topic);
  ivi_event_report_pub_ = node_->CreateWriter<EventReport>(
      topic_config.at("aeb_event_report").topic);
  AebContext::Instance()->chassis_msg.reader =
      reader_manager->CreateReader<Chassis>(
          topic_config.at("planning_chassis").topic);
  AebContext::Instance()->forbid_aeb_msg.reader =
      reader_manager->CreateReader<ForbidAeb>(
          topic_config.at("forbid_aeb").topic);
  AebContext::Instance()->pnc_forbid_aeb_msg.reader =
      reader_manager->CreateReader<ForbidAeb>(
          topic_config.at("pnc_forbid_aeb").topic);
  AebContext::Instance()->odom_msg.reader =
      reader_manager->CreateReader<DRResult>(
          topic_config.at("localization_odometry").topic);
  AebContext::Instance()->freespace_msg.reader =
      reader_manager->CreateReader<Freespace>(
          topic_config.at("lidar_perception_freespace").topic);
  AebContext::Instance()->perception_msg.reader =
      reader_manager->CreateReader<PerceptionObstacles>(
          topic_config.at("perception_obstacles").topic);
  AebContext::Instance()->localization_msg.reader =
      reader_manager->CreateReader<LocalizationEstimate>(
          topic_config.at("localization_pose").topic);
  AebContext::Instance()->patrol_msg.reader =
      reader_manager->CreateReader<PatrolStatus>(
          topic_config.at("patrol_status").topic);
  aeb_ptr_->Init();
  proc_thread_ = std::make_unique<std::thread>([this]() { Proc(); });
  neodrive::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "aeb_proc_thread_", proc_thread_.get());
  LOG_INFO("AebComponent init finished.");
  return true;
}

bool AebComponent::Proc() {
  LOG_WARN("[thread_name]aeb_component");
  auto aeb_config = config::AebConfig::Instance();
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  uint64_t period_time =
      1000 / std::max(aeb_config->aeb_config().aeb_period_frequency, 1);

  uint64_t start_time = 0;
  uint64_t end_time = 0;
  uint64_t delta_time = 0;

  while (!neodrive::cyber::IsShutdown()) {
    start_time = common::NowMilliSec();
    loop_start_time_ = start_time / 1e3;
    PERFORMANCE_TEST_START(aeb_component, "proc");
    UpdateEnvironment();
    collision_ptr_->VoteCollionCheck();
    aeb_ptr_->RunOnce();
    PublishAebCmd();
    PERFORMANCE_TEST_END(aeb_component, "proc");
    end_time = common::NowMilliSec();
    delta_time = end_time - start_time;
    if (delta_time < period_time) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(period_time - delta_time));
    } else {
      std::this_thread::yield();
    }
  }
  return true;
}

void AebComponent::UpdateEnvironment() {
  Observe();
  CheckTimeout();
  auto &context = *AebContext::Instance();
  if (context.chassis_msg.is_updated)
    context.environment.UpdateChassis(*context.chassis_msg.ptr);
  if (context.perception_msg.is_updated)
    context.environment.UpdateLidarObstacles(*context.perception_msg.ptr);
  if (context.localization_msg.is_updated)
    context.environment.UpdateLocalization(*context.localization_msg.ptr);
  context.is_imu_collision = context.patrol_msg.ptr->is_collision();
}

void AebComponent::Observe() {
  OBSERVE_FUNCTION(AebContext::Instance()->chassis_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->odom_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->freespace_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->perception_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->localization_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->forbid_aeb_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->pnc_forbid_aeb_msg);
  OBSERVE_FUNCTION(AebContext::Instance()->patrol_msg);
}

void AebComponent::CheckTimeout() {
  auto &aeb_config = config::AebConfig::Instance()->aeb_config();
  double strict_threshold = aeb_config.strict_timeout_check;
  double now_t = common::NowSec();
  CheckMessageTimeout(AebContext::Instance()->chassis_msg, strict_threshold,
                      now_t);
  CheckMessageTimeout(AebContext::Instance()->freespace_msg, strict_threshold,
                      now_t);
  CheckMessageTimeout(AebContext::Instance()->perception_msg, strict_threshold,
                      now_t);
  CheckMessageTimeout(AebContext::Instance()->localization_msg,
                      strict_threshold, now_t);
}

// Save timestamp information in AebCmd message, which is used for calculate
// time latency.
void FillTimeCollector(::neodrive::global::common::header::Header *header,
                       double loop_start_time) {
#define SET_TIMECOLLECTOR(var, name, in_msg)           \
  if (in_msg != nullptr) {                             \
    auto timestamp = in_msg->header().timestamp_sec(); \
    var->add_##name(timestamp);                        \
  }

  auto aeb = AebContext::Instance();

  auto freespace = (aeb->freespace_msg.ptr);
  SET_TIMECOLLECTOR(header->mutable_time_collector(), perc_freespace,
                    freespace);

  auto perception_obs = (aeb->perception_msg.ptr);
  SET_TIMECOLLECTOR(header->mutable_time_collector(), perc_obstacles,
                    perception_obs);

  auto chassis = (aeb->chassis_msg.ptr);
  SET_TIMECOLLECTOR(header->mutable_time_collector(), chassis, chassis);

  auto utm_localization = (aeb->localization_msg.ptr);
  SET_TIMECOLLECTOR(header->mutable_time_collector(), loc_utm_pose,
                    utm_localization);

  auto odometry_localization = (aeb->odom_msg.ptr);
  SET_TIMECOLLECTOR(header->mutable_time_collector(), loc_dr,
                    odometry_localization);

  header->mutable_time_collector()->add_aeb_cmd_runtime(
      cyber::Time::Now().ToSecond() - loop_start_time);
}

void AebComponent::PublishAebCmd() {
  auto aeb_cmd = aeb_cmd_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(aeb_cmd);
  double now_t = common::Now();
  aeb_cmd->mutable_header()->set_module_name("aeb");
  aeb_cmd->mutable_header()->set_timestamp_sec(now_t);
  ReportCollision();
  if (!AebContext::Instance()->is_aeb_active &&
      now_t - pre_active_t_ < kMinActiveInterval) {
    AebContext::Instance()->is_aeb_active = true;
  }
  if (!AebContext::Instance()->is_forbid_aeb &&
      AebContext::Instance()->is_aeb_active) {
    aeb_cmd->mutable_aeb_state()->set_state(AebState::ACTIVE);
    aeb_cmd->mutable_aeb_state()->set_is_fcw_active(true);
    aeb_cmd->set_brake(kAebBrakePercent);
    ReportEvent();
    ReportFcwWarning();
  } else if (AebContext::Instance()->is_forbid_aeb) {
    aeb_cmd->mutable_aeb_state()->set_state(AebState::OFF);
    aeb_cmd->mutable_aeb_state()->set_is_fcw_active(
        AebContext::Instance()->is_aeb_active);
    aeb_cmd->set_brake(0.0);
    if (AebContext::Instance()->is_aeb_active)
      ReportFcwWarning();
    else
      is_send_event = false;
  } else if (AebContext::Instance()->is_failure) {
    aeb_cmd->mutable_aeb_state()->set_state(AebState::FAILURE_REVERSIBLE);
    aeb_cmd->mutable_aeb_state()->set_is_fcw_active(false);
    aeb_cmd->set_brake(0.0);
    is_send_event = false;
  } else {
    aeb_cmd->mutable_aeb_state()->set_state(AebState::STANDBY);
    aeb_cmd->mutable_aeb_state()->set_is_fcw_active(
        AebContext::Instance()->is_aeb_active);
    aeb_cmd->set_brake(0.0);
    if (AebContext::Instance()->is_aeb_active)
      ReportFcwWarning();
    else
      is_send_event = false;
  }
  AebContext::Instance()->aeb_state = aeb_cmd->aeb_state().state();
  FillTimeCollector(aeb_cmd->mutable_header(), loop_start_time_);
  aeb_cmd_pub_->Write(aeb_cmd);
}

void AebComponent::ReportEvent() {
  if (is_send_event) return;
  pre_active_t_ = common::Now();
  auto event_reporter = event_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(event_reporter);
  event_reporter->mutable_header()->set_timestamp_sec(common::Now());
  event_reporter->mutable_header()->set_module_name("aeb");
  event_reporter->set_event_type(EventOfInterest::AEB);
  event_reporter->set_start_time(-kEventRecordTimeEpsilon);
  event_reporter->set_end_time(kEventRecordTimeEpsilon);
  event_reporter_pub_->Write(event_reporter);
  is_send_event = true;
}

void AebComponent::ReportFcwWarning() {
  auto ivi_event_report = event_report_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(ivi_event_report);
  ivi_event_report->clear_event_infos();
  auto info = ivi_event_report->add_event_infos();
  info->set_name("fcw active");
  info->set_level(EventInfo::WARN);
  ivi_event_report_pub_->Write(ivi_event_report);
}

void AebComponent::ReportCollision() {
  if (!AebContext::Instance()->is_collisioned) return;
  auto ivi_event_report = event_report_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(ivi_event_report);
  ivi_event_report->clear_event_infos();
  auto info = ivi_event_report->add_event_infos();
  info->set_name("collision happen");
  info->set_level(EventInfo::WARN);
  ivi_event_report_pub_->Write(ivi_event_report);
  LOG_INFO("event reported");
}
}  // namespace aeb
}  // namespace neodrive
