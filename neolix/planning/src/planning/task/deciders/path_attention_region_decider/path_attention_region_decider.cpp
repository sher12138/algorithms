#include "path_attention_region_decider.h"

#include "common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {

PathAttentionRegionDecider::PathAttentionRegionDecider() {
  name_ = "PathAttentionRegionDecider";
}

PathAttentionRegionDecider::~PathAttentionRegionDecider() { Reset(); }

ErrorCode PathAttentionRegionDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto &attention_conf = config::PlanningConfig::Instance()
                             ->planning_research_config()
                             .path_attention_region_decider_config;
  ComputeAttentionRegion(task_info);
  return ErrorCode::PLANNING_OK;
}

bool PathAttentionRegionDecider::ComputeAttentionRegion(TaskInfo &task_info) {
  auto decision_data = task_info.current_frame()
                           ->mutable_planning_data()
                           ->mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return false;
  }

  auto &ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return false;
  }

  auto &attention_conf = config::PlanningConfig::Instance()
                             ->planning_research_config()
                             .path_attention_region_decider_config;

  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &outside_data =
      task_info.current_frame()->mutable_outside_planner_data();
  const auto &bound_info =
      outside_data->path_context.original_path_boundary.path_boundary;
  const auto &adc_boundary = outside_data->path_obstacle_context.adc_boundary;

  auto &attention_obstacles =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->path_observe_ref_l_info.attention_dynamic_obstacles;
  std::vector<Obstacle> observe_obstacles{};
  static std::unordered_map<int, std::pair<double, int>> history_obs_table;

  for (size_t i = 0; i < decision_data->all_obstacle().size(); ++i) {
    auto obstacle = *decision_data->all_obstacle()[i];
    LOG_INFO("XXXobstacle current id {}", obstacle.id());

    if (obstacle.is_virtual()) {
      continue;
    }

    ReferencePoint reference_point;
    if (!ref_ptr->GetNearestRefPoint(obstacle.center_sl().s(),
                                     &reference_point)) {
      LOG_INFO("Get nearest reference point fail");
      continue;
    }

    bool is_static = obstacle.is_static() || obstacle.speed() < 1e-4;
    double obs_heading =
        is_static ? obstacle.heading() : obstacle.velocity_heading();
    auto it = history_obs_table.find(obstacle.id());
    if (it == history_obs_table.end()) {
      if (!is_static) {
        history_obs_table.insert(std::pair<int, std::pair<double, int>>(
            obstacle.id(),
            {obstacle.velocity_heading(), attention_conf.obs_maintaince_cnt}));
      }
    } else {
      it->second.second++;
      if (is_static) {
        obs_heading = it->second.first;
      } else {
        obs_heading =
            (1 - attention_conf.vel_heading_filter_ratio) * it->second.first +
            attention_conf.vel_heading_filter_ratio * obs_heading;
        it->second.first = obs_heading;
      }
    }

    if (obstacle.min_s() - attention_conf.attention_front_dis >
            adc_boundary.end_s() ||
        obstacle.max_s() + attention_conf.attention_back_dis <
            adc_boundary.start_s() ||
        obstacle.min_l() - attention_conf.attention_left_dis >
            adc_boundary.end_l() ||
        obstacle.max_l() + attention_conf.attention_right_dis <
            adc_boundary.start_l()) {
      LOG_INFO(
          "skip faraway obs id[{}], front[{}] back[{}] left[{}] right[{}].",
          obstacle.id(),
          obstacle.min_s() - attention_conf.attention_front_dis >
              adc_boundary.end_s(),
          obstacle.max_s() +
              attention_conf
                  .attention_back_dis<adc_boundary.start_s(),
                                      obstacle.min_l() -
                                          attention_conf.attention_left_dis>
                      adc_boundary.end_l(),
          obstacle.max_l() + attention_conf.attention_right_dis <
              adc_boundary.start_l());
      continue;
    }

    if (obstacle.speed() > attention_conf.observe_limit_speed) {
      LOG_INFO("skip high speed obstacle[{}] vel: {:.4f}", obstacle.id(),
               obstacle.speed());
    }

    double lateral_offset =
        obstacle.center_sl().l() -
        (adc_boundary.start_l() + adc_boundary.end_l()) / 2.0;
    if (!is_static && normalize_angle(obs_heading - reference_point.heading()) *
                              lateral_offset >
                          0.0) {
      LOG_INFO("skip away from adc obstacle[{}] lateral offset: {:.4f}",
               obstacle.id(), lateral_offset);
      continue;
    }

    double heading_diff =
        std::abs(normalize_angle(obs_heading - reference_point.heading()));
    if (heading_diff < attention_conf.filter_obs_heading_threshold ||
        M_PI - heading_diff < attention_conf.filter_obs_heading_threshold) {
      LOG_INFO(
          "skip normal obstacle[{}] obs_heading: {:.4f}, "
          "ref_heading: {:.4f}, heading_diff: {:.4f}",
          obstacle.id(), obs_heading, reference_point.heading(), heading_diff);
      continue;
    }

    double half_length =
        std::clamp(3.0 * inside_data.init_point.velocity(),
                   static_cast<double>(attention_conf.min_len),
                   static_cast<double>(attention_conf.max_len));
    double length = half_length * 2.;
    double start_width = std::clamp(
        5.0 * obstacle.speed(), static_cast<double>(attention_conf.min_wid),
        static_cast<double>(attention_conf.max_wid));
    double end_width = std::clamp(5.0 * obstacle.speed(),
                                  static_cast<double>(attention_conf.min_wid),
                                  static_cast<double>(attention_conf.max_wid));

    bool found = false;
    if (obstacle.max_l() < adc_boundary.start_l()) {
      start_width = attention_conf.min_wid * std::sin(heading_diff);
      end_width = end_width * std::sin(heading_diff);
    } else if (obstacle.min_l() > adc_boundary.end_l()) {
      start_width = start_width * std::sin(heading_diff);
      end_width = attention_conf.min_wid * std::sin(heading_diff);
    } else {
      start_width = start_width * std::sin(heading_diff);
      end_width = end_width * std::sin(heading_diff);
    }

    double v_x = obstacle.speed() * std::cos(obs_heading);
    double v_y = obstacle.speed() * std::sin(obs_heading);
    LOG_INFO(
        "obstacle[{}]: speed: {:.4f}, speed x: {:.4f}, speed y: {:.4f}, "
        "obs_heading: {:.4f}, speed heading: {:.4f}, ref heading: {:.4f}, "
        "heading_diff: {:.4f}, set obs_heading: {:.4f}",
        obstacle.id(), obstacle.speed(), v_x, v_y, obstacle.heading(),
        obstacle.velocity_heading(), reference_point.heading(), heading_diff,
        obs_heading);
    LOG_INFO("start_len: {:.4f}, end_len: {:.4f}, width: {:.4f}", start_width,
             end_width, length);

    for (double t = 0.0; t < attention_conf.predict_time; t += 0.1) {
      obstacle.set_center(
          Vec2d{decision_data->all_obstacle()[i]->center().x() + v_x * t,
                decision_data->all_obstacle()[i]->center().y() + v_y * t});

      const auto &points = decision_data->all_obstacle()[i]->polygon().points();
      std::vector<Vec2d> pre_points;
      for (auto &pt : points) {
        pre_points.emplace_back(Vec2d{pt.x() + v_x * t, pt.y() + v_y * t});
      }
      obstacle.mutable_polygon()->Init(pre_points);
      obstacle.init_with_reference_line(ref_ptr);

      double delt_s = inside_data.init_point.velocity() * t;

      if (decision_data->all_obstacle()[i]->min_s() > adc_boundary.end_s()) {
        if (obstacle.min_s() - half_length < adc_boundary.end_s() + delt_s) {
          found = true;
          LOG_INFO(
              "front obs[{}]: pre time: {:.3f}, obstacle.min_s():{:.3f}, "
              "adc_boundary.end_s(): {:.3f}",
              obstacle.id(), t, obstacle.min_s(),
              adc_boundary.end_s() + delt_s);
          break;
        }
      } else {
        if ((obstacle.max_s() > adc_boundary.start_s() - half_length + delt_s &&
             obstacle.max_s() < adc_boundary.end_s() + half_length + delt_s) ||
            (obstacle.min_s() > adc_boundary.start_s() - half_length + delt_s &&
             obstacle.min_s() < adc_boundary.end_s() + half_length + delt_s)) {
          found = true;
          LOG_INFO(
              "mid obs[{}]: pre time: {:.3f}, obstacle.max_s():{:.3f}, "
              "adc_boundary.start_s(): {:.3f}",
              obstacle.id(), t, obstacle.max_s(),
              adc_boundary.start_s() - half_length + delt_s);
          break;
        }
      }
    }

    if (found && !bound_info.empty()) {
      double begin_delt_s =
          (obstacle.min_s() - bound_info.front().lower_point.s()) / 0.1;
      double end_delt_s =
          (obstacle.max_s() - bound_info.front().lower_point.s()) / 0.1;
      size_t end_index = static_cast<size_t>(std::max(0.0, end_delt_s));
      size_t begin_index = static_cast<size_t>(std::max(0.0, begin_delt_s));
      LOG_INFO(
          "begin: {}, end: {}. bounds_info s: {:.4f}, begin_delt_s: {:.4f}, "
          "end_delt_s: {:.4f}",
          begin_index, end_index, bound_info.front().lower_point.s(),
          begin_delt_s, end_delt_s);
      if (begin_index <= end_index && end_index <= bound_info.size()) {
        double max_upper_l = 0, min_upper_l = 100;
        double max_lower_l = -100, min_lower_l = 0;

        // Screen obstacles based on the bounds
        for (size_t i = begin_index; i <= end_index; ++i) {
          max_upper_l = std::max(max_upper_l, bound_info[i].upper_point.l());
          min_upper_l = std::min(min_upper_l, bound_info[i].upper_point.l());
          max_lower_l = std::max(max_lower_l, bound_info[i].lower_point.l());
          min_lower_l = std::min(min_lower_l, bound_info[i].lower_point.l());
        }
        LOG_INFO(
            "lower_point l: {:.4f}, upper_point l: {:.4f}, obstacle.min_l: "
            "{:.4f}, obstacle.max_l(): {:.4f}",
            max_lower_l, min_upper_l, obstacle.min_l(), obstacle.max_l());
        if ((obstacle.min_l() - max_lower_l > 1.0 &&
             obstacle.min_l() - min_upper_l < 0.0) ||
            (obstacle.max_l() - min_upper_l < -1.0 &&
             obstacle.max_l() - max_lower_l > 0.0)) {
          double offset_x = -1 * (end_width - start_width) / 2.0 *
                            std::sin(reference_point.heading());
          double offset_y = (end_width - start_width) / 2.0 *
                            std::cos(reference_point.heading());
          Box2d box(Vec2d{obstacle.center().x() + offset_x,
                          obstacle.center().y() + offset_y},
                    reference_point.heading(), obstacle.width() + length,
                    obstacle.length() + end_width + start_width);
          auto obs = obstacle;
          std::vector<Vec2d> points_pre;
          box.get_all_corners(&points_pre);
          obs.set_center(box.center());
          obs.set_length(obstacle.width() + length);
          obs.set_width(obstacle.length() + end_width + start_width);
          obs.set_heading(reference_point.heading());
          obs.mutable_polygon()->Init(points_pre);
          obs.init_with_reference_line(ref_ptr);

          observe_obstacles.push_back(obs);
          LOG_INFO("push obs[{}]", obs.id());
        }
      }
    }
  }

  for (auto obs = history_obs_table.begin(); obs != history_obs_table.end();) {
    obs->second.second--;
    if (obs->second.second <= 0) {
      history_obs_table.erase(obs++);
    } else {
      ++obs;
    }
  }

  if (!observe_obstacles.empty()) {
    VisObstacles(observe_obstacles, "observe_obstacles");
  }

  for (size_t i = 0; i < observe_obstacles.size(); ++i) {
    attention_obstacles.push_back(observe_obstacles[i]);
  }

  return true;
}

void PathAttentionRegionDecider::VisObstacles(std::vector<Obstacle> &obstacles,
                                              const std::string &name) {
  if (!FLAGS_planning_enable_vis_event || obstacles.empty()) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto &p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto &obstacle : obstacles) {
    auto polygon = event->mutable_polygon()->Add();
    for (auto &pt : obstacle.polygon_corners())
      set_pt(polygon->add_point(), pt);

    auto text = event->mutable_text()->Add();
    set_pt(text->mutable_position(), obstacle.center());
    text->set_text("id: " + std::to_string(obstacle.id()));
  }
}

}  // namespace planning
}  // namespace neodrive