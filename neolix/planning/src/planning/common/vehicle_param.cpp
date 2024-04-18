#include "vehicle_param.h"

#include "common/data_center/data_center.h"
#include "common_config/config/common_config.h"

namespace neodrive {
namespace planning {

VehicleParam::VehicleParam() { init(); }

void VehicleParam::init() {
  auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (DataCenter::Instance()->use_center_pose()) {
    front_edge_to_center_ = ego_car_config.front_edge_to_center;
    back_edge_to_center_ = ego_car_config.back_edge_to_center;

  } else {
    front_edge_to_center_ = ego_car_config.front_edge_to_base_link;
    back_edge_to_center_ = ego_car_config.back_edge_to_base_link;
  }
  left_edge_to_center_ = ego_car_config.left_edge_to_center;
  right_edge_to_center_ = ego_car_config.right_edge_to_center;
  length_ = ego_car_config.length;
  width_ = ego_car_config.width;
  height_ = ego_car_config.height;
  min_turn_radius_ = ego_car_config.min_turn_radius;
  max_acceleration_ = ego_car_config.max_acceleration;
  max_deceleration_ = ego_car_config.max_deceleration;
  max_steer_angle_ = ego_car_config.max_steer_angle;
  max_steer_angle_rate_ = ego_car_config.max_steer_angle_rate;
  steer_ratio_ = ego_car_config.steer_ratio;
  wheel_base_ = ego_car_config.wheel_base;
  wheel_rolling_radius_ = ego_car_config.wheel_rolling_radius;
  CalculateVehicleCorner();
}

Box2d VehicleParam::get_adc_bounding_box(const Vec2d& adc_pose,
                                         const double adc_heading,
                                         const double lateral_buffer,
                                         const double front_buffer,
                                         const double end_buffer) {
  auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (DataCenter::Instance()->use_center_pose()) {
    front_edge_to_center_ = ego_car_config.front_edge_to_center;
  } else {
    front_edge_to_center_ = ego_car_config.front_edge_to_base_link;
  }
  const double mid_to_rear_center =
      (length_ + front_buffer + end_buffer) / 2.0 - front_edge_to_center_ -
      front_buffer;
  double x, y;
  if (DataCenter::Instance()->use_center_pose()) {
    x = adc_pose.x();
    y = adc_pose.y();
  } else {
    x = adc_pose.x() - mid_to_rear_center * std::cos(adc_heading);
    y = adc_pose.y() - mid_to_rear_center * std::sin(adc_heading);
  }

  return Box2d({x, y}, adc_heading, length_ + front_buffer + end_buffer,
               width_ + 2 * lateral_buffer);
}

Polygon2d VehicleParam::get_adc_polygon(const Vec2d& adc_pose,
                                        const double adc_heading,
                                        const double lateral_buffer,
                                        const double front_buffer,
                                        const double end_buffer) {
  return Polygon2d(get_adc_bounding_box(adc_pose, adc_heading, lateral_buffer,
                                        front_buffer, end_buffer));
}

double VehicleParam::front_edge_to_center() {
  auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (DataCenter::Instance()->use_center_pose()) {
    front_edge_to_center_ = ego_car_config.front_edge_to_center;
  } else {
    front_edge_to_center_ = ego_car_config.front_edge_to_base_link;
  }
  return front_edge_to_center_;
}

double VehicleParam::back_edge_to_center() {
  auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (DataCenter::Instance()->use_center_pose()) {
    back_edge_to_center_ = ego_car_config.back_edge_to_center;
  } else {
    back_edge_to_center_ = ego_car_config.back_edge_to_base_link;
  }
  return back_edge_to_center_;
}

double VehicleParam::left_edge_to_center() const {
  return left_edge_to_center_;
}

double VehicleParam::right_edge_to_center() const {
  return right_edge_to_center_;
}

double VehicleParam::length() const { return length_; }

double VehicleParam::width() const { return width_; }

double VehicleParam::height() const { return height_; }

double VehicleParam::min_turn_radius() const { return min_turn_radius_; }

double VehicleParam::max_acceleration() const { return max_acceleration_; }

double VehicleParam::max_deceleration() const { return max_deceleration_; }

double VehicleParam::max_steer_angle() const { return max_steer_angle_; }

double VehicleParam::max_steer_angle_rate() const {
  return max_steer_angle_rate_;
}

double VehicleParam::steer_ratio() const { return steer_ratio_; }

double VehicleParam::wheel_base() const { return wheel_base_; }

double VehicleParam::wheel_rolling_radius() const {
  return wheel_rolling_radius_;
}

double VehicleParam::left_front_x() const { return left_front_x_; }

double VehicleParam::left_front_y() const { return left_front_y_; }

double VehicleParam::right_front_x() const { return right_front_x_; }

double VehicleParam::right_front_y() const { return right_front_y_; }

double VehicleParam::right_rear_x() const { return right_rear_x_; }

double VehicleParam::right_rear_y() const { return right_rear_y_; }

double VehicleParam::left_rear_x() const { return left_rear_x_; }

double VehicleParam::left_rear_y() const { return left_rear_y_; }

void VehicleParam::CalculateVehicleCorner() {
  auto& ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (DataCenter::Instance()->use_center_pose()) {
    front_edge_to_center_ = ego_car_config.front_edge_to_center;
    back_edge_to_center_ = ego_car_config.back_edge_to_center;

  } else {
    front_edge_to_center_ = ego_car_config.front_edge_to_base_link;
    back_edge_to_center_ = ego_car_config.back_edge_to_base_link;
  }
  left_front_x_ = -1 * left_edge_to_center_;
  left_front_y_ = front_edge_to_center_;
  right_front_x_ = right_edge_to_center_;
  right_front_y_ = front_edge_to_center_;
  right_rear_x_ = right_edge_to_center_;
  right_rear_y_ = -1 * back_edge_to_center_;
  left_rear_x_ = -1 * left_edge_to_center_;
  left_rear_y_ = -1 * back_edge_to_center_;
}

}  // namespace planning
}  // namespace neodrive
