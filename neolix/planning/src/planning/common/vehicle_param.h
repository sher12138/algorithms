#pragma once

#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common_config/config/auto_ego_car_config.h"
#include "planning_macros.h"

namespace neodrive {
namespace planning {

class VehicleParam {
 public:
  ~VehicleParam() = default;

  void init();

  Box2d get_adc_bounding_box(const Vec2d& adc_pose, const double adc_heading,
                             const double lateral_buffer = 0.0,
                             const double front_buffer = 0.0,
                             const double end_buffer = 0.0);

  Polygon2d get_adc_polygon(const Vec2d& adc_pose, const double adc_heading,
                            const double lateral_buffer = 0.0,
                            const double front_buffer = 0.0,
                            const double end_buffer = 0.0);

  // get paras
  double front_edge_to_center();
  double back_edge_to_center();
  double left_edge_to_center() const;
  double right_edge_to_center() const;
  double length() const;
  double width() const;
  double height() const;
  double min_turn_radius() const;
  double max_acceleration() const;
  double max_deceleration() const;
  double max_steer_angle() const;
  double max_steer_angle_rate() const;
  double steer_ratio() const;
  double wheel_base() const;
  double wheel_rolling_radius() const;

  double left_front_x() const;
  double left_front_y() const;
  double right_front_x() const;
  double right_front_y() const;
  double right_rear_x() const;
  double right_rear_y() const;
  double left_rear_x() const;
  double left_rear_y() const;

 private:
  void CalculateVehicleCorner();
  // Car center point is car reference point, i.e., center of rear axle.
  double front_edge_to_center_;
  double back_edge_to_center_;
  double left_edge_to_center_;
  double right_edge_to_center_;

  double length_;
  double width_;
  double height_;

  double min_turn_radius_;
  double max_acceleration_;
  double max_deceleration_;

  // The following items are used to compute trajectory constraints in planning.
  // vehicle max steer angle
  double max_steer_angle_;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate_;
  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio_;
  // the distance between the front and back wheels
  double wheel_base_;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius_;
  // vehicle coordinate
  double left_front_x_ = 0.0;
  double left_front_y_ = 0.0;
  double right_front_x_ = 0.0;
  double right_front_y_ = 0.0;
  double right_rear_x_ = 0.0;
  double right_rear_y_ = 0.0;
  double left_rear_x_ = 0.0;
  double left_rear_y_ = 0.0;

 private:
  DECLARE_SINGLETON(VehicleParam);
};

}  // namespace planning
}  // namespace neodrive
