#pragma once

namespace neodrive {
namespace planning {
namespace sim_planner {
namespace control {

class IntelligentDriverModel {
 public:
  struct State {
    double s{0.0};        // longitudinal distance
    double v{0.0};        // longitudinal speed
    double s_front{0.0};  // leading vehicle
    double v_front{0.0};

    State() {}
    State(const double s_, const double v_, const double s_front_,
          const double v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  struct Param {
    double kDesiredVelocity = 0.0;
    double kVehicleLength = 5.0;                   // l_alpha-1
    double kMinimumSpacing = 1.0;                  // s0
    double kDesiredHeadwayTime = 0.5;              // T
    double kAcceleration = 2.0;                    // a
    double kComfortableBrakingDeceleration = 3.0;  // b
    double kHardBrakingDeceleration = 5.0;
    int kExponent = 4;  // delta
  };

  static bool getIdmDesiredAcceleration(const Param &param,
                                        const State &cur_state, double *acc);
  static bool getIIdmDesiredAcceleration(const Param &param,
                                         const State &cur_state, double *acc);
  static bool getAccDesiredAcceleration(const Param &param,
                                        const State &cur_state, double *acc);
};

}  // namespace control
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
