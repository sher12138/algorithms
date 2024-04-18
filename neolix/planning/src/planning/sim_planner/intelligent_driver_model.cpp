#include "intelligent_driver_model.h"

#include <math.h>

namespace neodrive {
namespace planning {
namespace sim_planner {
namespace control {

bool IntelligentDriverModel::getIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, double *acc) {
  double s_star =
      param.kMinimumSpacing +
      std::max(0.0,
               cur_state.v * param.kDesiredHeadwayTime +
                   cur_state.v * (cur_state.v - cur_state.v_front) /
                       (2.0 * sqrt(param.kAcceleration *
                                   param.kComfortableBrakingDeceleration)));
  double s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  *acc = param.kAcceleration *
         (1.0 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent) -
          pow(s_star / s_alpha, 2));

  return true;
}

bool IntelligentDriverModel::getIIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, double *acc) {
  // The Improved IntelligentDriverModel (IIDM) tries to address two
  // deficiencies of the original IDM model:
  // 1) If the actual speed exceeds the desired speed (e.g., after entering a
  // zone with a reduced speed limit), the deceleration is unrealistically
  // large, particularly for large values of the acceleration exponent δ.
  // 2) Near the desired speed v0, the steady-state gap becomes much
  // greater than s∗(v, 0) = s0 + vT so that the model parameter T loses its
  // meaning as the desired time gap. This means that a platoon of identical
  // drivers and vehicles disperses much more than observed. Moreover, not all
  // cars will reach the desired speed
  double a_free =
      cur_state.v <= param.kDesiredVelocity
          ? param.kAcceleration *
                (1 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent))
          : -param.kComfortableBrakingDeceleration *
                (1 - pow(param.kDesiredVelocity / cur_state.v,
                         param.kAcceleration * param.kExponent /
                             param.kComfortableBrakingDeceleration));
  double s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  double z =
      (param.kMinimumSpacing +
       std::max(0.0,
                cur_state.v * param.kDesiredHeadwayTime +
                    cur_state.v * (cur_state.v - cur_state.v_front) /
                        (2.0 * sqrt(param.kAcceleration *
                                    param.kComfortableBrakingDeceleration)))) /
      s_alpha;
  double a_out =
      cur_state.v <= param.kDesiredVelocity
          ? (z >= 1.0
                 ? param.kAcceleration * (1 - pow(z, 2))
                 : a_free * (1 - pow(z, 2.0 * param.kAcceleration / a_free)))
          : (z >= 1.0 ? a_free + param.kAcceleration * (1 - pow(z, 2))
                      : a_free);
  a_out = std::max(std::min(param.kAcceleration, a_out),
                   -param.kHardBrakingDeceleration);
  *acc = a_out;

  return true;
}

bool IntelligentDriverModel::getAccDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, double *acc) {
  double acc_iidm;
  getIIdmDesiredAcceleration(param, cur_state, &acc_iidm);

  // ~ Here we simply use a constant dec, ego comfortable dec, as the acc of
  // ~ leading vehicle.
  double ds = std::max(0.0, cur_state.s_front - cur_state.s);
  double acc_cah =
      (cur_state.v * cur_state.v * -param.kComfortableBrakingDeceleration) /
      (cur_state.v_front * cur_state.v_front -
       2 * ds * -param.kComfortableBrakingDeceleration);

  double coolness = 0.99;

  if (acc_iidm >= acc_cah) {
    *acc = acc_iidm;
  } else {
    *acc =
        (1 - coolness) * acc_iidm +
        coolness * (acc_cah - param.kComfortableBrakingDeceleration *
                                  tanh((acc_iidm - acc_cah) /
                                       -param.kComfortableBrakingDeceleration));
  }
  return true;
}

}  // namespace control
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
