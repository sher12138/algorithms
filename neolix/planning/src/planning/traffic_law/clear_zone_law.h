#pragma once

#include "src/planning/common/obstacle/obstacle.h"
#include "traffic_law.h"

namespace neodrive {
namespace planning {

class ClearZoneDecider {
 public:
  // TODO(wyc): should add default constructor, or delete it
  // because member parameters may have random values while default constructing
  ClearZoneDecider(double start_s, double end_s);
  bool NeedStop(const Obstacle& object, const Boundary& object_boundary) const;

  void SetStartS(const double& start_s);
  void SetEndS(const double& end_s);

  double StartS() const;
  double EndS() const;

 private:
  double start_route_s_;
  double end_route_s_;
  double check_point_s_;
};

using ClearZoneDeciderArray = std::vector<ClearZoneDecider>;

class ClearZoneLaw : public TrafficLaw {
 public:
  ClearZoneLaw();
  virtual ~ClearZoneLaw() = default;
  virtual ErrorCode Apply(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
      DecisionData* const decision_data,
      TrafficLawContext* const traffic_law_context) override;

 private:
  void FindFrontClearZone(const ReferenceLinePtr& reference_line,
                          const Boundary& adc_boundary,
                          ClearZoneDeciderArray* const clear_zone_deciders);
  void FindFrontCrosswalkClearZone(
      const ReferenceLinePtr& reference_line, const Boundary& adc_boundary,
      ClearZoneDeciderArray* const clear_zone_deciders);
  // TODO: use pointer for modifying param
  void MergeWithExistClearZoneDecider(
      ClearZoneDecider& clear_zone_decider,
      ClearZoneDeciderArray* const clear_zone_deciders);

  bool NeedStop(ClearZoneDecider& clear_zone_decider, double veh_stop_s);
};

}  // namespace planning
}  // namespace neodrive
