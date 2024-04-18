#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "common/reference_line/reference_point.h"

namespace neodrive {
namespace planning_rl {

struct Near100SRet {
  std::vector<std::vector<double>> nearest_100_refpoint;
  std::vector<double> nearest_100_refpoint_curvature;
  std::vector<double> nearest_100_refpoint_theta;
  double curvature;
  std::vector<double> refline_points_nearest;

  Near100SRet(std::vector<std::vector<double>> nearest_100_refpoint,
              std::vector<double> nearest_100_refpoint_curvature,
              std::vector<double> nearest_100_refpoint_theta, double curvature,
              std::vector<double> refline_points_nearest) {
    this->nearest_100_refpoint = nearest_100_refpoint;
    this->nearest_100_refpoint_curvature = nearest_100_refpoint_curvature;
    this->nearest_100_refpoint_theta = nearest_100_refpoint_theta;
    this->curvature = curvature;
    this->refline_points_nearest = refline_points_nearest;
  }
};

Near100SRet FindNearest100(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &refline_points_curvature,
    const std::vector<double> &thetas, const std::vector<double> &adc_point);

std::vector<std::vector<double>> FindNearest100Meter(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &refline_points_curvature,
    const std::vector<double> &thetas, const std::vector<double> &adc_point);

std::tuple<std::vector<ReferencePoint>, std::vector<ReferencePoint>> FindNearest110MeterSample(
    const std::vector<ReferencePoint> &refline_points,
    const std::vector<double> &adc_point, int sample_num);

std::vector<std::vector<double>> FindNearest100MeterRefpoints(
    const std::vector<std::vector<double>> &refline_points,
    const std::vector<double> &adc_point);

int FindNearestReflinePoint(const std::vector<ReferencePoint> &refline_points, const std::vector<double> &ADCpoint);

int FindNearestReflinePointS(const std::vector<ReferencePoint> &refline_points, const int start_index, const double predict_s);

std::vector<std::vector<double>> GetSupportPoint(
    const double &curvature, const std::vector<double> &start_point,
    double theta, const int &point_num);

std::vector<ReferencePoint> GetSupportRefPoint(
    const ReferencePoint &start_point,
    double theta, const int &point_num);

double NormalizeAngle(const double &angle);

int trans_bool2int(bool input);

std::vector<double> V1dGradient(const std::vector<double> &input);

std::vector<double> V1dAcrtan2(const std::vector<double> &inputy,
                               const std::vector<double> &inputx);

std::vector<double> CalCurvatureWithTheta(const std::vector<double> &x,
                                          const std::vector<double> &y,
                                          const std::vector<double> &theta);

std::tuple<std::vector<double>, std::vector<double>> CalCurvature(
    const std::vector<double> &x, const std::vector<double> &y);

std::vector<double> CalDKappa(const std::vector<double> &x,
                              const std::vector<double> &y,
                              const std::vector<double> &kappa);

bool CmpTotalCost(const std::vector<double> &x, const std::vector<double> &y);

std::vector<double> ConvertToRelativeCoordinate2D(
    const std::vector<double> &pt_in_world,
    const std::vector<double> &origin_in_world);

std::vector<double> ConvertToRelativeCoordinate(
    const std::vector<double> &pt_in_world,
    const std::vector<double> &origin_in_world);

std::vector<double> ConvertToRelativeCoordinate2D5input(const double &pt_x,
                                                        const double &pt_y,
                                                        const double &ori_x,
                                                        const double &ori_y,
                                                        const double &ori_yaw);

std::vector<double> ConvertToRelativeCoordinate6input(const double &pt_x,
                                                        const double &pt_y,
                                                        const double &pt_yaw,
                                                        const double &ori_x,
                                                        const double &ori_y,
                                                        const double &ori_yaw);

std::vector<double> ConvertToWorldCoordinate(
    const std::vector<double> &pt_in_relative,
    const std::vector<double> &origin_in_world);

std::vector<double> ConvertToWorldCoordinate6input(
    const double &pt_x, const double &pt_y, const double &pt_yaw,
    const double &ori_x, const double &ori_y, const double &ori_yaw);

/// Transform the coordinate of obs from world to ego vehicle
void WorldCoordToVehicleCoord(const double ego_world_x,
                              const double ego_world_y,
                              const double ego_world_theta,
                              const double obs_world_x,
                              const double obs_world_y,
                              const double obs_world_theta, double &obs_ego_x,
                              double &obs_ego_y, double &obs_ego_theta);

/// Transform the coordinates obs from ego vehicle to odometry
void VehicleCoordToOdometryCoord(const double ego_odo_x, const double ego_odo_y,
                                 const double ego_odo_theta,
                                 const double obs_ego_x, const double obs_ego_y,
                                 const double obs_ego_theta, double &obs_odo_x,
                                 double &obs_odo_y, double &obs_odo_theta);

/// padding
template <typename T>
std::vector<std::vector<T>> ConstrantPad2d(const std::vector<std::vector<T>> &input, const std::vector<int> &padding_size, T pad_value) {
    int padding_left_size = padding_size[0];
    int padding_right_size = padding_size[1];
    int padding_top_size = padding_size[2];
    int padding_bottom_size = padding_size[3];
    int rows = input.size();
    int cols = 5;
    if (rows > 0) {
        cols = input[0].size();
    }
    std::vector<std::vector<T>> output(rows + padding_top_size + padding_bottom_size, std::vector<T>(cols + padding_left_size + padding_right_size));

    for (int i = padding_top_size; i < rows + padding_top_size; i++) {
        for (int j = padding_left_size; j < cols + padding_left_size; j++) {
            output[i][j] = input[i - padding_top_size][j - padding_left_size];
        }
    }

    for (int i = padding_top_size; i < rows + padding_top_size; i++) {
        for (int j = 0; j < padding_left_size; j++) {
            output[i][j] = pad_value;
        }
        for (int j = cols + padding_left_size; j < cols + padding_left_size + padding_right_size; j++) {
            output[i][j] = pad_value;
        }
    }

    for (int i = 0; i < padding_top_size; i++) {
        for (int j = 0; j < cols + padding_left_size + padding_right_size; j++) {
            output[i][j] = pad_value;
        }
    }
    for (int i = rows + padding_top_size; i < rows + padding_top_size + padding_bottom_size; i++) {
        for (int j = 0; j < cols + padding_left_size + padding_right_size; j++) {
            output[i][j] = pad_value;
        }
    }

    return output;
}

}  // namespace planning_rl
}  // namespace neodrive